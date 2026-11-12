#!/usr/bin/env python3
"""
eksperimen.py - Menjalankan percobaan skripsi secara otomatis dan menyimpan data MENTAH.

Peran program ini (penerima = LAPTOP):
  * Menjadi penerima: menerima paket dari ESP32 lewat BLE dan UDP (Wi-Fi via router), lalu segera
    mengembalikannya (echo). Program ini TIDAK menghitung latency apa pun.
  * Mengatur sesi lewat kabel USB (serial): kirim START, tunggu ESP32 selesai, ambil data.
  * Mencatat sisi laptop (waktu paket tiba dan waktu echo selesai dikirim) pada jam laptop
    yang SAMA, untuk memisahkan keterlambatan akibat program laptop dari keterlambatan link.

Semua angka latency/loss/jitter berasal dari log ESP32 (satu jam, mikrodetik).

Contoh:
    python eksperimen.py --port COM22 --mode semua --ulang 5 --skenario S1 --jarak "1 m" --laptop-koneksi ethernet
    python eksperimen.py --port COM22 --mode wifi --ulang 1 --durasi 15 --skenario UJI   (uji awal Wi-Fi saja)
"""
import argparse
import asyncio
import csv
import json
import os
import platform
import queue
import random
import sys
import threading
import time
from datetime import datetime

T_DATA, T_ECHO, T_PROBE = 1, 2, 3
LINK_BLE, LINK_WIFI = 0, 1
PKT_LEN = 32
MODE_NAMA = {0: "ble", 1: "wifi", 2: "adaptif"}
MODE_ID = {"ble": 0, "wifi": 1, "adaptif": 2}
BLE_NAME = "GAMEPAD-ESP32"
BLE_TX_UUID = "beb5483e-36e1-4688-b7f5-ea07361b26a8"   # ESP32 -> laptop (notify)
BLE_RX_UUID = "d29759bc-4b6f-45a6-8e50-96f7e44fa70c"   # laptop -> ESP32 (write no response)
UDP_ESP_PORT = 4210
UDP_SIM_BLE_PORT = 4213   # hanya untuk simulator
TCP_PORT = 4212           # kanal kendali TCP (mode adaptor, tanpa kabel USB ke laptop)


def parse_kv(line):
    out = {}
    for tok in line.replace("#", "", 1).split()[1:]:
        if "=" in tok:
            k, v = tok.split("=", 1)
            out[k] = v
    return out


# --------------------------------------------------------------------------- sisi laptop
class Host:
    """Mencatat paket yang tiba dan menyiapkan echo secepat mungkin."""

    def __init__(self):
        self.sid = None
        self.active = False
        self.t0 = 0
        self.rows = []
        self.stale = 0
        self.bad = 0
        self.injeksi = None    # dict: link, jenis (tunda|buang|putus), nilai, mulai_s, selesai_s

    def aksi_injeksi(self, link, t_ns):
        """Kembalikan (buang: bool, tunda_s: float) untuk echo ini. Hanya untuk uji fungsional."""
        f = self.injeksi
        if not f or link != f["link"]:
            return False, 0.0
        dt = (t_ns - self.t0) / 1e9
        if not (f["mulai_s"] <= dt < f["selesai_s"]):
            return False, 0.0
        if f["jenis"] == "putus":
            return True, 0.0
        if f["jenis"] == "buang":
            return self.rnd_inj.random() < f["nilai"] / 100.0, 0.0
        return False, f["nilai"] / 1000.0      # tunda (ms)

    def begin(self, sid):
        self.sid, self.rows, self.stale, self.bad = sid, [], 0, 0
        self.t0 = time.perf_counter_ns()
        self.active = True

    def end(self):
        self.active = False

    def prepare(self, link_arrival, data):
        if len(data) != PKT_LEN:
            self.bad += 1
            return None
        typ = data[0]
        if typ not in (T_DATA, T_PROBE):
            return None
        if not self.active or data[2] != self.sid:
            self.stale += 1
            return None
        flags = 0
        if (sum(data[:30]) & 0xFFFF) != (data[30] | (data[31] << 8)):
            flags |= 1
        echo = bytes((T_ECHO, link_arrival, data[2], typ)) + data[4:8]
        seq = int.from_bytes(data[4:8], "little")
        return echo, typ, seq, flags

    def log(self, typ, link, seq, t_rx, t_done, flags):
        self.rows.append((typ, link, seq, t_rx - self.t0, t_done - self.t0, flags))


class UdpProto(asyncio.DatagramProtocol):
    def __init__(self, host, link_id, esp_addr, discover=False):
        self.host, self.link_id, self.esp_addr = host, link_id, esp_addr
        self.transport = None
        self.hello_ok = False
        self.discover = discover      # True: HELLO dikirim broadcast, alamat ESP32 dipelajari dari balasan
        self.peer_ip = None

    def connection_made(self, transport):
        self.transport = transport

    def datagram_received(self, data, addr):
        t = time.perf_counter_ns()
        if data == b"HACK":
            self.hello_ok = True
            if self.discover:
                self.esp_addr = (addr[0], addr[1])
                self.peer_ip = addr[0]
            return
        r = self.host.prepare(self.link_id, data)
        if r is None:
            return
        echo, typ, seq, flags = r
        buang, tunda = self.host.aksi_injeksi(self.link_id, t)
        if buang:
            self.host.log(typ, self.link_id, seq, t, t, flags | 8)       # echo dibuang oleh injeksi
            return
        if tunda > 0:
            asyncio.get_running_loop().call_later(tunda, self._kirim_tunda, echo, typ, seq, t, flags | 8, int(tunda * 1e9))
            return
        self.transport.sendto(echo, self.esp_addr)
        self.host.log(typ, self.link_id, seq, t, time.perf_counter_ns(), flags)

    def _kirim_tunda(self, echo, typ, seq, t, flags, tunda_ns):
        self.transport.sendto(echo, self.esp_addr)
        # waktu proses laptop dicatat TANPA tunda injeksi, agar tunda terhitung sebagai latency link
        self.host.log(typ, self.link_id, seq, t, time.perf_counter_ns() - tunda_ns, flags)

    def error_received(self, exc):
        pass

    async def hello(self, timeout=8.0):
        t_end = time.time() + timeout
        while time.time() < t_end and not self.hello_ok:
            self.transport.sendto(b"HELLO", self.esp_addr)
            await asyncio.sleep(0.3)
        return self.hello_ok


class BleLink:
    MAX_INFLIGHT = 8

    def __init__(self, host):
        self.host, self.client, self.inflight = host, None, 0
        self._jaga = None            # tugas latar: sambung ulang otomatis bila BLE putus
        self._menyambung = False

    @property
    def connected(self):
        return self.client is not None and self.client.is_connected

    def _on_putus(self, _client):
        print("  [BLE terputus, mencoba menyambung ulang di latar]", flush=True)

    async def connect(self, timeout=15.0, percobaan=3):
        from bleak import BleakScanner, BleakClient
        galat = None
        for _ in range(percobaan):
            try:
                if self.client is not None:
                    try:
                        await self.client.disconnect()
                    except Exception:
                        pass
                dev = await BleakScanner.find_device_by_name(BLE_NAME, timeout=timeout)
                if dev is None:
                    galat = "perangkat tidak ditemukan saat pemindaian"
                    continue
                client = BleakClient(dev, disconnected_callback=self._on_putus)
                await client.connect()
                await client.start_notify(BLE_TX_UUID, self._on_notify)
                self.client = client
                return
            except Exception as e:
                galat = e
                await asyncio.sleep(1.0)
        raise RuntimeError(f"ESP32 '{BLE_NAME}' tidak bisa disambung lewat BLE ({galat}). Pastikan Bluetooth laptop aktif dan "
                           f"perangkat tidak sedang terhubung ke aplikasi lain. Bila baru flash ulang, hapus "
                           f"perangkat itu dari Pengaturan Bluetooth Windows lalu coba lagi.")

    def mulai_jaga(self):
        """Sambung ulang otomatis selama program berjalan, termasuk di tengah sesi (berlaku sama untuk semua metode)."""
        if self._jaga is None:
            self._jaga = asyncio.ensure_future(self._loop_jaga())

    async def _loop_jaga(self):
        while True:
            await asyncio.sleep(1.0)
            if self.connected or self._menyambung:
                continue
            self._menyambung = True
            try:
                await self.connect(timeout=4.0, percobaan=1)
                print("  [BLE tersambung kembali]", flush=True)
            except Exception:
                pass
            finally:
                self._menyambung = False

    async def tunggu_tersambung(self, batas_s=25.0):
        t_end = time.time() + batas_s
        while time.time() < t_end:
            if self.connected:
                return True
            await asyncio.sleep(0.5)
        return self.connected

    def _on_notify(self, _sender, data):
        t = time.perf_counter_ns()
        r = self.host.prepare(LINK_BLE, bytes(data))
        if r is None:
            return
        echo, typ, seq, flags = r
        buang, tunda = self.host.aksi_injeksi(LINK_BLE, t)
        if buang:
            self.host.log(typ, LINK_BLE, seq, t, t, flags | 8)           # echo dibuang oleh injeksi
            return
        if tunda > 0:
            asyncio.get_running_loop().call_later(tunda, self._kirim, echo, typ, seq, t, flags | 8, int(tunda * 1e9))
            return
        self._kirim(echo, typ, seq, t, flags, 0)

    def _kirim(self, echo, typ, seq, t, flags, tunda_ns):
        if self.inflight >= self.MAX_INFLIGHT:
            self.host.log(typ, LINK_BLE, seq, t, t, flags | 2)    # echo dibuang (laptop kewalahan)
            return
        self.inflight += 1
        asyncio.ensure_future(self._echo(echo, typ, seq, t, flags, tunda_ns))

    async def _echo(self, echo, typ, seq, t_rx, flags, tunda_ns=0):
        try:
            await self.client.write_gatt_char(BLE_RX_UUID, echo, response=False)
        except Exception:
            flags |= 4
        finally:
            self.inflight -= 1
        self.host.log(typ, LINK_BLE, seq, t_rx, time.perf_counter_ns() - tunda_ns, flags)

    async def close(self):
        if self._jaga is not None:
            self._jaga.cancel()
        try:
            if self.client:
                await self.client.disconnect()
        except Exception:
            pass


# --------------------------------------------------------------------------- serial
class EspBase:
    """Antarmuka kanal kendali ESP32 (serial atau TCP)."""

    async def command(self, text, prefix, timeout=3.0):
        """Kirim perintah dan tunggu baris balasan berawalan `prefix`."""
        self.flush_input()
        try:
            self.send(text)
        except Exception:
            return None
        t_end = time.time() + timeout
        while time.time() < t_end:
            line = await self.readline(0.3)
            if line and line.startswith(prefix):
                return line
            if line and line.startswith("#ERR"):
                raise RuntimeError(f"ESP32 menolak '{text}': {line}")
        return None



class EspSerial(EspBase):
    def __init__(self, port, baud):
        import serial
        self.ser = serial.Serial()
        self.ser.port, self.ser.baudrate, self.ser.timeout = port, baud, 0.2
        self.ser.dtr = False      # cegah ESP32 ter-reset saat port dibuka
        self.ser.rts = False
        self.ser.open()
        self.q = queue.Queue()
        self._stop = False
        threading.Thread(target=self._run, daemon=True).start()

    def _run(self):
        buf = b""
        while not self._stop:
            try:
                chunk = self.ser.read(4096)
            except Exception:
                break
            if not chunk:
                continue
            buf += chunk
            while b"\n" in buf:
                raw, buf = buf.split(b"\n", 1)
                self.q.put(raw.decode("utf-8", errors="replace").strip())

    def send(self, text):
        self.ser.write((text + "\n").encode())

    def _get(self, timeout):
        try:
            return self.q.get(timeout=timeout)
        except queue.Empty:
            return None

    async def readline(self, timeout):
        return await asyncio.get_running_loop().run_in_executor(None, self._get, timeout)

    def flush_input(self):
        while not self.q.empty():
            self.q.get_nowait()

    def close(self):
        self._stop = True
        try:
            self.ser.close()
        except Exception:
            pass


class EspTcp(EspBase):
    """Kanal kendali lewat TCP (Wi-Fi) dengan antarmuka yang sama seperti EspSerial.
    Dipakai saat ESP32 diberi daya dari adaptor dan tidak tersambung USB ke laptop."""

    def __init__(self, ip, port=TCP_PORT):
        self.ip, self.port = ip, port
        self.q = asyncio.Queue()
        self.reader = self.writer = self.task = None
        self.closed = True

    async def connect(self, timeout=6.0):
        self.close()
        self.reader, self.writer = await asyncio.wait_for(asyncio.open_connection(self.ip, self.port), timeout)
        self.closed = False
        while not self.q.empty():
            self.q.get_nowait()
        self.task = asyncio.ensure_future(self._pump())

    async def _pump(self):
        try:
            while True:
                raw = await self.reader.readline()
                if not raw:
                    break
                self.q.put_nowait(raw.decode("utf-8", errors="replace").strip())
        except Exception:
            pass
        self.closed = True

    async def reconnect(self):
        for _ in range(5):
            try:
                await self.connect()
                return True
            except Exception:
                await asyncio.sleep(1.5)
        return False

    def send(self, text):
        if self.closed or self.writer is None:
            raise ConnectionError("kanal TCP putus")
        self.writer.write((text + "\n").encode())

    async def readline(self, timeout):
        try:
            return await asyncio.wait_for(self.q.get(), timeout)
        except asyncio.TimeoutError:
            return None

    def flush_input(self):
        while not self.q.empty():
            self.q.get_nowait()

    def close(self):
        self.closed = True
        try:
            if self.task:
                self.task.cancel()
            if self.writer:
                self.writer.close()
        except Exception:
            pass


# --------------------------------------------------------------------------- percobaan
async def jalankan_sesi(esp, host, mode, sid, dur):
    host.begin(sid)
    esp.flush_input()
    try:
        esp.send(f"START {mode} {dur} {sid}")
    except Exception as e:
        host.end()
        raise RuntimeError(f"Gagal mengirim START: {e}")
    t_end = time.time() + 4
    while True:
        line = await esp.readline(0.5)
        if line and line.startswith("#ACK START"):
            break
        if line and line.startswith("#ERR"):
            host.end()
            raise RuntimeError(f"START ditolak: {line}")
        if time.time() > t_end:
            host.end()
            raise RuntimeError("ESP32 tidak membalas START (#ACK). Cek kanal kendali (kabel atau Wi-Fi).")
    t_start = time.time()
    kosong = lambda: {"D": [], "P": [], "Q": [], "E": []}
    begin, rows = None, kosong()
    deadline = t_start + dur + 150
    while time.time() < deadline:
        if getattr(esp, "closed", False):
            # Kanal TCP putus (mis. gangguan Wi-Fi saat uji): sambung ulang lalu minta log lagi (data masih di RAM ESP32).
            if not await esp.reconnect():
                continue
            begin, rows = None, kosong()
            try:
                esp.send("DUMP")
            except Exception:
                pass
        line = await esp.readline(1.0)
        if line is None:
            continue
        if line.startswith("#ERR") and "sesi berjalan" in line and begin is None:
            await asyncio.sleep(1.5)                 # sesi belum selesai; coba minta lagi nanti
            try:
                esp.send("DUMP")
            except Exception:
                pass
            continue
        if line.startswith("#BEGIN"):
            begin, rows = parse_kv(line), kosong()
            host.end()
            continue
        if begin is None:
            continue
        if line.startswith("#END"):
            break
        if len(line) > 2 and line[1] == "," and line[0] in rows:
            rows[line[0]].append(line[2:].split(","))
    host.end()
    if begin is None:
        raise RuntimeError("Tidak menerima #BEGIN dari ESP32 (sesi tidak selesai atau kanal kendali terputus).")
    return begin, rows, time.time() - t_start


def validasi(begin, rows, dur, host):
    pesan = []
    ok = True
    for tag, key in (("D", "nd"), ("P", "np"), ("Q", "nq"), ("E", "ne")):
        if len(rows[tag]) != int(begin.get(key, -1)):
            ok = False
            pesan.append(f"jumlah baris {tag} = {len(rows[tag])} tetapi ESP32 melaporkan {begin.get(key)} (data serial terpotong)")
    nd = len(rows["D"])
    if abs(nd - dur * 50) > dur * 50 * 0.03 + 5:
        ok = False
        pesan.append(f"jumlah paket DATA {nd} jauh dari target {dur * 50} (laju kirim tidak stabil)")
    if host.bad:
        pesan.append(f"{host.bad} paket berukuran salah diterima laptop")
    return ok, pesan


def ringkas(rows, host, begin):
    D = rows["D"]
    n = len(D)
    if n == 0:
        return "tidak ada data"
    rtt = [int(r[2]) for r in D]
    deliv = [x for x in rtt if 0 < x <= 1_000_000]
    loss = 100.0 * (n - len(deliv)) / n
    seen = {r[2] for r in host.rows if r[0] == T_DATA}
    fwd = 100.0 * (n - len(seen)) / n
    s = f"paket {n} | loss {loss:.2f}% (tak sampai laptop {fwd:.2f}%)"
    if deliv:
        deliv.sort()
        s += f" | RTT median {deliv[len(deliv)//2]/1000:.1f} ms, p95 {deliv[int(0.95*(len(deliv)-1))]/1000:.1f} ms"
    ne = sum(1 for r in rows["E"] if r[1] == "1")
    if begin.get("mode") == "2":
        s += f" | pindah {ne}x"
    return s


def simpan(folder, begin, rows, host, meta):
    os.makedirs(folder, exist_ok=True)

    def tulis(nama, header, data):
        with open(os.path.join(folder, nama), "w", newline="", encoding="utf-8") as f:
            w = csv.writer(f, lineterminator="\n")
            w.writerow(header)
            w.writerows(data)
    cols = ["seq", "tx_us", "rtt_us", "link_tx", "link_rx", "rssi_ble", "rssi_wifi"]
    tulis("data.csv", cols, rows["D"])
    tulis("probe.csv", cols, rows["P"])
    tulis("trace.csv", ["t_ms", "skor_ble_x10", "skor_wifi_x10", "link_aktif"], rows["Q"])
    tulis("events.csv", ["t_us", "kind", "dari", "ke", "alasan", "pemicu", "skor_dari_x10", "skor_ke_x10", "extra_us",
                         "p90_rtt_x10_ms", "loss_x10_persen", "jitter_x10_ms", "rssi_dbm"], rows["E"])
    tulis("host_rx.csv", ["tipe", "link", "seq", "t_rx_ns", "t_selesai_echo_ns", "flags"], host.rows)
    with open(os.path.join(folder, "meta.json"), "w", encoding="utf-8") as f:
        json.dump(meta, f, indent=2, ensure_ascii=False)


def rencana(mode_arg, ulang, seed):
    modes = [0, 1, 2] if mode_arg == "semua" else [MODE_ID[mode_arg]]
    rnd = random.Random(seed)
    plan = []
    for r in range(1, ulang + 1):
        m = modes[:]
        rnd.shuffle(m)           # urutan acak per putaran (interleaving) agar tidak bias waktu
        plan += [(r, x) for x in m]
    return plan


async def status_siap(esp, butuh_ble, batas_s=8.0):
    """Minta STATUS berulang sampai ESP32 melaporkan BLE tersambung (ESP32 butuh sesaat setelah sambung ulang)."""
    t_end = time.time() + batas_s
    while True:
        s = parse_kv(await esp.command("STATUS", "#STATUS") or "")
        if not butuh_ble or s.get("ble") == "1" or time.time() > t_end:
            return s
        await asyncio.sleep(0.5)


async def main_async(a):
    loop = asyncio.get_running_loop()
    host = Host()
    if a.injeksi:
        try:
            lk, jn, nv, rentang = a.injeksi.split(":")
            m0, m1 = rentang.split("-")
            host.injeksi = {"link": {"ble": LINK_BLE, "wifi": LINK_WIFI}[lk.lower()], "link_nama": lk.lower(),
                            "jenis": jn.lower(), "nilai": float(nv), "mulai_s": float(m0), "selesai_s": float(m1)}
            if host.injeksi["jenis"] not in ("tunda", "buang", "putus"):
                raise ValueError
        except (ValueError, KeyError):
            sys.exit("Format --injeksi salah. Contoh: ble:tunda:200:20-40  |  ble:buang:30:20-40  |  wifi:putus:0:20-40")
        host.rnd_inj = random.Random(12345)
        if not a.skenario.upper().startswith("UJI"):
            sys.exit("--injeksi hanya untuk uji fungsional. Pakai label skenario berawalan UJI, mis. --skenario UJI_INJEKSI")
        print(f"*** INJEKSI GANGGUAN AKTIF: {a.injeksi} (data ini BUKAN data perbandingan skripsi) ***")
    # Kedua link SELALU disambungkan (beban probe di kedua radio sama pada semua metode).
    perlu_ble = not a.tanpa_ble
    perlu_wifi = True
    udp_wifi = udp_ble_sim = ble = None
    if a.tanpa_ble and a.mode in ("ble", "adaptif", "semua"):
        sys.exit("--tanpa-ble hanya boleh dipakai bersama --mode wifi (uji awal).")

    kontrol = "serial" if a.port else "tcp"
    if a.port:
        print(f"Membuka serial {a.port} @ {a.baud} ...")
        esp = EspSerial(a.port, a.baud)
    else:
        # Mode adaptor: ESP32 mendapat daya dari adaptor (tidak tersambung USB ke laptop); kendali lewat Wi-Fi.
        print("Mode kendali Wi-Fi (ESP32 tanpa kabel USB ke laptop).")
        discover = (a.esp_ip == "auto")
        target = ("255.255.255.255", UDP_ESP_PORT) if discover else (a.esp_ip, UDP_ESP_PORT)
        print("Mencari ESP32 di jaringan ..." if discover else f"Menghubungi ESP32 di {a.esp_ip} ...")
        _, udp_wifi = await loop.create_datagram_endpoint(
            lambda: UdpProto(host, LINK_WIFI, target, discover=discover), local_addr=("0.0.0.0", 0), allow_broadcast=True)
        if not await udp_wifi.hello(timeout=10.0):
            sys.exit("ESP32 tidak ditemukan/tidak membalas HELLO. Cek: ESP32 menyala dan tersambung ke router (lampu/serial), "
                     "laptop di router yang sama (bukan jaringan tamu), firewall Windows mengizinkan Python pada jaringan privat. "
                     "Bila pencarian otomatis gagal, isi IP ESP32 dari daftar perangkat router: --esp-ip 192.168.x.x")
        if discover:
            a.esp_ip = udp_wifi.peer_ip
        print(f"ESP32 ditemukan di {a.esp_ip}")
        esp = EspTcp(a.esp_ip)
        try:
            await esp.connect()
        except Exception as e:
            sys.exit(f"Gagal membuka kanal kendali TCP ke {a.esp_ip}:{TCP_PORT} ({e}). "
                     f"Pastikan firmware versi gp-2.0 atau lebih baru sudah di-flash.")

    pong = None
    for _ in range(8):
        pong = await esp.command("PING", "#PONG", timeout=1.5)
        if pong:
            break
    if not pong:
        sys.exit("ESP32 tidak membalas PING. Cek: port COM benar, Serial Monitor Arduino ditutup, firmware sudah di-flash.")
    simulasi = "sim=1" in pong
    print("ESP32:", pong)

    if a.esp_ip == "auto":
        st0 = parse_kv(await esp.command("STATUS", "#STATUS") or "")
        ip = st0.get("ip", "-")
        if st0.get("wifi") != "1" or ip in ("-", "0.0.0.0"):
            sys.exit("ESP32 belum tersambung ke router. Periksa SSID dan sandi di firmware/gamepad/secrets.h, "
                     "pastikan router 2,4 GHz, lalu upload ulang firmware.")
        a.esp_ip = ip
        print(f"IP ESP32 di router: {a.esp_ip} (laptop harus berada di router yang sama)")
    if perlu_wifi and udp_wifi is None:
        print(f"Menyambung UDP ke {a.esp_ip}:{UDP_ESP_PORT} ...")
        _, udp_wifi = await loop.create_datagram_endpoint(
            lambda: UdpProto(host, LINK_WIFI, (a.esp_ip, UDP_ESP_PORT)), local_addr=("0.0.0.0", 0))
        if not await udp_wifi.hello():
            sys.exit("ESP32 tidak membalas HELLO lewat Wi-Fi. Pastikan laptop berada di router yang sama dengan ESP32 "
                     "(bukan jaringan tamu/terisolasi), dan firewall Windows mengizinkan Python pada jaringan privat.")
    print("  Wi-Fi/UDP terhubung.")
    if perlu_ble:
        if a.ble_sim:
            _, udp_ble_sim = await loop.create_datagram_endpoint(
                lambda: UdpProto(host, LINK_BLE, (a.esp_ip, UDP_SIM_BLE_PORT)), local_addr=("0.0.0.0", 0))
            if not await udp_ble_sim.hello():
                sys.exit("Simulator BLE tidak membalas.")
            print("  BLE (SIMULASI lewat UDP) terhubung.")
        else:
            print(f"Mencari ESP32 '{BLE_NAME}' lewat BLE ...")
            ble = BleLink(host)
            try:
                await ble.connect()
            except RuntimeError as e:
                sys.exit(str(e))
            ble.mulai_jaga()
            print("  BLE terhubung.")

    bagian = []
    if a.skor_hanya_latency:
        a.set = (a.set or []) + ["w_lat=1", "w_loss=0", "w_jit=0", "w_rssi=0"]
        bagian.append("latency_saja")
    if a.ambang_ketat:   # analisis sensitivitas: RTT baik 100 ms, buruk/hilang 200 ms (satu arah 50/100 ms)
        a.set = (a.set or []) + ["rtt_good_ms=100", "rtt_bad_ms=200", "miss_timeout_ms=200"]
        bagian.append("ambang_ketat")
    if bagian and not a.varian:
        a.varian = "_".join(bagian)
    for kv in a.set or []:
        k, v = kv.split("=", 1)
        r = await esp.command(f"SET {k} {v}", "#OK")
        if not r:
            sys.exit(f"Gagal SET {kv}")
    params_line = await esp.command("PARAMS", "#PARAMS") or ""
    status_line = await esp.command("STATUS", "#STATUS") or ""
    print("Status:", status_line)
    st = parse_kv(status_line) if status_line else {}
    if st.get("bno") == "0":
        print("  ! PERINGATAN: sensor BNO055 tidak terdeteksi (data gerak nol). Pengukuran komunikasi tetap berjalan.")

    out_root = a.out + ("_SIMULASI" if simulasi else "")
    seed = a.seed if a.seed is not None else random.randrange(10**6)
    plan = rencana(a.mode, a.ulang, seed)
    sid0 = random.randrange(1, 100)
    print(f"\nRencana: {len(plan)} percobaan, durasi {a.durasi} dtk, skenario '{a.skenario}', urutan acak (seed {seed}).")
    if simulasi:
        print("*** MODE SIMULASI: data ini BUKAN data skripsi dan akan disimpan terpisah ***")
    perkiraan = len(plan) * (a.durasi + 8) / 60
    print(f"Perkiraan waktu total tanpa jeda: {perkiraan:.0f} menit.\n")

    ringkasan = []
    for i, (rep, mode) in enumerate(plan, 1):
        nama = MODE_NAMA[mode]
        label = nama if (mode != 2 or not a.varian) else f"{nama}_{a.varian}"
        if not a.tanpa_jeda:
            await loop.run_in_executor(None, input,
                f"[{i}/{len(plan)}] Siap: skenario {a.skenario}, metode {label.upper()}, ulangan {rep}. "
                f"Atur kondisi uji lalu tekan Enter untuk mulai... ")
        # cek link sebelum mulai (semua metode butuh kedua link aktif: beban probe sama di semua metode)
        if not a.ble_sim and ble is not None and not ble.connected:
            print("  BLE belum tersambung, menunggu sambung ulang (maks 25 dtk) ...", flush=True)
            if not await ble.tunggu_tersambung(25.0):
                sys.exit("BLE tidak bisa disambung ulang. Pastikan Bluetooth laptop menyala, lalu jalankan ulang perintah "
                         "dengan --mode/--ulang sesuai sesi yang belum selesai.")
        s = await status_siap(esp, butuh_ble=not a.ble_sim and ble is not None)
        if not a.ble_sim and ble is not None and s.get("ble") != "1":
            sys.exit("ESP32 melaporkan BLE tidak terhubung walau laptop sudah tersambung. Matikan lalu nyalakan Bluetooth "
                     "laptop, atau cabut-colok adaptor ESP32, lalu jalankan ulang.")
        if mode in (1, 2) and (s.get("wifi") != "1" or s.get("hello") != "1"):
            sys.exit("Jalur Wi-Fi ESP32 tidak aktif (belum tersambung ke router atau belum ada HELLO dari laptop).")
        sid = (sid0 + i) % 256
        t_iso = datetime.now().isoformat(timespec="seconds")
        print(f"[{i}/{len(plan)}] {label.upper()} ulangan {rep} berjalan {a.durasi} dtk ...", flush=True)
        try:
            begin, rows, wall = await jalankan_sesi(esp, host, mode, sid, a.durasi)
        except RuntimeError as e:
            print(f"  GAGAL: {e}")
            continue
        ok, pesan = validasi(begin, rows, a.durasi, host)
        ringkasan_txt = ringkas(rows, host, begin)
        print(f"  {'OK ' if ok else 'RUSAK'}: {ringkasan_txt}")
        for p in pesan:
            print("   !", p)
        stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        folder = os.path.join(out_root, a.skenario, f"{i:02d}_{label}_r{rep:02d}_{stamp}")
        meta = {
            "skenario": a.skenario, "mode": mode, "mode_nama": nama, "metode_label": label, "varian": a.varian, "kontrol": kontrol,
            "laptop_koneksi": a.laptop_koneksi, "esp_ip": a.esp_ip, "ulangan": rep, "urutan_ke": i,
            "durasi_s": a.durasi, "sid": sid, "mulai": t_iso, "jarak": a.jarak, "catatan": a.catatan,
            "simulasi": simulasi, "valid": ok, "pesan_validasi": pesan, "injeksi": host.injeksi,
            "firmware": pong, "begin": begin, "params": params_line, "status_awal": status_line,
            "host": {"os": platform.platform(), "python": sys.version.split()[0],
                     "paket_basi": host.stale, "paket_salah_ukuran": host.bad},
            "rencana": {"seed": seed, "mode_arg": a.mode, "ulang": a.ulang},
        }
        simpan(folder, begin, rows, host, meta)
        ringkasan.append((nama, rep, ok))

    n_ok = sum(1 for r in ringkasan if r[2])
    print(f"\nSelesai. {n_ok}/{len(plan)} percobaan valid. Data: {os.path.abspath(os.path.join(out_root, a.skenario))}")
    print("Langkah berikut:  python analisis.py", os.path.join(out_root, a.skenario))
    if ble:
        await ble.close()
    esp.close()


def main():
    ap = argparse.ArgumentParser(description="Jalankan percobaan komunikasi BLE/Wi-Fi/Adaptif dan simpan data mentah.")
    ap.add_argument("--port", default=None,
                    help="port serial ESP32, mis. COM22. KOSONGKAN bila ESP32 diberi daya dari adaptor (kendali lewat Wi-Fi/TCP)")
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--mode", choices=["ble", "wifi", "adaptif", "semua"], default="semua")
    ap.add_argument("--ulang", type=int, default=5, help="jumlah ulangan per metode (bawaan 5)")
    ap.add_argument("--durasi", type=int, default=60, help="detik per percobaan (5-120)")
    ap.add_argument("--skenario", default="S1", help="label skenario, mis. S1..S5")
    ap.add_argument("--jarak", default="", help="catatan jarak, mis. '1 m'")
    ap.add_argument("--catatan", default="", help="catatan kondisi (interferensi, lokasi, dsb.)")
    ap.add_argument("--laptop-koneksi", choices=["ethernet", "wifi"], default="",
                    help="cara laptop tersambung ke router (dicatat; kabel LAN disarankan agar radio Wi-Fi laptop bukan variabel)")
    ap.add_argument("--varian", default="", help="label varian algoritma adaptif (mis. latency_saja) untuk pembanding")
    ap.add_argument("--ambang-ketat", action="store_true",
                    help="analisis sensitivitas: RTT baik 100 ms dan buruk 200 ms (satu arah 50/100 ms) sebagai varian pembanding")
    ap.add_argument("--skor-hanya-latency", action="store_true",
                    help="skor adaptif hanya memakai latency (w_lat=1, lainnya 0) sebagai varian pembanding")
    ap.add_argument("--out", default="data", help="folder keluaran (bawaan: data)")
    ap.add_argument("--esp-ip", default="auto", help="IP ESP32 di router; bawaan auto = dibaca dari perintah STATUS")
    ap.add_argument("--set", action="append", help="ubah parameter adaptif, mis. --set dwell_ms=2000 (boleh berulang)")
    ap.add_argument("--seed", type=int, help="seed urutan acak (agar urutan bisa diulang)")
    ap.add_argument("--tanpa-jeda", action="store_true", help="jalan terus tanpa menunggu Enter")
    ap.add_argument("--tanpa-ble", action="store_true", help="uji awal Wi-Fi saja (hanya dengan --mode wifi); jangan dipakai untuk data skripsi")
    ap.add_argument("--injeksi", default=None, metavar="LINK:JENIS:NILAI:MULAI-SELESAI",
                    help="UJI FUNGSIONAL SAJA (bukan data perbandingan): ganggu echo di laptop. "
                         "LINK=ble|wifi, JENIS=tunda (ms) | buang (persen) | putus (nilai diabaikan), waktu dalam detik sesi. "
                         "Contoh: ble:tunda:200:20-40, ble:buang:30:20-40, ble:putus:0:20-40")
    ap.add_argument("--ble-sim", action="store_true", help="HANYA untuk uji dengan simulator")
    a = ap.parse_args()
    if not 5 <= a.durasi <= 120:
        sys.exit("--durasi harus 5..120")
    if not a.laptop_koneksi:
        print("Catatan: --laptop-koneksi belum diisi (ethernet/wifi). Isi agar kondisi pengujian terdokumentasi.")
    try:
        asyncio.run(main_async(a))
    except KeyboardInterrupt:
        print("\nDihentikan pengguna.")


if __name__ == "__main__":
    main()