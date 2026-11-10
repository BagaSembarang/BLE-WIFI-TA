#!/usr/bin/env python3
"""
simulator_esp32.py - ESP32 PALSU untuk menguji eksperimen.py dan analisis.py tanpa perangkat keras.

!!! DATA DARI SIMULATOR BUKAN DATA SKRIPSI !!!
Simulator menandai dirinya (sim=1) sehingga eksperimen.py menyimpan data ke folder *_SIMULASI
dan analisis.py menolak memprosesnya kecuali dengan --izinkan-simulasi.

Pemakaian (Linux/macOS saja, memakai pty):
    python simulator_esp32.py            # mencetak nama port serial palsu
    python eksperimen.py --port <port itu> --esp-ip 127.0.0.1 --ble-sim --mode semua --ulang 2 --durasi 12 --tanpa-jeda
"""
import os
import pty
import random
import socket
import sys
import threading
import time
import tty

T_DATA, T_ECHO, T_PROBE = 1, 2, 3


class Sim:
    def __init__(self, drop_once=False):
        self.drop_once = drop_once      # uji ketahanan: putuskan TCP sekali di tengah pengiriman log
        self._dropped = False
        self.last_dump = []
        self.sess_chan = ("pty",)
        self.master, slave = pty.openpty()
        tty.setraw(slave)
        self.port_name = os.ttyname(slave)
        self.sock = {0: self._bind(4213), 1: self._bind(4210)}
        self.hostaddr = {0: None, 1: None}
        self.sid = None
        self.echo = {}          # (otype, seq) -> (t_ns, link)
        self.state = 0
        self.lock = threading.Lock()
        self.rnd = random.Random(7)
        for link in (0, 1):
            threading.Thread(target=self._udp_rx, args=(link,), daemon=True).start()
        threading.Thread(target=self._serial_rx, daemon=True).start()
        self.tcp = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.tcp.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.tcp.bind(("127.0.0.1", 4212))
        self.tcp.listen(2)
        threading.Thread(target=self._tcp_accept, daemon=True).start()

    @staticmethod
    def _bind(port):
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        s.bind(("127.0.0.1", port))
        return s

    def out(self, text, chan=None):
        chan = chan or ("pty",)
        if chan[0] == "pty":
            os.write(self.master, (text + "\n").encode())
        else:
            try:
                chan[1].sendall((text + "\n").encode())
            except OSError:
                pass

    # -------------------------------------------------------------- TCP (kendali lewat Wi-Fi)
    def _tcp_accept(self):
        while True:
            conn, _ = self.tcp.accept()
            threading.Thread(target=self._tcp_rx, args=(conn,), daemon=True).start()

    def _tcp_rx(self, conn):
        buf = b""
        try:
            while True:
                chunk = conn.recv(1024)
                if not chunk:
                    break
                buf += chunk
                while b"\n" in buf:
                    line, buf = buf.split(b"\n", 1)
                    self.cmd(line.decode().strip(), ("tcp", conn))
        except OSError:
            pass

    # -------------------------------------------------------------- UDP
    def _udp_rx(self, link):
        s = self.sock[link]
        while True:
            data, addr = s.recvfrom(256)
            t = time.perf_counter_ns()
            if data == b"HELLO":
                self.hostaddr[link] = addr
                s.sendto(b"HACK", addr)
            elif len(data) == 8 and data[0] == T_ECHO and data[2] == self.sid:
                with self.lock:
                    self.echo[(data[3], int.from_bytes(data[4:8], "little"))] = (t, link)

    # -------------------------------------------------------------- serial
    def _serial_rx(self):
        buf = b""
        while True:
            buf += os.read(self.master, 1024)
            while b"\n" in buf:
                line, buf = buf.split(b"\n", 1)
                self.cmd(line.decode().strip())

    def cmd(self, line, chan=("pty",)):
        if line == "PING":
            self.out("#PONG fw=sim-1.0 sim=1", chan)
        elif line == "STATUS":
            self.out(f"#STATUS state={self.state} ble=1 wifi=1 ip=127.0.0.1 ch=6 hello={int(all(self.hostaddr.values()))} bno=1 imuerr=0 heap=0 rssi_ble=-60 rssi_wifi=-50 tcp=1", chan)
        elif line == "PARAMS":
            self.out("#PARAMS window_ms=1500 t_leave=60.000 dwell_ms=3000 (simulator)", chan)
        elif line.startswith("SET "):
            self.out("#OK " + line.split()[1], chan)
        elif line == "ABORT":
            self.out("#ACK ABORT", chan)
        elif line == "DUMP":
            if self.state != 0:
                self.out("#ERR sesi berjalan", chan)
            elif not self.last_dump:
                self.out("#ERR tidak ada data", chan)
            else:
                self._send_dump(chan, allow_drop=False)
        elif line.startswith("START "):
            _, mode, dur, sid = line.split()
            self.sess_chan = chan
            self.out(f"#ACK START mode={mode} dur={dur} sid={sid}", chan)
            threading.Thread(target=self.session, args=(int(mode), int(dur), int(sid)), daemon=True).start()

    # -------------------------------------------------------------- model kanal
    def channel(self, link, t, dur):
        """(peluang_hilang, rtt_tambahan_ms). BLE memburuk pada 40%-70% durasi."""
        deg = 0.4 * dur <= t < 0.7 * dur
        if link == 0:
            if deg:
                return 0.45, max(5.0, self.rnd.gauss(110, 25))
            return 0.004, max(5.0, self.rnd.gauss(30, 7))
        spike = 30 if self.rnd.random() < 0.02 else 0
        return 0.003, max(1.0, self.rnd.gauss(6, 2) + spike)

    def session(self, mode, dur, sid):
        self.sid, self.state = sid, 1
        with self.lock:
            self.echo.clear()
        t0 = time.perf_counter_ns()
        D, P, Q, E = [], [], [], []
        cur = 0 if mode in (0, 2) else 1
        t_sw1, t_sw2 = 0.4 * dur + 0.4, 0.7 * dur + 3.5
        nxt_d = nxt_p = 0.0
        seq_d = seq_p = 0
        pend = []
        while True:
            t = (time.perf_counter_ns() - t0) / 1e9
            if t >= dur:
                break
            if mode == 2:
                if cur == 0 and t_sw1 <= t < t_sw2:
                    cur = 1
                    E.append((int(t * 1e6), 1, 0, 1, 1, 1, 480, 960, 0, 3100, 50, 250, -70))
                    pend.append(("ho", t, 1))
                elif cur == 1 and t >= t_sw2 and not any(e[1] == 1 and e[3] == 0 for e in E):
                    cur = 0
                    E.append((int(t * 1e6), 1, 1, 0, 3, 0, 900, 850, 0, 600, 0, 40, -55))
            if t >= nxt_d:
                nxt_d += 0.02
                seq_d += 1
                link = cur
                D.append(self._emit(T_DATA, link, seq_d, t, dur, sid))
            if t >= nxt_p:
                nxt_p += 0.1
                seq_p += 1
                link = (cur ^ 1) if mode == 2 else (1 if mode == 0 else 0)
                P.append(self._emit(T_PROBE, link, seq_p, t, dur, sid))
            time.sleep(0.002)
        time.sleep(1.0)
        # susun hasil
        def fin(rec, otype):
            typ, link, seq, tx, extra_ms, dropped, nl = rec
            with self.lock:
                e = self.echo.get((otype, seq))
            if dropped or e is None:
                return [seq, int(tx * 1e6), 0, link, 255, -60, -50]
            real_us = (e[0] - t0) / 1e3 - tx * 1e6
            rtt = int(max(1, real_us + extra_ms * 1000))
            return [seq, int(tx * 1e6), rtt, link, e[1], -60 - self.rnd.randint(0, 6), -50 - self.rnd.randint(0, 4)]
        Dr = [fin(r, T_DATA) for r in D]
        Pr = [fin(r, T_PROBE) for r in P]
        for k in range(int(dur * 10)):
            t = k * 0.1
            deg = 0.4 * dur <= t < 0.7 * dur
            sb = 350 if deg else 950
            Q.append([int(t * 1000), sb, 960, 1 if (mode == 2 and any(e[0] / 1e6 <= t for e in E if e[3] == 1)) else (0 if mode != 1 else 1)])
        # handover kind 2
        for e in list(E):
            if e[1] == 1:
                E.append((e[0] + 45000, 2, 0, e[3], 0, 0, 0, 0, 45000, 0, 0, 0, 0))
        lines = [f"#BEGIN sid={sid} mode={mode} dur={dur} nd={len(Dr)} np={len(Pr)} nq={len(Q)} ne={len(E)} bno=1 fw=sim-1.0 sim=1"]
        lines += ["D," + ",".join(map(str, r)) for r in Dr]
        lines += ["P," + ",".join(map(str, r)) for r in Pr]
        lines += ["Q," + ",".join(map(str, r)) for r in Q]
        lines += ["E," + ",".join(map(str, e)) for e in E]
        lines.append("#END")
        self.last_dump = lines
        self.state = 0
        self._send_dump(self.sess_chan, allow_drop=True)

    def _send_dump(self, chan, allow_drop):
        for i, ln in enumerate(self.last_dump):
            if allow_drop and self.drop_once and not self._dropped and chan[0] == "tcp" and i == 150:
                self._dropped = True
                try:
                    chan[1].shutdown(socket.SHUT_RDWR)
                    chan[1].close()
                except OSError:
                    pass
                return                      # koneksi putus di tengah log (uji ketahanan)
            self.out(ln, chan)

    def _emit(self, typ, link, seq, t, dur, sid):
        loss, extra = self.channel(link, t, dur)
        dropped = self.rnd.random() < loss
        addr = self.hostaddr[link]
        if not dropped and addr is not None:
            pkt = bytearray(32)
            pkt[0], pkt[1], pkt[2] = typ, link, sid
            pkt[4:8] = seq.to_bytes(4, "little")
            s = sum(pkt[:30]) & 0xFFFF
            pkt[30], pkt[31] = s & 0xFF, s >> 8
            self.sock[link].sendto(bytes(pkt), addr)
        return (typ, link, seq, t, extra, dropped, 0)


if __name__ == "__main__":
    sim = Sim(drop_once="--drop-tcp" in sys.argv)
    print(sim.port_name, flush=True)
    print("Simulator ESP32 berjalan (Ctrl+C untuk berhenti). INGAT: data simulasi BUKAN data skripsi.", file=sys.stderr)
    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        pass
