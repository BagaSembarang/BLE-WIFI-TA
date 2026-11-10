/*
 * gamepad.ino - Pengontrol game gerakan ESP32-C3 + BNO055, uji komunikasi BLE vs Wi-Fi vs Adaptif
 *
 * PRINSIP DESAIN (agar data valid):
 *  1. Semua waktu diukur di ESP32 memakai SATU jam (esp_timer, mikrodetik). Paket TIDAK membawa
 *     timestamp; ESP32 menyimpan waktu kirim per nomor urut dan menghitung RTT saat echo kembali.
 *  2. Selama sesi TIDAK ada Serial.print per paket (tidak mengganggu pengukuran). Semua catatan
 *     disimpan di RAM, lalu dikirim ke laptop setelah sesi selesai.
 *  3. Satu firmware untuk tiga metode (BLE statis, Wi-Fi statis, Adaptif), dipilih lewat perintah
 *     serial START. Radio BLE dan Wi-Fi selalu aktif dan beban probe sama di semua metode.
 *  4. Packet loss = tidak ada echo dalam 1 detik (bukan 100 ms). Echo yang lebih lambat dari 100 ms
 *     tetap tercatat dengan RTT-nya (dihitung sebagai "terlambat" saat analisis, bukan "hilang").
 *
 * KONTROL: perintah dan pengambilan log bisa lewat USB serial ATAU lewat TCP port 4212 (Wi-Fi), dengan
 *   protokol baris yang sama. Mode TCP memungkinkan ESP32 diberi daya dari adaptor (tanpa kabel ke laptop).
 * PROTOKOL (laptop -> ESP32, satu baris): DUMP (kirim ulang log sesi terakhir) | PING | STATUS | PARAMS | SET <kunci> <nilai> |
 *   START <mode 0=BLE 1=WIFI 2=ADAPTIF> <durasi_detik> <sid 0..255> | ABORT
 * ESP32 -> laptop: baris berawalan '#' (kontrol) dan, setelah sesi, blok data #BEGIN ... #END.
 *

 * WI-FI: ESP32 mode STA, tersambung ke ROUTER (SSID/sandi di secrets.h). Laptop harus berada di router
 * yang sama (disarankan kabel LAN). Alamat IP ESP32 dilaporkan lewat perintah STATUS (ip=...).
 *
 * Paket DATA/PROBE (ESP32 -> laptop, 32 byte): [0]=tipe(1 DATA,3 PROBE) [1]=link(0 BLE,1 WIFI)
 *   [2]=sid [3]=0 [4..7]=seq u32 LE [8..15]=quat(4xint16) [16..21]=lia(3xint16) [22..27]=gyro(3xint16)
 *   [28..29]=0 [30..31]=checksum (jumlah byte 0..29, u16 LE)
 * Paket ECHO (laptop -> ESP32, 8 byte): [0]=2 [1]=link [2]=sid [3]=tipe asal [4..7]=seq u32 LE
 */
#include <Arduino.h>
#include <stdarg.h>
#include <Wire.h>
#include <WiFi.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include <esp_gap_ble_api.h>
#include <esp_wifi.h>
#include <esp_timer.h>
#include <lwip/sockets.h>
#include "adaptive_core.h"
#include "secrets.h"      // WIFI_SSID dan WIFI_PASS (JANGAN di-commit ke repo publik)

#define FW_VERSION "gp-2.0"

// ---------------- Konfigurasi (ubah di sini) ----------------
#define I2C_SDA        2        // ESP32-C3 (sama dengan firmware lama)
#define I2C_SCL        3
#define UDP_PORT       4210
#define BLE_NAME       "GAMEPAD-ESP32"
#define BLE_SERVICE_UUID  "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
#define BLE_CHAR_TX_UUID  "beb5483e-36e1-4688-b7f5-ea07361b26a8"   // ESP32 -> laptop (notify)
#define BLE_CHAR_RX_UUID  "d29759bc-4b6f-45a6-8e5s0-96f7e44fa70c"   // laptop -> ESP32 (write no response)

#define PKT_LEN          32
#define ECHO_LEN         8
#define T_DATA           1
#define T_ECHO           2
#define T_PROBE          3
#define DATA_PERIOD_US   20000     // 50 Hz
#define PROBE_PERIOD_US  50000     // 20 Hz pada link non-data (probe link cadangan)
#define DRAIN_US         1000000   // tunggu echo terakhir (batas packet loss = 1 detik)
#define MODE_BLE   0
#define MODE_WIFI  1
#define MODE_ADAPT 2
#define ST_IDLE 0
#define ST_RUN  1
#define ST_DRAIN 2
#define TCP_PORT 4212
#define CH_SERIAL 0
#define CH_TCP    1

// ---------------- Struktur catatan ----------------
struct Rec  { uint32_t tx_us; uint32_t rtt_us; uint8_t ltx; uint8_t lrx; int8_t rb; int8_t rw; };  // ltx: bit0=link, bit1=miss sudah dihitung
struct QRec { uint32_t t_ms; int16_t sb; int16_t sw; uint8_t act; };
struct ERec { uint32_t t_us; uint32_t extra; int16_t sfrom, sto, p90, loss, jit; uint8_t kind, from, to, reason, cause; int8_t rssi; };
struct EchoEv { int64_t rx_us; uint32_t seq; uint8_t link; uint8_t otype; uint8_t sid; };

static Rec*  recD = nullptr; static Rec* recP = nullptr; static QRec* recQ = nullptr;
static ERec  recE[64];
static uint32_t nD, nP, nQ, nE, capD, capP, capQ, scanD, scanP;
static int64_t  sessStartUs;
static uint8_t  sessSid, sessMode;
static uint32_t sessDurS;
static volatile uint8_t state = ST_IDLE;
static volatile bool abortReq = false;
static int64_t  drainEndUs;
static bool     hoPending = false; static int64_t hoUs; static uint8_t hoTo;

static AdaptiveCore core;
static inline int64_t missTimeoutUs() { return (int64_t)core.p.miss_timeout_ms * 1000; }
static QueueHandle_t echoQ;

// ---------------- Konsol perintah: USB serial atau TCP lewat Wi-Fi ----------------
static volatile int tcpClient = -1;               // soket klien TCP aktif (dikelola tcpTask)
static volatile uint8_t sessChan = CH_SERIAL;     // saluran yang memulai sesi (tujuan pengiriman log)
static SemaphoreHandle_t cmdMutex;
static void cwrite(uint8_t chan, const char* s, size_t n) {
  if (chan == CH_SERIAL) { Serial.write((const uint8_t*)s, n); return; }
  int c = tcpClient; size_t off = 0;
  while (c >= 0 && off < n) {
    int r = send(c, s + off, n - off, 0);
    if (r <= 0) { shutdown(c, SHUT_RDWR); return; }   // tcpTask yang menutup soket
    off += (size_t)r;
  }
}
static void cprintf(uint8_t chan, const char* fmt, ...) {
  char buf[200]; va_list ap; va_start(ap, fmt);
  int n = vsnprintf(buf, sizeof(buf), fmt, ap); va_end(ap);
  if (n > 0) cwrite(chan, buf, (size_t)(n < (int)sizeof(buf) ? n : (int)sizeof(buf) - 1));
}
static char dumpBuf[1400]; static size_t dumpLen = 0;      // baris log dikirim berkelompok (efisien lewat TCP)
static void dflush(uint8_t chan) { if (dumpLen) { cwrite(chan, dumpBuf, dumpLen); dumpLen = 0; } }
static void dadd(uint8_t chan, const char* fmt, ...) {
  char t[200]; va_list ap; va_start(ap, fmt);
  int n = vsnprintf(t, sizeof(t), fmt, ap); va_end(ap);
  if (n <= 0) return;
  if (n > (int)sizeof(t) - 1) n = (int)sizeof(t) - 1;
  if (dumpLen + (size_t)n > sizeof(dumpBuf)) dflush(chan);
  memcpy(dumpBuf + dumpLen, t, (size_t)n); dumpLen += (size_t)n;
}

// Status link
static volatile bool bleConnected = false, hasBda = false, hostKnown = false;
static volatile int8_t bleRssi = -99, wifiRssi = -99;
static esp_bd_addr_t remoteBda;
static BLECharacteristic* pTx = nullptr;
static int udpSock = -1;
static struct sockaddr_in hostAddr;

// IMU
static uint8_t imuBuf[26];          // 0x14..0x2D: gyro(6) eul(6) quat(8) lia(6)
static volatile bool bnoOk = false;
static volatile uint32_t imuErr = 0;
static uint8_t bnoAddr = 0x28;

// ---------------- Parameter yang bisa diubah lewat SET ----------------
struct KV { const char* k; void* p; char t; };
static KV kvs[] = {
  {"window_ms", &core.p.window_ms, 'u'}, {"eval_ms", &core.p.eval_ms, 'u'}, {"min_samples", &core.p.min_samples, 'b'},
  {"loss_bad", &core.p.loss_bad, 'f'}, {"rtt_good_ms", &core.p.rtt_good_ms, 'f'}, {"rtt_bad_ms", &core.p.rtt_bad_ms, 'f'},
  {"jit_bad_ms", &core.p.jit_bad_ms, 'f'}, {"rssi_good", &core.p.rssi_good, 'f'}, {"rssi_bad", &core.p.rssi_bad, 'f'},
  {"w_loss", &core.p.w_loss, 'f'}, {"w_lat", &core.p.w_lat, 'f'}, {"w_jit", &core.p.w_jit, 'f'}, {"w_rssi", &core.p.w_rssi, 'f'},
  {"t_leave", &core.p.t_leave, 'f'}, {"margin", &core.p.margin, 'f'}, {"t_return", &core.p.t_return, 'f'},
  {"return_tol", &core.p.return_tol, 'f'}, {"consec_leave", &core.p.consec_leave, 'b'}, {"consec_return", &core.p.consec_return, 'b'},
  {"dwell_ms", &core.p.dwell_ms, 'u'}, {"emerg_gap_ms", &core.p.emerg_gap_ms, 'u'},
  {"standby_alive_ms", &core.p.standby_alive_ms, 'u'}, {"emerg_min_score", &core.p.emerg_min_score, 'f'}, {"preferred", &core.p.preferred, 'b'},
  {"miss_timeout_ms", &core.p.miss_timeout_ms, 'u'}, {"emerg_silence_ms", &core.p.emerg_silence_ms, 'u'},
};
#define NKV (sizeof(kvs) / sizeof(kvs[0]))

// ---------------- BNO055 ----------------
static bool bnoWrite(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(bnoAddr); Wire.write(reg); Wire.write(val);
  return Wire.endTransmission() == 0;
}
static bool bnoRead(uint8_t reg, uint8_t* buf, uint8_t len) {
  Wire.beginTransmission(bnoAddr); Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom((int)bnoAddr, (int)len) != len) return false;
  for (int i = 0; i < len; i++) buf[i] = Wire.read();
  return true;
}
static void initBNO055() {
  Wire.begin(I2C_SDA, I2C_SCL, 100000);
  Wire.setTimeOut(1000);
  delay(500);
  uint8_t id = 0;
  for (int a = 0; a < 2 && !bnoOk; a++) {
    bnoAddr = (a == 0) ? 0x28 : 0x29;
    for (int t = 0; t < 5; t++) { if (bnoRead(0x00, &id, 1) && id == 0xA0) { bnoOk = true; break; } delay(50); }
  }
  if (bnoOk) {
    bnoWrite(0x3D, 0x00); delay(30);   // OPR_MODE = CONFIG
    bnoWrite(0x07, 0x00);              // PAGE_ID 0
    bnoWrite(0x3E, 0x00);              // PWR_MODE normal
    bnoWrite(0x3D, 0x0C); delay(50);   // OPR_MODE = NDOF
  }
}
static void sensorTask(void*) {
  for (;;) {
    if (bnoOk) {
      uint8_t tmp[26];
      if (bnoRead(0x14, tmp, 26)) memcpy(imuBuf, tmp, 26); else imuErr++;
    }
    vTaskDelay(pdMS_TO_TICKS(10));
  }
}

// ---------------- Echo masuk (dua sumber) ----------------
static inline void pushEcho(int64_t t_us, const uint8_t* d, uint8_t link) {
  EchoEv e; e.rx_us = t_us; e.link = link; e.sid = d[2]; e.otype = d[3];
  e.seq = (uint32_t)d[4] | ((uint32_t)d[5] << 8) | ((uint32_t)d[6] << 16) | ((uint32_t)d[7] << 24);
  xQueueSend(echoQ, &e, 0);
}

static void gapCallback(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t* param) {
  if (event == ESP_GAP_BLE_READ_RSSI_COMPLETE_EVT && param->read_rssi_cmpl.status == ESP_BT_STATUS_SUCCESS)
    bleRssi = param->read_rssi_cmpl.rssi;
}
class SrvCb : public BLEServerCallbacks {
  void onConnect(BLEServer* s, esp_ble_gatts_cb_param_t* param) {
    bleConnected = true;
    memcpy(remoteBda, param->connect.remote_bda, sizeof(esp_bd_addr_t)); hasBda = true;
    esp_ble_conn_update_params_t cp; memset(&cp, 0, sizeof(cp));
    memcpy(cp.bda, param->connect.remote_bda, sizeof(esp_bd_addr_t));
    cp.min_int = 0x06; cp.max_int = 0x0C; cp.latency = 0; cp.timeout = 400;   // minta interval 7.5-15 ms
    esp_ble_gap_update_conn_params(&cp);
  }
  void onDisconnect(BLEServer* s) {
    bleConnected = false; hasBda = false; bleRssi = -99;
    delay(50); s->startAdvertising();
  }
};
class RxCb : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic* c) {
    int64_t t = esp_timer_get_time();                 // stempel SEGERA saat echo tiba
    if (c->getLength() == ECHO_LEN) { const uint8_t* d = c->getData(); if (d[0] == T_ECHO) pushEcho(t, d, AC_LINK_BLE); }
  }
};
static void udpTask(void*) {
  uint8_t buf[64]; struct sockaddr_in from; socklen_t fl;
  for (;;) {
    fl = sizeof(from);
    int n = recvfrom(udpSock, buf, sizeof(buf), 0, (struct sockaddr*)&from, &fl);
    int64_t t = esp_timer_get_time();                 // stempel SEGERA setelah paket tiba
    if (n <= 0) continue;
    if (n == 5 && memcmp(buf, "HELLO", 5) == 0) {
      hostAddr = from; hostKnown = true;
      sendto(udpSock, "HACK", 4, 0, (struct sockaddr*)&from, fl);
      continue;
    }
    if (n == ECHO_LEN && buf[0] == T_ECHO) pushEcho(t, buf, AC_LINK_WIFI);
  }
}

// ---------------- Kirim paket ----------------
static void buildPkt(uint8_t* b, uint8_t type, uint8_t link, uint8_t sid, uint32_t seq, bool motion) {
  memset(b, 0, PKT_LEN);
  b[0] = type; b[1] = link; b[2] = sid;
  b[4] = seq & 0xFF; b[5] = (seq >> 8) & 0xFF; b[6] = (seq >> 16) & 0xFF; b[7] = (seq >> 24) & 0xFF;
  if (motion) { memcpy(b + 8, imuBuf + 12, 8); memcpy(b + 16, imuBuf + 20, 6); memcpy(b + 22, imuBuf + 0, 6); }
  uint16_t sum = 0; for (int i = 0; i < 30; i++) sum += b[i];
  b[30] = sum & 0xFF; b[31] = (sum >> 8) & 0xFF;
}
static inline void sendOnLink(uint8_t link, uint8_t* pkt) {
  if (link == AC_LINK_BLE) { if (bleConnected) { pTx->setValue(pkt, PKT_LEN); pTx->notify(); } }
  else if (hostKnown && WiFi.status() == WL_CONNECTED) sendto(udpSock, pkt, PKT_LEN, 0, (struct sockaddr*)&hostAddr, sizeof(hostAddr));
}
static inline uint32_t relMs(int64_t abs_us) { return (uint32_t)((abs_us - sessStartUs) / 1000); }

static void sendPacket(bool isData) {
  uint8_t link;
  if (isData) link = (sessMode == MODE_BLE) ? AC_LINK_BLE : (sessMode == MODE_WIFI) ? AC_LINK_WIFI : core.active;
  else        link = (uint8_t)((sessMode == MODE_ADAPT) ? (core.active ^ 1) : (sessMode == MODE_BLE ? AC_LINK_WIFI : AC_LINK_BLE));
  Rec* arr = isData ? recD : recP; uint32_t& n = isData ? nD : nP; uint32_t cap = isData ? capD : capP;
  if (n >= cap) return;
  uint32_t seq = n + 1;
  uint8_t pkt[PKT_LEN];
  buildPkt(pkt, isData ? T_DATA : T_PROBE, link, sessSid, seq, isData);
  Rec& r = arr[n++];
  r.rtt_us = 0; r.lrx = 255; r.ltx = link; r.rb = bleRssi; r.rw = wifiRssi;
  int64_t t = esp_timer_get_time();                   // stempel kirim tepat sebelum dikirim
  r.tx_us = (uint32_t)(t - sessStartUs);
  sendOnLink(link, pkt);
}

// ---------------- Pengolahan hasil ----------------
static void addEvent(uint32_t t_us, uint8_t kind, uint8_t from, uint8_t to, uint8_t reason, uint8_t cause,
                     int16_t sfrom, int16_t sto, int16_t p90, int16_t loss, int16_t jit, int8_t rssi, uint32_t extra) {
  if (nE >= 64) return;
  ERec& e = recE[nE++];
  e.t_us = t_us; e.extra = extra; e.sfrom = sfrom; e.sto = sto; e.p90 = p90; e.loss = loss; e.jit = jit;
  e.kind = kind; e.from = from; e.to = to; e.reason = reason; e.cause = cause; e.rssi = rssi;
}
static void processEcho(const EchoEv& ev) {
  if (state == ST_IDLE || ev.sid != sessSid) return;
  bool isData = (ev.otype == T_DATA);
  if (!isData && ev.otype != T_PROBE) return;
  Rec* arr = isData ? recD : recP; uint32_t n = isData ? nD : nP;
  if (ev.seq == 0 || ev.seq > n) return;
  Rec& r = arr[ev.seq - 1];
  if (r.rtt_us != 0) return;                           // echo ganda: abaikan
  int64_t rtt = (ev.rx_us - sessStartUs) - (int64_t)r.tx_us;
  if (rtt < 1) rtt = 1;
  if (rtt > 4000000000LL) rtt = 4000000000LL;
  r.rtt_us = (uint32_t)rtt; r.lrx = ev.link;
  uint8_t ltx = r.ltx & 1; uint32_t nowMs = relMs(ev.rx_us);
  if (!(r.ltx & 2)) {                                  // belum dicatat sebagai miss
    if (rtt <= missTimeoutUs()) core.onResult(ltx, true, (uint32_t)rtt, nowMs, isData);
    else { core.onResult(ltx, false, 0, nowMs, isData); r.ltx |= 2; }
  }
  if (hoPending && isData && ev.link == hoTo) {        // paket DATA pertama yang kembali lewat link baru
    addEvent((uint32_t)(ev.rx_us - sessStartUs), 2, 0, hoTo, 0, 0, 0, 0, 0, 0, 0, 0, (uint32_t)(ev.rx_us - hoUs));
    hoPending = false;
  }
}
static void scanMisses(int64_t nowUs) {
  uint32_t nowRel = (uint32_t)(nowUs - sessStartUs);
  for (int pass = 0; pass < 2; pass++) {
    Rec* arr = pass == 0 ? recD : recP; uint32_t n = pass == 0 ? nD : nP; uint32_t& sc = pass == 0 ? scanD : scanP;
    while (sc < n) {
      Rec& r = arr[sc];
      if ((int64_t)r.tx_us + missTimeoutUs() > (int64_t)nowRel) break;
      if (r.rtt_us == 0 && !(r.ltx & 2)) { r.ltx |= 2; core.onResult(r.ltx & 1, false, 0, nowRel / 1000, pass == 0); }
      sc++;
    }
  }
}

// ---------------- Sesi ----------------
static void freeSession() {
  free(recD); free(recP); free(recQ); recD = recP = nullptr; recQ = nullptr;
}
static void dumpSession(uint8_t chan) {
  dumpLen = 0;
  dadd(chan, "#BEGIN sid=%u mode=%u dur=%lu nd=%lu np=%lu nq=%lu ne=%lu bno=%u fw=%s\n", sessSid, sessMode,
       (unsigned long)sessDurS, (unsigned long)nD, (unsigned long)nP, (unsigned long)nQ, (unsigned long)nE,
       bnoOk ? 1 : 0, FW_VERSION);
  for (uint32_t i = 0; i < nD; i++)
    dadd(chan, "D,%lu,%lu,%lu,%u,%u,%d,%d\n", (unsigned long)(i + 1), (unsigned long)recD[i].tx_us, (unsigned long)recD[i].rtt_us,
         recD[i].ltx & 1, recD[i].lrx, recD[i].rb, recD[i].rw);
  for (uint32_t i = 0; i < nP; i++)
    dadd(chan, "P,%lu,%lu,%lu,%u,%u,%d,%d\n", (unsigned long)(i + 1), (unsigned long)recP[i].tx_us, (unsigned long)recP[i].rtt_us,
         recP[i].ltx & 1, recP[i].lrx, recP[i].rb, recP[i].rw);
  for (uint32_t i = 0; i < nQ; i++)
    dadd(chan, "Q,%lu,%d,%d,%u\n", (unsigned long)recQ[i].t_ms, recQ[i].sb, recQ[i].sw, recQ[i].act);
  for (uint32_t i = 0; i < nE; i++)
    dadd(chan, "E,%lu,%u,%u,%u,%u,%u,%d,%d,%lu,%d,%d,%d,%d\n", (unsigned long)recE[i].t_us, recE[i].kind, recE[i].from, recE[i].to,
         recE[i].reason, recE[i].cause, recE[i].sfrom, recE[i].sto, (unsigned long)recE[i].extra,
         recE[i].p90, recE[i].loss, recE[i].jit, recE[i].rssi);
  dadd(chan, "#END\n");
  dflush(chan);
  if (chan == CH_SERIAL) Serial.flush();
}
static void sampleRssi(int64_t nowUs) {
  if (bleConnected && hasBda) esp_ble_gap_read_rssi(remoteBda);
  if (WiFi.status() == WL_CONNECTED) { wifiRssi = (int8_t)WiFi.RSSI(); core.onRssi(AC_LINK_WIFI, wifiRssi, relMs(nowUs)); }
  else wifiRssi = -99;
  if (bleConnected) core.onRssi(AC_LINK_BLE, bleRssi, relMs(nowUs));
}

static void txTask(void*) {
  int64_t nextData = 0, nextProbe = 0, nextRssi = 0, nextQ = 0;
  uint8_t lastState = ST_IDLE;
  EchoEv ev;
  for (;;) {
    vTaskDelay(1);
    int64_t now = esp_timer_get_time();
    uint8_t st = state;
    while (xQueueReceive(echoQ, &ev, 0) == pdTRUE) processEcho(ev);
    if (st == ST_IDLE) { lastState = ST_IDLE; continue; }
    if (lastState == ST_IDLE) {                         // sesi baru dimulai
      nextData = nextProbe = now; nextRssi = now; nextQ = now + 100000;
    }
    lastState = st;
    scanMisses(now);
    if (sessMode == MODE_ADAPT && st == ST_RUN && core.evaluate(relMs(now))) {
      const AcDetail& d = core.last_detail;
      addEvent((uint32_t)(now - sessStartUs), 1, core.last_from, core.last_to, core.last_reason, core.last_cause,
               (int16_t)(core.last_score_from * 10), (int16_t)(core.last_score_to * 10),
               (int16_t)(d.p90_ms * 10), (int16_t)(d.loss * 1000), (int16_t)(d.jit_ms * 10), (int8_t)d.rssi, 0);
      hoPending = true; hoUs = now; hoTo = core.last_to;
    }
    if (now >= nextRssi) { nextRssi = now + 500000; sampleRssi(now); }
    if (now >= nextQ && nQ < capQ) {
      nextQ += 100000; uint32_t t = relMs(now);
      float sb = ac_score(core.win[0], t, core.p), sw = ac_score(core.win[1], t, core.p);
      recQ[nQ++] = {t, (int16_t)(sb * 10), (int16_t)(sw * 10), core.active};
    }
    if (st == ST_RUN) {
      if (now >= nextData)  { nextData += DATA_PERIOD_US; if (now - nextData > 5 * DATA_PERIOD_US) nextData = now + DATA_PERIOD_US; sendPacket(true); }
      if (now >= nextProbe) { nextProbe += PROBE_PERIOD_US; if (now - nextProbe > 5 * PROBE_PERIOD_US) nextProbe = now + PROBE_PERIOD_US; sendPacket(false); }
      if (abortReq || (now - sessStartUs) >= (int64_t)sessDurS * 1000000LL) { drainEndUs = now + DRAIN_US; state = ST_DRAIN; abortReq = false; }
    } else if (st == ST_DRAIN && now >= drainEndUs) {
      scanMisses(now + 2 * DRAIN_US);
      dumpSession(sessChan);
      state = ST_IDLE;                 // data tetap di RAM sampai START berikutnya (dapat diminta ulang dengan DUMP)
    }
  }
}

static bool startSession(uint8_t mode, uint32_t dur, uint8_t sid) {
  freeSession();                   // bebaskan log sesi sebelumnya
  capD = dur * 50 + 100; capP = dur * 20 + 50; capQ = dur * 10 + 30;
  recD = (Rec*)malloc(capD * sizeof(Rec)); recP = (Rec*)malloc(capP * sizeof(Rec)); recQ = (QRec*)malloc(capQ * sizeof(QRec));
  if (!recD || !recP || !recQ) { freeSession(); return false; }
  nD = nP = nQ = nE = scanD = scanP = 0; hoPending = false;
  sessMode = mode; sessSid = sid; sessDurS = dur;
  uint8_t initial = (mode == MODE_WIFI) ? AC_LINK_WIFI : AC_LINK_BLE;
  xQueueReset(echoQ);
  sessStartUs = esp_timer_get_time();
  core.reset(0, initial);
  state = ST_RUN;
  return true;
}

// ---------------- Perintah serial ----------------
static void printParams(uint8_t chan) {
  cprintf(chan, "#PARAMS");
  for (size_t i = 0; i < NKV; i++) {
    if (kvs[i].t == 'f') cprintf(chan, " %s=%.3f", kvs[i].k, *(float*)kvs[i].p);
    else if (kvs[i].t == 'u') cprintf(chan, " %s=%lu", kvs[i].k, (unsigned long)*(uint32_t*)kvs[i].p);
    else cprintf(chan, " %s=%u", kvs[i].k, *(uint8_t*)kvs[i].p);
  }
  cprintf(chan, "\n");
}
static void handleCmdInner(String line, uint8_t chan) {
  line.trim();
  if (line == "PING") { cprintf(chan, "#PONG fw=%s\n", FW_VERSION); return; }
  if (line == "STATUS") {
    bool wc = (WiFi.status() == WL_CONNECTED);
    cprintf(chan, "#STATUS state=%u ble=%d wifi=%d ip=%s ch=%d hello=%d bno=%d imuerr=%lu heap=%u rssi_ble=%d rssi_wifi=%d tcp=%d\n",
            state, bleConnected ? 1 : 0, wc ? 1 : 0, wc ? WiFi.localIP().toString().c_str() : "-", wc ? WiFi.channel() : 0,
            hostKnown ? 1 : 0, bnoOk ? 1 : 0, (unsigned long)imuErr, (unsigned)ESP.getFreeHeap(), bleRssi, wifiRssi,
            tcpClient >= 0 ? 1 : 0);
    return;
  }
  if (line == "PARAMS") { printParams(chan); return; }
  if (line == "ABORT") { abortReq = true; cprintf(chan, "#ACK ABORT\n"); return; }
  if (line == "DUMP") {
    if (state != ST_IDLE) cprintf(chan, "#ERR sesi berjalan\n");
    else if (!recD) cprintf(chan, "#ERR tidak ada data\n");
    else dumpSession(chan);
    return;
  }
  char key[32], val[32];
  if (sscanf(line.c_str(), "SET %31s %31s", key, val) == 2) {
    if (state != ST_IDLE) { cprintf(chan, "#ERR sesi berjalan\n"); return; }
    for (size_t i = 0; i < NKV; i++) if (strcmp(key, kvs[i].k) == 0) {
      if (kvs[i].t == 'f') *(float*)kvs[i].p = atof(val);
      else if (kvs[i].t == 'u') *(uint32_t*)kvs[i].p = strtoul(val, nullptr, 10);
      else *(uint8_t*)kvs[i].p = (uint8_t)strtoul(val, nullptr, 10);
      cprintf(chan, "#OK %s\n", key); return;
    }
    cprintf(chan, "#ERR kunci tidak dikenal\n"); return;
  }
  unsigned mode, dur, sid;
  if (sscanf(line.c_str(), "START %u %u %u", &mode, &dur, &sid) == 3) {
    if (state != ST_IDLE) { cprintf(chan, "#ERR sesi berjalan\n"); return; }
    if (mode > 2 || dur < 5 || dur > 120 || sid > 255) { cprintf(chan, "#ERR argumen\n"); return; }
    sessChan = chan;
    if (startSession((uint8_t)mode, dur, (uint8_t)sid)) cprintf(chan, "#ACK START mode=%u dur=%u sid=%u\n", mode, dur, sid);
    else cprintf(chan, "#ERR nomem\n");
    return;
  }
  cprintf(chan, "#ERR perintah tidak dikenal\n");
}
static void handleCmd(String line, uint8_t chan) {
  xSemaphoreTake(cmdMutex, portMAX_DELAY);
  handleCmdInner(line, chan);
  xSemaphoreGive(cmdMutex);
}

// Konsol TCP: menerima satu klien (laptop). Koneksi baru menggantikan koneksi lama.
static void tcpTask(void*) {
  int ls = socket(AF_INET, SOCK_STREAM, IPPROTO_IP);
  struct sockaddr_in a; memset(&a, 0, sizeof(a));
  a.sin_family = AF_INET; a.sin_port = htons(TCP_PORT); a.sin_addr.s_addr = htonl(INADDR_ANY);
  int one = 1; setsockopt(ls, SOL_SOCKET, SO_REUSEADDR, &one, sizeof(one));
  bind(ls, (struct sockaddr*)&a, sizeof(a));
  listen(ls, 1);
  int c = -1; char lb[96]; int ll = 0;
  for (;;) {
    fd_set rd; FD_ZERO(&rd); FD_SET(ls, &rd); int mx = ls;
    if (c >= 0) { FD_SET(c, &rd); if (c > mx) mx = c; }
    struct timeval tv; tv.tv_sec = 1; tv.tv_usec = 0;
    int r = select(mx + 1, &rd, NULL, NULL, &tv);
    if (r <= 0) continue;
    if (FD_ISSET(ls, &rd)) {
      struct sockaddr_in from; socklen_t fl = sizeof(from);
      int nc = accept(ls, (struct sockaddr*)&from, &fl);
      if (nc >= 0) {
        if (c >= 0) close(c);
        c = nc; tcpClient = c; ll = 0;
        int nd = 1; setsockopt(c, IPPROTO_TCP, TCP_NODELAY, &nd, sizeof(nd));
      }
    }
    if (c >= 0 && FD_ISSET(c, &rd)) {
      char b[64]; int n = recv(c, b, sizeof(b), 0);
      if (n <= 0) { tcpClient = -1; close(c); c = -1; ll = 0; continue; }
      for (int i = 0; i < n; i++) {
        char ch = b[i];
        if (ch == '\n' || ch == '\r') { if (ll) { lb[ll] = 0; ll = 0; handleCmd(String(lb), CH_TCP); } }
        else if (ll < (int)sizeof(lb) - 1) lb[ll++] = ch;
      }
    }
  }
}

// ---------------- setup / loop ----------------
void setup() {
  Serial.begin(115200);
  delay(300);
  initBNO055();
  echoQ = xQueueCreate(128, sizeof(EchoEv));
  cmdMutex = xSemaphoreCreateMutex();

  WiFi.mode(WIFI_STA);
  WiFi.setSleep(true);                 // matikan hemat daya Wi-Fi untuk latency rendah (bila didukung coexistence)
  WiFi.setAutoReconnect(true);
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  for (int i = 0; i < 60 && WiFi.status() != WL_CONNECTED; i++) delay(200);   // tunggu maks 12 detik
  udpSock = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
  struct sockaddr_in a; memset(&a, 0, sizeof(a));
  a.sin_family = AF_INET; a.sin_port = htons(UDP_PORT); a.sin_addr.s_addr = htonl(INADDR_ANY);
  bind(udpSock, (struct sockaddr*)&a, sizeof(a));

  BLEDevice::init(BLE_NAME);
  BLEDevice::setMTU(128);
  esp_ble_gap_register_callback(gapCallback);
  BLEServer* srv = BLEDevice::createServer();
  srv->setCallbacks(new SrvCb());
  BLEService* svc = srv->createService(BLE_SERVICE_UUID);
  pTx = svc->createCharacteristic(BLE_CHAR_TX_UUID, BLECharacteristic::PROPERTY_NOTIFY | BLECharacteristic::PROPERTY_READ);
  pTx->addDescriptor(new BLE2902());
  BLECharacteristic* pRx = svc->createCharacteristic(BLE_CHAR_RX_UUID, BLECharacteristic::PROPERTY_WRITE | BLECharacteristic::PROPERTY_WRITE_NR);
  pRx->setCallbacks(new RxCb());
  svc->start();
  BLEAdvertising* adv = BLEDevice::getAdvertising();
  adv->addServiceUUID(BLE_SERVICE_UUID);
  adv->setScanResponse(true);
  BLEDevice::startAdvertising();

  xTaskCreate(sensorTask, "imu", 4096, nullptr, 3, nullptr);
  xTaskCreate(udpTask,    "udp", 4096, nullptr, 6, nullptr);
  xTaskCreate(tcpTask,    "tcp", 8192, nullptr, 4, nullptr);
  xTaskCreate(txTask,     "tx",  8192, nullptr, 5, nullptr);
  Serial.printf("#READY fw=%s bno=%d wifi=%d ip=%s\n", FW_VERSION, bnoOk ? 1 : 0, WiFi.status() == WL_CONNECTED ? 1 : 0,
                WiFi.status() == WL_CONNECTED ? WiFi.localIP().toString().c_str() : "-");
}

void loop() {
  static String line;
  while (Serial.available()) {
    char c = (char)Serial.read();
    if (c == '\n' || c == '\r') { if (line.length()) { handleCmd(line, CH_SERIAL); line = ""; } }
    else if (line.length() < 80) line += c;
  }
  static uint32_t lastChk = 0;
  if (state == ST_IDLE && millis() - lastChk > 5000) {          // sambung ulang Wi-Fi hanya saat tidak ada sesi
    lastChk = millis();
    if (WiFi.status() != WL_CONNECTED) WiFi.reconnect();
  }
  delay(2);
}
