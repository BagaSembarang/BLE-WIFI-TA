/* ============================================================================
 * SKRIPSI SISTEM INFORMASI
 * KOMBINASI 3: WI-FI STATIS (UDP ROUTER) + NO SENSOR (BASELINE KONTROL)
 * 
 * Pengujian transmisi data gerak menggunakan Wi-Fi UDP melalui router tanpa beban sensor.
 * Durasi sesi: 1 MENIT (60 detik), timestamp JAM NYATA via NTP (HH:MM:SS.mmm WIB).
 * ============================================================================
 */

#include <Arduino.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include <time.h>          // Untuk NTP / RTC real-time

// ============================================================================
// PILIH MIKROKONTROLER (Ganti di sini: 1 untuk C3, 2 untuk S3)
// ============================================================================
#define MCU_ESP32_C3 1
#define MCU_ESP32_S3 2

#define TARGET_MCU MCU_ESP32_S3  // <-- Dipilih ESP32-C3

#if (TARGET_MCU == MCU_ESP32_C3)
  #define MCU_NAME "ESP32-C3"
#else
  #define MCU_NAME "ESP32-S3"
#endif

// ============================================================================
// KONFIGURASI ROUTER WI-FI & UDP
// ============================================================================
const char* WIFI_SSID     = "SIEGA";      // Sesuaikan SSID Router
const char* WIFI_PASSWORD = "Siega@2025";  // Sesuaikan Password Router

#define UDP_LOCAL_PORT        4210   // Port lokal ESP32
#define UDP_REMOTE_PORT       4211   // Port aplikasi laptop/penerima
#define SESSION_DURATION_MS   60000UL  // Durasi sesi: 1 menit
#define SAMPLE_RATE_HZ        50
#define TIMEOUT_RTT_MS        100

// NTP
#define NTP_SERVER1          "pool.ntp.org"
#define NTP_SERVER2          "time.google.com"
#define TZ_OFFSET_WIB        25200   // UTC+7 (WIB) dalam detik

// ============================================================================
// STRUKTUR PAYLOAD TETAP (52 BYTES IDENTIK)
// ============================================================================
#pragma pack(push, 1)
struct MotionPacket {
  uint32_t sequenceNumber;
  uint32_t sendTimestampMs;
  float quatW;
  float quatX;
  float quatY;
  float quatZ;
  float accelX;
  float accelY;
  float accelZ;
  float gyroX;
  float gyroY;
  float gyroZ;
  uint16_t buttonState;
  uint8_t sensorId;    // 0 = NO_SENSOR
  uint8_t checksum;
};
#pragma pack(pop)

WiFiUDP   udp;
IPAddress receiverIP(255, 255, 255, 255); // Broadcast sampai di-echo oleh penerima
uint16_t  receiverPort = UDP_REMOTE_PORT;
bool      receiverLocked = false;

uint32_t  sampleCount = 0;
uint32_t  seqNumber = 1;
bool      sessionActive = true;
uint32_t  sessionStartMs = 0;
time_t    ntpSyncEpoch = 0;
uint32_t  ntpSyncMillis = 0;
bool      ntpSynced = false;
float     lastValidLatency = 0.0f;
bool      hasPrevLatency = false;

struct SentRecord {
  uint32_t seq;
  uint32_t sentMs;
  int16_t  rssi;
  bool     pending;
};
SentRecord pendingList[16];

uint8_t calcChecksum(const MotionPacket& pkt) {
  const uint8_t* p = (const uint8_t*)&pkt;
  uint8_t cs = 0;
  for (size_t i = 0; i < sizeof(MotionPacket) - 1; i++) cs ^= p[i];
  return cs;
}

// Format -> "HH:MM:SS.mmm" JAM NYATA WIB via NTP
void formatTimestamp(uint32_t ms, char* buf, size_t bufLen) {
  if (ntpSynced && ntpSyncEpoch > 0) {
    uint32_t driftMs = (ms >= ntpSyncMillis) ? (ms - ntpSyncMillis) : 0;
    time_t realEpoch = ntpSyncEpoch + (driftMs / 1000);
    uint32_t milliPart = driftMs % 1000;
    struct tm* tmInfo = localtime(&realEpoch);
    snprintf(buf, bufLen, "%02d:%02d:%02d.%03u",
             tmInfo->tm_hour, tmInfo->tm_min, tmInfo->tm_sec, milliPart);
  } else {
    uint32_t totalSec = ms / 1000;
    snprintf(buf, bufLen, "%02u:%02u:%02u.%03u",
             totalSec / 3600, (totalSec % 3600) / 60, totalSec % 60, (uint32_t)(ms % 1000));
  }
}

void setup() {
  Serial.begin(115200);
  delay(1000);

  Serial.println("\n=======================================================");
  Serial.printf("  ESP32 MOTION CONTROLLER - %s\n", MCU_NAME);
  Serial.println("  METODE : WI-FI STATIS (UDP ROUTER)");
  Serial.println("  SENSOR : NO_SENSOR (BASELINE KONTROL)");
  Serial.println("  DURASI : 1 MENIT (60 DETIK)");
  Serial.println("  TIMESTAMP: JAM NYATA WIB via NTP (HH:MM:SS.mmm)");
  Serial.println("=======================================================");

  for (int i = 0; i < 16; i++) pendingList[i].pending = false;

  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

  Serial.printf("[Wi-Fi] Menghubungkan ke Router '%s'", WIFI_SSID);
  while (WiFi.status() != WL_CONNECTED) {
    delay(300);
    Serial.print(".");
  }
  Serial.println("\n[Wi-Fi] Berhasil terhubung ke Router!");
  Serial.printf("[Wi-Fi] IP ESP32: %s | RSSI: %d dBm\n", WiFi.localIP().toString().c_str(), WiFi.RSSI());

  // NTP Sync
  Serial.print("[NTP] Sinkronisasi jam...");
  configTime(TZ_OFFSET_WIB, 0, NTP_SERVER1, NTP_SERVER2);
  struct tm tmCheck;
  uint32_t ntpWait = millis();
  while (!getLocalTime(&tmCheck) && millis() - ntpWait < 10000) {
    delay(200); Serial.print(".");
  }
  Serial.println();
  if (getLocalTime(&tmCheck)) {
    ntpSyncEpoch = mktime(&tmCheck);
    ntpSyncMillis = millis();
    ntpSynced = true;
    Serial.printf("[NTP] Berhasil! Jam: %02d:%02d:%02d WIB\n",
                  tmCheck.tm_hour, tmCheck.tm_min, tmCheck.tm_sec);
  } else {
    Serial.println("[NTP] GAGAL! Timestamp fallback (elapsed).");
  }

  sessionStartMs = millis();
  udp.begin(UDP_LOCAL_PORT);
  Serial.printf("[UDP] Socket siap pada port: %d\n", UDP_LOCAL_PORT);
  Serial.println("CSV_HEADER:SequenceNumber,Timestamp,ElapsedMs,MCU,Sensor,Method,ActiveLink,RTT_ms,Latency_ms,Jitter_ms,PacketLoss,RSSI_dBm");
}

void loop() {
  uint32_t nowMs = millis();

  // 1. Baca Paket Echo UDP dari Penerima
  int packetSize = udp.parsePacket();
  if (packetSize >= 4) {
    uint8_t rxBuf[64];
    udp.read(rxBuf, sizeof(rxBuf));

    // Cek apakah sinyal START dari aplikasi laptop
    if (packetSize >= 5 && memcmp(rxBuf, "START", 5) == 0) {
      receiverIP = udp.remoteIP();
      receiverPort = UDP_REMOTE_PORT;
      receiverLocked = true;
      seqNumber = 1;
      sampleCount = 0;
      sessionStartMs = millis();
      sessionActive = true;
      for (int i = 0; i < 16; i++) pendingList[i].pending = false;
      Serial.println("[UDP] Menerima sinyal START! Sesi baru dimulai dari paket #1.");
      return;
    }

    // Kunci IP dan port penerima otomatis dari pengirim echo
    receiverIP = udp.remoteIP();
    receiverPort = udp.remotePort();
    receiverLocked = true;

    uint32_t echoSeq = 0;
    memcpy(&echoSeq, rxBuf, sizeof(uint32_t));

    for (int i = 0; i < 16; i++) {
      if (pendingList[i].pending && pendingList[i].seq == echoSeq) {
        pendingList[i].pending = false;

        float rtt = (float)(nowMs - pendingList[i].sentMs);
        if (rtt < 0) rtt = 0;
        float latency = rtt / 2.0f;
        float jitter = hasPrevLatency ? fabsf(latency - lastValidLatency) : 0.0f;
        lastValidLatency = latency;
        hasPrevLatency = true;

        sampleCount++;
        char tsStr[16];
        formatTimestamp(pendingList[i].sentMs, tsStr, sizeof(tsStr));
        Serial.printf("%u,%s,%u,%s,NO_SENSOR,WIFI_STATIC,WIFI,%.2f,%.2f,%.2f,0,%d\n",
                      echoSeq, tsStr, pendingList[i].sentMs - sessionStartMs,
                      MCU_NAME, rtt, latency, jitter, pendingList[i].rssi);
        if (millis() - sessionStartMs >= SESSION_DURATION_MS) {
          sessionActive = false;
          Serial.printf("\n[SELESAI] Sesi 1 menit selesai! Total: %u paket\n", sampleCount);
        }
        break;
      }
    }
  }

  // 2. Cek Timeout (Packet Loss)
  for (int i = 0; i < 16; i++) {
    if (pendingList[i].pending && (nowMs - pendingList[i].sentMs > TIMEOUT_RTT_MS)) {
      pendingList[i].pending = false;
      sampleCount++;
      char tsStr[16];
      formatTimestamp(pendingList[i].sentMs, tsStr, sizeof(tsStr));
      Serial.printf("%u,%s,%u,%s,NO_SENSOR,WIFI_STATIC,WIFI,0.00,0.00,0.00,1,%d\n",
                    pendingList[i].seq, tsStr, pendingList[i].sentMs - sessionStartMs,
                    MCU_NAME, pendingList[i].rssi);
      if (millis() - sessionStartMs >= SESSION_DURATION_MS) {
        sessionActive = false;
        Serial.printf("\n[SELESAI] Sesi 1 menit selesai! Total: %u paket\n", sampleCount);
      }
    }
  }

  // 3. Pengiriman Paket 50 Hz (20 ms)
  static uint32_t lastTxMs = 0;
  const uint32_t intervalMs = 1000 / SAMPLE_RATE_HZ;

  if (sessionActive && (WiFi.status() == WL_CONNECTED) && (nowMs - lastTxMs >= intervalMs)) {
    lastTxMs = nowMs;

    MotionPacket pkt;
    pkt.sequenceNumber  = seqNumber++;
    pkt.sendTimestampMs = nowMs;
    pkt.quatW  = 1.0f; pkt.quatX = 0.0f; pkt.quatY = 0.0f; pkt.quatZ = 0.0f;
    pkt.accelX = 0.0f; pkt.accelY = 0.0f; pkt.accelZ = 9.81f;
    pkt.gyroX  = 0.0f; pkt.gyroY  = 0.0f; pkt.gyroZ  = 0.0f;
    pkt.buttonState = 0;
    pkt.sensorId = 0; // NO_SENSOR
    pkt.checksum = calcChecksum(pkt);

    int16_t rssi = WiFi.RSSI();

    udp.beginPacket(receiverIP, receiverPort);
    udp.write((const uint8_t*)&pkt, sizeof(MotionPacket));
    udp.endPacket();

    for (int i = 0; i < 16; i++) {
      if (!pendingList[i].pending) {
        pendingList[i] = { pkt.sequenceNumber, pkt.sendTimestampMs, rssi, true };
        break;
      }
    }
  }
}
