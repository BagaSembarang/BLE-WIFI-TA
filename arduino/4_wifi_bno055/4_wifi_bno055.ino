/* ============================================================================
 * SKRIPSI SISTEM INFORMASI
 * KOMBINASI 4: WI-FI STATIS (UDP ROUTER) + BNO055 (9-AXIS SENSOR FUSION)
 * 
 * Pengujian transmisi data gerak menggunakan Wi-Fi UDP melalui router dengan sensor BNO055.
 * Durasi sesi: 1 MENIT (60 detik), timestamp JAM NYATA via NTP (HH:MM:SS.mmm WIB).
 * ============================================================================
 */

#include <Arduino.h>
#include <Wire.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include <time.h>          // Untuk NTP / RTC real-time

// ============================================================================
// PILIH MIKROKONTROLER (Ganti di sini: 1 untuk C3, 2 untuk S3)
// ============================================================================
#define MCU_ESP32_C3 1
#define MCU_ESP32_S3 2

#define TARGET_MCU MCU_ESP32_C3  // <-- Dipilih ESP32-C3 (SDA=2, SCL=3)

#if (TARGET_MCU == MCU_ESP32_C3)
  #define MCU_NAME "ESP32-C3"
  #define I2C_SDA  2
  #define I2C_SCL  3
#else
  #define MCU_NAME "ESP32-S3"
  #define I2C_SDA  8
  #define I2C_SCL  9
#endif

// ============================================================================
// KONFIGURASI ROUTER WI-FI & UDP
// ============================================================================
const char* WIFI_SSID     = "Cardinal";      // Sesuaikan SSID Router
const char* WIFI_PASSWORD = "12082231";  // Sesuaikan Password Router

#define UDP_LOCAL_PORT        4210
#define UDP_REMOTE_PORT       4211
#define SESSION_DURATION_MS   60000UL  // Durasi sesi: 1 menit
#define SAMPLE_RATE_HZ        50
#define TIMEOUT_RTT_MS        100

// NTP
#define NTP_SERVER1          "pool.ntp.org"
#define NTP_SERVER2          "time.google.com"
#define TZ_OFFSET_WIB        25200   // UTC+7 (WIB) dalam detik

// Register BNO055
uint8_t bnoAddr               = 0x28; // Otomatis disesuaikan ke 0x28 atau 0x29 saat scan
#define BNO055_PAGE_ID        0x07
#define BNO055_OPR_MODE       0x3D
#define BNO055_PWR_MODE       0x3E
#define BNO055_QUAT_DATA      0x20
#define BNO055_LIA_DATA       0x28
#define BNO055_GYRO_DATA      0x14

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
  uint8_t sensorId;    // 1 = BNO055
  uint8_t checksum;
};
#pragma pack(pop)

WiFiUDP   udp;
IPAddress receiverIP(255, 255, 255, 255);
uint16_t  receiverPort = UDP_REMOTE_PORT;
bool      receiverLocked = false;
bool      bnoDetected = false;

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

bool bnoWrite(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(bnoAddr);
  Wire.write(reg);
  Wire.write(val);
  return (Wire.endTransmission() == 0);
}

bool bnoRead(uint8_t reg, uint8_t* buf, uint8_t len) {
  Wire.beginTransmission(bnoAddr);
  Wire.write(reg);
  uint8_t err = Wire.endTransmission(false);
  if (err != 0) {
    Wire.beginTransmission(bnoAddr);
    Wire.write(reg);
    if (Wire.endTransmission(true) != 0) return false;
  }
  uint8_t readCount = Wire.requestFrom((int)bnoAddr, (int)len);
  if (readCount != len) return false;
  for (int i = 0; i < len; i++) {
    buf[i] = Wire.read();
  }
  return true;
}

void initBNO055() {
  Wire.begin(I2C_SDA, I2C_SCL, 100000); // 100 kHz I2C clock (stabil untuk BNO055)
  Wire.setTimeOut(1000);                // Anti-hang I2C timeout 1000 ms
  delay(500);                           // Waktu booting POR internal BNO055 Cortex-M0

  Serial.println("\n-------------------------------------------------------");
  Serial.printf("[BNO055] Memindai bus I2C (SDA: GPIO %d, SCL: GPIO %d)...\n", I2C_SDA, I2C_SCL);

  uint8_t id = 0;
  bnoDetected = false;
  uint8_t foundAddr = 0;

  // 1. Scan semua alamat I2C di bus (1 s/d 126)
  for (uint8_t a = 1; a < 127; a++) {
    Wire.beginTransmission(a);
    if (Wire.endTransmission() == 0) {
      Serial.printf("[I2C SCAN] DITEMUKAN perangkat I2C pada alamat 0x%02X!\n", a);
      if (a == 0x28 || a == 0x29) foundAddr = a;
    }
  }

  uint8_t testAddrs[2] = { (foundAddr != 0 ? foundAddr : 0x28), (foundAddr == 0x29 ? 0x28 : 0x29) };

  for (int aIdx = 0; aIdx < 2 && !bnoDetected; aIdx++) {
    bnoAddr = testAddrs[aIdx];
    for (int attempt = 1; attempt <= 5; attempt++) {
      if (bnoRead(0x00, &id, 1) && id == 0xA0) {
        bnoDetected = true;
        break;
      }
      delay(50);
    }
  }

  if (bnoDetected) {
    bnoWrite(BNO055_OPR_MODE, 0x00); // Config mode
    delay(30);
    bnoWrite(BNO055_PAGE_ID, 0x00);
    bnoWrite(BNO055_PWR_MODE, 0x00); // Normal power
    bnoWrite(BNO055_OPR_MODE, 0x0C); // NDOF Fusion
    delay(50);
    Serial.printf("[BNO055] SUKSES! Terdeteksi pada alamat 0x%02X (Chip ID: 0xA0). Mode NDOF 9-Axis Aktif!\n", bnoAddr);
  } else {
    Serial.printf("[BNO055] PERINGATAN: Sensor tidak merespons (ID terbaca: 0x%02X). Memakai fallback.\n", id);
    Serial.println("[BNO055] PETUNJUK: Jika [I2C SCAN] tidak menemukan apa-apa:");
    Serial.println("  1. Pastikan pin PS0 dan PS1 di BNO055 tersambung ke GND (Wajib I2C)!");
    Serial.println("  2. Pastikan RST dihubungkan ke 3.3V.");
    Serial.printf("  3. Pastikan SDA -> GPIO %d dan SCL -> GPIO %d.\n", I2C_SDA, I2C_SCL);
  }
}

void readBNO055(MotionPacket& pkt) {
  pkt.sensorId = 1;
  if (!bnoDetected) {
    // Auto-reconnect berkala jika sensor sempat terlepas atau terlambat menyala
    static uint32_t lastRetryMs = 0;
    if (millis() - lastRetryMs > 3000) {
      lastRetryMs = millis();
      initBNO055();
    }
    pkt.quatW = 1.0f; pkt.quatX = 0.0f; pkt.quatY = 0.0f; pkt.quatZ = 0.0f;
    pkt.accelX = 0.0f; pkt.accelY = 0.0f; pkt.accelZ = 9.81f;
    pkt.gyroX = 0.0f; pkt.gyroY = 0.0f; pkt.gyroZ = 0.0f;
    return;
  }
  uint8_t qb[8];
  if (bnoRead(BNO055_QUAT_DATA, qb, 8)) {
    int16_t w = (int16_t)(((uint16_t)qb[1] << 8) | qb[0]);
    int16_t x = (int16_t)(((uint16_t)qb[3] << 8) | qb[2]);
    int16_t y = (int16_t)(((uint16_t)qb[5] << 8) | qb[4]);
    int16_t z = (int16_t)(((uint16_t)qb[7] << 8) | qb[6]);
    const float scale = 1.0f / 16384.0f;
    pkt.quatW = w * scale; pkt.quatX = x * scale; pkt.quatY = y * scale; pkt.quatZ = z * scale;
  }
  uint8_t ab[6];
  if (bnoRead(BNO055_LIA_DATA, ab, 6)) {
    int16_t ax = (int16_t)(((uint16_t)ab[1] << 8) | ab[0]);
    int16_t ay = (int16_t)(((uint16_t)ab[3] << 8) | ab[2]);
    int16_t az = (int16_t)(((uint16_t)ab[5] << 8) | ab[4]);
    pkt.accelX = ax / 100.0f; pkt.accelY = ay / 100.0f; pkt.accelZ = az / 100.0f;
  }
  uint8_t gb[6];
  if (bnoRead(BNO055_GYRO_DATA, gb, 6)) {
    int16_t gx = (int16_t)(((uint16_t)gb[1] << 8) | gb[0]);
    int16_t gy = (int16_t)(((uint16_t)gb[3] << 8) | gb[2]);
    int16_t gz = (int16_t)(((uint16_t)gb[5] << 8) | gb[4]);
    pkt.gyroX = gx / 16.0f; pkt.gyroY = gy / 16.0f; pkt.gyroZ = gz / 16.0f;
  }
}

void setup() {
  Serial.begin(115200);
  uint32_t sStart = millis();
  while (!Serial && millis() - sStart < 2500);
  delay(500);

  Serial.println("\n=======================================================");
  Serial.printf("  ESP32 MOTION CONTROLLER - %s\n", MCU_NAME);
  Serial.println("  METODE : WI-FI STATIS (UDP ROUTER)");
  Serial.println("  SENSOR : BNO055 (9-AXIS SENSOR FUSION)");
  Serial.println("  DURASI : 1 MENIT (60 DETIK)");
  Serial.println("  TIMESTAMP: JAM NYATA WIB via NTP (HH:MM:SS.mmm)");
  Serial.println("=======================================================");

  for (int i = 0; i < 16; i++) pendingList[i].pending = false;

  initBNO055();

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
        Serial.printf("%u,%s,%u,%s,BNO055,WIFI_STATIC,WIFI,%.2f,%.2f,%.2f,0,%d\n",
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
      Serial.printf("%u,%s,%u,%s,BNO055,WIFI_STATIC,WIFI,0.00,0.00,0.00,1,%d\n",
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
    readBNO055(pkt);
    pkt.sequenceNumber  = seqNumber++;
    pkt.sendTimestampMs = nowMs;
    pkt.buttonState     = 0;
    pkt.checksum        = calcChecksum(pkt);

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
