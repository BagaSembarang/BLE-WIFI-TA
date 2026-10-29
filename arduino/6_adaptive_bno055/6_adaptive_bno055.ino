/* ============================================================================
 * SKRIPSI SISTEM INFORMASI
 * KOMBINASI 6: ADAPTIVE SWITCHING + BNO055 (9-AXIS SENSOR FUSION)
 * 
 * Pengujian transmisi data gerak dengan Adaptive Switching (BLE <-> Wi-Fi)
 * menggunakan sensor fisik BNO055 via I2C dengan Hysteresis & Dwell Time.
 * Durasi sesi: 1 MENIT (60 detik), timestamp JAM NYATA via NTP (HH:MM:SS.mmm WIB).
 * ============================================================================
 */

#include <Arduino.h>
#include <Wire.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include <time.h>          // Untuk NTP / RTC real-time
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include <esp_gap_ble_api.h>

// NTP
#define NTP_SERVER1       "pool.ntp.org"
#define NTP_SERVER2       "time.google.com"
#define TZ_OFFSET_WIB     25200   // UTC+7 (WIB) dalam detik

// ============================================================================
// PILIH MIKROKONTROLER (Ganti di sini: 1 untuk C3, 2 untuk S3)
// ============================================================================
#define MCU_ESP32_C3 1
#define MCU_ESP32_S3 2

#define TARGET_MCU MCU_ESP32_S3

#if (TARGET_MCU == MCU_ESP32_C3)
  #define MCU_NAME "ESP32-C3"
  #define I2C_SDA  2
  #define I2C_SCL  3
#else
  #define MCU_NAME "ESP32-S3"
  #define I2C_SDA  8
  #define I2C_SCL  9
#endif

// Router Wi-Fi & UDP
const char* WIFI_SSID     = "Cardinal";
const char* WIFI_PASSWORD = "12082231";
#define UDP_LOCAL_PORT    4210
#define UDP_REMOTE_PORT   4211

// BLE UUIDs
#define BLE_DEVICE_NAME   "ESP32-Motion-Ctrl"
#define BLE_SERVICE_UUID  "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
#define BLE_CHAR_TX_UUID  "beb5483e-36e1-4688-b7f5-ea07361b26a8"
#define BLE_CHAR_RX_UUID  "d29759bc-4b6f-45a6-8e50-96f7e44fa70c"

#define SESSION_DURATION_MS   60000UL  // Durasi sesi: 1 menit
#define SAMPLE_RATE_HZ        50
#define TIMEOUT_RTT_MS        100

// Parameter Adaptive Switching
#define DWELL_TIME_PACKETS    30      // Minimal 30 paket bertahan di link baru (anti-flapping)
#define ADAPTIVE_WINDOW       15      // Window 15 paket terakhir

#define LOSS_THRESH_BLE_WIFI  15.0f   // BLE -> WiFi jika loss > 15%
#define LAT_THRESH_BLE_WIFI   45.0f   // atau Latency > 45 ms
#define LOSS_THRESH_WIFI_BLE  20.0f   // WiFi -> BLE jika loss > 20% (Hysteresis)
#define LAT_THRESH_WIFI_BLE   55.0f   // atau Latency > 55 ms

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

enum LinkState { LINK_BLE = 0, LINK_WIFI = 1 };
LinkState currentLink = LINK_BLE;

WiFiUDP            udp;
IPAddress          receiverIP(255, 255, 255, 255);
uint16_t           receiverPort = UDP_REMOTE_PORT;
bool               receiverLocked = false;
bool               bnoDetected = false;

BLEServer*         pServer = nullptr;
BLECharacteristic* pTxChar = nullptr;
BLECharacteristic* pRxChar = nullptr;
bool               bleConnected = false;
int16_t            bleRssi = -65;
esp_bd_addr_t      remoteBda;
bool               hasRemoteBda = false;

volatile uint32_t  lastBleEchoSeq = 0;
volatile uint32_t  lastBleEchoTime = 0;
volatile bool      newBleEcho = false;

uint32_t           sampleCount = 0;
uint32_t           seqNumber = 1;
bool               sessionActive = true;
bool               streamingStarted = false; // Tunggu sinyal START dari laptop
uint32_t           connectMs = 0;
uint32_t           sessionStartMs = 0;   // millis() saat sesi mulai
time_t             ntpSyncEpoch = 0;     // Unix epoch saat NTP sync
uint32_t           ntpSyncMillis = 0;    // millis() saat NTP sync
bool               ntpSynced = false;    // Flag NTP berhasil
float              lastValidLatency = 0.0f;
bool               hasPrevLatency = false;

uint32_t           packetsSinceSwitch = 0;
uint32_t           totalSwitches = 0;

struct WindowRecord {
  bool      isLost;
  float     latency;
  float     jitter;
  LinkState link;
};
WindowRecord winBuf[ADAPTIVE_WINDOW];
size_t winIdx = 0;
size_t winCount = 0;
uint8_t consecutiveLoss = 0;

struct SentRecord {
  uint32_t  seq;
  uint32_t  sentMs;
  LinkState link;
  int16_t   rssi;
  bool      pending;
};
SentRecord pendingList[16];

uint8_t calcChecksum(const MotionPacket& pkt) {
  const uint8_t* p = (const uint8_t*)&pkt;
  uint8_t cs = 0;
  for (size_t i = 0; i < sizeof(MotionPacket) - 1; i++) cs ^= p[i];
  return cs;
}

// Format -> "HH:MM:SS.mmm" JAM NYATA WIB via NTP
// Jika NTP belum sync, fallback ke format elapsed sejak boot
void formatTimestamp(uint32_t ms, char* buf, size_t bufLen) {
  if (ntpSynced && ntpSyncEpoch > 0) {
    // Hitung jam nyata: epoch NTP + selisih millis sejak sync
    uint32_t driftMs = (ms >= ntpSyncMillis) ? (ms - ntpSyncMillis) : 0;
    time_t realEpoch = ntpSyncEpoch + (driftMs / 1000);
    uint32_t milliPart = driftMs % 1000;
    struct tm* tmInfo = localtime(&realEpoch);
    snprintf(buf, bufLen, "%02d:%02d:%02d.%03u",
             tmInfo->tm_hour, tmInfo->tm_min, tmInfo->tm_sec, milliPart);
  } else {
    // Fallback: elapsed sejak boot
    uint32_t elapsed = ms;
    uint32_t totalSec = elapsed / 1000;
    uint32_t milliPart = elapsed % 1000;
    snprintf(buf, bufLen, "%02u:%02u:%02u.%03u",
             totalSec / 3600, (totalSec % 3600) / 60, totalSec % 60, milliPart);
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

static void gapCallback(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param) {
  if (event == ESP_GAP_BLE_READ_RSSI_COMPLETE_EVT && param->read_rssi_cmpl.status == ESP_BT_STATUS_SUCCESS) {
    bleRssi = param->read_rssi_cmpl.rssi;
  }
}

class MyServerCallbacks: public BLEServerCallbacks {
  void onConnect(BLEServer* pServer, esp_ble_gatts_cb_param_t *param) {
    bleConnected = true;
    streamingStarted = false; // Tunggu sinyal START dari laptop
    sessionActive = true;
    sampleCount = 0;
    seqNumber = 1;            // Reset nomor sequence ke 1
    connectMs = millis();
    sessionStartMs = millis();
    memcpy(remoteBda, param->connect.remote_bda, sizeof(esp_bd_addr_t));
    hasRemoteBda = true;
    Serial.println("[BLE] Terhubung! Menunggu sinyal START dari laptop...");
  }
  void onDisconnect(BLEServer* pServer) {
    bleConnected = false;
    streamingStarted = false;
    sessionActive = false;
    hasRemoteBda = false;
    bleRssi = -99;
    Serial.println("[BLE] Terputus. Advertising ulang...");
    delay(50);
    pServer->startAdvertising();
  }
};

class MyRxCallbacks: public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic* pChar) {
    size_t len = pChar->getLength();
    const uint8_t* data = pChar->getData();

    // Handshake sinyal START dari aplikasi laptop
    if (len >= 5 && memcmp(data, "START", 5) == 0) {
      seqNumber = 1;
      sampleCount = 0;
      sessionStartMs = millis();
      streamingStarted = true;
      sessionActive = true;
      for (int i = 0; i < 16; i++) pendingList[i].pending = false;
      Serial.println("[BLE] Sinyal START diterima! Streaming dimulai persis dari paket #1.");
      return;
    }

    if (len >= 4) {
      uint32_t seq = 0;
      memcpy(&seq, data, sizeof(uint32_t));
      lastBleEchoSeq = seq;
      lastBleEchoTime = millis();
      newBleEcho = true;
    }
  }
};

void recordWindow(bool isLost, float lat, float jit, LinkState link) {
  packetsSinceSwitch++;
  if (isLost) consecutiveLoss++; else consecutiveLoss = 0;
  winBuf[winIdx] = { isLost, lat, jit, link };
  winIdx = (winIdx + 1) % ADAPTIVE_WINDOW;
  if (winCount < ADAPTIVE_WINDOW) winCount++;
}

// Pelacak Waktu & Status Handover (Durasi Transisi Realtime)
bool      handoverPending = false;
uint32_t  handoverStartMs = 0;
char      handoverStartTimeStr[16] = "";
LinkState handoverFromLink = LINK_BLE;
LinkState handoverToLink = LINK_WIFI;
uint32_t  handoverTriggerSeq = 0;

void triggerHandover(LinkState toLink, const char* reason) {
  handoverFromLink = currentLink;
  handoverToLink = toLink;
  handoverStartMs = millis();
  formatTimestamp(handoverStartMs, handoverStartTimeStr, sizeof(handoverStartTimeStr));
  handoverPending = true;
  handoverTriggerSeq = seqNumber;

  currentLink = toLink;
  packetsSinceSwitch = 0;
  totalSwitches++;

  const char* fromStr = (handoverFromLink == LINK_BLE) ? "BLE" : "Wi-Fi UDP";
  const char* toStr   = (handoverToLink == LINK_BLE) ? "BLE" : "Wi-Fi UDP";

  Serial.println("\n------------------------------------------------------------");
  Serial.printf("[HANDOVER-START #%u] Keputusan Switch Diambil!\n", totalSwitches);
  Serial.printf("  * Transisi      : %s -> %s\n", fromStr, toStr);
  Serial.printf("  * Alasan Pemicu : %s\n", reason);
  Serial.printf("  * Mulai Pukul   : %s WIB (Realtime)\n", handoverStartTimeStr);
  Serial.printf("  * Pada Sequence : Paket #%u\n", handoverTriggerSeq);
  Serial.println("  * Status        : Mengalihkan pengiriman paket ke link target...");
  Serial.println("------------------------------------------------------------\n");
}

void evaluateAdaptiveSwitch() {
  bool wifiConnected = (WiFi.status() == WL_CONNECTED);
  int16_t wifiRssi = wifiConnected ? WiFi.RSSI() : -99;

  // 1. Kasus Fisik Terputus (Physical Disconnection)
  if (!bleConnected && currentLink == LINK_BLE) {
    if (wifiConnected && wifiRssi >= -90) {
      triggerHandover(LINK_WIFI, "BLE Terputus Fisik (Koneksi Bluetooth Hilang)");
    } else {
      static uint32_t lastWarnMs = 0;
      if (millis() - lastWarnMs > 2500) {
        lastWarnMs = millis();
        Serial.printf("[ADAPTIVE-BLOCKED] BLE Terputus fisik, tetapi Wi-Fi juga tidak tersedia / sinyal lemah (%d dBm)!\n", wifiRssi);
      }
    }
    return;
  }
  if (!wifiConnected && currentLink == LINK_WIFI) {
    if (bleConnected && bleRssi >= -88) {
      triggerHandover(LINK_BLE, "Wi-Fi Terputus Fisik (Status Disconnected)");
    } else {
      static uint32_t lastWarnMs = 0;
      if (millis() - lastWarnMs > 2500) {
        lastWarnMs = millis();
        Serial.printf("[ADAPTIVE-BLOCKED] Wi-Fi Terputus fisik, tetapi BLE juga tidak terhubung / sinyal lemah (%d dBm)!\n", bleRssi);
      }
    }
    return;
  }

  // 2. Emergency Override Dwell Time (5x berturut loss)
  if (consecutiveLoss >= 5) {
    if (currentLink == LINK_BLE) {
      // CEK KUALITAS WI-FI TARGET SEBELUM SWITCH
      if (wifiConnected && wifiRssi >= -88) {
        consecutiveLoss = 0;
        triggerHandover(LINK_WIFI, "Emergency Override (5x Loss Berturut-turut pada BLE)");
        return;
      } else {
        Serial.printf("[ADAPTIVE-HOLD] Emergency BLE 5x loss, tetapi kualitas target Wi-Fi tidak memadai (RSSI: %d dBm, Status: %d). Bertahan di BLE.\n", wifiRssi, WiFi.status());
      }
    } else if (currentLink == LINK_WIFI) {
      // CEK KUALITAS BLE TARGET SEBELUM SWITCH
      if (bleConnected && bleRssi >= -85) {
        consecutiveLoss = 0;
        triggerHandover(LINK_BLE, "Emergency Override (5x Loss Berturut-turut pada Wi-Fi)");
        return;
      } else {
        Serial.printf("[ADAPTIVE-HOLD] Emergency Wi-Fi 5x loss, tetapi kualitas target BLE tidak memadai (RSSI: %d dBm, Status: %d). Bertahan di Wi-Fi.\n", bleRssi, bleConnected);
      }
    }
  }

  // 3. Minimum Dwell Time Check (Anti-Flapping Guard)
  if (packetsSinceSwitch < DWELL_TIME_PACKETS || winCount < 8) return;

  // 4. Hitung Statistik QoS Window
  uint16_t sampleCnt = 0, lostCnt = 0, validCnt = 0;
  float sumLat = 0, sumJit = 0;
  for (size_t i = 0; i < winCount; i++) {
    if (winBuf[i].link == currentLink) {
      sampleCnt++;
      if (winBuf[i].isLost) lostCnt++;
      else { sumLat += winBuf[i].latency; sumJit += winBuf[i].jitter; validCnt++; }
    }
  }
  if (sampleCnt == 0) return;

  float lossPct = ((float)lostCnt / sampleCnt) * 100.0f;
  float avgLat = (validCnt > 0) ? (sumLat / validCnt) : 999.0f;

  // 5. Aturan Switch BLE -> Wi-Fi (Evaluasi Kelayakan Target Wi-Fi)
  if (currentLink == LINK_BLE) {
    bool bleDegraded = (lossPct >= LOSS_THRESH_BLE_WIFI || avgLat >= LAT_THRESH_BLE_WIFI || (bleRssi <= -85 && bleRssi != 0));
    if (bleDegraded) {
      // PENGECEKAN KUALITAS TARGET WI-FI SEBELUM PINDAH:
      if (!wifiConnected) {
        Serial.println("[ADAPTIVE-HOLD] BLE terdegradasi, tetapi Wi-Fi belum tersambung ke Router. Switch dibatalkan.");
        return;
      }
      if (wifiRssi < -85) {
        Serial.printf("[ADAPTIVE-HOLD] BLE terdegradasi, tetapi sinyal target Wi-Fi sangat lemah (%d dBm < -85 dBm). Memindahkan link akan memperburuk koneksi. Switch ditunda.\n", wifiRssi);
        return;
      }
      if (bleRssi != 0 && bleRssi != -99 && wifiRssi < (bleRssi - 3)) {
        Serial.printf("[ADAPTIVE-HOLD] BLE terdegradasi, tetapi kualitas target Wi-Fi (%d dBm) berada di bawah kualitas BLE (%d dBm). Switch ditahan.\n", wifiRssi, bleRssi);
        return;
      }

      // Wi-Fi memenuhi syarat dan lebih baik dari BLE yang terdegradasi
      char reasonBuf[100];
      snprintf(reasonBuf, sizeof(reasonBuf), "BLE Terdegradasi (Loss: %.1f%%, Lat: %.1fms, RSSI: %d dBm | Target Wi-Fi: %d dBm)",
               lossPct, avgLat, bleRssi, wifiRssi);
      triggerHandover(LINK_WIFI, reasonBuf);
    }
  }
  // 6. Aturan Switch Wi-Fi -> BLE (Hysteresis Guard & Evaluasi Kelayakan Target BLE)
  else if (currentLink == LINK_WIFI) {
    bool wifiDegraded = (lossPct >= LOSS_THRESH_WIFI_BLE || avgLat >= LAT_THRESH_WIFI_BLE || (wifiRssi <= -88 && wifiRssi != 0));
    if (wifiDegraded) {
      // PENGECEKAN KUALITAS TARGET BLE SEBELUM PINDAH:
      if (!bleConnected) {
        Serial.println("[ADAPTIVE-HOLD] Wi-Fi terdegradasi, tetapi BLE tidak terhubung ke Workstation. Switch dibatalkan.");
        return;
      }
      if (bleRssi <= -82 && bleRssi != 0) {
        Serial.printf("[ADAPTIVE-HOLD] Wi-Fi terdegradasi, tetapi sinyal target BLE juga lemah (%d dBm <= -82 dBm). Switch ditunda demi stabilitas.\n", bleRssi);
        return;
      }
      if (wifiRssi != 0 && wifiRssi != -99 && bleRssi < (wifiRssi + 3)) {
        Serial.printf("[ADAPTIVE-HOLD] Wi-Fi terdegradasi, tetapi sinyal target BLE (%d dBm) tidak lebih baik dari Wi-Fi (%d dBm + 3dB Hysteresis). Switch ditahan.\n", bleRssi, wifiRssi);
        return;
      }

      // BLE memenuhi syarat dan lebih baik dari Wi-Fi yang terdegradasi
      char reasonBuf[100];
      snprintf(reasonBuf, sizeof(reasonBuf), "Wi-Fi Terdegradasi (Loss: %.1f%%, Lat: %.1fms, RSSI: %d dBm | Target BLE: %d dBm)",
               lossPct, avgLat, wifiRssi, bleRssi);
      triggerHandover(LINK_BLE, reasonBuf);
    }
  }
}

void setup() {
  Serial.begin(115200);
  uint32_t sStart = millis();
  while (!Serial && millis() - sStart < 2500); // Tunggu USB CDC Serial Monitor siap (khusus ESP32-S3)
  delay(500);

  Serial.println("\n=======================================================");
  Serial.printf("  ESP32 MOTION CONTROLLER - %s\n", MCU_NAME);
  Serial.println("  METODE : ADAPTIVE SWITCHING (BLE <-> WI-FI)");
  Serial.println("  SENSOR : BNO055 (9-AXIS SENSOR FUSION)");
  Serial.println("  DURASI : 1 MENIT (60 DETIK)");
  Serial.println("  TIMESTAMP: JAM NYATA WIB via NTP (HH:MM:SS.mmm)");
  Serial.println("=======================================================");

  for (int i = 0; i < 16; i++) pendingList[i].pending = false;

  initBNO055();

  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  Serial.print("[Wi-Fi] Menyambung ke router...");
  uint32_t startW = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - startW < 8000) {
    delay(250);
    Serial.print(".");
  }
  Serial.println();
  if (WiFi.status() == WL_CONNECTED) {
    Serial.printf("[Wi-Fi] Terhubung! IP: %s\n", WiFi.localIP().toString().c_str());

    // ---------------------------------------------------------------
    // SYNC NTP: Ambil jam nyata dari internet (WIB = UTC+7)
    // ---------------------------------------------------------------
    Serial.print("[NTP] Sinkronisasi jam dari internet...");
    configTime(TZ_OFFSET_WIB, 0, NTP_SERVER1, NTP_SERVER2);

    // Tunggu NTP berhasil (maks 10 detik)
    struct tm tmCheck;
    uint32_t ntpWaitStart = millis();
    while (!getLocalTime(&tmCheck) && millis() - ntpWaitStart < 10000) {
      delay(200);
      Serial.print(".");
    }
    Serial.println();

    if (getLocalTime(&tmCheck)) {
      ntpSyncEpoch  = mktime(&tmCheck);  // Simpan epoch WIB
      ntpSyncMillis = millis();          // Simpan referensi millis
      ntpSynced     = true;
      Serial.printf("[NTP] Berhasil! Jam sekarang: %02d:%02d:%02d WIB\n",
                    tmCheck.tm_hour, tmCheck.tm_min, tmCheck.tm_sec);
    } else {
      Serial.println("[NTP] GAGAL sync! Timestamp akan pakai elapsed sejak boot.");
      ntpSynced = false;
    }
    // ---------------------------------------------------------------
  } else {
    Serial.println("[Wi-Fi] Belum tersambung. Timestamp fallback (elapsed).");
  }
  udp.begin(UDP_LOCAL_PORT);

  BLEDevice::init(BLE_DEVICE_NAME);
  BLEDevice::setMTU(128);
  esp_ble_gap_register_callback(gapCallback);

  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());

  BLEService* pService = pServer->createService(BLE_SERVICE_UUID);
  pTxChar = pService->createCharacteristic(BLE_CHAR_TX_UUID, BLECharacteristic::PROPERTY_NOTIFY | BLECharacteristic::PROPERTY_READ);
  pTxChar->addDescriptor(new BLE2902());
  pRxChar = pService->createCharacteristic(BLE_CHAR_RX_UUID, BLECharacteristic::PROPERTY_WRITE | BLECharacteristic::PROPERTY_WRITE_NR);
  pRxChar->setCallbacks(new MyRxCallbacks());
  pService->start();

  BLEAdvertising* pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(BLE_SERVICE_UUID);
  pAdvertising->setScanResponse(true);
  BLEDevice::startAdvertising();

  Serial.println("[BLE] Advertising aktif.");
  Serial.println("CSV_HEADER:SequenceNumber,Timestamp,ElapsedMs,MCU,Sensor,Method,ActiveLink,RTT_ms,Latency_ms,Jitter_ms,PacketLoss,RSSI_dBm");
  sessionStartMs = millis();  // Catat waktu boot sebagai referensi awal
}

void loop() {
  uint32_t nowMs = millis();

  static uint32_t lastRssiMs = 0;
  if (bleConnected && hasRemoteBda && (nowMs - lastRssiMs > 500)) {
    lastRssiMs = nowMs;
    esp_ble_gap_read_rssi(remoteBda);
  }

  // 1. Cek Echo BLE
  if (newBleEcho) {
    uint32_t eSeq = lastBleEchoSeq;
    uint32_t eTime = lastBleEchoTime;
    newBleEcho = false;

    for (int i = 0; i < 16; i++) {
      if (pendingList[i].pending && pendingList[i].seq == eSeq && pendingList[i].link == LINK_BLE) {
        pendingList[i].pending = false;
        float rtt = (float)(eTime - pendingList[i].sentMs);
        if (rtt < 0) rtt = 0;
        float lat = rtt / 2.0f;
        float jit = hasPrevLatency ? fabsf(lat - lastValidLatency) : 0.0f;
        lastValidLatency = lat; hasPrevLatency = true;

        recordWindow(false, lat, jit, LINK_BLE);
        sampleCount++;
        char tsStr[16];
        formatTimestamp(pendingList[i].sentMs, tsStr, sizeof(tsStr));
        Serial.printf("%u,%s,%u,%s,BNO055,ADAPTIVE,BLE,%.2f,%.2f,%.2f,0,%d\n",
                      eSeq, tsStr, pendingList[i].sentMs - sessionStartMs,
                      MCU_NAME, rtt, lat, jit, pendingList[i].rssi);

        // Catat keberhasilan transisi Handover ke BLE
        if (handoverPending && currentLink == LINK_BLE) {
          uint32_t nowHandoverSuccess = millis();
          uint32_t handoverDurationMs = (nowHandoverSuccess >= handoverStartMs) ? (nowHandoverSuccess - handoverStartMs) : 0;
          char handoverEndTimeStr[16];
          formatTimestamp(nowHandoverSuccess, handoverEndTimeStr, sizeof(handoverEndTimeStr));
          handoverPending = false;

          Serial.println("\n============================================================");
          Serial.printf("[HANDOVER-SUCCESS #%u] 🚀 Transisi Wi-Fi -> BLE BERHASIL!\n", totalSwitches);
          Serial.println("  * Jalur Transisi : Wi-Fi UDP -> BLE");
          Serial.printf("  * Mulai Pukul    : %s WIB\n", handoverStartTimeStr);
          Serial.printf("  * Berhasil Pukul : %s WIB (Realtime)\n", handoverEndTimeStr);
          Serial.printf("  * Durasi Switch  : %u ms (Handover Delay / Disruption Time)\n", handoverDurationMs);
          Serial.printf("  * Paket Pertama  : Seq #%u\n", eSeq);
          Serial.println("============================================================\n");
        }

        // Cek apakah sesi 1 menit sudah selesai
        if (millis() - sessionStartMs >= SESSION_DURATION_MS) {
          sessionActive = false;
          Serial.printf("\n[SELESAI] Sesi 1 menit selesai! Total data: %u paket\n", sampleCount);
        }
        break;
      }
    }
  }

  // 2. Cek Echo Wi-Fi UDP
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
      streamingStarted = true;
      sessionActive = true;
      for (int i = 0; i < 16; i++) pendingList[i].pending = false;
      Serial.println("[UDP] Sinyal START diterima! Sesi baru dimulai dari paket #1.");
      return;
    }

    receiverIP = udp.remoteIP();
    receiverPort = udp.remotePort();
    receiverLocked = true;

    uint32_t echoSeq = 0;
    memcpy(&echoSeq, rxBuf, sizeof(uint32_t));

    for (int i = 0; i < 16; i++) {
      if (pendingList[i].pending && pendingList[i].seq == echoSeq && pendingList[i].link == LINK_WIFI) {
        pendingList[i].pending = false;
        float rtt = (float)(nowMs - pendingList[i].sentMs);
        if (rtt < 0) rtt = 0;
        float lat = rtt / 2.0f;
        float jit = hasPrevLatency ? fabsf(lat - lastValidLatency) : 0.0f;
        lastValidLatency = lat; hasPrevLatency = true;

        recordWindow(false, lat, jit, LINK_WIFI);
        sampleCount++;
        char tsStr[16];
        formatTimestamp(pendingList[i].sentMs, tsStr, sizeof(tsStr));
        Serial.printf("%u,%s,%u,%s,BNO055,ADAPTIVE,WIFI,%.2f,%.2f,%.2f,0,%d\n",
                      echoSeq, tsStr, pendingList[i].sentMs - sessionStartMs,
                      MCU_NAME, rtt, lat, jit, pendingList[i].rssi);

        // Catat keberhasilan transisi Handover ke Wi-Fi UDP
        if (handoverPending && currentLink == LINK_WIFI) {
          uint32_t nowHandoverSuccess = millis();
          uint32_t handoverDurationMs = (nowHandoverSuccess >= handoverStartMs) ? (nowHandoverSuccess - handoverStartMs) : 0;
          char handoverEndTimeStr[16];
          formatTimestamp(nowHandoverSuccess, handoverEndTimeStr, sizeof(handoverEndTimeStr));
          handoverPending = false;

          Serial.println("\n============================================================");
          Serial.printf("[HANDOVER-SUCCESS #%u] 🚀 Transisi BLE -> Wi-Fi BERHASIL!\n", totalSwitches);
          Serial.println("  * Jalur Transisi : BLE -> Wi-Fi UDP");
          Serial.printf("  * Mulai Pukul    : %s WIB\n", handoverStartTimeStr);
          Serial.printf("  * Berhasil Pukul : %s WIB (Realtime)\n", handoverEndTimeStr);
          Serial.printf("  * Durasi Switch  : %u ms (Handover Delay / Disruption Time)\n", handoverDurationMs);
          Serial.printf("  * Paket Pertama  : Seq #%u\n", echoSeq);
          Serial.println("============================================================\n");
        }

        // Cek apakah sesi 1 menit sudah selesai
        if (millis() - sessionStartMs >= SESSION_DURATION_MS) {
          sessionActive = false;
          Serial.printf("\n[SELESAI] Sesi 1 menit selesai! Total data: %u paket\n", sampleCount);
        }
        break;
      }
    }
  }

  // 3. Cek Timeout (Packet Loss)
  for (int i = 0; i < 16; i++) {
    if (pendingList[i].pending && (nowMs - pendingList[i].sentMs > TIMEOUT_RTT_MS)) {
      pendingList[i].pending = false;
      recordWindow(true, 0.0f, 0.0f, pendingList[i].link);
      sampleCount++;
      const char* lStr = (pendingList[i].link == LINK_BLE) ? "BLE" : "WIFI";
      char tsStr[16];
      formatTimestamp(pendingList[i].sentMs, tsStr, sizeof(tsStr));
      Serial.printf("%u,%s,%u,%s,BNO055,ADAPTIVE,%s,0.00,0.00,0.00,1,%d\n",
                    pendingList[i].seq, tsStr, pendingList[i].sentMs - sessionStartMs,
                    MCU_NAME, lStr, pendingList[i].rssi);

      // Cek apakah sesi 1 menit sudah selesai
      if (millis() - sessionStartMs >= SESSION_DURATION_MS) {
        sessionActive = false;
        Serial.printf("\n[SELESAI] Sesi 1 menit selesai! Total data: %u paket\n", sampleCount);
      }
    }
  }

  // 4. Evaluasi Adaptive Switching
  evaluateAdaptiveSwitch();

  // Auto-start fallback jika setelah 4 detik terhubung belum ada sinyal START (misal tes manual)
  if ((bleConnected || (WiFi.status() == WL_CONNECTED)) && !streamingStarted && (nowMs - connectMs > 4000)) {
    streamingStarted = true;
    seqNumber = 1;
    sessionStartMs = nowMs;
    Serial.println("[ADAPTIVE] Auto-start timeout: Memulai streaming dari paket #1.");
  }

  // 5. Pengiriman Paket 50 Hz
  static uint32_t lastTxMs = 0;
  const uint32_t intervalMs = 1000 / SAMPLE_RATE_HZ;

  if (sessionActive && streamingStarted && (nowMs - lastTxMs >= intervalMs)) {
    lastTxMs = nowMs;

    MotionPacket pkt;
    readBNO055(pkt);
    pkt.sequenceNumber  = seqNumber++;
    pkt.sendTimestampMs = nowMs;
    pkt.buttonState     = 0;
    pkt.checksum        = calcChecksum(pkt);

    bool sentOk = false;
    int16_t curRssi = -99;

    if (currentLink == LINK_BLE && bleConnected) {
      curRssi = bleRssi;
      pTxChar->setValue((uint8_t*)&pkt, sizeof(MotionPacket));
      pTxChar->notify();
      sentOk = true;
    } else if (currentLink == LINK_WIFI && (WiFi.status() == WL_CONNECTED)) {
      curRssi = WiFi.RSSI();
      udp.beginPacket(receiverIP, receiverPort);
      udp.write((const uint8_t*)&pkt, sizeof(MotionPacket));
      udp.endPacket();
      sentOk = true;
    }

    if (sentOk) {
      for (int i = 0; i < 16; i++) {
        if (!pendingList[i].pending) {
          pendingList[i] = { pkt.sequenceNumber, pkt.sendTimestampMs, currentLink, curRssi, true };
          break;
        }
      }
    }
  }
}
