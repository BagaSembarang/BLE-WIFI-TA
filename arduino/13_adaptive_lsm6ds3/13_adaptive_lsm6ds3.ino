/* ============================================================================
 * SKRIPSI SISTEM INFORMASI
 * KOMBINASI 13: ADAPTIVE SWITCHING (BLE <-> WI-FI) + LSM6DS3 (6-AXIS SENSOR FUSION)
 * 
 * Pengujian transmisi data gerak dengan Adaptive Switching (BLE <-> Wi-Fi UDP)
 * menggunakan sensor fisik ST LSM6DS3 via I2C dengan Hysteresis & Dwell Time.
 * Durasi sesi: 1 MENIT (60 detik), timestamp JAM NYATA via NTP (HH:MM:SS.mmm WIB).
 *
 * Wiring LSM6DS3 (I2C):
 *   VCC       -> 3.3V
 *   GND       -> GND
 *   SDA       -> GPIO 2 (ESP32-C3) / GPIO 8 (ESP32-S3)
 *   SCL       -> GPIO 3 (ESP32-C3) / GPIO 9 (ESP32-S3)
 *   CS        -> 3.3V (atau pull-up bawaan modul untuk mode I2C)
 *   SA0 / SDO -> GND (Alamat I2C: 0x6A) | 3.3V (Alamat I2C: 0x6B)
 * ============================================================================
 */

#include <Arduino.h>
#include <Wire.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include <time.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include <esp_gap_ble_api.h>

// NTP
#define NTP_SERVER1       "pool.ntp.org"
#define NTP_SERVER2       "time.google.com"
#define TZ_OFFSET_WIB     25200

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

#define SESSION_DURATION_MS   60000UL
#define SAMPLE_RATE_HZ        50
#define TIMEOUT_RTT_MS        100

// Parameter Adaptive Switching
#define DWELL_TIME_PACKETS    30
#define ADAPTIVE_WINDOW       15

#define LOSS_THRESH_BLE_WIFI  15.0f
#define LAT_THRESH_BLE_WIFI   45.0f
#define LOSS_THRESH_WIFI_BLE  20.0f
#define LAT_THRESH_WIFI_BLE   55.0f

// Register LSM6DS3
uint8_t lsmAddr               = 0x6A;
#define LSM6DS3_WHO_AM_I      0x0F
#define LSM6DS3_CTRL1_XL      0x10
#define LSM6DS3_CTRL2_G       0x11
#define LSM6DS3_CTRL3_C       0x12
#define LSM6DS3_OUTX_L_G      0x22
#define LSM6DS3_OUTX_L_XL     0x28

#define ACCEL_SENSITIVITY     (0.000061f * 9.80665f)
#define GYRO_SENSITIVITY      (0.00875f * (PI / 180.0f))

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
  uint8_t sensorId;    // 4 = LSM6DS3
  uint8_t checksum;
};
#pragma pack(pop)

enum LinkState { LINK_BLE = 0, LINK_WIFI = 1 };
LinkState currentLink = LINK_BLE;

WiFiUDP            udp;
IPAddress          receiverIP(255, 255, 255, 255);
uint16_t           receiverPort = UDP_REMOTE_PORT;
bool               receiverLocked = false;
bool               lsmDetected = false;

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
bool               streamingStarted = false;
uint32_t           connectMs = 0;
uint32_t           sessionStartMs = 0;
time_t             ntpSyncEpoch = 0;
uint32_t           ntpSyncMillis = 0;
bool               ntpSynced = false;
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

// ============================================================================
// MADGWICK AHRS SENSOR FUSION
// ============================================================================
float beta = 0.1f;
float q0 = 1.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f;

void madgwickUpdate(float gx, float gy, float gz, float ax, float ay, float az, float dt) {
  float recipNorm;
  float s0, s1, s2, s3;
  float qDot1, qDot2, qDot3, qDot4;
  float _2q0, _2q1, _2q2, _2q3, _4q0, _4q1, _4q2, _8q1, _8q2, q0q0, q1q1, q2q2, q3q3;

  qDot1 = 0.5f * (-q1 * gx - q2 * gy - q3 * gz);
  qDot2 = 0.5f * (q0 * gx + q2 * gz - q3 * gy);
  qDot3 = 0.5f * (q0 * gy - q1 * gz + q3 * gx);
  qDot4 = 0.5f * (q0 * gz + q1 * gy - q2 * gx);

  float aNormSq = ax * ax + ay * ay + az * az;
  if (aNormSq > 0.01f) {
    recipNorm = 1.0f / sqrtf(aNormSq);
    ax *= recipNorm; ay *= recipNorm; az *= recipNorm;

    _2q0=2.0f*q0; _2q1=2.0f*q1; _2q2=2.0f*q2; _2q3=2.0f*q3;
    _4q0=4.0f*q0; _4q1=4.0f*q1; _4q2=4.0f*q2;
    _8q1=8.0f*q1; _8q2=8.0f*q2;
    q0q0=q0*q0; q1q1=q1*q1; q2q2=q2*q2; q3q3=q3*q3;

    s0=_4q0*q2q2+_2q2*ax+_4q0*q1q1-_2q1*ay;
    s1=_4q1*q3q3-_2q3*ax+4.0f*q0q0*q1-_2q0*ay-_4q1+_8q1*q1q1+_8q1*q2q2+_4q1*az;
    s2=4.0f*q0q0*q2+_2q0*ax+_4q2*q3q3-_2q3*ay-_4q2+_8q2*q1q1+_8q2*q2q2+_4q2*az;
    s3=4.0f*q1q1*q3-_2q1*ax+4.0f*q2q2*q3-_2q2*ay;

    float sNormSq=s0*s0+s1*s1+s2*s2+s3*s3;
    if (sNormSq > 0.0001f) {
      recipNorm=1.0f/sqrtf(sNormSq);
      s0*=recipNorm; s1*=recipNorm; s2*=recipNorm; s3*=recipNorm;
      qDot1-=beta*s0; qDot2-=beta*s1; qDot3-=beta*s2; qDot4-=beta*s3;
    }
  }

  q0+=qDot1*dt; q1+=qDot2*dt; q2+=qDot3*dt; q3+=qDot4*dt;

  float qNormSq=q0*q0+q1*q1+q2*q2+q3*q3;
  if (qNormSq > 0.00001f) {
    recipNorm=1.0f/sqrtf(qNormSq);
    q0*=recipNorm; q1*=recipNorm; q2*=recipNorm; q3*=recipNorm;
  } else {
    q0=1.0f; q1=0.0f; q2=0.0f; q3=0.0f;
  }
}

uint8_t calcChecksum(const MotionPacket& pkt) {
  const uint8_t* p = (const uint8_t*)&pkt;
  uint8_t cs = 0;
  for (size_t i = 0; i < sizeof(MotionPacket) - 1; i++) cs ^= p[i];
  return cs;
}

void formatTimestamp(uint32_t ms, char* buf, size_t bufLen) {
  if (ntpSynced && ntpSyncEpoch > 0) {
    uint32_t driftMs = (ms >= ntpSyncMillis) ? (ms - ntpSyncMillis) : 0;
    time_t realEpoch = ntpSyncEpoch + (driftMs / 1000);
    uint32_t milliPart = driftMs % 1000;
    struct tm* tmInfo = localtime(&realEpoch);
    snprintf(buf, bufLen, "%02d:%02d:%02d.%03u",
             tmInfo->tm_hour, tmInfo->tm_min, tmInfo->tm_sec, milliPart);
  } else {
    uint32_t elapsed = ms;
    uint32_t totalSec = elapsed / 1000;
    uint32_t milliPart = elapsed % 1000;
    snprintf(buf, bufLen, "%02u:%02u:%02u.%03u",
             totalSec / 3600, (totalSec % 3600) / 60, totalSec % 60, milliPart);
  }
}

// I2C Helpers
bool lsmWrite(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(lsmAddr);
  Wire.write(reg); Wire.write(val);
  return (Wire.endTransmission() == 0);
}

bool lsmRead(uint8_t reg, uint8_t* buf, uint8_t len) {
  Wire.beginTransmission(lsmAddr);
  Wire.write(reg);
  uint8_t err = Wire.endTransmission(false);
  if (err != 0) {
    Wire.beginTransmission(lsmAddr);
    Wire.write(reg);
    if (Wire.endTransmission(true) != 0) return false;
  }
  uint8_t count = Wire.requestFrom((int)lsmAddr, (int)len);
  if (count != len) return false;
  for (int i = 0; i < len; i++) buf[i] = Wire.read();
  return true;
}

void initLSM6DS3() {
  Wire.begin(I2C_SDA, I2C_SCL, 400000);
  delay(100);

  Serial.println("\n-------------------------------------------------------");
  Serial.printf("[LSM6DS3] Memindai bus I2C (SDA: GPIO %d, SCL: GPIO %d)...\n", I2C_SDA, I2C_SCL);

  uint8_t id = 0;
  lsmDetected = false;
  uint8_t foundAddr = 0;

  for (uint8_t a = 1; a < 127; a++) {
    Wire.beginTransmission(a);
    if (Wire.endTransmission() == 0) {
      Serial.printf("[I2C SCAN] DITEMUKAN perangkat I2C pada alamat 0x%02X!\n", a);
      if (a == 0x6A || a == 0x6B) foundAddr = a;
    }
  }

  uint8_t testAddrs[2] = { (foundAddr != 0 ? foundAddr : 0x6A), (foundAddr == 0x6B ? 0x6A : 0x6B) };
  for (int aIdx = 0; aIdx < 2 && !lsmDetected; aIdx++) {
    lsmAddr = testAddrs[aIdx];
    for (int attempt = 1; attempt <= 5; attempt++) {
      if (lsmRead(LSM6DS3_WHO_AM_I, &id, 1)) {
        Serial.printf("[LSM6DS3] Probe 0x%02X -> WHO_AM_I: 0x%02X\n", lsmAddr, id);
        if (id == 0x69 || id == 0x6A || id == 0x6C || (id != 0x00 && id != 0xFF)) {
          lsmDetected = true;
          break;
        }
      }
      delay(50);
    }
  }

  if (lsmDetected) {
    lsmWrite(LSM6DS3_CTRL3_C, 0x01); delay(50);
    lsmWrite(LSM6DS3_CTRL3_C, 0x44); delay(10);
    lsmWrite(LSM6DS3_CTRL1_XL, 0x40); delay(10);
    lsmWrite(LSM6DS3_CTRL2_G, 0x40); delay(20);

    const char* chipVariant = "LSM6DS3";
    if (id == 0x69) chipVariant = "LSM6DS3 (Chip ID: 0x69)";
    else if (id == 0x6A) chipVariant = "LSM6DS3TR-C (Chip ID: 0x6A)";
    else if (id == 0x6C) chipVariant = "LSM6DSO (Chip ID: 0x6C)";
    else chipVariant = "LSM6DS3 Family / Compatible";

    Serial.printf("[LSM6DS3] SUKSES! Terdeteksi %s pada 0x%02X. Mode Aktif 50Hz Transmisi!\n", chipVariant, lsmAddr);
  } else {
    Serial.printf("[LSM6DS3] PERINGATAN: Sensor tidak merespons di 0x6A maupun 0x6B (SDA: GPIO %d, SCL: GPIO %d).\n", I2C_SDA, I2C_SCL);
  }
  Serial.println("-------------------------------------------------------\n");
}

static float lastAx = 0.0f, lastAy = 0.0f, lastAz = 9.81f;
static float lastGx = 0.0f, lastGy = 0.0f, lastGz = 0.0f;
static uint8_t lsmFailCount = 0;

void readLSM6DS3(MotionPacket& pkt) {
  pkt.sensorId = 4; // 4 = LSM6DS3
  if (!lsmDetected) {
    static uint32_t lastRetryMs = 0;
    if (millis() - lastRetryMs > 3000) { lastRetryMs = millis(); initLSM6DS3(); }
    pkt.quatW=1.0f; pkt.quatX=0.0f; pkt.quatY=0.0f; pkt.quatZ=0.0f;
    pkt.accelX=0.0f; pkt.accelY=0.0f; pkt.accelZ=9.81f;
    pkt.gyroX=0.0f; pkt.gyroY=0.0f; pkt.gyroZ=0.0f;
    return;
  }

  uint8_t rawBuf[12];
  bool ok = lsmRead(LSM6DS3_OUTX_L_G, rawBuf, 12);
  if (ok) {
    lsmFailCount = 0;
    int16_t rawGx = (int16_t)(((uint16_t)rawBuf[1] << 8) | rawBuf[0]);
    int16_t rawGy = (int16_t)(((uint16_t)rawBuf[3] << 8) | rawBuf[2]);
    int16_t rawGz = (int16_t)(((uint16_t)rawBuf[5] << 8) | rawBuf[4]);
    int16_t rawAx = (int16_t)(((uint16_t)rawBuf[7] << 8) | rawBuf[6]);
    int16_t rawAy = (int16_t)(((uint16_t)rawBuf[9] << 8) | rawBuf[8]);
    int16_t rawAz = (int16_t)(((uint16_t)rawBuf[11] << 8) | rawBuf[10]);

    lastAx = (float)rawAx * ACCEL_SENSITIVITY;
    lastAy = (float)rawAy * ACCEL_SENSITIVITY;
    lastAz = (float)rawAz * ACCEL_SENSITIVITY;

    lastGx = (float)rawGx * GYRO_SENSITIVITY;
    lastGy = (float)rawGy * GYRO_SENSITIVITY;
    lastGz = (float)rawGz * GYRO_SENSITIVITY;

    madgwickUpdate(lastGx, lastGy, lastGz, lastAx, lastAy, lastAz, 0.020f);
  } else {
    lsmFailCount++;
    if (lsmFailCount >= 10) {
      Serial.println("[LSM6DS3] I2C Error berulang! Mencoba reinisialisasi...");
      lsmDetected = false; lsmFailCount = 0;
    }
  }

  pkt.accelX = lastAx; pkt.accelY = lastAy; pkt.accelZ = lastAz;
  pkt.gyroX  = lastGx; pkt.gyroY  = lastGy; pkt.gyroZ  = lastGz;
  pkt.quatW  = q0;     pkt.quatX  = q1;     pkt.quatY  = q2;     pkt.quatZ = q3;
}

static void gapCallback(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param) {
  if (event == ESP_GAP_BLE_READ_RSSI_COMPLETE_EVT && param->read_rssi_cmpl.status == ESP_BT_STATUS_SUCCESS) {
    bleRssi = param->read_rssi_cmpl.rssi;
  }
}

class MyServerCallbacks: public BLEServerCallbacks {
  void onConnect(BLEServer* pServer, esp_ble_gatts_cb_param_t *param) {
    bleConnected = true;
    streamingStarted = false;
    sessionActive = true;
    sampleCount = 0;
    seqNumber = 1;
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
  delay(1000);

  Serial.println("\n=======================================================");
  Serial.printf("  ESP32 MOTION CONTROLLER - %s\n", MCU_NAME);
  Serial.println("  METODE : ADAPTIVE SWITCHING (BLE <-> WI-FI)");
  Serial.println("  SENSOR : LSM6DS3 (6-AXIS SENSOR FUSION)");
  Serial.println("  DURASI : 1 MENIT (60 DETIK)");
  Serial.println("  TIMESTAMP: JAM NYATA WIB via NTP (HH:MM:SS.mmm)");
  Serial.println("=======================================================");

  for (int i = 0; i < 16; i++) pendingList[i].pending = false;

  initLSM6DS3();

  // Koneksi Wi-Fi
  Serial.printf("[Wi-Fi] Menghubungkan ke SSID: %s ", WIFI_SSID);
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  uint32_t wStart = millis();
  while (WiFi.status() != WL_CONNECTED && (millis() - wStart < 12000)) {
    delay(300); Serial.print(".");
  }
  Serial.println();
  if (WiFi.status() == WL_CONNECTED) {
    Serial.printf("[Wi-Fi] Berhasil terhubung! IP: %s\n", WiFi.localIP().toString().c_str());
  } else {
    Serial.println("[Wi-Fi] Gagal terhubung router. Mode BLE tetap aktif.");
  }

  // NTP Sync
  if (WiFi.status() == WL_CONNECTED) {
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
    }
  }

  udp.begin(UDP_LOCAL_PORT);

  // Inisialisasi BLE
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
  sessionStartMs = millis();
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
        Serial.printf("%u,%s,%u,%s,LSM6DS3,ADAPTIVE,BLE,%.2f,%.2f,%.2f,0,%d\n",
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
        Serial.printf("%u,%s,%u,%s,LSM6DS3,ADAPTIVE,WIFI,%.2f,%.2f,%.2f,0,%d\n",
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
      Serial.printf("%u,%s,%u,%s,LSM6DS3,ADAPTIVE,%s,0.00,0.00,0.00,1,%d\n",
                    pendingList[i].seq, tsStr, pendingList[i].sentMs - sessionStartMs,
                    MCU_NAME, lStr, pendingList[i].rssi);

      if (millis() - sessionStartMs >= SESSION_DURATION_MS) {
        sessionActive = false;
        Serial.printf("\n[SELESAI] Sesi 1 menit selesai! Total data: %u paket\n", sampleCount);
      }
    }
  }

  // 4. Evaluasi Adaptive Switching
  evaluateAdaptiveSwitch();

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
    readLSM6DS3(pkt);
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
