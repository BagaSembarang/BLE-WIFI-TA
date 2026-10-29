/* ============================================================================
 * SKRIPSI SISTEM INFORMASI
 * KOMBINASI 2: BLE STATIS + BNO055 (9-AXIS SENSOR FUSION)
 * 
 * Pengujian transmisi data gerak menggunakan BLE dengan sensor fisik BNO055 via I2C.
 * Durasi sesi: 1 MENIT (60 detik), timestamp JAM NYATA via NTP (HH:MM:SS.mmm WIB).
 * WiFi dinyalakan SEMENTARA hanya untuk sync NTP, lalu dimatikan sebelum BLE aktif.
 * ============================================================================
 */

#include <Arduino.h>
#include <Wire.h>
#include <WiFi.h>             // Untuk NTP sementara
#include <time.h>             // Untuk NTP / RTC real-time
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include <esp_gap_ble_api.h>
#include <esp_wifi.h>         // Untuk esp_wifi_stop() sebelum BLE init

// NTP (WiFi sementara hanya untuk sync jam)
const char* WIFI_SSID_NTP  = "Cardinal";
const char* WIFI_PASS_NTP  = "12082231";
#define NTP_SERVER1          "pool.ntp.org"
#define NTP_SERVER2          "time.google.com"
#define TZ_OFFSET_WIB        25200   // UTC+7 (WIB) dalam detik

// ============================================================================
// PILIH MIKROKONTROLER (Ganti di sini: 1 untuk C3, 2 untuk S3)
// ============================================================================
#define MCU_ESP32_C3 1
#define MCU_ESP32_S3 2

#define TARGET_MCU MCU_ESP32_S3  // <-- Dipilih ESP32-C3 (SDA=2, SCL=3)

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
// KONFIGURASI PENGUJIAN & BLE
// ============================================================================
#define SESSION_DURATION_MS   60000UL  // Durasi sesi: 1 menit
#define SAMPLE_RATE_HZ        50
#define TIMEOUT_RTT_MS        100

#define BLE_DEVICE_NAME       "ESP32-Motion-Ctrl"
#define BLE_SERVICE_UUID      "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
#define BLE_CHAR_TX_UUID      "beb5483e-36e1-4688-b7f5-ea07361b26a8"
#define BLE_CHAR_RX_UUID      "d29759bc-4b6f-45a6-8e50-96f7e44fa70c"

// Register BNO055 I2C
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

BLEServer*         pServer = nullptr;
BLECharacteristic* pTxChar = nullptr;
BLECharacteristic* pRxChar = nullptr;
bool               deviceConnected = false;
int16_t            currentRssi = -65;
esp_bd_addr_t      remoteBda;
bool               hasRemoteBda = false;
bool               bnoDetected = false;

volatile uint32_t  lastEchoSeq = 0;
volatile uint32_t  lastEchoTimeMs = 0;
volatile bool      newEchoReceived = false;

uint32_t           sampleCount = 0;
uint32_t           seqNumber = 1;
bool               sessionActive = true;
bool               streamingStarted = false; // Tunggu sinyal START dari laptop
uint32_t           connectMs = 0;
uint32_t           sessionStartMs = 0;
time_t             ntpSyncEpoch = 0;
uint32_t           ntpSyncMillis = 0;
bool               ntpSynced = false;
float              lastValidLatency = 0.0f;
bool               hasPrevLatency = false;

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

// BNO055 I2C Helpers
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
    // Fallback: Beberapa board ESP32-S3 memerlukan endTransmission(true) dengan STOP condition
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

  // 1. Scan semua alamat I2C di bus (1 s/d 126) untuk verifikasi kabel fisik SDA/SCL
  for (uint8_t a = 1; a < 127; a++) {
    Wire.beginTransmission(a);
    if (Wire.endTransmission() == 0) {
      Serial.printf("[I2C SCAN] DITEMUKAN perangkat I2C pada alamat 0x%02X!\n", a);
      if (a == 0x28 || a == 0x29) foundAddr = a;
    }
  }

  uint8_t testAddrs[2] = { (foundAddr != 0 ? foundAddr : 0x28), (foundAddr == 0x29 ? 0x28 : 0x29) };

  // Coba scan di alamat target dengan retry hingga 5 kali
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
    bnoWrite(BNO055_OPR_MODE, 0x0C); // NDOF 9-Axis Fusion
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
  pkt.sensorId = 1; // BNO055
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
  // Baca Quat
  uint8_t qb[8];
  if (bnoRead(BNO055_QUAT_DATA, qb, 8)) {
    int16_t w = (int16_t)(((uint16_t)qb[1] << 8) | qb[0]);
    int16_t x = (int16_t)(((uint16_t)qb[3] << 8) | qb[2]);
    int16_t y = (int16_t)(((uint16_t)qb[5] << 8) | qb[4]);
    int16_t z = (int16_t)(((uint16_t)qb[7] << 8) | qb[6]);
    const float scale = 1.0f / 16384.0f;
    pkt.quatW = w * scale; pkt.quatX = x * scale; pkt.quatY = y * scale; pkt.quatZ = z * scale;
  }
  // Baca Linier Accel
  uint8_t ab[6];
  if (bnoRead(BNO055_LIA_DATA, ab, 6)) {
    int16_t ax = (int16_t)(((uint16_t)ab[1] << 8) | ab[0]);
    int16_t ay = (int16_t)(((uint16_t)ab[3] << 8) | ab[2]);
    int16_t az = (int16_t)(((uint16_t)ab[5] << 8) | ab[4]);
    pkt.accelX = ax / 100.0f; pkt.accelY = ay / 100.0f; pkt.accelZ = az / 100.0f;
  }
  // Baca Gyro
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
    currentRssi = param->read_rssi_cmpl.rssi;
  }
}

class MyServerCallbacks: public BLEServerCallbacks {
  void onConnect(BLEServer* pServer, esp_ble_gatts_cb_param_t *param) {
    deviceConnected = true;
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
    deviceConnected = false;
    streamingStarted = false;
    sessionActive = false;
    hasRemoteBda = false;
    currentRssi = -99;
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
      lastEchoSeq = seq;
      lastEchoTimeMs = millis();
      newEchoReceived = true;
    }
  }
};

void setup() {
  Serial.begin(115200);
  uint32_t sStart = millis();
  while (!Serial && millis() - sStart < 2500); // Tunggu USB CDC Serial Monitor siap (khusus ESP32-S3)
  delay(500);

  Serial.println("\n=======================================================");
  Serial.printf("  ESP32 MOTION CONTROLLER - %s\n", MCU_NAME);
  Serial.println("  METODE : BLE STATIS");
  Serial.println("  SENSOR : BNO055 (9-AXIS SENSOR FUSION)");
  Serial.println("  DURASI : 1 MENIT (60 DETIK)");
  Serial.println("  TIMESTAMP: JAM NYATA WIB via NTP (HH:MM:SS.mmm)");
  Serial.println("=======================================================");

  for (int i = 0; i < 16; i++) pendingList[i].pending = false;

  initBNO055();

  // -----------------------------------------------------------------------
  // STEP 1: Nyalakan WiFi SEMENTARA hanya untuk sync NTP
  // -----------------------------------------------------------------------
  Serial.print("[NTP] Menyambung WiFi untuk sync jam...");
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID_NTP, WIFI_PASS_NTP);
  uint32_t wStart = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - wStart < 10000) {
    delay(250); Serial.print(".");
  }
  Serial.println();
  if (WiFi.status() == WL_CONNECTED) {
    Serial.printf("[NTP] WiFi OK (%s). Sync jam...", WiFi.localIP().toString().c_str());
    configTime(TZ_OFFSET_WIB, 0, NTP_SERVER1, NTP_SERVER2);
    struct tm tmCheck;
    uint32_t ntpWait = millis();
    while (!getLocalTime(&tmCheck) && millis() - ntpWait < 10000) {
      delay(200); Serial.print(".");
    }
    Serial.println();
    if (getLocalTime(&tmCheck)) {
      ntpSyncEpoch  = mktime(&tmCheck);
      ntpSyncMillis = millis();
      ntpSynced     = true;
      Serial.printf("[NTP] Berhasil! Jam: %02d:%02d:%02d WIB\n",
                    tmCheck.tm_hour, tmCheck.tm_min, tmCheck.tm_sec);
    } else {
      Serial.println("[NTP] GAGAL! Timestamp fallback (elapsed).");
    }
    WiFi.disconnect(true); WiFi.mode(WIFI_OFF);
    esp_wifi_stop();          // Pastikan radio WiFi benar-benar berhenti
    Serial.println("[NTP] WiFi dimatikan. Memulai BLE...");
    delay(300);
  } else {
    Serial.println("[NTP] WiFi GAGAL. Timestamp fallback.");
    WiFi.disconnect(true); WiFi.mode(WIFI_OFF);
    esp_wifi_stop();          // Pastikan radio WiFi benar-benar berhenti
    delay(300);
  }
  // -----------------------------------------------------------------------

  // STEP 2: Inisialisasi BLE
  BLEDevice::init(BLE_DEVICE_NAME);  // WAJIB dipanggil sebelum apapun!
  BLEDevice::setMTU(128);
  esp_ble_gap_register_callback(gapCallback);

  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());

  BLEService* pService = pServer->createService(BLE_SERVICE_UUID);

  pTxChar = pService->createCharacteristic(
    BLE_CHAR_TX_UUID,
    BLECharacteristic::PROPERTY_NOTIFY | BLECharacteristic::PROPERTY_READ
  );
  pTxChar->addDescriptor(new BLE2902());

  pRxChar = pService->createCharacteristic(
    BLE_CHAR_RX_UUID,
    BLECharacteristic::PROPERTY_WRITE | BLECharacteristic::PROPERTY_WRITE_NR
  );
  pRxChar->setCallbacks(new MyRxCallbacks());

  pService->start();

  BLEAdvertising* pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(BLE_SERVICE_UUID);
  pAdvertising->setScanResponse(true);
  BLEDevice::startAdvertising();

  sessionStartMs = millis();
  Serial.println("[BLE] Menunggu koneksi dari aplikasi laptop/penerima...");
  Serial.println("CSV_HEADER:SequenceNumber,Timestamp,ElapsedMs,MCU,Sensor,Method,ActiveLink,RTT_ms,Latency_ms,Jitter_ms,PacketLoss,RSSI_dBm");
}

void loop() {
  uint32_t nowMs = millis();

  static uint32_t lastRssiMs = 0;
  if (deviceConnected && hasRemoteBda && (nowMs - lastRssiMs > 500)) {
    lastRssiMs = nowMs;
    esp_ble_gap_read_rssi(remoteBda);
  }

  // 1. Cek Echo
  if (newEchoReceived) {
    uint32_t eSeq = lastEchoSeq;
    uint32_t eTime = lastEchoTimeMs;
    newEchoReceived = false;

    for (int i = 0; i < 16; i++) {
      if (pendingList[i].pending && pendingList[i].seq == eSeq) {
        pendingList[i].pending = false;

        float rtt = (float)(eTime - pendingList[i].sentMs);
        if (rtt < 0) rtt = 0;
        float latency = rtt / 2.0f;
        float jitter = hasPrevLatency ? fabsf(latency - lastValidLatency) : 0.0f;
        lastValidLatency = latency;
        hasPrevLatency = true;

        sampleCount++;
        char tsStr[16];
        formatTimestamp(pendingList[i].sentMs, tsStr, sizeof(tsStr));
        Serial.printf("%u,%s,%u,%s,BNO055,BLE_STATIC,BLE,%.2f,%.2f,%.2f,0,%d\n",
                      eSeq, tsStr, pendingList[i].sentMs - sessionStartMs,
                      MCU_NAME, rtt, latency, jitter, pendingList[i].rssi);
        if (millis() - sessionStartMs >= SESSION_DURATION_MS) {
          sessionActive = false;
          Serial.printf("\n[SELESAI] Sesi 1 menit selesai! Total: %u paket\n", sampleCount);
        }
        break;
      }
    }
  }

  // 2. Cek Timeout
  for (int i = 0; i < 16; i++) {
    if (pendingList[i].pending && (nowMs - pendingList[i].sentMs > TIMEOUT_RTT_MS)) {
      pendingList[i].pending = false;
      sampleCount++;
      char tsStr[16];
      formatTimestamp(pendingList[i].sentMs, tsStr, sizeof(tsStr));
      Serial.printf("%u,%s,%u,%s,BNO055,BLE_STATIC,BLE,0.00,0.00,0.00,1,%d\n",
                    pendingList[i].seq, tsStr, pendingList[i].sentMs - sessionStartMs,
                    MCU_NAME, pendingList[i].rssi);
      if (millis() - sessionStartMs >= SESSION_DURATION_MS) {
        sessionActive = false;
        Serial.printf("\n[SELESAI] Sesi 1 menit selesai! Total: %u paket\n", sampleCount);
      }
    }
  }

  // Auto-start fallback jika setelah 4 detik terhubung belum ada sinyal START (misal tes manual)
  if (deviceConnected && !streamingStarted && (nowMs - connectMs > 4000)) {
    streamingStarted = true;
    seqNumber = 1;
    sessionStartMs = nowMs;
    Serial.println("[BLE] Auto-start timeout: Memulai streaming dari paket #1.");
  }

  // 3. Pengiriman 50 Hz
  static uint32_t lastTxMs = 0;
  const uint32_t intervalMs = 1000 / SAMPLE_RATE_HZ;

  if (sessionActive && deviceConnected && streamingStarted && (nowMs - lastTxMs >= intervalMs)) {
    lastTxMs = nowMs;

    MotionPacket pkt;
    readBNO055(pkt);
    pkt.sequenceNumber  = seqNumber++;
    pkt.sendTimestampMs = nowMs;
    pkt.buttonState     = 0;
    pkt.checksum        = calcChecksum(pkt);

    pTxChar->setValue((uint8_t*)&pkt, sizeof(MotionPacket));
    pTxChar->notify();

    for (int i = 0; i < 16; i++) {
      if (!pendingList[i].pending) {
        pendingList[i] = { pkt.sequenceNumber, pkt.sendTimestampMs, currentRssi, true };
        break;
      }
    }
  }
}
