/* ============================================================================
 * SKRIPSI SISTEM INFORMASI
 * EKSPERIMEN: BLE STATIS NO SENSOR - BURST / BATCH TRANSMISSION (MISAL 100 DATA)
 * 
 * Modifikasi dari Kombinasi 1 (ble_nosensor) untuk pengujian transmisi data
 * sekaligus banyak (Burst Mode, default: 100 paket per burst).
 * 
 * Tetap mempertahankan:
 * 1. Payload standar 52-bytes (kompatibel penuh dengan app_penerima_gui.py)
 * 2. Real-time RTT, Latency, Jitter, dan Packet Loss tracking
 * 3. NTP Real-time Clock Sync via WiFi sebelum BLE aktif
 * 4. Kompatibilitas ESP32-C3 dan ESP32-S3
 * ============================================================================
 */

#include <Arduino.h>
#include <WiFi.h>             // Untuk NTP sementara
#include <time.h>             // Untuk NTP / RTC real-time
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include <esp_gap_ble_api.h>

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

#define TARGET_MCU MCU_ESP32_C3  // <-- Dipilih ESP32-C3

#if (TARGET_MCU == MCU_ESP32_C3)
  #define MCU_NAME "ESP32-C3"
#else
  #define MCU_NAME "ESP32-S3"
#endif

// ============================================================================
// KONFIGURASI PENGUJIAN & BLE BURST MODE
// ============================================================================
#define SESSION_DURATION_MS   60000UL  // Durasi sesi: 1 menit
#define TIMEOUT_RTT_MS        200      // Batas waktu tunggu echo sebelum dianggap Loss (dinaikkan sedikit untuk burst)

// PENGATURAN BATCH / BURST PENGIRIMAN DATA
#define BURST_PACKET_COUNT    100      // Jumlah paket yang ditembakkan per burst (misal 100 paket)
#define BURST_INTERVAL_MS     2500     // Jeda antar-batch burst (misal setiap 2.5 detik tembak 100 paket)
#define INTER_PACKET_DELAY_MS 4        // Jeda mikro antar-paket (3-5 ms) agar BLE buffer internal tidak overflow

#define BLE_DEVICE_NAME       "ESP32-Motion-Ctrl"
#define BLE_SERVICE_UUID      "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
#define BLE_CHAR_TX_UUID      "beb5483e-36e1-4688-b7f5-ea07361b26a8" // ESP32 -> Penerima
#define BLE_CHAR_RX_UUID      "d29759bc-4b6f-45a6-8e50-96f7e44fa70c" // Echo Penerima -> ESP32

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

// Variabel Global
BLEServer*         pServer = nullptr;
BLECharacteristic* pTxChar = nullptr;
BLECharacteristic* pRxChar = nullptr;
bool               deviceConnected = false;
int16_t            currentRssi = -65;
esp_bd_addr_t      remoteBda;
bool               hasRemoteBda = false;

volatile uint32_t  lastEchoSeq = 0;
volatile uint32_t  lastEchoTimeMs = 0;
volatile bool      newEchoReceived = false;

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

// Pelacakan RTT Non-blocking diperbesar ukurannya untuk menampung burst 100 paket
#define PENDING_LIST_SIZE 150
struct SentRecord {
  uint32_t seq;
  uint32_t sentMs;
  int16_t  rssi;
  bool     pending;
};
SentRecord pendingList[PENDING_LIST_SIZE];

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
    uint32_t elapsed = ms;
    uint32_t totalSec = elapsed / 1000;
    snprintf(buf, bufLen, "%02u:%02u:%02u.%03u",
             totalSec / 3600, (totalSec % 3600) / 60, totalSec % 60, (uint32_t)(elapsed % 1000));
  }
}

// Callback GAP untuk pembacaan RSSI
static void gapCallback(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param) {
  if (event == ESP_GAP_BLE_READ_RSSI_COMPLETE_EVT && param->read_rssi_cmpl.status == ESP_BT_STATUS_SUCCESS) {
    currentRssi = param->read_rssi_cmpl.rssi;
  }
}

// Callback Server BLE
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

// Callback Karakteristik RX (Echo masuk / Sinyal START)
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
      for (int i = 0; i < PENDING_LIST_SIZE; i++) pendingList[i].pending = false;
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

// Fungsi pembantu untuk memproses echo yang masuk
void checkEcho() {
  if (newEchoReceived) {
    uint32_t eSeq = lastEchoSeq;
    uint32_t eTime = lastEchoTimeMs;
    newEchoReceived = false;

    for (int i = 0; i < PENDING_LIST_SIZE; i++) {
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
        Serial.printf("%u,%s,%u,%s,NO_SENSOR,BLE_BURST,BLE,%.2f,%.2f,%.2f,0,%d\n",
                      eSeq, tsStr, pendingList[i].sentMs - sessionStartMs,
                      MCU_NAME, rtt, latency, jitter, pendingList[i].rssi);

        if (millis() - sessionStartMs >= SESSION_DURATION_MS) {
          sessionActive = false;
          Serial.printf("\n[SELESAI] Sesi 1 menit selesai! Total data: %u paket\n", sampleCount);
        }
        break;
      }
    }
  }
}

// Fungsi pembantu untuk cek timeout RTT
void checkTimeout(uint32_t nowMs) {
  for (int i = 0; i < PENDING_LIST_SIZE; i++) {
    if (pendingList[i].pending && (nowMs - pendingList[i].sentMs > TIMEOUT_RTT_MS)) {
      pendingList[i].pending = false;
      sampleCount++;
      char tsStr[16];
      formatTimestamp(pendingList[i].sentMs, tsStr, sizeof(tsStr));
      Serial.printf("%u,%s,%u,%s,NO_SENSOR,BLE_BURST,BLE,0.00,0.00,0.00,1,%d\n",
                    pendingList[i].seq, tsStr, pendingList[i].sentMs - sessionStartMs,
                    MCU_NAME, pendingList[i].rssi);

      if (millis() - sessionStartMs >= SESSION_DURATION_MS) {
        sessionActive = false;
        Serial.printf("\n[SELESAI] Sesi 1 menit selesai! Total data: %u paket\n", sampleCount);
      }
    }
  }
}

void setup() {
  Serial.begin(115200);
  delay(1000);

  Serial.println("\n=======================================================");
  Serial.printf("  ESP32 MOTION CONTROLLER - %s\n", MCU_NAME);
  Serial.println("  METODE : BLE STATIS (BURST BATCH MODE)");
  Serial.printf("  BATCH  : %d PAKET PER BURST (Interval %d ms)\n", BURST_PACKET_COUNT, BURST_INTERVAL_MS);
  Serial.println("  SENSOR : NO_SENSOR (BASELINE KONTROL)");
  Serial.println("  DURASI : 1 MENIT (60 DETIK)");
  Serial.println("  TIMESTAMP: JAM NYATA WIB via NTP (HH:MM:SS.mmm)");
  Serial.println("=======================================================");

  for (int i = 0; i < PENDING_LIST_SIZE; i++) pendingList[i].pending = false;

  // -----------------------------------------------------------------------
  // STEP 1: Nyalakan WiFi SEMENTARA hanya untuk sync NTP
  // -----------------------------------------------------------------------
  Serial.print("[NTP] Menyambung WiFi untuk sync jam...");
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID_NTP, WIFI_PASS_NTP);
  uint32_t wStart = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - wStart < 10000) {
    delay(250);
    Serial.print(".");
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
      Serial.printf("[NTP] Berhasil! Jam sekarang: %02d:%02d:%02d WIB\n",
                    tmCheck.tm_hour, tmCheck.tm_min, tmCheck.tm_sec);
    } else {
      Serial.println("[NTP] GAGAL sync! Timestamp fallback (elapsed).");
    }
    // MATIKAN WiFi setelah NTP selesai
    WiFi.disconnect(true);
    WiFi.mode(WIFI_OFF);
    Serial.println("[NTP] WiFi dimatikan. Memulai BLE...");
    delay(200);
  } else {
    Serial.println("[NTP] WiFi GAGAL. Timestamp fallback (elapsed).");
    WiFi.disconnect(true);
    WiFi.mode(WIFI_OFF);
  }
  // -----------------------------------------------------------------------

  // STEP 2: Inisialisasi BLE
  BLEDevice::init(BLE_DEVICE_NAME);
  BLEDevice::setMTU(256); // Meminta MTU lebih besar agar throughput lancar
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

  // Polling RSSI setiap 500ms
  static uint32_t lastRssiMs = 0;
  if (deviceConnected && hasRemoteBda && (nowMs - lastRssiMs > 500)) {
    lastRssiMs = nowMs;
    esp_ble_gap_read_rssi(remoteBda);
  }

  // 1. Cek Echo yang Diterima
  checkEcho();

  // 2. Cek Timeout (Packet Loss)
  checkTimeout(nowMs);

  // Auto-start fallback jika setelah 4 detik terhubung belum ada sinyal START (misal tes serial manual)
  if (deviceConnected && !streamingStarted && (nowMs - connectMs > 4000)) {
    streamingStarted = true;
    seqNumber = 1;
    sessionStartMs = nowMs;
    Serial.println("[BLE] Auto-start timeout: Memulai streaming dari paket #1.");
  }

  // 3. Pengiriman Paket Mode Burst (Batch misal 100 paket)
  static uint32_t lastBurstMs = 0;

  if (sessionActive && deviceConnected && streamingStarted && (nowMs - lastBurstMs >= BURST_INTERVAL_MS)) {
    lastBurstMs = nowMs;
    Serial.printf("[BURST] Memulai transmisi batch %d paket...\n", BURST_PACKET_COUNT);

    for (int b = 0; b < BURST_PACKET_COUNT; b++) {
      if (!sessionActive || !deviceConnected) break;

      uint32_t pktMs = millis();
      MotionPacket pkt;
      pkt.sequenceNumber  = seqNumber++;
      pkt.sendTimestampMs = pktMs;
      pkt.quatW  = 1.0f; pkt.quatX = 0.0f; pkt.quatY = 0.0f; pkt.quatZ = 0.0f;
      pkt.accelX = 0.0f; pkt.accelY = 0.0f; pkt.accelZ = 9.81f;
      pkt.gyroX  = 0.0f; pkt.gyroY  = 0.0f; pkt.gyroZ  = 0.0f;
      pkt.buttonState = 0;
      pkt.sensorId = 0; // NO_SENSOR
      pkt.checksum = calcChecksum(pkt);

      // Kirim via BLE Notify
      pTxChar->setValue((uint8_t*)&pkt, sizeof(MotionPacket));
      pTxChar->notify();

      // Catat ke daftar pending RTT
      for (int i = 0; i < PENDING_LIST_SIZE; i++) {
        if (!pendingList[i].pending) {
          pendingList[i] = { pkt.sequenceNumber, pkt.sendTimestampMs, currentRssi, true };
          break;
        }
      }

      // Jeda mikro krusial antar-paket (3-5 ms) agar buffer GATT BLE tidak tersumbat/drop
      if (INTER_PACKET_DELAY_MS > 0) {
        delay(INTER_PACKET_DELAY_MS);
      }

      // Cek echo & timeout di sela-sela pengiriman burst
      checkEcho();
      checkTimeout(millis());
    }
    Serial.println("[BURST] Selesai 1 batch!");
  }
}
