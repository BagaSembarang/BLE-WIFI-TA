/* ============================================================================
 * SKRIPSI SISTEM INFORMASI
 * TRANSMISI WI-FI STATIS (UDP ROUTER) + MPU-6050 (6-AXIS SENSOR FUSION)
 *
 * Pengujian transmisi data gerak menggunakan Wi-Fi UDP melalui router dengan sensor MPU-6050.
 * Durasi sesi: 1 MENIT (60 detik), timestamp JAM NYATA via NTP (HH:MM:SS.mmm WIB).
 * Menggunakan direct register reading via Wire.h dan algoritma Madgwick AHRS internal.
 *
 * Library yang dibutuhkan:
 *   - Wire.h (built-in)
 *   - WiFi.h + WiFiUdp.h (built-in dalam ESP32 Arduino core)
 *
 * Wiring MPU-6050:
 *   VCC  -> 3.3V
 *   GND  -> GND
 *   SDA  -> GPIO 2 (ESP32-C3) / GPIO 8 (ESP32-S3)
 *   SCL  -> GPIO 3 (ESP32-C3) / GPIO 9 (ESP32-S3)
 *   AD0  -> GND (Alamat I2C: 0x68) | HIGH (Alamat I2C: 0x69)
 * ============================================================================
 */

#include <Arduino.h>
#include <Wire.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include <time.h>

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
// KONFIGURASI ROUTER WI-FI & UDP
// ============================================================================
const char* WIFI_SSID     = "Cardinal";       // Sesuaikan SSID Router
const char* WIFI_PASSWORD = "12082231";       // Sesuaikan Password Router

#define UDP_LOCAL_PORT        4210
#define UDP_REMOTE_PORT       4211
#define SESSION_DURATION_MS   60000UL
#define SAMPLE_RATE_HZ        50
#define TIMEOUT_RTT_MS        100

// NTP
#define NTP_SERVER1          "pool.ntp.org"
#define NTP_SERVER2          "time.google.com"
#define TZ_OFFSET_WIB        25200   // UTC+7 (WIB) dalam detik

// ============================================================================
// REGISTER MPU-6050
// ============================================================================
uint8_t mpuAddr               = 0x68; // 0x68 (AD0=GND), 0x69 (AD0=HIGH)

#define MPU6050_WHO_AM_I      0x75   // Expected: 0x68
#define MPU6050_PWR_MGMT_1    0x6B
#define MPU6050_SMPLRT_DIV    0x19
#define MPU6050_CONFIG        0x1A
#define MPU6050_GYRO_CONFIG   0x1B
#define MPU6050_ACCEL_CONFIG  0x1C
#define MPU6050_ACCEL_XOUT_H  0x3B
#define ACCEL_SCALE_FACTOR    16384.0f  // +-2g -> 16384 LSB/g
#define GYRO_SCALE_FACTOR     131.0f    // +-250dps -> 131 LSB/(deg/s)

// ============================================================================
// STRUKTUR PAYLOAD TETAP (52 BYTES IDENTIK dengan versi lain)
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
  uint8_t sensorId;    // 3 = MPU6050
  uint8_t checksum;
};
#pragma pack(pop)

WiFiUDP   udp;
IPAddress receiverIP(255, 255, 255, 255);
uint16_t  receiverPort = UDP_REMOTE_PORT;
bool      receiverLocked = false;
bool      mpuDetected = false;

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

// ============================================================================
// MADGWICK AHRS FILTER (6-DOF: Accel + Gyro)
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

// ============================================================================
// I2C HELPER MPU-6050
// ============================================================================
bool mpuWrite(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(mpuAddr);
  Wire.write(reg); Wire.write(val);
  return (Wire.endTransmission() == 0);
}

bool mpuRead(uint8_t reg, uint8_t* buf, uint8_t len) {
  Wire.beginTransmission(mpuAddr);
  Wire.write(reg);
  uint8_t err = Wire.endTransmission(false);
  if (err != 0) {
    Wire.beginTransmission(mpuAddr);
    Wire.write(reg);
    if (Wire.endTransmission(true) != 0) return false;
  }
  uint8_t count = Wire.requestFrom((int)mpuAddr, (int)len);
  if (count != len) return false;
  for (int i = 0; i < len; i++) buf[i] = Wire.read();
  return true;
}

void initMPU6050() {
  Wire.begin(I2C_SDA, I2C_SCL, 400000);  // 400 kHz Fast Mode
  delay(100);

  Serial.println("\n-------------------------------------------------------");
  Serial.printf("[MPU6050] Memindai bus I2C (SDA: GPIO %d, SCL: GPIO %d)...\n", I2C_SDA, I2C_SCL);

  uint8_t id = 0;
  mpuDetected = false;
  uint8_t foundAddr = 0;

  // Scan semua alamat I2C
  for (uint8_t a = 1; a < 127; a++) {
    Wire.beginTransmission(a);
    if (Wire.endTransmission() == 0) {
      Serial.printf("[I2C SCAN] DITEMUKAN perangkat I2C pada alamat 0x%02X!\n", a);
      if (a == 0x68 || a == 0x69) foundAddr = a;
    }
  }

  // Coba deteksi MPU-6050 di 0x68 atau 0x69
  uint8_t testAddrs[2] = { (foundAddr != 0 ? foundAddr : 0x68), (foundAddr == 0x69 ? 0x68 : 0x69) };
  for (int aIdx = 0; aIdx < 2 && !mpuDetected; aIdx++) {
    mpuAddr = testAddrs[aIdx];
    for (int attempt = 1; attempt <= 5; attempt++) {
      if (mpuRead(MPU6050_WHO_AM_I, &id, 1)) {
        Serial.printf("[MPU6050] Probe 0x%02X -> WHO_AM_I: 0x%02X\n", mpuAddr, id);
        // Mendukung MPU-6050 (0x68), MPU-6500 (0x70), MPU-9250 (0x71/0x73), maupun clone (0x72, dll)
        if (id == 0x68 || id == 0x70 || id == 0x71 || id == 0x72 || id == 0x73 || (id != 0x00 && id != 0xFF)) {
          mpuDetected = true;
          break;
        }
      }
      delay(50);
    }
  }

  if (mpuDetected) {
    mpuWrite(MPU6050_PWR_MGMT_1, 0x80); delay(100);  // Device Reset
    mpuWrite(MPU6050_PWR_MGMT_1, 0x01); delay(30);   // Wake up + PLL clock
    mpuWrite(MPU6050_SMPLRT_DIV, 19);   delay(5);    // 50 Hz: 1000/(1+19)
    mpuWrite(MPU6050_CONFIG, 0x03);      delay(5);    // DLPF 44Hz bandwidth
    mpuWrite(MPU6050_GYRO_CONFIG, 0x00); delay(5);    // Gyro +-250 dps
    mpuWrite(MPU6050_ACCEL_CONFIG, 0x00); delay(10);  // Accel +-2g

    const char* chipVariant = "MPU-6050";
    if (id == 0x70) chipVariant = "MPU-6500 (Kompatibel MPU-6050)";
    else if (id == 0x71 || id == 0x73) chipVariant = "MPU-9250 (Kompatibel MPU-6050)";
    else if (id != 0x68) chipVariant = "MPU Family / Clone";

    Serial.printf("[MPU6050] SUKSES! Terdeteksi %s pada 0x%02X (Chip ID: 0x%02X). Mode Aktif 50Hz!\n", chipVariant, mpuAddr, id);
    Serial.println("[MPU6050] Konfigurasi: Accel +/-2g | Gyro +/-250dps | DLPF 44Hz | 50Hz Sample Rate");
  } else {
    Serial.printf("[MPU6050] PERINGATAN: Sensor tidak merespons di 0x68 maupun 0x69 (SDA: GPIO %d, SCL: GPIO %d).\n", I2C_SDA, I2C_SCL);
    if (foundAddr == 0) Serial.println("[MPU6050] INFO: Bus I2C kosong. Periksa koneksi kabel!");
  }
  Serial.println("-------------------------------------------------------\n");
}

static float lastAx=0.0f, lastAy=0.0f, lastAz=9.81f;
static float lastGx=0.0f, lastGy=0.0f, lastGz=0.0f;
static uint8_t mpuFailCount=0;

void readMPU6050(MotionPacket& pkt) {
  pkt.sensorId = 3; // 3 = MPU6050
  if (!mpuDetected) {
    static uint32_t lastRetryMs = 0;
    if (millis() - lastRetryMs > 3000) { lastRetryMs = millis(); initMPU6050(); }
    pkt.quatW=1.0f; pkt.quatX=0.0f; pkt.quatY=0.0f; pkt.quatZ=0.0f;
    pkt.accelX=0.0f; pkt.accelY=0.0f; pkt.accelZ=9.81f;
    pkt.gyroX=0.0f; pkt.gyroY=0.0f; pkt.gyroZ=0.0f;
    return;
  }

  // Baca 14 byte sekaligus: ACCEL (6B) + TEMP (2B, diabaikan) + GYRO (6B)
  uint8_t rawBuf[14];
  bool ok = mpuRead(MPU6050_ACCEL_XOUT_H, rawBuf, 14);
  if (ok) {
    mpuFailCount = 0;
    int16_t rawAx=(int16_t)(((uint16_t)rawBuf[0]<<8)|rawBuf[1]);
    int16_t rawAy=(int16_t)(((uint16_t)rawBuf[2]<<8)|rawBuf[3]);
    int16_t rawAz=(int16_t)(((uint16_t)rawBuf[4]<<8)|rawBuf[5]);
    // rawBuf[6..7] = Temperature (skip)
    int16_t rawGx=(int16_t)(((uint16_t)rawBuf[8]<<8)|rawBuf[9]);
    int16_t rawGy=(int16_t)(((uint16_t)rawBuf[10]<<8)|rawBuf[11]);
    int16_t rawGz=(int16_t)(((uint16_t)rawBuf[12]<<8)|rawBuf[13]);

    // Konversi: Accel -> m/s^2, Gyro -> rad/s
    lastAx=(rawAx/ACCEL_SCALE_FACTOR)*9.80665f;
    lastAy=(rawAy/ACCEL_SCALE_FACTOR)*9.80665f;
    lastAz=(rawAz/ACCEL_SCALE_FACTOR)*9.80665f;
    lastGx=(rawGx/GYRO_SCALE_FACTOR)*(PI/180.0f);
    lastGy=(rawGy/GYRO_SCALE_FACTOR)*(PI/180.0f);
    lastGz=(rawGz/GYRO_SCALE_FACTOR)*(PI/180.0f);

    // Update filter Madgwick (dt = 20ms untuk 50Hz)
    madgwickUpdate(lastGx, lastGy, lastGz, lastAx, lastAy, lastAz, 0.020f);
  } else {
    mpuFailCount++;
    if (mpuFailCount >= 10) {
      Serial.println("[MPU6050] I2C Error berulang! Mencoba reinisialisasi...");
      mpuDetected = false; mpuFailCount = 0;
    }
  }

  pkt.accelX=lastAx; pkt.accelY=lastAy; pkt.accelZ=lastAz;
  pkt.gyroX=lastGx;  pkt.gyroY=lastGy;  pkt.gyroZ=lastGz;
  pkt.quatW=q0; pkt.quatX=q1; pkt.quatY=q2; pkt.quatZ=q3;
}

uint8_t calcChecksum(const MotionPacket& pkt) {
  const uint8_t* p = (const uint8_t*)&pkt;
  uint8_t cs = 0;
  for (size_t i = 0; i < sizeof(MotionPacket)-1; i++) cs ^= p[i];
  return cs;
}

void formatTimestamp(uint32_t ms, char* buf, size_t bufLen) {
  if (ntpSynced && ntpSyncEpoch > 0) {
    uint32_t driftMs=(ms>=ntpSyncMillis)?(ms-ntpSyncMillis):0;
    time_t realEpoch=ntpSyncEpoch+(driftMs/1000);
    uint32_t milliPart=driftMs%1000;
    struct tm* tmInfo=localtime(&realEpoch);
    snprintf(buf,bufLen,"%02d:%02d:%02d.%03u",tmInfo->tm_hour,tmInfo->tm_min,tmInfo->tm_sec,milliPart);
  } else {
    uint32_t totalSec=ms/1000;
    snprintf(buf,bufLen,"%02u:%02u:%02u.%03u",totalSec/3600,(totalSec%3600)/60,totalSec%60,(uint32_t)(ms%1000));
  }
}

void setup() {
  Serial.begin(115200);
  delay(1000);

  Serial.println("\n=======================================================");
  Serial.printf("  ESP32 MOTION CONTROLLER - %s\n", MCU_NAME);
  Serial.println("  METODE : WI-FI STATIS (UDP ROUTER)");
  Serial.println("  SENSOR : MPU-6050 (6-AXIS SENSOR FUSION)");
  Serial.println("  DURASI : 1 MENIT (60 DETIK)");
  Serial.println("  TIMESTAMP: JAM NYATA WIB via NTP (HH:MM:SS.mmm)");
  Serial.println("=======================================================");

  for (int i=0; i<16; i++) pendingList[i].pending = false;

  initMPU6050();

  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

  Serial.printf("[Wi-Fi] Menghubungkan ke Router '%s'", WIFI_SSID);
  while (WiFi.status() != WL_CONNECTED) {
    delay(300); Serial.print(".");
  }
  Serial.println("\n[Wi-Fi] Berhasil terhubung ke Router!");
  Serial.printf("[Wi-Fi] IP ESP32: %s | RSSI: %d dBm\n", WiFi.localIP().toString().c_str(), WiFi.RSSI());

  // NTP Sync
  Serial.print("[NTP] Sinkronisasi jam...");
  configTime(TZ_OFFSET_WIB, 0, NTP_SERVER1, NTP_SERVER2);
  struct tm tmCheck;
  uint32_t ntpWait = millis();
  while (!getLocalTime(&tmCheck) && millis()-ntpWait < 10000) {
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

    // Cek sinyal START dari aplikasi laptop
    if (packetSize >= 5 && memcmp(rxBuf, "START", 5) == 0) {
      receiverIP = udp.remoteIP();
      receiverPort = UDP_REMOTE_PORT;
      receiverLocked = true;
      seqNumber = 1;
      sampleCount = 0;
      sessionStartMs = millis();
      sessionActive = true;
      for (int i=0; i<16; i++) pendingList[i].pending = false;
      Serial.println("[UDP] Menerima sinyal START! Sesi baru dimulai dari paket #1.");
      return;
    }

    receiverIP = udp.remoteIP();
    receiverPort = udp.remotePort();
    receiverLocked = true;

    uint32_t echoSeq = 0;
    memcpy(&echoSeq, rxBuf, sizeof(uint32_t));

    for (int i=0; i<16; i++) {
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
        Serial.printf("%u,%s,%u,%s,MPU6050,WIFI_STATIC,WIFI,%.2f,%.2f,%.2f,0,%d\n",
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
  for (int i=0; i<16; i++) {
    if (pendingList[i].pending && (nowMs - pendingList[i].sentMs > TIMEOUT_RTT_MS)) {
      pendingList[i].pending = false;
      sampleCount++;
      char tsStr[16];
      formatTimestamp(pendingList[i].sentMs, tsStr, sizeof(tsStr));
      Serial.printf("%u,%s,%u,%s,MPU6050,WIFI_STATIC,WIFI,0.00,0.00,0.00,1,%d\n",
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

    MotionPacket pkt = {0};
    readMPU6050(pkt);
    pkt.sequenceNumber  = seqNumber++;
    pkt.sendTimestampMs = nowMs;
    pkt.buttonState     = 0;
    pkt.checksum        = calcChecksum(pkt);

    int16_t rssi = WiFi.RSSI();

    udp.beginPacket(receiverIP, receiverPort);
    udp.write((const uint8_t*)&pkt, sizeof(MotionPacket));
    udp.endPacket();

    for (int i=0; i<16; i++) {
      if (!pendingList[i].pending) {
        pendingList[i] = { pkt.sequenceNumber, pkt.sendTimestampMs, rssi, true };
        break;
      }
    }
  }
}
