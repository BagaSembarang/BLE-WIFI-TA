# PANDUAN PRAKTIS PENGUJIAN SKRIPSI (METODE 5X PERCOBAAN)

Panduan ini mengatur alur pengujian skripsi di mana **setiap kombinasi diuji sebanyak 5 kali percobaan (masing-masing 1000 titik data)**, kemudian sistem otomatis menghitung **nilai rata-rata akhir (Grand Mean)** untuk naskah skripsi Bab 4 Anda.

---

## 1. Daftar Kode Arduino per Kombinasi (Folder `arduino/`)

Buka folder kombinasi yang ingin diuji, lalu *double click* file `.ino` untuk membukanya di Arduino IDE:

| No | Folder Proyek | Metode Komunikasi | Sensor IMU | Deskripsi |
|---|---|---|---|---|
| **1** | [`arduino/1_ble_nosensor/1_ble_nosensor.ino`](file:///d:/Skripsi/sistem/arduino/1_ble_nosensor/1_ble_nosensor.ino) | BLE Statis | No Sensor | Baseline kontrol tanpa sensor fisik |
| **2** | [`arduino/2_ble_bno055/2_ble_bno055.ino`](file:///d:/Skripsi/sistem/arduino/2_ble_bno055/2_ble_bno055.ino) | BLE Statis | BNO055 | Full 9-Axis Sensor Fusion via I2C |
| **3** | [`arduino/3_wifi_nosensor/3_wifi_nosensor.ino`](file:///d:/Skripsi/sistem/arduino/3_wifi_nosensor/3_wifi_nosensor.ino) | Wi-Fi UDP | No Sensor | Baseline kontrol via router Wi-Fi |
| **4** | [`arduino/4_wifi_bno055/4_wifi_bno055.ino`](file:///d:/Skripsi/sistem/arduino/4_wifi_bno055/4_wifi_bno055.ino) | Wi-Fi UDP | BNO055 | Transmisi data BNO055 via router Wi-Fi |
| **5** | [`arduino/5_adaptive_nosensor/5_adaptive_nosensor.ino`](file:///d:/Skripsi/sistem/arduino/5_adaptive_nosensor/5_adaptive_nosensor.ino) | Adaptive Switching | No Sensor | Otomasi perpindahan BLE <-> Wi-Fi |
| **6** | [`arduino/6_adaptive_bno055/6_adaptive_bno055.ino`](file:///d:/Skripsi/sistem/arduino/6_adaptive_bno055/6_adaptive_bno055.ino) | Adaptive Switching | BNO055 | Otomasi perpindahan dengan sensor BNO055 |
| **7** | [`arduino/7_wifi_icm20948/7_wifi_icm20948.ino`](file:///d:/Skripsi/sistem/arduino/7_wifi_icm20948/7_wifi_icm20948.ino) | Wi-Fi UDP | ICM-20948 | Transmisi 9-DOF Madgwick via router Wi-Fi |
| **8** | [`arduino/8_ble_icm20948/8_ble_icm20948.ino`](file:///d:/Skripsi/sistem/arduino/8_ble_icm20948/8_ble_icm20948.ino) | BLE Statis | ICM-20948 | Transmisi 9-DOF Madgwick via BLE |
| **9** | [`arduino/9_ble_mpu6050/9_ble_mpu6050.ino`](file:///d:/Skripsi/sistem/arduino/9_ble_mpu6050/9_ble_mpu6050.ino) | BLE Statis | MPU-6050 | Transmisi 6-DOF Madgwick via BLE |
| **10** | [`arduino/10_wifi_mpu6050/10_wifi_mpu6050.ino`](file:///d:/Skripsi/sistem/arduino/10_wifi_mpu6050/10_wifi_mpu6050.ino) | Wi-Fi UDP | MPU-6050 | Transmisi 6-DOF Madgwick via router Wi-Fi |
| **11** | [`arduino/11_ble_lsm6ds3/11_ble_lsm6ds3.ino`](file:///d:/Skripsi/sistem/arduino/11_ble_lsm6ds3/11_ble_lsm6ds3.ino) | BLE Statis | LSM6DS3 | Transmisi 6-DOF Madgwick via BLE |
| **12** | [`arduino/12_wifi_lsm6ds3/12_wifi_lsm6ds3.ino`](file:///d:/Skripsi/sistem/arduino/12_wifi_lsm6ds3/12_wifi_lsm6ds3.ino) | Wi-Fi UDP | LSM6DS3 | Transmisi 6-DOF Madgwick via router Wi-Fi |
| **13** | [`arduino/13_adaptive_lsm6ds3/13_adaptive_lsm6ds3.ino`](file:///d:/Skripsi/sistem/arduino/13_adaptive_lsm6ds3/13_adaptive_lsm6ds3.ino) | Adaptive Switching | LSM6DS3 | Otomasi perpindahan BLE <-> Wi-Fi + LSM6DS3 |

> **Skema Pengkabelan (Wiring) LSM6DS3 ke ESP32:**
> - **VCC** -> 3.3V
> - **GND** -> GND
> - **SDA** -> GPIO 2 (ESP32-C3) / GPIO 8 (ESP32-S3)
> - **SCL** -> GPIO 3 (ESP32-C3) / GPIO 9 (ESP32-S3)
> - **CS**  -> 3.3V (atau biarkan jika modul memiliki pull-up otomatis untuk mode I2C)
> - **SA0 / SDO** -> GND (Alamat 0x6A) atau 3.3V (Alamat 0x6B)

> **Ganti Board MCU (ESP32-S3 atau ESP32-C3):**  
> Pada baris header di file `.ino`, tentukan board yang digunakan:
> ```cpp
> #define TARGET_MCU MCU_ESP32_C3  // atau MCU_ESP32_S3
> ```

---

## 2. Cara Menjalankan Pengujian 5x Percobaan di Laptop

1. Di folder `sistem_penerima/`, *double click* file:
   ```text
   jalankan_aplikasi.bat
   ```
2. Pada aplikasi desktop yang muncul:
   - Pilih **Kombinasi yang Diuji** (misal: *Kombinasi 1: BLE Statis + NO_SENSOR*).
   - Di sampingnya tertulis **Percobaan Ke: [ 1 ] dari 5**.
   - Klik tombol hijau **"▶ Mulai Percobaan (1000 Data)"**.
3. Aplikasi akan merekam data hingga 1000 titik data, lalu **berhenti otomatis** dan menyimpan file:
   ```text
   Kombinasi_1_BLE_NO_SENSOR_Percobaan_1_YYYYMMDD_HHMMSS.csv
   ```
4. Angka percobaan otomatis naik menjadi **Percobaan Ke: [ 2 ] dari 5**.
5. Ulangi klik tombol **"Mulai Percobaan"** hingga selesai 5 kali percobaan (total 5.000 titik data).

---

## 3. Melihat Hasil Akhir Rata-rata 5x Percobaan di Website

Setelah 5 percobaan selesai:

1. Di folder utama `D:\Skripsi\sistem\`, *double click* file:
   ```text
   buka_dashboard.bat
   ```
2. Browser akan terbuka di `http://localhost:8080`.
3. Di tab **"Rekapitulasi Rata-rata 5x Percobaan"**:
   - Pilih **Kombinasi** yang baru saja Anda uji.
   - Website akan otomatis menyusun **Tabel 5x Percobaan**:
     - *Percobaan 1*: Rata-rata Latency, Jitter, Packet Loss, RSSI.
     - *Percobaan 2*: Rata-rata Latency, Jitter, Packet Loss, RSSI.
     - *Percobaan 3*: Rata-rata Latency, Jitter, Packet Loss, RSSI.
     - *Percobaan 4*: Rata-rata Latency, Jitter, Packet Loss, RSSI.
     - *Percobaan 5*: Rata-rata Latency, Jitter, Packet Loss, RSSI.
     - **BARIS HASIL AKHIR (RATA-RATA TOTAL / GRAND MEAN)**: Angka representatif akhir untuk skripsi Anda.
4. Klik tombol **"📋 Salin Tabel untuk Word"** untuk langsung menempelkan (*paste*) tabel rapi tersebut ke Bab 4 Skripsi Anda.
