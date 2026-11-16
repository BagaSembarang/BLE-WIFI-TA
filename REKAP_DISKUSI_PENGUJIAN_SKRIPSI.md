# REKAPITULASI DISKUSI TEKNIS & AKADEMIS SKRIPSI
**Topik:** Otomasi Komunikasi Adaptif BLE dan Wi-Fi pada Game Controller ESP32  
**Fokus Diskusi:** Analisis Nilai QoS Rata, Pengiriman Batch 100 Paket, Karakteristik Game Controller, dan Perbandingan Sensor

---

## DAFTAR ISI
1. [Bagian 1: Analisis Pengiriman Data Banyak Sekaligus (Batch/Burst 100 Paket)](#bagian-1-analisis-pengiriman-data-banyak-sekaligus-batchburst-100-paket)
2. [Bagian 2: Mengapa Data Pengujian QoS Selama Ini Cenderung Selalu Rata?](#bagian-2-mengapa-data-pengujian-qos-selama-ini-cenderung-selalu-rata)
3. [Bagian 3: Apakah Logika Dosen "Kirim 100 Sekaligus" Masuk Akal?](#bagian-3-apakah-logika-dosen-kirim-100-sekaligus-masuk-akal)
4. [Bagian 4: Apakah Konsep Kontroler Game Memang Boleh Menahan 100 Data?](#bagian-4-apakah-konsep-kontroler-game-memang-boleh-menahan-100-data)
5. [Bagian 5: Mengapa Setiap Sensor (No Sensor, BNO055, MPU-6050, LSM6DS3, ICM-20948) Menghasilkan Data QoS yang Sama?](#bagian-5-mengapa-setiap-sensor-menghasilkan-data-qos-yang-sama)
6. [Bagian 6: Panduan Penulisan Bab 4 (Hasil & Pembahasan) dan Bahan Sidang Skripsi](#bagian-6-panduan-penulisan-bab-4-hasil--pembahasan-dan-bahan-sidang-skripsi)

---

## BAGIAN 1: Analisis Pengiriman Data Banyak Sekaligus (Batch/Burst 100 Paket) - tambahan biar realistis aja ttp dicoba

### 1.1 Batasan Fisik Protokol BLE (ATT MTU)
* Pada sistem skripsi ini, struktur data per sampel didefinisikan secara baku sebesar **52 byte** (`struct MotionPacket`).
* Protokol Bluetooth Low Energy (BLE 4.2/5.0) memiliki batas kapasitas transmisi per notifikasi (*ATT MTU*) maksimal sekitar **512 byte**.
* Jika 100 sampel dipaksa digabung ke dalam **satu paket notifikasi fisik**, total ukurannya:
  $$\text{100 paket} \times 52\text{ byte} = \mathbf{5.200\text{ byte (5,2 KB)}}$$
  Ukuran ini **melampaui kapasitas maksimal MTU BLE**, sehingga secara arsitektur protokol BLE mustahil dikirim dalam satu notifikasi tunggal.

### 1.2 Solusi yang Diimplementasikan: Mode Burst Transmission
Untuk memenuhi skenario pengiriman banyak data tanpa merusak protokol BLE dan tetap kompatibel dengan aplikasi penerima Python (`app_penerima_gui.py`):
1. **Format Paket Tetap:** Setiap paket tetap berukuran 52 byte (format struct tidak diubah).
2. **Loop Burst Cepat:** Mikrokontroler menembakkan 100 paket berturut-turut dalam satu siklus pengiriman.
3. **Inter-Packet Delay (3–5 ms):** Diberikan jeda mikro agar antrean buffer internal GATT BLE ESP32 dan stack Bluetooth Windows tidak mengalami *buffer overflow/congestion drop*.
4. **Peningkatan Kapasitas Pelacak RTT:** Kapasitas array `pendingList` diperbesar dari 16 menjadi **150 slot** agar seluruh 100 paket yang ditembakkan dapat dicatat waktu RTT, Latensi, Jitter, dan Loss-nya.

> **File Implementasi:**  
> Telah dibuat file Arduino: `arduino/14_ble_nosensor_burst100/14_ble_nosensor_burst100.ino`

---

## BAGIAN 2: Mengapa Data Pengujian QoS Selama Ini Cenderung Selalu Rata?

Saat Anda melakukan pengujian (Percobaan 1 hingga 5 pada kondisi normal 50 Hz):
* **Rata-rata RTT:** Selalu berkisar di $\approx 18 - 20\text{ ms}$
* **Rata-rata Latensi:** Selalu berkisar di $\approx 9 - 10\text{ ms}$
* **Jitter:** Sangat kecil ($< 1\text{ ms}$)
* **Packet Loss:** Tetap $0\%$

### Penyebab Ilmiah:
1. **Beban Trafik Sangat Ringan (Idle Traffic):**  
   Pada frekuensi 50 Hz dengan paket 52 byte, bandwidth yang terpakai hanya:
   $$\text{Bandwidth} = 52\text{ byte} \times 8\text{ bit} \times 50\text{ Hz} = 20.800\text{ bps} \approx \mathbf{20{,}8\text{ kbps}}$$
   Kapasitas link BLE ESP32 adalah **1.000 kbps (1 Mbps)**. Beban trafik jaringan yang Anda uji hanya memakan $\approx 2\%$ dari kapasitas link.
2. **Tidak Ada Antrean Buffer (Queuing Delay = 0):**  
   Karena saluran sangat longgar, paket yang masuk ke radio langsung dikirim saat itu juga tanpa pernah mengantre di buffer.
3. **Murni Mencerminkan Hardware Connection Interval:**  
   Nilai ~9–10 ms tersebut adalah konstanta waktu sinkronisasi koneksi BLE (*Connection Interval*) bawaan stack BLE antara ESP32 dan chip Bluetooth laptop. Diulang berapa kali pun dalam kondisi ideal di atas meja, nilainya pasti akan konsisten.

---

## BAGIAN 3: Apakah Logika Dosen "Kirim 100 Sekaligus" Masuk Akal?

### Jawabannya: **YA, SANGAT MASUK AKAL, jika diposisikan sebagai "Stress Testing" (Uji Beban Ekstrem).**

Dalam penelitian jaringan komputer dan QoS:
* Jika jaringan hanya diuji dalam kondisi santai (20 kbps), peneliti tidak akan pernah mengetahui batas ketahanan sistem.
* Dosen menyarankan pengiriman 100 paket sekaligus dengan tujuan:
  1. **Memaksa buffer radio penuh:** Mengetahui kapasitas antrean stack BLE.
  2. **Melihat Queuing Delay:** Membuktikan bahwa saat antrean menumpuk, latensi akan melonjak (misal dari 10 ms naik ke 50–100 ms).
  3. **Memicu Packet Loss & Jitter:** Melihat apakah ada paket yang dibuang saat transmisi padat.
* **Kesimpulan:** Sebagai skenario **Stress Test**, logika dosen sangat valid secara ilmiah untuk memperkaya variasi grafik di Bab 4.

---

## BAGIAN 4: Apakah Konsep Kontroler Game Memang Boleh Menahan 100 Data?

### Jawabannya: **TIDAK. Konsep kontroler game TIDAK BOLEH menahan 100 data.**

Ada dikotomi mendasar dalam arsitektur sistem IoT:

| Karakteristik | IoT Pemantauan / Data Logger (Monitoring) | IoT HMI / Game Controller (Skripsi Anda) |
| :--- | :--- | :--- |
| **Contoh Kasus** | Sensor suhu cuaca, kelembapan tanah, smart meter | Gamepad ESP32, remote drone, controller VR |
| **Metode Kirim** | **Batching (Kumpul dulu baru kirim):** Simpan 100 data di RAM, kirim sekaligus per 1 jam. | **Streaming Real-Time:** 1 data sensor terbaca, langsung ditembak saat itu juga. |
| **Toleransi Delay** | Sangat longgar (delay 10 detik – 1 jam tidak masalah). | **Nol Toleransi (< 30 ms).** Delay di atas 50 ms sudah terasa mengganggu (*lag*). |
| **Konsekuensi jika tahan 100 data** | Hemat baterai, sangat dianjurkan. | **Terjadi lag 2 detik (2.000 ms).** Kontroler menjadi rusak secara fungsional (*unplayable*). |

> **Perhitungan Delay Akumulasi:**  
> Pada sampling 50 Hz (1 paket tiap 20 ms), untuk mengumpulkan 100 data dibutuhkan waktu:  
> $$100 \times 20\text{ ms} = \mathbf{2.000\text{ ms (2 Detik)}}$$  
> Pemain menggerakkan stik ke kiri, tetapi karakter di layar baru merespons 2 detik kemudian. Ini melanggar prinsip utama sistem kontrol gerak interaktif.

---

## BAGIAN 5: Mengapa Setiap Sensor Menghasilkan Data QoS yang Sama?

Pertanyaan: *"Kenapa No Sensor, BNO055, MPU-6050, LSM6DS3, dan ICM-20948 data Latensi dan RTT-nya sama persis?"*

### Penyebab 1: Metrik yang Diukur Adalah Metrik Lapisan Jaringan Radio (QoS)
* Parameter RTT, Latensi, Jitter, Packet Loss, dan RSSI mengukur performa **gelombang radio 2.4 GHz di udara**, bukan performa sensor.
* Bagi antena radio:
  - Ukuran paket **identik (52 byte)** di semua kombinasi sensor.
  - Interval pengiriman **identik (50 Hz / 20 ms)**.
  - Jarak dan lingkungan **identik (di atas meja pengujian)**.
* Antena tidak membedakan isi muatan (apakah data berasal dari MPU6050 atau BNO055). Selama ukuran paketnya 52 byte, waktu rambat gelombang elektromagnetik bolak-balik akan selalu sama.

### Penyebab 2: Waktu Pembacaan Sensor Tertelan oleh Interval Timer 20 ms
Di dalam kode Arduino:
```cpp
if (nowMs - lastTxMs >= 20) {     // Timer interval 20 ms
    lastTxMs = nowMs;
    readSensor(pkt);              // Pembacaan sensor via I2C (hanya 0.5 - 2 ms)
    pkt.sendTimestampMs = nowMs;  // Timestamp dicatat SETELAH baca sensor!
    pTxChar->notify();            // Tembak ke BLE
}
```
1. Waktu pembacaan bus I2C sensor fisik hanya berlangsung **0,5 s/d 2,5 milidetik**. Waktu ini jauh lebih kecil daripada jeda timer 20 ms, sehingga mikrokontroler selalu selesai membaca sensor jauh sebelum jadwal pengiriman tiba.
2. Timestamp transmisi (`sendTimestampMs`) dicatat sesaat sebelum transmisi radio dimulai, sehingga durasi komputasi sensor tidak terakumulasi ke dalam perhitungan RTT jaringan.

### Lalu Apa yang Membedakan Antar Sensor?
Perbedaan antar sensor terletak pada **Kualitas Data Fisik**, bukan pada QoS Jaringan:
* **BNO055:** Memiliki prosesor internal (*Digital Motion Processor / DMP*). Nilai rotasi/Quaternion sangat akurat, stabil, dan bebas *drift* (melayang).
* **MPU-6050 / LSM6DS3:** Sensor 6-axis standar tanpa sensor fusi kompas internal, nilai sudut akan mengalami *gyro drift* perlahan seiring waktu.
* **No Sensor:** Nilai koordinat statis/dummy sebagai kontrol baseline murni beban komunikasi.

---

## BAGIAN 6: Panduan Penulisan Bab 4 (Hasil & Pembahasan) dan Bahan Sidang Skripsi

### 6.1 Kalimat Narasi Ilmiah untuk Bab 4
Gunakan narasi berikut untuk menjelaskan temuan penelitian Anda:

> *"Berdasarkan hasil pengujian pada Kombinasi BLE dengan variasi sensor (No Sensor, BNO055, MPU-6050, ICM-20948, dan LSM6DS3), didapatkan nilai rata-rata RTT berkisar antara 18,4 ms – 19,8 ms, Latensi berkisar antara 9,2 ms – 9,9 ms, dan Packet Loss 0% yang relatif seragam.*
> 
> *Secara empiris, hal ini membuktikan bahwa **jenis sensor inersia yang digunakan tidak memberikan dampak signifikan terhadap performa Quality of Service (QoS) lapisan transmisi radio**, selama ukuran muatan paket (payload size) dijaga tetap seragam (52 byte) dan waktu akuisisi pembacaan sensor pada bus I2C (< 3 ms) tidak melampaui batas periode sampling transmisi (20 ms / 50 Hz).*
> 
> *Diferensiasi performa antar sensor terletak pada **akurasi dan stabilitas nilai orientasi gerak (Quaternion dan Akselerasi)**, bukan pada kecepatan transmisi paket data di udara."*

### 6.2 Cara Menyikapi Saran Dosen Pembimbing
Jika berdiskusi dengan dosen terkait data yang rata:
1. **Apresiasi saran dosen:** Jelaskan bahwa pengiriman banyak data (batch 100) telah disiapkan sebagai **Uji Beban Ekstrem (Stress Test)** menggunakan sketsa `14_ble_nosensor_burst100.ino`.
2. **Jelaskan batasan kontroler game:** Jelaskan secara proporsional bahwa untuk operasional normal game controller, pengiriman harus tetap dilakukan secara *real-time stream* (1 per 1) agar tidak menimbulkan *input lag* 2 detik.
3. **Tawarkan variasi pengujian alami:** Jika ingin data pengujian normal lebih dinamis di Bab 4, variasikan **jarak pengujian (1 meter, 5 meter, 10 meter terhalang dinding)** atau **interferensi sinyal radio**, karena pada kondisi tersebut nilai RSSI, Jitter, dan Packet Loss akan berfluktuasi secara alami sesuai skenario riil dunia nyata.


## BAGIAN 1: Analisis Pengiriman Data Banyak Sekaligus (Batch/Burst 100 Paket) - tambahan biar realistis aja ttp dicoba

note : cari jurnal dimana dia dianggap delay nya kebangetan (Bluetooth) n kondisi seperti apa wifi dianggap parah













Fase 5: Ambil data skripsi

Semua perintah dijalankan dari folder host. Ganti wifi dengan ethernet kalau pakai kabel.

S1, diam, empat jarak (sekitar 70 menit):

python eksperimen.py --mode semua --ulang 5 --durasi 60 --skenario S1_0.5m --jarak "0.5 m" --catatan "diam, sepi" --laptop-koneksi wifi --seed 11 --tanpa-jeda
python eksperimen.py --mode semua --ulang 5 --durasi 60 --skenario S1_1m --jarak "1 m" --catatan "diam, sepi" --laptop-koneksi wifi --seed 12 --tanpa-jeda
python eksperimen.py --mode semua --ulang 5 --durasi 60 --skenario S1_2m --jarak "2 m" --catatan "diam, sepi" --laptop-koneksi wifi --seed 13 --tanpa-jeda
python eksperimen.py --mode semua --ulang 5 --durasi 60 --skenario S1_3m --jarak "3 m" --catatan "diam, sepi" --laptop-koneksi wifi --seed 14 --tanpa-jeda

Setelah S1 selesai, kirim hasilnya ke saya untuk cek kalibrasi (adaptif tidak boleh sering pindah tanpa gangguan). Setelah itu parameter dibekukan.

S2, bergerak EX-1 dengan metronom 60 bpm:

	

(Tanpa --tanpa-jeda, supaya Anda siap bergerak sebelum menekan Enter.)

S3, interferensi dua level (gangguan menyala sepanjang semua sesi level itu):

python eksperimen.py --mode semua --ulang 5 --durasi 60 --skenario S3_L1 --jarak "1.5 m" --catatan "L1: 1 HP streaming via router" --laptop-koneksi wifi --seed 31
python eksperimen.py --mode semua --ulang 5 --durasi 60 --skenario S3_L2 --jarak "1.5 m" --catatan "L2: 3 HP streaming + unduhan besar" --laptop-koneksi wifi --seed 32 --tanpa-jeda

S4, gangguan mendadak:

python eksperimen.py --mode semua --ulang 5 --durasi 60 --skenario S4 --jarak "1.5 m" --catatan "BT laptop mati detik 20-35" --laptop-koneksi wifi --seed 40 --isyarat-bt 20,35 	

S5, lingkungan padat (lab atau ruang publik saat ramai):

python eksperimen.py --mode semua --ulang 5 --durasi 60 --skenario S5 --jarak "1.5 m" --catatan "lab jam sibuk" --laptop-koneksi wifi --seed 50

Analisis sensitivitas (adaptif saja, di S4):

python eksperimen.py --mode adaptif --ulang 5 --durasi 60 --skenario S4 --skor-hanya-latency --varian latency_saja --catatan "BT laptop mati detik 20-35" --laptop-koneksi wifi --seed 41 --isyarat-bt 20,35
python eksperimen.py --mode adaptif --ulang 5 --durasi 60 --skenario S4 --ambang-ketat --varian ambang_ketat --catatan "BT laptop mati detik 20-35" --laptop-koneksi wifi --seed 42 --isyarat-bt 20,35
Fase 6: Setelah setiap skenario
Buka dashboard (streamlit run dashboard.py), cek tabel kelengkapan. Kalau ada sesi tidak valid, jalankan ulang hanya metode yang kurang (--mode ble --ulang 1, misalnya).
Salin folder data ke Google Drive.
Commit dan push (folder skenario skripsi otomatis ikut, folder UJI tidak).
Satu skenario sebaiknya selesai dalam satu sesi kerja yang sama.
Fase 7: Analisis dan Bab 4
python analisis.py data
streamlit run dashboard.py

Lalu Bab 4 disusun dengan urutan yang sudah kita bahas: pelaksanaan pengujian, uji fungsional, karakterisasi BLE dan Wi-Fi, perbandingan tiga metode per skenario, pemicu dan durasi perpindahan, contoh linimasa S4, sensitivitas, keterbatasan.

Mulai dari Fase 1. Kalau mau saya yang push perubahan kodenya, bilang saja, nanti Anda tinggal git pull lalu langsung ke Fase 2.



Rencana: 15 percobaan, durasi 60 dtk, skenario 'S1_0.5m', urutan acak (seed 11).
Perkiraan waktu total tanpa jeda: 17 menit.

  BLE baru tersambung, menunggu 6 dtk agar koneksi stabil ...
[1/15] BLE ulangan 1 berjalan 60 dtk ...
  OK : paket 3000 | loss 0.00% (tak sampai laptop 0.00%) | RTT median 51.9 ms, p95 88.7 ms
[2/15] ADAPTIF ulangan 1 berjalan 60 dtk ...
  OK : paket 3000 | loss 0.00% (tak sampai laptop 0.00%) | RTT median 48.7 ms, p95 84.1 ms | pindah 0x
[3/15] WIFI ulangan 1 berjalan 60 dtk ...
  OK : paket 3000 | loss 0.00% (tak sampai laptop 0.00%) | RTT median 16.2 ms, p95 59.9 ms
[4/15] ADAPTIF ulangan 2 berjalan 60 dtk ...
  OK : paket 3000 | loss 0.00% (tak sampai laptop 0.00%) | RTT median 49.3 ms, p95 85.3 ms | pindah 0x
[5/15] BLE ulangan 2 berjalan 60 dtk ...
  OK : paket 3000 | loss 0.00% (tak sampai laptop 0.00%) | RTT median 49.5 ms, p95 84.2 ms
[6/15] WIFI ulangan 2 berjalan 60 dtk ...
  OK : paket 3000 | loss 0.13% (tak sampai laptop 0.00%) | RTT median 21.0 ms, p95 64.1 ms
[7/15] ADAPTIF ulangan 3 berjalan 60 dtk ...
  OK : paket 3000 | loss 0.00% (tak sampai laptop 0.00%) | RTT median 51.2 ms, p95 86.9 ms | pindah 0x
[8/15] WIFI ulangan 3 berjalan 60 dtk ...
  OK : paket 3000 | loss 0.10% (tak sampai laptop 0.00%) | RTT median 17.0 ms, p95 60.5 ms
[9/15] BLE ulangan 3 berjalan 60 dtk ...
  OK : paket 3000 | loss 0.00% (tak sampai laptop 0.00%) | RTT median 51.7 ms, p95 87.4 ms
[10/15] WIFI ulangan 4 berjalan 60 dtk ...
  OK : paket 3000 | loss 0.10% (tak sampai laptop 0.00%) | RTT median 21.8 ms, p95 64.2 ms
[11/15] BLE ulangan 4 berjalan 60 dtk ...
  OK : paket 3000 | loss 0.00% (tak sampai laptop 0.00%) | RTT median 51.1 ms, p95 87.0 ms
[12/15] ADAPTIF ulangan 4 berjalan 60 dtk ...
  OK : paket 3000 | loss 0.00% (tak sampai laptop 0.00%) | RTT median 48.8 ms, p95 93.0 ms | pindah 0x
[13/15] ADAPTIF ulangan 5 berjalan 60 dtk ...
  OK : paket 3000 | loss 2.93% (tak sampai laptop 0.00%) | RTT median 24.6 ms, p95 71.1 ms | pindah 6x
[14/15] WIFI ulangan 5 berjalan 60 dtk ...
  OK : paket 3000 | loss 0.20% (tak sampai laptop 0.00%) | RTT median 18.8 ms, p95 61.4 ms
[15/15] BLE ulangan 5 berjalan 60 dtk ...
  OK : paket 3000 | loss 2.43% (tak sampai laptop 0.00%) | RTT median 49.6 ms, p95 85.9 ms

Selesai. 15/15 percobaan valid. Data: D:\Skripsi\sistem\gamepad_ta\host\data\S1_0.5m
Langkah berikut:  python analisis.py data\S1_0.5m
  [BLE terputus, mencoba menyambung ulang di latar]