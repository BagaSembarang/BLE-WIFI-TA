# Sistem Pengujian Skripsi: Adaptive Switching BLE <-> Wi-Fi untuk Pengontrol Exergaming (ESP32-C3 + BNO055, penerima laptop)

Studi kasus: exergaming di laptop. Pemain bergerak, pengontrol (ESP32-C3 + BNO055) mengirim data gerakan ke laptop
lewat BLE atau Wi-Fi (melalui router, tanpa internet). Penelitian melihat **latency yang terjadi**, **cara kerja adaptive
switching**, dan **kondisi yang memicu perpindahan**.

Semua angka berasal dari satu jam, yaitu jam ESP32. Tidak ada nilai pengganti dan tidak ada angka bawaan di kode.

```
firmware/gamepad/gamepad.ino        firmware ESP32-C3 (satu firmware untuk semua metode)
firmware/gamepad/adaptive_core.h    mesin keputusan adaptif (juga diuji di PC)
firmware/gamepad/secrets.h          SSID dan sandi router (EDIT; jangan di-commit)
firmware/test/                      uji unit mesin keputusan (./run_tests.sh, 10 skenario)
host/eksperimen.py                  menjalankan percobaan dan menyimpan data mentah
host/analisis.py                    tabel, uji statistik, tabel pemicu switching, grafik
host/simulator_esp32.py             ESP32 palsu untuk uji skrip (BUKAN sumber data skripsi)
```

## 1. Cara kerja singkat

1. Laptop terhubung ke ESP32 lewat **BLE**, lewat **Wi-Fi** (ESP32 dan laptop sama-sama di router), dan lewat
   **kanal kendali** untuk perintah START dan pengambilan log: kabel USB (uji bertahap) atau TCP lewat Wi-Fi
   (pengambilan data, ESP32 diberi daya dari adaptor tanpa kabel ke laptop).
2. ESP32 mengirim paket gerakan 32 byte, 50 kali per detik, lewat link yang dipilih metode: **BLE statis**,
   **Wi-Fi statis**, atau **Adaptif**.
3. Laptop langsung mengembalikan setiap paket (echo). ESP32 menghitung **RTT** = waktu echo kembali - waktu kirim,
   dari jam ESP32 yang sama. Tidak ada sinkronisasi jam antar perangkat.
4. Link yang tidak membawa data tetap diukur lewat **probe** 20 Hz (di semua metode). Beban radio jadi sama antar
   metode, dan mesin adaptif punya bukti kualitas link cadangan sebelum berpindah.
5. Selama sesi ESP32 **tidak mencetak apa pun** ke serial. Catatan disimpan di RAM lalu dikirim setelah sesi.
6. Setiap perpindahan adaptif mencatat **pemicunya** (latency, loss, jitter, RSSI, atau putus) beserta nilai
   indikator saat keputusan, supaya "kondisi apa yang memicu switching" bisa dijawab dari data.

### Definisi metrik (untuk Bab 3.5)

| Metrik | Definisi |
|---|---|
| RTT | selisih waktu echo diterima dan paket dikirim, jam ESP32 (mikrodetik) |
| Latency satu arah (estimasi) | RTT / 2 (kolom `oneway_est_*` memakai RTT dikurangi waktu proses laptop, lalu dibagi 2) |
| Packet loss | paket DATA tanpa echo dalam 1 detik, dibagi total paket DATA |
| lambat100 / lambat150 | persentase paket dengan latency satu arah > 100 ms / > 150 ms |
| buruk150 | loss + lambat150 (paket yang tidak berguna bagi permainan menurut anggaran latency) |
| Jitter | rata-rata selisih mutlak RTT antar paket berurutan di link yang sama |
| Jeda masukan terpanjang | selisih waktu kirim terbesar antara dua paket "berguna" (latency satu arah <= 150 ms) |
| Waktu handover | dari keputusan pindah sampai paket DATA pertama kembali lewat link baru |
| Pemicu | indikator dengan kontribusi penalti terbesar pada link asal saat keputusan; `putus` untuk jalur darurat |

## 2. Dasar ambang switching (dari literatur)

Latency adalah indikator **utama**. Titik pindah karena latency ditetapkan pada **latency satu arah p90 sekitar 130 ms
(RTT sekitar 260 ms)**. Angka ini **turunan** dari anggaran latency exergame, bukan angka tunggal dari satu jurnal:

| Langkah | Nilai | Dasar | Sifat |
|---|---|---|---|
| Titik balik performa exergame | ~400 ms (407 ms) | Durnez dkk., IEEE Trans. Games 2022 (37 partisipan, exergame desktop) | Temuan jurnal |
| Anggaran latency total (gerak sampai tampil) | <= 300 ms | Durnez dkk. 2022 | Temuan jurnal |
| Dampak minimal | di bawah ~200 ms | Durnez dkk. 2022 | Temuan jurnal |
| Latency lokal layar, OS, game di laptop | 21-102 ms | Raaen dan Petlund 2015 (MMSys), dikutip Durnez dkk. | Dikutip, konferensi |
| Sensor dan jadwal kirim | ~30 ms | IMU <= 10 ms + kirim 50 Hz <= 20 ms | Rancangan, perlu diukur |
| Komponen non-link | ~130 ms | 102 + 30 | Turunan |
| Jatah link | 170 ms satu arah | 300 - 130 | Turunan |
| **Batas buruk (pindah)** | **130 ms satu arah (RTT ~260 ms)** | 170 - margin 40 | Turunan |
| Batas baik (tanpa penalti) | 70 ms satu arah (RTT 140 ms) | 200 - 130 | Turunan |
| Jitter | tidak fatal sampai ~50 ms | Schmid dkk., PACM HCI 2023 | Temuan jurnal |
| Histeresis dan dwell time | latency konstan diprioritaskan | Halbhuber dkk., PACM HCI 2022: perubahan level latency merusak flow | Temuan jurnal |

Di kode: normalisasi latency dari RTT p90 140 ms (penalti 0) sampai 300 ms (penalti penuh); dengan bobot dan
`t_leave = 65`, titik pindah karena latency saja jatuh pada RTT p90 ~265 ms (satu arah ~132 ms). Loss di atas ~13%
(tanpa echo dalam 300 ms) atau link aktif diam 250 ms juga memicu perpindahan.

**Mengapa bukan 75-100 ms?** Studi avatar/pelacakan halus melaporkan penurunan mulai ~75 ms (Waltemate dkk. 2016,
konferensi) dan 80-120 ms tergantung kecepatan gerak (Hoyet dkk. 2019, konferensi). Pada laptop, komponen non-link
saja sudah 21-102 ms, jadi memindahkan jalur tidak bisa memenuhi ambang itu. Exergame bergerak kasar lebih cocok
dengan Durnez dkk. Ambang ketat tetap diuji sebagai **analisis sensitivitas**: `--ambang-ketat` (RTT baik 100 ms,
buruk 200 ms), muncul sebagai kelompok `adaptif_ambang_ketat`.

Yang belum punya dasar jurnal langsung (tulis jujur di laporan):
1. Tidak ditemukan jurnal yang menetapkan "pindah pada X ms" untuk BLE dan Wi-Fi pada exergaming (juga lewat Consensus).
2. Latency non-link 21-102 ms berasal dari konferensi yang dikutip Durnez dkk.; ganti dengan pengukuran sendiri bila bisa.
3. Bobot, histeresis, dwell, dan ambang putus adalah parameter desain: **dikalibrasi** dari S1, lalu **dibekukan** sebelum S3-S5.
4. Batas exergame berasal dari satu studi; semua ambang dapat diubah tanpa flash ulang: `--set rtt_bad_ms=200`.

**Varian pembanding** (semuanya otomatis menjadi kelompok sendiri di analisis):
`--skor-hanya-latency` (usulan "latency saja"), `--ambang-ketat` (sensitivitas), atau keduanya.

## 3. Persiapan (sekali saja)

**Perangkat keras:** BNO055 ke ESP32-C3: SDA = GPIO2, SCL = GPIO3, PS0 dan PS1 ke GND (ubah `I2C_SDA`/`I2C_SCL` di
`gamepad.ino` bila berbeda).

**Router dan laptop:**
- Router harus 2,4 GHz (ESP32-C3 tidak mendukung 5 GHz) dan boleh tanpa internet.
- Laptop dan ESP32 harus di jaringan router yang sama (bukan jaringan tamu/terisolasi).
- **Disarankan laptop memakai kabel LAN ke router.** Dengan begitu radio Wi-Fi laptop bukan variabel pengukuran, dan
  jalur Wi-Fi yang diuji = ESP32 ke router. Catat pilihan Anda lewat `--laptop-koneksi ethernet|wifi`.
- Izinkan Python pada firewall Windows untuk jaringan **privat**.

**Firmware (Arduino IDE):**
1. Buka `firmware/gamepad/secrets.h`, isi `WIFI_SSID` dan `WIFI_PASS`.
2. Board: *ESP32C3 Dev Module*, **USB CDC On Boot: Enabled**. Tidak perlu library tambahan.
3. Buka `gamepad.ino` (file `.h` harus satu folder), Upload.
4. Serial Monitor: ketik `PING` -> `#PONG fw=gp-2.0`; ketik `STATUS` -> harus `wifi=1 ip=192.168.x.x`.
   **Tutup Serial Monitor** sebelum menjalankan Python.

**Python (3.10):** di folder `host`: `pip install -r requirements.txt`.

**Bluetooth Windows:** hapus perangkat `GAMEPAD-ESP32` dari Pengaturan Bluetooth bila pernah dipasangkan
(mencegah masalah cache setelah flash ulang).

## 3b. Daya dan kendali ESP32: dua mode

| | Mode USB | Mode adaptor (untuk data skripsi) |
|---|---|---|
| Daya ESP32 | dari laptop lewat USB | dari adaptor/charger USB (colok ke stop kontak) |
| Kendali (START, log) | USB serial (`--port COM22`) | TCP lewat Wi-Fi port 4212 (tanpa `--port`) |
| Dipakai untuk | flash firmware, uji bertahap, debugging | semua percobaan skripsi (S1-S5) |

Pakai **satu konfigurasi daya yang sama untuk semua skenario** (adaptor yang sama, kabel yang sama), supaya perbandingan
antar skenario setara. Selama sesi kabel USB tidak membawa data apa pun; yang diuji tetap BLE dan Wi-Fi.

Cara pindah ke mode adaptor:
1. Flash firmware dan cek `STATUS` lewat USB sampai `wifi=1 ip=192.168.x.x`.
2. Cabut dari laptop, colok ke adaptor, tunggu sekitar 15 detik sampai tersambung ke router.
3. Jalankan program **tanpa `--port`**. Program mencari ESP32 otomatis di jaringan (broadcast). Bila gagal, lihat IP ESP32
   di daftar perangkat router dan isi `--esp-ip 192.168.x.x`.

Kanal TCP hanya dipakai sebelum START dan setelah sesi selesai; selama sesi 60 detik kanal ini diam. Log tetap
tersimpan di RAM ESP32 sampai START berikutnya. Bila koneksi putus saat pengiriman log (mis. gangguan Wi-Fi di S3-S5),
program menyambung ulang dan meminta log lagi (`DUMP`) secara otomatis. Untuk kasus gangguan berat, hentikan gangguan
sebentar setelah sesi selesai agar log terkirim.

## 4. Uji bertahap (WAJIB sebelum ambil data skripsi)

Jalankan di folder `host` (ganti `COM22`). Langkah 1-4 boleh memakai USB. **Tambahkan satu langkah akhir tanpa `--port`**
(mode adaptor) sebelum mengambil data. Setiap langkah harus berhasil sebelum lanjut.

| Langkah | Perintah | Hasil yang diharapkan |
|---|---|---|
| 1. Wi-Fi saja | `python eksperimen.py --port COM22 --mode wifi --tanpa-ble --ulang 1 --durasi 15 --skenario UJI --tanpa-jeda` | `OK`, RTT median beberapa ms |
| 2. BLE | `python eksperimen.py --port COM22 --mode ble --ulang 1 --durasi 15 --skenario UJI --tanpa-jeda` | `OK`, RTT median puluhan ms |
| 3. Adaptif | `python eksperimen.py --port COM22 --mode adaptif --ulang 1 --durasi 30 --skenario UJI --tanpa-jeda` | `OK`; tutup antena ESP32 dengan tangan lalu lihat `pindah` |
| 4. Semua | `python eksperimen.py --port COM22 --mode semua --ulang 2 --durasi 15 --skenario UJI --tanpa-jeda` | 6 percobaan `OK` |
| 5. Mode adaptor | cabut USB, colok adaptor, lalu `python eksperimen.py --mode semua --ulang 2 --durasi 15 --skenario UJI --tanpa-jeda` | ESP32 ditemukan otomatis, 6 percobaan `OK` |
| 6. Analisis | `python analisis.py data/UJI` | folder `hasil_analisis` terbentuk |

Bila loss atau RTT besar, buka `ringkasan_trial.csv` dan bandingkan `gagal_sampai_laptop_pct` (tidak tiba di laptop),
`echo_hilang_pct` (tiba tetapi echo tidak kembali), `proses_laptop_median_ms`, dan `echo_dibuang_laptop`. Kolom-kolom ini
menunjukkan **di mana** hambatannya. Data skenario `UJI` jangan dimasukkan ke analisis skripsi.

## 5. Mengambil data skripsi

Prinsip: `--mode semua` (urutan metode diacak per putaran) agar perubahan kondisi radio dari waktu ke waktu tidak
bias ke satu metode. 5 ulangan per metode. Satu label skenario = satu kondisi tetap.

```
python eksperimen.py --mode semua --ulang 5 --durasi 60 --skenario S1_1m --jarak "1 m" --laptop-koneksi ethernet
python eksperimen.py --mode adaptif --skor-hanya-latency --ulang 5 --durasi 60 --skenario S1_1m --laptop-koneksi ethernet
```

| Skenario | Cara menjalankan |
|---|---|
| S1 jarak statis | satu label per jarak (`S1_0.5m`, `S1_1m`, `S1_2m`, `S1_3m`); dipakai juga untuk **kalibrasi ambang** |
| S2 gerakan exergaming | `--skenario S2` dengan pola gerak tetap yang sama tiap percobaan (mis. ayunan lengan mengikuti metronom); catat di `--catatan` |
| S3 interferensi | satu label per tingkat tetap (`S3_level1`, ...); catat jumlah dan jenis perangkat aktif di `--catatan` |
| S4 transisi baik ke buruk | gangguan yang sama pada **detik yang sama** (stopwatch), mis. detik ke-20: tutup antena atau menjauh |
| S5 kepadatan | `--skenario S5 --catatan "n perangkat Wi-Fi/BLE aktif"` |

Data tersimpan di `data/<skenario>/<nomor>_<metode>_r<ulangan>_<waktu>/`: `data.csv`, `probe.csv`, `trace.csv`
(skor kualitas tiap 100 ms), `events.csv` (perpindahan + pemicu), `host_rx.csv`, `meta.json` (semua parameter,
firmware, kondisi). **Jangan edit file mentah.**

## 6. Analisis

```
python analisis.py data/S1_1m        # satu skenario
python analisis.py data              # semua skenario
```
Keluaran di `hasil_analisis/`: `ringkasan_trial.csv`, `ringkasan_kelompok.csv` (rata-rata, simpangan baku, CI 95%),
`uji_statistik.txt` (ANOVA, Tukey HSD, Kruskal-Wallis, Shapiro-Wilk, eta kuadrat), grafik perbandingan dan linimasa,
serta dua tabel untuk pertanyaan "kondisi apa yang memicu switching":
- `pemicu_switching.csv`: satu baris per perpindahan (waktu, arah, alasan, pemicu, skor, latency p90, loss, jitter, RSSI, durasi handover)
- `ringkasan_pemicu.csv`: rekap per skenario, metode, arah, dan pemicu

Satuan analisis adalah **percobaan**, bukan paket (paket dalam satu sesi saling berkorelasi). Data dari simulator
otomatis ditolak oleh `analisis.py`.

## 7. Perbedaan dari proposal (perlu disesuaikan di laporan)

| Proposal | Sistem ini | Alasan |
|---|---|---|
| Penerima telepon pintar | **laptop** | studi kasus exergaming di laptop; pencatatan terkendali |
| ESP32 sebagai titik akses | **Wi-Fi melalui router (tanpa internet)** | keputusan penelitian saat ini |
| WebSocket (TCP) | **UDP** | TCP menyembunyikan loss sebagai lonjakan latency, sehingga loss tidak terukur jelas |
| BNO055 atau sensor lain | **BNO055 saja** | satu sensor |
| Skor gabungan, histeresis, dwell | sama, dengan ambang dari literatur dan pemicu tercatat | lihat bagian 2 |
| Throughput | tidak diukur | ukuran dan laju paket tetap |

## 8. Batasan yang jujur

- Firmware **belum pernah dikompilasi/dijalankan di board nyata oleh pembuatnya**. Yang sudah diuji: logika adaptif
  (10 skenario di PC), sintaks firmware dengan header tiruan, dan seluruh alur laptop-sampai-analisis dengan
  simulator. Jalur BLE (`bleak`), Wi-Fi ke router, dan sensor baru teruji saat Anda menjalankan uji bertahap.
  Bila Arduino IDE menampilkan error, kirim pesannya apa adanya.
- RTT BLE dipengaruhi interval koneksi yang dinegosiasikan Windows (ESP32 hanya meminta 7,5-15 ms).
- Hemat daya Wi-Fi dimatikan lewat `WiFi.setSleep(false)`, tetapi coexistence BLE+Wi-Fi bisa memaksa mode hemat daya
  tertentu; pengaruhnya terlihat pada RTT Wi-Fi.
- Ambang awal adalah hipotesis berbasis literatur dan harus dikalibrasi dengan data karakterisasi Anda sendiri.
- Dengan 5 percobaan per metode (3 kelompok), ANOVA baru mencapai daya 80% bila perbedaan antar metode sangat besar (eta kuadrat sekitar 0,45 ke atas); untuk efek besar yang umum (eta kuadrat 0,14) dayanya hanya sekitar 21%. Hasil tidak signifikan jangan ditafsirkan sebagai tidak ada perbedaan. Laporkan ukuran efek dan CI 95%, dan tambah ulangan pada S3 dan S4 bila waktu memungkinkan.

## 9. Masalah umum

| Gejala | Penyebab / tindakan |
|---|---|
| `ESP32 tidak membalas PING` | port salah; Serial Monitor masih terbuka; firmware belum di-flash |
| `ESP32 belum tersambung ke router` | SSID/sandi di `secrets.h` salah; router 5 GHz saja; upload ulang |
| `tidak membalas HELLO lewat Wi-Fi` | laptop beda jaringan dengan ESP32; jaringan tamu; firewall Python |
| `tidak ditemukan lewat BLE` | Bluetooth laptop mati; perangkat terhubung ke aplikasi lain; hapus pairing lama |
| `RUSAK: jumlah baris ...` | data terpotong; ulangi; pada mode USB coba kabel lain |
| `ESP32 tidak ditemukan` (mode adaptor) | ESP32 belum tersambung ke router; broadcast diblokir router; isi `--esp-ip` manual |
| `Gagal membuka kanal kendali TCP` | firmware lama (sebelum gp-2.0 versi kendali TCP); flash ulang |
| Loss besar, `gagal_sampai_laptop_pct` kecil | echo terlambat/hilang di sisi laptop; lihat `echo_dibuang_laptop`; tutup program berat |

## 10. Sumber ambang (terverifikasi)

- W. Durnez dkk., "Spaz! The Effects of Local Latency on Player Actions in a Desktop-based Exergame," IEEE Transactions on Games, vol. 14, pp. 623-631, 2022, doi: 10.1109/TG.2021.3128714.
- A. Schmid dkk., "Small Latency Variations Do Not Affect Player Performance in First-Person Shooters," Proc. ACM HCI 7 (CHI PLAY), 2023, doi: 10.1145/3611027.
- D. Halbhuber, V. Schwind, N. Henze, "Don't Break my Flow: Effects of Switching Latency in Shooting Video Games," Proc. ACM HCI 6 (CHI PLAY), 2022, doi: 10.1145/3549492.
- Pendukung (konferensi): T. Waltemate dkk., ACM VRST 2016, doi: 10.1145/2993369.2993381; L. Hoyet dkk., IEEE AIVR 2019, doi: 10.1109/AIVR46125.2019.00066.
