// adaptive_core.h - Mesin keputusan komunikasi adaptif BLE <-> Wi-Fi (v2)
// Portabel: TIDAK bergantung Arduino/ESP-IDF, sehingga logika yang sama diuji di PC
// (firmware/test/test_adaptive.cpp) dan dipakai apa adanya di ESP32.
//
// ASAL AMBANG (semua dapat diubah lewat perintah SET, dan dicatat di meta.json):
//  Latency adalah indikator UTAMA. Batasnya DITURUNKAN dari literatur (lihat README bagian 2):
//   - Durnez dkk. (2022, IEEE Trans. Games; 37 partisipan, exergame desktop): dampak latency kecil
//     di bawah ~200 ms, performa turun tajam pada 300-500 ms, titik balik ~400 ms, anggaran total
//     yang disarankan <= 300 ms (dari gerakan sampai tampil di layar).
//   - Komponen non-link (layar + OS + game di laptop: 21-102 ms menurut Raaen & Petlund, dikutip
//     Durnez dkk.; sensor + jadwal kirim: ~30 ms rancangan) ~ 130 ms.
//   Jatah link = 300 - 130 = 170 ms, dikurangi margin 40 ms -> latency satu arah buruk ~ 130 ms
//   (RTT ~ 260 ms). Batas "baik" = 200 - 130 = 70 ms satu arah (RTT 140 ms).
//   Firmware mengukur RTT; latency satu arah diperkirakan RTT/2. Normalisasi latency: RTT p90 140 ms
//   -> penalti 0, RTT p90 300 ms -> penalti penuh. Dengan bobot di bawah dan t_leave = 65, titik pindah
//   karena latency saja jatuh pada RTT p90 ~ 265 ms (satu arah ~ 132 ms).
//  Jitter: Schmid dkk. (2023, PACM HCI) - variasi latency ~50 ms tidak menurunkan performa secara
//   signifikan, sehingga jitter baru dianggap buruk pada ~50 ms dan bobotnya kecil.
//  Loss (tidak ada echo dalam miss_timeout) dianggap latency tak terhingga; bobotnya tinggi agar
//   putusnya paket tetap memicu perpindahan. RSSI hanya indikator dini (bobot kecil).
//  Histeresis/dwell: Halbhuber dkk. (2022, PACM HCI) - perubahan level latency selama bermain merusak
//   flow; latency konstan sebaiknya diprioritaskan, jadi perpindahan sering harus dicegah.
//  Bobot, histeresis, dwell, dan ambang putus adalah parameter DESAIN (bukan dari jurnal) dan
//  dikalibrasi dari data karakterisasi lalu dibekukan sebelum pengujian utama.
//
// MEKANISME:
//  - Skor kualitas 0..100 per link pada jendela waktu geser. Link cadangan diukur lewat probe.
//  - Histeresis: t_leave (keluar) berbeda dari t_return (kembali) + margin selisih skor antar link.
//  - Waktu tunggu minimum (dwell) dan evaluasi beruntun agar satu pengukuran buruk tidak memicu.
//  - Jalur darurat: link aktif diam (tanpa echo) >= emerg_silence_ms dan link cadangan layak.
//  - Setiap perpindahan mencatat PEMICU (cause) dan nilai indikator saat keputusan.
#pragma once
#include <stdint.h>
#include <string.h>

#define AC_LINK_BLE   0
#define AC_LINK_WIFI  1
#define AC_MAX_EV     192

enum AcReason : uint8_t { AC_REASON_NONE = 0, AC_REASON_SCORE = 1, AC_REASON_EMERGENCY = 2, AC_REASON_RETURN = 3 };
enum AcCause  : uint8_t { AC_CAUSE_NONE = 0, AC_CAUSE_LATENCY = 1, AC_CAUSE_LOSS = 2, AC_CAUSE_JITTER = 3,
                          AC_CAUSE_RSSI = 4, AC_CAUSE_OUTAGE = 5 };

struct AcParams {
  uint32_t window_ms       = 1500;    // lebar jendela statistik
  uint32_t eval_ms         = 100;     // periode evaluasi keputusan
  uint8_t  min_samples     = 5;       // sampel minimum agar skor sah
  uint32_t miss_timeout_ms = 300;     // tanpa echo selama ini (RTT) = hilang (setara latency satu arah 150 ms)
  // normalisasi komponen (nilai "buruk" -> komponen bernilai 1)
  float rtt_good_ms = 140.0f;         // p90 RTT <= ini tanpa penalti (satu arah ~70 ms)
  float rtt_bad_ms  = 300.0f;         // p90 RTT >= ini penalti penuh (satu arah ~150 ms)
  float jit_bad_ms  = 50.0f;
  float loss_bad    = 0.15f;
  float rssi_good   = -70.0f;
  float rssi_bad    = -90.0f;
  // bobot (total 1.0). Latency utama; set w_lat=1 dan lainnya 0 untuk skor "hanya latency".
  float w_lat = 0.45f, w_loss = 0.40f, w_jit = 0.075f, w_rssi = 0.075f;
  // keputusan
  float    t_leave       = 65.0f;     // link aktif ditinggalkan bila skor < ini ...
  float    margin        = 20.0f;     // ... dan link lain lebih baik minimal sebesar ini
  float    t_return      = 85.0f;     // kembali ke link utama bila skornya >= ini
  float    return_tol    = 10.0f;
  uint8_t  consec_leave  = 3;         // x eval_ms beruntun
  uint8_t  consec_return = 20;        // x eval_ms beruntun
  uint32_t dwell_ms      = 3000;
  uint32_t emerg_silence_ms = 250;    // link aktif tanpa echo selama ini = putus
  uint32_t emerg_gap_ms  = 500;       // jarak minimum dari perpindahan sebelumnya
  uint32_t standby_alive_ms = 500;
  float    emerg_min_score = 50.0f;   // link tujuan darurat harus punya skor >= ini (bila sudah sah)
  uint32_t settle_ms     = 6000;      // link yang baru tersambung (ulang) belum boleh DIPILIH lewat skor sebelum ini.
                                      // Uji awal: setelah BLE tersambung ulang, probe 20 Hz sudah baik tetapi beban data
                                      // 50 Hz masih tertunda 160-250 ms selama ~4-8 dtk (negosiasi koneksi), sehingga
                                      // kembali terlalu cepat memicu bolak-balik. Jalur darurat tidak terpengaruh.
  uint8_t  preferred     = AC_LINK_BLE;
};

struct AcEv { uint32_t t_ms; uint32_t rtt_us; uint8_t ok; };

struct AcLinkWin {
  AcEv ev[AC_MAX_EV];
  uint16_t head, n;
  float rssi; uint32_t rssi_ms; bool rssi_valid;
  uint32_t last_ok_ms; bool any_ok;
  void clear() { head = 0; n = 0; rssi = 0; rssi_ms = 0; rssi_valid = false; last_ok_ms = 0; any_ok = false; }
  void add(uint32_t t, bool ok, uint32_t rtt_us) {
    ev[head].t_ms = t; ev[head].rtt_us = rtt_us; ev[head].ok = ok ? 1 : 0;
    head = (uint16_t)((head + 1) % AC_MAX_EV);
    if (n < AC_MAX_EV) n++;
    if (ok) { any_ok = true; last_ok_ms = t; }
  }
};

struct AcDetail {
  bool  valid;
  float score;                 // 0..100
  float p90_ms, loss, jit_ms, rssi;
  float pen_lat, pen_loss, pen_jit, pen_rssi;   // kontribusi penalti (poin skor)
};

static inline float ac_clamp01(float x) { return x < 0 ? 0 : (x > 1 ? 1 : x); }

static inline AcDetail ac_detail(const AcLinkWin& w, uint32_t now, const AcParams& p) {
  AcDetail d; memset(&d, 0, sizeof(d)); d.score = -1.0f;
  float r[AC_MAX_EV]; int k = 0, n_ok = 0, n_miss = 0;
  for (uint16_t i = 0; i < w.n; i++) {
    uint16_t idx = (uint16_t)((w.head + AC_MAX_EV - w.n + i) % AC_MAX_EV);
    const AcEv& e = w.ev[idx];
    uint32_t age = (now >= e.t_ms) ? (now - e.t_ms) : 0;
    if (age > p.window_ms) continue;
    if (e.ok) { r[k++] = e.rtt_us / 1000.0f; n_ok++; } else n_miss++;
  }
  int total = n_ok + n_miss;
  if (total < p.min_samples) return d;
  d.valid = true;
  d.loss = (float)n_miss / (float)total;
  if (k >= 2) {
    float j = 0;
    for (int i = 1; i < k; i++) j += (r[i] > r[i-1]) ? (r[i] - r[i-1]) : (r[i-1] - r[i]);
    d.jit_ms = j / (float)(k - 1);
  }
  if (k > 0) {
    float s[AC_MAX_EV];
    for (int i = 0; i < k; i++) s[i] = r[i];
    for (int i = 1; i < k; i++) { float v = s[i]; int j = i - 1; while (j >= 0 && s[j] > v) { s[j+1] = s[j]; j--; } s[j+1] = v; }
    int ix = (int)((0.9f * k) + 0.999f) - 1; if (ix < 0) ix = 0; if (ix >= k) ix = k - 1;
    d.p90_ms = s[ix];
  }
  d.rssi = w.rssi_valid ? w.rssi : 0;
  float nl   = ac_clamp01(d.loss / p.loss_bad);
  float nlat = (k > 0) ? ac_clamp01((d.p90_ms - p.rtt_good_ms) / (p.rtt_bad_ms - p.rtt_good_ms)) : 1.0f;
  float nj   = ac_clamp01(d.jit_ms / p.jit_bad_ms);
  float nr   = 0;
  if (w.rssi_valid && (now - w.rssi_ms) < 3000) nr = ac_clamp01((p.rssi_good - w.rssi) / (p.rssi_good - p.rssi_bad));
  d.pen_lat = 100.0f * p.w_lat * nlat; d.pen_loss = 100.0f * p.w_loss * nl;
  d.pen_jit = 100.0f * p.w_jit * nj;   d.pen_rssi = 100.0f * p.w_rssi * nr;
  float s = 100.0f - (d.pen_lat + d.pen_loss + d.pen_jit + d.pen_rssi);
  d.score = s < 0 ? 0 : (s > 100 ? 100 : s);
  return d;
}

// Skor 0..100 (lebih besar = lebih baik). -1 bila sampel belum cukup.
static inline float ac_score(const AcLinkWin& w, uint32_t now, const AcParams& p) { return ac_detail(w, now, p).score; }

class AdaptiveCore {
 public:
  AcParams p;
  AcLinkWin win[2];
  uint8_t  active;
  float    score[2];
  // info perpindahan terakhir
  uint8_t  last_reason, last_cause, last_from, last_to;
  float    last_score_from, last_score_to;
  AcDetail last_detail;            // indikator link ASAL saat keputusan

  void reset(uint32_t now_ms, uint8_t initial) {
    win[0].clear(); win[1].clear();
    active = initial; lastEval = now_ms; lastSwitch = now_ms;
    leaveCnt = retCnt = 0;
    score[0] = score[1] = -1.0f;
    last_reason = AC_REASON_NONE; last_cause = AC_CAUSE_NONE; last_from = last_to = initial;
    last_score_from = last_score_to = -1.0f; memset(&last_detail, 0, sizeof(last_detail));
    fresh[0] = fresh[1] = false; up_ms[0] = up_ms[1] = now_ms;
  }

  // Dipanggil saat sebuah link tersambung (ulang) di tengah sesi: riwayat lama dibuang, masa stabilisasi dimulai.
  void onLinkUp(uint8_t link, uint32_t now_ms) {
    link &= 1;
    win[link].clear();
    fresh[link] = true; up_ms[link] = now_ms;
  }
  bool settled(uint8_t link, uint32_t now_ms) const {
    link &= 1;
    return !fresh[link] || (now_ms - up_ms[link]) >= p.settle_ms;
  }

  // Hasil satu paket: ok=true (echo kembali dalam miss_timeout, rtt_us valid) atau ok=false (hilang).
  void onResult(uint8_t link, bool ok, uint32_t rtt_us, uint32_t now_ms, bool /*activeData*/) {
    win[link & 1].add(now_ms, ok, rtt_us);
  }
  void onRssi(uint8_t link, float rssi, uint32_t now_ms) {
    win[link & 1].rssi = rssi; win[link & 1].rssi_ms = now_ms; win[link & 1].rssi_valid = true;
  }

  // true bila HARUS pindah (active sudah diperbarui; baca last_*).
  bool evaluate(uint32_t now) {
    if ((now - lastEval) < p.eval_ms) return false;
    lastEval = now;
    uint8_t a = active, b = (uint8_t)(a ^ 1);
    AcDetail dA = ac_detail(win[a], now, p), dB = ac_detail(win[b], now, p);
    score[a] = dA.score; score[b] = dB.score;
    float sA = dA.score, sB = dB.score;

    // 1) Jalur darurat: link aktif diam, link cadangan terbukti hidup dan layak
    uint32_t ref = win[a].any_ok ? (win[a].last_ok_ms > lastSwitch ? win[a].last_ok_ms : lastSwitch) : lastSwitch;
    uint32_t silence = (now >= ref) ? (now - ref) : 0;
    bool standbyAlive = win[b].any_ok && (now - win[b].last_ok_ms) <= p.standby_alive_ms
                        && (sB < 0 || sB >= p.emerg_min_score);
    if (silence >= p.emerg_silence_ms && standbyAlive && (now - lastSwitch) >= p.emerg_gap_ms)
      return doSwitch(now, AC_REASON_EMERGENCY, AC_CAUSE_OUTAGE, dA, sB);

    // 2) Tinggalkan link aktif yang memburuk
    bool leaveCond = (sA >= 0 && sB >= 0 && sA < p.t_leave && sB >= sA + p.margin) && settled(b, now);
    leaveCnt = leaveCond ? (uint8_t)(leaveCnt + 1) : 0;

    // 3) Kembali ke link utama bila sudah pulih
    bool retCond = (a != p.preferred) && sB >= p.t_return && (sA < 0 || sB >= sA - p.return_tol) && settled(b, now);
    retCnt = retCond ? (uint8_t)(retCnt + 1) : 0;

    bool dwellOk = (now - lastSwitch) >= p.dwell_ms;
    if (dwellOk && leaveCnt >= p.consec_leave) return doSwitch(now, AC_REASON_SCORE, dominantCause(dA), dA, sB);
    if (dwellOk && retCnt   >= p.consec_return) return doSwitch(now, AC_REASON_RETURN, dominantCause(dA), dA, sB);
    return false;
  }

 private:
  uint32_t lastEval, lastSwitch;
  uint8_t  leaveCnt, retCnt;
  bool     fresh[2];
  uint32_t up_ms[2];

  static uint8_t dominantCause(const AcDetail& d) {
    if (!d.valid) return AC_CAUSE_NONE;
    float m = d.pen_lat; uint8_t c = AC_CAUSE_LATENCY;
    if (d.pen_loss > m) { m = d.pen_loss; c = AC_CAUSE_LOSS; }
    if (d.pen_jit  > m) { m = d.pen_jit;  c = AC_CAUSE_JITTER; }
    if (d.pen_rssi > m) { m = d.pen_rssi; c = AC_CAUSE_RSSI; }
    return m > 0.5f ? c : (uint8_t)AC_CAUSE_NONE;
  }

  bool doSwitch(uint32_t now, uint8_t reason, uint8_t cause, const AcDetail& dFrom, float sTo) {
    last_from = active; last_to = (uint8_t)(active ^ 1);
    last_reason = reason; last_cause = cause; last_detail = dFrom;
    last_score_from = dFrom.score; last_score_to = sTo;
    active = last_to; lastSwitch = now; leaveCnt = retCnt = 0;
    return true;
  }
};