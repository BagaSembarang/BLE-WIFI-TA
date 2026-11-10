// Uji unit mesin keputusan adaptif (v2) di PC. Build & jalankan: ./run_tests.sh
#include <cstdio>
#include <cstdlib>
#include <vector>
#include <functional>
#include "../gamepad/adaptive_core.h"

struct Chan { float loss, rtt, sd, rssi; };
typedef std::function<Chan(uint32_t t_ms, int link)> Scenario;

static uint32_t rs = 12345;
static float urand() { rs ^= rs << 13; rs ^= rs >> 17; rs ^= rs << 5; return (rs & 0xFFFFFF) / 16777216.0f; }
static float nrand() { float s = 0; for (int i = 0; i < 12; i++) s += urand(); return s - 6.0f; }

struct Pending { uint32_t at; int link; bool ok; uint32_t rtt_us; bool activeData; };
struct Result { int switches; uint32_t first_switch; int first_to; uint8_t first_reason, first_cause; std::vector<uint32_t> times; };

static Result run(Scenario sc, uint32_t dur_ms, AcParams prm = AcParams()) {
  AdaptiveCore c; c.p = prm; c.reset(0, AC_LINK_BLE);
  std::vector<Pending> pend; Result r{0, 0, -1, 0, 0, {}};
  uint32_t nextData = 0, nextProbe = 50, nextRssi = 0;
  for (uint32_t t = 0; t <= dur_ms; t++) {
    auto launch = [&](int link, bool activeData) {
      Chan ch = sc(t, link);
      bool lost = urand() < ch.loss;
      float rtt = ch.rtt + ch.sd * nrand(); if (rtt < 1) rtt = 1;
      if (lost || rtt > (float)prm.miss_timeout_ms) pend.push_back({t + prm.miss_timeout_ms, link, false, 0, activeData});
      else pend.push_back({t + (uint32_t)rtt, link, true, (uint32_t)(rtt * 1000), activeData});
    };
    if (t == nextData) { launch(c.active, true); nextData += 20; }
    if (t == nextProbe) { launch(c.active ^ 1, false); nextProbe += 50; }
    for (size_t i = 0; i < pend.size();) {
      if (pend[i].at <= t) { c.onResult(pend[i].link, pend[i].ok, pend[i].rtt_us, t, pend[i].activeData); pend.erase(pend.begin() + i); } else i++;
    }
    if (t >= nextRssi) { nextRssi += 500; c.onRssi(0, sc(t, 0).rssi, t); c.onRssi(1, sc(t, 1).rssi, t); }
    if (c.evaluate(t)) {
      if (r.switches == 0) { r.first_switch = t; r.first_to = c.last_to; r.first_reason = c.last_reason; r.first_cause = c.last_cause; }
      r.switches++; r.times.push_back(t);
    }
  }
  return r;
}

static int fails = 0;
#define CHECK(name, cond) do { bool ok_ = (cond); printf("  [%s] %s\n", ok_ ? "PASS" : "FAIL", name); if (!ok_) fails++; } while (0)

int main() {
  const Chan good_ble{0.005f, 40, 8, -60}, good_wifi{0.003f, 8, 3, -55};
  printf("A. Kondisi sehat -> tidak pindah\n");
  { Result r = run([&](uint32_t, int l){ return l == 0 ? good_ble : good_wifi; }, 60000);
    printf("   switches=%d\n", r.switches); CHECK("0 perpindahan", r.switches == 0); }

  printf("B. Latency BLE memburuk (RTT 250+-30 ms, tanpa loss) pada t=20 s\n");
  { Result r = run([&](uint32_t t, int l){ if (l == 1) return good_wifi; return t < 20000 ? good_ble : Chan{0.0f, 250, 30, -60}; }, 60000);
    printf("   switches=%d first=%u ms to=%d reason=%d cause=%d\n", r.switches, r.first_switch, r.first_to, r.first_reason, r.first_cause);
    CHECK("pindah ke Wi-Fi", r.first_to == AC_LINK_WIFI);
    CHECK("pindah <= 6 s sejak degradasi", r.first_switch >= 20000 && r.first_switch <= 26000);
    CHECK("pemicu = LATENCY", r.first_cause == AC_CAUSE_LATENCY);
    CHECK("tepat 1 perpindahan", r.switches == 1); }

  printf("B2. Loss BLE 30%% (latency normal) pada t=20 s\n");
  { Result r = run([&](uint32_t t, int l){ if (l == 1) return good_wifi; return t < 20000 ? good_ble : Chan{0.30f, 40, 8, -60}; }, 60000);
    printf("   switches=%d first=%u ms cause=%d\n", r.switches, r.first_switch, r.first_cause);
    CHECK("pindah ke Wi-Fi", r.first_to == AC_LINK_WIFI);
    CHECK("pemicu = LOSS", r.first_cause == AC_CAUSE_LOSS); }

  printf("C. BLE mati total pada t=20 s -> jalur darurat\n");
  { Result r = run([&](uint32_t t, int l){ if (l == 1) return good_wifi; return t < 20000 ? good_ble : Chan{1.0f, 0, 0, -60}; }, 60000);
    printf("   first=%u ms reason=%d cause=%d (delay %d ms)\n", r.first_switch, r.first_reason, r.first_cause, (int)r.first_switch - 20000);
    CHECK("alasan = EMERGENCY, pemicu = OUTAGE", r.first_reason == AC_REASON_EMERGENCY && r.first_cause == AC_CAUSE_OUTAGE);
    CHECK("pindah < 700 ms sejak BLE mati", r.first_switch >= 20000 && r.first_switch - 20000 < 700); }

  printf("D. BLE buruk 20-40 s lalu pulih -> kembali ke BLE\n");
  { Result r = run([&](uint32_t t, int l){ if (l == 1) return good_wifi; return (t >= 20000 && t < 40000) ? Chan{0.0f, 250, 30, -60} : good_ble; }, 70000);
    printf("   switches=%d times:", r.switches); for (auto x : r.times) printf(" %u", x); printf("\n");
    CHECK("tepat 2 perpindahan (pergi lalu kembali)", r.switches == 2);
    CHECK("kembali ke BLE <= 8 s setelah pulih", r.times.size() == 2 && r.times[1] >= 40000 && r.times[1] <= 48000); }

  printf("E. BLE naik-turun cepat tiap 400 ms (uji histeresis/dwell)\n");
  { Result r = run([&](uint32_t t, int l){ if (l == 1) return good_wifi; return ((t / 400) % 2) ? Chan{0.0f, 270, 30, -60} : good_ble; }, 60000);
    printf("   switches=%d (batas dwell 3 s => maks 20)\n", r.switches);
    CHECK("perpindahan <= 8 dalam 60 s", r.switches <= 8); }

  printf("F. Kedua link sama-sama buruk -> jangan bolak-balik\n");
  { Result r = run([&](uint32_t, int){ return Chan{0.30f, 250, 30, -60}; }, 60000);
    printf("   switches=%d\n", r.switches); CHECK("perpindahan <= 1", r.switches <= 1); }

  printf("G. Wi-Fi (cadangan) buruk, BLE sehat -> tetap di BLE\n");
  { Result r = run([&](uint32_t, int l){ return l == 0 ? good_ble : Chan{0.5f, 200, 50, -60}; }, 60000);
    printf("   switches=%d\n", r.switches); CHECK("0 perpindahan", r.switches == 0); }

  printf("H. Jitter besar saja (RTT 60+-40 ms, tidak melebihi ambang latency) -> tidak pindah\n");
  { Result r = run([&](uint32_t, int l){ return l == 0 ? Chan{0.003f, 60, 40, -60} : good_wifi; }, 60000);
    printf("   switches=%d\n", r.switches); CHECK("0 perpindahan (sesuai Schmid dkk.: variasi ~50 ms tidak fatal)", r.switches == 0); }

  printf("I. RSSI BLE lemah saja (-90 dBm) tetapi latency normal -> tidak pindah\n");
  { Result r = run([&](uint32_t, int l){ return l == 0 ? Chan{0.005f, 40, 8, -90} : good_wifi; }, 60000);
    printf("   switches=%d\n", r.switches); CHECK("0 perpindahan (RSSI hanya indikator dini)", r.switches == 0); }

  printf("J. Skor 'hanya latency' (w_lat=1, lainnya 0), RTT BLE 250+-30 ms pada t=20 s\n");
  { AcParams p; p.w_lat = 1.0f; p.w_loss = p.w_jit = p.w_rssi = 0.0f;
    Result r = run([&](uint32_t t, int l){ if (l == 1) return good_wifi; return t < 20000 ? good_ble : Chan{0.0f, 250, 30, -60}; }, 60000, p);
    printf("   switches=%d first=%u ms cause=%d\n", r.switches, r.first_switch, r.first_cause);
    CHECK("pindah dengan pemicu LATENCY", r.first_to == AC_LINK_WIFI && r.first_cause == AC_CAUSE_LATENCY); }

  printf("\n%s (%d gagal)\n", fails ? "ADA YANG GAGAL" : "SEMUA UJI LULUS", fails);
  return fails ? 1 : 0;
}
