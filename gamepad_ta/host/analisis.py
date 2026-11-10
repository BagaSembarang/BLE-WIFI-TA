#!/usr/bin/env python3
"""
analisis.py - Mengolah data mentah hasil eksperimen.py menjadi tabel, uji statistik, dan grafik.

Pemakaian:
    python analisis.py data/S1                 # satu skenario
    python analisis.py data                    # semua skenario di bawah folder data
Keluaran (folder hasil_analisis/ di sebelah folder data, atau --out):
    ringkasan_trial.csv      satu baris per percobaan (semua metrik)
    ringkasan_kelompok.csv   rata-rata, simpangan baku, CI 95% per skenario x metode
    uji_statistik.txt        ANOVA satu arah + Tukey HSD + Kruskal-Wallis per metrik
    fig_*.png                grafik

Prinsip statistik:
  * Satuan analisis = PERCOBAAN (bukan paket). Paket dalam satu sesi saling berkorelasi,
    memakainya sebagai sampel bebas akan menggelembungkan signifikansi (pseudo-replikasi).
  * Definisi: loss = tidak ada echo dalam 1 detik. Latency satu arah diperkirakan RTT/2.
    Ambang latency berasal dari literatur exergame (Durnez dkk., 2022): satu arah > 100 ms = "lambat" (peringatan),
    > 150 ms = melewati jatah link (setengah dari anggaran total 300 ms). buruk150_pct = loss + lambat150.
  * Latency dilaporkan sebagai RTT (ukuran langsung) dan estimasi satu arah = RTT/2 setelah
    dikurangi waktu proses di laptop (diukur pada jam laptop yang sama).
"""
import argparse
import csv
import glob
import json
import os
import sys

import numpy as np

WARMUP_S = 1.0
LOSS_WINDOW_US = 1_000_000
OWD_WARN_MS = 100.0      # latency satu arah (RTT/2) peringatan
OWD_BAD_MS = 150.0       # jatah link: setengah anggaran total 300 ms (Durnez dkk., 2022)
ALASAN = {1: "skor", 2: "darurat", 3: "kembali"}
PEMICU = {0: "-", 1: "latency", 2: "loss", 3: "jitter", 4: "rssi", 5: "putus"}
LINKNAMA = {0: "BLE", 1: "Wi-Fi"}


def baca_csv(path):
    if not os.path.exists(path):
        return []
    with open(path, newline="", encoding="utf-8") as f:
        return list(csv.DictReader(f))


def kolom(rows, key, tipe=float):
    return np.array([tipe(r[key]) for r in rows]) if rows else np.array([])


def hitung(folder, meta, warmup_s):
    D = baca_csv(os.path.join(folder, "data.csv"))
    P = baca_csv(os.path.join(folder, "probe.csv"))
    E = baca_csv(os.path.join(folder, "events.csv"))
    H = baca_csv(os.path.join(folder, "host_rx.csv"))
    m = {"skenario": meta["skenario"], "metode": meta.get("metode_label", meta["mode_nama"]), "ulangan": meta["ulangan"],
         "folder": os.path.basename(folder), "valid": bool(meta.get("valid", False)),
         "jarak": meta.get("jarak", ""), "durasi_s": meta["durasi_s"]}
    if not D:
        return m
    seq = kolom(D, "seq", int)
    tx = kolom(D, "tx_us")
    rtt = kolom(D, "rtt_us")
    ltx = kolom(D, "link_tx", int)
    lrx = kolom(D, "link_rx", int)
    rb = kolom(D, "rssi_ble")
    rw = kolom(D, "rssi_wifi")
    keep = tx >= warmup_s * 1e6
    seq, tx, rtt, ltx, lrx, rb, rw = (a[keep] for a in (seq, tx, rtt, ltx, lrx, rb, rw))
    n = len(seq)
    deliv = (rtt > 0) & (rtt <= LOSS_WINDOW_US)
    m["n_paket"] = n
    m["laju_kirim_hz"] = n / max(1e-9, (tx.max() - tx.min()) / 1e6) if n > 1 else 0
    m["loss_pct"] = 100.0 * (n - deliv.sum()) / n
    m["lambat100_pct"] = 100.0 * (deliv & (rtt / 2000.0 > OWD_WARN_MS)).sum() / n
    m["lambat150_pct"] = 100.0 * (deliv & (rtt / 2000.0 > OWD_BAD_MS)).sum() / n
    m["buruk150_pct"] = m["loss_pct"] + m["lambat150_pct"]
    usable = deliv & (rtt / 2000.0 <= OWD_BAD_MS)
    r_ms = rtt[deliv] / 1000.0
    if len(r_ms):
        m.update(rtt_mean_ms=r_ms.mean(), rtt_median_ms=np.median(r_ms), rtt_p95_ms=np.percentile(r_ms, 95),
                 rtt_p99_ms=np.percentile(r_ms, 99), rtt_max_ms=r_ms.max())
        idx = np.where(deliv)[0]
        if len(idx) >= 2:
            d = np.abs(np.diff(r_ms))
            same = lrx[idx][1:] == lrx[idx][:-1]          # jitter hanya antar paket di link yang sama
            m["jitter_ms"] = float(d[same].mean()) if same.any() else float("nan")
    # Sisi laptop: pisahkan keterlambatan akibat program laptop
    first = {}
    for r in H:
        if int(r["tipe"]) == 1 and int(r["seq"]) not in first:
            first[int(r["seq"])] = (float(r["t_rx_ns"]), float(r["t_selesai_echo_ns"]), int(r["flags"]))
    seen = np.array([int(s) in first for s in seq])
    m["gagal_sampai_laptop_pct"] = 100.0 * (n - seen.sum()) / n
    m["echo_hilang_pct"] = 100.0 * ((seen & ~deliv).sum()) / n
    m["echo_dibuang_laptop"] = sum(1 for v in first.values() if v[2] & 2)
    both = deliv & seen
    if both.any():
        proc = np.array([(first[int(s)][1] - first[int(s)][0]) / 1e6 for s in seq[both]])   # ms
        adj = rtt[both] / 1000.0 - proc
        m["proses_laptop_median_ms"] = float(np.median(proc))
        m["oneway_est_median_ms"] = float(np.median(adj) / 2.0)
        m["oneway_est_p95_ms"] = float(np.percentile(adj, 95) / 2.0)
    # Kontinuitas masukan
    tx_ok = tx[usable]
    if len(tx_ok) >= 2:
        gaps = np.diff(tx_ok) / 1000.0
        m["stall_maks_ms"] = float(gaps.max())
        m["stall_gt100_kali"] = int((gaps > 100).sum())
        m["stall_gt200_kali"] = int((gaps > 200).sum())
    # Pemakaian link & RSSI
    m["pakai_ble_pct"] = 100.0 * (ltx == 0).sum() / n
    m["pakai_wifi_pct"] = 100.0 * (ltx == 1).sum() / n
    vb, vw = rb[rb > -99], rw[rw > -99]
    m["rssi_ble_dbm"] = float(vb.mean()) if len(vb) else float("nan")
    m["rssi_wifi_dbm"] = float(vw.mean()) if len(vw) else float("nan")
    # Handover
    sw = [r for r in E if int(r["kind"]) == 1]
    ho = [int(r["extra_us"]) / 1000.0 for r in E if int(r["kind"]) == 2]
    m["handover_kali"] = len(sw)
    if ho:
        m["handover_durasi_median_ms"] = float(np.median(ho))
        m["handover_durasi_maks_ms"] = float(max(ho))
    ts = sorted(int(r["t_us"]) for r in sw)
    m["flapping_kali"] = int(sum(1 for a, b in zip(ts, ts[1:]) if b - a < 5_000_000))
    m["alasan_darurat"] = sum(1 for r in sw if int(r["alasan"]) == 2)
    for kode, nama_p in PEMICU.items():
        if kode:
            m[f"pemicu_{nama_p}"] = sum(1 for r in sw if int(r["pemicu"]) == kode)
    # Probe = pengukuran link non-data (ada di semua metode)
    if P:
        ptx = kolom(P, "tx_us"); prt = kolom(P, "rtt_us"); pl = kolom(P, "link_tx", int)
        k = ptx >= warmup_s * 1e6
        for nama, lid in (("ble", 0), ("wifi", 1)):
            sel = k & (pl == lid)
            if sel.sum() >= 5:
                d = (prt[sel] > 0) & (prt[sel] <= LOSS_WINDOW_US)
                m[f"probe_{nama}_loss_pct"] = 100.0 * (sel.sum() - d.sum()) / sel.sum()
                if d.any():
                    m[f"probe_{nama}_rtt_median_ms"] = float(np.median(prt[sel][d]) / 1000.0)
    return m


def muat(root, izinkan_sim, warmup_s):
    metas = sorted(glob.glob(os.path.join(root, "**", "meta.json"), recursive=True))
    if not metas:
        sys.exit(f"Tidak ada meta.json di bawah {root}")
    hasil, ditolak = [], 0
    for mp in metas:
        meta = json.load(open(mp, encoding="utf-8"))
        if meta.get("simulasi") and not izinkan_sim:
            ditolak += 1
            continue
        if not meta.get("valid", False):
            print(f"  (dilewati, validasi gagal) {os.path.dirname(mp)}: {meta.get('pesan_validasi')}")
            continue
        hasil.append((os.path.dirname(mp), meta, hitung(os.path.dirname(mp), meta, warmup_s)))
    if ditolak:
        print(f"!! {ditolak} percobaan SIMULASI ditolak (bukan data skripsi). Pakai --izinkan-simulasi hanya untuk menguji skrip ini.")
    if not hasil:
        sys.exit("Tidak ada percobaan valid untuk dianalisis.")
    return hasil


METRIK_UTAMA = [
    ("loss_pct", "Packet loss (%)"), ("lambat100_pct", "Latency satu arah >100 ms (%)"),
    ("buruk150_pct", "Loss + latency satu arah >150 ms (%)"),
    ("rtt_median_ms", "RTT median (ms)"), ("rtt_p95_ms", "RTT p95 (ms)"), ("jitter_ms", "Jitter (ms)"),
    ("oneway_est_median_ms", "Estimasi latency satu arah median (ms)"),
    ("stall_maks_ms", "Jeda masukan terpanjang (ms)"), ("stall_gt100_kali", "Jeda >100 ms (kali)"),
    ("handover_durasi_median_ms", "Durasi handover median (ms)"), ("handover_kali", "Jumlah handover"),
]


def tulis_trial(rows, path):
    keys = []
    for r in rows:
        for k in r:
            if k not in keys:
                keys.append(k)
    with open(path, "w", newline="", encoding="utf-8") as f:
        w = csv.DictWriter(f, fieldnames=keys, lineterminator="\n")
        w.writeheader()
        for r in rows:
            w.writerow({k: (round(v, 4) if isinstance(v, float) else v) for k, v in r.items()})


def ci95(x):
    from scipy import stats
    x = np.asarray(x, float)
    x = x[~np.isnan(x)]
    if len(x) < 2:
        return float("nan")
    return float(stats.t.ppf(0.975, len(x) - 1) * x.std(ddof=1) / np.sqrt(len(x)))


def tulis_kelompok(rows, path):
    grup = {}
    for r in rows:
        grup.setdefault((r["skenario"], r["metode"]), []).append(r)
    with open(path, "w", newline="", encoding="utf-8") as f:
        w = csv.writer(f, lineterminator="\n")
        w.writerow(["skenario", "metode", "n_percobaan", "metrik", "rata2", "sd", "ci95_setengah_lebar", "min", "maks"])
        for (sk, md), rs in sorted(grup.items()):
            for key, _ in METRIK_UTAMA:
                v = np.array([r.get(key, np.nan) for r in rs], float)
                v = v[~np.isnan(v)]
                if len(v) == 0:
                    continue
                w.writerow([sk, md, len(v), key, round(v.mean(), 4), round(v.std(ddof=1), 4) if len(v) > 1 else "",
                            round(ci95(v), 4), round(v.min(), 4), round(v.max(), 4)])


def uji_statistik(rows, path):
    from scipy import stats
    L = []
    L.append("UJI STATISTIK - satuan analisis = percobaan (bukan paket)")
    L.append("H0: tidak ada perbedaan rata-rata metrik antar metode komunikasi. alpha = 0.05\n")
    skenarios = sorted({r["skenario"] for r in rows})
    for sk in skenarios:
        rs = [r for r in rows if r["skenario"] == sk]
        L.append("=" * 78)
        L.append(f"SKENARIO {sk}")
        L.append("=" * 78)
        for key, label in METRIK_UTAMA:
            g = {}
            for r in rs:
                v = r.get(key, np.nan)
                if v is not None and not (isinstance(v, float) and np.isnan(v)):
                    g.setdefault(r["metode"], []).append(float(v))
            g = {k: np.array(v) for k, v in g.items() if len(v) >= 2}
            if len(g) < 2:
                continue
            L.append(f"\n{label}  [{key}]")
            for k, v in sorted(g.items()):
                L.append(f"   {k:8s} n={len(v):2d}  rata2={v.mean():10.3f}  sd={v.std(ddof=1):9.3f}")
            groups = [v for _, v in sorted(g.items())]
            names = [k for k, _ in sorted(g.items())]
            if all(np.ptp(v) == 0 for v in groups) and len({v[0] for v in groups}) == 1:
                L.append("   (semua nilai identik, uji tidak relevan)")
                continue
            F, p = stats.f_oneway(*groups)
            allv = np.concatenate(groups)
            ss_b = sum(len(v) * (v.mean() - allv.mean()) ** 2 for v in groups)
            ss_t = ((allv - allv.mean()) ** 2).sum()
            eta2 = ss_b / ss_t if ss_t > 0 else float("nan")
            L.append(f"   ANOVA satu arah: F={F:.3f}  p={p:.4g}  eta^2={eta2:.3f}  -> {'BEDA signifikan' if p < 0.05 else 'tidak beda signifikan'}")
            try:
                H, pk = stats.kruskal(*groups)
                L.append(f"   Kruskal-Wallis (tanpa asumsi normal): H={H:.3f}  p={pk:.4g}")
            except ValueError:
                pass
            if len(groups) >= 3 and p < 0.05:
                res = stats.tukey_hsd(*groups)
                for i in range(len(names)):
                    for j in range(i + 1, len(names)):
                        beda = groups[i].mean() - groups[j].mean()
                        L.append(f"   Tukey {names[i]} vs {names[j]}: selisih rata2={beda:+.3f}  p={res.pvalue[i][j]:.4g}  "
                                 f"{'*' if res.pvalue[i][j] < 0.05 else ''}")
            elif len(groups) == 2 and p < 0.05:
                L.append("   (dua kelompok: ANOVA setara uji-t; tidak perlu post-hoc)")
            resid = np.concatenate([v - v.mean() for v in groups])
            if len(resid) >= 8 and np.ptp(resid) > 0:
                sh = stats.shapiro(resid)
                L.append(f"   Shapiro-Wilk residual: p={sh.pvalue:.3g}  -> {'normalitas ditolak, utamakan Kruskal-Wallis' if sh.pvalue < 0.05 else 'normalitas tidak ditolak'}")
        L.append("")
    L.append("Catatan: jumlah percobaan per kelompok kecil, sehingga kekuatan uji terbatas. "
             "Laporkan ukuran efek (eta^2) dan CI 95%, bukan hanya nilai p.")
    open(path, "w", encoding="utf-8").write("\n".join(L))
    return "\n".join(L)


def tulis_pemicu(hasil, outdir):
    """Satu baris per perpindahan: kondisi apa yang memicu (latency/loss/jitter/RSSI/putus) dan nilainya."""
    baris = []
    for folder, meta, m in hasil:
        if not m["metode"].startswith("adaptif"):
            continue
        E = baca_csv(os.path.join(folder, "events.csv"))
        for i, r in enumerate(E):
            if int(r["kind"]) != 1:
                continue
            ho = next((int(x["extra_us"]) / 1000.0 for x in E[i + 1:] if int(x["kind"]) == 2), "")
            baris.append({
                "skenario": m["skenario"], "metode": m["metode"], "ulangan": m["ulangan"],
                "waktu_s": round(int(r["t_us"]) / 1e6, 2),
                "dari": LINKNAMA[int(r["dari"])], "ke": LINKNAMA[int(r["ke"])],
                "alasan": ALASAN.get(int(r["alasan"]), "?"), "pemicu": PEMICU.get(int(r["pemicu"]), "?"),
                "skor_dari": int(r["skor_dari_x10"]) / 10.0, "skor_ke": int(r["skor_ke_x10"]) / 10.0,
                "p90_rtt_ms": int(r["p90_rtt_x10_ms"]) / 10.0, "latency_satu_arah_p90_ms": int(r["p90_rtt_x10_ms"]) / 20.0,
                "loss_persen": int(r["loss_x10_persen"]) / 10.0, "jitter_ms": int(r["jitter_x10_ms"]) / 10.0,
                "rssi_dbm": int(r["rssi_dbm"]), "durasi_handover_ms": ho})
    if not baris:
        return
    with open(os.path.join(outdir, "pemicu_switching.csv"), "w", newline="", encoding="utf-8") as f:
        w = csv.DictWriter(f, fieldnames=list(baris[0].keys()), lineterminator="\n")
        w.writeheader()
        w.writerows(baris)
    grup = {}
    for b in baris:
        grup.setdefault((b["skenario"], b["metode"], f"{b['dari']}->{b['ke']}", b["pemicu"]), []).append(b)
    with open(os.path.join(outdir, "ringkasan_pemicu.csv"), "w", newline="", encoding="utf-8") as f:
        w = csv.writer(f, lineterminator="\n")
        w.writerow(["skenario", "metode", "arah", "pemicu", "jumlah", "latency_satu_arah_p90_rata2_ms", "loss_rata2_persen",
                    "jitter_rata2_ms", "rssi_rata2_dbm", "durasi_handover_median_ms"])
        for (sk, md, ar, pm), bs in sorted(grup.items()):
            dur = [b["durasi_handover_ms"] for b in bs if b["durasi_handover_ms"] != ""]
            w.writerow([sk, md, ar, pm, len(bs), round(np.mean([b["latency_satu_arah_p90_ms"] for b in bs]), 1),
                        round(np.mean([b["loss_persen"] for b in bs]), 2), round(np.mean([b["jitter_ms"] for b in bs]), 1),
                        round(np.mean([b["rssi_dbm"] for b in bs]), 1), round(float(np.median(dur)), 1) if dur else ""])


def grafik(hasil, rows, outdir, label_prefix):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    warna = {"ble": "#2f6fdb", "wifi": "#e07b00", "adaptif": "#2a9d4b"}
    urut = {"ble": 0, "wifi": 1, "adaptif": 2}
    skenarios = sorted({r["skenario"] for r in rows})
    for sk in skenarios:
        rs = [r for r in rows if r["skenario"] == sk]
        metodes = sorted({r["metode"] for r in rs}, key=lambda m: (urut.get(m, 3), m))
        fig, ax = plt.subplots(2, 3, figsize=(14, 7))
        for a, (key, label) in zip(ax.ravel(), [("loss_pct", "Packet loss (%)"), ("buruk150_pct", "Loss + latency satu arah >150 ms (%)"),
                                                ("rtt_median_ms", "RTT median (ms)"), ("rtt_p95_ms", "RTT p95 (ms)"),
                                                ("jitter_ms", "Jitter (ms)"), ("stall_maks_ms", "Jeda masukan terpanjang (ms)")]):
            data = [[r.get(key, np.nan) for r in rs if r["metode"] == m] for m in metodes]
            data = [[x for x in d if x == x] for d in data]
            try:
                bp = a.boxplot(data, tick_labels=[m.upper() for m in metodes], patch_artist=True)
            except TypeError:                      # matplotlib < 3.9
                bp = a.boxplot(data, labels=[m.upper() for m in metodes], patch_artist=True)
            for patch, m in zip(bp["boxes"], metodes):
                patch.set_facecolor(warna.get(m, "#8a8a8a")); patch.set_alpha(0.55)
            for i, d in enumerate(data, 1):
                a.scatter(np.full(len(d), i) + np.random.uniform(-0.08, 0.08, len(d)), d, s=14, c="k", zorder=3)
            a.set_title(label, fontsize=10); a.grid(alpha=0.3)
        fig.suptitle(f"{label_prefix}Skenario {sk}: perbandingan metode (tiap titik = satu percobaan)")
        fig.tight_layout()
        fig.savefig(os.path.join(outdir, f"fig_perbandingan_{sk}.png"), dpi=140)
        plt.close(fig)
    # Linimasa percobaan adaptif pertama per skenario
    for sk in skenarios:
        cand = [(f, me, m) for f, me, m in hasil if m["skenario"] == sk and m["metode"].startswith("adaptif")]
        if not cand:
            continue
        f, me, m = max(cand, key=lambda x: x[2].get("handover_kali", 0))
        D = baca_csv(os.path.join(f, "data.csv")); Q = baca_csv(os.path.join(f, "trace.csv")); E = baca_csv(os.path.join(f, "events.csv"))
        tx = kolom(D, "tx_us") / 1e6; rtt = kolom(D, "rtt_us") / 1000.0; lrx = kolom(D, "link_rx", int)
        ok = (rtt > 0) & (rtt <= 1000)
        fig, ax = plt.subplots(2, 1, figsize=(12, 6), sharex=True)
        for lid, nm in ((0, "BLE"), (1, "Wi-Fi")):
            s = ok & (lrx == lid)
            ax[0].scatter(tx[s], rtt[s], s=5, c=warna["ble"] if lid == 0 else warna["wifi"], label=f"RTT via {nm}")
        ax[0].scatter(tx[~ok], np.zeros((~ok).sum()), s=10, c="r", marker="x", label="hilang")
        ax[0].set_ylabel("RTT (ms)"); ax[0].set_yscale("symlog", linthresh=50); ax[0].legend(loc="upper right"); ax[0].grid(alpha=0.3)
        if Q:
            qt = kolom(Q, "t_ms") / 1000.0
            ax[1].plot(qt, kolom(Q, "skor_ble_x10") / 10.0, c=warna["ble"], label="skor BLE")
            ax[1].plot(qt, kolom(Q, "skor_wifi_x10") / 10.0, c=warna["wifi"], label="skor Wi-Fi")
        for r in E:
            if int(r["kind"]) == 1:
                for a in ax:
                    a.axvline(int(r["t_us"]) / 1e6, c="k", ls="--", lw=0.8)
        ax[1].set_ylabel("skor kualitas (0-100)"); ax[1].set_xlabel("waktu sesi (detik)"); ax[1].legend(loc="lower right"); ax[1].grid(alpha=0.3)
        fig.suptitle(f"{label_prefix}Skenario {sk}, adaptif ulangan {m['ulangan']} (garis putus = perpindahan)")
        fig.tight_layout()
        fig.savefig(os.path.join(outdir, f"fig_linimasa_adaptif_{sk}.png"), dpi=140)
        plt.close(fig)


def main():
    ap = argparse.ArgumentParser(description="Analisis data percobaan komunikasi BLE/Wi-Fi/Adaptif.")
    ap.add_argument("folder", help="folder data (mis. data/S1 atau data)")
    ap.add_argument("--out", help="folder keluaran (bawaan: hasil_analisis di sebelah folder data)")
    ap.add_argument("--warmup", type=float, default=WARMUP_S, help="detik awal yang dibuang (bawaan 1)")
    ap.add_argument("--izinkan-simulasi", action="store_true", help="HANYA untuk menguji skrip ini dengan data simulator")
    a = ap.parse_args()
    out = a.out or os.path.join(os.path.dirname(os.path.abspath(a.folder.rstrip("/\\"))), "hasil_analisis")
    hasil = muat(a.folder, a.izinkan_simulasi, a.warmup)
    rows = [h[2] for h in hasil]
    sim = any(h[1].get("simulasi") for h in hasil)
    if sim:
        out += "_SIMULASI"
    os.makedirs(out, exist_ok=True)
    prefix = "[SIMULASI - BUKAN DATA SKRIPSI] " if sim else ""
    tulis_trial(rows, os.path.join(out, "ringkasan_trial.csv"))
    tulis_kelompok(rows, os.path.join(out, "ringkasan_kelompok.csv"))
    txt = uji_statistik(rows, os.path.join(out, "uji_statistik.txt"))
    tulis_pemicu(hasil, out)
    grafik(hasil, rows, out, prefix)
    print(f"{len(rows)} percobaan valid dianalisis. Hasil di: {os.path.abspath(out)}\n")
    print(txt[:3500])


if __name__ == "__main__":
    main()
