#!/usr/bin/env python3
"""
dashboard.py - Sistem informasi pemantauan dan analisis kinerja komunikasi BLE / Wi-Fi / Adaptif.

Dashboard ini HANYA MEMBACA data mentah hasil eksperimen.py (folder data/, satu folder per sesi:
meta.json, data.csv, probe.csv, trace.csv, events.csv, host_rx.csv). Tidak ada data yang diubah.
Semua perhitungan memakai fungsi di analisis.py, sehingga angka di dashboard sama dengan angka di laporan.

Menjalankan:
    pip install -r requirements.txt
    streamlit run dashboard.py
    streamlit run dashboard.py -- --data D:\\Skripsi\\sistem\\gamepad_ta\\host\\data
"""
import argparse
import glob
import json
import os
import sys
import tempfile

import numpy as np
import pandas as pd
import plotly.graph_objects as go
import streamlit as st
from plotly.subplots import make_subplots

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import analisis as A  # noqa: E402

# --------------------------------------------------------------------------- konstanta tampilan
WARNA_METODE = {"ble": "#2f6fdb", "wifi": "#e07b00", "adaptif": "#2a9d4b"}
WARNA_VARIAN = ["#7a4fbf", "#c2417a", "#4a8a8a", "#8a6a2a"]
WARNA_LINK = {0: "#2f6fdb", 1: "#e07b00"}
WARNA_PEMICU = {"latency": "#d1495b", "loss": "#edae49", "jitter": "#66a182", "rssi": "#00798c", "putus": "#30343f", "-": "#999999",
                "kembali (BLE pulih)": "#2a9d4b"}
URUT_METODE = {"ble": 0, "wifi": 1, "adaptif": 2}
LABEL_METRIK = dict(A.METRIK_UTAMA)
KIND = {1: "keputusan pindah", 2: "handover selesai"}
GRAFIK_CFG = {"displaylogo": False, "toImageButtonOptions": {"format": "png", "scale": 3}}

PARAM_BAWAAN = {"rtt_good_ms": 140.0, "rtt_bad_ms": 300.0, "t_leave": 65.0, "t_return": 85.0,
                "miss_timeout_ms": 300.0, "dwell_ms": 3000.0}


def urut_metode(m):
    return (URUT_METODE.get(m, 3), m)


def warna_metode(metode, semua):
    if metode in WARNA_METODE:
        return WARNA_METODE[metode]
    lain = sorted(x for x in semua if x not in WARNA_METODE)
    return WARNA_VARIAN[lain.index(metode) % len(WARNA_VARIAN)] if metode in lain else "#8a8a8a"


def parse_params(line):
    out = dict(PARAM_BAWAAN)
    if line:
        for tok in line.replace("#", "", 1).split()[1:]:
            if "=" in tok:
                k, v = tok.split("=", 1)
                try:
                    out[k] = float(v)
                except ValueError:
                    pass
    return out


def fmt(v, nd=2):
    if v is None or v == "" or (isinstance(v, float) and np.isnan(v)):
        return "-"
    return f"{v:.{nd}f}" if isinstance(v, (float, np.floating)) else str(v)


# --------------------------------------------------------------------------- pemuatan data (baca saja)
def tanda_tangan(root):
    """Daftar (path, waktu ubah) meta.json, dipakai agar cache dimuat ulang bila ada sesi baru."""
    paths = sorted(glob.glob(os.path.join(root, "**", "meta.json"), recursive=True))
    return tuple((p, os.path.getmtime(p)) for p in paths)


@st.cache_data(show_spinner="Memuat dan menghitung data sesi ...")
def muat_semua(root, _sig, warmup):
    sesi = []
    for mp, _ in _sig:
        folder = os.path.dirname(mp)
        try:
            meta = json.load(open(mp, encoding="utf-8"))
        except Exception as e:
            sesi.append({"folder": folder, "meta": {}, "m": None, "galat": f"meta.json rusak: {e}"})
            continue
        try:
            m = A.hitung(folder, meta, warmup)
            galat = ""
        except Exception as e:
            m, galat = None, f"gagal dihitung: {e}"
        sesi.append({"folder": folder, "meta": meta, "m": m, "galat": galat})
    return sesi


@st.cache_data(show_spinner=False)
def baca_df(path):
    return pd.read_csv(path) if os.path.exists(path) and os.path.getsize(path) > 0 else pd.DataFrame()


def label_sesi(s):
    me = s["meta"]
    return (f"{me.get('skenario', '?')} | {me.get('metode_label', me.get('mode_nama', '?'))} | "
            f"ulangan {me.get('ulangan', '?')} | {os.path.basename(s['folder'])}")


# --------------------------------------------------------------------------- sidebar
def argumen():
    ap = argparse.ArgumentParser(add_help=False)
    ap.add_argument("--data", default=None)
    a, _ = ap.parse_known_args()
    return a


st.set_page_config(page_title="Dashboard Komunikasi Adaptif BLE/Wi-Fi", page_icon="📶", layout="wide")
arg = argumen()
here = os.path.dirname(os.path.abspath(__file__))
bawaan = arg.data or os.path.join(here, "data")

with st.sidebar:
    st.header("Sumber data")
    root = st.text_input("Folder data", value=bawaan,
                         help="Folder keluaran eksperimen.py. Berisi subfolder per skenario, lalu satu folder per sesi.")
    warmup = st.number_input("Masa pemanasan dibuang (detik)", min_value=0.0, max_value=10.0, value=A.WARMUP_S, step=0.5)
    st.divider()
    st.header("Penyaringan")
    ikut_sim = st.checkbox("Sertakan data SIMULASI", value=False, help="Hanya untuk menguji dashboard. Bukan data skripsi.")
    ikut_inj = st.checkbox("Sertakan uji INJEKSI", value=False, help="Uji fungsional mesin keputusan. Bukan data perbandingan.")
    ikut_uji = st.checkbox("Sertakan skenario UJI*", value=False, help="Sesi uji bertahap. Bukan data skripsi.")
    if st.button("Muat ulang data", width="stretch"):
        st.cache_data.clear()

st.title("Dashboard Kinerja Komunikasi Adaptif BLE / Wi-Fi")
st.caption("Pengontrol exergaming ESP32-C3 + BNO055. Dashboard membaca data mentah tanpa mengubahnya; "
           "perhitungan memakai analisis.py yang sama dengan yang dipakai untuk laporan.")

if not os.path.isdir(root):
    st.error(f"Folder tidak ditemukan: `{root}`. Isi path folder data di panel kiri.")
    st.stop()

sig = tanda_tangan(root)
if not sig:
    st.warning(f"Belum ada sesi (meta.json) di `{root}`. Jalankan eksperimen.py terlebih dahulu.")
    st.stop()

semua = muat_semua(root, sig, warmup)


def lolos_saring(s):
    me = s["meta"]
    if me.get("simulasi") and not ikut_sim:
        return False
    if me.get("injeksi") and not ikut_inj:
        return False
    if str(me.get("skenario", "")).upper().startswith("UJI") and not ikut_uji and not me.get("injeksi"):
        return False
    return True


tersaring = [s for s in semua if lolos_saring(s)]
dibuang = len(semua) - len(tersaring)
skenario_ada = sorted({s["meta"].get("skenario", "?") for s in tersaring})
with st.sidebar:
    pilih_sk = st.multiselect("Skenario", skenario_ada, default=skenario_ada)
    st.caption(f"{len(semua)} sesi ditemukan, {dibuang} disembunyikan oleh penyaring.")

tersaring = [s for s in tersaring if s["meta"].get("skenario") in pilih_sk]
valid = [s for s in tersaring if s["m"] is not None and s["meta"].get("valid")]
rows = [s["m"] for s in valid]
hasil = [(s["folder"], s["meta"], s["m"]) for s in valid]

if any(s["meta"].get("simulasi") for s in tersaring):
    st.warning("Tampilan ini memuat DATA SIMULASI. Angka di bawah bukan data skripsi.", icon="⚠️")
if any(s["meta"].get("injeksi") for s in tersaring):
    st.info("Tampilan ini memuat sesi uji INJEKSI gangguan (uji fungsional), bukan data perbandingan metode.", icon="ℹ️")
if not tersaring:
    st.warning("Tidak ada sesi yang lolos penyaring. Ubah pilihan di panel kiri.")
    st.stop()

tab_ring, tab_band, tab_detail, tab_pemicu, tab_ekspor = st.tabs(
    ["Ringkasan", "Perbandingan Metode", "Detail Sesi", "Pemicu Perpindahan", "Ekspor"])

# =========================================================================== 1. RINGKASAN
with tab_ring:
    n_tidak = sum(1 for s in tersaring if not s["meta"].get("valid") or s["m"] is None)
    c = st.columns(5)
    c[0].metric("Sesi valid", len(valid))
    c[1].metric("Sesi tidak valid", n_tidak)
    c[2].metric("Skenario", len({r["skenario"] for r in rows}) if rows else 0)
    c[3].metric("Total paket data", f"{sum(int(r.get('n_paket', 0)) for r in rows):,}".replace(",", "."))
    c[4].metric("Total perpindahan", sum(int(r.get("handover_kali", 0)) for r in rows))

    st.subheader("Kelengkapan percobaan")
    st.caption("Jumlah sesi valid per skenario dan metode. Target rancangan: 5 ulangan per metode (sel merah = belum lengkap).")
    if rows:
        df_n = pd.DataFrame([{"skenario": r["skenario"], "metode": r["metode"]} for r in rows])
        piv = df_n.value_counts().unstack(fill_value=0)
        piv = piv[sorted(piv.columns, key=urut_metode)]
        target = st.number_input("Target ulangan per metode", 1, 20, 5, key="target")
        st.dataframe(piv.style.map(lambda v: "background-color:#f6d5d5" if v < target else "background-color:#d7efd9"),
                     width="stretch")

    st.subheader("Daftar sesi")
    tampil_tidak_valid = st.checkbox("Tampilkan juga sesi tidak valid", value=True)
    daftar = []
    for s in tersaring:
        me, m = s["meta"], s["m"] or {}
        ok = bool(me.get("valid")) and s["m"] is not None
        if not ok and not tampil_tidak_valid:
            continue
        daftar.append({
            "skenario": me.get("skenario"), "metode": me.get("metode_label", me.get("mode_nama")),
            "ulangan": me.get("ulangan"), "mulai": me.get("mulai"), "jarak": me.get("jarak"),
            "catatan": me.get("catatan"), "valid": ok,
            "pesan validasi": "; ".join(me.get("pesan_validasi") or []) or s["galat"],
            "paket": m.get("n_paket"), "loss %": m.get("loss_pct"), "RTT median ms": m.get("rtt_median_ms"),
            "RTT p95 ms": m.get("rtt_p95_ms"), "jitter ms": m.get("jitter_ms"), "perpindahan": m.get("handover_kali"),
            "folder": os.path.basename(s["folder"]),
        })
    st.dataframe(pd.DataFrame(daftar), width="stretch", hide_index=True,
                 column_config={"loss %": st.column_config.NumberColumn(format="%.2f"),
                                "RTT median ms": st.column_config.NumberColumn(format="%.1f"),
                                "RTT p95 ms": st.column_config.NumberColumn(format="%.1f"),
                                "jitter ms": st.column_config.NumberColumn(format="%.1f")})

# =========================================================================== 2. PERBANDINGAN
with tab_band:
    if not rows:
        st.info("Belum ada sesi valid.")
    else:
        sk_list = sorted({r["skenario"] for r in rows})
        sk = st.selectbox("Skenario", sk_list, key="band_sk")
        rs = [r for r in rows if r["skenario"] == sk]
        metodes = sorted({r["metode"] for r in rs}, key=urut_metode)

        st.subheader(f"Statistik deskriptif, skenario {sk}")
        st.caption("Satuan analisis = percobaan. Format: rata-rata ± simpangan baku [CI 95%], n = jumlah percobaan.")
        kel = pd.DataFrame(A.kelompok(rs))
        if not kel.empty:
            def sel(x):
                sd = f" ± {x['sd']:.2f}" if x["sd"] != "" else ""
                ci = f" [±{x['ci95_setengah_lebar']:.2f}]" if not np.isnan(x["ci95_setengah_lebar"]) else ""
                return f"{x['rata2']:.2f}{sd}{ci} (n={x['n_percobaan']})"
            kel["nilai"] = kel.apply(sel, axis=1)
            tab = kel.pivot(index="metrik", columns="metode", values="nilai")
            tab = tab.reindex([k for k, _ in A.METRIK_UTAMA if k in tab.index])
            tab.index = [LABEL_METRIK.get(k, k) for k in tab.index]
            tab = tab[sorted(tab.columns, key=urut_metode)].fillna("-")
            st.dataframe(tab, width="stretch", height=38 * (len(tab) + 1) + 4)

        st.subheader("Sebaran per metode")
        pilihan = [k for k, _ in A.METRIK_UTAMA if any(k in r for r in rs)]
        mk = st.selectbox("Metrik", pilihan, format_func=lambda k: LABEL_METRIK.get(k, k), key="band_mk")
        fig = go.Figure()
        for md in metodes:
            v = [r.get(mk) for r in rs if r["metode"] == md and r.get(mk) is not None and r.get(mk) == r.get(mk)]
            fig.add_trace(go.Box(y=v, name=md.upper(), boxpoints="all", jitter=0.3, pointpos=0,
                                 marker_color=warna_metode(md, metodes), line_color=warna_metode(md, metodes)))
        fig.update_layout(height=420, showlegend=False, yaxis_title=LABEL_METRIK.get(mk, mk),
                          title=f"Skenario {sk}: {LABEL_METRIK.get(mk, mk)} (tiap titik = satu percobaan)",
                          margin=dict(t=50, b=30))
        st.plotly_chart(fig, width="stretch", config=GRAFIK_CFG)

        st.subheader("Antar skenario")
        st.caption("Rata-rata per metode dengan batang galat CI 95%, untuk melihat pola dari kondisi stabil ke kondisi terganggu.")
        kel_all = pd.DataFrame(A.kelompok(rows))
        kel_all = kel_all[kel_all["metrik"] == mk]
        if not kel_all.empty:
            fig2 = go.Figure()
            mets_all = sorted(kel_all["metode"].unique(), key=urut_metode)
            for md in mets_all:
                d = kel_all[kel_all["metode"] == md]
                fig2.add_trace(go.Bar(x=d["skenario"], y=d["rata2"], name=md.upper(),
                                      error_y=dict(type="data", array=d["ci95_setengah_lebar"].fillna(0)),
                                      marker_color=warna_metode(md, mets_all)))
            fig2.update_layout(barmode="group", height=400, yaxis_title=LABEL_METRIK.get(mk, mk), margin=dict(t=30, b=30))
            st.plotly_chart(fig2, width="stretch", config=GRAFIK_CFG)

        st.subheader(f"Uji statistik, skenario {sk}")
        st.caption("ANOVA satu arah + Tukey HSD (bila signifikan), Kruskal-Wallis sebagai pembanding non-parametrik, "
                   "Shapiro-Wilk untuk normalitas residual. alpha = 0,05.")
        uji, tukey = [], []
        for key, label in A.METRIK_UTAMA:
            u = A.uji_metrik(rs, key)
            if u is None:
                continue
            if u["identik"]:
                uji.append({"metrik": label, "kesimpulan": "semua nilai identik, uji tidak relevan"})
                continue
            if u["shapiro_p"] is None:
                saran = "-"
            else:
                saran = "Kruskal-Wallis" if u["shapiro_p"] < 0.05 else "ANOVA"
            p_acuan = u["p_kw"] if saran == "Kruskal-Wallis" and u["p_kw"] is not None else u["p"]
            uji.append({"metrik": label, "F": u["F"], "p ANOVA": u["p"], "eta²": u["eta2"], "H": u["H"],
                        "p Kruskal-Wallis": u["p_kw"], "p Shapiro-Wilk": u["shapiro_p"], "acuan": saran,
                        "kesimpulan": "beda signifikan" if p_acuan < 0.05 else "tidak beda signifikan"})
            for a, b, beda, pt in u["tukey"]:
                tukey.append({"metrik": label, "pasangan": f"{a} vs {b}", "selisih rata-rata": beda, "p": pt,
                              "signifikan": "ya" if pt < 0.05 else "tidak"})
        if uji:
            st.dataframe(pd.DataFrame(uji), width="stretch", hide_index=True,
                         column_config={k: st.column_config.NumberColumn(format="%.4f")
                                        for k in ["F", "p ANOVA", "eta²", "H", "p Kruskal-Wallis", "p Shapiro-Wilk"]})
        else:
            st.info("Uji membutuhkan minimal 2 metode dengan minimal 2 percobaan valid masing-masing.")
        if tukey:
            st.markdown("**Uji lanjut Tukey HSD**")
            st.dataframe(pd.DataFrame(tukey), width="stretch", hide_index=True,
                         column_config={"selisih rata-rata": st.column_config.NumberColumn(format="%+.3f"),
                                        "p": st.column_config.NumberColumn(format="%.4f")})
        n_min = min((g[1] for key, _ in A.METRIK_UTAMA for u in [A.uji_metrik(rs, key)] if u for g in u["grup"]), default=0)
        if n_min and n_min < 5:
            st.warning(f"Jumlah percobaan per kelompok masih {n_min}. Daya uji rendah: hasil tidak signifikan "
                       "tidak berarti tidak ada perbedaan. Laporkan juga eta² dan CI 95%.")

# =========================================================================== 3. DETAIL SESI
with tab_detail:
    pilihan_sesi = [s for s in tersaring if s["m"] is not None]
    if not pilihan_sesi:
        st.info("Tidak ada sesi yang bisa ditampilkan.")
    else:
        bawaan_idx = next((i for i, s in enumerate(pilihan_sesi) if (s["m"] or {}).get("handover_kali", 0) > 0), 0)
        s = st.selectbox("Pilih sesi", pilihan_sesi, index=bawaan_idx, format_func=label_sesi, key="det_sesi")
        me, m, folder = s["meta"], s["m"], s["folder"]
        P = parse_params(me.get("params", ""))

        if not me.get("valid"):
            st.error("Sesi ini TIDAK VALID dan tidak ikut analisis: " + "; ".join(me.get("pesan_validasi") or []))
        if me.get("injeksi"):
            f = me["injeksi"]
            st.info(f"Uji injeksi: link {f.get('link_nama')} jenis {f.get('jenis')} nilai {f.get('nilai')} "
                    f"pada detik {f.get('mulai_s')} sampai {f.get('selesai_s')}.")

        c = st.columns(6)
        c[0].metric("Paket data", m.get("n_paket", 0))
        c[1].metric("Packet loss", f"{fmt(m.get('loss_pct'))} %")
        c[2].metric("RTT median", f"{fmt(m.get('rtt_median_ms'), 1)} ms")
        c[3].metric("RTT p95", f"{fmt(m.get('rtt_p95_ms'), 1)} ms")
        c[4].metric("Jitter", f"{fmt(m.get('jitter_ms'), 1)} ms")
        c[5].metric("Perpindahan", m.get("handover_kali", 0))

        D = baca_df(os.path.join(folder, "data.csv"))
        PR = baca_df(os.path.join(folder, "probe.csv"))
        Q = baca_df(os.path.join(folder, "trace.csv"))
        E = baca_df(os.path.join(folder, "events.csv"))

        log_y = st.checkbox("Sumbu RTT logaritmik", value=False)
        ada_skor = not Q.empty
        nbaris = 3 if ada_skor else 1
        tinggi = [0.5, 0.35, 0.15] if ada_skor else [1.0]
        judul = ["RTT per paket (ms)", "Skor kualitas link (0-100)", "Link aktif"] if ada_skor else ["RTT per paket (ms)"]
        fig = make_subplots(rows=nbaris, cols=1, shared_xaxes=True, row_heights=tinggi, vertical_spacing=0.06,
                            subplot_titles=judul)
        if not D.empty:
            t = D["tx_us"] / 1e6
            rtt = D["rtt_us"] / 1000.0
            ok = (D["rtt_us"] > 0) & (D["rtt_us"] <= A.LOSS_WINDOW_US)
            for lid, nm in ((0, "BLE"), (1, "Wi-Fi")):
                sel = ok & (D["link_rx"] == lid)
                if sel.any():
                    fig.add_trace(go.Scattergl(x=t[sel], y=rtt[sel], mode="markers", name=f"data via {nm}",
                                               marker=dict(size=4, color=WARNA_LINK[lid])), row=1, col=1)
            if (~ok).any():
                fig.add_trace(go.Scattergl(x=t[~ok], y=np.full((~ok).sum(), 2.0 if log_y else 0.0), mode="markers",
                                           name="hilang (tanpa echo 1 dtk)", marker=dict(size=7, color="#d62728", symbol="x")),
                              row=1, col=1)
        if not PR.empty:
            okp = (PR["rtt_us"] > 0) & (PR["rtt_us"] <= A.LOSS_WINDOW_US)
            for lid, nm in ((0, "BLE"), (1, "Wi-Fi")):
                sel = okp & (PR["link_tx"] == lid)
                if sel.any():
                    fig.add_trace(go.Scattergl(x=PR["tx_us"][sel] / 1e6, y=PR["rtt_us"][sel] / 1000.0, mode="markers",
                                               name=f"probe {nm}", marker=dict(size=3, color=WARNA_LINK[lid], opacity=0.25),
                                               visible="legendonly"), row=1, col=1)
        fig.add_hline(y=P["rtt_good_ms"], line=dict(color="#2a9d4b", dash="dot"), row=1, col=1,
                      annotation_text=f"RTT baik {P['rtt_good_ms']:.0f}", annotation_position="top left")
        fig.add_hline(y=P["rtt_bad_ms"], line=dict(color="#d62728", dash="dot"), row=1, col=1,
                      annotation_text=f"RTT buruk {P['rtt_bad_ms']:.0f}", annotation_position="top left")
        if ada_skor:
            qt = Q["t_ms"] / 1000.0
            for kol, nm, lid in (("skor_ble_x10", "skor BLE", 0), ("skor_wifi_x10", "skor Wi-Fi", 1)):
                y = Q[kol] / 10.0
                y = y.where(y >= 0)        # skor < 0 = belum sah (sampel kurang)
                fig.add_trace(go.Scatter(x=qt, y=y, mode="lines", name=nm, line=dict(color=WARNA_LINK[lid], width=1.5)),
                              row=2, col=1)
            fig.add_hline(y=P["t_leave"], line=dict(color="#d62728", dash="dash"), row=2, col=1,
                          annotation_text=f"ambang keluar {P['t_leave']:.0f}", annotation_position="bottom left")
            fig.add_hline(y=P["t_return"], line=dict(color="#2a9d4b", dash="dash"), row=2, col=1,
                          annotation_text=f"ambang kembali {P['t_return']:.0f}", annotation_position="top left")
            fig.add_trace(go.Scatter(x=qt, y=Q["link_aktif"], mode="lines", line_shape="hv", name="link aktif",
                                     line=dict(color="#555555")), row=3, col=1)
            fig.update_yaxes(tickvals=[0, 1], ticktext=["BLE", "Wi-Fi"], range=[-0.3, 1.3], row=3, col=1)
            fig.update_yaxes(range=[0, 105], row=2, col=1)
        if not E.empty:
            for _, r in E[E["kind"] == 1].iterrows():
                x = r["t_us"] / 1e6
                pm = A.PEMICU.get(int(r["pemicu"]), "?")
                al = A.ALASAN.get(int(r["alasan"]), "?")
                for rr in range(1, nbaris + 1):
                    fig.add_vline(x=x, line=dict(color=WARNA_PEMICU.get(pm, "#000"), dash="dash", width=1.2), row=rr, col=1)
                teks = f"{A.LINKNAMA[int(r['dari'])]}→{A.LINKNAMA[int(r['ke'])]}<br>{al if pm == '-' else pm}"
                fig.add_annotation(x=x, y=1, yref="paper", yshift=24, text=teks, showarrow=False, font=dict(size=10),
                                   bgcolor="rgba(255,255,255,0.85)", bordercolor=WARNA_PEMICU.get(pm, "#999"), yanchor="bottom")
        if log_y:
            fig.update_yaxes(type="log", row=1, col=1)
        fig.update_xaxes(title_text="waktu sesi (detik)", row=nbaris, col=1)
        fig.update_layout(height=800 if ada_skor else 440, legend=dict(orientation="h", y=-0.13, yanchor="top"), margin=dict(t=120, b=110),
                          title=dict(text=f"Linimasa sesi: {label_sesi(s)}", y=0.99, yanchor="top"))
        st.plotly_chart(fig, width="stretch", config=GRAFIK_CFG)
        st.caption("Garis putus vertikal = keputusan perpindahan (warna menurut pemicu). Probe link cadangan tersembunyi; "
                   "klik di legenda untuk menampilkan.")

        kiri, kanan = st.columns([3, 2])
        with kiri:
            st.markdown("**Peristiwa perpindahan**")
            if E.empty:
                st.caption("Tidak ada perpindahan pada sesi ini.")
            else:
                ev = pd.DataFrame({
                    "waktu (dtk)": (E["t_us"] / 1e6).round(3),
                    "jenis": E["kind"].map(KIND).fillna("?"),
                    "dari": E["dari"].map(A.LINKNAMA), "ke": E["ke"].map(A.LINKNAMA),
                    "alasan": E["alasan"].map(A.ALASAN).fillna("-"), "pemicu": E["pemicu"].map(A.PEMICU).fillna("-"),
                    "skor asal": E["skor_dari_x10"] / 10.0, "skor tujuan": E["skor_ke_x10"] / 10.0,
                    "RTT p90 (ms)": E["p90_rtt_x10_ms"] / 10.0, "loss (%)": E["loss_x10_persen"] / 10.0,
                    "jitter (ms)": E["jitter_x10_ms"] / 10.0, "RSSI (dBm)": E["rssi_dbm"],
                    "durasi handover (ms)": np.where(E["kind"] == 2, E["extra_us"] / 1000.0, np.nan),
                })
                st.dataframe(ev, width="stretch", hide_index=True)
        with kanan:
            st.markdown("**Sumber keterlambatan dan kontinuitas**")
            st.dataframe(pd.DataFrame([
                ("Paket tidak sampai laptop (%)", fmt(m.get("gagal_sampai_laptop_pct"))),
                ("Sampai laptop, echo hilang (%)", fmt(m.get("echo_hilang_pct"))),
                ("Waktu proses laptop median (ms)", fmt(m.get("proses_laptop_median_ms"), 3)),
                ("Estimasi latency satu arah median (ms)", fmt(m.get("oneway_est_median_ms"), 1)),
                ("Paket lambat >150 ms satu arah (%)", fmt(m.get("lambat150_pct"))),
                ("Jeda masukan terpanjang (ms)", fmt(m.get("stall_maks_ms"), 1)),
                ("Jeda >100 ms / >200 ms (kali)", f"{fmt(m.get('stall_gt100_kali'))} / {fmt(m.get('stall_gt200_kali'))}"),
                ("Pemakaian BLE / Wi-Fi (%)", f"{fmt(m.get('pakai_ble_pct'), 1)} / {fmt(m.get('pakai_wifi_pct'), 1)}"),
                ("RSSI BLE / Wi-Fi (dBm)", f"{fmt(m.get('rssi_ble_dbm'), 1)} / {fmt(m.get('rssi_wifi_dbm'), 1)}"),
                ("Flapping (<5 dtk, kali)", fmt(m.get("flapping_kali"))),
            ], columns=["indikator", "nilai"]), width="stretch", hide_index=True)

        with st.expander("Metadata dan kondisi pengujian"):
            st.json({k: me.get(k) for k in ["skenario", "metode_label", "ulangan", "urutan_ke", "mulai", "durasi_s", "jarak",
                                             "catatan", "laptop_koneksi", "kontrol", "esp_ip", "firmware", "params",
                                             "status_awal", "valid", "pesan_validasi", "injeksi", "simulasi", "rencana", "host"]})

# =========================================================================== 4. PEMICU
with tab_pemicu:
    bp = A.baris_pemicu(hasil) if hasil else []
    if not bp:
        st.info("Belum ada perpindahan pada sesi adaptif yang tersaring.")
    else:
        df = pd.DataFrame(bp)
        df["arah"] = df["dari"] + " → " + df["ke"]
        df["kategori"] = np.where(df["alasan"] == "kembali", "kembali (BLE pulih)", df["pemicu"])
        keluar = df[df["alasan"] != "kembali"]
        c = st.columns(4)
        c[0].metric("Total perpindahan", len(df))
        c[1].metric("Lewat jalur darurat", int((df["alasan"] == "darurat").sum()))
        dur = pd.to_numeric(df["durasi_handover_ms"], errors="coerce")
        c[2].metric("Handover median", f"{fmt(float(dur.median()), 1)} ms" if dur.notna().any() else "-")
        c[3].metric("Handover maksimum", f"{fmt(float(dur.max()), 1)} ms" if dur.notna().any() else "-")

        kiri, kanan = st.columns(2)
        with kiri:
            hit = df.groupby(["skenario", "kategori"]).size().reset_index(name="jumlah")
            fig = go.Figure()
            for pm in sorted(hit["kategori"].unique()):
                d = hit[hit["kategori"] == pm]
                fig.add_trace(go.Bar(x=d["skenario"], y=d["jumlah"], name=pm, marker_color=WARNA_PEMICU.get(pm, "#888")))
            fig.update_layout(barmode="stack", height=380, title="Frekuensi pemicu per skenario", yaxis_title="jumlah perpindahan",
                              margin=dict(t=50, b=30))
            st.plotly_chart(fig, width="stretch", config=GRAFIK_CFG)
        with kanan:
            fig = go.Figure()
            for pm in sorted(keluar["pemicu"].unique()):
                d = keluar[keluar["pemicu"] == pm]
                fig.add_trace(go.Scatter(x=d["latency_satu_arah_p90_ms"], y=d["loss_persen"], mode="markers", name=pm,
                                         marker=dict(size=10, color=WARNA_PEMICU.get(pm, "#888")),
                                         text=d["skenario"] + " | " + d["arah"] + " | " + d["alasan"],
                                         hovertemplate="%{text}<br>latency p90 %{x:.1f} ms<br>loss %{y:.1f} %<extra></extra>"))
            fig.add_vline(x=PARAM_BAWAAN["rtt_bad_ms"] / 2, line=dict(color="#d62728", dash="dot"),
                          annotation_text="batas buruk satu arah")
            fig.update_layout(height=380, title="Kondisi link asal saat keputusan pindah keluar", xaxis_title="latency satu arah p90 (ms)",
                              yaxis_title="loss pada jendela (%)", margin=dict(t=50, b=30),
                              xaxis=dict(rangemode="tozero"), yaxis=dict(rangemode="tozero"))
            st.plotly_chart(fig, width="stretch", config=GRAFIK_CFG)

        if dur.notna().any():
            fig = go.Figure()
            for skn in sorted(df["skenario"].unique()):
                d = dur[df["skenario"] == skn].dropna()
                fig.add_trace(go.Box(y=d, name=skn, boxpoints="all", jitter=0.3, pointpos=0))
            fig.update_layout(height=340, title="Durasi handover per skenario", yaxis_title="ms", showlegend=False,
                              margin=dict(t=50, b=30))
            st.plotly_chart(fig, width="stretch", config=GRAFIK_CFG)

        st.markdown("**Ringkasan per skenario, arah, dan pemicu**")
        ring = df.assign(durasi=dur).groupby(["skenario", "metode", "arah", "kategori"]).agg(
            jumlah=("waktu_s", "size"), latency_p90_rata2_ms=("latency_satu_arah_p90_ms", "mean"),
            loss_rata2_persen=("loss_persen", "mean"), jitter_rata2_ms=("jitter_ms", "mean"),
            rssi_rata2_dbm=("rssi_dbm", "mean"), handover_median_ms=("durasi", "median")).reset_index().round(2)
        st.dataframe(ring, width="stretch", hide_index=True)
        with st.expander("Semua perpindahan"):
            st.dataframe(df.drop(columns=["arah", "kategori"]), width="stretch", hide_index=True)

# =========================================================================== 5. EKSPOR
with tab_ekspor:
    st.markdown("Unduh tabel hasil untuk Bab 4. Isinya sama dengan keluaran `analisis.py` untuk sesi yang sedang tersaring. "
                "Grafik di setiap halaman bisa diunduh sebagai PNG lewat ikon kamera di pojok kanan atas grafik.")
    if not rows:
        st.info("Belum ada sesi valid.")
    else:
        with tempfile.TemporaryDirectory() as tmp:
            A.tulis_trial(rows, os.path.join(tmp, "t.csv"))
            A.tulis_kelompok(rows, os.path.join(tmp, "k.csv"))
            teks_uji = A.uji_statistik(rows, os.path.join(tmp, "u.txt"))
            trial_csv = open(os.path.join(tmp, "t.csv"), "rb").read()
            kel_csv = open(os.path.join(tmp, "k.csv"), "rb").read()
        c = st.columns(4)
        c[0].download_button("ringkasan_trial.csv", trial_csv, "ringkasan_trial.csv", "text/csv", width="stretch")
        c[1].download_button("ringkasan_kelompok.csv", kel_csv, "ringkasan_kelompok.csv", "text/csv", width="stretch")
        c[2].download_button("uji_statistik.txt", teks_uji.encode("utf-8"), "uji_statistik.txt", "text/plain",
                             width="stretch")
        bp = A.baris_pemicu(hasil)
        c[3].download_button("pemicu_switching.csv", pd.DataFrame(bp).to_csv(index=False).encode("utf-8") if bp else b"",
                             "pemicu_switching.csv", "text/csv", disabled=not bp, width="stretch")
        with st.expander("Pratinjau uji_statistik.txt"):
            st.code(teks_uji, language=None)