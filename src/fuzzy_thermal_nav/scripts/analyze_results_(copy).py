#!/usr/bin/env python3
# ============================================================================
# analyze_results.py  -  Ubah results.csv menjadi tabel + grafik untuk BAB 4
# ----------------------------------------------------------------------------
# Pakai:
#   python3 analyze_results.py --data ~/TA/final_ws/experiment_data
#
# Membaca experiment_data/<SCENARIO>/results.csv untuk tiap skenario yang ada
# (S1..S8), lalu menghasilkan:
#   - summary_table.csv  : ringkasan per skenario (mean +/- std)
#   - fig_success_rate.png, fig_collision_rate.png, fig_hit_cat.png
#   - fig_time_box.png, fig_path_box.png, fig_clearance_box.png,
#     fig_smoothness_box.png
# Semua disimpan di <data>/figures/
#
# Tidak butuh ROS. Butuh: pandas, matplotlib, numpy.
# ============================================================================

import os
import argparse
import numpy as np
import pandas as pd
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

SCENARIOS = ["S1", "S2", "S3", "S4", "S5", "S6", "S7", "S8"]


def load_all(data_dir):
    frames = []
    for s in SCENARIOS:
        f = os.path.join(data_dir, s, "results.csv")
        if os.path.exists(f):
            try:
                df = pd.read_csv(f)
                df["scenario"] = s
                frames.append(df)
            except Exception as e:
                print(f"  ! gagal baca {f}: {e}")
    if not frames:
        raise SystemExit(f"Tidak ada results.csv di {data_dir}/<SCENARIO>/")
    return pd.concat(frames, ignore_index=True)


def rate(series):
    return 100.0 * series.sum() / max(len(series), 1)


def summarize(df):
    rows = []
    for s in SCENARIOS:
        d = df[df["scenario"] == s]
        if len(d) == 0:
            continue
        coll = (d["collision_count"] > 0).astype(int) if "collision_count" in d else pd.Series([0]*len(d))
        hitcat = d["hit_cat"] if "hit_cat" in d else pd.Series([0]*len(d))
        rows.append({
            "scenario": s,
            "n": len(d),
            "success_rate_%": round(rate(d["success"]), 1),
            "collision_rate_%": round(rate(coll), 1),
            "hit_cat_rate_%": round(rate(hitcat), 1),
            "time_mean": round(d["time_sec"].mean(), 1),
            "time_std": round(d["time_sec"].std(ddof=0), 1),
            "path_mean": round(d["path_length_m"].mean(), 2),
            "path_std": round(d["path_length_m"].std(ddof=0), 2),
            "min_clear_mean": round(d.get("min_clearance_m", pd.Series([np.nan])).mean(), 3),
            "avg_ang_mean": round(d["avg_angular_vel"].mean(), 3),
            "recov_mean": round(d["recovery_count"].mean(), 2),
        })
    return pd.DataFrame(rows)


def bar(df, col, ylabel, title, outpath, pct=False):
    d = df.dropna(subset=[col])
    if d.empty:
        return
    fig, ax = plt.subplots(figsize=(7, 4))
    bars = ax.bar(d["scenario"], d[col], color="#4C72B0", edgecolor="black", linewidth=0.6)
    ax.set_ylabel(ylabel)
    ax.set_title(title)
    if pct:
        ax.set_ylim(0, 105)
    for b, v in zip(bars, d[col]):
        ax.text(b.get_x() + b.get_width() / 2, v + (1 if pct else max(d[col]) * 0.01),
                f"{v:.0f}" if pct else f"{v:.2f}", ha="center", va="bottom", fontsize=9)
    ax.grid(axis="y", alpha=0.3)
    fig.tight_layout()
    fig.savefig(outpath, dpi=160)
    plt.close(fig)
    print("  +", os.path.basename(outpath))


def box(rawdf, col, ylabel, title, outpath):
    if col not in rawdf:
        return

    # Collect data per scenario (in order)
    data = []
    labels = []
    for s in SCENARIOS:
        arr = rawdf[rawdf["scenario"] == s][col].dropna().values
        if len(arr) > 0:
            data.append(arr)
            labels.append(s)

    if not data:
        return

    # Build stats dict for each group (for ax.bxp)
    bxp_stats = []
    for arr in data:
        q1 = np.percentile(arr, 25)
        q3 = np.percentile(arr, 75)
        iqr = q3 - q1
        lower = q1 - 1.5 * iqr
        upper = q3 + 1.5 * iqr
        fliers = arr[(arr < lower) | (arr > upper)]
        # whiskers: min/max within fences
        whisker_low = arr[arr >= lower].min() if np.any(arr >= lower) else q1
        whisker_high = arr[arr <= upper].max() if np.any(arr <= upper) else q3
        bxp_stats.append({
            'med': np.median(arr),
            'q1': q1,
            'q3': q3,
            'whislo': whisker_low,
            'whishi': whisker_high,
            'fliers': fliers,
            'mean': np.mean(arr)          # used for mean marker
        })

    fig, ax = plt.subplots(figsize=(7, 4))
    positions = np.arange(1, len(labels) + 1)

    # Draw boxplots with means
    ax.bxp(bxp_stats, positions=positions, showmeans=True)
    ax.set_xticks(positions)
    ax.set_xticklabels(labels)
    ax.set_ylabel(ylabel)
    ax.set_title(title)
    ax.grid(axis='y', alpha=0.3)

    fig.tight_layout()
    fig.savefig(outpath, dpi=160)
    plt.close(fig)
    print("  +", os.path.basename(outpath))

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--data", default=os.path.expanduser("~/TA/final_ws/experiment_data"))
    args = ap.parse_args()

    raw = load_all(args.data)
    figdir = os.path.join(args.data, "figures")
    os.makedirs(figdir, exist_ok=True)

    summary = summarize(raw)
    print("\n===== RINGKASAN PER SKENARIO =====")
    print(summary.to_string(index=False))
    summary.to_csv(os.path.join(figdir, "summary_table.csv"), index=False)
    print("\nTabel disimpan:", os.path.join(figdir, "summary_table.csv"))

    print("\n===== GRAFIK =====")
    bar(summary, "success_rate_%", "Success rate (%)", "Tingkat Keberhasilan per Skenario",
        os.path.join(figdir, "fig_success_rate.png"), pct=True)
    bar(summary, "collision_rate_%", "Collision rate (%)", "Tingkat Tabrakan per Skenario",
        os.path.join(figdir, "fig_collision_rate.png"), pct=True)
    bar(summary, "hit_cat_rate_%", "Hit-cat rate (%)", "Tabrakan dengan Kucing (blind-spot) per Skenario",
        os.path.join(figdir, "fig_hit_cat.png"), pct=True)

    box(raw, "time_sec", "Waktu (detik)", "Waktu Tempuh per Skenario",
        os.path.join(figdir, "fig_time_box.png"))
    box(raw, "path_length_m", "Panjang lintasan (m)", "Panjang Lintasan per Skenario",
        os.path.join(figdir, "fig_path_box.png"))
    box(raw, "min_clearance_m", "Min clearance (m)  (negatif = menembus)",
        "Jarak Aman Minimum per Skenario", os.path.join(figdir, "fig_clearance_box.png"))
    box(raw, "avg_angular_vel", "Rata-rata |kecepatan sudut| (rad/s)",
        "Kehalusan Gerak per Skenario", os.path.join(figdir, "fig_smoothness_box.png"))

    print("\nSelesai. Semua gambar ada di:", figdir)


if __name__ == "__main__":
    main()
