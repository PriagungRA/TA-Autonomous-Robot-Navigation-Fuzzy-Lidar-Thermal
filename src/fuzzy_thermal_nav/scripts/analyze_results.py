#!/usr/bin/env python3
# ============================================================================
# analyze_results.py  -  Analisis hasil eksperimen gaya jurnal (Scopus Q1)
# ----------------------------------------------------------------------------
# Fokus: perbandingan BERPASANGAN baseline(non-fuzzy) vs fuzzy di world yang
# sama (S1/S5, S2/S6, S3/S7, S4/S8), dengan uji Mann-Whitney U + effect size,
# tabel siap-LaTeX, dan figure publikasi (PDF vektor + PNG 300 dpi).
#
# Pakai:
#   pip install pandas numpy matplotlib scipy
#   python3 analyze_results.py --data ~/TA/final_ws/experiment_data
#
# Output (di <data>/figures_pub/):
#   summary_table.csv / .tex        ringkasan mean+/-std per skenario
#   pairwise_stats.csv / .tex       uji pasangan (median, U, p, r, signifikansi)
#   fig_success_collision.(pdf|png) success & collision-free (grouped bar)
#   fig_clearance.(pdf|png)         min-clearance berpasangan (headline)
#   fig_time.(pdf|png)              waktu tempuh berpasangan (trade-off)
#   fig_panel.(pdf|png)             panel 2x2: clearance, time, linV, angV
# ============================================================================

import os, argparse, warnings
import numpy as np
import pandas as pd
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import Patch
try:
    from scipy.stats import mannwhitneyu
    HAVE_SCIPY = True
except Exception:
    HAVE_SCIPY = False
    warnings.warn("scipy tidak ada -> p-value tidak dihitung. `pip install scipy`")

# ---- Pasangan baseline(non-fuzzy) -> fuzzy (world sama) --------------------
PAIRS = [
    ("S1", "S5", "Open"),
    ("S2", "S6", "Single obstacle"),
    ("S3", "S7", "Static multi + cat"),
    ("S4", "S8", "Dynamic (moving human)"),
]
SCEN_ORDER = [s for p in PAIRS for s in p[:2]]

# ---- Palet colorblind-safe (Okabe-Ito) ------------------------------------
C_BASE = "#0072B2"   # biru  = baseline (non-fuzzy)
C_FUZZY = "#D55E00"  # vermillion = fuzzy
GRID = "#B0B0B0"

# ---- Gaya publikasi -------------------------------------------------------
def set_pub_style():
    plt.rcParams.update({
        "font.family": "serif",
        "font.serif": ["Times New Roman", "DejaVu Serif", "Nimbus Roman"],
        "mathtext.fontset": "dejavuserif",
        "font.size": 10,
        "axes.titlesize": 11,
        "axes.labelsize": 10,
        "xtick.labelsize": 9,
        "ytick.labelsize": 9,
        "legend.fontsize": 9,
        "axes.linewidth": 0.8,
        "axes.edgecolor": "#333333",
        "axes.grid": True,
        "grid.color": GRID,
        "grid.linewidth": 0.5,
        "grid.alpha": 0.4,
        "axes.axisbelow": True,
        "figure.dpi": 120,
        "savefig.dpi": 300,
        "savefig.bbox": "tight",
        "pdf.fonttype": 42,   # font ter-embed (wajib utk submission)
        "ps.fonttype": 42,
    })

def stars(p):
    if p is None or np.isnan(p): return "n/a"
    if p < 0.001: return "***"
    if p < 0.01:  return "**"
    if p < 0.05:  return "*"
    return "ns"

def mwu(a, b):
    """Return (U, p, rank-biserial effect size). NaN-safe."""
    a = np.asarray(a, float); b = np.asarray(b, float)
    a = a[~np.isnan(a)]; b = b[~np.isnan(b)]
    if not HAVE_SCIPY or len(a) < 2 or len(b) < 2:
        return np.nan, np.nan, np.nan
    if np.all(a == a[0]) and np.all(b == b[0]) and a[0] == b[0]:
        return np.nan, np.nan, np.nan
    U, p = mannwhitneyu(a, b, alternative="two-sided")
    r = 1.0 - (2.0 * U) / (len(a) * len(b))   # rank-biserial correlation
    return U, p, r


def load_all(data_dir):
    frames = []
    for s in SCEN_ORDER:
        f = os.path.join(data_dir, s, "results.csv")
        if os.path.exists(f):
            df = pd.read_csv(f)
            df["scenario"] = s
            frames.append(df)
    if not frames:
        raise SystemExit(f"Tidak ada results.csv di {data_dir}/<SCENARIO>/")
    return pd.concat(frames, ignore_index=True)


# ---------------------------------------------------------------- TABEL 1
def summary_table(df, figdir):
    rows = []
    for s in SCEN_ORDER:
        d = df[df.scenario == s]
        if d.empty: continue
        def ms(col, dec=2):
            if col not in d: return "-"
            return f"{d[col].mean():.{dec}f} $\\pm$ {d[col].std(ddof=1):.{dec}f}"
        rows.append({
            "Scenario": s,
            "n": len(d),
            "Success (%)": round(100*d.success.mean(), 0),
            "Coll.-free (%)": round(100*d.collision_free.mean(), 0),
            "Time (s)": ms("time_sec", 1),
            "Path (m)": ms("path_length_m", 2),
            "Min clear. (m)": ms("min_clearance_m", 3),
            "Lin. vel (m/s)": ms("avg_linear_vel", 3),
            "Ang. vel (rad/s)": ms("avg_angular_vel", 3),
        })
    t = pd.DataFrame(rows)
    t.to_csv(os.path.join(figdir, "summary_table.csv"), index=False)
    with open(os.path.join(figdir, "summary_table.tex"), "w") as f:
        f.write(t.to_latex(index=False, escape=False, column_format="l"+"r"*(len(t.columns)-1)))
    print("\n===== TABEL 1: ringkasan per skenario ====="); print(t.to_string(index=False))
    return t


# ---------------------------------------------------------------- TABEL 2
def pairwise_table(df, figdir):
    metrics = [("min_clearance_m","Min clearance (m)","high"),
               ("time_sec","Time (s)","low"),
               ("avg_linear_vel","Lin. vel (m/s)","-"),
               ("avg_angular_vel","Ang. vel (rad/s)","low"),
               ("path_length_m","Path length (m)","low")]
    rows = []
    for b, fz, name in PAIRS:
        db, dfz = df[df.scenario==b], df[df.scenario==fz]
        if db.empty or dfz.empty: continue
        for col, mlabel, _ in metrics:
            if col not in df: continue
            a, c = db[col].values, dfz[col].values
            U, p, r = mwu(a, c)
            rows.append({
                "Pair": f"{b} vs {fz}",
                "Environment": name,
                "Metric": mlabel,
                "Baseline (median)": round(np.nanmedian(a), 3),
                "Fuzzy (median)": round(np.nanmedian(c), 3),
                "U": "" if np.isnan(U) else int(U),
                "p": "" if np.isnan(p) else f"{p:.4f}",
                "r": "" if np.isnan(r) else f"{r:+.2f}",
                "Sig.": stars(p),
            })
    t = pd.DataFrame(rows)
    t.to_csv(os.path.join(figdir, "pairwise_stats.csv"), index=False)
    with open(os.path.join(figdir, "pairwise_stats.tex"), "w") as f:
        f.write(t.to_latex(index=False, escape=True, column_format="lllrrrrrc"))
    print("\n===== TABEL 2: uji pasangan (Mann-Whitney U) =====")
    print(t.to_string(index=False))
    return t


# ---------------------------------------------------------- helper plot
def _sig_bracket(ax, x1, x2, y, text, lw=1.0):
    h = (ax.get_ylim()[1]-ax.get_ylim()[0]) * 0.02
    ax.plot([x1,x1,x2,x2], [y, y+h, y+h, y], lw=lw, c="#333333")
    ax.text((x1+x2)/2, y+h, text, ha="center", va="bottom", fontsize=9)

def _paired_box(ax, df, col, ylabel, short_labels=False):
    """Boxplot berpasangan baseline|fuzzy per environment + titik data + signifikansi.
    Plot each box individually to avoid NumPy inhomogeneous‑shape errors."""
    positions, centers, xticklabels = [], [], []
    pos = 1.0
    # First pass: collect positions and store data for significance brackets
    pair_data = []
    for b, fz, name in PAIRS:
        a = df[df.scenario == b][col].dropna().values
        c = df[df.scenario == fz][col].dropna().values
        if len(a) == 0 or len(c) == 0:
            continue
        positions.extend([pos, pos + 0.7])
        centers.append(pos + 0.35)
        xticklabels.append(f"{b}/{fz}" if short_labels else f"{name}\n({b}/{fz})")
        pair_data.append((a, c, pos))
        pos += 2.0

    # Plot each box individually
    rng = np.random.default_rng(0)
    for idx, (a, c, p) in enumerate(pair_data):
        # Baseline box
        bp = ax.boxplot([a], positions=[p], widths=0.55, patch_artist=True,
                        showmeans=True,
                        meanprops=dict(marker="D", markerfacecolor="white",
                                       markeredgecolor="black", markersize=4),
                        medianprops=dict(color="black", lw=1.2),
                        flierprops=dict(marker="o", markersize=3, alpha=0.4))
        for patch in bp["boxes"]:
            patch.set_facecolor(C_BASE)
            patch.set_alpha(0.55)
            patch.set_edgecolor("#333333")
        # Fuzzy box
        bp = ax.boxplot([c], positions=[p + 0.7], widths=0.55, patch_artist=True,
                        showmeans=True,
                        meanprops=dict(marker="D", markerfacecolor="white",
                                       markeredgecolor="black", markersize=4),
                        medianprops=dict(color="black", lw=1.2),
                        flierprops=dict(marker="o", markersize=3, alpha=0.4))
        for patch in bp["boxes"]:
            patch.set_facecolor(C_FUZZY)
            patch.set_alpha(0.55)
            patch.set_edgecolor("#333333")
        # Scatter points with jitter
        for x, data, color in zip([p, p + 0.7], [a, c], [C_BASE, C_FUZZY]):
            if len(data) > 0:
                ax.scatter(x + rng.uniform(-0.13, 0.13, len(data)),
                           data, s=14, color=color,
                           edgecolor="white", linewidth=0.4, zorder=3, alpha=0.9)

    # Significance brackets (per pair)
    for (a, c, p) in pair_data:
        _, pval, _ = mwu(a, c)
        allv = np.concatenate([a, c])
        ytop = np.nanmax(allv) if len(allv) else 0
        _sig_bracket(ax, p, p + 0.7,
                     ytop + (ax.get_ylim()[1] * 0.03 + 1e-6),
                     stars(pval))

    ax.set_xticks(centers)
    ax.set_xticklabels(xticklabels)
    ax.set_ylabel(ylabel)
    ax.margins(y=0.18)

def fig_success_collision(df, figdir):
    set_pub_style()
    fig, ax = plt.subplots(figsize=(7.0, 3.2))
    names = [p[2] for p in PAIRS]; x = np.arange(len(PAIRS)); w = 0.38
    base = [100*df[df.scenario==p[0]].success.mean() for p in PAIRS]
    fuz  = [100*df[df.scenario==p[1]].success.mean() for p in PAIRS]
    b1 = ax.bar(x-w/2, base, w, label="Baseline (non-fuzzy)", color=C_BASE, edgecolor="#222222", lw=0.6)
    b2 = ax.bar(x+w/2, fuz,  w, label="Proposed (fuzzy)", color=C_FUZZY, edgecolor="#222222", lw=0.6)
    for bars in (b1, b2):
        for bar in bars:
            ax.text(bar.get_x()+bar.get_width()/2, bar.get_height()+1,
                    f"{bar.get_height():.0f}", ha="center", va="bottom", fontsize=8)
    ax.set_xticks(x); ax.set_xticklabels([f"{n}\n({p[0]}/{p[1]})" for n,p in zip(names,PAIRS)])
    ax.set_ylabel("Success rate (%)"); ax.set_ylim(0, 112)
    ax.legend(frameon=False, ncol=2, loc="lower center", bbox_to_anchor=(0.5, 1.02))
    fig.tight_layout()
    for ext in ("pdf","png"):
        fig.savefig(os.path.join(figdir, f"fig_success_collision.{ext}"))
    plt.close(fig); print("  + fig_success_collision.(pdf|png)")


def fig_single(df, col, ylabel, fname, figdir):
    set_pub_style()
    fig, ax = plt.subplots(figsize=(7.0, 3.6))
    _paired_box(ax, df, col, ylabel)
    leg = [Patch(facecolor=C_BASE, alpha=0.55, edgecolor="#333333", label="Baseline (non-fuzzy)"),
           Patch(facecolor=C_FUZZY, alpha=0.55, edgecolor="#333333", label="Proposed (fuzzy)")]
    ax.legend(handles=leg, frameon=False, ncol=2, loc="lower center", bbox_to_anchor=(0.5, 1.02))
    fig.tight_layout()
    for ext in ("pdf","png"):
        fig.savefig(os.path.join(figdir, f"{fname}.{ext}"))
    plt.close(fig); print(f"  + {fname}.(pdf|png)")


def fig_panel(df, figdir):
    set_pub_style()
    panels = [("min_clearance_m","Minimum clearance (m)","(a)"),
              ("time_sec","Traversal time (s)","(b)"),
              ("avg_linear_vel","Mean linear velocity (m/s)","(c)"),
              ("avg_angular_vel","Mean |angular velocity| (rad/s)","(d)")]
    fig, axes = plt.subplots(2, 2, figsize=(10.5, 7.4))
    for ax, (col, ylab, tag) in zip(axes.ravel(), panels):
        _paired_box(ax, df, col, ylab, short_labels=True)
        ax.set_title(tag, loc="left", fontweight="bold")
    leg = [Patch(facecolor=C_BASE, alpha=0.55, edgecolor="#333333", label="Baseline (non-fuzzy)"),
           Patch(facecolor=C_FUZZY, alpha=0.55, edgecolor="#333333", label="Proposed (fuzzy)")]
    fig.legend(handles=leg, frameon=False, ncol=2, loc="upper center", bbox_to_anchor=(0.5, 1.02))
    fig.tight_layout(rect=[0,0,1,0.98])
    for ext in ("pdf","png"):
        fig.savefig(os.path.join(figdir, f"fig_panel.{ext}"))
    plt.close(fig); print("  + fig_panel.(pdf|png)")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--data", default=os.path.expanduser("~/TA/final_ws/experiment_data"))
    args = ap.parse_args()
    raw = load_all(args.data)
    figdir = os.path.join(args.data, "figures_pub"); os.makedirs(figdir, exist_ok=True)

    summary_table(raw, figdir)
    pairwise_table(raw, figdir)
    print("\n===== FIGURE =====")
    fig_success_collision(raw, figdir)
    fig_single(raw, "min_clearance_m", "Minimum clearance (m)", "fig_clearance", figdir)
    fig_single(raw, "time_sec", "Traversal time (s)", "fig_time", figdir)
    fig_panel(raw, figdir)
    print("\nSelesai. Semua output di:", figdir)
    if not HAVE_SCIPY:
        print("CATATAN: scipy belum terpasang -> p-value kosong. `pip install scipy` lalu jalankan lagi.")


if __name__ == "__main__":
    main()