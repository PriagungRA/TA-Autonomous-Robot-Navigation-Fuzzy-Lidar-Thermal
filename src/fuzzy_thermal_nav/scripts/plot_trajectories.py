#!/usr/bin/env python3
# ============================================================================
# plot_trajectories.py  -  Gambar BUKTI VISUAL lintasan robot untuk BAB 4
# ----------------------------------------------------------------------------
# Membaca traj_trial_*.csv + obstacles.csv (dihasilkan experiment_logger.py)
# lalu menggambar semua lintasan trial di atas obstacle. Titik di mana robot
# MENEMBUS obstacle ditandai X merah. Kucing diberi warna merah menonjol.
#
# Pakai:
#   python3 plot_trajectories.py --data ~/TA/final_ws/experiment_data --scenario S3
#   python3 plot_trajectories.py --data ~/TA/final_ws/experiment_data --all
#
# Opsi peta latar (kalau mau):
#   --map ~/TA/final_ws/src/asr_navigation/maps/grit_1_edited_2.yaml
#
# Tidak butuh ROS. Butuh: numpy, matplotlib, pyyaml (untuk peta, opsional).
# ============================================================================

import os
import glob
import argparse
import csv
import math
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

ROBOT_RADIUS = 0.27   # samakan dgn logger (radius keliling)


def read_obstacles(scen_dir):
    f = os.path.join(scen_dir, "obstacles.csv")
    obs = []
    if os.path.exists(f):
        with open(f) as fh:
            for r in csv.DictReader(fh):
                obs.append((r["name"], float(r["x"]), float(r["y"]), float(r["radius"])))
    return obs


def read_traj(path):
    xs, ys = [], []
    with open(path) as fh:
        for r in csv.DictReader(fh):
            xs.append(float(r["x"])); ys.append(float(r["y"]))
    return np.array(xs), np.array(ys)


def try_load_map(map_yaml):
    """Kembalikan (img, extent) bila peta bisa dibaca, else None."""
    if not map_yaml or not os.path.exists(map_yaml):
        return None
    try:
        import yaml
        with open(map_yaml) as f:
            m = yaml.safe_load(f)
        img_path = m["image"]
        if not os.path.isabs(img_path):
            img_path = os.path.join(os.path.dirname(map_yaml), img_path)
        img = plt.imread(img_path)
        res = m["resolution"]; ox, oy = m["origin"][0], m["origin"][1]
        h, w = img.shape[0], img.shape[1]
        extent = [ox, ox + w * res, oy, oy + h * res]
        return (img, extent)
    except Exception as e:
        print("  ! peta tidak bisa dimuat:", e)
        return None


def plot_scenario(data_dir, scenario, map_yaml=None, dynamic=None):
    dynamic = set(dynamic or [])
    scen_dir = os.path.join(data_dir, scenario)
    traj_files = sorted(glob.glob(os.path.join(scen_dir, "traj_trial_*.csv")))
    if not traj_files:
        print(f"  ! tidak ada traj untuk {scenario}")
        return

    obs = read_obstacles(scen_dir)
    fig, ax = plt.subplots(figsize=(8, 8))

    bg = try_load_map(map_yaml)
    if bg:
        img, extent = bg
        ax.imshow(img, cmap="gray", extent=extent, origin="upper", alpha=0.5, zorder=0)

    # obstacle
    for name, ox, oy, r in obs:
        is_cat = ("cat" in name)
        is_dyn = (name in dynamic)
        col = "#D62728" if is_cat else "#7F7F7F"
        ax.add_patch(plt.Circle((ox, oy), r, color=col, alpha=0.55, zorder=2,
                                ls="--" if is_dyn else "-"))
        lbl = "KUCING" if is_cat else name.replace("obstacle_", "")
        if is_dyn:
            lbl += "\n(bergerak)"
        ax.text(ox, oy, lbl, ha="center", va="center", fontsize=7, zorder=3)

    # lintasan tiap trial
    n_coll = 0
    for k, tf in enumerate(traj_files):
        xs, ys = read_traj(tf)
        if len(xs) == 0:
            continue
        ax.plot(xs, ys, lw=1.2, alpha=0.7, zorder=4)
        # start & goal sekali
        if k == 0:
            ax.plot(xs[0], ys[0], "o", color="green", ms=10, zorder=5, label="start")
            ax.plot(xs[-1], ys[-1], "*", color="black", ms=12, zorder=5, label="akhir")
        # tandai titik menembus obstacle (lewati obstacle bergerak: posisinya tak reliable)
        for name, ox, oy, r in obs:
            if name in dynamic:
                continue
            d = np.hypot(xs - ox, ys - oy)
            hit = d < (ROBOT_RADIUS + r)
            if hit.any():
                idx = np.argmin(d)
                ax.plot(xs[idx], ys[idx], "x", color="red", ms=11, mew=2.5, zorder=6)
                n_coll += 1

    # legend bersih (tanpa duplikat)
    handles, labels = ax.get_legend_handles_labels()
    uniq = dict(zip(labels, handles))
    ax.plot([], [], "x", color="red", ms=10, mew=2.5, label="tabrakan")
    handles, labels = ax.get_legend_handles_labels()
    uniq = dict(zip(labels, handles))
    ax.legend(uniq.values(), uniq.keys(), loc="best", fontsize=9)

    ax.set_aspect("equal", adjustable="datalim")
    ax.set_xlabel("x (m)"); ax.set_ylabel("y (m)")
    ax.set_title(f"Lintasan {scenario} ({len(traj_files)} trial) — X merah = menembus obstacle")
    ax.grid(alpha=0.3)
    fig.tight_layout()

    figdir = os.path.join(data_dir, "figures")
    os.makedirs(figdir, exist_ok=True)
    out = os.path.join(figdir, f"traj_{scenario}.png")
    fig.savefig(out, dpi=160)
    plt.close(fig)
    print(f"  + {os.path.basename(out)}  ({len(traj_files)} trial, {n_coll} titik tabrakan)")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--data", default=os.path.expanduser("~/TA/final_ws/experiment_data"))
    ap.add_argument("--scenario", default=None)
    ap.add_argument("--all", action="store_true")
    ap.add_argument("--map", default=None, help="path map .yaml (opsional, untuk latar)")
    ap.add_argument("--dynamic", default="",
                    help="nama obstacle bergerak, dipisah koma (X tabrakan tdk ditandai utk ini). "
                         "Contoh utk S4: obstacle_human_1,obstacle_human_2")
    args = ap.parse_args()
    dyn = [s.strip() for s in args.dynamic.split(",") if s.strip()]

    scens = ["S1", "S2", "S3", "S4", "S5", "S6", "S7", "S8"] if args.all else [args.scenario]
    scens = [s for s in scens if s]
    if not scens:
        raise SystemExit("Beri --scenario S3  atau  --all")

    print("=== PLOT LINTASAN ===")
    for s in scens:
        if os.path.isdir(os.path.join(args.data, s)):
            plot_scenario(args.data, s, args.map, dyn)


if __name__ == "__main__":
    main()