#!/usr/bin/env python3
# plot_trajectories_dynamic.py  (versi + X tabrakan ground-truth)
import os, glob, csv, argparse
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.collections import LineCollection

ROBOT_RADIUS_COLL = 0.20   # samakan dgn logger utk deteksi tabrakan
HUMAN_SWEEP = {
    "obstacle_human_1": {"A": (5.268695, -0.186458), "B": (3.084140, 1.964217), "r": 0.25},
    "obstacle_human_2": {"A": (7.245282,  1.757134), "B": (4.921940, 3.999455), "r": 0.25},
}

def read_obstacles(scen_dir):
    f = os.path.join(scen_dir, "obstacles.csv"); out = {}
    if os.path.exists(f):
        for row in csv.DictReader(open(f)):
            out[row["name"]] = (float(row["x"]), float(row["y"]), float(row["radius"]))
    return out

def read_traj(path):
    t,x,y = [],[],[]
    for row in csv.DictReader(open(path)):
        t.append(float(row["t"])); x.append(float(row["x"])); y.append(float(row["y"]))
    return np.array(t), np.array(x), np.array(y)

def read_obs_track(scen_dir, tid):
    f = os.path.join(scen_dir, f"traj_obs_trial_{tid}.csv")
    if not os.path.exists(f): return None
    d = {}
    for row in csv.DictReader(open(f)):
        d.setdefault(row["name"], []).append((float(row["t"]), float(row["x"]), float(row["y"])))
    return {k: np.array(v) for k, v in d.items()}

def obstacle_radii(scen_dir):
    rad = {n: r for n,(_,_,r) in read_obstacles(scen_dir).items()}
    for n, sw in HUMAN_SWEEP.items(): rad.setdefault(n, sw["r"])
    return rad

def detect_collisions(scen_dir, tid, traj, rad):
    otrk = read_obs_track(scen_dir, tid)
    if not otrk: return []
    t,x,y = traj; hits = []
    for name, arr in otrk.items():
        r = rad.get(name, 0.25); ot = arr[:,0]
        idx = np.clip(np.searchsorted(ot, t), 0, len(ot)-1)
        d = np.hypot(x-arr[idx,1], y-arr[idx,2])
        clr = d - ROBOT_RADIUS_COLL - r; j = int(np.argmin(clr))
        if clr[j] < 0.0: hits.append((x[j], y[j], name, clr[j]))
    return hits

def draw_sweep_band(ax, A, B, r, label=None):
    A=np.array(A); B=np.array(B); d=B-A; L=np.hypot(*d); n=np.array([-d[1],d[0]])/L
    poly=np.array([A+n*r, B+n*r, B-n*r, A-n*r])
    ax.fill(poly[:,0], poly[:,1], color="red", alpha=0.13, zorder=1, label=label)
    for c in (A,B): ax.add_patch(plt.Circle(c, r, color="red", alpha=0.13, zorder=1))
    ax.plot([A[0],B[0]],[A[1],B[1]], color="red", ls="--", lw=1.0, alpha=0.5, zorder=1)

def time_colored_line(ax, x, y, cmap="viridis", lw=1.6, alpha=0.85):
    pts=np.array([x,y]).T.reshape(-1,1,2); segs=np.concatenate([pts[:-1],pts[1:]],axis=1)
    lc=LineCollection(segs, cmap=cmap, alpha=alpha, zorder=4)
    lc.set_array(np.linspace(0,1,len(segs))); lc.set_linewidth(lw); ax.add_collection(lc); return lc

def plot_scenario(data_dir, scenario, mark_collisions=True):
    scen_dir = os.path.join(data_dir, scenario)
    traj_files = sorted(glob.glob(os.path.join(scen_dir,"traj_trial_*.csv")),
                        key=lambda p:int(p.split("_")[-1].split(".")[0]))
    if not traj_files: print(f"  (tidak ada traj di {scen_dir})"); return
    obs = read_obstacles(scen_dir); rad = obstacle_radii(scen_dir)
    fig, ax = plt.subplots(figsize=(8.5,8))
    first=True
    for name, sw in HUMAN_SWEEP.items():
        draw_sweep_band(ax, sw["A"], sw["B"], sw["r"],
                        label="koridor human (sapuan)" if first else None); first=False
    for name,(ox,oy,r) in obs.items():
        if name in HUMAN_SWEEP: continue
        is_cat=(name=="obstacle_cat")
        ax.add_patch(plt.Circle((ox,oy), r, color="red" if is_cat else "dimgray", alpha=0.6, zorder=2))
        ax.text(ox,oy,"KUCING" if is_cat else name.replace("obstacle_",""),
                fontsize=7, ha="center", va="center", zorder=3)
    last_lc, coll_pts = None, []
    for k,tf in enumerate(traj_files):
        tid=int(tf.split("_")[-1].split(".")[0]); t,x,y = read_traj(tf)
        last_lc = time_colored_line(ax,x,y)
        if k==0:
            ax.plot(x[0],y[0],"o",color="lime",ms=11,zorder=6,label="start")
            ax.plot(x[-1],y[-1],"*",color="black",ms=14,zorder=6,label="goal")
        otrk=read_obs_track(scen_dir,tid)
        if otrk:
            for name,arr in otrk.items():
                if name in HUMAN_SWEEP:
                    ax.plot(arr[:,1],arr[:,2],color="orange",lw=0.5,alpha=0.30,zorder=3)
        if mark_collisions: coll_pts += detect_collisions(scen_dir, tid, (t,x,y), rad)
    for (cx,cy,name,clr) in coll_pts:
        ax.plot(cx,cy,marker="X",color="red",ms=15,mew=2.0,markeredgecolor="black",zorder=7)
    if coll_pts:
        ax.plot([],[],marker="X",color="red",ms=12,markeredgecolor="black",lw=0,
                label=f"tabrakan ground-truth (n={len(coll_pts)})")
    if last_lc is not None:
        cb=fig.colorbar(last_lc, ax=ax, fraction=0.045, pad=0.02)
        cb.set_label("waktu relatif (start \u2192 goal)")
    ax.set_aspect("equal"); ax.grid(alpha=0.3)
    ax.set_xlabel("x (m)"); ax.set_ylabel("y (m)")
    ax.set_title(f"Lintasan {scenario} ({len(traj_files)} trial)\n"
                 f"warna=waktu | pita merah=koridor sapuan | X=tabrakan nyata ({len(coll_pts)})")
    ax.legend(loc="upper left", fontsize=8)
    out=os.path.join(scen_dir, f"traj_{scenario}_dynamic.png")
    fig.tight_layout(); fig.savefig(out, dpi=150); plt.close(fig)
    print(f"  + {out}  ({len(traj_files)} trial, {len(coll_pts)} tabrakan ground-truth)")

if __name__=="__main__":
    ap=argparse.ArgumentParser()
    ap.add_argument("--data", default=os.path.expanduser("~/TA/final_ws/experiment_data"))
    ap.add_argument("--scenario", default="S8")
    ap.add_argument("--no-collisions", action="store_true")
    a=ap.parse_args()
    print("=== PLOT LINTASAN DINAMIS ===")
    plot_scenario(a.data, a.scenario, mark_collisions=not a.no_collisions)