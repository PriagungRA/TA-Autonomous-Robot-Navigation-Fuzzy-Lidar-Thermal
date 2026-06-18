#!/usr/bin/env python3
"""
Analisis S7 dengan path refinement baru.
Cek apakah waypoint sudah digeser dan clearance kucing meningkat.
"""

import os
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Circle, Rectangle
import glob

# =====================================
# CONFIG
# =====================================
S7_DIR = "/home/ubuntu/TA/final_ws/experiment_data/S7"
NUM_TRIALS = 10

CAT_POS_X = 2.8  # approximate cat center (dari log)
CAT_POS_Y = 0.65
CAT_RADIUS = 0.20

ROBOT_RADIUS = 0.32
SAFE_MARGIN = 0.25
SAFE_DIST = CAT_RADIUS + ROBOT_RADIUS + SAFE_MARGIN  # 0.77m

# =====================================
# LOAD DATA
# =====================================
print(f"📂 Loading S7 data from {S7_DIR}...")

traj_files = sorted(glob.glob(f"{S7_DIR}/traj_trial_*.csv"))
print(f"✓ Found {len(traj_files)} trajectory files")

results_file = f"{S7_DIR}/results.csv"
if os.path.exists(results_file):
    results_df = pd.read_csv(results_file)
    print(f"✓ Results CSV loaded: {len(results_df)} trials")
    print(results_df.head())
else:
    print("⚠ Results CSV not found")

# =====================================
# ANALYZE TRAJECTORIES
# =====================================
print("\n" + "="*60)
print("TRAJECTORY ANALYSIS")
print("="*60)

min_clearances = []
clearance_at_critical_zone = []
waypoints_shifted = 0

for i, traj_file in enumerate(traj_files):
    trial_num = i + 1
    
    try:
        traj_df = pd.read_csv(traj_file)
        
        if len(traj_df) == 0:
            print(f"Trial {trial_num:2d}: ⚠ Empty trajectory")
            continue
        
        # Ambil posisi robot
        robot_x = traj_df['robot_x'].values
        robot_y = traj_df['robot_y'].values
        
        # Hitung jarak minimum ke kucing
        distances = np.sqrt((robot_x - CAT_POS_X)**2 + (robot_y - CAT_POS_Y)**2)
        min_dist = np.min(distances)
        min_clearances.append(min_dist)
        
        # Clearance = jarak_min - robot_radius
        clearance = min_dist - ROBOT_RADIUS
        
        # Cek di zona kritis (x ~ 2.8-3.2, y ~ 0.5-0.8)
        in_zone = (robot_x >= 2.7) & (robot_x <= 3.2) & (robot_y >= 0.4) & (robot_y <= 0.9)
        if np.any(in_zone):
            zone_distances = distances[in_zone]
            clearance_zone = np.min(zone_distances) - ROBOT_RADIUS
            clearance_at_critical_zone.append(clearance_zone)
        else:
            clearance_at_critical_zone.append(clearance)
        
        status = "✓ SAFE" if min_dist >= SAFE_DIST else f"⚠ MEPET ({min_dist:.2f}m < {SAFE_DIST:.2f}m)"
        
        print(f"Trial {trial_num:2d}: min_dist={min_dist:.3f}m, clearance={clearance:.3f}m {status}")
        
    except Exception as e:
        print(f"Trial {trial_num:2d}: ✗ Error - {str(e)}")

print("\n" + "="*60)
print("SUMMARY")
print("="*60)
min_clearances = np.array(min_clearances)
clearance_at_critical_zone = np.array(clearance_at_critical_zone)

print(f"Min clearances (min_dist - robot_radius):")
print(f"  Mean:    {np.mean(clearance_at_critical_zone):.3f} m")
print(f"  Min:     {np.min(clearance_at_critical_zone):.3f} m")
print(f"  Max:     {np.max(clearance_at_critical_zone):.3f} m")
print(f"  Std:     {np.std(clearance_at_critical_zone):.3f} m")

safe_count = np.sum(min_clearances >= SAFE_DIST)
print(f"\n✓ Safe trials (clearance >= {SAFE_DIST:.2f}m): {safe_count}/{len(min_clearances)} ({100*safe_count/len(min_clearances):.0f}%)")

# =====================================
# PLOT
# =====================================
print("\n📊 Generating plots...")

fig, axes = plt.subplots(2, 2, figsize=(14, 12))
fig.suptitle("S7 Scenario Analysis - Thermal-Aware Global Path Refinement", fontsize=14, fontweight='bold')

# Plot 1: Clearance progression
ax = axes[0, 0]
ax.plot(range(1, len(clearance_at_critical_zone)+1), clearance_at_critical_zone, 'o-', linewidth=2, markersize=8, label='Clearance')
ax.axhline(y=SAFE_MARGIN, color='orange', linestyle='--', linewidth=2, label=f'Safe margin ({SAFE_MARGIN}m)')
ax.axhline(y=0, color='red', linestyle='--', linewidth=2, label='Collision')
ax.grid(True, alpha=0.3)
ax.set_xlabel("Trial #", fontsize=11)
ax.set_ylabel("Clearance from Cat (m)", fontsize=11)
ax.set_title("Minimum Clearance per Trial", fontweight='bold')
ax.legend()

# Plot 2: Distance to cat
ax = axes[0, 1]
ax.plot(range(1, len(min_clearances)+1), min_clearances, 's-', linewidth=2, markersize=8, color='green', label='Min distance to cat')
ax.axhline(y=SAFE_DIST, color='orange', linestyle='--', linewidth=2, label=f'Safe distance ({SAFE_DIST:.2f}m)')
ax.grid(True, alpha=0.3)
ax.set_xlabel("Trial #", fontsize=11)
ax.set_ylabel("Distance to Cat Center (m)", fontsize=11)
ax.set_title("Minimum Distance to Cat Center", fontweight='bold')
ax.legend()

# Plot 3: Trajectories with cat keep-out region
ax = axes[1, 0]

# Draw cat keep-out region
circle_keep_out = Circle((CAT_POS_X, CAT_POS_Y), SAFE_DIST, 
                         color='red', alpha=0.2, linewidth=2, linestyle='--', label='Keep-out region')
ax.add_patch(circle_keep_out)

# Draw cat body
circle_cat = Circle((CAT_POS_X, CAT_POS_Y), CAT_RADIUS, 
                    color='red', alpha=0.5, label='Cat (est.)')
ax.add_patch(circle_cat)

# Plot all trajectories
colors = plt.cm.viridis(np.linspace(0, 1, NUM_TRIALS))
for i, traj_file in enumerate(traj_files):
    try:
        traj_df = pd.read_csv(traj_file)
        ax.plot(traj_df['robot_x'], traj_df['robot_y'], 'o-', 
               alpha=0.6, markersize=3, color=colors[i], linewidth=1, label=f'T{i+1}')
    except:
        pass

ax.set_xlim(2.0, 4.0)
ax.set_ylim(-0.2, 1.5)
ax.set_aspect('equal')
ax.grid(True, alpha=0.3)
ax.set_xlabel("X (m)", fontsize=11)
ax.set_ylabel("Y (m)", fontsize=11)
ax.set_title("All Trajectories (S7 - Critical Zone)", fontweight='bold')
ax.legend(loc='upper right', fontsize=8)

# Plot 4: Histogram of clearances
ax = axes[1, 1]
ax.hist(clearance_at_critical_zone, bins=8, color='skyblue', edgecolor='black', alpha=0.7)
ax.axvline(x=SAFE_MARGIN, color='orange', linestyle='--', linewidth=2, label=f'Target margin ({SAFE_MARGIN}m)')
ax.axvline(x=0, color='red', linestyle='--', linewidth=2, label='Collision limit')
ax.axvline(x=np.mean(clearance_at_critical_zone), color='green', linestyle='-', linewidth=2, label=f'Mean ({np.mean(clearance_at_critical_zone):.3f}m)')
ax.grid(True, alpha=0.3, axis='y')
ax.set_xlabel("Clearance (m)", fontsize=11)
ax.set_ylabel("Frequency", fontsize=11)
ax.set_title("Clearance Distribution", fontweight='bold')
ax.legend()

plt.tight_layout()
plt.savefig(f"{S7_DIR}/refinement_analysis.png", dpi=150, bbox_inches='tight')
print(f"✓ Saved: {S7_DIR}/refinement_analysis.png")

# =====================================
# DETAILED TRAJECTORY COMPARISON
# =====================================
fig2, axes2 = plt.subplots(2, 5, figsize=(18, 8))
fig2.suptitle("S7 Individual Trial Trajectories (All 10 Trials)", fontsize=14, fontweight='bold')

for i, traj_file in enumerate(traj_files):
    row = i // 5
    col = i % 5
    ax = axes2[row, col]
    
    try:
        traj_df = pd.read_csv(traj_file)
        
        # Draw cat keep-out region
        circle_keep_out = Circle((CAT_POS_X, CAT_POS_Y), SAFE_DIST, 
                                color='red', alpha=0.1, linewidth=1, linestyle='--')
        ax.add_patch(circle_keep_out)
        
        # Draw cat body
        circle_cat = Circle((CAT_POS_X, CAT_POS_Y), CAT_RADIUS, 
                           color='red', alpha=0.6, linewidth=1)
        ax.add_patch(circle_cat)
        
        # Plot trajectory
        ax.plot(traj_df['robot_x'], traj_df['robot_y'], 'b-', linewidth=1.5, alpha=0.7)
        ax.plot(traj_df['robot_x'].iloc[0], traj_df['robot_y'].iloc[0], 'go', markersize=8, label='Start')
        ax.plot(traj_df['robot_x'].iloc[-1], traj_df['robot_y'].iloc[-1], 'r*', markersize=12, label='End')
        
        min_dist = np.min(np.sqrt((traj_df['robot_x'] - CAT_POS_X)**2 + (traj_df['robot_y'] - CAT_POS_Y)**2))
        clearance = min_dist - ROBOT_RADIUS
        status = "✓" if clearance >= SAFE_MARGIN else "✗"
        
        ax.set_xlim(2.0, 4.0)
        ax.set_ylim(-0.2, 1.5)
        ax.set_aspect('equal')
        ax.grid(True, alpha=0.2)
        ax.set_title(f"Trial {i+1} {status}\nClear={clearance:.3f}m", fontsize=9, fontweight='bold')
        
    except Exception as e:
        ax.text(0.5, 0.5, f"Error: {str(e)}", ha='center', va='center', transform=ax.transAxes)
        ax.set_xlim(2.0, 4.0)
        ax.set_ylim(-0.2, 1.5)

plt.tight_layout()
plt.savefig(f"{S7_DIR}/all_trials_detail.png", dpi=150, bbox_inches='tight')
print(f"✓ Saved: {S7_DIR}/all_trials_detail.png")

print("\n✅ Analysis complete!")
