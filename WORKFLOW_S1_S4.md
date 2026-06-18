# Workflow Data Collection S1-S4 (Baseline)

## Single Command Startup

Semua sudah ter-organize dalam `nav_sim.launch`. Cukup jalankan:

```bash
# S1 Open Space
roslaunch ros_sim nav_sim.launch scenario:=S1

# S2 Single Human
roslaunch ros_sim nav_sim.launch scenario:=S2

# S3 Multiple Static Obstacles
roslaunch ros_sim nav_sim.launch scenario:=S3

# S4 Dynamic Obstacles
roslaunch ros_sim nav_sim.launch scenario:=S4
```

## Apa yang terjadi otomatis:

1. ✅ **Gazebo simulation** + world (S1_open.world, S2_single_human.world, dst)
2. ✅ **Robot spawn** + localization
3. ✅ **move_base** + APF local planner (thermal-adaptive)
4. ✅ **Semantic thermal** node (image processing)
5. ✅ **Thermal feature extractor** (publishes traversability/corridor metrics)
6. ✅ **Experiment logger** (records all metrics per trial)
7. ✅ **Goal sender** (sends target automatically after 2s delay)
8. ✅ **RViz visualization**

## Data Flow:

```
Gazebo Camera
    ↓
semantic_thermal.py (/semantic_mask)
    ↓
thermal_feature_extractor.py (/traversability_score, /corridor_width, /center_path)
    ↓
APF planner (thermal-adaptive gain: rep_gain = 1.0 + (1.0 - traversability))
    ↓
move_base (/cmd_vel)
    ↓
experiment_logger.py (collision detection via /scan, thermal metrics aggregation)
    ↓
experiment_data/S1/results.csv (10 baris = 10 trial)
```

## Workflow per Scenario:

### Trial 1-10 untuk S1:
```bash
# Terminal 1
roslaunch ros_sim experiment.launch scenario:=S1

# Tunggu semua node siap (cek di terminal)
# Setelah goal sent: robot mulai navigasi, logger mencatat

# Tunggu finish (success atau timeout 120s)
# CSV S1/results.csv + 1 baris
```

### Ulangi 9x lagi untuk S1 (9 terminal baru):
```bash
roslaunch ros_sim experiment.launch scenario:=S1
```
→ Auto-append ke `S1/results.csv` (total 10 baris)

### Kemudian S2-S4 sama:
```bash
roslaunch ros_sim experiment.launch scenario:=S2
# ... repeat 10x
```

## Output Structure:

```
experiment_data/
├── S1/
│   └── results.csv  (10 rows = 10 trials)
├── S2/
│   └── results.csv  (10 rows = 10 trials)
├── S3/
│   └── results.csv
└── S4/
    └── results.csv
```

## CSV Columns (auto-header):

```
success, time_sec, path_length_m, avg_linear_vel, avg_angular_vel,
recovery_count, collision_count, stuck, min_obstacle_dist_m,
avg_obstacle_dist_m, avg_traversability, avg_corridor_width
```

## Import ke Excel untuk BAB 4:

```
Scenario | Success | Time | Path | Collision | MinDist | Traversability
S1 Avg   | 0.9     | 45.2 | 20.5 | 0.1       | 0.85    | 0.88
S2 Avg   | 0.8     | 52.1 | 22.3 | 0.5       | 0.52    | 0.72
S3 Avg   | 0.7     | 58.9 | 24.1 | 1.2       | 0.38    | 0.65
S4 Avg   | 0.6     | 65.3 | 26.5 | 2.1       | 0.28    | 0.58
```

## Tips:

- Jangan close terminal sampai "Goal sent" + robot starting
- Jika mau stop: `Ctrl+C` di terminal utama
- Jika collision banyak: Adjust `rep_gain` formula di APF atau increase `d0` safety distance
- Jika thermal tidak working: Check `/traversability_score` topic dengan `rostopic echo`

---

**Next Step:** Build & Test
```bash
cd ~/TA/final_ws
catkin_make
```

Setelah build sukses, run S1 trial 1 untuk test semua komponen.
