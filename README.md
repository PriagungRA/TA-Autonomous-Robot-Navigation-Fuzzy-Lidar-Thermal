# README — RUN SIMULASI TA: PSO / APF Navigation

Dokumentasi singkat untuk menjalankan simulasi navigation (MLPSO global planner + APF local planner) pada workspace TA.
Lokasi workspace

```
~/TA/final_ws
```
Prasyarat

- ROS (ROS1) terpasang dan terkonfigurasi
- Dependensi paket di workspace sudah terinstall

Persiapan build

1. Build workspace (jalankan jika belum atau setelah mengubah kode):

```bash
cd ~/TA/final_ws
catkin_make
```
2. Setelah build, selalu source workspace di setiap terminal baru:

```bash
cd ~/TA/final_ws
source devel/setup.bash
```
Panduan menjalankan sistem (multi-terminal)

CATATAN: Jalankan perintah `source devel/setup.bash` di setiap terminal baru sebelum perintah ROS berikut.

Terminal 1 — ROSCORE

Jalankan ROS master dan biarkan tetap hidup:

```bash
cd ~/TA/final_ws
source devel/setup.bash
roscore
```
Terminal 2 — Graph generator

PENTING: Hapus cache graph lama sebelum memulai graph generator (jika perlu):

```bash
cd ~/TA/final_ws
source devel/setup.bash
rm -rf ~/TA/final_ws/src/tuw_multi_robot/tuw_voronoi_graph/cfg/graph/grit_1_edited/cache
roslaunch tuw_voronoi_graph graph_generator.launch
```
Jika berhasil:

- Tidak ada error seperti `boost::archive::archive_exception`
- Node graph generator tetap berjalan

Terminal 3 — Navigation simulation (Gazebo + RViz + move_base + planner)

```bash
cd ~/TA/final_ws
source devel/setup.bash
roslaunch ros_sim nav_sim.launch
```
Yang harus muncul:

- Gazebo
- RViz
- Robot spawn di Gazebo
- Map muncul di RViz
- MLPSO Plugin initialized
- APF Planner Plugin initialized

Menetapkan goal (di RViz)

1. Di RViz pilih tool `2D Nav Goal`.
2. Klik titik tujuan pada map.

Hasil normal (jika berhasil):

- Global path muncul di RViz
- Robot bergerak menuju goal
- Obstacle avoidance aktif
- Planner MLPSO (global) + APF (local) bekerja

Debug & Troubleshooting

Jika robot tidak berjalan:

1. Pastikan graph generator tidak crash. Jika muncul error `boost::archive::archive_exception`, hapus cache lalu restart graph generator:

```bash
rm -rf ~/TA/final_ws/src/tuw_multi_robot/tuw_voronoi_graph/cfg/graph/grit_1_edited/cache
# lalu re-run graph generator seperti di Terminal 2
```

2. Cek apakah goal berhasil dikirim dari RViz:

```bash
rostopic echo /move_base_simple/goal
```

Jika ada data pada topic tersebut, RViz mengirim goal dengan benar.

3. Cek apakah planner mengeluarkan velocity command:

```bash
rostopic echo /cmd_vel
```

Jika kosong, planner gagal menghasilkan path/command.

Jika map tidak muncul di RViz:

1. Pastikan `Fixed Frame` di RViz diset ke `map`.
2. Tambahkan display `Map` dan set Topic ke `/map`.

Untuk masalah laser/URDF lidar (jika laser tampak aneh):

Periksa pengaturan `rpy` pada URDF lidar, contoh yang umum:

```
rpy="-1.57 0 0"
```

Jika perlu kompilasi ulang workspace (mis. `devel/setup.bash` error):

```bash
cd ~/TA/final_ws
catkin_make
```

Topik penting (debug):

- Scan: `rostopic echo /scan`
- Map: `rostopic echo /map`
- Velocity command: `rostopic echo /cmd_vel`
- Goal dari RViz: `rostopic echo /move_base_simple/goal`
- TF tree: `rosrun tf view_frames`

Arsitektur sistem (alur data)

Gazebo
	↓
Robot + Lidar
	↓
/scan
	↓
Costmap
	↓
MLPSO Global Planner
	↓
APF Local Planner
	↓
move_base
	↓
/cmd_vel
	↓
Robot Navigation

Planner yang digunakan

- Global planner: MLPSO
- Local planner: APF (Artificial Potential Field)

Fitur simulasi

- Autonomous navigation
- Obstacle avoidance (laser-based)
- Global path planning (MLPSO)
- Reactive local planning (APF)
- Visualisasi di RViz dan simulasi fisika di Gazebo

Catatan tambahan

- Selalu pastikan `devel/setup.bash` di-source di setiap terminal.
- Jika ada error build, jalankan `catkin_make` dan perhatikan pesan error untuk dependency yang hilang.

Butuh bantuan lebih lanjut?

Jika Anda ingin, saya bisa:
- Menambahkan `README.md` markdown (terpisah) dan commit perubahan.
- Membuat skrip shell untuk membuka semua terminal dan menjalankan perintah yang diperlukan.
