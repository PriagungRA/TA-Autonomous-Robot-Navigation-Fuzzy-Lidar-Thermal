#!/usr/bin/env python3

import rospy
import csv
import math
import time
import os

from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Twist
from move_base_msgs.msg import MoveBaseActionResult
from move_base_msgs.msg import RecoveryStatus
from std_msgs.msg import Float32
from std_msgs.msg import Float32MultiArray
from gazebo_msgs.msg import ModelStates


class ExperimentLogger:

    def __init__(self):

        self.goal_received = False
        self.start_time = None

        self.path_length = 0.0
        self.prev_x = None
        self.prev_y = None

        self.lin_sum = 0.0
        self.ang_sum = 0.0
        self.cmd_count = 0

        self.recovery_count = 0
        self.finished = False

        # ================================
        # SCENARIO CONFIG
        # ================================
        self.scenario = rospy.get_param("~scenario", "S1")
        self.trial_id = rospy.get_param("~trial_id", 1)
        self.max_experiment_time = rospy.get_param("~max_experiment_time", 120)
        rospy.Timer(rospy.Duration(1.0), self.timeout_cb)
        rospy.loginfo(f"SCENARIO: {self.scenario}")

        # ================================
        # GROUND-TRUTH COLLISION CONFIG
        # --------------------------------
        # WAJIB pakai ground-truth: LiDAR TIDAK bisa melihat kucing (blind
        # spot), jadi collision kucing tak akan terdeteksi kalau pakai /scan.
        # Tabrakan = jarak(pusat robot, pusat obstacle) < r_robot + r_obstacle.
        # ================================
        self.robot_model_name = rospy.get_param("~robot_model", "CAD_ASR_WITH_OMNI")
        self.robot_radius = rospy.get_param("~robot_radius", 0.20)  # radius TERDALAM (inscribed) footprint persegi 0.27x0.32 -> separuh sisi pendek = 0.16-0.20. Pakai 0.20 = konservatif tapi TIDAK menghitung pojok yang tak menyentuh (hindari X palsu).

        # radius tiap obstacle (m) - sesuaikan dgn world-mu.
        # kucing & human dari S3/S4. Tambahkan kalau ada obstacle lain di jalur.
        self.obstacle_radii = {
            "obstacle_cat":      0.18,
            "obstacle_human_1":  0.25,
            "obstacle_human_2":  0.25,
            "obstacle":          0.25,
        }
        rospy.loginfo("=== LOGGER: robot_radius = %.3f m (inscribed footprint persegi) ===",
                      self.robot_radius)
        rospy.loginfo("=== Obstacle dilacak: %s ===", list(self.obstacle_radii.keys()))

        self.collision_count = 0
        self.in_collision = False
        # min_clearance = jarak permukaan terdekat (negatif = MENEMBUS)
        self.min_clearance = 999.0
        self.clearances = []
        self.collided_with = set()   # nama obstacle yang pernah ditabrak

        # rekaman lintasan untuk GAMBAR bukti (BAB 4)
        self.traj = []                 # (t, x, y) robot
        self.obstacle_positions = {}   # {name: (x, y, r)} untuk plot
        # [S8] track posisi obstacle per-WAKTU (utk plot dinamis yg jujur)
        self.obs_track = {}            # {name: [(t, x, y), ...]}
        self.last_obs_log_t = -1.0     # throttle perekaman track (detik)

        # ================================
        # THERMAL METRICS
        # ================================
        self.traversability_scores = []
        self.corridor_widths = []

        # [RM2-LOG] time series fusi traversability: [(t, trav_lidar, trav_thermal, trav_eff), ...]
        self.trav_fusion_log = []

        # ================================
        # SUBSCRIBERS
        # ================================
        rospy.Subscriber("/odom", Odometry, self.odom_cb)
        rospy.Subscriber("/move_base_simple/goal", PoseStamped, self.goal_cb)
        rospy.Subscriber("/cmd_vel", Twist, self.cmd_cb)
        rospy.Subscriber("/move_base/result", MoveBaseActionResult, self.result_cb)
        rospy.Subscriber("/move_base/recovery_status", RecoveryStatus, self.recovery_cb)
        rospy.Subscriber("/gazebo/model_states", ModelStates, self.model_states_cb, queue_size=1)
        rospy.Subscriber("/traversability_score", Float32, self.trav_cb)
        rospy.Subscriber("/corridor_width", Float32, self.width_cb)
        rospy.Subscriber("/trav_fusion_debug", Float32MultiArray, self.trav_fusion_cb)

        rospy.loginfo("LOGGER STARTED (collision = GROUND TRUTH, termasuk kucing)")

    # ---------------------------------
    def reset_trial_metrics(self):
        self.path_length = 0.0
        self.prev_x = None
        self.prev_y = None
        self.lin_sum = 0.0
        self.ang_sum = 0.0
        self.cmd_count = 0
        self.recovery_count = 0
        self.collision_count = 0
        self.in_collision = False
        self.min_clearance = 999.0
        self.clearances = []
        self.collided_with = set()
        self.traj = []                 # [(t, x, y)] lintasan robot
        self.obstacle_positions = {}   # {name: (x, y, r)} untuk plot
        self.obs_track = {}            # {name: [(t, x, y)]} track per-waktu
        self.last_obs_log_t = -1.0
        self.traversability_scores = []
        self.corridor_widths = []
        self.trav_fusion_log = []          # [RM2-LOG] reset per trial

    # ---------------------------------
    def goal_cb(self, msg):
        if self.goal_received and not self.finished:
            rospy.logwarn("NEW GOAL IGNORED - CURRENT TRIAL STILL RUNNING")
            return
        self.reset_trial_metrics()
        # re-read param tiap goal -> batch-runner bisa ganti trial_id per trial
        self.scenario = rospy.get_param("/experiment_scenario", self.scenario)
        self.trial_id = rospy.get_param("/trial_id", self.trial_id)
        self.goal_received = True
        self.finished = False
        self.start_time = time.time()
        rospy.loginfo(f"GOAL RECEIVED - {self.scenario} trial {self.trial_id} - TIMER STARTED")

    # ---------------------------------
    def odom_cb(self, msg):
        if not self.goal_received or self.finished:
            return
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        if self.prev_x is not None:
            dx = x - self.prev_x
            dy = y - self.prev_y
            self.path_length += math.sqrt(dx * dx + dy * dy)
        self.prev_x = x
        self.prev_y = y
        t = time.time() - self.start_time if self.start_time else 0.0
        self.traj.append((round(t, 2), round(x, 4), round(y, 4)))

    # ---------------------------------
    def cmd_cb(self, msg):
        if not self.goal_received or self.finished:
            return
        self.lin_sum += abs(msg.linear.x)
        self.ang_sum += abs(msg.angular.z)
        self.cmd_count += 1

    # ---------------------------------
    def recovery_cb(self, msg):
        if not self.goal_received or self.finished:
            return
        self.recovery_count += 1

    # ---------------------------------
    def model_states_cb(self, msg):
        """Collision & clearance dari GROUND TRUTH (bisa lihat kucing)."""
        if not self.goal_received or self.finished:
            return

        if self.robot_model_name not in msg.name:
            return
        ri = msg.name.index(self.robot_model_name)
        rx = msg.pose[ri].position.x
        ry = msg.pose[ri].position.y

        frame_min_clear = 999.0
        hit_now = False

        # [S8] stempel waktu utk track dinamis (throttle ~20 Hz biar file kecil)
        t_now = (time.time() - self.start_time) if self.start_time else 0.0
        log_track = (t_now - self.last_obs_log_t) >= 0.05
        if log_track:
            self.last_obs_log_t = t_now

        for name, r_obs in self.obstacle_radii.items():
            if name not in msg.name:
                continue
            oi = msg.name.index(name)
            ox = msg.pose[oi].position.x
            oy = msg.pose[oi].position.y

            d = math.hypot(ox - rx, oy - ry)
            clearance = d - self.robot_radius - r_obs   # <0 = menembus

            if clearance < frame_min_clear:
                frame_min_clear = clearance

            self.obstacle_positions[name] = (round(ox, 3), round(oy, 3), r_obs)
            # rekam track per-waktu (utk human bergerak di S8)
            if log_track:
                self.obs_track.setdefault(name, []).append(
                    (round(t_now, 2), round(ox, 3), round(oy, 3)))

            if clearance < 0.0:
                hit_now = True
                if name not in self.collided_with:
                    rospy.logwarn("KONTAK PERTAMA: %s  clearance=%.3f m  (d=%.3f, r_robot=%.2f, r_obs=%.2f)",
                                  name, clearance, d, self.robot_radius, r_obs)
                self.collided_with.add(name)

        if frame_min_clear < 999.0:
            self.clearances.append(frame_min_clear)
            if frame_min_clear < self.min_clearance:
                self.min_clearance = frame_min_clear

        # hitung 1 event tabrakan sampai robot lepas lagi (histeresis)
        if hit_now and not self.in_collision:
            self.collision_count += 1
            self.in_collision = True
            rospy.logwarn_throttle(0.5, "COLLISION! menembus: %s", list(self.collided_with))
        elif frame_min_clear > 0.05:
            self.in_collision = False

    # ---------------------------------
    def trav_cb(self, msg):
        if not self.goal_received or self.finished:
            return
        self.traversability_scores.append(msg.data)

    # ---------------------------------
    def width_cb(self, msg):
        if not self.goal_received or self.finished:
            return
        self.corridor_widths.append(msg.data)

    # ---------------------------------
    def trav_fusion_cb(self, msg):
        """[RM2-LOG] Bukti empiris fusi: tau_eff = min(tau_lidar, tau_thermal)."""
        if not self.goal_received or self.finished or len(msg.data) < 3:
            return
        t = time.time() - self.start_time if self.start_time else 0.0
        trav_lidar, trav_thermal, trav_eff = msg.data[0], msg.data[1], msg.data[2]
        self.trav_fusion_log.append((
            round(t, 3), round(trav_lidar, 4), round(trav_thermal, 4), round(trav_eff, 4)
        ))

    # ---------------------------------
    def write_csv(self, row):
        base_dir = os.path.expanduser("~/TA/final_ws/experiment_data")
        scenario_dir = os.path.join(base_dir, self.scenario)
        os.makedirs(scenario_dir, exist_ok=True)
        filename = os.path.join(scenario_dir, "results.csv")
        write_header = not os.path.exists(filename)
        with open(filename, "a") as f:
            writer = csv.writer(f)
            if write_header:
                writer.writerow([
                    "scenario", "trial_id", "success", "goal_reached",
                    "collision_free", "time_sec", "path_length_m",
                    "avg_linear_vel", "avg_angular_vel", "recovery_count",
                    "collision_count", "stuck", "min_clearance_m",
                    "avg_clearance_m", "hit_cat", "hit_obstacles",
                    "avg_traversability", "avg_corridor_width"
                ])
            writer.writerow(row)

    # ---------------------------------
    def build_row(self, success, goal_reached, elapsed, stuck):
        collision_free = 1 if self.collision_count == 0 else 0
        avg_lin = self.lin_sum / max(self.cmd_count, 1)
        avg_ang = self.ang_sum / max(self.cmd_count, 1)

        min_clear = round(self.min_clearance, 3) if self.min_clearance < 999 else 0.0
        avg_clear = round(sum(self.clearances) / max(len(self.clearances), 1), 3) if self.clearances else 0.0

        hit_cat = 1 if "obstacle_cat" in self.collided_with else 0
        hit_obstacles = "|".join(sorted(self.collided_with)) if self.collided_with else "none"

        avg_trav = round(sum(self.traversability_scores) / max(len(self.traversability_scores), 1), 3) if self.traversability_scores else 0.0
        avg_width = round(sum(self.corridor_widths) / max(len(self.corridor_widths), 1), 2) if self.corridor_widths else 0.0

        return [
            self.scenario, self.trial_id, success, goal_reached,
            collision_free, round(elapsed, 2), round(self.path_length, 2),
            round(avg_lin, 3), round(avg_ang, 3), self.recovery_count,
            self.collision_count, stuck, min_clear, avg_clear,
            hit_cat, hit_obstacles, avg_trav, avg_width
        ]

    # ---------------------------------
    def save_trajectory(self):
        """Simpan lintasan + posisi obstacle untuk plot trajektori."""
        base_dir = os.path.expanduser("~/TA/final_ws/experiment_data")
        scen_dir = os.path.join(base_dir, self.scenario)
        os.makedirs(scen_dir, exist_ok=True)

        # lintasan robot
        traj_file = os.path.join(scen_dir, f"traj_trial_{self.trial_id}.csv")
        with open(traj_file, "w") as f:
            w = csv.writer(f)
            w.writerow(["t", "x", "y"])
            w.writerows(self.traj)

        # posisi obstacle (ditimpa tiap trial, isinya sama untuk statis)
        obs_file = os.path.join(scen_dir, "obstacles.csv")
        with open(obs_file, "w") as f:
            w = csv.writer(f)
            w.writerow(["name", "x", "y", "radius"])
            for name, (ox, oy, r) in self.obstacle_positions.items():
                w.writerow([name, ox, oy, r])

        # [S8] track obstacle per-WAKTU (utk plot dinamis yg jujur).
        # Hanya ditulis kalau ada data (mis. skenario dinamis).
        if self.obs_track:
            obs_traj_file = os.path.join(scen_dir, f"traj_obs_trial_{self.trial_id}.csv")
            with open(obs_traj_file, "w") as f:
                w = csv.writer(f)
                w.writerow(["t", "name", "x", "y"])
                for name, pts in self.obs_track.items():
                    for (tt, ox, oy) in pts:
                        w.writerow([tt, name, ox, oy])

        # [RM2-LOG] time series fusi traversability (tau_lidar, tau_thermal, tau_eff)
        if self.trav_fusion_log:
            fusion_file = os.path.join(scen_dir, f"trav_fusion_trial_{self.trial_id}.csv")
            with open(fusion_file, "w") as f:
                w = csv.writer(f)
                w.writerow(["t", "trav_lidar", "trav_thermal_filt", "trav_eff"])
                w.writerows(self.trav_fusion_log)

    # ---------------------------------
    def timeout_cb(self, event):
        if not self.goal_received or self.finished or self.start_time is None:
            return
        elapsed = time.time() - self.start_time
        if elapsed < self.max_experiment_time:
            return
        rospy.logwarn("TIMEOUT REACHED - SAVING FAILURE RESULT")
        self.finished = True
        row = self.build_row(success=0, goal_reached=0, elapsed=elapsed, stuck=1)
        self.save_trajectory()
        self.write_csv(row)
        rospy.loginfo("TIMEOUT RESULT SAVED")
        rospy.loginfo(row)

    # ---------------------------------
    def result_cb(self, msg):
        if self.finished:
            return
        if not self.goal_received:
            rospy.logwarn("MOVE_BASE RESULT IGNORED - NO GOAL RECEIVED YET")
            return

        status = msg.status.status
        if status not in [3, 4]:    # 3=SUCCEEDED, 4=ABORTED
            return

        self.finished = True
        elapsed = time.time() - self.start_time if self.start_time else 0.0

        goal_reached = 1 if status == 3 else 0
        # stuck: di-abort move_base (gagal cari jalur/oscillation) ATAU lama
        # stuck: di-abort move_base (status 4) ATAU mentok batas waktu skenario.
        # [FIX] dulu hardcoded 120 -> bisa salah label trial S8 yang sah tapi lama.
        stuck = 1 if (status == 4 or elapsed >= self.max_experiment_time) else 0
        collision_free = 1 if self.collision_count == 0 else 0
        success = 1 if (goal_reached and collision_free) else 0

        row = self.build_row(success, goal_reached, elapsed, stuck)
        self.save_trajectory()
        self.write_csv(row)
        rospy.loginfo("RESULT SAVED")
        rospy.loginfo(row)


if __name__ == "__main__":
    rospy.init_node("experiment_logger")
    ExperimentLogger()
    rospy.spin()