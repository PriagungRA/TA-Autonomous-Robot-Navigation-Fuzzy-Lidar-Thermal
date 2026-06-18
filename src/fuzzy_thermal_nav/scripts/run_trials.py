#!/usr/bin/env python3
# ============================================================================
# run_trials.py  -  BATCH RUNNER untuk satu skenario
# ----------------------------------------------------------------------------
# Cara pakai (sim sudah jalan untuk skenario yang diinginkan):
#   1) roslaunch asr_navigation nav_sim.launch scenario:=S1
#   2) rosrun fuzzy_thermal_nav experiment_logger.py        (di terminal lain)
#   3) rosrun fuzzy_thermal_nav run_trials.py _scenario:=S1 _n:=10
#
# Per trial: reset robot ke posisi start -> clear costmap -> set trial_id ->
# kirim goal -> tunggu selesai (move_base result / timeout) -> ulang.
#
# CATATAN planar_move: odom-nya dibaca dari pose model asli, jadi reset via
# set_model_state AMAN (odom ikut update). Tidak perlu relaunch tiap trial.
#
# CATATAN S4 (dinamis): jalankan dynamic_obstacles.py terpisah. Fase gerak
# human tidak di-reset antar trial (lanjut) -> ini menambah varian alami;
# perbanyak trial untuk S4.
# ============================================================================

import rospy
import time

from geometry_msgs.msg import PoseStamped, Pose
from gazebo_msgs.msg import ModelStates
from gazebo_msgs.srv import SetModelState
from gazebo_msgs.msg import ModelState
from move_base_msgs.msg import MoveBaseActionResult
from std_srvs.srv import Empty


class TrialRunner:
    def __init__(self):
        self.scenario   = rospy.get_param("~scenario", "S1")
        self.n_trials   = int(rospy.get_param("~n", 10))
        self.robot_name = rospy.get_param("~robot_model", "CAD_ASR_WITH_OMNI")
        self.max_time   = float(rospy.get_param("~max_time", 120.0))
        self.settle     = float(rospy.get_param("~settle", 2.0))

        # goal (default = goal lama dari send_goal.py)
        self.gx = float(rospy.get_param("~goal_x", 9.158967))
        self.gy = float(rospy.get_param("~goal_y", 6.240290))
        self.gz_z = float(rospy.get_param("~goal_qz", 0.0861667))
        self.gz_w = float(rospy.get_param("~goal_qw", 0.9962807))

        rospy.set_param("/experiment_scenario", self.scenario)

        self.start_pose = None
        self.result_status = None

        self.goal_pub = rospy.Publisher("/move_base_simple/goal", PoseStamped, queue_size=1)
        rospy.Subscriber("/gazebo/model_states", ModelStates, self.model_cb, queue_size=1)
        rospy.Subscriber("/move_base/result", MoveBaseActionResult, self.result_cb, queue_size=1)

        rospy.loginfo("Menunggu /gazebo/set_model_state ...")
        rospy.wait_for_service("/gazebo/set_model_state")
        self.set_state = rospy.ServiceProxy("/gazebo/set_model_state", SetModelState)
        try:
            rospy.wait_for_service("/move_base/clear_costmaps", timeout=10.0)
            self.clear_costmaps = rospy.ServiceProxy("/move_base/clear_costmaps", Empty)
        except rospy.ROSException:
            self.clear_costmaps = None
            rospy.logwarn("clear_costmaps tidak tersedia, dilewati.")

    # ----------------------------------------------------------------
    def model_cb(self, msg):
        if self.start_pose is None and self.robot_name in msg.name:
            i = msg.name.index(self.robot_name)
            self.start_pose = msg.pose[i]
            rospy.loginfo("Posisi START robot direkam: (%.2f, %.2f)",
                          self.start_pose.position.x, self.start_pose.position.y)

    def result_cb(self, msg):
        self.result_status = msg.status.status   # 3=sukses, 4=abort

    # ----------------------------------------------------------------
    def reset_robot(self):
        st = ModelState()
        st.model_name = self.robot_name
        st.pose = self.start_pose
        st.reference_frame = "world"
        # twist nol
        st.twist.linear.x = st.twist.linear.y = st.twist.linear.z = 0.0
        st.twist.angular.x = st.twist.angular.y = st.twist.angular.z = 0.0
        try:
            self.set_state(st)
        except rospy.ServiceException as e:
            rospy.logwarn("reset robot gagal: %s", e)

    def send_goal(self):
        g = PoseStamped()
        g.header.frame_id = "map"
        g.header.stamp = rospy.Time.now()
        g.pose.position.x = self.gx
        g.pose.position.y = self.gy
        g.pose.orientation.z = self.gz_z
        g.pose.orientation.w = self.gz_w
        self.goal_pub.publish(g)

    # ----------------------------------------------------------------
    def run(self):
        # tunggu start pose terekam
        t0 = time.time()
        while self.start_pose is None and not rospy.is_shutdown():
            if time.time() - t0 > 15:
                rospy.logerr("Tidak dapat pose robot dari /gazebo/model_states. Cek nama model '%s'.", self.robot_name)
                return
            rospy.sleep(0.2)

        for i in range(1, self.n_trials + 1):
            if rospy.is_shutdown():
                break
            rospy.loginfo("================ %s TRIAL %d/%d ================",
                          self.scenario, i, self.n_trials)

            rospy.set_param("/trial_id", i)
            rospy.set_param("/experiment_scenario", self.scenario)

            self.reset_robot()
            rospy.sleep(1.0)
            if self.clear_costmaps:
                try: self.clear_costmaps()
                except rospy.ServiceException: pass
            rospy.sleep(self.settle)

            self.result_status = None
            self.send_goal()
            rospy.loginfo("Goal terkirim, menunggu selesai (maks %.0f dtk)...", self.max_time)

            # tunggu hasil atau timeout
            t_start = time.time()
            while not rospy.is_shutdown():
                if self.result_status in (3, 4):
                    rospy.loginfo("Trial %d selesai (status %d)", i, self.result_status)
                    break
                if time.time() - t_start > self.max_time + 10:
                    rospy.logwarn("Trial %d timeout di runner.", i)
                    break
                rospy.sleep(0.2)

            rospy.sleep(2.0)  # beri waktu logger menulis CSV

        rospy.loginfo("=========== SELESAI: %d trial untuk %s ===========",
                      self.n_trials, self.scenario)


if __name__ == "__main__":
    rospy.init_node("run_trials")
    TrialRunner().run()
