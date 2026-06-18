#!/usr/bin/env python3
# ============================================================================
# dynamic_obstacles.py  (S4 - human bergerak bolak-balik)
# ----------------------------------------------------------------------------
# human_1 menyusuri garis A1 <-> B1, human_2 menyusuri A2 <-> B2 (bolak-balik
# mulus pakai 0.5*(1-cos)). z & orientasi diset benar supaya human tidak
# nyungsep / terguling.
# ============================================================================

import rospy
import math

from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import SetModelState

rospy.init_node("dynamic_obstacles")
rospy.wait_for_service('/gazebo/set_model_state')
set_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)

Z = 0.550025          # tinggi pusat human (JANGAN 0 -> nyungsep)
W = 0.4               # kecepatan sudut osilasi (rad/s). besar = cepat
RATE = 200.0

# --- endpoint gerak (dari datamu) ---
# human_1 : A1 <-> B1
A1 = (5.268695, -0.186458)
B1 = (3.084140,  1.964217)
# human_2 : A2 <-> B2
B2 = (4.921940,  3.999455)
A2 = (7.245282,  1.757134)


def lerp(a, b, s):
    return a + (b - a) * s


def make_state(name, ax, ay, bx, by, s):
    st = ModelState()
    st.model_name = name
    st.pose.position.x = lerp(ax, bx, s)
    st.pose.position.y = lerp(ay, by, s)
    st.pose.position.z = Z
    # orientasi tegak, tidak berputar (quaternion valid w=1)
    st.pose.orientation.x = 0.0
    st.pose.orientation.y = 0.0
    st.pose.orientation.z = 0.0
    st.pose.orientation.w = 1.0
    st.reference_frame = "world"
    return st


rate = rospy.Rate(RATE)
t = 0.0
dt = 1.0 / RATE

while not rospy.is_shutdown():
    # s berosilasi 0..1..0 (A -> B -> A) secara mulus
    s = 0.5 * (1.0 - math.cos(W * t))

    set_state(make_state("obstacle_human_1", A1[0], A1[1], B1[0], B1[1], s))
    set_state(make_state("obstacle_human_2", A2[0], A2[1], B2[0], B2[1], s))

    t += dt
    rate.sleep()