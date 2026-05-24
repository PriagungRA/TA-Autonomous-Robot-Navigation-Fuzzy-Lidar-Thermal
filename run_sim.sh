#!/bin/bash

# =========================
# TERMINAL 1 : ROSCORE
# =========================
gnome-terminal -- bash -c "
source /opt/ros/noetic/setup.bash;
cd ~/TA/final_ws;
source devel/setup.bash;
roscore;
exec bash
"

sleep 2

# =========================
# TERMINAL 2 : GRAPH GENERATOR
# =========================
gnome-terminal -- bash -c "
source /opt/ros/noetic/setup.bash;
cd ~/TA/final_ws;
source devel/setup.bash;

rm -rf ~/TA/final_ws/src/tuw_multi_robot/tuw_voronoi_graph/cfg/graph/grit_1_edited_2/cache/

roslaunch tuw_voronoi_graph graph_generator.launch;

exec bash
"

sleep 5

# =========================
# TERMINAL 3 : NAVIGATION SIM
# =========================
gnome-terminal -- bash -c "
source /opt/ros/noetic/setup.bash;
cd ~/TA/final_ws;
source devel/setup.bash;

roslaunch ros_sim nav_sim.launch;

exec bash
"