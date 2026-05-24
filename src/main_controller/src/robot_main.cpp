#include <ros/ros.h>
// #include "robot.h"
#include "control_layout.h"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "robot_main");

    // ROS_INFO("Starting node...");

    Robot asr;

    return 0;
}