//ROS-Libraries
#include <ros/ros.h>
#include <ros/package.h>

#include "std_msgs/Bool.h"
#include "std_msgs/Int32MultiArray.h"
#include "std_msgs/Float32MultiArray.h"

#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Pose2D.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"

#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>

#include <sensor_msgs/Joy.h>
#include <sensor_msgs/BatteryState.h>
#include <sensor_msgs/JoyFeedback.h>
#include <sensor_msgs/JoyFeedbackArray.h>

#include <tf/transform_broadcaster.h>
#include "main_controller/ControllerData.h"


//STD-Libraries
#include <iostream>
#include <string>
#include <queue>
#include <array>
#include "stdio.h"
#include "stdlib.h"

#include <Eigen/Dense>

#define MATH_PI 3.1415926535897932384626433832795

class Robot
{
public:
    Robot();

    ~Robot();

private:
    int     robot_vel[3]       = {0, 0, 0};
    uint8_t StatusControl      = 0;
    uint8_t GuidedMode         = 0;
    bool    rumble_status;   
    bool    obstacle_status, crashed_status, prev_crashed;
    int pathLeft               = 0; 

    struct DS4_t 
    {
        uint8_t Buttons[18];
        float   Axis[4] = {0, 0, 0, 0};
        uint8_t prev_button[18];
    };

    struct Path_t{
        std::queue<float> x;
        std::queue<float> y;
        std::queue<float> theta;
    };

    struct Pose_t{
        float x;
        float y;
        float theta;
    };

    typedef enum
    {
        SQUARE    = 0,
        TRIANGLE  = 1,
        CIRCLE    = 2,
        CROSS     = 3,
        L1        = 4,
        L2        = 5,
        R1        = 6,
        R2        = 7,
        SHARE     = 8,
        OPTIONS   = 9,
        DPadLeft  = 14,
        DPadUp    = 15,
        DPadRight = 16,
        DPadDown  = 17
    } DS4_Button;
    
    DS4_t Controller;
    Path_t path;
    Pose_t robot_pose;
    Pose_t robot_pose_odom;
    Pose_t next_pose;

    Pose_t local_vel;
    Pose_t pure_pursuit_vel;
    Pose_t obstacle_avoider_vel;

    ros::NodeHandle     Nh;
    ros::Subscriber     Sub_Joy;
    ros::Subscriber     Sub_Joy_Battery;
    ros::Subscriber     Sub_Path;
    ros::Subscriber     Sub_Pose;
    ros::Subscriber     Sub_Pose_Odom;
    ros::Subscriber     Sub_Crashed;
    ros::Subscriber     Sub_Obstacle;
    ros::Subscriber     Sub_Obs_Vel;
    
    ros::Publisher      Pub_Vel;
    ros::Publisher      Pub_Joy_Feedback;
    ros::Publisher      Pub_Origin;
    ros::Publisher      Pub_Pure_Pursuit;
    ros::Publisher      Pub_Local_Desired_Vel;
    ros::Publisher      Pub_Mission_Status;
    ros::Rate           RosRate;
    ros::Time           prev_time;

    sensor_msgs::JoyFeedback        MsgJoyLED_R;
    sensor_msgs::JoyFeedback        MsgJoyLED_G;
    sensor_msgs::JoyFeedback        MsgJoyLED_B;
    sensor_msgs::JoyFeedback        MsgJoyRumble;
    sensor_msgs::JoyFeedbackArray   MsgJoyFeedbackArray;
    geometry_msgs::PoseStamped      origin_msg;
    geometry_msgs::PoseStamped      pure_pursuit_msg; 
    geometry_msgs::Twist            local_desired_vel_msg;
    std_msgs::Bool                  mission_status_msg;

    main_controller::ControllerData         vel_msg;

    void ClearPath                (Path_t &path);
    Pose_t PurePursuit            (Pose_t robotPose, Path_t &path, float offset, bool obstacle);
    Pose_t PointToPointPID        (Pose_t robotPose, Pose_t targetPose);
    Pose_t PointToPointPIDV2      (Pose_t robot_pose, Pose_t target_pose);
    Pose_t PointToPointLQR        (Pose_t robotPose, Pose_t targetPose, float maxSpeed);
    Pose_t Global_to_Local_Vel    (Pose_t robot_pose, Pose_t global_vel);
    
    void Joy_Callback             (const sensor_msgs::Joy::ConstPtr &joy_msg);
    void Path_Callback            (const nav_msgs::Path::ConstPtr &path_msg);
    void Pose_Callback            (const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &pose_msg);
    void Pose_Odom_Callback       (const nav_msgs::Odometry::ConstPtr &pose_msg);
    void Crashed_Status_Callback  (const std_msgs::Bool::ConstPtr &crashed_msg);
    void Obstacle_Status_Callback (const std_msgs::Bool::ConstPtr &obs_status_msg);
    void Obstacle_Vel_Callback    (const geometry_msgs::Twist::ConstPtr &obs_vel_msg);
};