#include "ros/ros.h"
#include "rs232.h"
#include "stdlib.h"
#include "stdio.h"
#include <string.h>
#include "std_msgs/Int32.h"
#include "std_msgs/Int16MultiArray.h"
#include "std_msgs/Float32MultiArray.h"
#include "geometry_msgs/Pose2D.h"
#include "geometry_msgs/Twist.h"
#include <iostream>

#include "main_controller/ControllerData.h"
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>

class Comhardware
{
public:
    Comhardware();

    ~Comhardware();

private:
    int     Cport_nr = 38; //38 is rs232 by id
    int     Bdrate = 115200;
    int     RobotSpeed[3] = {0, 0, 0};
    int     StatusControl;
    int     i;
    int     n;
    int     DataLen;
    int     DataSTM[5] = {0, 0, 0, 0, 0};
    float   VelocityFilter[3]= {0, 0, 0};
    float   PosisiOdom[3] = {0, 0, 0};
    float   OffsetPos[3];
    float   PositionPrev[3] = {0, 0, 0}; 
    float   VelocityRaw[3] = {0, 0, 0};
    float   PositionFiltered[3] = {0, 0, 0};
    char    BitLamp = 0b0000000;
    char    Mode[4] = {'8', 'N', '1', 0};

    unsigned char   Buf[4096];
    std::string     sLocal[5];

    ros::NodeHandle     Nh;
    ros::Publisher      OdomPub;
    ros::Publisher      VelPub;
    ros::Subscriber     SpeedSub;
    nav_msgs::Odometry  Odom;
    
    ros::Rate   RosRate;
    ros::Time   CurrentTime;
    ros::Timer  ThreadSerialTransmit;
    ros::Timer  ThreadSerialReceived;

    geometry_msgs::Quaternion OdomQuat;
    geometry_msgs::Twist      RobotVel;
    
    main_controller::ControllerData     MsgSpeed;
    ros::MultiThreadedSpinner           Mts;

    void SerialTransmitEvent(const ros::TimerEvent &event);
    void SerialReceiveEvent(const ros::TimerEvent &event);
    void SpeedSubCallback(const main_controller::ControllerData &msg);
};