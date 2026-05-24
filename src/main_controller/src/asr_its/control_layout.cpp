#include <ros/ros.h>
#include "control_layout.h"

Robot::Robot(): RosRate(200)
{
    // Initialize
    ROS_INFO("Robot Main Controller");
    
    // Subscriber & Publisher
    Sub_Joy               = Nh.subscribe("/joy", 18, &Robot::Joy_Callback, this);

    Sub_Path              = Nh.subscribe("/path", 10, &Robot::Path_Callback, this);
    Sub_Pose              = Nh.subscribe("/amcl_pose", 10, &Robot::Pose_Callback, this);
    Sub_Pose_Odom         = Nh.subscribe("/odom", 10, &Robot::Pose_Odom_Callback, this);

    Sub_Crashed           = Nh.subscribe("/crashed", 1, &Robot::Crashed_Status_Callback, this);
    Sub_Obstacle          = Nh.subscribe("/obstacle_detected", 1, &Robot::Obstacle_Status_Callback, this);
    Sub_Obs_Vel           = Nh.subscribe("/velocity_obstacle/opt_vel", 10, &Robot::Obstacle_Vel_Callback, this);

    Pub_Vel               = Nh.advertise<main_controller::ControllerData>("robot/cmd_vel", 10);
    // Pub_Joy_Feedback      = Nh.advertise<sensor_msgs::JoyFeedbackArray>("/set_feedback", 10);
    Pub_Origin            = Nh.advertise<geometry_msgs::PoseStamped>("/goal", 1);
    Pub_Pure_Pursuit      = Nh.advertise<geometry_msgs::PoseStamped>("/pure_pursuit_pose", 10);
    Pub_Local_Desired_Vel = Nh.advertise<geometry_msgs::Twist>("/main_controller/local_desired_vel", 10);
    Pub_Mission_Status    = Nh.advertise<std_msgs::Bool>("/main_controller/mission_status", 1);

    // Initialize Speed Variable
    for (int i = 0; i <=2 ; i++)
    {
        vel_msg.data.push_back(0);
    }

    // Initialize Joystick LED Feedback
    MsgJoyLED_R.type        = 0;
    MsgJoyLED_R.id          = 0;
    MsgJoyLED_R.intensity   = 0.0;

    MsgJoyLED_G.type        = 0;
    MsgJoyLED_G.id          = 1;
    MsgJoyLED_R.intensity   = 0.0;

    MsgJoyLED_B.type        = 0;
    MsgJoyLED_B.id          = 2;
    MsgJoyLED_B.intensity   = 0.0;

    MsgJoyRumble.type       = 1;
    MsgJoyRumble.id         = 1;
    MsgJoyRumble.intensity  = 0.0;    

    MsgJoyFeedbackArray.array.push_back(MsgJoyLED_R);
    MsgJoyFeedbackArray.array.push_back(MsgJoyLED_G);
    MsgJoyFeedbackArray.array.push_back(MsgJoyLED_B);
    MsgJoyFeedbackArray.array.push_back(MsgJoyRumble);

    // Initialize Pure Purusit
    // int pathLeft;
    // double distance_left;

    while(ros::ok())
    {   
        // Print Robot Speed (DEBUG)
        // std::cout << "x : " << robot_vel[0] << " y : " << robot_vel[1] << " Theta : " << robot_vel[2] << " Status : " << vel_msg.StatusControl << std::endl;
        // std::cout << "pose= x: " << robot_pose.x << " y: " << robot_pose.y << " theta: " << robot_pose.theta*(180/MATH_PI) << std::endl;

        // Set Status Control using TRIANGLE Button
        if (Controller.Buttons[TRIANGLE] == 0 && Controller.prev_button[TRIANGLE] == 1)
        {
            StatusControl ^= 1;
            vel_msg.StatusControl = StatusControl;
        }
        Controller.prev_button[TRIANGLE] = Controller.Buttons[TRIANGLE];

        // Clear Path Generated using CIRCLE Button
        if (Controller.Buttons[CIRCLE] == 0 && Controller.prev_button[CIRCLE] == 1)
        {
            ClearPath(path);
        }
        Controller.prev_button[CIRCLE] = Controller.Buttons[CIRCLE];

        // Set GUIDED/MANUAL Mode Using OPTIONS Button
        if (Controller.Buttons[OPTIONS] == 0 && Controller.prev_button[OPTIONS] == 1)
        {
            GuidedMode ^= 1;
            // Set Rumble Feedback
            rumble_status = 1;
            MsgJoyRumble.intensity  = 0.5;
            prev_time = ros::Time::now();
        }
        Controller.prev_button[OPTIONS] = Controller.Buttons[OPTIONS];

        // Reset local Odom
        if (Controller.Buttons[SQUARE] == 0 && Controller.prev_button[SQUARE] == 1)
        {
            robot_pose_odom.x -= robot_pose_odom.x;
            robot_pose_odom.y -= robot_pose_odom.y;
            robot_pose_odom.theta -= robot_pose_odom.theta;
        }
        Controller.prev_button[SQUARE] = Controller.Buttons[SQUARE];

        // Set RTH Mode Using SHARE Button
        if (Controller.Buttons[SHARE] == 1 && Controller.prev_button[SHARE] == 0)
        {
            // Clear Current Path
            ClearPath(path);

            // Add Header Goal Message
            origin_msg.header.stamp = ros::Time::now();
            origin_msg.header.frame_id = "map";

            // Set Goal to Origin Position
            origin_msg.pose.position.x = 0.0;
            origin_msg.pose.position.y = 0.0;
            origin_msg.pose.position.z = 0.0;

            // Set Goal to Zero Degree Orientation
            origin_msg.pose.orientation.x = 0.0;
            origin_msg.pose.orientation.y = 0.0;
            origin_msg.pose.orientation.z = 0.0;
            origin_msg.pose.orientation.w = 1.0;

            // Publish Goal Message
            Pub_Origin.publish(origin_msg);

            // Set Rumble Feedback
            rumble_status = 1;
            MsgJoyRumble.intensity  = 0.5;
            prev_time = ros::Time::now();
        }
        Controller.prev_button[SHARE] = Controller.Buttons[SHARE];

        // Rumble Feedback Event
        if (rumble_status && ros::Time::now() - prev_time >= ros::Duration(0.57))
        {
            MsgJoyRumble.intensity  = 0.0;
            rumble_status = 0;
        }

        // // Clear Path if Robot CRASHED
        // if (prev_crashed == 0 && crashed_status == 1)
        // {
        //     // DEBUG
        //     ClearPath(path);
        //     ROS_INFO("Robot stopped because path is closed. Recalculating path... ");
        // }
        prev_crashed = crashed_status;

        // Go to Autonomous Mode
        if (GuidedMode)
        {
            // Go to AUTONOMOUS Mode with Indicator
            if (vel_msg.StatusControl)
            {
                // Set YELLOW Indicator for GUIDED Mode
                MsgJoyLED_R.intensity = 0.3;
                MsgJoyLED_G.intensity = 0.3;
                MsgJoyLED_B.intensity = 0.0;

                // Prevent going to origin if there's no path
                if(path.x.size() <= 0)
                {
                    robot_vel[0] = 0.0;
                    robot_vel[1] = 0.0;
                    robot_vel[2] = 0.0;

                    mission_status_msg.data = false;
                }

                // Search for Closest Node using Pure Pursuit
                else
                {
                    mission_status_msg.data = true;
                    next_pose = PurePursuit(robot_pose, path, 0.1, obstacle_status);

                    // Push Pure Pursuit Next Target to Publisher Messages
                    tf2::Quaternion next_theta;
                    next_theta.setRPY(0, 0, next_pose.theta);
                    next_theta = next_theta.normalize();

                    pure_pursuit_msg.header.frame_id  = "map";
                    pure_pursuit_msg.pose.position.x  = next_pose.x;
                    pure_pursuit_msg.pose.position.y  = next_pose.y;
                    pure_pursuit_msg.pose.orientation.x = next_theta.x();
                    pure_pursuit_msg.pose.orientation.y = next_theta.y();
                    pure_pursuit_msg.pose.orientation.z = next_theta.z();
                    pure_pursuit_msg.pose.orientation.w = next_theta.w();

                    // PID Controller
                    // pure_pursuit_vel = PointToPointPID(robot_pose, next_pose);

                    //PID Controller by Nawab
                    // pure_pursuit_vel = PointToPointPIDV2(robot_pose, next_pose);

                    // LQR Controller
                    pure_pursuit_vel = PointToPointLQR(robot_pose, next_pose, 30);

                    // Convert Pure Pursuit Velocity to Local Velocity
                    local_vel = Global_to_Local_Vel(robot_pose, pure_pursuit_vel);
                    robot_vel[0] = local_vel.x;
                    robot_vel[1] = local_vel.y;
                    robot_vel[2] = local_vel.theta;

                    // Push Local Velocity to Publisher
                    local_desired_vel_msg.linear.x = local_vel.x * cos(MATH_PI/2) + local_vel.y * sin(MATH_PI/2);
                    local_desired_vel_msg.linear.y = -1 * local_vel.x * sin(MATH_PI/2) + local_vel.y * cos(MATH_PI/2);
                    local_desired_vel_msg.angular.z = local_vel.theta;

                    // Obstacle Avoidance Control with WHITE Indicator
                    if(obstacle_status && pathLeft != 1)
                    {
                        // Set LED Feedback
                        MsgJoyLED_R.intensity = 1.0;
                        MsgJoyLED_G.intensity = 1.0;
                        MsgJoyLED_B.intensity = 1.0;

                        robot_vel[0] = obstacle_avoider_vel.x * cos(MATH_PI/2) - obstacle_avoider_vel.y * sin(MATH_PI/2);
                        robot_vel[1] = obstacle_avoider_vel.x * sin(MATH_PI/2) + obstacle_avoider_vel.y * cos(MATH_PI/2);
                        robot_vel[2] = obstacle_avoider_vel.theta;
                    }
                }
                
            }

            // Pause AUTONOMOUS Mode with RED Indicator
            else
            {   
                mission_status_msg.data = false;
                // Set LED Feedback
                MsgJoyLED_R.intensity = 1.0;
                MsgJoyLED_G.intensity = 0.0;
                MsgJoyLED_B.intensity = 0.13;  

                // Push Local Velocity Publisher to Zero
                local_desired_vel_msg.linear.x = 0.0;
                local_desired_vel_msg.linear.y = 0.0;
                local_desired_vel_msg.angular.z = 0.0;

                // Set Robot Speed to Zero (Safety Issues)
                for(int i = 0; i<=2; i++)
                {
                    robot_vel[i] = 0;
                }    
            }
        }

        // Go to Manual Control Mode
        else 
        {
            mission_status_msg.data = false;
            // Go to Manual Control RUN Mode with GREEN Indicator
            if (vel_msg.StatusControl)
            {
                // Set LED Feedback
                MsgJoyLED_R.intensity = 0.12;
                MsgJoyLED_G.intensity = 0.75;
                MsgJoyLED_B.intensity = 0.13;

                // Set Robot Speed from Joy Axis
                robot_vel[0] = -1 * Controller.Axis[0] * 45;
                robot_vel[1] = Controller.Axis[1] * 45;
                robot_vel[2] = Controller.Axis[2] * 20;

                // Apply R2 speed limiter in Manual RUN: cap rotation to ±0.7
                if (Controller.Buttons[R2])
                {
                    if (robot_vel[2] > 2) {
                        robot_vel[2] = 2;
                    } else if (robot_vel[2] < -2) {
                        robot_vel[2] = -2;
                    }

                    double max_lin = 10.0;
                    double vx = robot_vel[0];
                    double vy = robot_vel[1];
                    double speed = std::sqrt(vx*vx + vy*vy);
                    if (speed > max_lin && speed > 0.0) {
                        double scale = max_lin / speed;
                        robot_vel[0] = vx * scale;
                        robot_vel[1] = vy * scale;
                    }
                }
            }

            // Go to Manual Control LOCK Mode with RED Indicator
            else
            {
                // Set LED Feedback
                MsgJoyLED_R.intensity = 1.0;
                MsgJoyLED_G.intensity = 0.0;
                MsgJoyLED_B.intensity = 0.13;     

                // Set Robot Speed to Zero (Safety Issues)
                for(int i = 0; i<=2; i++)
                {
                    robot_vel[i] = 0;
                }     
            }
        }

        // Limit Robot Speed to 30 cm/s
        for(int i = 0 ; i<=2 ; i++)
        {
            vel_msg.data.at(i) = robot_vel[i];
            if(vel_msg.data.at(i) >= 40)
            {
                vel_msg.data.at(i) = 40;
            }
            else if(vel_msg.data.at(i) <= -40)
            {
                vel_msg.data.at(i) = -40;
            }
        }

        MsgJoyFeedbackArray.array.at(0) = MsgJoyLED_R;
        MsgJoyFeedbackArray.array.at(1) = MsgJoyLED_G;
        MsgJoyFeedbackArray.array.at(2) = MsgJoyLED_B;
        MsgJoyFeedbackArray.array.at(3) = MsgJoyRumble;

        // Publish Topics
        
        Pub_Vel.publish(vel_msg);
        Pub_Pure_Pursuit.publish(pure_pursuit_msg);
        // Pub_Joy_Feedback.publish(MsgJoyFeedbackArray);
        Pub_Local_Desired_Vel.publish(local_desired_vel_msg);
        Pub_Mission_Status.publish(mission_status_msg);

        ros::spinOnce(); 
        RosRate.sleep();
    }
};

Robot::~Robot(){}

void Robot::Joy_Callback (const sensor_msgs::Joy::ConstPtr &joy_msg)
{
    for(int i = 0; i<4; i++)
    {
        Controller.Axis[i] = joy_msg->axes[i];
    }

    for(int i = 0; i<18; i++)
    {
        Controller.Buttons[i] = joy_msg->buttons[i];
    }
}

void Robot::Path_Callback (const nav_msgs::Path::ConstPtr &path_msg)
{
    ClearPath(path);
    double  roll, pitch, yaw;

    for(int i = 0; i < path_msg->poses.size(); ++i)
    {
        // Convert Quaternion to Euler Yaw (Rad)
        tf::Quaternion q(
        path_msg->poses[i].pose.orientation.x,
        path_msg->poses[i].pose.orientation.y,
        path_msg->poses[i].pose.orientation.z,
        path_msg->poses[i].pose.orientation.w);
        tf::Matrix3x3 m(q);

        m.getRPY(roll, pitch, yaw);
        // Push Subscriber Topics to Path Array
        path.x.push(path_msg->poses[i].pose.position.x);
        path.y.push(path_msg->poses[i].pose.position.y);
        path.theta.push(yaw);    

    }
}

void Robot::Pose_Callback (const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &pose_msg)
{
    double  roll, pitch, yaw;

    // Convert Quaternion to Euler Yaw (Rad)
    tf::Quaternion q(
        pose_msg->pose.pose.orientation.x,
        pose_msg->pose.pose.orientation.y,
        pose_msg->pose.pose.orientation.z,
        pose_msg->pose.pose.orientation.w);
    tf::Matrix3x3 m(q);

    m.getRPY(roll, pitch, yaw);
    // Push Subscriber Topics to Current Robot Pose Variable
    robot_pose.x = pose_msg->pose.pose.position.x;
    robot_pose.y = pose_msg->pose.pose.position.y;   
    robot_pose.theta = yaw;
}

void Robot::Pose_Odom_Callback (const nav_msgs::Odometry::ConstPtr &pose_msg)
{
    double  roll, pitch, yaw;

    // Convert Quaternion to Euler Yaw (Rad)
    tf::Quaternion q(
        pose_msg->pose.pose.orientation.x,
        pose_msg->pose.pose.orientation.y,
        pose_msg->pose.pose.orientation.z,
        pose_msg->pose.pose.orientation.w);
    tf::Matrix3x3 m(q);

    m.getRPY(roll, pitch, yaw);
    // Push Subscriber Topics to Current Robot Pose Variable
    robot_pose_odom.x = pose_msg->pose.pose.position.x;
    robot_pose_odom.y = pose_msg->pose.pose.position.y;   
    robot_pose_odom.theta = yaw;
}

void Robot::Obstacle_Status_Callback (const std_msgs::Bool::ConstPtr &obs_status_msg)
{
    obstacle_status = obs_status_msg->data;
}

void Robot::Crashed_Status_Callback (const std_msgs::Bool::ConstPtr &crashed_msg)
{
    crashed_status = crashed_msg->data;
}

void Robot::Obstacle_Vel_Callback    (const geometry_msgs::Twist::ConstPtr &obs_vel_msg)
{
    obstacle_avoider_vel.x = obs_vel_msg->linear.x;
    obstacle_avoider_vel.y = obs_vel_msg->linear.y;
    obstacle_avoider_vel.theta = obs_vel_msg->angular.z;
}

void Robot::ClearPath(Path_t &path)
{
    while(!path.x.empty())
    {
        path.x.pop();
        path.y.pop();
        path.theta.pop();
    }
}

Robot::Pose_t Robot::PurePursuit(Pose_t robot_pose, Path_t &path, float offset, bool obstacle)
{

    Pose_t target_pose;
    target_pose = robot_pose;

    float distance = 0.0;
    float delta_heading = 0.0;
    float theta_error = 0.0;
    float dx, dy, dot_product;
    pathLeft = path.x.size();

    // // Collision Avoidance Mode
    if(obstacle)
    {
        offset *= 10;
        while(pathLeft > 1)
        {
            // Check for lookahead point
            target_pose.x = path.x.front();
            target_pose.y = path.y.front();
            target_pose.theta = path.theta.front();
            distance = sqrt(pow((target_pose.x - robot_pose.x), 2) + pow((target_pose.y - robot_pose.y), 2));

            // Check if the path point is behind the vehicle
            dx = target_pose.x - robot_pose.x;
            dy = target_pose.y - robot_pose.y;
            dot_product = dx * cos(robot_pose.theta) + dy * sin(robot_pose.theta);

            if(distance >= offset && dot_product > 0)
            {
                return target_pose;
            }
            else
            {
                path.x.pop(); path.y.pop(); path.theta.pop();
            }
        }
        // If Path Left is 1
        target_pose.x = path.x.front();
        target_pose.y = path.y.front();
        target_pose.theta = path.theta.front();

        // Check Distance and Theta Error for Last Target Point
        distance = sqrt(pow((target_pose.x - robot_pose.x), 2) + pow((target_pose.y - robot_pose.y), 2));
        theta_error = target_pose.theta - robot_pose.theta;
        
        if(abs(theta_error) >= MATH_PI)
        {
            if(theta_error > 0)
            {
                theta_error = theta_error - 2*MATH_PI;
            }
            else
            {
                theta_error = theta_error + 2*MATH_PI;
            }
        }

        //Untuk Nawab 
        // (distance <= 0.1 && theta_error <= MATH_PI/18 && theta_error >= -MATH_PI/18)
        if(distance <= 0.1 && theta_error <= MATH_PI/18 && theta_error >= -MATH_PI/18) // sebelum : 0.033 and Math pi /36
        {
            // Stop the Robot and Clear Path
            path.x.pop(); target_pose.x = robot_pose.x;
            path.y.pop(); target_pose.y = robot_pose.y;
            path.theta.pop(); target_pose.theta = robot_pose.theta;
            ROS_INFO("Path Finished!");
        }
    }

    // Normal Mode
    else
    {
        // Define Next Target Pose
        target_pose.x = path.x.front();
        target_pose.y = path.y.front();
        target_pose.theta = path.theta.front();

        // Check for Distance Error
        distance = sqrt(pow((path.x.front() - robot_pose.x), 2) + pow((path.y.front() - robot_pose.y), 2));
        
        // Check for Theta Error
        // delta_heading = path.theta.front() - robot_pose.theta;        
        // if(abs(delta_heading) >= MATH_PI)
        // {
        //     if(delta_heading > 0)
        //     {
        //         delta_heading = delta_heading - 2*MATH_PI;
        //     }
        //     else
        //     {
        //         delta_heading = delta_heading + 2*MATH_PI;
        //     }
        // }  

        if(pathLeft > 1)
        {   
            while(distance < offset)
            {
                path.x.pop(); target_pose.x = path.x.front();
                path.y.pop(); target_pose.y = path.y.front();  
                path.theta.pop(); target_pose.theta = path.theta.front();
                if(path.x.size() <= 1)
                    break;
                distance = sqrt(pow((target_pose.x - robot_pose.x), 2) + pow((target_pose.y - robot_pose.y), 2));
            }
        }

        else
        {
            target_pose.x = path.x.front();
            target_pose.y = path.y.front();
            target_pose.theta = path.theta.front()/* - 180 * (MATH_PI/180)*/;

            // Check Distance and Theta Error for Last Target Point
            distance = sqrt(pow((target_pose.x - robot_pose.x), 2) + pow((target_pose.y - robot_pose.y), 2));
            theta_error = target_pose.theta - robot_pose.theta;
            
            if(abs(theta_error) >= MATH_PI)
            {
                if(theta_error > 0)
                {
                    theta_error = theta_error - 2*MATH_PI;
                }
                else
                {
                    theta_error = theta_error + 2*MATH_PI;
                }
            }

            if(distance <= 0.033 && theta_error <= MATH_PI/36 && theta_error >= -MATH_PI/36)
            {
            // Stop the Robot and Clear Path
            path.x.pop(); target_pose.x = robot_pose.x;
            path.y.pop(); target_pose.y = robot_pose.y;
            path.theta.pop(); target_pose.theta = robot_pose.theta;

            // Set Rumble Feedback
            rumble_status = 1;
            MsgJoyRumble.intensity  = 0.5;
            prev_time = ros::Time::now();

            ROS_INFO("Path Finished!");
            }
        }
    }
    return target_pose;
}

Robot::Pose_t Robot::PointToPointPID(Pose_t robot_pose, Pose_t target_pose)
{
    Pose_t robot_vel;
    robot_vel.x = robot_vel.y = robot_vel.theta = 0.0;

    float output[3] = {0.0, 0.0, 0.0};

    float proportional[3] = {0.0, 0.0, 0.0};
    float integral[3] = {0.0, 0.0, 0.0};
    float derivative [3]= {0.0, 0.0, 0.0};

    float error[3] = {0.0, 0.0, 0.0};
    static float prevError[3] = {0.0, 0.0, 0.0};
    static float sumError[3] = {0.0, 0.0, 0.0};

    float kp[3] = {2.3, 2.3, 0.3}; //{2.5, 2.5, 0.2}
    float ki[3] = {0.0, 0.0, 0.0}; // {0.0005, 0.0005, 0.0};
    float kd[3] = {1.8, 1.8, 0.27}; // {1.8, 1.8, 0.13}


    error[0] = target_pose.x - robot_pose.x;
    error[1] = target_pose.y - robot_pose.y;
    error[2] = target_pose.theta - robot_pose.theta;

    if(abs(error[2]) >= MATH_PI){
        if(error[2] > 0){
            error[2] = error[2] - 2*MATH_PI;
        }
        else{
            error[2] = error[2] + 2*MATH_PI;
        }
    }

    for(int i=0 ; i<=2 ; i++){
        sumError[i] += error[i];

        proportional[i] = kp[i] * error[i];
        integral[i] = ki[i] * sumError[i];
        derivative[i] = kd[i] * (error[i] - prevError[i]);

        prevError[i] = error[i];

        output[i] = proportional[i] + integral[i] + derivative[i];
    }

    // Get Theta Control Only if Heading Error more than 36 degree
    if(error[2] >= MATH_PI/6 || error[2] <= -MATH_PI/6)
    {
        output[0] = 0.0;
        output[1] = 0.0;
    }
    
    robot_vel.x = output[0]*100;
    robot_vel.y = output[1]*100;
    robot_vel.theta = output[2]*100;

    return robot_vel;
}

Robot::Pose_t Robot::PointToPointPIDV2(Pose_t robot_pose, Pose_t target_pose){
    Pose_t robot_vel;

    robot_vel.x = robot_vel.y = robot_vel.theta = 0.0;

    float output[2] = {0.0, 0.0};

    float proportional[2] = {0.0, 0.0};
    float integral[2] = {0.0, 0.0};
    float derivative [2]= {0.0, 0.0};
    
    float error[2] = {0.0, 0.0}; //Error Heading = [0], Error Distance = [1]
    static float prevError[2] = {0.0, 0.0};
    static float sumError[2] = {0.0, 0.0};

    float kp[2] = {0.2, 0.5};
    float ki[2] = {0, 0};
    float kd[2] = {0.17, 0};

    error[0] = atan2((target_pose.y - robot_pose.y), (target_pose.x - robot_pose.x));
    error[1] = sqrt(pow((target_pose.x - robot_pose.y), 2) + pow((target_pose.y - robot_pose.y), 2));

    for(int i=0 ; i<=1 ; i++){
        sumError[i] += error[i];

        proportional[i] = kp[i] * error[i];
        integral[i] = ki[i] * sumError[i];
        derivative[i] = kd[i] * (error[i] - prevError[i]);

        prevError[i] = error[i];

        output[i] = proportional[i] + integral[i] + derivative[i];
    }

    robot_vel.theta = output[0];
    robot_vel.x = output[1]*100;
    robot_vel.y = 0;

    std::cout << "[" << robot_vel.x << "," << robot_vel.y << "," << robot_vel.theta << "]" << std :: endl;
    return robot_vel;
}

Robot::Pose_t Robot::PointToPointLQR(Pose_t robot_pose, Pose_t target_pose, float maxSpeed)
{
    const int maxIterations = 100;
    const double convergenceThreshold = 1e-6;

    float error[3] = {0, 0, 0};
    float output[3] = {0, 0, 0};
    float dt = 1;

    Pose_t robot_vel;
    robot_vel.x = robot_vel.y = robot_vel.theta = 0;


    error[0] = target_pose.x - robot_pose.x;
    error[1] = target_pose.y - robot_pose.y;
    error[2] = target_pose.theta - robot_pose.theta;

    // Nearest Angle
    if(abs(error[2]) >= MATH_PI){
        if(error[2] > 0){
            error[2] = error[2] - 2*MATH_PI;
        }
        else{
            error[2] = error[2] + 2*MATH_PI;
        }
    }

    // Define the system dynamics matrices A, B
    Eigen::MatrixXd A(3, 3);
    A << 1, 0, 0,
         0, 1, 0,
         0, 0, 1;

    Eigen::MatrixXd B(3, 3);
    B << -1*dt,     0,     0,
             0, -1*dt,     0,
             0,     0, -1*dt;

    // Define the Q and R matrices
    // P2P Q = diag(30, 30, 10)
    // PTrack diag(225, 225, 50)
    Eigen::MatrixXd Q(3, 3);
    Q << 225, 0, 0,
         0, 225, 0,
         0, 0, 25;

    Eigen::MatrixXd R(3, 3);
    R << 1, 0, 0,
         0, 1, 0,
         0, 0, 1;

    // Solve the Algebraic Riccati Equation
    Eigen::MatrixXd P = Q;

    // Perform the iterative computation
    for (int i = 0; i < maxIterations; ++i) {
        Eigen::MatrixXd P_prev = P;
        P = A.transpose() * P * A - A.transpose() * P * B * (R + B.transpose() * P * B).inverse() * B.transpose() * P * A + Q;
        // std::cout << P << std::endl;

        // Check for convergence
        double normDiff = (P - P_prev).norm();
        if (normDiff < convergenceThreshold) {
            std::cout << "Convergence achieved after " << i+1 << " iterations." << std::endl;
            break;
        }
    }

    // Calculate the gain matrix K
    // Eigen::MatrixXd K = (R + B.transpose() * P * B).inverse() * B.transpose() * P * A;
    Eigen::MatrixXd K = R.inverse() * B.transpose() * P;

    Eigen::Vector3d Error(error[0], error[1], error[2]);
    Eigen::Vector3d U = -K * Error;

    // Extract individual control inputs
    output[0] = U[0];
    output[1] = U[1];
    output[2] = U[2];

    // Speed Limiter Only
    for(int i=0 ; i<=2 ; i++){
        if(output[i] >= maxSpeed){
            output[i] = maxSpeed;
        }
        else if(output[i] <= -maxSpeed){
            output[i] = -maxSpeed;
        }
    }

    robot_vel.x = output[0];
    robot_vel.y = output[1];
    robot_vel.theta = output[2];

    // Display the optimal LQR Parameter
    std::cout << "Optimal control gain matrix K:" << std::endl;
    std::cout << K << std::endl;
    std::cout << "error E " << std::endl;
    std::cout << Error << std::endl;
    std::cout << "control U " << std::endl;
    std::cout << U << std::endl;
    // Display the control inputs
    std::cout << "Control inputs:" << std::endl;
    std::cout << "robot_vel.x: " << robot_vel.x << std::endl;
    std::cout << "robot_vel.y: " << robot_vel.y << std::endl;
    std::cout << "robot_vel.z: " << robot_vel.theta << std::endl;

    return robot_vel;

}

Robot::Pose_t Robot::Global_to_Local_Vel(Pose_t robot_pose, Pose_t global_vel)
{
    Pose_t local_vel;

    local_vel.x = global_vel.x * sin(robot_pose.theta) - global_vel.y * cos(robot_pose.theta);
    local_vel.y = global_vel.x * cos(robot_pose.theta) + global_vel.y * sin(robot_pose.theta);
    local_vel.theta = global_vel.theta;

    return local_vel;
}
