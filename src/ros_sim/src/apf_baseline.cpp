#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <nav_core/base_local_planner.h>
#include <tf2_ros/buffer.h>
#include <tf2/utils.h>
#include <tf/tf.h>
#include <base_local_planner/goal_functions.h>
#include <iostream>
#include <vector>
#include <cmath>
#include <std_msgs/Float32.h>
#include <std_msgs/Float32MultiArray.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <mutex>

#include <pluginlib/class_list_macros.h>

struct Coordinate {
    double x, y;
};

struct robotState {
    Coordinate pos;
    double yaw;
    double v_linear;
    double v_angular;
};

namespace APF {

class APFPlanner : public nav_core::BaseLocalPlanner {
    private:
        bool initialized_ = false;
        std::vector<geometry_msgs::PoseStamped> global_plan_;
        tf2_ros::Buffer* tf_;
        costmap_2d::Costmap2DROS* costmap_ros_;

        // APF variables
        double k_att = 2.0;
        double f_att_x = 0.0;
        double f_att_y = 0.0;
        double k_rep = 1.0;
        double f_rep_x = 0.0;
        double f_rep_y = 0.0;
        double d0 = 0.8;

        double traversability_score_ = 1.0;
        double corridor_width_ = 300.0;
        double center_path_ = 320.0;

        // ====== THERMAL ZONE (backup-camera: lebar 3 zona = lebar robot) ======
        // 1.0 = zona lapang (bisa dilewati), 0.0 = zona terhalang.
        double zone_left_   = 1.0;
        double zone_center_ = 1.0;
        double zone_right_  = 1.0;
        int    zone_turn_dir_ = 0;   // commitment: 0 none, +1 belok kiri, -1 kanan

        ros::Subscriber trav_sub_;
        ros::Subscriber width_sub_;
        ros::Subscriber center_sub_;
        ros::Subscriber thermal_sub_;
        ros::Subscriber zone_sub_;
        
        cv::Mat thermal_semantic_;
        std::mutex thermal_mutex_;

        // Fallback within fallback SM variables
        enum apf_state {NORMAL, AVOIDANCE};

        apf_state current_state_ = NORMAL;

        double prev_ftotal_x = 0.0;
        double prev_ftotal_y = 0.0;

        int local_minima_counter_ = 0;
        int local_minima_threshold_ = 20; // 20 cycles of oscillation before switching to avoidance

        int avoidance_counter_ = 0;
        int avoidance_threshold_ = 80; // 8 second time to avoid
      
        double v_target_x = 0.0;
        double v_target_y = 0.0;

        // Wp tracking variables
        int current_progress_id = 0;

        double lookahead_dist = 0.5;
        double target_yaw = 0.0;

        bool is_safe_ = true;

        ros::Time last_avoidance_time_ = ros::Time(0);

        ros::Time last_pose_time_ = ros::Time(0);
        double last_pose_x_ = 0.0;
        double last_pose_y_ = 0.0;
        double stuck_time_threshold_ = 6.0; // seconds awal 6
        double stuck_distance_threshold_ = 0.01; // meters 

        bool pathCost(double start_x, double start_y, double end_x, double end_y, costmap_2d::Costmap2D* costmap){
            double dist = std::hypot(end_x - start_x, end_y - start_y);
            int steps = std::max(1, (int)std::ceil(dist / (costmap->getResolution()))); // std max to avoid division by zero when robot is too close to goal

            // Sampling points along the line from start to end using linear interpolation (parameteric)
            for(int i = 0; i <= steps; ++i){
                double t = (double)i / steps;
                double sample_x = start_x + t * (end_x - start_x);
                double sample_y = start_y + t * (end_y - start_y);

                unsigned int mx, my;
                if(costmap->worldToMap(sample_x, sample_y, mx, my)){
                    if(costmap->getCost(mx, my) >= 160){ // So robot stays in avoidance mode in tight spaces.
                        return false;
                    }
                }
                else {
                    return false;
                }
            }
            return true;
        }
        
        void traversabilityCallback(
            const std_msgs::Float32::ConstPtr& msg)
        {
            traversability_score_ = msg->data;
        }

        void corridorWidthCallback(
            const std_msgs::Float32::ConstPtr& msg)
        {
            corridor_width_ = msg->data;
        }

        void centerPathCallback(
            const std_msgs::Float32::ConstPtr& msg)
        {
            center_path_ = msg->data;
        }

        // Zona thermal [left, center, right]. Di-EMA biar tidak jitter.
        void zoneCallback(
            const std_msgs::Float32MultiArray::ConstPtr& msg)
        {
            if(msg->data.size() < 3) return;
            double a = 0.5; // faktor smoothing (0=lambat, 1=mentah)
            zone_left_   = (1.0 - a) * zone_left_   + a * msg->data[0];
            zone_center_ = (1.0 - a) * zone_center_ + a * msg->data[1];
            zone_right_  = (1.0 - a) * zone_right_  + a * msg->data[2];
        }
        
        void thermalCallback(
            const sensor_msgs::Image::ConstPtr& msg)
        {
            std::lock_guard<std::mutex> lock(thermal_mutex_);
            try {
                cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, msg->encoding);
                thermal_semantic_ = cv_ptr->image.clone();
            } catch (cv_bridge::Exception& e) {
                ROS_ERROR("cv_bridge exception: %s", e.what());
            }
        }

    public:
        APFPlanner() {}
        APFPlanner(std::string name, tf2_ros::Buffer* tf, costmap_2d::Costmap2DROS* costmap_ros){
            initialize(name, tf, costmap_ros);
        }

        void initialize(std::string name, tf2_ros::Buffer* tf, costmap_2d::Costmap2DROS* costmap_ros){
            if(!initialized_){
                ros::NodeHandle private_nh("~/" + name);

                tf_ = tf;
                costmap_ros_ = costmap_ros;

                initialized_ = true;
                ros::NodeHandle nh;
                // ========================================
                // THERMAL FUSION ENABLED FOR S1-S4
                // ========================================
                trav_sub_ = nh.subscribe(
                    "/traversability_score",
                    1,
                    &APFPlanner::traversabilityCallback,
                    this
                );
                width_sub_ = nh.subscribe(
                    "/corridor_width",
                    1,
                    &APFPlanner::corridorWidthCallback,
                    this
                );
                center_sub_ = nh.subscribe(
                    "/center_path",
                    1,
                    &APFPlanner::centerPathCallback,
                    this
                );
                zone_sub_ = nh.subscribe(
                    "/thermal_zones",
                    1,
                    &APFPlanner::zoneCallback,
                    this
                );
                // ========================================
                // THERMAL HOT-OBSTACLE MASK
                // ========================================
                thermal_sub_ = nh.subscribe(
                    "/thermal_obstacle_mask",
                    1,
                    &APFPlanner::thermalCallback,
                    this
                );
                ROS_INFO("APF Planner Plugin initialized (THERMAL TRAVERSABILITY FUSION).");
            }
        }

        bool setPlan(const std::vector<geometry_msgs::PoseStamped>& plan){
            global_plan_ = plan;
            current_progress_id = 0;

            return true;
        }

        bool computeVelocityCommands(geometry_msgs::Twist& cmd_vel){
            if(global_plan_.empty()) return false;

            // ========================================
            // DEBUG: VERIFY THERMAL DATA
            // ========================================
            ROS_INFO_THROTTLE(
                1.0,
                "THERMAL: TRAV=%.2f WIDTH=%.1f CENTER=%.1f",
                traversability_score_,
                corridor_width_,
                center_path_
            );

            // reset forces
            f_att_x = 0.0; f_att_y = 0.0;
            f_rep_x = 0.0; f_rep_y = 0.0;

            // reset obs vars
            double min_dist_obs = 1000000.0;
            double closest_obs_x = 0.0;
            double closest_obs_y = 0.0;
            bool found_any_obs = false;

            // read the robot's current position
            geometry_msgs::PoseStamped global_pose;
            costmap_ros_->getRobotPose(global_pose);
            double robot_x = global_pose.pose.position.x;
            double robot_y = global_pose.pose.position.y;
            double robot_yaw = tf2::getYaw(global_pose.pose.orientation);

            ros::Time now = ros::Time::now();

            if(last_pose_time_ == ros::Time(0)){
                last_pose_time_ = now;
                last_pose_x_ = robot_x;
                last_pose_y_ = robot_y;
            }

            if((now - last_pose_time_).toSec() > stuck_time_threshold_){
                double dist_moved = std::hypot(robot_x - last_pose_x_, robot_y - last_pose_y_);
                
                if(dist_moved < stuck_distance_threshold_){
                    ROS_WARN("APF: Robot seems to be stuck either from local minima or flickering states.");
                    current_state_ = AVOIDANCE;
                    last_pose_time_ = now;
                    last_pose_x_ = robot_x;
                    last_pose_y_ = robot_y;
                    avoidance_counter_ = 0;
                }
                else {
                    last_pose_time_ = now;
                    last_pose_x_ = robot_x;
                    last_pose_y_ = robot_y;
                }
            }

            // read the robot's goal (crucial for final check)
            double goal_x = global_plan_.back().pose.position.x;
            double goal_y = global_plan_.back().pose.position.y;
            double goal_yaw = tf2::getYaw(global_plan_.back().pose.orientation);

            std::vector<geometry_msgs::PoseStamped> transformed_plan;
            
            if (!base_local_planner::transformGlobalPlan(
                    *tf_, 
                    global_plan_, 
                    global_pose, 
                    *(costmap_ros_->getCostmap()), 
                    costmap_ros_->getGlobalFrameID(), 
                    transformed_plan)) {
                
                ROS_WARN("APF Debugger: TF Transform failed. Halting robot for safety.");
                cmd_vel.linear.x = 0.0;
                cmd_vel.linear.y = 0.0;
                cmd_vel.angular.z = 0.0;
                return false; 
            }

            if(transformed_plan.empty()){
                ROS_WARN("APF Debugger: Transformed plan is empty. Halting robot for safety.");
                cmd_vel.linear.x = 0.0;
                cmd_vel.linear.y = 0.0;
                cmd_vel.angular.z = 0.0;
                return false; 
            }

            double target_x = robot_x;
            double target_y = robot_y;

            int closest_index = 0;
            double min_dist = 1000000.0;

            // Read costmap_grid
            costmap_2d::Costmap2D* costmap = costmap_ros_->getCostmap();
            int size_x = costmap->getSizeInCellsX();
            int size_y = costmap->getSizeInCellsY();

            for(int i = current_progress_id; i < transformed_plan.size(); ++i){
                double wp_x = transformed_plan[i].pose.position.x;
                double wp_y = transformed_plan[i].pose.position.y;

                double dist_to_wp = std::hypot(wp_x - robot_x, wp_y - robot_y);

                if(dist_to_wp < min_dist){
                    min_dist = dist_to_wp;
                    closest_index = i;
                }
            }

            double scan_radius = 1.0; 
            int radius_cells = std::ceil(scan_radius / costmap->getResolution());

            unsigned int robot_mx, robot_my;
            if (costmap->worldToMap(robot_x, robot_y, robot_mx, robot_my)) {
                
                int min_x = std::max(0, (int)robot_mx - radius_cells);
                int max_x = std::min((int)size_x - 1, (int)robot_mx + radius_cells);
                int min_y = std::max(0, (int)robot_my - radius_cells);
                int max_y = std::min((int)size_y - 1, (int)robot_my + radius_cells);

                for(int i = min_x; i <= max_x; ++i){
                    for(int j = min_y; j <= max_y; ++j){

                        if(costmap->getCost(i,j) >= 200){

                            double obs_x;
                            double obs_y;
                            costmap->mapToWorld(i, j, obs_x, obs_y);

                            double dist_to_obs = std::hypot(obs_x - robot_x, obs_y - robot_y);

                            if(dist_to_obs < 0.7){
                                if(dist_to_obs < min_dist_obs){
                                    min_dist_obs = dist_to_obs;
                                    closest_obs_x = obs_x;
                                    closest_obs_y = obs_y;
                                    found_any_obs = true;
                                }
                            }
                        }
                    }
                }
            } else {
                ROS_WARN_THROTTLE(1.0, "APF: Robot is off the costmap!");
            }

            // Scan for obstacles
            // for(int i = 0; i < size_x; ++i){
            //     for(int j = 0; j < size_y; ++j){

            //         if(costmap->getCost(i,j) >= 50){

            //             double obs_x;
            //             double obs_y;
            //             costmap->mapToWorld(i, j, obs_x, obs_y);

            //             double dist_to_obs = std::sqrt(std::pow(obs_x - robot_x, 2) + std::pow(obs_y - robot_y, 2));

            //             if(dist_to_obs < d0 && dist_to_obs > 0.35){
            //                 if(dist_to_obs < min_dist_obs){
            //                     min_dist_obs = dist_to_obs;
            //                     closest_obs_x = obs_x;
            //                     closest_obs_y = obs_y;
            //                     found_any_obs = true;
            //                 }
            //             }
            //         }
            //     }
            // }

            current_progress_id = closest_index;

            int best_id = current_progress_id;
            double min_dist_lookahead = 1000000.0;
            double search_dist = lookahead_dist;

            double final_x = transformed_plan.back().pose.position.x;
            double final_y = transformed_plan.back().pose.position.y;
            double dist_to_final = std::hypot(final_x - robot_x, final_y - robot_y);

            // Dynamic lookahead setup if needed, if not just go with lookahead dist.
            if(found_any_obs == true){
                search_dist = lookahead_dist; 
            }

            if(dist_to_final < 0.4){
                target_x = final_x;
                target_y = final_y;
            }
            else {

                int cell_R = std::ceil((lookahead_dist / 2) / costmap->getResolution());

                for(int i = current_progress_id; i < transformed_plan.size(); ++i){
                    double wp_x = transformed_plan[i].pose.position.x;
                    double wp_y = transformed_plan[i].pose.position.y;
                    double dist_to_wp = std::hypot(wp_x - robot_x, wp_y - robot_y);
                
                    unsigned int mx, my;
                    bool is_safe_ = true;
                    if (costmap->worldToMap(wp_x, wp_y, mx, my)) {
                        for(int dx = -cell_R; dx <= cell_R; ++dx){
                            for(int dy = -cell_R; dy <= cell_R; ++dy){

                                int check_x = mx + dx;
                                int check_y = my + dy;

                                if(check_x >= 0 && check_x < size_x && check_y >= 0 && check_y < size_y){
                                    if(costmap->getCost(check_x, check_y) >= 200){
                                        is_safe_ = false;
                                        break;
                                    }
                                }
                            }
                            if(is_safe_ == false) break;
                        }
                    }

                    if(is_safe_ == false){
                        continue;
                    }

                    best_id = i;

                    if(dist_to_wp >= search_dist){
                        break;
                    }
                }

                if(best_id >= transformed_plan.size()) best_id = transformed_plan.size() - 1;

                target_x = transformed_plan[best_id].pose.position.x;
                target_y = transformed_plan[best_id].pose.position.y;
            }

            // Attractive force calculation
            f_att_x = k_att * (target_x - robot_x);
            f_att_y = k_att * (target_y - robot_y);

            double att_mag = std::hypot(f_att_x, f_att_y);

            double max_att_mag = k_att * lookahead_dist;

            if(att_mag > max_att_mag){
                double scale = max_att_mag / att_mag;

                f_att_x *= scale;
                f_att_y *= scale;
            }

            // ========================================
            // THERMAL OBSTACLE REPULSION
            // ========================================
            double thermal_rep_x = 0.0;
            double thermal_rep_y = 0.0;

            {
                std::lock_guard<std::mutex> lock(thermal_mutex_);
                if (!thermal_semantic_.empty()) {
                    cv::Mat thermal_gray;

                    if (thermal_semantic_.channels() == 1) {
                        thermal_gray = thermal_semantic_;
                    }
                    else {
                        cv::cvtColor(thermal_semantic_, thermal_gray, cv::COLOR_BGR2GRAY);
                    }

                    int h = thermal_gray.rows;
                    int w = thermal_gray.cols;
                    double sum_x = 0.0;
                    double sum_y = 0.0;
                    double hot_count = 0.0;

                    for (int y = h / 3; y < h; y += 8) {
                        for (int x = 0; x < w; x += 8) {
                            if (thermal_gray.at<unsigned char>(y, x) < 100) {
                                continue;
                            }

                            sum_x += x;
                            sum_y += y;
                            hot_count += 1.0;
                        }
                    }

                    if (hot_count > 8.0) {
                        double cx = sum_x / hot_count;
                        double cy = sum_y / hot_count;
                        double obstacle_side = (cx - (w / 2.0)) / (w / 2.0);
                        double closeness = std::max(0.0, (cy - (h / 3.0)) / (h * 2.0 / 3.0));
                        double strength = std::min(0.35, 0.12 + 0.25 * closeness);

                        if (std::abs(obstacle_side) < 0.15) {
                            obstacle_side = (center_path_ >= 320.0) ? -0.4 : 0.4;
                        }

                        double local_rep_x = -0.04 * strength;
                        double local_rep_y = obstacle_side * strength;

                        thermal_rep_x = local_rep_x * std::cos(robot_yaw) - local_rep_y * std::sin(robot_yaw);
                        thermal_rep_y = local_rep_x * std::sin(robot_yaw) + local_rep_y * std::cos(robot_yaw);
                    }

                    ROS_INFO_THROTTLE(
                        1.0,
                        "THERMAL_OBS_REP: %.2f %.2f",
                        thermal_rep_x,
                        thermal_rep_y
                    );
                }
            }

            double f_total_x = 0.0;
            double f_total_y = 0.0;

            // Planner state management
            if(current_state_ == NORMAL){

                // reset tuning variables
                // k_rep = 1.0; 
                // d0 = 0.8; 
                k_rep = 0.7;
                d0 = 0.7;

                // Repulsive force calculation

                if(found_any_obs == true){
                    if(min_dist_obs < 0.15) min_dist_obs = 0.15;

                    // ======================================
                    // THERMAL ADAPTIVE GAIN (NON-FUZZY)
                    // ======================================
                    // Repulsive force is scaled by thermal traversability
                    // Poor terrain → stronger repulsion (be more cautious)
                    // Good terrain → normal repulsion
                    // ======================================
                    double rep_gain = 1.0 + (1.0 - traversability_score_);

                    ROS_INFO_THROTTLE(
                        1.0,
                        "APF NORMAL: rep_gain=%.3f (trav=%.2f)",
                        rep_gain,
                        traversability_score_
                    );

                    double f_rep =
                        rep_gain *
                        k_rep *
                        ((1.0 / min_dist_obs) - (1.0 / d0))
                        * std::pow((1.0 / min_dist_obs), 2);
                        
                    // X and Y components of repulsive force
                    f_rep_x = f_rep * ((robot_x - closest_obs_x) / min_dist_obs);
                    f_rep_y = f_rep * ((robot_y - closest_obs_y) / min_dist_obs);
                }

                // Total force calculation
                f_total_x = f_att_x + f_rep_x + thermal_rep_x;
                f_total_y = f_att_y + f_rep_y + thermal_rep_y;

                double prev_mag = std::hypot(prev_ftotal_x, prev_ftotal_y);
                double curr_mag = std::hypot(f_total_x, f_total_y);

                if(curr_mag > 0.01 && prev_mag > 0.01){

                    double num = (f_total_x * prev_ftotal_x) + (f_total_y * prev_ftotal_y);
                    double denum = curr_mag * prev_mag;

                    double cos_theta = num / denum; // Normalized value based on the research paper

                    double oscillation_threshold = -0.3; // Lower = more strict, Higher = more tolerant. 

                    if(cos_theta < oscillation_threshold){
                        if((ros::Time::now() - last_avoidance_time_).toSec() > 3.0){
                            ROS_WARN("APF : Oscillation detected. Switching to AVOIDANCE mode.");
                            current_state_ = AVOIDANCE;
                            local_minima_counter_ = 0;
                        }
                    }
                }

            }

            else if(current_state_ == AVOIDANCE){
                avoidance_counter_++;

                // k_rep = 0.01; 
                // d0 = 0.6;
                k_rep = 1.0;
                d0 = 1.0;

                if(avoidance_counter_ > avoidance_threshold_){
                    ROS_WARN("APF : Stuck in avoidance for too long. Let's see what PSO can do.");
                    current_state_ = NORMAL;
                    avoidance_counter_ = 0;
                    return false;
                }

                // Virtual point generation for manuvering around obstacles
                int num_points = 72;
                double R = std::max(0.45, search_dist);

                double best_score = -1000000.0;
                double best_vx = target_x;
                double best_vy = target_y;
                double global_target_x = target_x;
                double global_target_y = target_y;

                // Weights for tuning
                double K1 = 1.0; // Momentum : Sudut menguntungkan ke arah waypoint yang dituju
                double K2 = 0.5; // Direction : Sudut menguntungkan dilihat dari orientasi robot
                double K3 = 3.0; // Safety : Jarak aman dari rintangan
                double K4 = 4.0; // Costmap penalty
                double K5 = 1.8; // Progress : Seberapa jauh dari titik virtual ke waypoint

                for(int i = 0; i < num_points; ++i){
                    double angle = (i * (2.0 * M_PI / num_points));
                    double vx = robot_x + R * std::cos(angle);
                    double vy = robot_y + R * std::sin(angle);

                    // Cek untuk memastikan titik virtual tidak dibawah rintangan
                    unsigned int mx, my;
                    bool is_valid = true;
                    if(costmap->worldToMap(vx, vy, mx, my)){
                        if(costmap->getCost(mx, my) >= 160){
                            is_valid = false;
                        }
                    }
                    else {
                        is_valid = false;
                    }

                    if(is_valid == false) continue;

                    // Rumus evaluasi sesuai paper yang dibaca

                    // theta 1 
                    double v_yaw = std::atan2(vy - robot_y, vx - robot_x);
                    double theta1_diff = v_yaw - robot_yaw;
                    double cos_theta1 = std::cos(theta1_diff);

                    // theta 2
                    double goal_dir = std::atan2(target_y - robot_y, target_x - robot_x);
                    double theta2_diff = v_yaw - goal_dir;
                    double cos_theta2 = std::cos(theta2_diff);

                    // l1
                    double l1 = 0.5;
                    if(found_any_obs == true){
                        l1 = std::min(1.0, std::hypot(vx - closest_obs_x, vy - closest_obs_y));
                    }

                    double cost_penalty = 0.0;

                    if(costmap->worldToMap(vx, vy, mx, my)){
                        cost_penalty = (double)costmap->getCost(mx,my) / 252.0;
                    }

                    // l2 (negative karena semakin jauh dari goal semakin buruk)
                    double l2 = -std::hypot(vx - target_x, vy - target_y);

                    double eval_score =
                        (K1 * cos_theta1)
                        + (K2 * cos_theta2)
                        + (K3 * l1)
                        - (K4 * cost_penalty)
                        + (K5 * l2);

                    if(eval_score > best_score){
                        best_score = eval_score;
                        best_vx = vx;
                        best_vy = vy;
                    }
                }

                if(best_score == -1000000.0){
                    ROS_WARN("APF : No valid virtual point found! backing out slowly.");
                    cmd_vel.linear.x = -0.05;
                    cmd_vel.linear.y = 0.0;
                    cmd_vel.angular.z = 0.2;
                    return true;
                }


                // Cek udah cukup aman atau belum
                if(pathCost(robot_x, robot_y, global_target_x, global_target_y, costmap) == true && min_dist_obs > 0.6){
                    ROS_INFO("APF : Safe enough, let's go back to NORMAL mode.");
                    current_state_ = NORMAL;
                    avoidance_counter_ = 0;

                    last_avoidance_time_ = ros::Time::now();
                }
                target_x = best_vx;
                target_y = best_vy;

                f_att_x = k_att * (target_x - robot_x);
                f_att_y = k_att * (target_y - robot_y);

                double new_att_mag = std::hypot(f_att_x, f_att_y);
                double new_max_att_mag = k_att * lookahead_dist;

                if(new_att_mag > new_max_att_mag){
                    double scale = new_max_att_mag / new_att_mag;
                    f_att_x *= scale;
                    f_att_y *= scale;
                }

                // Repulsive force calculation

                if(found_any_obs == true){
                    if(min_dist_obs < 0.15) min_dist_obs = 0.15;

                    // ======================================
                    // THERMAL ADAPTIVE GAIN (NON-FUZZY)
                    // ======================================
                    // Repulsive force is scaled by thermal traversability
                    // Poor terrain → stronger repulsion (be more cautious)
                    // Good terrain → normal repulsion
                    // ======================================
                    double rep_gain = 1.0 + (1.0 - traversability_score_);

                    ROS_INFO_THROTTLE(
                        1.0,
                        "APF AVOIDANCE: rep_gain=%.3f (trav=%.2f)",
                        rep_gain,
                        traversability_score_
                    );

                    double f_rep =
                        rep_gain *
                        k_rep *
                        ((1.0 / min_dist_obs) - (1.0 / d0))
                        * std::pow((1.0 / min_dist_obs), 2);
                                            
                    // X and Y components of repulsive force
                    f_rep_x = f_rep * ((robot_x - closest_obs_x) / min_dist_obs);
                    f_rep_y = f_rep * ((robot_y - closest_obs_y) / min_dist_obs);
                }

                // Total force calculation
                f_total_x = f_att_x + f_rep_x + thermal_rep_x;
                f_total_y = f_att_y + f_rep_y + thermal_rep_y;
            }

            // Save current total forces as previous for calculation if needed
            prev_ftotal_x = f_total_x; 
            prev_ftotal_y = f_total_y; 


            // Yaw calculation
            target_yaw = std::atan2(f_total_y, f_total_x);

            // ============================================================
            // [SMOOTH-EDIT] THERMAL CORRIDOR STEERING (ikuti jalan yang terbuka)
            // Kalau tampilan thermal depan menunjukkan jalan menyempit/buntu
            // (traversability rendah), heading langsung dibiaskan ke kolom
            // paling lapang (center_path_) supaya robot menepi ke celah lebih
            // AWAL & halus -- bukan jalan sampai mepet tembok, berhenti, muter.
            // Ini yang diminta dosen: "scan depan buntu -> langsung belok".
            // center_path_ : 0..640, tengah = 320.
            // CATATAN: kalau beloknya ke arah SALAH, balik tanda 'err'.
            // Set thermal_steer_gain = 0.0 untuk mematikan.
            // ============================================================
            // ============================================================
            // [SMOOTH-EDIT] THERMAL CORRIDOR STEERING
            // DIMATIKAN (gain 0.0). Penyebab "godek-godek": center_path_
            // noisy -> heading dibiaskan kiri-kanan-kiri-kanan. Penghindaran
            // sekarang ditangani costmap (virtual laserscan thermal) + APF
            // yang sudah mulus, jadi bias ini tidak diperlukan.
            //
            // Kalau nanti mau cornering proaktif lagi, set gain kecil (mis.
            // 0.25) DAN tambahkan filter/deadband pada center_path_ dulu
            // supaya tidak jitter. Untuk sekarang biarkan 0.0.
            // ============================================================
            double thermal_steer_gain = 0.0;
            if(current_state_ == NORMAL && traversability_score_ < 0.6 && thermal_steer_gain > 0.0){
                double err = (center_path_ - 320.0) / 320.0;   // [-1 .. +1]
                target_yaw += thermal_steer_gain * err;
            }

            // Velocity calculation
            double max_vel = 0.5;
            double vel_x = f_total_x;
            double vel_y = f_total_y;
            // [SMOOTH-EDIT] k_theta 0.25->0.6 : koreksi heading lebih tegas
            double k_theta = 0.6;
            // [SMOOTH-EDIT] cap angular 0.2->0.5 : boleh belok cepat (anti "diem-muter")
            double max_angular_vel = 0.5;
            double min_angular_vel = -0.5;

            double vel_mag = std::hypot(f_total_x, f_total_y);

            if(vel_mag > max_vel){

                double scale = max_vel / vel_mag;

                vel_x *= scale;
                vel_y *= scale;
            }

            // Driving the path
            double raw_yaw_diff = target_yaw - robot_yaw;
            double yaw_diff = std::atan2(std::sin(raw_yaw_diff), std::cos(raw_yaw_diff));
            double angular_vel = k_theta * yaw_diff;

            double local_vel_x = vel_x * std::cos(robot_yaw) + vel_y * std::sin(robot_yaw);
            double local_vel_y = -vel_x * std::sin(robot_yaw) + vel_y * std::cos(robot_yaw);

            double align_tolerance = 1.2; //~37 degrees

            // ============================================================
            // [SMOOTH-EDIT] SMOOTH TURN COUPLING (anti "diem-muter-diem-muter")
            // Kecepatan linear diskalakan KONTINYU terhadap error heading,
            // jadi robot "menikung" masuk ke belokan, bukan berhenti lalu
            // muter di tempat lalu jalan lagi.
            //   yaw_diff = 0     -> faktor 1.0 (jalan penuh)
            //   yaw_diff = 90deg -> faktor ~0.15 (nyaris cuma muter, tapi
            //                       angular sudah cepat jadi langsung lewat)
            // Set ke 1.0 (matikan) kalau mau perilaku lama.
            // ============================================================
            double turn_speed_factor = std::cos(yaw_diff);
            if(turn_speed_factor < 0.15) turn_speed_factor = 0.15;
            local_vel_x *= turn_speed_factor;
            local_vel_y *= turn_speed_factor;
            (void)align_tolerance; // dibiarkan agar tidak mengubah deklarasi lain

            double dist_to_goal = std::sqrt(std::pow(goal_x - robot_x, 2) + std::pow(goal_y - robot_y, 2));

            // Final alignment
            if(dist_to_goal < 0.2){
                local_vel_x = 0.0;
                local_vel_y = 0.0;

                double raw_final_error = goal_yaw - robot_yaw;
                double final_error = std::atan2(std::sin(raw_final_error), std::cos(raw_final_error));
                
                angular_vel = k_theta * final_error;
            }

            // ============================================================
            // [ZONE-GATE] BACKUP-CAMERA: apakah robot MUAT lewat? ke mana?
            // ------------------------------------------------------------
            // 3 zona thermal (lebar = lebar robot). zone=1 lapang, 0 buntu.
            //   - center buntu, samping ada yg lapang -> pelan + belok ke situ
            //   - SEMUA buntu -> STOP maju, putar pelan cari celah (anti tabrak)
            //   - center lapang -> lepas commitment, jalan normal
            // Commitment (zone_turn_dir_) mencegah godek-godek: sekali pilih
            // sisi, tahan sampai depan lapang lagi.
            // CATATAN ARAH: kalau belok ke sisi yang SALAH, balik tanda
            // (+1 <-> -1) di baris "zone_turn_dir_ = ...".
            // Set zone_enable=false untuk mematikan seluruh blok ini.
            // ============================================================
            bool   zone_enable     = true;
            double zone_thresh     = 0.55;  // center < ini -> dianggap terhalang
            double zone_block_all  = 0.35;  // semua zona < ini -> buntu total
            double zone_turn_rate  = 0.5;   // rad/s tambahan saat menghindar
            // [STUCK-FIX] KUNCI KEAMANAN: gate hanya boleh bertindak kalau
            // thermal MEMANG melihat halangan (traversability rendah). Kalau
            // depan lapang (TRAV tinggi), robot WAJIB jalan -- mencegah stuck
            // gara-gara segmentasi lantai per-zona yang kadang patchy.
            bool front_has_obstacle = (traversability_score_ < 0.6);
            if(zone_enable && front_has_obstacle && current_state_ == NORMAL && dist_to_goal >= 0.2){
                bool center_block = (zone_center_ < zone_thresh);
                if(center_block){
                    if(zone_turn_dir_ == 0){
                        // pilih sisi paling lapang
                        zone_turn_dir_ = (zone_left_ >= zone_right_) ? +1 : -1;
                    }
                    bool all_block = (zone_left_  < zone_block_all &&
                                      zone_center_< zone_block_all &&
                                      zone_right_ < zone_block_all);
                    if(all_block){
                        // buntu total -> MUNDUR pelan + putar untuk keluar dari
                        // himpitan (mencegah FREEZE selamanya). Lidar 360 deg
                        // jadi belakang relatif aman. Setelah mundur, global
                        // planner (replan 1Hz) cari rute memutar.
                        local_vel_x = -0.10;
                        local_vel_y = 0.0;
                        angular_vel = zone_turn_dir_ * zone_turn_rate;
                    } else {
                        // ada celah samping -> pelankan & belok ke celah
                        local_vel_x *= 0.25;
                        local_vel_y *= 0.25;
                        angular_vel += zone_turn_dir_ * zone_turn_rate;
                    }
                    ROS_WARN_THROTTLE(1.0,
                        "ZONE-GATE: L=%.2f C=%.2f R=%.2f -> belok %s",
                        zone_left_, zone_center_, zone_right_,
                        (zone_turn_dir_ > 0 ? "KIRI" : "KANAN"));
                } else {
                    zone_turn_dir_ = 0; // depan lapang -> lepas commitment
                }
            }

            if(angular_vel > max_angular_vel) angular_vel = max_angular_vel;
            if(angular_vel < min_angular_vel) angular_vel = min_angular_vel;


            cmd_vel.linear.x = local_vel_x;
            cmd_vel.linear.y = local_vel_y;
            cmd_vel.angular.z = angular_vel;

            // ROS_INFO("Current waypoint: %d | Fatt: %.2f | Frep: %.2f | Force: %.2f", current_progress_id, att_mag, rep_mag, f_mag);
            // ROS_INFO("Force x: %.2f | Force y: %.2f", f_total_x, f_total_y);

            // ----- DEBUG TELEMETRY -----
            // std::string state_str = (current_state_ == NORMAL) ? "NORMAL" : "AVOID";
            
            // ROS_INFO("----------------------------------------");
            // ROS_INFO("[State]: %s | [Min Dist Obs]: %.2fm", state_str.c_str(), min_dist_obs);
            
            // if(current_state_ == AVOIDANCE) {
            //     ROS_INFO("[VP Target]: X: %.2f, Y: %.2f", target_x, target_y);
            // }

            // ROS_INFO("[Forces] F_att: (%.2f, %.2f) | F_rep: (%.2f, %.2f)", f_att_x, f_att_y, f_rep_x, f_rep_y);
            // ROS_INFO("[Output] YawDiff: %.2f rad | Vx: %.2f | Wz: %.2f", yaw_diff, local_vel_x, angular_vel);
            // ROS_INFO("----------------------------------------");

            return true;
        }

        bool isGoalReached(){

            // simple check so it doesn't compute before the global plan is set
            if(global_plan_.empty()) return false;
            
            // read the robot's current position
            geometry_msgs::PoseStamped global_pose;
            costmap_ros_->getRobotPose(global_pose);
            double robot_x = global_pose.pose.position.x;
            double robot_y = global_pose.pose.position.y;
            double robot_yaw = tf2::getYaw(global_pose.pose.orientation);

            // read the goal position
            double goal_x = global_plan_.back().pose.position.x;
            double goal_y = global_plan_.back().pose.position.y;
            double goal_yaw = tf2::getYaw(global_plan_.back().pose.orientation);

            double raw_yaw_diff = goal_yaw - robot_yaw;
            double yaw_diff = std::atan2(std::sin(raw_yaw_diff), std::cos(raw_yaw_diff));

            double dist_to_goal = std::sqrt(std::pow(goal_x - robot_x, 2) + std::pow(goal_y - robot_y, 2));

            if(dist_to_goal < 0.2 && std::abs(yaw_diff) < 0.26) {
                return true;
            }

            return false;
        }
        

};

}

PLUGINLIB_EXPORT_CLASS(APF::APFPlanner, nav_core::BaseLocalPlanner);