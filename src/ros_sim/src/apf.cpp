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
#include <sensor_msgs/LaserScan.h>   // [FUZZY] baca /thermal_blindspot_scan
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <mutex>
#include <limits>                    // [FUZZY] infinity untuk jarak blind-spot

#include <pluginlib/class_list_macros.h>

// [FUZZY] kontroler fuzzy Mamdani (port C++ dari desain scikit-fuzzy BAB 3)
#include "fuzzy_controller.h"

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

        // [FUZZY] flag rosparam: false -> kontrol crisp lama (baseline S1-S4),
        //         true -> pakai fuzzy Mamdani (S5-S8). Default false biar aman.
        bool use_fuzzy_ = false;



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

        // ====== [TRAVERSABILITY-AWARE] LiDAR clearance ======
        // Layer 2: nilai apakah robot MUAT lewat, dari geometri LiDAR (lebar
        // koridor metrik) DIKOREKSI thermal untuk blindspot vertikal LiDAR
        // (kucing rendah yang tak ke-scan). Hasilnya jadi input 'trav' fuzzy.
        ros::Subscriber scan_sub_;
        std::mutex      scan_mutex_;
        std::vector<float> scan_ranges_;
        double scan_angle_min_  = 0.0;
        double scan_angle_inc_  = 0.0;
        double scan_range_max_  = 10.0;
        bool   scan_ready_      = false;
        // parameter (bisa override via rosparam ~<name>/...)
        double robot_width_     = 0.55; // lebar fisik robot (m) ~0.52-0.62
        double clearance_full_  = 0.25; // clearance(m) saat trav dianggap 1.0 (LAYAK)
        double side_win_lo_     = 0.96; // ~55 deg  : batas bawah jendela samping
        double side_win_hi_     = 2.18; // ~125 deg : batas atas  jendela samping
        // state low-pass untuk redam noise thermal (anti zig-zag)
        double thermal_filt_    = 1.0;
        // hasil terakhir (buat logging skripsi)
        double last_corridor_m_ = 2.0;
        double last_clearance_  = 1.0;
        double last_trav_lidar_ = 1.0;

        // ====== [CAT KEEP-OUT] jaga jarak aman dari kucing (blindspot LiDAR) ======
        // Pakai /thermal_blindspot_scan (sensor metrik kucing). Robot dijaga agar
        // PUSAT-nya >= (radius_robot + batas_aman) dari PERMUKAAN kucing, jadi tepi
        // robot selalu punya 'batas_aman' ke tepi kucing. Arah hindar muncul
        // otomatis dari gaya tarik-goal + tolak-radial-kucing.
        ros::Subscriber blind_sub_;
        std::mutex      blind_mutex_;
        double blind_range_   = std::numeric_limits<double>::infinity();
        double blind_bearing_ = 0.0;
        ros::Time blind_stamp_;
        // posisi kucing terkunci di world (biar tetap saat robot menyusur lewat)
        double cat_world_x_   = 0.0;
        double cat_world_y_   = 0.0;
        ros::Time last_cat_time_;
        bool   cat_init_      = false;
        // parameter (rosparam ~<name>/...)
        double cat_safe_margin_ = 0.08; // batas aman tepi-robot ke tepi-kucing (m)
        double cat_radius_      = 0.18; // radius kucing (m)
        double cat_influence_   = 0.35; // zona pengaruh di luar batas aman (m)
        double cat_keep_gain_   = 3.0;  // kekuatan tolakan
        double cat_latch_time_  = 2.0;  // detik commit walau deteksi sesaat hilang
        double cat_fresh_time_  = 0.6;  // deteksi dianggap 'baru' bila < ini (detik)

        // Fallback within fallback SM variables
        enum apf_state {NORMAL, AVOIDANCE};

        apf_state current_state_ = NORMAL;

        double prev_ftotal_x = 0.0;
        double prev_ftotal_y = 0.0;

        int local_minima_counter_ = 0;
        int local_minima_threshold_ = 20; // 20 cycles of oscillation before switching to avoidance

        int avoidance_counter_ = 0;
        int avoidance_threshold_ = 80; // 8 second time to avoid
        int avoidance_side_ = 0;   // 0=belum pilih, +1=kiri, -1=kanan
      
        double v_target_x = 0.0;
        double v_target_y = 0.0;
        double corridor_shift_ = 0.0;  // (tidak dipakai lagi - keep-out menggantikan)

        // Wp tracking variables
        int current_progress_id = 0;

        double lookahead_dist = 0.7;
        double target_yaw = 0.0;

        bool is_safe_ = true;

        // APF tunable params (readable via rosparam, set di nav_sim.launch)
        double oscillation_threshold_ = -0.3;
        bool   thermal_rep_enable_    = true;
        double k_rep_normal_          = 0.7;
        double d0_normal_             = 0.7;
        // [S8-DINAMIS] gate deteksi obstacle utk repulsi LiDAR. DEFAULT = perilaku
        // lama (S7): 0.7 m / scan 1.0 m. Diperlebar HANYA utk S4/S8 via launch,
        // jadi S5-S7 byte-identik -> hasil lama tetap valid.
        double obs_detect_radius_     = 0.7;
        double obs_scan_radius_       = 1.0;
        // [S8 DINAMIS] Repulsi PREDIKTIF (velocity-aware). Default OFF -> S5-S7
        // byte-identik. Saat ON: obstacle yang MENDEKAT cepat (human nyebrang)
        // dapat dorongan + jangkauan lebih awal; dinding statis (laju mendekat
        // kecil) tidak terpengaruh.
        bool   dyn_predict_enable_    = false;
        double dyn_predict_tau_       = 0.5;   // horizon prediksi (s): perluas d0
        double dyn_predict_kgain_     = 1.0;   // boost kekuatan saat mendekat
        double dyn_predict_min_close_ = 0.25;  // ambang laju mendekat (m/s) baru aktif
        double prev_min_dist_obs_     = -1.0;  // state internal (jangan di-set launch)
        ros::Time prev_obs_time_      = ros::Time(0);
        // [S8 ANTI-FREEZE] cegah "freezing robot": kalau obstacle mendekat &
        // robot nyaris berhenti, MINGGIR (vacate) drpd diam ditabrak. OFF (0) =
        // perilaku lama. di-gate dyn_predict_enable_ -> S5-S7 tak terpengaruh.
        double dyn_closing_           = 0.0;   // laju mendekat terkini (m/s)
        double dyn_min_escape_speed_  = 0.0;   // m/s; >0 utk S8
        double dyn_freeze_speed_      = 0.08;  // di bawah ini = dianggap mematung
        // [S8 TEMPORAL YIELD] "mengantri": kalau obstacle dinamis menyeberang
        // DI DEPAN (antara robot & goal) sambil mendekat, robot MELAMBAT/menahan
        // di tempat sampai obstacle lewat, baru menerobos. Anti-deadlock via
        // batas waktu. Default OFF -> S5-S7 tak terpengaruh.
        bool   yield_enable_          = false;
        double yield_close_thresh_    = 0.30;  // m/s; obstacle harus mendekat
        double yield_cone_deg_        = 70.0;  // obstacle dianggap "di depan" dlm cone ini
        double yield_trigger_dist_    = 1.40;  // mulai antri saat jarak < ini
        double yield_release_dist_    = 1.70;  // lepas saat jarak > ini (histeresis)
        double yield_max_time_        = 5.0;   // batas antri (detik) -> anti-deadlock
        double yield_commit_time_     = 2.0;   // setelah lepas, jangan antri lagi (terobos)
        double yield_slow_factor_     = 0.12;  // skala kecepatan saat antri
        bool   yielding_              = false;
        ros::Time yield_start_time_   = ros::Time(0);
        ros::Time yield_release_time_ = ros::Time(0);

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

        // [TRAVERSABILITY-AWARE] simpan scan LiDAR mentah
        void scanCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
        {
            std::lock_guard<std::mutex> lock(scan_mutex_);
            scan_ranges_     = msg->ranges;
            scan_angle_min_  = msg->angle_min;
            scan_angle_inc_  = msg->angle_increment;
            scan_range_max_  = msg->range_max;
            scan_ready_      = true;
        }

        // [CAT KEEP-OUT] ambil titik kucing TERDEKAT dari blindspot scan
        void blindCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
        {
            double best = std::numeric_limits<double>::infinity();
            double best_ang = 0.0;
            for(size_t i = 0; i < msg->ranges.size(); ++i){
                double r = msg->ranges[i];
                if(!std::isfinite(r) || r <= 0.01) continue;
                if(r < best){
                    best = r;
                    best_ang = msg->angle_min + i * msg->angle_increment;
                }
            }
            std::lock_guard<std::mutex> lock(blind_mutex_);
            blind_range_   = best;          // inf kalau tak ada deteksi
            blind_bearing_ = best_ang;
            blind_stamp_   = ros::Time::now();
        }

        // ====== [TRAVERSABILITY-AWARE] LAYER 2: penilaian kelayakan lintasan ======
        // Mengukur lebar koridor metrik dari LiDAR (jarak ke dinding kiri+kanan di
        // jendela menyamping), lalu clearance = lebar_koridor - lebar_robot.
        // clearance besar -> robot MUAT (trav~1) ; clearance<=0 -> tak muat (trav~0).
        // Dikoreksi thermal: kucing rendah (tak ke-scan LiDAR) MENURUNKAN trav.
        // Mengembalikan trav efektif [0..1] untuk input fuzzy.
        double computeTraversability(double thermal_trav)
        {
            // --- 1. lebar koridor dari LiDAR (kiri + kanan, menyamping) ---
            double left_clear  = scan_range_max_;
            double right_clear = scan_range_max_;
            {
                std::lock_guard<std::mutex> lock(scan_mutex_);
                if(scan_ready_ && !scan_ranges_.empty()){
                    for(size_t i = 0; i < scan_ranges_.size(); ++i){
                        double r = scan_ranges_[i];
                        if(!std::isfinite(r) || r <= 0.01) continue;
                        double a = scan_angle_min_ + i * scan_angle_inc_; // 0=depan,+kiri
                        double aa = std::fabs(a);
                        // jendela menyamping (~55..125 deg) KIRI (+) & KANAN (-)
                        if(aa < side_win_lo_ || aa > side_win_hi_) continue;
                        // komponen lateral murni = r*sin|a| (jarak tegak-lurus ke dinding)
                        double lat = r * std::sin(aa);
                        if(a > 0.0){ if(lat < left_clear)  left_clear  = lat; }
                        else       { if(lat < right_clear) right_clear = lat; }
                    }
                }
            }
            double corridor_m = left_clear + right_clear;          // lebar koridor (m)
            double clearance  = corridor_m - robot_width_;          // sisa ruang (m)
            double trav_lidar = clearance / clearance_full_;        // map ke 0..1
            if(trav_lidar < 0.0) trav_lidar = 0.0;
            if(trav_lidar > 1.0) trav_lidar = 1.0;

            // --- 2. koreksi thermal (blindspot LiDAR) : low-pass ASIMETRIS ---
            // thermal_trav turun saat kucing rendah masuk ROI -> menurunkan kelayakan
            // walau LiDAR bilang koridor lapang. Asimetris: TURUN cepat (begitu kucing
            // muncul, demi keamanan), NAIK lambat (anti-kedip piksel thermal).
            double a_smooth = (thermal_trav < thermal_filt_) ? 0.50 : 0.20;
            thermal_filt_ = (1.0 - a_smooth) * thermal_filt_ + a_smooth * thermal_trav;
            double trav_eff = std::min(trav_lidar, thermal_filt_);

            // simpan utk logging
            last_corridor_m_ = corridor_m;
            last_clearance_  = clearance;
            last_trav_lidar_ = trav_lidar;
            return trav_eff;
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
                // [TRAVERSABILITY-AWARE] LiDAR untuk ukur clearance koridor
                scan_sub_ = nh.subscribe(
                    "/scan",
                    1,
                    &APFPlanner::scanCallback,
                    this
                );
                // [CAT KEEP-OUT] sensor metrik kucing (blindspot)
                blind_sub_ = nh.subscribe(
                    "/thermal_blindspot_scan",
                    1,
                    &APFPlanner::blindCallback,
                    this
                );
                last_cat_time_ = ros::Time(0);
                ROS_INFO("APF Planner Plugin initialized (THERMAL TRAVERSABILITY FUSION).");

                // [FUZZY] baca flag use_fuzzy. Cek di private ns planner dulu
                // (~/<name>/use_fuzzy), lalu fallback ke global (/use_fuzzy).
                if(!private_nh.getParam("use_fuzzy", use_fuzzy_)){
                    nh.param("use_fuzzy", use_fuzzy_, false);
                }
                // [TRAVERSABILITY-AWARE] parameter clearance (boleh override rosparam)
                private_nh.param("robot_width",    robot_width_,    robot_width_);
                private_nh.param("clearance_full", clearance_full_, clearance_full_);
                private_nh.param("cat_safe_margin", cat_safe_margin_, cat_safe_margin_);
                private_nh.param("cat_radius",      cat_radius_,      cat_radius_);
                private_nh.param("cat_influence",   cat_influence_,   cat_influence_);
                private_nh.param("cat_keep_gain",   cat_keep_gain_,   cat_keep_gain_);
                private_nh.param("cat_latch_time",  cat_latch_time_,  cat_latch_time_);
                // APF gain overrides (nav_sim.launch -> APFPlanner/...)
                private_nh.param("k_att",                k_att,                k_att);
                private_nh.param("k_rep_normal",         k_rep_normal_,        k_rep_normal_);
                private_nh.param("d0_normal",            d0_normal_,           d0_normal_);
                private_nh.param("oscillation_threshold",oscillation_threshold_,oscillation_threshold_);
                private_nh.param("thermal_rep_enable",   thermal_rep_enable_,  thermal_rep_enable_);
                private_nh.param("obs_detect_radius",     obs_detect_radius_,   obs_detect_radius_);
                private_nh.param("obs_scan_radius",       obs_scan_radius_,     obs_scan_radius_);
                private_nh.param("dyn_predict_enable",    dyn_predict_enable_,  dyn_predict_enable_);
                private_nh.param("dyn_predict_tau",       dyn_predict_tau_,     dyn_predict_tau_);
                private_nh.param("dyn_predict_kgain",     dyn_predict_kgain_,   dyn_predict_kgain_);
                private_nh.param("dyn_predict_min_close", dyn_predict_min_close_, dyn_predict_min_close_);
                private_nh.param("dyn_min_escape_speed",  dyn_min_escape_speed_, dyn_min_escape_speed_);
                private_nh.param("dyn_freeze_speed",      dyn_freeze_speed_,     dyn_freeze_speed_);
                private_nh.param("yield_enable",        yield_enable_,        yield_enable_);
                private_nh.param("yield_close_thresh",  yield_close_thresh_,  yield_close_thresh_);
                private_nh.param("yield_cone_deg",      yield_cone_deg_,      yield_cone_deg_);
                private_nh.param("yield_trigger_dist",  yield_trigger_dist_,  yield_trigger_dist_);
                private_nh.param("yield_release_dist",  yield_release_dist_,  yield_release_dist_);
                private_nh.param("yield_max_time",      yield_max_time_,      yield_max_time_);
                private_nh.param("yield_commit_time",   yield_commit_time_,   yield_commit_time_);
                private_nh.param("yield_slow_factor",   yield_slow_factor_,   yield_slow_factor_);
                ROS_WARN("[APF] k_att=%.2f k_rep_n=%.2f d0_n=%.2f osc_thr=%.2f thermal_rep=%s",
                         k_att, k_rep_normal_, d0_normal_, oscillation_threshold_,
                         thermal_rep_enable_ ? "ON" : "OFF");
                ROS_WARN("[APF] obs_detect_radius=%.2f obs_scan_radius=%.2f",
                         obs_detect_radius_, obs_scan_radius_);
                ROS_WARN("[TRAV] robot_width=%.2f m, clearance_full=%.2f m",
                         robot_width_, clearance_full_);
                ROS_WARN("[FUZZY] use_fuzzy = %s", use_fuzzy_ ? "TRUE (Mamdani aktif, S5-S8)"
                                                              : "FALSE (kontrol crisp, S1-S4)");
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

            double scan_radius = obs_scan_radius_;   // [S8] dulu hardcoded 1.0
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

                            if(dist_to_obs < obs_detect_radius_){   // [S8] dulu hardcoded 0.7
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

            // [FUZZY] Compute speed scale and repulsive gain
            double speed_scale = 1.0;
            double rep_gain = 1.0;
            if(use_fuzzy_){
                // [TRAVERSABILITY-AWARE] trav = kelayakan lewat (LiDAR clearance
                // dikoreksi thermal blindspot), BUKAN lagi skor piksel mentah.
                double trav_eff = computeTraversability(traversability_score_);
                fuzzy::fuzzyCompute(min_dist_obs, trav_eff, speed_scale, rep_gain);
                ROS_INFO_THROTTLE(1.0,
                    "[FUZZY] jarak=%.2f trav=%.2f (koridor=%.2fm clr=%.2fm tLid=%.2f tThr=%.2f) -> spd=%.3f gain=%.3f",
                    (min_dist_obs > 2.0 ? 2.0 : min_dist_obs),
                    trav_eff, last_corridor_m_, last_clearance_,
                    last_trav_lidar_, thermal_filt_, speed_scale, rep_gain);
            }

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

            // Attractive force calculation (dihitung ULANG setelah safety-corridor
            // shift di bawah, kalau kucing terdeteksi)
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
            // [SAFETY-CORRIDOR] di-hoist ke sini biar kepakai di blok shift bawah
            double hot_count     = 0.0;
            double obstacle_side = 0.0;
            double closeness     = 0.0;

            if (thermal_rep_enable_) {
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
                    hot_count = 0.0;

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
                        obstacle_side = (cx - (w / 2.0)) / (w / 2.0);
                        closeness = std::max(0.0, (cy - (h / 3.0)) / (h * 2.0 / 3.0));
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

            // ========================================
            // [CAT KEEP-OUT] jaga jarak aman dari kucing
            // ========================================
            // Pakai posisi kucing dari /thermal_blindspot_scan (metrik). Saat
            // terdeteksi, kunci posisinya di world. Selama latch aktif, tolak
            // robot supaya PUSAT-nya >= D_safe dari permukaan kucing, dengan
            // D_safe = radius_robot + batas_aman. Tepi robot jadi selalu punya
            // 'batas_aman' ke tepi kucing. Arah hindar = radial menjauhi kucing
            // (digabung gaya tarik-goal -> otomatis lewat sisi yang menuju goal).
            double f_cat_x = 0.0;
            double f_cat_y = 0.0;
            {
                double brange, bbear; ros::Time bstamp;
                {
                    std::lock_guard<std::mutex> lock(blind_mutex_);
                    brange = blind_range_; bbear = blind_bearing_; bstamp = blind_stamp_;
                }
                ros::Time now_t = ros::Time::now();
                // deteksi BARU & valid -> kunci/refresh PUSAT kucing di world
                if(std::isfinite(brange) &&
                   (now_t - bstamp).toSec() < cat_fresh_time_){
                    double dd_to_center = brange + cat_radius_;   // ke pusat kucing
                    cat_world_x_ = robot_x + dd_to_center * std::cos(robot_yaw + bbear);
                    cat_world_y_ = robot_y + dd_to_center * std::sin(robot_yaw + bbear);
                    last_cat_time_ = now_t;
                    cat_init_ = true;
                }
                // LATCH BERBASIS-JARAK: sekali terkunci, keep-out tetap aktif
                // sampai robot JELAS lewat kucing (d_cat besar lagi). Ini bikin
                // SATU deteksi (sebelum mepet) cukup menjaga margin sepanjang lewat,
                // walau kucing keluar FOV kamera 10+ detik.
                if(cat_init_ && use_fuzzy_){
                    double dx = robot_x - cat_world_x_;
                    double dy = robot_y - cat_world_y_;
                    double d_cat = std::hypot(dx, dy);
                    // pusat-robot harus >= radius_kucing + radius_robot + margin dari pusat kucing
                    double D_safe = cat_radius_ + robot_width_ * 0.5 + cat_safe_margin_;
                    double d0c    = D_safe + cat_influence_;
                    if(d_cat < d0c){
                        double dd = d_cat - D_safe;       // >0 di luar batas, <0 melanggar
                        double f;
                        if(dd > 0.0){
                            double t = (d0c - d_cat) / cat_influence_; // 0..1
                            if(t > 1.0) t = 1.0;
                            f = cat_keep_gain_ * t * t;     // ramp halus
                        }else{
                            f = cat_keep_gain_ * (1.0 + (-dd) * 12.0); // sudah masuk -> dorong keras
                        }
                        double inv = 1.0 / std::max(d_cat, 1e-3);
                        f_cat_x = f * dx * inv;
                        f_cat_y = f * dy * inv;
                        ROS_INFO_THROTTLE(1.0,
                            "[CAT KEEP-OUT] d=%.2fm (D_safe=%.2f) f=%.2f -> (%.2f,%.2f)",
                            d_cat, D_safe, f, f_cat_x, f_cat_y);
                    }
                    // lupakan kunci hanya kalau robot SUDAH jelas lewat & tak ada
                    // deteksi baru beberapa saat (siap untuk obstacle berikutnya)
                    if(d_cat > d0c + 0.5 &&
                       (now_t - last_cat_time_).toSec() > 1.0){
                        cat_init_ = false;
                    }
                }
            }

            double f_total_x = 0.0;
            double f_total_y = 0.0;

            // Planner state management
            if(current_state_ == NORMAL){

                // reset tuning variables
                // k_rep = 1.0; 
                // d0 = 0.8; 
                k_rep = k_rep_normal_;
                d0 = d0_normal_;

                // Repulsive force calculation

                if(found_any_obs == true){
                    if(min_dist_obs < 0.15) min_dist_obs = 0.15;

                    // [S8 DINAMIS] Repulsi prediktif: kalau obstacle terdekat
                    // MENDEKAT cepat (laju mendekat > ambang), perluas zona
                    // pengaruh (d0) & kuatkan dorongan SEBELUM kontak. Dinding
                    // statis -> laju mendekat ~0 -> tak terpengaruh.
                    double rep_extra_gain = 1.0;
                    if(dyn_predict_enable_){
                        ros::Time tnow = ros::Time::now();
                        dyn_closing_ = 0.0;
                        if(prev_min_dist_obs_ > 0.0 && !prev_obs_time_.isZero()){
                            double dt = (tnow - prev_obs_time_).toSec();
                            if(dt > 1e-3 && dt < 0.5){
                                double closing = (prev_min_dist_obs_ - min_dist_obs) / dt; // >0 = mendekat
                                dyn_closing_ = closing;
                                if(closing > dyn_predict_min_close_){
                                    double c = std::min(closing, 1.5);
                                    d0 += dyn_predict_tau_ * c;            // belok/menjauh lebih awal
                                    rep_extra_gain = 1.0 + dyn_predict_kgain_ * c; // dorong lebih kuat
                                    ROS_WARN_THROTTLE(0.5,
                                        "[DYN-PREDICT] closing=%.2f m/s -> d0=%.2f gain x%.2f",
                                        closing, d0, rep_extra_gain);
                                }
                            }
                        }
                        prev_min_dist_obs_ = min_dist_obs;
                        prev_obs_time_     = tnow;
                    }

                    double base_gain = use_fuzzy_
                        ? rep_gain
                        : (1.0 + (1.0 - traversability_score_));

                    ROS_INFO_THROTTLE(
                        1.0,
                        "APF NORMAL: gain=%.3f (trav=%.2f)%s",
                        base_gain,
                        traversability_score_,
                        use_fuzzy_ ? " [fuzzy]" : ""
                    );

                    double f_rep =
                        base_gain *
                        k_rep *
                        ((1.0 / min_dist_obs) - (1.0 / d0))
                        * std::pow((1.0 / min_dist_obs), 2);

                    f_rep *= rep_extra_gain;   // [S8 DINAMIS] boost prediktif

                    // X and Y components of repulsive force
                    f_rep_x = f_rep * ((robot_x - closest_obs_x) / min_dist_obs);
                    f_rep_y = f_rep * ((robot_y - closest_obs_y) / min_dist_obs);
                }
                else if(dyn_predict_enable_){
                    prev_min_dist_obs_ = -1.0;   // reset: tak ada obstacle terdekat
                }

                // Total force calculation
                f_total_x = f_att_x + f_rep_x + thermal_rep_x + f_cat_x;
                f_total_y = f_att_y + f_rep_y + thermal_rep_y + f_cat_y;

                double prev_mag = std::hypot(prev_ftotal_x, prev_ftotal_y);
                double curr_mag = std::hypot(f_total_x, f_total_y);

                if(curr_mag > 0.01 && prev_mag > 0.01){

                    double num = (f_total_x * prev_ftotal_x) + (f_total_y * prev_ftotal_y);
                    double denum = curr_mag * prev_mag;

                    double cos_theta = num / denum; // Normalized value based on the research paper

                    double oscillation_threshold = oscillation_threshold_;

                    if(cos_theta < oscillation_threshold){
                        if((ros::Time::now() - last_avoidance_time_).toSec() > 3.0){
                            ROS_WARN("APF : Oscillation detected. Switching to AVOIDANCE mode.");
                            current_state_ = AVOIDANCE;
                            local_minima_counter_ = 0;

                            /* commit ke sisi yang lebih lapang */
                            if(zone_right_ > zone_left_)
                                avoidance_side_ = -1;     // kanan
                            else
                                avoidance_side_ = +1;     // kiri
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
                    
                    // COMMIT: hanya cari satu sisi (hemisphere)
                    if(avoidance_side_ != 0){
                        if(avoidance_side_ > 0){  // kiri: -pi/2 .. pi/2
                            if(angle > M_PI / 2.0 && angle < 3.0 * M_PI / 2.0)
                                continue;  // skip sisi kanan
                        } else {                  // kanan: pi/2 .. 3pi/2 (wrapping)
                            if(angle < M_PI / 2.0 || angle > 3.0 * M_PI / 2.0)
                                continue;  // skip sisi kiri
                        }
                    }
                    
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
                    avoidance_side_ = 0;

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

                    double base_gain = use_fuzzy_
                        ? rep_gain
                        : (1.0 + (1.0 - traversability_score_));

                    ROS_INFO_THROTTLE(
                        1.0,
                        "APF AVOIDANCE: gain=%.3f (trav=%.2f)%s",
                        base_gain,
                        traversability_score_,
                        use_fuzzy_ ? " [fuzzy]" : ""
                    );

                    double f_rep =
                        base_gain *
                        k_rep *
                        ((1.0 / min_dist_obs) - (1.0 / d0))
                        * std::pow((1.0 / min_dist_obs), 2);
                                            
                    // X and Y components of repulsive force
                    f_rep_x = f_rep * ((robot_x - closest_obs_x) / min_dist_obs);
                    f_rep_y = f_rep * ((robot_y - closest_obs_y) / min_dist_obs);
                }

                // Total force calculation
                f_total_x = f_att_x + f_rep_x + thermal_rep_x + f_cat_x;
                f_total_y = f_att_y + f_rep_y + thermal_rep_y + f_cat_y;
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

            // [FUZZY] Apply speed scale to velocity
            local_vel_x *= speed_scale;
            local_vel_y *= speed_scale;

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
            // ============================================================
            bool   zone_enable     = true;
            double zone_thresh     = 0.55;  // center < ini -> dianggap terhalang
            double zone_block_all  = 0.35;  // semua zona < ini -> buntu total
            double zone_turn_rate  = 0.5;   // rad/s tambahan saat menghindar
            bool front_has_obstacle = (traversability_score_ < 0.6);
            if(zone_enable && front_has_obstacle && current_state_ == NORMAL && dist_to_goal >= 0.2){
                bool center_block = (zone_center_ < zone_thresh);
                if(center_block){
                    if(zone_turn_dir_ == 0){
                        zone_turn_dir_ = (zone_left_ >= zone_right_) ? +1 : -1;
                    }
                    bool all_block = (zone_left_  < zone_block_all &&
                                      zone_center_< zone_block_all &&
                                      zone_right_ < zone_block_all);
                    if(all_block){
                        local_vel_x = -0.10;
                        local_vel_y = 0.0;
                        angular_vel = zone_turn_dir_ * zone_turn_rate;
                    } else {
                        local_vel_x *= 0.25;
                        local_vel_y *= 0.25;
                        angular_vel += zone_turn_dir_ * zone_turn_rate;
                    }
                    ROS_WARN_THROTTLE(1.0,
                        "ZONE-GATE: L=%.2f C=%.2f R=%.2f -> belok %s",
                        zone_left_, zone_center_, zone_right_,
                        (zone_turn_dir_ > 0 ? "KIRI" : "KANAN"));
                } else {
                    zone_turn_dir_ = 0;
                }
            }

            // ============================================================
            // [S8 TEMPORAL YIELD] Mengantri di mulut bottleneck: kalau obstacle
            // dinamis menyeberang DI DEPAN (antara robot & goal) sambil mendekat,
            // robot melambat/menahan sampai obstacle lewat, baru menerobos.
            // - "di depan": bearing obstacle dlm cone thd arah goal.
            // - hanya MOVING obstacle yg memicu (butuh closing>thresh; dinding
            //   statis -> saat robot melambat closing->0 -> lepas sendiri).
            // - anti-deadlock: batas waktu antri + commit window (terobos stlh lepas).
            // Default OFF (yield_enable_=false) -> S5-S7 byte-identik.
            // ============================================================
            if(yield_enable_ && current_state_ == NORMAL && dist_to_goal > 0.6 && found_any_obs){
                ros::Time tnow = ros::Time::now();
                double obs_bearing = std::atan2(closest_obs_y - robot_y, closest_obs_x - robot_x);
                double goal_dir    = std::atan2(goal_y - robot_y, goal_x - robot_x);
                double rel = std::atan2(std::sin(obs_bearing - goal_dir), std::cos(obs_bearing - goal_dir));
                bool ahead   = std::fabs(rel) < (yield_cone_deg_ * M_PI / 180.0);
                bool closing = dyn_closing_ > yield_close_thresh_;
                bool in_commit = (!yield_release_time_.isZero() &&
                                  (tnow - yield_release_time_).toSec() < yield_commit_time_);

                if(!yielding_){
                    if(!in_commit && ahead && closing && min_dist_obs < yield_trigger_dist_){
                        yielding_ = true;
                        yield_start_time_ = tnow;
                    }
                } else {
                    bool crossed = (!ahead) || (min_dist_obs > yield_release_dist_) || (dyn_closing_ < 0.0);
                    bool timeout = (tnow - yield_start_time_).toSec() > yield_max_time_;
                    if(crossed || timeout){
                        yielding_ = false;
                        yield_release_time_ = tnow;
                    }
                }

                if(yielding_){
                    // melambat di tempat (tetap merayap kecil -> tak kena oscillation-timeout)
                    local_vel_x *= yield_slow_factor_;
                    local_vel_y *= yield_slow_factor_;
                    angular_vel *= 0.5;
                    ROS_WARN_THROTTLE(0.5,
                        "[YIELD] antri: obstacle nyebrang depan (d=%.2f closing=%.2f) -> tahan",
                        min_dist_obs, dyn_closing_);
                }
            }

            // ============================================================
            // [S8 ANTI-FREEZE] LAST RESORT: kalau yield gagal & obstacle SANGAT
            // dekat (mematung) -> minggir, jangan diam ditabrak.
            // ============================================================
            if(dyn_predict_enable_ && dyn_min_escape_speed_ > 0.0 &&
               current_state_ == NORMAL && found_any_obs &&
               dyn_closing_ > dyn_predict_min_close_ &&
               min_dist_obs < 0.45 &&
               dist_to_goal > 0.5){
                double cur_sp = std::hypot(local_vel_x, local_vel_y);
                if(cur_sp < dyn_freeze_speed_){
                    double awx = robot_x - closest_obs_x;
                    double awy = robot_y - closest_obs_y;
                    double an  = std::hypot(awx, awy);
                    if(an > 1e-3){
                        awx/=an; awy/=an;
                        double gdx = goal_x - robot_x, gdy = goal_y - robot_y;
                        double gn  = std::hypot(gdx, gdy);
                        if(gn > 1e-3){ gdx/=gn; gdy/=gn; }
                        double mx = 0.65*awx + 0.35*gdx;   // dominan menjauhi obstacle
                        double my = 0.65*awy + 0.35*gdy;
                        double mn = std::hypot(mx, my);
                        if(mn < 1e-3){ mx = awx; my = awy; mn = 1.0; }
                        mx/=mn; my/=mn;
                        // world -> body frame
                        local_vel_x = dyn_min_escape_speed_ * ( mx*std::cos(robot_yaw) + my*std::sin(robot_yaw));
                        local_vel_y = dyn_min_escape_speed_ * (-mx*std::sin(robot_yaw) + my*std::cos(robot_yaw));
                        ROS_WARN_THROTTLE(0.3,
                            "[ANTI-FREEZE] mematung (v=%.2f) -> minggir @%.2f m/s",
                            cur_sp, dyn_min_escape_speed_);
                    }
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