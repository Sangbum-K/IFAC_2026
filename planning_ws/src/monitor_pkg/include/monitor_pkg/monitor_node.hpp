// 주행 상태 모니터 (ae_hyu_monitor 이식, 팀 인터페이스에 맞게 토픽 파라미터화).
// /pf/pose/odom(실차)|/ego_racecar/odom(sim) + /car_state/frenet/odom 기반으로
// 랩타임/CTE/속도/조향/장애물 요약을 /monitor/* (OverlayText, Float32)로 발행.
// RViz rviz_2d_overlay_plugins로 표시 — rviz/monitor.rviz 참고.
#ifndef MONITOR_PKG__MONITOR_NODE_HPP_
#define MONITOR_PKG__MONITOR_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <ackermann_msgs/msg/ackermann_drive_stamped.hpp>
#include <rviz_2d_overlay_msgs/msg/overlay_text.hpp>
#include <std_msgs/msg/float32.hpp>
#include <ae_hyu_msgs/msg/wpnt_array.hpp>
#include <ae_hyu_msgs/msg/obstacle_array.hpp>
#include <mutex>
#include <vector>
#include <limits>

class MonitorNode : public rclcpp::Node {
    public:
        explicit MonitorNode(const std::string &node_name, const rclcpp::NodeOptions &options = rclcpp::NodeOptions());
        virtual ~MonitorNode();

        void Run();

    private:
        // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - //
        // Function

        // Callback function
        inline void CallbackOdom(const nav_msgs::msg::Odometry::SharedPtr msg) {
            std::lock_guard<std::mutex> lock(mutex_odom_);
            i_current_odom_ = *msg;
            b_is_odom_ = true;
        }
        inline void CallbackFrenetOdom(const nav_msgs::msg::Odometry::SharedPtr msg) {
            std::lock_guard<std::mutex> lock(mutex_frenet_odom_);
            i_current_frenet_odom_ = *msg;
            b_is_frenet_odom_ = true;
        }
        inline void CallbackGlobalWaypoints(const ae_hyu_msgs::msg::WpntArray::SharedPtr msg) {
            if (!msg->wpnts.empty() && !b_is_max_s_) {
                max_s_ = msg->wpnts.back().s_m;  // Get the last s value as max_s
                b_is_max_s_ = true;
            }
        }
        inline void CallbackObstacles(const ae_hyu_msgs::msg::ObstacleArray::SharedPtr msg) {
            std::lock_guard<std::mutex> lock(mutex_obstacles_);
            i_current_obstacles_ = *msg;
            b_is_obstacles_ = true;
        }
        inline void CallbackSteeringInput(const ackermann_msgs::msg::AckermannDriveStamped::SharedPtr msg) {
            std::lock_guard<std::mutex> lock(mutex_steering_input_);
            i_current_steering_input_ = *msg;
            b_is_steering_input_ = true;
        }

        // Monitor function
        void PublishSpeedInfo(const nav_msgs::msg::Odometry& odom);
        void PublishSteeringInput();
        void UpdateLapInfo(const nav_msgs::msg::Odometry& frenet_odom);
        void PublishLapInfo();
        void PublishMeanLapTime();
        void PublishFastestLapTime();
        void UpdateCTE(const nav_msgs::msg::Odometry& frenet_odom);
        void PublishMeanCTE();
        void PublishMaxCTE();
        void PublishCurrentCTE();
        void PublishObstacleInfo();

        // Helper function for time formatting
        std::string format_time_3dp(double time_val);

        // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - //
        // Variable

        // Subscribers
        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr                    s_odom_;
        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr                    s_frenet_odom_;
        rclcpp::Subscription<ae_hyu_msgs::msg::WpntArray>::SharedPtr                s_global_waypoints_;
        rclcpp::Subscription<ae_hyu_msgs::msg::ObstacleArray>::SharedPtr            s_obstacles_;
        rclcpp::Subscription<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr s_steering_input_;

        // Input
        nav_msgs::msg::Odometry                       i_current_odom_;
        nav_msgs::msg::Odometry                       i_current_frenet_odom_;
        ae_hyu_msgs::msg::ObstacleArray               i_current_obstacles_;
        ackermann_msgs::msg::AckermannDriveStamped    i_current_steering_input_;

        // Mutex
        std::mutex mutex_odom_;
        std::mutex mutex_frenet_odom_;
        std::mutex mutex_obstacles_;
        std::mutex mutex_steering_input_;

        // Publishers
        rclcpp::Publisher<rviz_2d_overlay_msgs::msg::OverlayText>::SharedPtr  p_lap_info_;
        rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr                  p_current_speed_info_;
        rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr                  p_steering_input_;
        rclcpp::Publisher<rviz_2d_overlay_msgs::msg::OverlayText>::SharedPtr  p_mean_lap_time_;
        rclcpp::Publisher<rviz_2d_overlay_msgs::msg::OverlayText>::SharedPtr  p_fastest_lap_time_;
        rclcpp::Publisher<rviz_2d_overlay_msgs::msg::OverlayText>::SharedPtr  p_mean_cte_;
        rclcpp::Publisher<rviz_2d_overlay_msgs::msg::OverlayText>::SharedPtr  p_max_cte_;
        rclcpp::Publisher<rviz_2d_overlay_msgs::msg::OverlayText>::SharedPtr  p_current_cte_;
        rclcpp::Publisher<rviz_2d_overlay_msgs::msg::OverlayText>::SharedPtr  p_obstacle_info_;

        // Timer
        rclcpp::TimerBase::SharedPtr t_run_node_;

        // Flag
        bool b_is_odom_ = false;
        bool b_is_frenet_odom_ = false;
        bool b_is_max_s_ = false;
        bool b_is_obstacles_ = false;
        bool b_is_steering_input_ = false;

        // Global Variables
        double loop_rate_hz_;
        double max_s_ = 0.0;  // Maximum s value (track length)
        double start_zone_threshold_;  // Start zone threshold parameter
        int text_size_;  // Text size for overlay

        // Lap tracking variables
        int lap_count_ = 0;
        double previous_s_ = 0.0;
        bool is_first_measurement_ = true;
        bool is_race_started_ = false;  // Track if we started from proper position

        // Lap time tracking variables
        rclcpp::Time lap_start_time_;
        std::vector<double> lap_times_;  // Store completed lap times
        double fastest_lap_time_ = std::numeric_limits<double>::max();
        double mean_lap_time_ = 0.0;     // Mean lap time for recent 5 laps
        double previous_lap_time_ = 0.0;  // Store previous completed lap time

        // CTE tracking variables
        std::vector<double> cte_values_;  // Store CTE values for current lap
        double max_cte_ = 0.0;           // Maximum CTE across all laps
        double current_lap_cte_sum_ = 0.0;
        int current_lap_cte_count_ = 0;
        double mean_cte_ = 0.0;          // Mean CTE for current lap
        double current_cte_ = 0.0;       // Current CTE value
        double total_cte_sum_ = 0.0;     // Sum for 10-lap mean
        int total_cte_count_ = 0;        // Count for 10-lap mean
        double ten_lap_mean_cte_ = 0.0;  // Mean CTE over last 10 laps
};

#endif  // MONITOR_PKG__MONITOR_NODE_HPP_
