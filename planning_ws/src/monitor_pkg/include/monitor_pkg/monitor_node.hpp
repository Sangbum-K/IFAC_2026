// 주행 상태 모니터 (ae_hyu_monitor 이식, 팀 인터페이스에 맞게 토픽 파라미터화).
// /pf/pose/odom(실차)|/ego_racecar/odom(sim) + /car_state/frenet/odom 기반으로
// 랩타임/CTE/속도/조향을 /monitor/* (OverlayText, Float32)로 발행.
// 현재 위치/자세는 텍스트가 아니라 RViz Odometry 화살표로 본다 (monitor.rviz).
//
// 추가로 실제 주행 궤적(trail)을 속도 컬러맵 LINE_STRIP으로,
// 레이스라인 대비 이탈량을 사다리(LINE_LIST) 마커로 발행하고,
// s 구간별 속도/CTE 통계를 랩마다 CSV로 남긴다.
// RViz rviz_2d_overlay_plugins로 표시 — rviz/monitor.rviz 참고.
#ifndef MONITOR_PKG__MONITOR_NODE_HPP_
#define MONITOR_PKG__MONITOR_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <ackermann_msgs/msg/ackermann_drive_stamped.hpp>
#include <rviz_2d_overlay_msgs/msg/overlay_text.hpp>
#include <std_msgs/msg/float32.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <f1tenth_planning_msgs/msg/wpnt_array.hpp>
#include <deque>
#include <mutex>
#include <string>
#include <vector>
#include <limits>

class MonitorNode : public rclcpp::Node {
    public:
        explicit MonitorNode(const std::string &node_name, const rclcpp::NodeOptions &options = rclcpp::NodeOptions());
        virtual ~MonitorNode();

        void Run();

    private:
        // 주행 궤적 한 점. 그릴 때(속도/CTE 컬러맵)와 이탈량 사다리에 함께 쓴다.
        struct TrailPoint {
            double x = 0.0;
            double y = 0.0;
            double v = 0.0;        // 실제 속도 [m/s]
            double v_plan = 0.0;   // 같은 s의 레이스라인 목표 속도 [m/s]
            double s = 0.0;
            double d = 0.0;        // 부호 있는 횡방향 편차 [m]
            double ref_x = 0.0;    // 같은 s의 레이스라인 위치 (이탈량 사다리용)
            double ref_y = 0.0;
            bool has_ref = false;
        };

        // s 구간별 통계 (구간 길이 = segment_length_m). 랩 완료 시 CSV로 flush.
        struct SegmentStats {
            int count = 0;
            double sum_v = 0.0;
            double max_v = 0.0;
            double min_v = std::numeric_limits<double>::max();
            double sum_v_plan = 0.0;
            double sum_abs_d = 0.0;
            double max_abs_d = 0.0;
            double first_t = -1.0;  // 구간 최초/최종 관측 시각 [s] — 구간 소요시간 추정
            double last_t = -1.0;
        };

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
        void CallbackGlobalWaypoints(const f1tenth_planning_msgs::msg::WpntArray::SharedPtr msg);
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

        // Trail / segment function
        void UpdateTrail(const nav_msgs::msg::Odometry& odom, const nav_msgs::msg::Odometry& frenet_odom);
        void UpdateSegments(double s, double v, double v_plan, double d, double now_sec);
        void PublishTrailMarkers();
        void PublishDeviationMarkers();
        void PublishSegmentInfo();
        void FinalizeLap(int lap_number, double lap_time);
        void WriteLapSegmentCsv(int lap_number, double lap_time);
        void WriteLapSummaryCsv(int lap_number, double lap_time);
        // 레이스라인(글로벌 웨이포인트)을 s로 보간 — 목표속도/기준 위치를 얻는다.
        bool LookupRaceline(double s, double* ref_x, double* ref_y, double* v_plan) const;
        std_msgs::msg::ColorRGBA TrailColor(const TrailPoint& p) const;

        // Helper function for time formatting
        std::string format_time_3dp(double time_val);

        // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - //
        // Variable

        // Subscribers
        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr                    s_odom_;
        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr                    s_frenet_odom_;
        rclcpp::Subscription<f1tenth_planning_msgs::msg::WpntArray>::SharedPtr                s_global_waypoints_;
        rclcpp::Subscription<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr s_steering_input_;

        // Input
        nav_msgs::msg::Odometry                       i_current_odom_;
        nav_msgs::msg::Odometry                       i_current_frenet_odom_;
        ackermann_msgs::msg::AckermannDriveStamped    i_current_steering_input_;

        // Mutex
        std::mutex mutex_odom_;
        std::mutex mutex_frenet_odom_;
        std::mutex mutex_steering_input_;
        std::mutex mutex_raceline_;

        // Publishers
        rclcpp::Publisher<rviz_2d_overlay_msgs::msg::OverlayText>::SharedPtr  p_lap_info_;
        rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr                  p_current_speed_info_;
        rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr                  p_steering_input_;
        rclcpp::Publisher<rviz_2d_overlay_msgs::msg::OverlayText>::SharedPtr  p_mean_lap_time_;
        rclcpp::Publisher<rviz_2d_overlay_msgs::msg::OverlayText>::SharedPtr  p_fastest_lap_time_;
        rclcpp::Publisher<rviz_2d_overlay_msgs::msg::OverlayText>::SharedPtr  p_mean_cte_;
        rclcpp::Publisher<rviz_2d_overlay_msgs::msg::OverlayText>::SharedPtr  p_max_cte_;
        rclcpp::Publisher<rviz_2d_overlay_msgs::msg::OverlayText>::SharedPtr  p_current_cte_;
        rclcpp::Publisher<rviz_2d_overlay_msgs::msg::OverlayText>::SharedPtr  p_segment_info_;
        rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr    p_driven_path_;
        rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr    p_deviation_;

        // Timer
        rclcpp::TimerBase::SharedPtr t_run_node_;

        // Flag
        bool b_is_odom_ = false;
        bool b_is_frenet_odom_ = false;
        bool b_is_max_s_ = false;
        bool b_is_steering_input_ = false;

        // Global Variables
        double loop_rate_hz_;
        double max_s_ = 0.0;  // Maximum s value (track length)
        double start_zone_threshold_;  // Start zone threshold parameter
        int text_size_;  // Text size for overlay
        std::string map_frame_;

        // Lap tracking variables
        int lap_count_ = 0;
        double previous_s_ = 0.0;
        bool is_first_measurement_ = true;
        // (is_race_started_ 제거: 쓰기만 하고 읽는 곳이 없었다. '중간 출발 랩은
        //  통계에서 뺀다'는 의도가 있었다면 미구현으로 남아 있던 것이다 —
        //  현재는 유효/무효 출발 모두 동일하게 카운트/타이밍한다.)

        // Lap time tracking variables
        rclcpp::Time lap_start_time_;
        std::vector<double> lap_times_;  // Store completed lap times
        double fastest_lap_time_ = std::numeric_limits<double>::max();
        double mean_lap_time_ = 0.0;     // Mean lap time for recent 5 laps
        double previous_lap_time_ = 0.0;  // Store previous completed lap time

        // CTE tracking variables
              double max_cte_ = 0.0;           // Maximum CTE across all laps
        double current_lap_cte_sum_ = 0.0;
        int current_lap_cte_count_ = 0;
        double mean_cte_ = 0.0;          // Mean CTE for current lap
        double current_cte_ = 0.0;       // Current CTE value
        double total_cte_sum_ = 0.0;     // Sum for 10-lap mean
        int total_cte_count_ = 0;        // Count for 10-lap mean
        double ten_lap_mean_cte_ = 0.0;  // Mean CTE over last 10 laps

        // Raceline cache (global waypoints) — s 오름차순 가정
        std::vector<double> wp_s_;
        std::vector<double> wp_x_;
        std::vector<double> wp_y_;
        std::vector<double> wp_v_;
        double raceline_v_max_ = 0.0;

        // Trail parameters
        bool trail_enabled_;
        double trail_min_dist_m_;
        int trail_max_points_;
        double trail_line_width_;
        std::string trail_color_mode_;   // speed | cte | speed_error
        double trail_v_min_;
        double trail_v_max_;             // 0이면 레이스라인 최대속도에서 자동
        double trail_cte_max_;      // cte 색 스케일 [m]
        double trail_v_err_max_;    // speed_error 색 스케일 [m/s]
        int trail_keep_previous_laps_;
        double trail_publish_rate_hz_;
        bool deviation_enabled_;
        int deviation_stride_;

        // Trail state
        std::vector<TrailPoint> trail_;                        // 현재 랩
        std::deque<std::vector<TrailPoint>> previous_trails_;  // 지난 랩 (고스트)
        double trail_publish_accum_ = 0.0;
        bool trail_published_this_tick_ = false;

        // Segment parameters / state
        double segment_length_m_;
        bool segment_log_enabled_;
        std::string segment_log_dir_;
        std::string session_dir_;
        bool session_dir_ready_ = false;
        std::vector<SegmentStats> segments_;
        int current_segment_idx_ = -1;
        double current_segment_v_ = 0.0;
        double current_segment_v_plan_ = 0.0;
};

#endif  // MONITOR_PKG__MONITOR_NODE_HPP_
