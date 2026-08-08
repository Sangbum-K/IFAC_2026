#include "monitor_pkg/monitor_node.hpp"
#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <ctime>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <sstream>

namespace {

// matplotlib viridis 근사 (아래 9개 앵커 선형보간). 시뮬 플롯
// (scripts/global_traj_sim)과 같은 색 순서라 두 그림을 나란히 비교할 수 있다.
constexpr double kViridis[9][3] = {
    {0.267, 0.005, 0.329}, {0.283, 0.141, 0.458}, {0.254, 0.265, 0.530},
    {0.207, 0.372, 0.553}, {0.164, 0.471, 0.558}, {0.128, 0.567, 0.551},
    {0.135, 0.659, 0.518}, {0.267, 0.749, 0.441}, {0.993, 0.906, 0.144},
};

std_msgs::msg::ColorRGBA ViridisColor(double t, double alpha) {
    t = std::clamp(t, 0.0, 1.0);
    const double pos = t * 8.0;
    const int i = std::min(7, static_cast<int>(pos));
    const double f = pos - i;

    std_msgs::msg::ColorRGBA c;
    c.r = static_cast<float>(kViridis[i][0] + f * (kViridis[i + 1][0] - kViridis[i][0]));
    c.g = static_cast<float>(kViridis[i][1] + f * (kViridis[i + 1][1] - kViridis[i][1]));
    c.b = static_cast<float>(kViridis[i][2] + f * (kViridis[i + 1][2] - kViridis[i][2]));
    c.a = static_cast<float>(alpha);
    return c;
}

// 0(초록) -> 1(빨강). CTE / 속도오차처럼 "작을수록 좋다"는 값에 쓴다.
std_msgs::msg::ColorRGBA GreenRedColor(double t, double alpha) {
    t = std::clamp(t, 0.0, 1.0);
    std_msgs::msg::ColorRGBA c;
    c.r = static_cast<float>(t);
    c.g = static_cast<float>(1.0 - t);
    c.b = 0.15f;
    c.a = static_cast<float>(alpha);
    return c;
}

std::string ExpandHome(const std::string& path) {
    if (path.empty() || path[0] != '~') {
        return path;
    }
    const char* home = std::getenv("HOME");
    if (home == nullptr) {
        return path;
    }
    return std::string(home) + path.substr(1);
}

std::string WallClockStamp() {
    const std::time_t now = std::time(nullptr);
    std::tm tm_buf{};
    localtime_r(&now, &tm_buf);
    char buf[32];
    std::strftime(buf, sizeof(buf), "%Y%m%d_%H%M%S", &tm_buf);
    return std::string(buf);
}

}  // namespace

MonitorNode::MonitorNode(const std::string &node_name, const rclcpp::NodeOptions &options)
    : Node(node_name, options), lap_start_time_(this->get_clock()->now()) {

    // QoS init
    auto qos_profile = rclcpp::QoS(rclcpp::KeepLast(10));

    // Parameters - declare and get values
    this->declare_parameter("monitor/loop_rate_hz", 20.0);
    this->declare_parameter("monitor/start_zone_threshold", 1.0);
    this->declare_parameter("monitor/text_size", 18);
    // 환경별 토픽 (config/environments/*.yaml에서 override)
    // sim: /ego_racecar/odom, /drive
    // real_car: /pf/pose/odom, /ackermann_cmd
    this->declare_parameter("monitor/odom_topic", "/ego_racecar/odom");
    this->declare_parameter("monitor/ackermann_topic", "/drive");
    this->declare_parameter("monitor/map_frame", "map");

    // 주행 궤적(trail) — 실제로 지나간 자리를 속도 컬러맵 선으로 남긴다
    this->declare_parameter("monitor/trail/enabled", true);
    this->declare_parameter("monitor/trail/min_dist_m", 0.05);
    this->declare_parameter("monitor/trail/max_points", 20000);
    this->declare_parameter("monitor/trail/line_width", 0.06);
    this->declare_parameter("monitor/trail/color_mode", "speed");  // speed | cte | speed_error
    this->declare_parameter("monitor/trail/v_min", 0.0);
    this->declare_parameter("monitor/trail/v_max", 0.0);  // 0 -> 레이스라인 최대속도에서 자동
    this->declare_parameter("monitor/trail/cte_max", 0.5);
    // speed_error 색 스케일 [m/s]: 빨강 = 계획보다 이만큼 느림
    this->declare_parameter("monitor/trail/v_err_max", 2.0);
    this->declare_parameter("monitor/trail/keep_previous_laps", 1);
    this->declare_parameter("monitor/trail/publish_rate_hz", 5.0);
    // 레이스라인 대비 이탈량 사다리 마커
    this->declare_parameter("monitor/deviation/enabled", true);
    this->declare_parameter("monitor/deviation/stride", 5);
    // s 구간별 속도/CTE 기록
    this->declare_parameter("monitor/segment/length_m", 1.0);
    this->declare_parameter("monitor/segment/log_enabled", true);
    this->declare_parameter("monitor/segment/log_dir", "~/monitor_logs");

    loop_rate_hz_ = this->get_parameter("monitor/loop_rate_hz").as_double();
    start_zone_threshold_ = this->get_parameter("monitor/start_zone_threshold").as_double();
    text_size_ = this->get_parameter("monitor/text_size").as_int();
    std::string odom_topic = this->get_parameter("monitor/odom_topic").as_string();
    std::string ackermann_topic = this->get_parameter("monitor/ackermann_topic").as_string();
    map_frame_ = this->get_parameter("monitor/map_frame").as_string();

    trail_enabled_ = this->get_parameter("monitor/trail/enabled").as_bool();
    trail_min_dist_m_ = this->get_parameter("monitor/trail/min_dist_m").as_double();
    trail_max_points_ = static_cast<int>(this->get_parameter("monitor/trail/max_points").as_int());
    trail_line_width_ = this->get_parameter("monitor/trail/line_width").as_double();
    trail_color_mode_ = this->get_parameter("monitor/trail/color_mode").as_string();
    trail_v_min_ = this->get_parameter("monitor/trail/v_min").as_double();
    trail_v_max_ = this->get_parameter("monitor/trail/v_max").as_double();
    trail_cte_max_ = this->get_parameter("monitor/trail/cte_max").as_double();
    trail_v_err_max_ =
        this->get_parameter("monitor/trail/v_err_max").as_double();
    trail_keep_previous_laps_ = static_cast<int>(this->get_parameter("monitor/trail/keep_previous_laps").as_int());
    trail_publish_rate_hz_ = this->get_parameter("monitor/trail/publish_rate_hz").as_double();
    deviation_enabled_ = this->get_parameter("monitor/deviation/enabled").as_bool();
    deviation_stride_ = std::max(1, static_cast<int>(this->get_parameter("monitor/deviation/stride").as_int()));
    segment_length_m_ = std::max(0.05, this->get_parameter("monitor/segment/length_m").as_double());
    segment_log_enabled_ = this->get_parameter("monitor/segment/log_enabled").as_bool();
    segment_log_dir_ = ExpandHome(this->get_parameter("monitor/segment/log_dir").as_string());

    RCLCPP_INFO(this->get_logger(), "loop_rate_hz: %f", loop_rate_hz_);
    RCLCPP_INFO(this->get_logger(), "start_zone_threshold: %f", start_zone_threshold_);
    RCLCPP_INFO(this->get_logger(), "text_size: %d", text_size_);
    RCLCPP_INFO(this->get_logger(), "odom_topic: %s", odom_topic.c_str());
    RCLCPP_INFO(this->get_logger(), "ackermann_topic: %s", ackermann_topic.c_str());
    RCLCPP_INFO(this->get_logger(), "trail: %s (color_mode: %s), segment: %.2f m, log: %s",
                trail_enabled_ ? "on" : "off", trail_color_mode_.c_str(), segment_length_m_,
                segment_log_enabled_ ? segment_log_dir_.c_str() : "off");

    // Subscribers init
    s_odom_ = this->create_subscription<nav_msgs::msg::Odometry>(
        odom_topic, qos_profile, std::bind(&MonitorNode::CallbackOdom, this, std::placeholders::_1));
    s_frenet_odom_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/car_state/frenet/odom", qos_profile, std::bind(&MonitorNode::CallbackFrenetOdom, this, std::placeholders::_1));
    s_global_waypoints_ = this->create_subscription<f1tenth_planning_msgs::msg::WpntArray>(
        "/global_waypoints", qos_profile, std::bind(&MonitorNode::CallbackGlobalWaypoints, this, std::placeholders::_1));
    s_steering_input_ = this->create_subscription<ackermann_msgs::msg::AckermannDriveStamped>(
        ackermann_topic, qos_profile, std::bind(&MonitorNode::CallbackSteeringInput, this, std::placeholders::_1));

    // Publisher init
    p_lap_info_ = this->create_publisher<rviz_2d_overlay_msgs::msg::OverlayText>(
        "/monitor/lap_info", qos_profile);
    p_current_speed_info_ = this->create_publisher<std_msgs::msg::Float32>(
        "/monitor/current_speed", qos_profile);
    p_steering_input_ = this->create_publisher<std_msgs::msg::Float32>(
        "/monitor/steering_input", qos_profile);
    p_mean_lap_time_ = this->create_publisher<rviz_2d_overlay_msgs::msg::OverlayText>(
        "/monitor/mean_lap_time", qos_profile);
    p_fastest_lap_time_ = this->create_publisher<rviz_2d_overlay_msgs::msg::OverlayText>(
        "/monitor/fastest_lap_time", qos_profile);
    p_mean_cte_ = this->create_publisher<rviz_2d_overlay_msgs::msg::OverlayText>(
        "/monitor/mean_cte", qos_profile);
    p_max_cte_ = this->create_publisher<rviz_2d_overlay_msgs::msg::OverlayText>(
        "/monitor/max_cte", qos_profile);
    p_current_cte_ = this->create_publisher<rviz_2d_overlay_msgs::msg::OverlayText>(
        "/monitor/current_cte", qos_profile);
    p_segment_info_ = this->create_publisher<rviz_2d_overlay_msgs::msg::OverlayText>(
        "/monitor/segment_info", qos_profile);
    // 궤적 마커는 늦게 붙는 RViz도 바로 볼 수 있게 transient_local
    auto marker_qos = rclcpp::QoS(rclcpp::KeepLast(1)).transient_local();
    p_driven_path_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
        "/monitor/driven_path", marker_qos);
    p_deviation_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
        "/monitor/deviation_markers", marker_qos);

    // Timer init
    t_run_node_ = this->create_wall_timer(
        std::chrono::milliseconds((int64_t)(1000 / loop_rate_hz_)),
        [this]() { this->Run(); });

    RCLCPP_INFO(this->get_logger(), "MonitorNode initialized successfully");
}

MonitorNode::~MonitorNode() {}

// Helper function for time formatting with 3 decimal places
std::string MonitorNode::format_time_3dp(double time_val) {
    std::ostringstream oss;
    oss << std::fixed << std::setprecision(3) << time_val;
    return oss.str();
}

void MonitorNode::Run() {
    RCLCPP_INFO_THROTTLE(this->get_logger(), *get_clock(), 1000, "Monitor Running ...");

    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - //
    // Get subscribe variables
    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - //
    if (b_is_odom_ == false) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *get_clock(), 1000, "Waiting for odom...");
        return;
    }
    if (b_is_frenet_odom_ == false) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *get_clock(), 1000, "Waiting for frenet odom...");
        return;
    }
    // Track length is optional for monitoring, so no need to wait for it

    nav_msgs::msg::Odometry odom; {
        std::lock_guard<std::mutex> lock(mutex_odom_);
        odom = i_current_odom_;
    }
    nav_msgs::msg::Odometry frenet_odom; {
        std::lock_guard<std::mutex> lock(mutex_frenet_odom_);
        frenet_odom = i_current_frenet_odom_;
    }

    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - //
    // Process & Publish monitor information
    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - //

    // Update and publish lap information
    // (랩 경계에서 궤적/구간 통계가 정리되므로 UpdateTrail보다 먼저 호출한다)
    UpdateLapInfo(frenet_odom);
    PublishLapInfo();

    // Update CTE information
    UpdateCTE(frenet_odom);

    // 주행 궤적 + 구간별 속도 기록
    UpdateTrail(odom, frenet_odom);
    PublishTrailMarkers();
    PublishDeviationMarkers();
    PublishSegmentInfo();

    // Publish lap time information
    PublishMeanLapTime();
    PublishFastestLapTime();

    // Publish CTE information
    PublishMeanCTE();
    PublishMaxCTE();
    PublishCurrentCTE();

    // Publish speed, steering information
    PublishSpeedInfo(odom);
    PublishSteeringInput();
}

void MonitorNode::PublishSpeedInfo(const nav_msgs::msg::Odometry& odom) {
    // Calculate current speed from odometry
    double linear_velocity = odom.twist.twist.linear.x;

    // Publish speed in m/s for PieChart visualization
    std_msgs::msg::Float32 speed_msg;
    speed_msg.data = static_cast<float>(linear_velocity);
    p_current_speed_info_->publish(speed_msg);
}

void MonitorNode::PublishSteeringInput() {
    if (b_is_steering_input_) {
        std_msgs::msg::Float32 steering_msg;
        steering_msg.data = i_current_steering_input_.drive.steering_angle;
        p_steering_input_->publish(steering_msg);
    }
}

void MonitorNode::UpdateLapInfo(const nav_msgs::msg::Odometry& frenet_odom) {
    // Get current s coordinate
    double current_s = frenet_odom.pose.pose.position.x;

    // Wait until we have max_s information
    if (max_s_ <= 0.0) {
        return;
    }

    // First measurement - check if we're starting from proper position
    if (is_first_measurement_) {
        // Check if starting position is in the start zone
        bool in_start_zone = (current_s < start_zone_threshold_) ||
                             (current_s > (max_s_ - start_zone_threshold_));

        if (in_start_zone) {
            lap_count_ = 0;  // Start from lap 0
            lap_start_time_ = this->get_clock()->now();  // Start timing
            RCLCPP_INFO(this->get_logger(),
                       "Race started from proper position. Current s: %.2f, threshold: %.2f",
                       current_s, start_zone_threshold_);
        } else {
            lap_count_ = 0;  // Start counting from 0 even for invalid start
            lap_start_time_ = this->get_clock()->now();  // Start timing even for invalid start
            RCLCPP_WARN(this->get_logger(),
                       "Race started from middle of track! Current s: %.2f, max_s: %.2f, threshold: %.2f",
                       current_s, max_s_, start_zone_threshold_);
            RCLCPP_WARN(this->get_logger(),
                       "Lap counting and timing enabled from invalid start position!");
        }

        previous_s_ = current_s;
        is_first_measurement_ = false;
        return;
    }

    // Lap completion detection: crossed from high s to low s
    if (previous_s_ > (max_s_ - start_zone_threshold_) &&
        current_s < start_zone_threshold_) {

        // Calculate lap time
        auto current_time = this->get_clock()->now();
        double lap_time = (current_time - lap_start_time_).seconds();

        // Record lap time if it's not the first lap (regardless of start validity)
        if (lap_count_ > 0) {
            lap_times_.push_back(lap_time);
            previous_lap_time_ = lap_time;  // Store previous lap time

            // Update fastest lap time
            if (lap_time < fastest_lap_time_) {
                fastest_lap_time_ = lap_time;
            }

            // Calculate mean lap time for recent 5 laps
            size_t recent_laps = std::min(static_cast<size_t>(5), lap_times_.size());
            double sum = 0.0;
            for (size_t i = lap_times_.size() - recent_laps; i < lap_times_.size(); i++) {
                sum += lap_times_[i];
            }
            mean_lap_time_ = sum / recent_laps;

            RCLCPP_INFO(this->get_logger(),
                       "Lap %d completed! Time: %.3f s, Fastest: %.3f s, Mean: %.3f s, Mean CTE: %.3f m",
                       lap_count_, lap_time, fastest_lap_time_, mean_lap_time_, mean_cte_);
        }

        // 방금 끝난 랩의 궤적/구간 통계를 정리 (CSV 기록 + 고스트로 이동)
        FinalizeLap(lap_count_, lap_time);

        lap_count_++;
        lap_start_time_ = current_time;  // Reset lap start time for next lap

        // Reset CTE tracking for new lap
        current_lap_cte_sum_ = 0.0;
        current_lap_cte_count_ = 0;
        mean_cte_ = 0.0;

        // Reset mean CTE every 10 laps
        if (lap_count_ % 10 == 0) {
            total_cte_sum_ = 0.0;
            total_cte_count_ = 0;
            ten_lap_mean_cte_ = 0.0;
            RCLCPP_INFO(this->get_logger(), "10-lap mean CTE reset at lap %d", lap_count_);
        }

        RCLCPP_INFO(this->get_logger(),
                   "Starting lap %d. s: %.2f -> %.2f", lap_count_, previous_s_, current_s);
    }

    // Update previous s for next iteration
    previous_s_ = current_s;
}

void MonitorNode::PublishLapInfo() {
    rviz_2d_overlay_msgs::msg::OverlayText lap_msg;
    lap_msg.action = rviz_2d_overlay_msgs::msg::OverlayText::ADD;
    lap_msg.text_size = text_size_;  // Use parameter value

    // Calculate current lap time (time since lap started)
    auto current_time = this->get_clock()->now();
    double current_lap_time = (current_time - lap_start_time_).seconds();

    // Create lap info text with current lap number, current lap time, and previous lap time
    std::string lap_text = "Lap: " + std::to_string(lap_count_) +
                          " | Lap Time: " + format_time_3dp(current_lap_time) + "s";

    if (previous_lap_time_ > 0.0) {
        lap_text += " | Prev. Lap Time: " + format_time_3dp(previous_lap_time_) + "s";
    }

    lap_msg.text = lap_text;

    p_lap_info_->publish(lap_msg);
}

void MonitorNode::PublishMeanLapTime() {
    rviz_2d_overlay_msgs::msg::OverlayText msg;
    msg.action = rviz_2d_overlay_msgs::msg::OverlayText::ADD;
    msg.text_size = text_size_;

    if (lap_times_.empty()) {
        msg.text = "Mean Lap Time (5lap): N/A";
    } else {
        msg.text = "Mean Lap Time (5lap): " + format_time_3dp(mean_lap_time_) + "s";
    }

    p_mean_lap_time_->publish(msg);
}

void MonitorNode::PublishFastestLapTime() {
    rviz_2d_overlay_msgs::msg::OverlayText msg;
    msg.action = rviz_2d_overlay_msgs::msg::OverlayText::ADD;
    msg.text_size = text_size_;

    if (fastest_lap_time_ == std::numeric_limits<double>::max()) {
        msg.text = "Fastest Lap Time: N/A";
    } else {
        msg.text = "Fastest Lap Time: " + format_time_3dp(fastest_lap_time_) + "s";
    }

    p_fastest_lap_time_->publish(msg);
}

void MonitorNode::UpdateCTE(const nav_msgs::msg::Odometry& frenet_odom) {
    // Get current Cross Track Error (frenet d coordinate)
    current_cte_ = std::abs(frenet_odom.pose.pose.position.y);

    // Track CTE if we have valid data (lap_count > 0)
    if (lap_count_ > 0) {
        // Update current lap CTE statistics
        current_lap_cte_sum_ += current_cte_;
        current_lap_cte_count_++;
        mean_cte_ = current_lap_cte_sum_ / current_lap_cte_count_;

        // Update 10-lap rolling mean CTE
        total_cte_sum_ += current_cte_;
        total_cte_count_++;
        ten_lap_mean_cte_ = total_cte_sum_ / total_cte_count_;

        // Update maximum CTE across all laps
        if (current_cte_ > max_cte_) {
            max_cte_ = current_cte_;
        }
    }
}

void MonitorNode::PublishMeanCTE() {
    rviz_2d_overlay_msgs::msg::OverlayText msg;
    msg.action = rviz_2d_overlay_msgs::msg::OverlayText::ADD;
    msg.text_size = text_size_;

    if (total_cte_count_ == 0) {
        msg.text = "Mean CTE (10lap): N/A";
    } else {
        msg.text = "Mean CTE (10lap): " + std::to_string(ten_lap_mean_cte_).substr(0, 5) + "m";
    }

    p_mean_cte_->publish(msg);
}

void MonitorNode::PublishMaxCTE() {
    rviz_2d_overlay_msgs::msg::OverlayText msg;
    msg.action = rviz_2d_overlay_msgs::msg::OverlayText::ADD;
    msg.text_size = text_size_;

    if (max_cte_ == 0.0) {
        msg.text = "Max CTE: N/A";
    } else {
        msg.text = "Max CTE: " + std::to_string(max_cte_).substr(0, 5) + "m";
    }

    p_max_cte_->publish(msg);
}

void MonitorNode::PublishCurrentCTE() {
    rviz_2d_overlay_msgs::msg::OverlayText msg;
    msg.action = rviz_2d_overlay_msgs::msg::OverlayText::ADD;
    msg.text_size = text_size_;

    // Show current CTE always (even for invalid starts)
    msg.text = "Current CTE: " + std::to_string(current_cte_).substr(0, 5) + "m";

    p_current_cte_->publish(msg);
}


// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - //
// 주행 궤적 / 구간별 속도 기록
// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - //

void MonitorNode::CallbackGlobalWaypoints(const f1tenth_planning_msgs::msg::WpntArray::SharedPtr msg) {
    if (msg->wpnts.empty()) {
        return;
    }
    if (!b_is_max_s_) {
        max_s_ = msg->wpnts.back().s_m;  // Get the last s value as max_s
        b_is_max_s_ = true;
    }

    // 레이스라인 캐시 — 구간별 "목표 속도"와 이탈량 기준선으로 쓴다.
    std::lock_guard<std::mutex> lock(mutex_raceline_);
    const size_t n = msg->wpnts.size();
    if (wp_s_.size() == n) {
        return;  // 글로벌 레이스라인은 한 번 계산되면 바뀌지 않는다
    }
    wp_s_.clear(); wp_x_.clear(); wp_y_.clear(); wp_v_.clear();
    wp_s_.reserve(n); wp_x_.reserve(n); wp_y_.reserve(n); wp_v_.reserve(n);
    raceline_v_max_ = 0.0;
    for (const auto& w : msg->wpnts) {
        wp_s_.push_back(w.s_m);
        wp_x_.push_back(w.x_m);
        wp_y_.push_back(w.y_m);
        wp_v_.push_back(w.vx_mps);
        raceline_v_max_ = std::max(raceline_v_max_, w.vx_mps);
    }
    RCLCPP_INFO(this->get_logger(),
                "Raceline cached: %zu wpnts, track %.2f m, v_max %.2f m/s",
                n, max_s_, raceline_v_max_);
}

bool MonitorNode::LookupRaceline(double s, double* ref_x, double* ref_y, double* v_plan) const {
    if (wp_s_.size() < 2) {
        return false;
    }
    // s는 단조 증가 배열 — 구간을 이분탐색해서 선형보간
    const double s_clamped = std::clamp(s, wp_s_.front(), wp_s_.back());
    const auto it = std::upper_bound(wp_s_.begin(), wp_s_.end(), s_clamped);
    size_t hi = static_cast<size_t>(it - wp_s_.begin());
    if (hi == 0) hi = 1;
    if (hi >= wp_s_.size()) hi = wp_s_.size() - 1;
    const size_t lo = hi - 1;

    const double ds = wp_s_[hi] - wp_s_[lo];
    const double f = (ds > 1e-9) ? (s_clamped - wp_s_[lo]) / ds : 0.0;
    *ref_x = wp_x_[lo] + f * (wp_x_[hi] - wp_x_[lo]);
    *ref_y = wp_y_[lo] + f * (wp_y_[hi] - wp_y_[lo]);
    *v_plan = wp_v_[lo] + f * (wp_v_[hi] - wp_v_[lo]);
    return true;
}

void MonitorNode::UpdateTrail(const nav_msgs::msg::Odometry& odom,
                              const nav_msgs::msg::Odometry& frenet_odom) {
    const double now_sec = this->get_clock()->now().seconds();
    const double x = odom.pose.pose.position.x;
    const double y = odom.pose.pose.position.y;
    const double v = odom.twist.twist.linear.x;
    const double s = frenet_odom.pose.pose.position.x;
    const double d = frenet_odom.pose.pose.position.y;

    double ref_x = 0.0, ref_y = 0.0, v_plan = 0.0;
    bool has_ref = false; {
        std::lock_guard<std::mutex> lock(mutex_raceline_);
        has_ref = LookupRaceline(s, &ref_x, &ref_y, &v_plan);
    }

    // 구간 통계는 (궤적 점 간격과 무관하게) 매 루프 갱신
    UpdateSegments(s, v, has_ref ? v_plan : 0.0, d, now_sec);
    current_segment_v_ = v;
    current_segment_v_plan_ = has_ref ? v_plan : 0.0;

    if (!trail_enabled_) {
        return;
    }

    if (!trail_.empty()) {
        const double step = std::hypot(x - trail_.back().x, y - trail_.back().y);
        // 2D Pose Estimate 등으로 위치가 튀면 직선으로 이어 그리지 말고 새로 시작
        constexpr double kJumpDist = 1.0;
        if (step > kJumpDist) {
            RCLCPP_WARN(this->get_logger(),
                        "Pose jumped %.2f m — restarting driven trail", step);
            trail_.clear();
        } else if (step < trail_min_dist_m_) {
            return;
        }
    }

    TrailPoint p;
    p.x = x; p.y = y; p.v = v; p.v_plan = v_plan;
    p.s = s; p.d = d;
    p.ref_x = ref_x; p.ref_y = ref_y; p.has_ref = has_ref;
    trail_.push_back(p);

    if (trail_max_points_ > 0 && static_cast<int>(trail_.size()) > trail_max_points_) {
        // 앞쪽 10%를 한 번에 버린다 (매 점마다 erase하면 O(n) 반복)
        const size_t drop = trail_.size() / 10 + 1;
        trail_.erase(trail_.begin(), trail_.begin() + drop);
    }
}

void MonitorNode::UpdateSegments(double s, double v, double v_plan, double d, double now_sec) {
    if (max_s_ <= 0.0) {
        return;  // 트랙 길이를 모르면 구간을 나눌 수 없다
    }
    const size_t num_segments =
        std::max<size_t>(1, static_cast<size_t>(std::ceil(max_s_ / segment_length_m_)));
    if (segments_.size() != num_segments) {
        segments_.assign(num_segments, SegmentStats());
    }

    const size_t idx = std::min(num_segments - 1,
                                static_cast<size_t>(std::max(0.0, s) / segment_length_m_));
    SegmentStats& seg = segments_[idx];
    seg.count++;
    seg.sum_v += v;
    seg.max_v = std::max(seg.max_v, v);
    seg.min_v = std::min(seg.min_v, v);
    seg.sum_v_plan += v_plan;
    const double abs_d = std::abs(d);
    seg.sum_abs_d += abs_d;
    seg.max_abs_d = std::max(seg.max_abs_d, abs_d);
    if (seg.first_t < 0.0) {
        seg.first_t = now_sec;
    }
    seg.last_t = now_sec;

    current_segment_idx_ = static_cast<int>(idx);
}

std_msgs::msg::ColorRGBA MonitorNode::TrailColor(const TrailPoint& p) const {
    if (trail_color_mode_ == "cte") {
        return GreenRedColor(std::abs(p.d) / std::max(1e-3, trail_cte_max_), 1.0);
    }
    if (trail_color_mode_ == "speed_error") {
        // 목표보다 얼마나 느린지 (레이스라인 정보가 없으면 0)
        const double err = p.has_ref ? std::max(0.0, p.v_plan - p.v) : 0.0;
        // 예전엔 trail_cte_max_(거리 [m]) * 4.0 을 속도오차 [m/s] 스케일로
        // 재사용했다 — 단위가 다른 물리량이라 cte_max 를 만지면 speed_error
        // 그림의 붉기 기준이 조용히 같이 바뀌었다.
        return GreenRedColor(err / std::max(1e-3, trail_v_err_max_), 1.0);
    }
    // 기본: 속도 컬러맵 (시뮬 플롯과 같은 viridis)
    const double v_max = (trail_v_max_ > 0.0)
                             ? trail_v_max_
                             : ((raceline_v_max_ > 0.0) ? raceline_v_max_ : 8.0);
    const double span = std::max(1e-3, v_max - trail_v_min_);
    return ViridisColor((p.v - trail_v_min_) / span, 1.0);
}

void MonitorNode::PublishTrailMarkers() {
    trail_published_this_tick_ = false;
    if (!trail_enabled_) {
        return;
    }
    // 궤적은 점이 많아 매 루프 발행하면 낭비 — publish_rate_hz로 낮춘다
    trail_publish_accum_ += 1.0 / std::max(1e-3, loop_rate_hz_);
    if (trail_publish_accum_ < 1.0 / std::max(1e-3, trail_publish_rate_hz_)) {
        return;
    }
    trail_publish_accum_ = 0.0;
    trail_published_this_tick_ = true;

    visualization_msgs::msg::MarkerArray array;
    const auto stamp = this->get_clock()->now();

    // 지난 랩 (회색 고스트) — 이번 랩과 겹쳐 보면 라인이 랩마다 얼마나 흔들리는지 보인다
    int ghost_id = 0;
    for (const auto& ghost : previous_trails_) {
        visualization_msgs::msg::Marker m;
        m.header.frame_id = map_frame_;
        m.header.stamp = stamp;
        m.ns = "driven_prev";
        m.id = ghost_id++;
        m.type = visualization_msgs::msg::Marker::LINE_STRIP;
        m.action = visualization_msgs::msg::Marker::ADD;
        m.pose.orientation.w = 1.0;
        m.scale.x = trail_line_width_ * 0.7;
        m.color.r = 0.6f; m.color.g = 0.6f; m.color.b = 0.6f; m.color.a = 0.35f;
        m.points.reserve(ghost.size());
        for (const auto& p : ghost) {
            geometry_msgs::msg::Point pt;
            pt.x = p.x; pt.y = p.y; pt.z = 0.02;
            m.points.push_back(pt);
        }
        if (m.points.size() >= 2) {
            array.markers.push_back(m);
        }
    }
    // (고스트 DELETE 루프 제거: previous_trails_ 는 FinalizeLap 의 push_back 과
    //  상한 초과 시 pop_front 만 하므로 크기가 단조 비감소이고,
    //  trail_keep_previous_laps_ 는 생성자에서 한 번만 읽는다 — ghost_id 가
    //  직전 값보다 작아질 수 없어 루프 본문에 도달할 수 없었다.
    //  trail_keep_previous_laps_ 를 동적 파라미터로 바꾸면 다시 필요해진다.)

    // 현재 랩 — 점마다 속도(또는 CTE) 색
    visualization_msgs::msg::Marker cur;
    cur.header.frame_id = map_frame_;
    cur.header.stamp = stamp;
    cur.ns = "driven";
    cur.id = 0;
    cur.type = visualization_msgs::msg::Marker::LINE_STRIP;
    cur.action = trail_.size() >= 2 ? visualization_msgs::msg::Marker::ADD
                                    : visualization_msgs::msg::Marker::DELETE;
    cur.pose.orientation.w = 1.0;
    cur.scale.x = trail_line_width_;
    cur.color.a = 1.0f;
    cur.points.reserve(trail_.size());
    cur.colors.reserve(trail_.size());
    for (const auto& p : trail_) {
        geometry_msgs::msg::Point pt;
        pt.x = p.x; pt.y = p.y; pt.z = 0.03;
        cur.points.push_back(pt);
        cur.colors.push_back(TrailColor(p));
    }
    array.markers.push_back(cur);

    p_driven_path_->publish(array);
}

void MonitorNode::PublishDeviationMarkers() {
    if (!deviation_enabled_ || !trail_published_this_tick_) {
        return;  // 궤적을 발행한 틱에만 같이 갱신
    }

    visualization_msgs::msg::Marker m;
    m.header.frame_id = map_frame_;
    m.header.stamp = this->get_clock()->now();
    m.ns = "deviation";
    m.id = 0;
    m.type = visualization_msgs::msg::Marker::LINE_LIST;
    m.action = visualization_msgs::msg::Marker::ADD;
    m.pose.orientation.w = 1.0;
    m.scale.x = std::max(0.01, trail_line_width_ * 0.4);
    m.color.a = 1.0f;

    // 궤적 점 -> 같은 s의 레이스라인 점을 잇는 사다리. 길수록/붉을수록 많이 벗어난 것.
    for (size_t i = 0; i < trail_.size(); i += static_cast<size_t>(deviation_stride_)) {
        const TrailPoint& p = trail_[i];
        if (!p.has_ref) {
            continue;
        }
        geometry_msgs::msg::Point a, b;
        a.x = p.x; a.y = p.y; a.z = 0.03;
        b.x = p.ref_x; b.y = p.ref_y; b.z = 0.03;
        m.points.push_back(a);
        m.points.push_back(b);
        const auto c = GreenRedColor(std::abs(p.d) / std::max(1e-3, trail_cte_max_), 0.9);
        m.colors.push_back(c);
        m.colors.push_back(c);
    }
    if (m.points.empty()) {
        m.action = visualization_msgs::msg::Marker::DELETE;
    }

    visualization_msgs::msg::MarkerArray array;
    array.markers.push_back(m);
    p_deviation_->publish(array);
}

void MonitorNode::PublishSegmentInfo() {
    rviz_2d_overlay_msgs::msg::OverlayText msg;
    msg.action = rviz_2d_overlay_msgs::msg::OverlayText::ADD;
    msg.text_size = text_size_;

    if (current_segment_idx_ < 0) {
        msg.text = "Segment: N/A";
        p_segment_info_->publish(msg);
        return;
    }

    std::ostringstream oss;
    oss << std::fixed << std::setprecision(2);
    oss << "Seg " << current_segment_idx_
        << " [" << (current_segment_idx_ * segment_length_m_) << "-"
        << ((current_segment_idx_ + 1) * segment_length_m_) << "m]"
        << " | v " << current_segment_v_;
    if (current_segment_v_plan_ > 0.0) {
        const double err = current_segment_v_ - current_segment_v_plan_;
        oss << " / plan " << current_segment_v_plan_
            << " (" << std::showpos << err << std::noshowpos << ")";
    }
    oss << " m/s | d " << current_cte_ << "m";
    msg.text = oss.str();

    p_segment_info_->publish(msg);
}

void MonitorNode::FinalizeLap(int lap_number, double lap_time) {
    // 완주한 랩만 기록 (lap 0은 시작 위치에서 스타트라인까지의 워밍업 구간)
    if (lap_number > 0 && segment_log_enabled_ && !segments_.empty()) {
        WriteLapSegmentCsv(lap_number, lap_time);
        WriteLapSummaryCsv(lap_number, lap_time);
    }

    // 현재 궤적을 고스트로 넘기고 새 랩을 시작
    if (trail_keep_previous_laps_ > 0 && trail_.size() >= 2) {
        previous_trails_.push_back(trail_);
        while (static_cast<int>(previous_trails_.size()) > trail_keep_previous_laps_) {
            previous_trails_.pop_front();
        }
    }
    trail_.clear();
    segments_.clear();
    current_segment_idx_ = -1;
}

void MonitorNode::WriteLapSegmentCsv(int lap_number, double lap_time) {
    if (!session_dir_ready_) {
        session_dir_ = segment_log_dir_ + "/" + WallClockStamp();
        std::error_code ec;
        std::filesystem::create_directories(session_dir_, ec);
        if (ec) {
            RCLCPP_ERROR(this->get_logger(), "Cannot create log dir %s: %s",
                         session_dir_.c_str(), ec.message().c_str());
            segment_log_enabled_ = false;  // 매 랩 실패 로그를 반복하지 않는다
            return;
        }
        session_dir_ready_ = true;
        RCLCPP_INFO(this->get_logger(), "Segment logs -> %s", session_dir_.c_str());
    }

    std::ostringstream name;
    name << session_dir_ << "/lap_" << std::setw(3) << std::setfill('0') << lap_number
         << "_segments.csv";
    std::ofstream out(name.str());
    if (!out) {
        RCLCPP_ERROR(this->get_logger(), "Cannot write %s", name.str().c_str());
        return;
    }

    out << "# lap=" << lap_number << " lap_time_s=" << format_time_3dp(lap_time)
        << " segment_length_m=" << segment_length_m_
        << " track_length_m=" << max_s_ << "\n";
    out << "seg_idx,s_start_m,s_end_m,samples,dt_s,v_mean_mps,v_max_mps,v_min_mps,"
           "v_plan_mean_mps,v_err_mean_mps,cte_mean_m,cte_max_m\n";
    out << std::fixed << std::setprecision(3);

    for (size_t i = 0; i < segments_.size(); ++i) {
        const SegmentStats& seg = segments_[i];
        if (seg.count == 0) {
            continue;  // 이번 랩에 지나지 않은 구간 (중간 출발 등)
        }
        // 구간 소요시간: 다음 구간 진입 시각까지. 마지막 구간은 관측 구간 길이로 대체.
        double dt = seg.last_t - seg.first_t;
        if (i + 1 < segments_.size() && segments_[i + 1].first_t > seg.first_t) {
            dt = segments_[i + 1].first_t - seg.first_t;
        }
        const double v_mean = seg.sum_v / seg.count;
        const double v_plan_mean = seg.sum_v_plan / seg.count;
        out << i << ","
            << (i * segment_length_m_) << ","
            << ((i + 1) * segment_length_m_) << ","
            << seg.count << ","
            << dt << ","
            << v_mean << ","
            << seg.max_v << ","
            << (seg.min_v == std::numeric_limits<double>::max() ? 0.0 : seg.min_v) << ","
            << v_plan_mean << ","
            << (v_mean - v_plan_mean) << ","
            << (seg.sum_abs_d / seg.count) << ","
            << seg.max_abs_d << "\n";
    }
    out.close();

    RCLCPP_INFO(this->get_logger(), "Lap %d segment log written: %s",
                lap_number, name.str().c_str());
}

void MonitorNode::WriteLapSummaryCsv(int lap_number, double lap_time) {
    if (!session_dir_ready_) {
        return;
    }
    const std::string path = session_dir_ + "/laps.csv";
    const bool need_header = !std::filesystem::exists(path);
    std::ofstream out(path, std::ios::app);
    if (!out) {
        RCLCPP_ERROR(this->get_logger(), "Cannot append %s", path.c_str());
        return;
    }
    if (need_header) {
        out << "lap,lap_time_s,mean_cte_m,max_cte_m,v_mean_mps,v_max_mps,"
               "v_plan_mean_mps,worst_seg_idx,worst_seg_v_err_mps\n";
    }

    // 랩 전체 평균 + "목표 대비 가장 느렸던 구간" — 어디서 시간을 잃었는지 바로 보인다
    double sum_v = 0.0, sum_v_plan = 0.0, v_max = 0.0, lap_max_cte = 0.0;
    int count = 0;
    int worst_idx = -1;
    double worst_err = 0.0;
    for (size_t i = 0; i < segments_.size(); ++i) {
        const SegmentStats& seg = segments_[i];
        if (seg.count == 0) {
            continue;
        }
        sum_v += seg.sum_v;
        sum_v_plan += seg.sum_v_plan;
        v_max = std::max(v_max, seg.max_v);
        lap_max_cte = std::max(lap_max_cte, seg.max_abs_d);
        count += seg.count;
        const double err = (seg.sum_v - seg.sum_v_plan) / seg.count;
        if (worst_idx < 0 || err < worst_err) {
            worst_idx = static_cast<int>(i);
            worst_err = err;
        }
    }
    if (count == 0) {
        return;
    }

    out << std::fixed << std::setprecision(3)
        << lap_number << "," << lap_time << "," << mean_cte_ << "," << lap_max_cte << ","
        << (sum_v / count) << "," << v_max << "," << (sum_v_plan / count) << ","
        << worst_idx << "," << worst_err << "\n";
}


int main(int argc, char **argv) {
    std::string node_name = "monitor_node";

    // Initialize node
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MonitorNode>(node_name));
    rclcpp::shutdown();
    return 0;
}
