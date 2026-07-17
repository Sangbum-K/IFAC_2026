#include "planning_pkg/local_trajectory_node.hpp"

namespace {
// 음수 안전 modulo (트랙 s 좌표 wrap-around 용)
double Mod(double a, double m) {
    double r = std::fmod(a, m);
    return (r < 0.0) ? r + m : r;
}
}  // namespace

LocalTrajectoryNode::LocalTrajectoryNode(const std::string &node_name, const rclcpp::NodeOptions &options)
    : Node(node_name, options) {

    // QoS init - Use RELIABLE QoS to match frenet_frame_node
    auto qos_profile = rclcpp::QoS(rclcpp::KeepLast(10)).reliable();

    // Parameters - declare and get values
    this->declare_parameter("local_trajectory/loop_rate_hz", 40.0);
    this->declare_parameter("local_trajectory/num_local_waypoints", 80);
    this->declare_parameter("local_trajectory/avoidance_timeout_sec", 0.5);

    loop_rate_hz_ = this->get_parameter("local_trajectory/loop_rate_hz").as_double();
    num_local_waypoints_ = this->get_parameter("local_trajectory/num_local_waypoints").as_int();
    avoidance_timeout_sec_ = this->get_parameter("local_trajectory/avoidance_timeout_sec").as_double();

    RCLCPP_INFO(this->get_logger(), "loop_rate_hz: %f", loop_rate_hz_);
    RCLCPP_INFO(this->get_logger(), "num_local_waypoints: %d", num_local_waypoints_);
    
    // Subscriber init
    s_global_waypoints_ = this->create_subscription<ae_hyu_msgs::msg::WpntArray>(
        "/global_waypoints", qos_profile,
        std::bind(&LocalTrajectoryNode::CallbackGlobalWaypoints, this, std::placeholders::_1));
    
    s_frenet_odom_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/car_state/frenet/odom", qos_profile,
        std::bind(&LocalTrajectoryNode::CallbackFrenetOdom, this, std::placeholders::_1));

    // spliner_node 회피 궤적 (없거나 비어 있으면 raceline 추종)
    s_avoidance_waypoints_ = this->create_subscription<ae_hyu_msgs::msg::WpntArray>(
        "/planner/avoidance/otwpnts", qos_profile,
        std::bind(&LocalTrajectoryNode::CallbackAvoidanceWaypoints, this, std::placeholders::_1));


    // Publisher init
    p_local_waypoints_ = this->create_publisher<ae_hyu_msgs::msg::WpntArray>(
        "/local_waypoints", qos_profile);
    
    // Timer init
    t_publish_ = this->create_wall_timer(
        std::chrono::milliseconds((int64_t)(1000 / loop_rate_hz_)),
        [this]() { this->Run(); });
        
    RCLCPP_INFO(this->get_logger(), "LocalTrajectoryNode initialized successfully");
}

LocalTrajectoryNode::~LocalTrajectoryNode() {}

void LocalTrajectoryNode::CallbackGlobalWaypoints(const ae_hyu_msgs::msg::WpntArray::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(mutex_global_waypoints_);
    global_waypoints_ = msg->wpnts;
    has_global_waypoints_ = true;
    
    RCLCPP_INFO_ONCE(this->get_logger(), "Received global waypoints: %zu waypoints", global_waypoints_.size());
}

void LocalTrajectoryNode::CallbackFrenetOdom(const nav_msgs::msg::Odometry::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(mutex_frenet_odom_);
    current_frenet_odom_ = *msg;
    has_frenet_odom_ = true;
    
    RCLCPP_DEBUG(this->get_logger(), "Received Frenet odom: s=%.3f, d=%.3f",
                msg->pose.pose.position.x, msg->pose.pose.position.y);
}

void LocalTrajectoryNode::CallbackAvoidanceWaypoints(const ae_hyu_msgs::msg::WpntArray::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(mutex_avoidance_);
    avoidance_waypoints_ = msg->wpnts;
    avoidance_stamp_ = msg->header.stamp;
}

int LocalTrajectoryNode::FindClosestWaypointIndex(double s_coordinate, 
                                                const std::vector<ae_hyu_msgs::msg::Wpnt>& global_waypoints) {
    if (global_waypoints.empty()) {
        return -1;
    }
    
    // 검색 범위 최적화 (frenet_converter와 유사한 로직)
    int search_range = 20;  // ±20개 waypoint 검색
    int trajectory_size = static_cast<int>(global_waypoints.size());
    
    // 트랙 끝부분에서는 wrap-around 검색 수행
    bool check_wrap_around = (last_closest_index_ > trajectory_size * 0.8) || 
                            (last_closest_index_ < trajectory_size * 0.2);
    
    double min_s_diff = std::numeric_limits<double>::max();
    int closest_index = last_closest_index_;
    
    if (check_wrap_around) {
        // 전체 검색 (트랙 경계 근처에서만)
        for (int i = 0; i < trajectory_size; ++i) {
            double s_diff = std::abs(global_waypoints[i].s_m - s_coordinate);
            if (s_diff < min_s_diff) {
                min_s_diff = s_diff;
                closest_index = i;
            }
        }
        RCLCPP_DEBUG(this->get_logger(), "Wrap-around search used, found closest: %d", closest_index);
    } else {
        // 지역 검색 (대부분의 경우)
        int start_idx = std::max(0, last_closest_index_ - search_range);
        int end_idx = std::min(trajectory_size - 1, last_closest_index_ + search_range);
        
        for (int i = start_idx; i <= end_idx; ++i) {
            double s_diff = std::abs(global_waypoints[i].s_m - s_coordinate);
            if (s_diff < min_s_diff) {
                min_s_diff = s_diff;
                closest_index = i;
            }
        }
        RCLCPP_DEBUG(this->get_logger(), 
            "Local search range: [%d, %d], Found closest: %d", 
            start_idx, end_idx, closest_index);
    }
    
    // 지역 검색 결과가 너무 멀면(순간이동/리스폰으로 s가 점프) 전체 재탐색
    if (!check_wrap_around && min_s_diff > 3.0) {
        for (int i = 0; i < trajectory_size; ++i) {
            double s_diff = std::abs(global_waypoints[i].s_m - s_coordinate);
            if (s_diff < min_s_diff) {
                min_s_diff = s_diff;
                closest_index = i;
            }
        }
        RCLCPP_WARN(this->get_logger(),
            "s jump detected, global re-search -> closest index %d", closest_index);
    }

    // 다음 검색을 위해 저장
    last_closest_index_ = closest_index;

    return closest_index;
}

void LocalTrajectoryNode::Run() {
    RCLCPP_INFO_THROTTLE(this->get_logger(), *get_clock(), 2000,
                        "RUN: has_global=%s, has_frenet=%s", 
                        has_global_waypoints_ ? "true" : "false",
                        has_frenet_odom_ ? "true" : "false");

    // Check if we have all required data
    if (!has_global_waypoints_) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *get_clock(), 1000, 
                            "Waiting for global waypoints...");
        return;
    }
    
    if (!has_frenet_odom_) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *get_clock(), 1000, 
                            "Waiting for Frenet odometry...");
        return;
    }

    // 실행 확인 로그
    RCLCPP_INFO_THROTTLE(this->get_logger(), *get_clock(), 2000,
                         "Publishing local waypoints...");
    
    // Get current data with thread safety
    std::vector<ae_hyu_msgs::msg::Wpnt> global_waypoints_copy;
    nav_msgs::msg::Odometry frenet_odom_copy;
    
    {
        std::lock_guard<std::mutex> lock1(mutex_global_waypoints_);
        global_waypoints_copy = global_waypoints_;
    }
    
    {
        std::lock_guard<std::mutex> lock2(mutex_frenet_odom_);
        frenet_odom_copy = current_frenet_odom_;
    }
    
    // Process with copied data (no more locks needed)
    PublishLocalWaypoints(global_waypoints_copy, frenet_odom_copy);
}

void LocalTrajectoryNode::PublishLocalWaypoints(const std::vector<ae_hyu_msgs::msg::Wpnt>& global_waypoints_copy,
                                              const nav_msgs::msg::Odometry& frenet_odom_copy) {
    if (global_waypoints_copy.empty()) {
        return;
    }
    
    // Get current s coordinate from Frenet odometry
    double current_s = frenet_odom_copy.pose.pose.position.x;
    
    // Find closest waypoint index based on s coordinate
    int start_idx = FindClosestWaypointIndex(current_s, global_waypoints_copy);
    if (start_idx < 0) {
        RCLCPP_WARN(this->get_logger(), "Could not find closest waypoint");
        return;
    }
    
    // Create local waypoints message
    ae_hyu_msgs::msg::WpntArray local_waypoints;
    local_waypoints.header.stamp = this->now();
    local_waypoints.header.frame_id = "map";
    
    int trajectory_size = static_cast<int>(global_waypoints_copy.size());
    
    // Pre-allocate vector for better performance
    local_waypoints.wpnts.reserve(num_local_waypoints_);
    
    // Extract waypoints (with wrap-around for circular tracks)
    for (int i = 0; i < num_local_waypoints_; ++i) {
        int wp_idx = (start_idx + i) % trajectory_size;
        const auto& source_waypoint = global_waypoints_copy[wp_idx];

        // Create new waypoint with sequential ID
        ae_hyu_msgs::msg::Wpnt waypoint = source_waypoint;
        waypoint.id = static_cast<int32_t>(i);  // Sequential ID starting from 0

        local_waypoints.wpnts.push_back(waypoint);
    }

    // 회피 궤적이 신선하면 겹치는 s 구간을 회피 궤적으로 치환 (splice)
    std::vector<ae_hyu_msgs::msg::Wpnt> avoidance_copy;
    {
        std::lock_guard<std::mutex> lock(mutex_avoidance_);
        if (!avoidance_waypoints_.empty() &&
            (this->now() - avoidance_stamp_).seconds() < avoidance_timeout_sec_) {
            avoidance_copy = avoidance_waypoints_;
        }
    }
    if (!avoidance_copy.empty()) {
        double track_length = global_waypoints_copy.back().s_m +
            (global_waypoints_copy.back().s_m - global_waypoints_copy[trajectory_size - 2].s_m);
        SpliceAvoidanceWaypoints(local_waypoints.wpnts, avoidance_copy, track_length);
    }

    // Publish local waypoints
    p_local_waypoints_->publish(local_waypoints);
    
    RCLCPP_DEBUG(this->get_logger(), 
        "Published %d local waypoints starting from index %d (s=%.2f)", 
        num_local_waypoints_, start_idx, current_s);
}

void LocalTrajectoryNode::SpliceAvoidanceWaypoints(
    std::vector<ae_hyu_msgs::msg::Wpnt>& local_wpnts,
    const std::vector<ae_hyu_msgs::msg::Wpnt>& avoidance_wpnts,
    double track_length) {
    if (avoidance_wpnts.size() < 2) {
        return;
    }
    // 회피 궤적의 s 구간 [av_start, av_end] (wrap 가능)
    double av_start = avoidance_wpnts.front().s_m;
    double av_span = Mod(avoidance_wpnts.back().s_m - av_start, track_length);
    // 회피 궤적 점 간격 (spliner는 0.1m 균일 샘플)
    double av_spacing = av_span / static_cast<double>(avoidance_wpnts.size() - 1);
    if (av_spacing <= 0.0) {
        return;
    }

    int replaced = 0;
    for (auto& wpnt : local_wpnts) {
        double offset = Mod(wpnt.s_m - av_start, track_length);
        if (offset > av_span) {
            continue;  // 회피 구간 밖 -> global raceline 유지
        }
        int av_idx = std::min(
            static_cast<int>(offset / av_spacing + 0.5),
            static_cast<int>(avoidance_wpnts.size()) - 1);
        int32_t id_keep = wpnt.id;
        wpnt = avoidance_wpnts[av_idx];
        wpnt.id = id_keep;  // 로컬 윈도우 내 순차 ID 유지
        ++replaced;
    }

    if (replaced > 0) {
        RCLCPP_INFO_THROTTLE(this->get_logger(), *get_clock(), 2000,
            "Spliced %d/%zu local waypoints with avoidance trajectory",
            replaced, local_wpnts.size());
    }
}

int main(int argc, char **argv) {
    std::string node_name = "local_trajectory_node";

    // Initialize node
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LocalTrajectoryNode>(node_name));
    rclcpp::shutdown();
    return 0;
}
