#include "planning_pkg/frenet_frame_node.hpp"

#include "planning_pkg/pitwall.hpp"

FrenetFrameNode::FrenetFrameNode(const std::string &node_name, const rclcpp::NodeOptions &options)
    : Node(node_name, options) {

    // QoS init - Use RELIABLE QoS to match simulator's /ego_racecar/odom
    auto qos_profile = rclcpp::QoS(rclcpp::KeepLast(10)).reliable();
    pitwall::Init(this);
    
    // Parameters - declare and get values
    this->declare_parameter("frenet_frame/loop_rate_hz", 40.0);
    this->declare_parameter("frenet_frame/odom_mode", "sim_gt");  // "sim_gt", "sim_pf", "real"
    
    loop_rate_hz_ = this->get_parameter("frenet_frame/loop_rate_hz").as_double();
    odom_mode_ = this->get_parameter("frenet_frame/odom_mode").as_string();
    
    RCLCPP_INFO(this->get_logger(), "loop_rate_hz: %f", loop_rate_hz_);
    RCLCPP_INFO(this->get_logger(), "odom_mode: %s", odom_mode_.c_str());
    
    // Subscriber init - different topics based on odom_mode
    if (odom_mode_ == "sim_gt") {
        // GT Simulation mode: Ground Truth odometry from simulator
        s_odom_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/ego_racecar/odom", qos_profile, 
            std::bind(&FrenetFrameNode::CallbackOdom, this, std::placeholders::_1));
        RCLCPP_INFO(this->get_logger(), "Subscribing to GT odom: /ego_racecar/odom");
    } else if (odom_mode_ == "sim_pf") {
        // PF Simulation mode: Particle Filter odometry in simulation
        s_odom_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/pf/pose/odom", qos_profile, 
            std::bind(&FrenetFrameNode::CallbackOdom, this, std::placeholders::_1));
        RCLCPP_INFO(this->get_logger(), "Subscribing to PF odom (sim): /pf/pose/odom");
    } else if (odom_mode_ == "real") {
        // PF Real mode: Particle Filter odometry in real world
        s_odom_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/pf/pose/odom", qos_profile, 
            std::bind(&FrenetFrameNode::CallbackOdom, this, std::placeholders::_1));
        RCLCPP_INFO(this->get_logger(), "Subscribing to PF odom (real): /pf/pose/odom");
    } else {
        RCLCPP_ERROR(this->get_logger(), "Invalid odom_mode: %s. Use 'sim_gt', 'sim_pf', or 'real'", odom_mode_.c_str());
        return;
    }
    
    // Subscribe to global waypoints
    s_global_waypoints_ = this->create_subscription<f1tenth_planning_msgs::msg::WpntArray>(
        "/global_waypoints", qos_profile,
        std::bind(&FrenetFrameNode::CallbackGlobalWaypoints, this, std::placeholders::_1));
    
    // Publisher init
    p_frenet_odom_ = this->create_publisher<nav_msgs::msg::Odometry>(
        "/car_state/frenet/odom", qos_profile);
    
    // Timer init
    t_process_ = this->create_wall_timer(
        std::chrono::duration<double>(1.0 / loop_rate_hz_),
        [this]() { this->Run(); });
        
    RCLCPP_INFO(this->get_logger(), "FrenetFrameNode initialized successfully");
}

FrenetFrameNode::~FrenetFrameNode() {}

void FrenetFrameNode::CallbackOdom(const nav_msgs::msg::Odometry::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(mutex_odom_);
    current_odom_ = *msg;
    has_odom_ = true;
    
    RCLCPP_DEBUG(this->get_logger(), "Received odom: x=%.3f, y=%.3f", 
                msg->pose.pose.position.x, msg->pose.pose.position.y);
}

void FrenetFrameNode::CallbackGlobalWaypoints(const f1tenth_planning_msgs::msg::WpntArray::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(mutex_trajectory_);
    
    // Convert WpntArray to internal trajectory format
    global_trajectory_.clear();
    global_trajectory_.reserve(msg->wpnts.size());
    
    for (const auto& wpnt : msg->wpnts) {
        std::vector<double> row = {
            wpnt.s_m,           // s_m
            wpnt.x_m,           // x_m
            wpnt.y_m,           // y_m
            wpnt.d_right,       // d_right
            wpnt.d_left,        // d_left
            wpnt.psi_rad,       // psi_rad
            wpnt.kappa_radpm,   // kappa_radpm
            wpnt.vx_mps,        // vx_mps
            wpnt.ax_mps2        // ax_mps2
        };
        global_trajectory_.push_back(row);
    }
    
    has_trajectory_ = true;
    
    RCLCPP_INFO_ONCE(this->get_logger(), "Received global trajectory: %zu waypoints", global_trajectory_.size());
}

void FrenetFrameNode::Run() {
    // Check if we have all required data
    if (!has_odom_) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *get_clock(), 1000, 
                            "Waiting for odom (%s)...", odom_mode_.c_str());
        return;
    }
    
    if (!has_trajectory_) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *get_clock(), 1000, 
                            "Waiting for global trajectory...");
        return;
    }
    
    // Get current data with thread safety - copy once
    nav_msgs::msg::Odometry current_odom_copy;
    std::vector<std::vector<double>> trajectory_copy;
    
    {
        std::lock_guard<std::mutex> lock(mutex_odom_);
        current_odom_copy = current_odom_;
    }
    
    {
        std::lock_guard<std::mutex> lock(mutex_trajectory_);
        trajectory_copy = global_trajectory_;
    }
    
    // Process Frenet conversion with copied data (no more locks needed)
    ProcessFrenetConversion(current_odom_copy, trajectory_copy);
}

void FrenetFrameNode::ProcessFrenetConversion(const nav_msgs::msg::Odometry& odom_copy, 
                                                 const std::vector<std::vector<double>>& trajectory_copy) {
    // Extract position and velocities from odom
    double vehicle_x = odom_copy.pose.pose.position.x;
    double vehicle_y = odom_copy.pose.pose.position.y;
    double vx = odom_copy.twist.twist.linear.x;
    double vy = odom_copy.twist.twist.linear.y;
    
    // Extract vehicle heading from quaternion
    auto q = odom_copy.pose.pose.orientation;
    double vehicle_theta = atan2(2.0 * (q.w * q.z + q.x * q.y), 
                                1.0 - 2.0 * (q.y * q.y + q.z * q.z));

    // Convert position to Frenet coordinates
    FrenetCoordinates frenet_coords = CartesianToFrenet(vehicle_x, vehicle_y, trajectory_copy);
    
    // Convert velocities to Frenet coordinates
    auto frenet_vel = CartesianVelocityToFrenet(vx, vy, vehicle_theta, trajectory_copy);
    double vs = frenet_vel.first;   // longitudinal velocity
    double vd = frenet_vel.second;  // lateral velocity
    
    // Publish Frenet coordinates with velocities
    PublishFrenetOdometry(odom_copy.header.stamp, frenet_coords, vs, vd);
    pitwall::Log("lat_err", frenet_coords.d);
}

int FrenetFrameNode::FindClosestWaypoint(double vehicle_x, double vehicle_y, 
                                             const std::vector<std::vector<double>>& trajectory) {
    if (trajectory.empty()) return -1;
    
    // For circular tracks, expand search range when near boundaries
    int search_range = 20;  // ±20개 waypoint 검색
    int trajectory_size = (int)trajectory.size();
    
    // If we're near the end of trajectory, also check the beginning
    bool check_wrap_around = (last_closest_index_ > trajectory_size * 0.8) || 
                            (last_closest_index_ < trajectory_size * 0.2);
    
    double min_distance_sq = std::numeric_limits<double>::max();
    int closest_index = last_closest_index_;
    
    if (check_wrap_around) {
        // Search entire trajectory when near boundaries to handle wrap-around
        for (int i = 0; i < trajectory_size; ++i) {
            double wp_x = trajectory[i][1];  // x_m
            double wp_y = trajectory[i][2];  // y_m
            
            double dx = vehicle_x - wp_x;
            double dy = vehicle_y - wp_y;
            double distance_sq = dx * dx + dy * dy;
            
            if (distance_sq < min_distance_sq) {
                min_distance_sq = distance_sq;
                closest_index = i;
            }
        }
        RCLCPP_DEBUG(this->get_logger(), "Wrap-around search used, found closest: %d", closest_index);
    } else {
        // Normal local search
        int start_idx = std::max(0, last_closest_index_ - search_range);
        int end_idx = std::min(trajectory_size - 1, last_closest_index_ + search_range);
        
        for (int i = start_idx; i <= end_idx; ++i) {
            double wp_x = trajectory[i][1];  // x_m
            double wp_y = trajectory[i][2];  // y_m
            
            double dx = vehicle_x - wp_x;
            double dy = vehicle_y - wp_y;
            double distance_sq = dx * dx + dy * dy;
            
            if (distance_sq < min_distance_sq) {
                min_distance_sq = distance_sq;
                closest_index = i;
            }
        }
        RCLCPP_DEBUG(this->get_logger(), 
            "Local search range: [%d, %d], Found closest: %d", 
            start_idx, end_idx, closest_index);
    }
    
    // 지역 검색 결과가 너무 멀면(순간이동/리스폰) 전체 재탐색
    // — RViz 2D Pose Estimate로 차량을 옮기면 지역 검색이 실패하므로 필수
    if (!check_wrap_around && min_distance_sq > 3.0 * 3.0) {
        for (int i = 0; i < trajectory_size; ++i) {
            double dx = vehicle_x - trajectory[i][1];
            double dy = vehicle_y - trajectory[i][2];
            double distance_sq = dx * dx + dy * dy;
            if (distance_sq < min_distance_sq) {
                min_distance_sq = distance_sq;
                closest_index = i;
            }
        }
        RCLCPP_WARN(this->get_logger(),
            "Pose jump detected, global re-search -> closest index %d", closest_index);
        pitwall::Event("pose jump — 전체 재탐색");
    }

    // Save for next search
    last_closest_index_ = closest_index;

    return closest_index;
}

FrenetFrameNode::FrenetCoordinates FrenetFrameNode::CartesianToFrenet(double x, double y, 
                                                                           const std::vector<std::vector<double>>& trajectory) {
    FrenetCoordinates frenet;
    frenet.s = 0.0;
    frenet.d = 0.0;
    
    if (trajectory.empty()) {
        RCLCPP_WARN(this->get_logger(), "Global trajectory is empty, cannot convert to Frenet");
        return frenet;
    }
    
    // Find closest waypoint
    int closest_idx = FindClosestWaypoint(x, y, trajectory);
    
    if (closest_idx < 0) {
        RCLCPP_WARN(this->get_logger(), "Could not find closest waypoint");
        return frenet;
    }
    
    // Current waypoint data
    double wp_x = trajectory[closest_idx][1];    // x_m
    double wp_y = trajectory[closest_idx][2];    // y_m
    double wp_s = trajectory[closest_idx][0];    // s_m
    double wp_psi = trajectory[closest_idx][5];  // psi_rad
    
    // Vector from waypoint to vehicle
    double dx = x - wp_x;
    double dy = y - wp_y;
    
    // Project onto track tangent to get s offset
    double ds = dx * cos(wp_psi) + dy * sin(wp_psi);
    
    // Project onto track normal to get d offset
    // Convention: positive d = left side of track (when looking in direction of travel)
    double dd = -dx * sin(wp_psi) + dy * cos(wp_psi);
    
    // Final Frenet coordinates
    frenet.s = wp_s + ds;
    frenet.d = dd;
    
    // Handle lap transitions for circular tracks
    if (!trajectory.empty()) {
        double max_s = trajectory.back()[0];
        if (frenet.s > max_s + 0.1) {
            frenet.s = frenet.s - max_s;
        }
    }
    
    // Detailed debugging output
    RCLCPP_INFO_THROTTLE(this->get_logger(), *get_clock(), 1000,
        "[Frenet] XY(%.2f,%.2f) => s:%.2f d:%.2f", x, y, frenet.s, frenet.d);

    return frenet;
}

std::pair<double, double> FrenetFrameNode::CartesianVelocityToFrenet(double vx, double vy, double vehicle_theta,
                                                                        const std::vector<std::vector<double>>& trajectory) {
    // base_link 기준 속도를 map 기준 속도로 변환
    // vx, vy: base_link 기준 (x: 전방, y: 좌측)
    // vehicle_theta: map 기준 차량 헤딩 (x축 기준 CCW)
    if (trajectory.empty()) {
        RCLCPP_WARN(this->get_logger(), "Global trajectory is empty, cannot convert velocity to Frenet");
        return {0.0, 0.0};
    }

    int closest_idx = last_closest_index_;
    if (closest_idx < 0 || closest_idx >= (int)trajectory.size()) {
        RCLCPP_WARN(this->get_logger(), "Invalid closest waypoint index for velocity conversion");
        return {0.0, 0.0};
    }

    // 트랙 방향(psi, map 기준)
    double track_psi = trajectory[closest_idx][5];

    // 1. base_link -> map 변환
    double vx_map = vx * cos(vehicle_theta) - vy * sin(vehicle_theta);
    double vy_map = vx * sin(vehicle_theta) + vy * cos(vehicle_theta);

    // 2. map -> frenet 변환
    // vs: 트랙 종방향(Reference Line 방향) 속도
    // vd: 트랙 좌측이 +인 횡방향 속도
    double vs = vx_map * cos(track_psi) + vy_map * sin(track_psi);
    double vd = -vx_map * sin(track_psi) + vy_map * cos(track_psi);

    RCLCPP_DEBUG(this->get_logger(),
        "Velocity conversion: BaseLink(vx=%.3f, vy=%.3f, theta=%.3f) -> Map(vx=%.3f, vy=%.3f) -> Track_psi=%.3f -> Frenet(vs=%.3f, vd=%.3f)",
        vx, vy, vehicle_theta, vx_map, vy_map, track_psi, vs, vd);

    return {vs, vd};
}

void FrenetFrameNode::PublishFrenetOdometry(const builtin_interfaces::msg::Time& timestamp,
                                               const FrenetCoordinates& frenet_coords,
                                               double vs, double vd) {
    // Create new Frenet odometry message
    nav_msgs::msg::Odometry frenet_odom;
    
    // Header
    frenet_odom.header.stamp = timestamp;
    frenet_odom.header.frame_id = "frenet";
    frenet_odom.child_frame_id = "base_link_frenet";
    
    // Pose: s, d coordinates
    frenet_odom.pose.pose.position.x = frenet_coords.s;  // s coordinate
    frenet_odom.pose.pose.position.y = frenet_coords.d;  // d coordinate
    frenet_odom.pose.pose.position.z = 0.0;
    
    // Orientation (keep as identity quaternion for Frenet frame)
    frenet_odom.pose.pose.orientation.x = 0.0;
    frenet_odom.pose.pose.orientation.y = 0.0;
    frenet_odom.pose.pose.orientation.z = 0.0;
    frenet_odom.pose.pose.orientation.w = 1.0;
    
    // Twist: vs, vd velocities
    frenet_odom.twist.twist.linear.x = vs;  // longitudinal velocity
    frenet_odom.twist.twist.linear.y = vd;  // lateral velocity
    frenet_odom.twist.twist.linear.z = 0.0;
    frenet_odom.twist.twist.angular.x = 0.0;
    frenet_odom.twist.twist.angular.y = 0.0;
    frenet_odom.twist.twist.angular.z = 0.0;
    
    // Covariances (set to zero)
    for (int i = 0; i < 36; ++i) {
        frenet_odom.pose.covariance[i] = 0.0;
        frenet_odom.twist.covariance[i] = 0.0;
    }
    
    // Publish the Frenet odometry message
    p_frenet_odom_->publish(frenet_odom);
    
    RCLCPP_DEBUG(this->get_logger(), 
        "Published Frenet odom: s=%.3f, d=%.3f, vs=%.3f, vd=%.3f", 
        frenet_coords.s, frenet_coords.d, vs, vd);
}

int main(int argc, char **argv) {
    std::string node_name = "frenet_frame_node";

    // Initialize node
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<FrenetFrameNode>(node_name));
    rclcpp::shutdown();
    return 0;
}
