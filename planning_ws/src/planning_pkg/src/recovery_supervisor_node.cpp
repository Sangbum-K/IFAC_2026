#include "planning_pkg/recovery_supervisor_node.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

#include "planning_pkg/pitwall.hpp"

namespace {
double Mod(double a, double m) {
    double r = std::fmod(a, m);
    return (r < 0.0) ? r + m : r;
}
double WrapToPi(double angle) {
    while (angle > M_PI) angle -= 2.0 * M_PI;
    while (angle < -M_PI) angle += 2.0 * M_PI;
    return angle;
}
bool NormalizedYaw(const geometry_msgs::msg::Quaternion &q, double *yaw) {
    const double norm_sq =
        q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w;
    if (!std::isfinite(norm_sq) || norm_sq < 1.0e-12 || yaw == nullptr) {
        return false;
    }
    const double inv_norm = 1.0 / std::sqrt(norm_sq);
    const double x = q.x * inv_norm;
    const double y = q.y * inv_norm;
    const double z = q.z * inv_norm;
    const double w = q.w * inv_norm;
    *yaw = std::atan2(
        2.0 * (w * z + x * y),
        1.0 - 2.0 * (y * y + z * z));
    return std::isfinite(*yaw);
}
}  // namespace

const char *RecoverySupervisorNode::StateName(State s) {
    switch (s) {
        case State::WAITING_FOR_INPUTS: return "WAITING_FOR_INPUTS";
        case State::DRIVING: return "DRIVING";
        case State::STUCK_DETECTED: return "STUCK_DETECTED";
        case State::COLLISION_STOP: return "COLLISION_STOP";
        case State::RECOVER_REVERSE: return "RECOVER_REVERSE";
        case State::RECOVER_STOP: return "RECOVER_STOP";
        case State::RECOVER_ALIGN: return "RECOVER_ALIGN";
        case State::STOP: return "STOP";
    }
    return "UNKNOWN";
}

RecoverySupervisorNode::RecoverySupervisorNode(
    const std::string &node_name, const rclcpp::NodeOptions &options)
    : Node(node_name, options) {

    auto qos = rclcpp::QoS(rclcpp::KeepLast(10)).reliable();

    // 단독 실행 시에는 명시적 opt-in이 필요하다. planner_stack의 안전
    // 프로파일은 이 값을 true로 넘기고 control과 paired lease로 연결한다.
    // false면 구독/발행/타이머를 만들지 않는다.
    this->declare_parameter("recovery/enabled", false);
    if (!this->get_parameter("recovery/enabled").as_bool()) {
        RCLCPP_WARN(this->get_logger(),
            "recovery/enabled=false — 리커버리 배선 전면 차단(구독/발행/타이머 없음). "
            "다시 켤 때: ros2 launch ... recovery:=true");
        return;
    }

    pitwall::Init(this);

    this->declare_parameter("recovery/loop_rate_hz", 20.0);
    this->declare_parameter("recovery/stuck_timeout_sec", 1.0);
    this->declare_parameter("recovery/stuck_min_progress_m", 0.10);
    this->declare_parameter("recovery/min_forward_cmd_mps", 0.2);
    this->declare_parameter("recovery/boundary_stop_margin_m", 0.15);
    this->declare_parameter("recovery/boundary_persist_sec", 0.2);
    this->declare_parameter("recovery/vehicle_width_m", 0.30);
    this->declare_parameter("recovery/vehicle_length_m", 0.55);
    this->declare_parameter("recovery/boundary_reaction_time_sec", 0.20);
    this->declare_parameter("recovery/boundary_braking_decel_mps2", 4.0);
    this->declare_parameter("recovery/require_boundary_horizon", true);
    this->declare_parameter("recovery/rear_check_dist_m", 1.5);
    this->declare_parameter("recovery/front_check_dist_m", 1.5);
    this->declare_parameter("recovery/allow_reverse", false);
    this->declare_parameter("recovery/rear_coverage_confirmed", false);
    this->declare_parameter("recovery/recovery_reverse_speed_mps", 0.5);
    this->declare_parameter("recovery/recovery_max_reverse_distance_m", 0.5);
    this->declare_parameter("recovery/recovery_timeout_sec", 4.0);
    this->declare_parameter("recovery/recovery_max_attempts", 2);
    this->declare_parameter("recovery/align_tolerance_rad", 0.35);
    this->declare_parameter("recovery/align_speed_mps", 0.5);
    this->declare_parameter("recovery/odom_timeout_sec", 0.5);
    this->declare_parameter("recovery/perception_timeout_sec", 0.5);
    this->declare_parameter("recovery/local_wpnts_timeout_sec", 0.5);
    this->declare_parameter("recovery/source_stamp_future_tolerance_sec", 0.1);
    this->declare_parameter("recovery/require_source_stamps", true);
    this->declare_parameter("recovery/readiness_dwell_sec", 0.2);
    this->declare_parameter("recovery/stopped_speed_threshold_mps", 0.05);
    this->declare_parameter("recovery/stopped_dwell_sec", 0.2);
    this->declare_parameter("recovery/pose_odom_topic", std::string("/pf/pose/odom"));

    loop_rate_hz_ = this->get_parameter("recovery/loop_rate_hz").as_double();
    stuck_timeout_sec_ = this->get_parameter("recovery/stuck_timeout_sec").as_double();
    stuck_min_progress_m_ = this->get_parameter("recovery/stuck_min_progress_m").as_double();
    min_forward_cmd_mps_ = this->get_parameter("recovery/min_forward_cmd_mps").as_double();
    boundary_stop_margin_m_ = this->get_parameter("recovery/boundary_stop_margin_m").as_double();
    boundary_persist_sec_ = this->get_parameter("recovery/boundary_persist_sec").as_double();
    vehicle_width_m_ = this->get_parameter("recovery/vehicle_width_m").as_double();
    vehicle_length_m_ = this->get_parameter("recovery/vehicle_length_m").as_double();
    boundary_reaction_time_sec_ =
        this->get_parameter("recovery/boundary_reaction_time_sec").as_double();
    boundary_braking_decel_mps2_ =
        this->get_parameter("recovery/boundary_braking_decel_mps2").as_double();
    require_boundary_horizon_ =
        this->get_parameter("recovery/require_boundary_horizon").as_bool();
    rear_check_dist_m_ = this->get_parameter("recovery/rear_check_dist_m").as_double();
    front_check_dist_m_ = this->get_parameter("recovery/front_check_dist_m").as_double();
    allow_reverse_ = this->get_parameter("recovery/allow_reverse").as_bool();
    rear_coverage_confirmed_ =
        this->get_parameter("recovery/rear_coverage_confirmed").as_bool();
    recovery_reverse_speed_mps_ =
        this->get_parameter("recovery/recovery_reverse_speed_mps").as_double();
    recovery_max_reverse_distance_m_ =
        this->get_parameter("recovery/recovery_max_reverse_distance_m").as_double();
    recovery_timeout_sec_ = this->get_parameter("recovery/recovery_timeout_sec").as_double();
    recovery_max_attempts_ =
        static_cast<int>(this->get_parameter("recovery/recovery_max_attempts").as_int());
    align_tolerance_rad_ = this->get_parameter("recovery/align_tolerance_rad").as_double();
    align_speed_mps_ = this->get_parameter("recovery/align_speed_mps").as_double();
    odom_timeout_sec_ = this->get_parameter("recovery/odom_timeout_sec").as_double();
    perception_timeout_sec_ =
        this->get_parameter("recovery/perception_timeout_sec").as_double();
    local_wpnts_timeout_sec_ =
        this->get_parameter("recovery/local_wpnts_timeout_sec").as_double();
    source_stamp_future_tolerance_sec_ =
        this->get_parameter("recovery/source_stamp_future_tolerance_sec").as_double();
    require_source_stamps_ =
        this->get_parameter("recovery/require_source_stamps").as_bool();
    readiness_dwell_sec_ =
        this->get_parameter("recovery/readiness_dwell_sec").as_double();
    stopped_speed_threshold_mps_ =
        this->get_parameter("recovery/stopped_speed_threshold_mps").as_double();
    stopped_dwell_sec_ =
        this->get_parameter("recovery/stopped_dwell_sec").as_double();
    const std::string pose_topic =
        this->get_parameter("recovery/pose_odom_topic").as_string();

    const auto require_range = [](double value, double minimum, double maximum,
        const char *name) {
        if (!std::isfinite(value) || value < minimum || value > maximum) {
            throw std::invalid_argument(
                std::string(name) + " must be finite and in [" +
                std::to_string(minimum) + ", " + std::to_string(maximum) + "]");
        }
    };
    require_range(loop_rate_hz_, 1.0, 200.0, "recovery/loop_rate_hz");
    require_range(stuck_timeout_sec_, 0.05, 60.0, "recovery/stuck_timeout_sec");
    require_range(
        stuck_min_progress_m_, 0.0, 10.0, "recovery/stuck_min_progress_m");
    require_range(
        min_forward_cmd_mps_, 0.0, 20.0, "recovery/min_forward_cmd_mps");
    require_range(
        boundary_stop_margin_m_, 0.0, 2.0, "recovery/boundary_stop_margin_m");
    require_range(
        boundary_persist_sec_, 0.0, 5.0, "recovery/boundary_persist_sec");
    require_range(vehicle_width_m_, 0.05, 5.0, "recovery/vehicle_width_m");
    require_range(vehicle_length_m_, 0.05, 10.0, "recovery/vehicle_length_m");
    require_range(
        boundary_reaction_time_sec_, 0.0, 5.0,
        "recovery/boundary_reaction_time_sec");
    require_range(
        boundary_braking_decel_mps2_, 0.1, 30.0,
        "recovery/boundary_braking_decel_mps2");
    require_range(rear_check_dist_m_, 0.0, 30.0, "recovery/rear_check_dist_m");
    require_range(
        front_check_dist_m_, 0.0, 30.0, "recovery/front_check_dist_m");
    require_range(
        recovery_reverse_speed_mps_, 0.0, 2.0,
        "recovery/recovery_reverse_speed_mps");
    require_range(
        recovery_max_reverse_distance_m_, 0.0, 10.0,
        "recovery/recovery_max_reverse_distance_m");
    require_range(
        recovery_timeout_sec_, 0.1, 60.0, "recovery/recovery_timeout_sec");
    if (recovery_max_attempts_ < 0 || recovery_max_attempts_ > 10) {
        throw std::invalid_argument(
            "recovery/recovery_max_attempts must be in [0, 10]");
    }
    require_range(
        align_tolerance_rad_, 0.0, M_PI, "recovery/align_tolerance_rad");
    require_range(align_speed_mps_, 0.0, 2.0, "recovery/align_speed_mps");
    require_range(odom_timeout_sec_, 0.01, 5.0, "recovery/odom_timeout_sec");
    require_range(
        perception_timeout_sec_, 0.01, 5.0,
        "recovery/perception_timeout_sec");
    require_range(
        local_wpnts_timeout_sec_, 0.01, 5.0,
        "recovery/local_wpnts_timeout_sec");
    require_range(
        source_stamp_future_tolerance_sec_, 0.0, 1.0,
        "recovery/source_stamp_future_tolerance_sec");
    require_range(
        readiness_dwell_sec_, 0.0, 10.0, "recovery/readiness_dwell_sec");
    require_range(
        stopped_speed_threshold_mps_, 0.0, 1.0,
        "recovery/stopped_speed_threshold_mps");
    require_range(stopped_dwell_sec_, 0.0, 10.0, "recovery/stopped_dwell_sec");
    if (pose_topic.empty()) {
        throw std::invalid_argument("recovery/pose_odom_topic must not be empty");
    }

    s_frenet_odom_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/car_state/frenet/odom", qos,
        std::bind(&RecoverySupervisorNode::CallbackFrenetOdom, this, std::placeholders::_1));
    s_pose_odom_ = this->create_subscription<nav_msgs::msg::Odometry>(
        pose_topic, qos,
        std::bind(&RecoverySupervisorNode::CallbackPoseOdom, this, std::placeholders::_1));
    s_global_wpnts_ = this->create_subscription<f1tenth_planning_msgs::msg::WpntArray>(
        "/global_waypoints", qos,
        std::bind(&RecoverySupervisorNode::CallbackGlobalWaypoints, this, std::placeholders::_1));
    s_local_wpnts_ = this->create_subscription<f1tenth_planning_msgs::msg::WpntArray>(
        "/local_waypoints", qos,
        std::bind(&RecoverySupervisorNode::CallbackLocalWaypoints, this, std::placeholders::_1));
    // 실차는 브릿지 출력(/perception/obstacles_frenet)으로 override — spliner와 동일 소스
    this->declare_parameter("recovery/obstacle_topic", std::string("/perception/obstacles"));
    const std::string obstacle_topic =
        this->get_parameter("recovery/obstacle_topic").as_string();
    s_obstacles_ = this->create_subscription<f1tenth_planning_msgs::msg::ObstacleArray>(
        obstacle_topic, qos,
        std::bind(&RecoverySupervisorNode::CallbackObstacles, this, std::placeholders::_1));
    s_estop_ = this->create_subscription<std_msgs::msg::Bool>(
        "/safety/emergency_stop", qos,
        std::bind(&RecoverySupervisorNode::CallbackEStop, this, std::placeholders::_1));
    s_reset_ = this->create_subscription<std_msgs::msg::Bool>(
        "/planner/recovery/reset", qos,
        std::bind(&RecoverySupervisorNode::CallbackReset, this, std::placeholders::_1));

    p_state_ = this->create_publisher<std_msgs::msg::String>(
        "/planner/recovery/state", qos);
    p_target_speed_ = this->create_publisher<std_msgs::msg::Float64>(
        "/planner/recovery/target_speed", qos);
    p_marker_ = this->create_publisher<visualization_msgs::msg::Marker>(
        "/planner/recovery/marker", qos);

    t_run_ = this->create_wall_timer(
        std::chrono::duration<double>(1.0 / loop_rate_hz_),
        [this]() { this->Run(); });

    RCLCPP_INFO(this->get_logger(),
        "RecoverySupervisorNode: stuck_timeout=%.1fs reverse=%s rear_coverage=%s "
        "reverse_limit=%.2fm/s max %d attempts",
        stuck_timeout_sec_, allow_reverse_ ? "OPT-IN" : "OFF",
        rear_coverage_confirmed_ ? "confirmed" : "unknown",
        recovery_reverse_speed_mps_, recovery_max_attempts_);
}

void RecoverySupervisorNode::CallbackFrenetOdom(const nav_msgs::msg::Odometry::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(mutex_);
    cur_s_ = msg->pose.pose.position.x;
    cur_d_ = msg->pose.pose.position.y;
    cur_vs_ = msg->twist.twist.linear.x;
    frenet_receipt_stamp_ = this->now();
    frenet_source_stamp_ =
        rclcpp::Time(msg->header.stamp, this->get_clock()->get_clock_type());
    has_frenet_ = true;
}

void RecoverySupervisorNode::CallbackPoseOdom(const nav_msgs::msg::Odometry::SharedPtr msg) {
    double yaw = 0.0;
    if (!NormalizedYaw(msg->pose.pose.orientation, &yaw) ||
        !std::isfinite(msg->twist.twist.linear.x) ||
        !std::isfinite(msg->twist.twist.linear.y)) {
        RCLCPP_ERROR_THROTTLE(
            this->get_logger(), *get_clock(), 1000,
            "Rejecting pose odometry with invalid orientation or velocity");
        return;
    }
    std::lock_guard<std::mutex> lock(mutex_);
    cur_yaw_ = yaw;
    cur_speed_norm_ = std::hypot(
        msg->twist.twist.linear.x, msg->twist.twist.linear.y);
    pose_receipt_stamp_ = this->now();
    pose_source_stamp_ =
        rclcpp::Time(msg->header.stamp, this->get_clock()->get_clock_type());
    has_pose_ = true;
}

void RecoverySupervisorNode::CallbackGlobalWaypoints(const f1tenth_planning_msgs::msg::WpntArray::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(mutex_);
    // 트랙 길이 = 마지막 s + 점 간격. spliner_node / local_trajectory_node /
    // perception 브릿지와 같은 정의를 써야 wrap 근처 s가 어긋나지 않는다.
    if (msg->wpnts.size() < 2) {
        return;
    }
    double spacing = msg->wpnts[1].s_m - msg->wpnts[0].s_m;
    if (!(spacing > 1e-6)) {
        RCLCPP_ERROR_THROTTLE(this->get_logger(), *get_clock(), 5000,
            "global waypoints의 s 간격이 %.6f — 트랙 길이 갱신 불가", spacing);
        return;
    }
    track_len_ = msg->wpnts.back().s_m + spacing;
    has_global_ = std::isfinite(track_len_) && track_len_ > 0.0;
}

void RecoverySupervisorNode::CallbackLocalWaypoints(const f1tenth_planning_msgs::msg::WpntArray::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(mutex_);
    local_wpnts_ = *msg;
    has_local_ = !msg->wpnts.empty();
    local_wpnts_receipt_stamp_ = this->now();
    local_wpnts_source_stamp_ =
        rclcpp::Time(msg->header.stamp, this->get_clock()->get_clock_type());
}

void RecoverySupervisorNode::CallbackObstacles(const f1tenth_planning_msgs::msg::ObstacleArray::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(mutex_);
    obstacles_ = *msg;
    obstacles_receipt_stamp_ = this->now();
    obstacles_source_stamp_ =
        rclcpp::Time(msg->header.stamp, this->get_clock()->get_clock_type());
    has_obstacles_ = true;
}

void RecoverySupervisorNode::CallbackEStop(const std_msgs::msg::Bool::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(mutex_);
    estop_ = msg->data;
}

void RecoverySupervisorNode::CallbackReset(const std_msgs::msg::Bool::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (msg->data) reset_requested_ = true;
}

bool RecoverySupervisorNode::StampFresh(
    const rclcpp::Time &now, const rclcpp::Time &receipt_stamp,
    const rclcpp::Time &source_stamp, double timeout_sec) const {
    const auto valid_age = [&](const rclcpp::Time &stamp) {
        if (stamp.nanoseconds() <= 0) {
            return false;
        }
        const double age = (now - stamp).seconds();
        return std::isfinite(age) &&
               age >= -source_stamp_future_tolerance_sec_ &&
               age <= timeout_sec;
    };

    if (!valid_age(receipt_stamp)) {
        return false;
    }
    if (source_stamp.nanoseconds() <= 0) {
        return !require_source_stamps_;
    }
    return valid_age(source_stamp);
}

bool RecoverySupervisorNode::InputsReady(
    const rclcpp::Time &now, std::string *reason) const {
    const auto not_ready = [&](const char *why) {
        if (reason != nullptr) {
            *reason = why;
        }
        return false;
    };

    if (!has_global_ || !(track_len_ > 0.0) || !std::isfinite(track_len_)) {
        return not_ready("global waypoints missing/invalid");
    }
    if (!has_frenet_) {
        return not_ready("Frenet odometry not received");
    }
    if (!has_pose_) {
        return not_ready("pose odometry not received");
    }
    if (!has_local_ || local_wpnts_.wpnts.empty()) {
        return not_ready("local waypoints not received");
    }
    // 빈 배열도 유효한 "장애물 없음" 관측이지만, 배열 메시지 자체를 한 번도
    // 받지 않은 상태는 자율주행 준비 완료가 아니다.
    if (!has_obstacles_) {
        return not_ready("perception not received");
    }

    if (!StampFresh(
            now, frenet_receipt_stamp_, frenet_source_stamp_, odom_timeout_sec_)) {
        return not_ready("Frenet odometry receipt/source stale");
    }
    if (!StampFresh(
            now, pose_receipt_stamp_, pose_source_stamp_, odom_timeout_sec_)) {
        return not_ready("pose odometry receipt/source stale");
    }
    if (!StampFresh(
            now, local_wpnts_receipt_stamp_, local_wpnts_source_stamp_,
            local_wpnts_timeout_sec_)) {
        return not_ready("local waypoints receipt/source stale");
    }
    if (!StampFresh(
            now, obstacles_receipt_stamp_, obstacles_source_stamp_,
            perception_timeout_sec_)) {
        return not_ready("perception receipt/source stale");
    }

    if (!std::isfinite(cur_s_) || !std::isfinite(cur_d_) ||
        !std::isfinite(cur_vs_) || !std::isfinite(cur_yaw_) ||
        !std::isfinite(cur_speed_norm_)) {
        return not_ready("odometry contains non-finite values");
    }
    for (const auto &w : local_wpnts_.wpnts) {
        if (!std::isfinite(w.s_m) || !std::isfinite(w.d_m) ||
            !std::isfinite(w.x_m) || !std::isfinite(w.y_m) ||
            !std::isfinite(w.d_left) || !std::isfinite(w.d_right) ||
            !std::isfinite(w.psi_rad) || !std::isfinite(w.vx_mps)) {
            return not_ready("local waypoints contain non-finite values");
        }
    }
    for (const auto &obs : obstacles_.obstacles) {
        if (!std::isfinite(obs.s_center) || !std::isfinite(obs.s_start) ||
            !std::isfinite(obs.s_end) || !std::isfinite(obs.d_center) ||
            !std::isfinite(obs.d_left) || !std::isfinite(obs.d_right) ||
            !std::isfinite(obs.vs) || !std::isfinite(obs.vd) ||
            !std::isfinite(obs.size)) {
            return not_ready("perception contains non-finite values");
        }
    }
    return true;
}

bool RecoverySupervisorNode::IsStuck(double now_sec) {
    // 전진 명령(local path 첫 wpnt vx)이 있는데 s 진행량이 임계값 미만인 상태가
    // stuck_timeout_sec 이상 지속되면 stuck.
    double cmd_v = has_local_ ? local_wpnts_.wpnts.front().vx_mps : 0.0;
    if (cmd_v < min_forward_cmd_mps_) {
        stuck_since_sec_ = -1.0;  // 정지 명령 중에는 stuck 아님
        return false;
    }
    if (stuck_since_sec_ < 0.0) {
        stuck_since_sec_ = now_sec;
        stuck_ref_s_ = cur_s_;
        return false;
    }
    double progress = Mod(cur_s_ - stuck_ref_s_, track_len_);
    if (progress > track_len_ / 2.0) progress = 0.0;  // 후진/역방향은 진행으로 안 봄
    if (progress >= stuck_min_progress_m_) {
        stuck_since_sec_ = now_sec;  // 진행함 -> 타이머 리셋
        stuck_ref_s_ = cur_s_;
        return false;
    }
    return (now_sec - stuck_since_sec_) > stuck_timeout_sec_;
}

bool RecoverySupervisorNode::IsCurrentBoundaryViolation() const {
    if (!has_local_ || local_wpnts_.wpnts.empty()) return true;
    // 첫 local wpnt의 남은 경계 여유를 차량 실제 d와 heading으로 보정하고
    // point가 아닌 차량 직사각 footprint 전체를 검사한다.
    const auto &w = local_wpnts_.wpnts.front();
    const double dev = cur_d_ - w.d_m;           // 경로 대비 횡 이탈 (d>0=왼쪽)
    const double heading_error = WrapToPi(cur_yaw_ - w.psi_rad);
    const double half_width = 0.5 * vehicle_width_m_;
    const double half_length = 0.5 * vehicle_length_m_;
    const double footprint_extent =
        half_width * std::abs(std::cos(heading_error)) +
        half_length * std::abs(std::sin(heading_error));
    const double margin_left = w.d_left - dev;
    const double margin_right = w.d_right + dev;
    const double required = footprint_extent + boundary_stop_margin_m_;
    return !std::isfinite(margin_left) || !std::isfinite(margin_right) ||
           std::min(margin_left, margin_right) < required;
}

double RecoverySupervisorNode::StoppingDistance() const {
    const double speed = std::abs(cur_speed_norm_);
    return 0.5 * vehicle_length_m_ +
           speed * boundary_reaction_time_sec_ +
           speed * speed / (2.0 * boundary_braking_decel_mps2_);
}

bool RecoverySupervisorNode::IsBoundaryViolation() const {
    if (IsCurrentBoundaryViolation()) {
        return true;
    }

    const double horizon = StoppingDistance();
    const double tracking_error =
        std::abs(cur_d_ - local_wpnts_.wpnts.front().d_m);
    const double required =
        0.5 * vehicle_width_m_ + boundary_stop_margin_m_ + tracking_error;
    double covered = 0.0;
    const auto &wpnts = local_wpnts_.wpnts;
    for (std::size_t i = 0; i < wpnts.size(); ++i) {
        if (i > 0) {
            const double segment = std::hypot(
                wpnts[i].x_m - wpnts[i - 1].x_m,
                wpnts[i].y_m - wpnts[i - 1].y_m);
            if (!std::isfinite(segment)) {
                return true;
            }
            covered += segment;
        }
        if (covered > horizon) {
            return false;
        }
        if (std::min(wpnts[i].d_left, wpnts[i].d_right) < required) {
            return true;
        }
    }
    // 고속에서 제동거리만큼 경계를 확인할 local horizon이 없다면 안전을
    // 증명할 수 없으므로 fail-closed.
    return require_boundary_horizon_ && covered + 1e-6 < horizon;
}

bool RecoverySupervisorNode::IsRearSafe(double now_sec) const {
    // 현재 perception이 후방을 실제로 커버한다는 명시적 설정과 reverse opt-in이
    // 둘 다 없으면, 전방 전용 센서의 빈 배열을 "후방 안전"으로 해석하지 않는다.
    if (!allow_reverse_ || !rear_coverage_confirmed_ || !has_obstacles_ ||
        !(track_len_ > 0.0)) {
        return false;
    }
    const double receipt_age = now_sec - obstacles_receipt_stamp_.seconds();
    const double source_age = now_sec - obstacles_source_stamp_.seconds();
    if (!std::isfinite(receipt_age) ||
        receipt_age < -source_stamp_future_tolerance_sec_ ||
        receipt_age > perception_timeout_sec_ ||
        (require_source_stamps_ &&
         (!std::isfinite(source_age) ||
          source_age < -source_stamp_future_tolerance_sec_ ||
          source_age > perception_timeout_sec_))) {
        return false;
    }
    const double check_dist = rear_check_dist_m_ + 0.5 * vehicle_length_m_;
    for (const auto &obs : obstacles_.obstacles) {
        const double behind = std::min({
            Mod(cur_s_ - obs.s_start, track_len_),
            Mod(cur_s_ - obs.s_end, track_len_),
            Mod(cur_s_ - obs.s_center, track_len_)});
        if (!std::isfinite(behind) || behind < check_dist) {
            return false;
        }
    }
    return true;
}

bool RecoverySupervisorNode::IsFrontSafe(double now_sec) const {
    if (!has_obstacles_ || !(track_len_ > 0.0)) {
        return false;
    }
    const double receipt_age = now_sec - obstacles_receipt_stamp_.seconds();
    const double source_age = now_sec - obstacles_source_stamp_.seconds();
    if (!std::isfinite(receipt_age) ||
        receipt_age < -source_stamp_future_tolerance_sec_ ||
        receipt_age > perception_timeout_sec_ ||
        (require_source_stamps_ &&
         (!std::isfinite(source_age) ||
          source_age < -source_stamp_future_tolerance_sec_ ||
          source_age > perception_timeout_sec_))) {
        return false;
    }

    const double check_dist = std::max(front_check_dist_m_, StoppingDistance());
    for (const auto &obs : obstacles_.obstacles) {
        const double ahead = std::min({
            Mod(obs.s_start - cur_s_, track_len_),
            Mod(obs.s_end - cur_s_, track_len_),
            Mod(obs.s_center - cur_s_, track_len_)});
        if (!std::isfinite(ahead) || ahead < check_dist) {
            return false;
        }
    }
    return true;
}

bool RecoverySupervisorNode::IsHeadingAligned() const {
    if (!has_local_ || local_wpnts_.wpnts.empty() || !std::isfinite(cur_yaw_)) {
        return false;
    }
    double path_psi = local_wpnts_.wpnts.front().psi_rad;
    return std::abs(WrapToPi(cur_yaw_ - path_psi)) < align_tolerance_rad_;
}

bool RecoverySupervisorNode::IsStoppedForDwell(double now_sec) {
    if (!std::isfinite(cur_speed_norm_) ||
        cur_speed_norm_ > stopped_speed_threshold_mps_) {
        stopped_since_sec_ = -1.0;
        return false;
    }
    if (stopped_since_sec_ < 0.0) {
        stopped_since_sec_ = now_sec;
    }
    return (now_sec - stopped_since_sec_) >= stopped_dwell_sec_;
}

void RecoverySupervisorNode::TransitionTo(State next, const std::string &reason) {
    if (next == state_) return;
    RCLCPP_WARN(this->get_logger(), "[recovery] %s -> %s (%s)",
                StateName(state_), StateName(next), reason.c_str());
    pitwall::Event(std::string("recovery ") + StateName(state_) + " -> " +
                   StateName(next) + " (" + reason + ")");
    state_ = next;
}

void RecoverySupervisorNode::Run() {
    std::lock_guard<std::mutex> lock(mutex_);
    const rclcpp::Time now = this->now();
    const double now_sec = now.seconds();
    std::string readiness_reason;
    const bool inputs_ready = InputsReady(now, &readiness_reason);
    if (has_pose_ && std::isfinite(cur_speed_norm_) &&
        cur_speed_norm_ > stopped_speed_threshold_mps_) {
        stopped_since_sec_ = -1.0;
    }

    // 최우선: E-stop은 어떤 상태에서도 래치 STOP.
    if (estop_) {
        TransitionTo(State::STOP, "E-stop");
    } else if (state_ == State::WAITING_FOR_INPUTS) {
        // 부팅 중 입력 부재를 영구 STOP으로 래치하지 않는다. 대신 입력이 모두
        // 신선하고 실제 차량이 정지한 상태가 dwell 동안 유지될 때까지 0m/s를
        // 계속 발행한다.
        if (!inputs_ready) {
            ready_since_sec_ = -1.0;
            stopped_since_sec_ = -1.0;
            RCLCPP_WARN_THROTTLE(
                this->get_logger(), *get_clock(), 2000,
                "[recovery] waiting for inputs: %s", readiness_reason.c_str());
        } else {
            if (ready_since_sec_ < 0.0) {
                ready_since_sec_ = now_sec;
            }
            const bool ready_dwell =
                (now_sec - ready_since_sec_) >= readiness_dwell_sec_;
            if (ready_dwell && IsStoppedForDwell(now_sec)) {
                recovery_attempts_ = 0;
                stuck_since_sec_ = -1.0;
                boundary_since_sec_ = -1.0;
                TransitionTo(State::DRIVING, "all inputs fresh and vehicle stopped");
            }
        }
    } else if (!inputs_ready && state_ != State::STOP) {
        // 한 번 주행 준비가 된 뒤 source/receipt 중 하나라도 끊기면 자동 재개하지
        // 않는다. 사람이 원인을 해결하고 reset해야 한다.
        TransitionTo(State::STOP, readiness_reason);
    }

    switch (state_) {
        case State::WAITING_FOR_INPUTS:
            break;

        case State::DRIVING:
            // 현재 footprint가 이미 임계 경계를 넘으면 debounce 없이 즉시 정지.
            // 제동거리 앞 경로의 위험은 짧은 persistence로 센서 스파이크를 거른다.
            if (IsCurrentBoundaryViolation()) {
                boundary_since_sec_ = -1.0;
                TransitionTo(State::COLLISION_STOP, "current footprint boundary risk");
            } else if (IsBoundaryViolation()) {
                if (boundary_since_sec_ < 0.0) {
                    boundary_since_sec_ = now_sec;
                }
                if ((now_sec - boundary_since_sec_) >= boundary_persist_sec_) {
                    boundary_since_sec_ = -1.0;
                    TransitionTo(
                        State::COLLISION_STOP,
                        "boundary risk inside speed-dependent stopping horizon");
                }
            } else {
                boundary_since_sec_ = -1.0;
                if (IsStuck(now_sec)) {
                    TransitionTo(State::STUCK_DETECTED, "no progress under forward cmd");
                }
            }
            break;

        case State::COLLISION_STOP:
            // 속도 명령이 아니라 map odom의 실제 평면 속도 norm이 dwell 동안
            // 충분히 낮아야 방향 전환 평가를 시작한다.
            if (IsStoppedForDwell(now_sec)) {
                TransitionTo(State::STUCK_DETECTED, "stopped, evaluate recovery");
            }
            break;

        case State::STUCK_DETECTED:
            if (!IsStoppedForDwell(now_sec)) {
                break;
            }
            if (!allow_reverse_) {
                TransitionTo(State::STOP, "reverse disabled (STOP-only default)");
            } else if (recovery_attempts_ >= recovery_max_attempts_) {
                TransitionTo(State::STOP, "max recovery attempts exceeded");
            } else if (!IsRearSafe(now_sec)) {
                TransitionTo(State::STOP, "rear not safe/unknown, no reverse");
            } else if (IsCurrentBoundaryViolation()) {
                TransitionTo(State::STOP, "insufficient boundary clearance for reverse");
            } else {
                recovery_attempts_++;
                recover_start_sec_ = now_sec;
                recover_start_s_ = cur_s_;
                TransitionTo(State::RECOVER_REVERSE, "rear safe, limited reverse");
            }
            break;

        case State::RECOVER_REVERSE: {
            double reversed = Mod(recover_start_s_ - cur_s_, track_len_);
            if (reversed > track_len_ / 2.0) reversed = 0.0;
            bool timeout = (now_sec - recover_start_sec_) > recovery_timeout_sec_;
            if (timeout) {
                TransitionTo(State::STOP, "reverse timeout");
            } else if (!IsRearSafe(now_sec)) {
                TransitionTo(State::STOP, "rear became unsafe during reverse");
            } else if (IsCurrentBoundaryViolation()) {
                TransitionTo(State::STOP, "boundary became unsafe during reverse");
            } else if (reversed >= recovery_max_reverse_distance_m_) {
                stopped_since_sec_ = -1.0;
                TransitionTo(State::RECOVER_STOP, "reverse distance reached");
            }
            break;
        }

        case State::RECOVER_STOP:
            // 음수에서 양수 명령으로 즉시 뒤집지 않는다. 실제 정지 확인 후에만
            // forward align을 시작한다.
            if (IsStoppedForDwell(now_sec)) {
                recover_start_sec_ = now_sec;
                TransitionTo(State::RECOVER_ALIGN, "stopped after reverse");
            }
            break;

        case State::RECOVER_ALIGN:
            if (IsBoundaryViolation()) {
                TransitionTo(State::STOP, "boundary unsafe during align");
            } else if (!IsFrontSafe(now_sec)) {
                TransitionTo(State::STOP, "front unsafe/unknown during align");
            } else if (IsHeadingAligned()) {
                stuck_since_sec_ = -1.0;  // stuck 타이머 초기화 후 저속 재출발
                TransitionTo(State::DRIVING, "heading aligned, resume low speed");
            } else if ((now_sec - recover_start_sec_) > recovery_timeout_sec_) {
                TransitionTo(State::STOP, "align timeout");
            }
            break;

        case State::STOP:
            // 래치 상태. 사람 개입(/planner/recovery/reset true)으로만 해제.
            if (reset_requested_ && !estop_) {
                reset_requested_ = false;
                recovery_attempts_ = 0;
                stuck_since_sec_ = -1.0;
                boundary_since_sec_ = -1.0;
                ready_since_sec_ = -1.0;
                if (!inputs_ready) {
                    stopped_since_sec_ = -1.0;
                    TransitionTo(State::WAITING_FOR_INPUTS, "manual reset, inputs not ready");
                } else if (IsStoppedForDwell(now_sec)) {
                    TransitionTo(State::DRIVING, "manual reset");
                }
            }
            break;
    }
    reset_requested_ = false;

    PublishOutputs();
}

void RecoverySupervisorNode::PublishOutputs() {
    std_msgs::msg::String st;
    st.data = StateName(state_);
    p_state_->publish(st);

    std_msgs::msg::Float64 v;
    switch (state_) {
        case State::DRIVING:
            v.data = std::numeric_limits<double>::quiet_NaN();  // override 없음
            break;
        case State::RECOVER_REVERSE:
            v.data = -recovery_reverse_speed_mps_;  // Control이 자체 하드리밋 재적용
            break;
        case State::RECOVER_ALIGN:
            v.data = align_speed_mps_;
            break;
        default:  // STUCK_DETECTED / COLLISION_STOP / STOP
            v.data = 0.0;
            break;
    }
    p_target_speed_->publish(v);

    // RViz 상태 텍스트 마커
    visualization_msgs::msg::Marker mrk;
    mrk.header.frame_id = "map";
    mrk.header.stamp = this->now();
    mrk.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
    mrk.id = 0;
    mrk.text = std::string("recovery: ") + StateName(state_);
    mrk.scale.z = 0.5;
    mrk.color.a = 1.0;
    mrk.color.r = (state_ == State::DRIVING) ? 0.0 : 1.0;
    mrk.color.g = (state_ == State::DRIVING) ? 1.0 : 0.0;
    mrk.pose.orientation.w = 1.0;
    p_marker_->publish(mrk);
}

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<RecoverySupervisorNode>("recovery_supervisor_node"));
    rclcpp::shutdown();
    return 0;
}
