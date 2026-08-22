#include "planning_pkg/recovery_supervisor_node.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
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
        case State::RECOVER_REVERSE: return "RECOVER_REVERSE";
        case State::RECOVER_STOP: return "RECOVER_STOP";
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
    this->declare_parameter("recovery/vehicle_width_m", 0.30);
    this->declare_parameter("recovery/vehicle_length_m", 0.55);
    this->declare_parameter("recovery/wall_contact_margin_m", 0.05);
    this->declare_parameter("recovery/allow_reverse", false);
    this->declare_parameter("recovery/recovery_reverse_speed_mps", 0.5);
    this->declare_parameter("recovery/recovery_max_reverse_distance_m", 0.5);
    this->declare_parameter("recovery/recovery_timeout_sec", 4.0);
    this->declare_parameter("recovery/recovery_max_attempts", 3);
    this->declare_parameter("recovery/progress_reset_m", 3.0);
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
    vehicle_width_m_ = this->get_parameter("recovery/vehicle_width_m").as_double();
    vehicle_length_m_ = this->get_parameter("recovery/vehicle_length_m").as_double();
    wall_contact_margin_m_ =
        this->get_parameter("recovery/wall_contact_margin_m").as_double();
    allow_reverse_ = this->get_parameter("recovery/allow_reverse").as_bool();
    recovery_reverse_speed_mps_ =
        this->get_parameter("recovery/recovery_reverse_speed_mps").as_double();
    recovery_max_reverse_distance_m_ =
        this->get_parameter("recovery/recovery_max_reverse_distance_m").as_double();
    recovery_timeout_sec_ = this->get_parameter("recovery/recovery_timeout_sec").as_double();
    recovery_max_attempts_ =
        static_cast<int>(this->get_parameter("recovery/recovery_max_attempts").as_int());
    progress_reset_m_ = this->get_parameter("recovery/progress_reset_m").as_double();
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
    require_range(vehicle_width_m_, 0.05, 5.0, "recovery/vehicle_width_m");
    require_range(vehicle_length_m_, 0.05, 10.0, "recovery/vehicle_length_m");
    require_range(
        wall_contact_margin_m_, -0.30, 0.30, "recovery/wall_contact_margin_m");
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
    // 후진거리의 2배보다 커야 "제자리 왕복"이 진행으로 오인되지 않는다.
    require_range(progress_reset_m_, 0.5, 50.0, "recovery/progress_reset_m");
    if (progress_reset_m_ < 2.0 * recovery_max_reverse_distance_m_) {
        throw std::invalid_argument(
            "recovery/progress_reset_m must be >= 2 * recovery_max_reverse_distance_m");
    }
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
        "RecoverySupervisorNode: stuck_timeout=%.1fs reverse=%s "
        "reverse_limit=%.2fm/s dist=%.2fm max %d consecutive attempts "
        "wall_contact<=%.2fm progress_reset=%.1fm",
        stuck_timeout_sec_, allow_reverse_ ? "ON (wall contact only)" : "OFF",
        recovery_reverse_speed_mps_, recovery_max_reverse_distance_m_,
        recovery_max_attempts_, wall_contact_margin_m_, progress_reset_m_);
}

void RecoverySupervisorNode::CallbackFrenetOdom(const nav_msgs::msg::Odometry::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(mutex_);
    cur_s_ = msg->pose.pose.position.x;
    cur_d_ = msg->pose.pose.position.y;
    frenet_receipt_stamp_ = this->now();
    frenet_source_stamp_ =
        rclcpp::Time(msg->header.stamp, this->get_clock()->get_clock_type());
    has_frenet_ = true;
}

void RecoverySupervisorNode::CallbackPoseOdom(const nav_msgs::msg::Odometry::SharedPtr msg) {
    double yaw = 0.0;
    if (!NormalizedYaw(msg->pose.pose.orientation, &yaw) ||
        !std::isfinite(msg->pose.pose.position.x) ||
        !std::isfinite(msg->pose.pose.position.y) ||
        !std::isfinite(msg->twist.twist.linear.x) ||
        !std::isfinite(msg->twist.twist.linear.y)) {
        RCLCPP_ERROR_THROTTLE(
            this->get_logger(), *get_clock(), 1000,
            "Rejecting pose odometry with invalid pose or velocity");
        return;
    }
    std::lock_guard<std::mutex> lock(mutex_);
    cur_yaw_ = yaw;
    cur_x_ = msg->pose.pose.position.x;
    cur_y_ = msg->pose.pose.position.y;
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
    // local_trajectory_node::ComputeTrackLength / frenet_frame_node / spliner 와
    // 같은 정의를 쓴다: (마지막 s - 첫 s) + 폐합 구간 기하 거리.
    // back_s + 균일 간격은 점간격이 비균일한 CSV 에서 갈라진다.
    const double closure = std::hypot(
        msg->wpnts.front().x_m - msg->wpnts.back().x_m,
        msg->wpnts.front().y_m - msg->wpnts.back().y_m);
    (void)spacing;
    track_len_ = msg->wpnts.back().s_m - msg->wpnts.front().s_m + closure;
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
        !std::isfinite(cur_x_) || !std::isfinite(cur_y_) ||
        !std::isfinite(cur_yaw_) ||
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

bool RecoverySupervisorNode::IsFrontalWallContact(
    double *nose_margin, double *heading_err) const {
    if (nose_margin != nullptr) *nose_margin = std::numeric_limits<double>::quiet_NaN();
    if (heading_err != nullptr) *heading_err = std::numeric_limits<double>::quiet_NaN();
    if (!has_local_ || local_wpnts_.wpnts.empty()) {
        return false;
    }
    // IsCurrentBoundaryViolation과 같은 코리도 기하로, 차 중심 대신 노즈
    // (전방 half_length 투영)의 횡 오프셋으로 경계 여유를 본다. 진단 전용.
    const auto &w = local_wpnts_.wpnts.front();
    const double he = WrapToPi(cur_yaw_ - w.psi_rad);
    const double nose_dev =
        (cur_d_ - w.d_m) + 0.5 * vehicle_length_m_ * std::sin(he);
    const double margin =
        std::min(w.d_left - nose_dev, w.d_right + nose_dev);
    if (nose_margin != nullptr) *nose_margin = margin;
    if (heading_err != nullptr) *heading_err = he;
    return std::isfinite(margin) && margin <= wall_contact_margin_m_ &&
           std::abs(he) < M_PI / 2.0;
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

void RecoverySupervisorNode::TransitionTo(
    State next, const std::string &reason, double now_sec) {
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
        TransitionTo(State::STOP, "E-stop", now_sec);
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
                last_recover_s_ = -1.0;
                stuck_since_sec_ = -1.0;
                TransitionTo(
                    State::DRIVING, "all inputs fresh and vehicle stopped",
                    now_sec);
            }
        }
    } else if (!inputs_ready && state_ != State::STOP) {
        // 한 번 주행 준비가 된 뒤 source/receipt 중 하나라도 끊기면 자동 재개하지
        // 않는다. 사람이 원인을 해결하고 reset해야 한다.
        TransitionTo(State::STOP, readiness_reason, now_sec);
    }

    switch (state_) {
        case State::WAITING_FOR_INPUTS:
            break;

        case State::DRIVING: {
            // 복구 후 progress_reset_m 이상 정상 전진하면 "같은 지점 연속 실패"
            // 카운터를 리셋한다 — 시도는 무제한, 래치는 제자리 반복에만.
            // 후진 중 s 감소는 Mod가 track_len 근처 값이 되므로 < L/2 가 거른다.
            if (recovery_attempts_ > 0 && last_recover_s_ >= 0.0 &&
                track_len_ > 0.0) {
                const double progressed =
                    Mod(cur_s_ - last_recover_s_, track_len_);
                if (progressed >= progress_reset_m_ &&
                    progressed < track_len_ / 2.0) {
                    RCLCPP_INFO(this->get_logger(),
                        "[recovery] %.1fm progressed since last recovery — "
                        "consecutive attempt counter reset", progressed);
                    recovery_attempts_ = 0;
                    last_recover_s_ = -1.0;
                }
            }
            // 발동 조건은 "완전히 박았을 때"뿐이다 (run_0822_191159 근거):
            //   ① 전진 명령인데 stuck_timeout 동안 무진행  (IsStuck)
            //   ② 노즈가 코리도 경계에 닿음                (IsFrontalWallContact)
            //   ③ 실제로 완전히 멈춰 있음                  (IsStoppedForDwell)
            // 셋을 모두 요구한다. 예전의 경계 예측 정지 경로는
            // 삭제됐다 — 그 판정은 레이스라인 에이펙스 여유(0.445m)와 부호 없는
            // 횡오차를 비교해서 정상 주행을 오탐했고, 그때 control이 조향까지
            // 0으로 꺾어(publish_stop) 코너에서 벽으로 직진하게 만들었다.
            double nose_margin = 0.0;
            double heading_err = 0.0;
            const bool wall_contact =
                IsFrontalWallContact(&nose_margin, &heading_err);
            if (!IsStuck(now_sec) || !wall_contact ||
                !IsStoppedForDwell(now_sec)) {
                break;
            }
            if (!allow_reverse_) {
                // 운용 킬스위치: 상대차/사람이 있는 세션은 STOP-only로.
                TransitionTo(
                    State::STOP, "reverse disabled (STOP-only mode)", now_sec);
                break;
            }
            if (recovery_attempts_ >= recovery_max_attempts_) {
                TransitionTo(
                    State::STOP,
                    "consecutive recovery attempts exhausted at same spot",
                    now_sec);
                break;
            }
            recovery_attempts_++;
            recover_start_sec_ = now_sec;
            recover_start_x_ = cur_x_;
            recover_start_y_ = cur_y_;
            last_recover_s_ = cur_s_;
            char reason[176];
            std::snprintf(reason, sizeof(reason),
                "rammed into wall (attempt %d/%d, nose_margin=%.2fm "
                "heading_err=%.2frad)",
                recovery_attempts_, recovery_max_attempts_,
                nose_margin, heading_err);
            TransitionTo(State::RECOVER_REVERSE, reason, now_sec);
            break;
        }

        case State::RECOVER_REVERSE: {
            // 유클리드 후진거리: 벽에 직각으로 박은 자세는 frenet s가 거의 안
            // 변해서 s 기반 측정으로는 타임아웃까지 후진해 버린다.
            const double reversed = std::hypot(
                cur_x_ - recover_start_x_, cur_y_ - recover_start_y_);
            if (reversed >= recovery_max_reverse_distance_m_) {
                stopped_since_sec_ = -1.0;
                TransitionTo(
                    State::RECOVER_STOP, "reverse distance reached", now_sec);
            } else if ((now_sec - recover_start_sec_) > recovery_timeout_sec_) {
                // STOP 래치가 아니라 전진 재시도: 뒤벽 등으로 후진이 안 되는
                // 반복 실패는 DRIVING 의 연속 카운터가 래치한다.
                stopped_since_sec_ = -1.0;
                TransitionTo(
                    State::RECOVER_STOP, "reverse timeout, retry forward",
                    now_sec);
            }
            break;
        }

        case State::RECOVER_STOP:
            // 음수에서 양수 명령으로 즉시 뒤집지 않는다. 실제 정지 확인 후
            // ALIGN 없이 DRIVING 직행 — 복귀 조향/속도는 정상 체인이 최근접
            // 포인트에서 잇는다 (control이 후진 중 slew 기준점을 0으로 리셋해
            // 저속 재출발이 된다). grace는 TransitionTo가 시작한다.
            if (IsStoppedForDwell(now_sec)) {
                stuck_since_sec_ = -1.0;
                TransitionTo(
                    State::DRIVING,
                    "recovered, resume from nearest raceline point", now_sec);
            }
            break;

        case State::STOP:
            // 래치 상태. 사람 개입(/planner/recovery/reset true)으로만 해제.
            // dwell 타이머는 reset 수신 여부와 무관하게 매 사이클 돌려야 한다.
            // 예전에는 IsStoppedForDwell 이 reset 분기 안에서만 호출돼,
            // 첫 reset 이 타이머를 "시작"만 시키고 (now-now)>=dwell 이 false 라
            // 전이하지 못한 채 소비됐다 — 최소 2회(0.2 s 이상 간격) 눌러야
            // 풀렸고, 운용자는 그걸 입력 stale 로 오진했다.
            (void)IsStoppedForDwell(now_sec);
            if (reset_requested_ && !estop_) {
                reset_requested_ = false;
                recovery_attempts_ = 0;
                last_recover_s_ = -1.0;
                stuck_since_sec_ = -1.0;
                ready_since_sec_ = -1.0;
                if (!inputs_ready) {
                    stopped_since_sec_ = -1.0;
                    TransitionTo(
                        State::WAITING_FOR_INPUTS,
                        "manual reset, inputs not ready", now_sec);
                } else if (IsStoppedForDwell(now_sec)) {
                    TransitionTo(State::DRIVING, "manual reset", now_sec);
                } else {
                    // 아직 정지 dwell 이 안 찼다 — reset 을 버리지 않고 유지해
                    // 다음 사이클에 자동으로 전이하게 한다.
                    reset_requested_ = true;
                    RCLCPP_INFO_THROTTLE(this->get_logger(), *get_clock(), 1000,
                        "[recovery] reset 접수 — 정지 dwell(%.2fs) 대기 중",
                        stopped_dwell_sec_);
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
        default:  // WAITING_FOR_INPUTS / RECOVER_STOP / STOP
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
