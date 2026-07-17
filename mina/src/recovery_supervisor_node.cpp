#include "planning_pkg/recovery_supervisor_node.hpp"

#include <algorithm>
#include <limits>

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
double YawFromQuat(const geometry_msgs::msg::Quaternion &q) {
    return std::atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z));
}
}  // namespace

const char *RecoverySupervisorNode::StateName(State s) {
    switch (s) {
        case State::DRIVING: return "DRIVING";
        case State::STUCK_DETECTED: return "STUCK_DETECTED";
        case State::COLLISION_STOP: return "COLLISION_STOP";
        case State::RECOVER_REVERSE: return "RECOVER_REVERSE";
        case State::RECOVER_ALIGN: return "RECOVER_ALIGN";
        case State::STOP: return "STOP";
    }
    return "UNKNOWN";
}

RecoverySupervisorNode::RecoverySupervisorNode(
    const std::string &node_name, const rclcpp::NodeOptions &options)
    : Node(node_name, options) {

    auto qos = rclcpp::QoS(rclcpp::KeepLast(10)).reliable();

    this->declare_parameter("recovery/loop_rate_hz", 20.0);
    this->declare_parameter("recovery/stuck_timeout_sec", 1.0);
    this->declare_parameter("recovery/stuck_min_progress_m", 0.10);
    this->declare_parameter("recovery/min_forward_cmd_mps", 0.2);
    this->declare_parameter("recovery/boundary_stop_margin_m", 0.15);
    this->declare_parameter("recovery/rear_check_dist_m", 1.5);
    this->declare_parameter("recovery/recovery_reverse_speed_mps", 0.5);
    this->declare_parameter("recovery/recovery_max_reverse_distance_m", 0.5);
    this->declare_parameter("recovery/recovery_timeout_sec", 4.0);
    this->declare_parameter("recovery/recovery_max_attempts", 2);
    this->declare_parameter("recovery/align_tolerance_rad", 0.35);
    this->declare_parameter("recovery/align_speed_mps", 0.5);
    this->declare_parameter("recovery/odom_timeout_sec", 0.5);
    this->declare_parameter("recovery/perception_timeout_sec", 0.5);
    this->declare_parameter("recovery/pose_odom_topic", std::string("/pf/pose/odom"));

    loop_rate_hz_ = this->get_parameter("recovery/loop_rate_hz").as_double();
    stuck_timeout_sec_ = this->get_parameter("recovery/stuck_timeout_sec").as_double();
    stuck_min_progress_m_ = this->get_parameter("recovery/stuck_min_progress_m").as_double();
    min_forward_cmd_mps_ = this->get_parameter("recovery/min_forward_cmd_mps").as_double();
    boundary_stop_margin_m_ = this->get_parameter("recovery/boundary_stop_margin_m").as_double();
    rear_check_dist_m_ = this->get_parameter("recovery/rear_check_dist_m").as_double();
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
    const std::string pose_topic =
        this->get_parameter("recovery/pose_odom_topic").as_string();

    s_frenet_odom_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/car_state/frenet/odom", qos,
        std::bind(&RecoverySupervisorNode::CallbackFrenetOdom, this, std::placeholders::_1));
    s_pose_odom_ = this->create_subscription<nav_msgs::msg::Odometry>(
        pose_topic, qos,
        std::bind(&RecoverySupervisorNode::CallbackPoseOdom, this, std::placeholders::_1));
    s_local_wpnts_ = this->create_subscription<ae_hyu_msgs::msg::WpntArray>(
        "/local_waypoints", qos,
        std::bind(&RecoverySupervisorNode::CallbackLocalWaypoints, this, std::placeholders::_1));
    s_obstacles_ = this->create_subscription<ae_hyu_msgs::msg::ObstacleArray>(
        "/perception/obstacles", qos,
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
        std::chrono::milliseconds((int64_t)(1000 / loop_rate_hz_)),
        [this]() { this->Run(); });

    RCLCPP_INFO(this->get_logger(),
        "RecoverySupervisorNode: stuck_timeout=%.1fs reverse=%.2fm/s max %d attempts",
        stuck_timeout_sec_, recovery_reverse_speed_mps_, recovery_max_attempts_);
}

void RecoverySupervisorNode::CallbackFrenetOdom(const nav_msgs::msg::Odometry::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(mutex_);
    cur_s_ = msg->pose.pose.position.x;
    cur_d_ = msg->pose.pose.position.y;
    cur_vs_ = msg->twist.twist.linear.x;
    frenet_stamp_ = this->now();
    has_frenet_ = true;
}

void RecoverySupervisorNode::CallbackPoseOdom(const nav_msgs::msg::Odometry::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(mutex_);
    cur_yaw_ = YawFromQuat(msg->pose.pose.orientation);
    pose_stamp_ = this->now();
}

void RecoverySupervisorNode::CallbackLocalWaypoints(const ae_hyu_msgs::msg::WpntArray::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(mutex_);
    local_wpnts_ = *msg;
    has_local_ = !msg->wpnts.empty();
    // 트랙 길이 추정 (wrap-around용, 마지막 s가 최대값 근사)
    for (const auto &w : msg->wpnts) {
        gb_max_s_ = std::max(gb_max_s_ == 1e9 ? 0.0 : gb_max_s_, w.s_m);
    }
}

void RecoverySupervisorNode::CallbackObstacles(const ae_hyu_msgs::msg::ObstacleArray::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(mutex_);
    obstacles_ = *msg;
    obstacles_stamp_ = this->now();
}

void RecoverySupervisorNode::CallbackEStop(const std_msgs::msg::Bool::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(mutex_);
    estop_ = msg->data;
}

void RecoverySupervisorNode::CallbackReset(const std_msgs::msg::Bool::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (msg->data) reset_requested_ = true;
}

bool RecoverySupervisorNode::InputsStale(double now_sec) const {
    return (now_sec - rclcpp::Time(frenet_stamp_).seconds()) > odom_timeout_sec_ ||
           (now_sec - rclcpp::Time(obstacles_stamp_).seconds()) > perception_timeout_sec_;
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
    double progress = Mod(cur_s_ - stuck_ref_s_, gb_max_s_);
    if (progress > gb_max_s_ / 2.0) progress = 0.0;  // 후진/역방향은 진행으로 안 봄
    if (progress >= stuck_min_progress_m_) {
        stuck_since_sec_ = now_sec;  // 진행함 -> 타이머 리셋
        stuck_ref_s_ = cur_s_;
        return false;
    }
    return (now_sec - stuck_since_sec_) > stuck_timeout_sec_;
}

bool RecoverySupervisorNode::IsBoundaryViolation() const {
    if (!has_local_) return false;
    // 첫 local wpnt의 남은 경계 여유를 차량 실제 d와의 오프셋으로 보정.
    const auto &w = local_wpnts_.wpnts.front();
    double dev = cur_d_ - w.d_m;                 // 경로 대비 횡 이탈 (d>0=왼쪽)
    double margin_left = w.d_left - dev;         // 왼쪽 경계까지 남은 거리
    double margin_right = w.d_right + dev;       // 오른쪽 경계까지 남은 거리
    return std::min(margin_left, margin_right) < boundary_stop_margin_m_;
}

bool RecoverySupervisorNode::IsRearSafe() const {
    // 후방 rear_check_dist_m 구간에 장애물(정적/동적/사람 추정 무관)이
    // 하나라도 있으면 후진 금지.
    for (const auto &obs : obstacles_.obstacles) {
        double behind = Mod(cur_s_ - obs.s_end, gb_max_s_);
        if (behind < rear_check_dist_m_) {
            return false;
        }
    }
    return true;
}

bool RecoverySupervisorNode::IsHeadingAligned() const {
    if (!has_local_) return false;
    double path_psi = local_wpnts_.wpnts.front().psi_rad;
    return std::abs(WrapToPi(cur_yaw_ - path_psi)) < align_tolerance_rad_;
}

void RecoverySupervisorNode::TransitionTo(State next, const std::string &reason) {
    if (next == state_) return;
    RCLCPP_WARN(this->get_logger(), "[recovery] %s -> %s (%s)",
                StateName(state_), StateName(next), reason.c_str());
    state_ = next;
}

void RecoverySupervisorNode::Run() {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!has_frenet_ || !has_local_) {
        return;  // 입력 없으면 판단 보류 (초기 부팅)
    }
    double now_sec = this->now().seconds();

    // 최우선: E-stop / 입력 stale -> 항상 STOP (자동 복구 금지)
    if (estop_) {
        TransitionTo(State::STOP, "E-stop");
    } else if (InputsStale(now_sec) && state_ != State::STOP) {
        TransitionTo(State::STOP, "odom/perception timeout");
    }

    switch (state_) {
        case State::DRIVING:
            if (IsBoundaryViolation()) {
                TransitionTo(State::COLLISION_STOP, "boundary margin violated");
            } else if (IsStuck(now_sec)) {
                TransitionTo(State::STUCK_DETECTED, "no progress under forward cmd");
            }
            break;

        case State::COLLISION_STOP:
            // 저속 정지 완료 후 stuck 복구 절차로 합류 (동일 후방 안전 검사 경유)
            if (std::abs(cur_vs_) < 0.05) {
                TransitionTo(State::STUCK_DETECTED, "stopped, evaluate recovery");
            }
            break;

        case State::STUCK_DETECTED:
            if (recovery_attempts_ >= recovery_max_attempts_) {
                TransitionTo(State::STOP, "max recovery attempts exceeded");
            } else if (!IsRearSafe()) {
                TransitionTo(State::STOP, "rear not safe, no reverse");
            } else {
                recovery_attempts_++;
                recover_start_sec_ = now_sec;
                recover_start_s_ = cur_s_;
                TransitionTo(State::RECOVER_REVERSE, "rear safe, limited reverse");
            }
            break;

        case State::RECOVER_REVERSE: {
            double reversed = Mod(recover_start_s_ - cur_s_, gb_max_s_);
            if (reversed > gb_max_s_ / 2.0) reversed = 0.0;
            bool timeout = (now_sec - recover_start_sec_) > recovery_timeout_sec_;
            if (timeout) {
                TransitionTo(State::STOP, "reverse timeout");
            } else if (!IsRearSafe()) {
                TransitionTo(State::STOP, "rear became unsafe during reverse");
            } else if (reversed >= recovery_max_reverse_distance_m_) {
                recover_start_sec_ = now_sec;  // align 타이머 재사용
                TransitionTo(State::RECOVER_ALIGN, "reverse distance reached");
            }
            break;
        }

        case State::RECOVER_ALIGN:
            if (IsHeadingAligned()) {
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
                TransitionTo(State::DRIVING, "manual reset");
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
