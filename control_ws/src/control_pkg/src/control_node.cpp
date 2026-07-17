// /local_waypoints(ae_hyu_msgs/WpntArray)를 추종하는 pure pursuit 컨트롤러.
// planning_ws의 시뮬 전용 waypoint_tracker_node.py에서 검증한 로직의 실차 포팅.
//
//   /local_waypoints ──▶ control ──▶ /ackermann_cmd (실차, vesc_ackermann)
//   odom(환경별)     ──▶    │            /drive     (시뮬, f1tenth_gym)
//   /safety/emergency_stop ─┘  (E-stop 시 즉시 정지 명령)
//
// - 벽 충돌/스턱 복구: 전진 명령 중 정지가 지속되면 경로 방향으로 기수가
//   돌아가도록 후진 후 감속 재개 (TRACK <-> REVERSE 상태머신)
// - autonomous 모드: /control/set_mode(Trigger)로 토글. 실차는 기본 OFF로
//   부팅해 조종기 주행을 방해하지 않고, 서비스 콜로 자율주행 시작.
// - 비활성 상태에선 아무것도 발행하지 않는다 (teleop과 토픽 충돌 방지).
#include <algorithm>
#include <cmath>
#include <limits>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <vector>

#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"
#include "example_interfaces/srv/trigger.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"

#include "ae_hyu_msgs/msg/wpnt.hpp"
#include "ae_hyu_msgs/msg/wpnt_array.hpp"

class ControlNode : public rclcpp::Node
{
public:
  ControlNode()
  : Node("control")
  {
    // --- 인터페이스 ---
    control_rate_ = this->declare_parameter<double>("control_rate", 40.0);
    const auto wpnt_topic =
      this->declare_parameter<std::string>("local_waypoints_topic", "/local_waypoints");
    const auto odom_topic =
      this->declare_parameter<std::string>("odom_topic", "/ego_racecar/odom");
    const auto ackermann_topic =
      this->declare_parameter<std::string>("ackermann_topic", "/drive");
    const auto estop_topic =
      this->declare_parameter<std::string>("estop_topic", "/safety/emergency_stop");
    const auto service_name =
      this->declare_parameter<std::string>("service_name", "/control/set_mode");
    autonomous_ = this->declare_parameter<bool>("autonomous_on_start", false);

    // --- pure pursuit ---
    t_lookahead_ = this->declare_parameter<double>("pp/t_lookahead", 0.5);
    min_lookahead_ = this->declare_parameter<double>("pp/min_lookahead_m", 1.2);
    max_lookahead_ = this->declare_parameter<double>("pp/max_lookahead_m", 3.0);
    wheelbase_ = this->declare_parameter<double>("pp/wheelbase_m", 0.33);
    max_steer_ = this->declare_parameter<double>("pp/max_steer_rad", 0.4189);
    speed_scale_ = this->declare_parameter<double>("pp/speed_scale", 0.5);
    max_speed_ = this->declare_parameter<double>("pp/max_speed_mps", 4.0);
    wpnt_timeout_ = this->declare_parameter<double>("pp/waypoint_timeout_sec", 0.5);

    // --- 스턱 복구 ---
    enable_recovery_ = this->declare_parameter<bool>("recovery/enable", true);
    stuck_detect_sec_ = this->declare_parameter<double>("recovery/stuck_detect_sec", 0.6);
    reverse_speed_ = this->declare_parameter<double>("recovery/reverse_speed_mps", 1.2);
    reverse_max_sec_ = this->declare_parameter<double>("recovery/reverse_max_sec", 3.0);
    reverse_min_dist_ = this->declare_parameter<double>("recovery/reverse_min_dist_m", 0.3);
    resume_speed_cap_ = this->declare_parameter<double>("recovery/resume_speed_cap_mps", 2.0);
    resume_cap_sec_ = this->declare_parameter<double>("recovery/resume_cap_sec", 1.5);

    auto qos = rclcpp::QoS(rclcpp::KeepLast(10)).reliable();
    s_wpnts_ = this->create_subscription<ae_hyu_msgs::msg::WpntArray>(
      wpnt_topic, qos,
      std::bind(&ControlNode::CallbackWaypoints, this, std::placeholders::_1));
    s_odom_ = this->create_subscription<nav_msgs::msg::Odometry>(
      odom_topic, qos,
      std::bind(&ControlNode::CallbackOdom, this, std::placeholders::_1));
    s_estop_ = this->create_subscription<std_msgs::msg::Bool>(
      estop_topic, qos,
      std::bind(&ControlNode::CallbackEstop, this, std::placeholders::_1));

    p_cmd_ = this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>(
      ackermann_topic, qos);

    srv_mode_ = this->create_service<example_interfaces::srv::Trigger>(
      service_name,
      std::bind(&ControlNode::CallbackSetMode, this,
                std::placeholders::_1, std::placeholders::_2));

    t_control_ = this->create_wall_timer(
      std::chrono::milliseconds((int64_t)(1000 / control_rate_)),
      [this]() { this->Run(); });

    RCLCPP_INFO(this->get_logger(),
      "Control node ready: odom=%s -> %s @ %.0f Hz, autonomous=%s, "
      "speed_scale=%.2f max_speed=%.1f m/s",
      odom_topic.c_str(), ackermann_topic.c_str(), control_rate_,
      autonomous_ ? "ON" : "OFF (call /control/set_mode)", speed_scale_, max_speed_);
  }

private:
  void CallbackWaypoints(const ae_hyu_msgs::msg::WpntArray::SharedPtr msg)
  {
    if (msg->wpnts.empty()) {
      return;
    }
    std::lock_guard<std::mutex> lock(mutex_wpnts_);
    wpnts_ = msg->wpnts;
    wpnts_stamp_ = this->now();
  }

  void CallbackOdom(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(mutex_odom_);
    odom_ = *msg;
    has_odom_ = true;
  }

  void CallbackEstop(const std_msgs::msg::Bool::SharedPtr msg)
  {
    if (msg->data && !estop_) {
      RCLCPP_ERROR(this->get_logger(), "E-STOP activated — 즉시 정지");
    }
    estop_ = msg->data;
  }

  void CallbackSetMode(
    const std::shared_ptr<example_interfaces::srv::Trigger::Request> /*request*/,
    std::shared_ptr<example_interfaces::srv::Trigger::Response> response)
  {
    autonomous_ = !autonomous_;
    mode_ = Mode::kTrack;
    stuck_since_valid_ = false;
    if (!autonomous_) {
      // OFF 전환: 마지막 속도 명령이 유지되지 않도록 정지를 한 번 발행 후 침묵
      PublishStop();
    }
    response->success = true;
    response->message = autonomous_ ? "autonomous ON" : "autonomous OFF";
    RCLCPP_WARN(this->get_logger(), "Mode toggled: %s", response->message.c_str());
  }

  void PublishStop()
  {
    ackermann_msgs::msg::AckermannDriveStamped cmd;
    cmd.header.stamp = this->now();
    cmd.header.frame_id = "base_link";
    p_cmd_->publish(cmd);
  }

  // 차량 헤딩과 로컬 경로 진행 방향의 각도차 [-pi, pi]
  double PathHeadingError(const std::vector<ae_hyu_msgs::msg::Wpnt> & wpnts,
                          double cx, double cy, double yaw) const
  {
    size_t nearest = 0;
    double best = std::numeric_limits<double>::max();
    for (size_t i = 0; i < wpnts.size(); ++i) {
      double dx = wpnts[i].x_m - cx;
      double dy = wpnts[i].y_m - cy;
      double d2 = dx * dx + dy * dy;
      if (d2 < best) {
        best = d2;
        nearest = i;
      }
    }
    size_t ahead = std::min(nearest + 5, wpnts.size() - 1);  // ~0.5m 앞
    if (ahead == nearest) {
      return 0.0;
    }
    double path_yaw = std::atan2(wpnts[ahead].y_m - wpnts[nearest].y_m,
                                 wpnts[ahead].x_m - wpnts[nearest].x_m);
    double err = path_yaw - yaw;
    while (err > M_PI) err -= 2.0 * M_PI;
    while (err < -M_PI) err += 2.0 * M_PI;
    return err;
  }

  void Run()
  {
    // E-stop: 모드와 무관하게 정지 명령을 계속 발행
    if (estop_) {
      PublishStop();
      return;
    }
    // 수동 모드: 발행 안 함 (teleop이 /ackermann_cmd를 소유)
    if (!autonomous_) {
      return;
    }
    if (!has_odom_) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *get_clock(), 2000,
                           "Waiting for odom...");
      return;
    }

    // 최신 상태 복사
    nav_msgs::msg::Odometry odom;
    {
      std::lock_guard<std::mutex> lock(mutex_odom_);
      odom = odom_;
    }
    std::vector<ae_hyu_msgs::msg::Wpnt> wpnts;
    {
      std::lock_guard<std::mutex> lock(mutex_wpnts_);
      if (wpnts_.empty() ||
          (this->now() - wpnts_stamp_).seconds() > wpnt_timeout_) {
        PublishStop();  // 로컬 경로 두절 -> 안전 정지
        RCLCPP_WARN_THROTTLE(this->get_logger(), *get_clock(), 2000,
                             "Local waypoints stale/missing -> stop");
        return;
      }
      wpnts = wpnts_;
    }

    double cx = odom.pose.pose.position.x;
    double cy = odom.pose.pose.position.y;
    const auto & q = odom.pose.pose.orientation;
    double yaw = std::atan2(2.0 * (q.w * q.z + q.x * q.y),
                            1.0 - 2.0 * (q.y * q.y + q.z * q.z));
    double v = odom.twist.twist.linear.x;
    rclcpp::Time t = this->now();

    ackermann_msgs::msg::AckermannDriveStamped cmd;
    cmd.header.stamp = t;
    cmd.header.frame_id = "base_link";

    // --- REVERSE: 스턱 복구 중 ---
    if (mode_ == Mode::kReverse) {
      double heading_err = PathHeadingError(wpnts, cx, cy, yaw);
      double reversed = std::hypot(cx - reverse_start_x_, cy - reverse_start_y_);
      double elapsed = (t - reverse_start_t_).seconds();
      bool done = (std::abs(heading_err) < 0.8 && reversed > reverse_min_dist_) ||
                  elapsed > reverse_max_sec_;
      if (done) {
        mode_ = Mode::kTrack;
        stuck_since_valid_ = false;
        resume_until_ = t + rclcpp::Duration::from_seconds(resume_cap_sec_);
        RCLCPP_INFO(this->get_logger(),
          "복구 완료: %.2fm 후진, heading_err=%.2frad — 주행 재개", reversed, heading_err);
      } else {
        // 후진 시 yaw_rate = v/L*tan(δ), v<0 이므로 δ는 목표 회전의 반대 부호
        cmd.drive.steering_angle =
          (heading_err > 0 ? -1.0 : 1.0) * std::min(max_steer_, 0.35);
        cmd.drive.speed = -std::abs(reverse_speed_);
        p_cmd_->publish(cmd);
        return;
      }
    }

    // --- TRACK: pure pursuit ---
    double la = std::clamp(t_lookahead_ * std::abs(v), min_lookahead_, max_lookahead_);

    // 로컬 경로는 차량 앞으로 정렬돼 있음 -> lookahead 이상 떨어진 첫 점
    size_t target_idx = wpnts.size() - 1;
    for (size_t i = 0; i < wpnts.size(); ++i) {
      double dx = wpnts[i].x_m - cx;
      double dy = wpnts[i].y_m - cy;
      if (std::hypot(dx, dy) >= la) {
        target_idx = i;
        break;
      }
    }
    double dx = wpnts[target_idx].x_m - cx;
    double dy = wpnts[target_idx].y_m - cy;
    double y_b = -dx * std::sin(yaw) + dy * std::cos(yaw);
    double L = std::max(std::hypot(dx, dy), 1e-3);
    double curvature = 2.0 * y_b / (L * L);
    double steer = std::atan(wheelbase_ * curvature);

    double speed = wpnts[target_idx].vx_mps * speed_scale_;
    speed = std::min(speed, max_speed_);            // 하드웨어 상한
    if (resume_until_.has_value() && t < *resume_until_) {
      speed = std::min(speed, resume_speed_cap_);   // 복구 직후 감속 재가속
    }

    cmd.drive.steering_angle = std::clamp(steer, -max_steer_, max_steer_);
    cmd.drive.speed = speed;
    p_cmd_->publish(cmd);

    // --- 스턱 감지: 전진 명령 중인데 실제로 못 움직임 ---
    if (enable_recovery_ && speed > 0.3 && std::abs(v) < 0.1) {
      if (!stuck_since_valid_) {
        stuck_since_ = t;
        stuck_since_valid_ = true;
      } else if ((t - stuck_since_).seconds() > stuck_detect_sec_) {
        mode_ = Mode::kReverse;
        reverse_start_t_ = t;
        reverse_start_x_ = cx;
        reverse_start_y_ = cy;
        stuck_since_valid_ = false;
        RCLCPP_WARN(this->get_logger(),
          "벽 충돌/스턱 감지 (cmd=%.1f m/s, 실제=%.2f m/s) — 후진 복구 시작", speed, v);
      }
    } else {
      stuck_since_valid_ = false;
    }
  }

  enum class Mode { kTrack, kReverse };

  // Parameters
  double control_rate_{40.0};
  double t_lookahead_{0.5};
  double min_lookahead_{1.2};
  double max_lookahead_{3.0};
  double wheelbase_{0.33};
  double max_steer_{0.4189};
  double speed_scale_{0.5};
  double max_speed_{4.0};
  double wpnt_timeout_{0.5};
  bool enable_recovery_{true};
  double stuck_detect_sec_{0.6};
  double reverse_speed_{1.2};
  double reverse_max_sec_{3.0};
  double reverse_min_dist_{0.3};
  double resume_speed_cap_{2.0};
  double resume_cap_sec_{1.5};

  // State
  bool autonomous_{false};
  bool estop_{false};
  Mode mode_{Mode::kTrack};
  std::vector<ae_hyu_msgs::msg::Wpnt> wpnts_;
  rclcpp::Time wpnts_stamp_{0, 0, RCL_ROS_TIME};
  nav_msgs::msg::Odometry odom_;
  bool has_odom_{false};
  bool stuck_since_valid_{false};
  rclcpp::Time stuck_since_{0, 0, RCL_ROS_TIME};
  rclcpp::Time reverse_start_t_{0, 0, RCL_ROS_TIME};
  double reverse_start_x_{0.0};
  double reverse_start_y_{0.0};
  std::optional<rclcpp::Time> resume_until_;
  std::mutex mutex_wpnts_;
  std::mutex mutex_odom_;

  rclcpp::Subscription<ae_hyu_msgs::msg::WpntArray>::SharedPtr s_wpnts_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr s_odom_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr s_estop_;
  rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr p_cmd_;
  rclcpp::Service<example_interfaces::srv::Trigger>::SharedPtr srv_mode_;
  rclcpp::TimerBase::SharedPtr t_control_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ControlNode>());
  rclcpp::shutdown();
  return 0;
}
