// 스택 헬스 모니터: perception/planning/control/localization/hardware의 핵심
// 토픽 수신 상태를 감시해 /integration/status로 1줄 요약을 주기 발행한다.
//
// 의도적으로 "구독만" 한다 — 어떤 서비스도 호출하지 않는다.
// (이전 골격은 /control/set_mode Trigger를 주기 호출했는데, 실제 control_node에서
//  그 서비스는 자율주행 ON/OFF 토글이라 주기 호출 시 제어가 계속 꺼졌다 켜진다.)

#include <chrono>
#include <memory>
#include <string>
#include <vector>

#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"
#include "f1tenth_perception_msgs/msg/obstacle_array.hpp"
#include "f1tenth_planning_msgs/msg/wpnt_array.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "std_msgs/msg/string.hpp"

class IntegrationNode : public rclcpp::Node
{
public:
  IntegrationNode()
  : Node("integration")
  {
    const auto scan_topic =
      this->declare_parameter<std::string>("scan_topic", "/scan");
    const auto odom_topic =
      this->declare_parameter<std::string>("odom_topic", "/pf/pose/odom");
    const auto perception_topic =
      this->declare_parameter<std::string>("perception_topic", "/perception/obstacles");
    const auto waypoints_topic =
      this->declare_parameter<std::string>("waypoints_topic", "/local_waypoints");
    const auto drive_topic =
      this->declare_parameter<std::string>("drive_topic", "/ackermann_cmd");
    stale_sec_ = this->declare_parameter<double>("stale_sec", 1.0);
    // perception은 장애물이 없어도 빈 배열을 주기 발행하는 구현과 이벤트성
    // 발행 구현이 섞여 있어 별도(느슨한) 타임아웃을 쓴다.
    perception_stale_sec_ = this->declare_parameter<double>("perception_stale_sec", 3.0);
    const auto period = this->declare_parameter<double>("status_period_sec", 1.0);

    monitors_.resize(kCount);
    monitors_[kScan].name = "scan";
    monitors_[kOdom].name = "odom";
    monitors_[kPerception].name = "perception";
    monitors_[kPlanning].name = "planning";
    monitors_[kControl].name = "control";

    sub_scan_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
      scan_topic, rclcpp::SensorDataQoS(),
      [this](sensor_msgs::msg::LaserScan::ConstSharedPtr) {touch(kScan);});
    sub_odom_ = this->create_subscription<nav_msgs::msg::Odometry>(
      odom_topic, 10,
      [this](nav_msgs::msg::Odometry::ConstSharedPtr msg) {
        touch(kOdom);
        speed_ = msg->twist.twist.linear.x;
      });
    sub_obs_ = this->create_subscription<f1tenth_perception_msgs::msg::ObstacleArray>(
      perception_topic, 10,
      [this](f1tenth_perception_msgs::msg::ObstacleArray::ConstSharedPtr msg) {
        touch(kPerception);
        obstacle_count_ = msg->obstacles.size();
      });
    sub_wpnts_ = this->create_subscription<f1tenth_planning_msgs::msg::WpntArray>(
      waypoints_topic, 10,
      [this](f1tenth_planning_msgs::msg::WpntArray::ConstSharedPtr msg) {
        touch(kPlanning);
        waypoint_count_ = msg->wpnts.size();
      });
    sub_drive_ = this->create_subscription<ackermann_msgs::msg::AckermannDriveStamped>(
      drive_topic, 10,
      [this](ackermann_msgs::msg::AckermannDriveStamped::ConstSharedPtr msg) {
        touch(kControl);
        cmd_speed_ = msg->drive.speed;
      });

    pub_status_ = this->create_publisher<std_msgs::msg::String>("/integration/status", 10);
    timer_ = this->create_wall_timer(
      std::chrono::duration<double>(period),
      std::bind(&IntegrationNode::report, this));

    RCLCPP_INFO(
      this->get_logger(),
      "Integration health monitor: scan=%s odom=%s perception=%s planning=%s control=%s",
      scan_topic.c_str(), odom_topic.c_str(), perception_topic.c_str(),
      waypoints_topic.c_str(), drive_topic.c_str());
  }

private:
  enum Channel {kScan = 0, kOdom, kPerception, kPlanning, kControl, kCount};

  struct Monitor
  {
    std::string name;
    rclcpp::Time last_stamp{0, 0, RCL_ROS_TIME};
    bool seen{false};
    bool ok{false};
  };

  void touch(Channel ch)
  {
    monitors_[ch].last_stamp = this->now();
    monitors_[ch].seen = true;
  }

  void report()
  {
    const auto now = this->now();
    std::string line;
    for (auto & m : monitors_) {
      const double timeout =
        (m.name == "perception") ? perception_stale_sec_ : stale_sec_;
      const bool ok = m.seen && (now - m.last_stamp).seconds() < timeout;
      if (ok != m.ok) {
        if (ok) {
          RCLCPP_INFO(this->get_logger(), "[%s] OK", m.name.c_str());
        } else {
          RCLCPP_WARN(
            this->get_logger(), "[%s] %s", m.name.c_str(),
            m.seen ? "STALE" : "NO DATA");
        }
        m.ok = ok;
      }
      if (!line.empty()) {line += ' ';}
      line += m.name + ':' + (ok ? "OK" : (m.seen ? "STALE" : "WAIT"));
    }
    line += " | obs=" + std::to_string(obstacle_count_) +
      " wpnts=" + std::to_string(waypoint_count_) +
      " v=" + std::to_string(speed_).substr(0, 4) +
      " cmd=" + std::to_string(cmd_speed_).substr(0, 4);

    std_msgs::msg::String msg;
    msg.data = line;
    pub_status_->publish(msg);
  }

  std::vector<Monitor> monitors_;
  double stale_sec_{1.0};
  double perception_stale_sec_{3.0};
  size_t obstacle_count_{0};
  size_t waypoint_count_{0};
  double speed_{0.0};
  double cmd_speed_{0.0};

  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr sub_scan_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odom_;
  rclcpp::Subscription<f1tenth_perception_msgs::msg::ObstacleArray>::SharedPtr sub_obs_;
  rclcpp::Subscription<f1tenth_planning_msgs::msg::WpntArray>::SharedPtr sub_wpnts_;
  rclcpp::Subscription<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr sub_drive_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_status_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<IntegrationNode>());
  rclcpp::shutdown();
  return 0;
}
