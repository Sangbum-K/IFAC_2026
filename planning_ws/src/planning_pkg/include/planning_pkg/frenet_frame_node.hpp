// 원본 planner_pkg에 헤더가 누락되어 있어 cpp 구현부로부터 복원 작성함.
// odom(map 좌표)을 Frenet (s, d, vs, vd)로 변환해
// /car_state/frenet/odom (nav_msgs/Odometry 재활용: pose.x=s, pose.y=d)으로 발행.
// odom_mode: sim_gt=/ego_racecar/odom, sim_pf|real=/pf/pose/odom
#ifndef PLANNING_PKG__FRENET_FRAME_NODE_HPP_
#define PLANNING_PKG__FRENET_FRAME_NODE_HPP_

#include <cmath>
#include <limits>
#include <mutex>
#include <string>
#include <utility>
#include <vector>

#include "builtin_interfaces/msg/time.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"

#include "f1tenth_planning_msgs/msg/wpnt_array.hpp"

class FrenetFrameNode : public rclcpp::Node
{
public:
  explicit FrenetFrameNode(
    const std::string & node_name,
    const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  ~FrenetFrameNode() override;

private:
  struct FrenetCoordinates
  {
    double s{0.0};
    double d{0.0};
  };

  void CallbackOdom(const nav_msgs::msg::Odometry::SharedPtr msg);
  void CallbackGlobalWaypoints(const f1tenth_planning_msgs::msg::WpntArray::SharedPtr msg);
  void Run();

  void ProcessFrenetConversion(
    const nav_msgs::msg::Odometry & odom_copy,
    const std::vector<std::vector<double>> & trajectory_copy);
  int FindClosestWaypoint(
    double vehicle_x, double vehicle_y,
    const std::vector<std::vector<double>> & trajectory);
  FrenetCoordinates CartesianToFrenet(
    double x, double y,
    const std::vector<std::vector<double>> & trajectory);
  std::pair<double, double> CartesianVelocityToFrenet(
    double vx, double vy, double vehicle_theta,
    const std::vector<std::vector<double>> & trajectory);
  void PublishFrenetOdometry(
    const builtin_interfaces::msg::Time & timestamp,
    const FrenetCoordinates & frenet_coords,
    double vs, double vd);

  // Parameters
  double loop_rate_hz_{40.0};
  std::string odom_mode_;   // "sim_gt" | "sim_pf" | "real"

  // State
  nav_msgs::msg::Odometry current_odom_;
  bool has_odom_{false};
  std::vector<std::vector<double>> global_trajectory_;
  bool has_trajectory_{false};
  int last_closest_index_{0};
  std::mutex mutex_odom_;
  std::mutex mutex_trajectory_;

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr s_odom_;
  rclcpp::Subscription<f1tenth_planning_msgs::msg::WpntArray>::SharedPtr s_global_waypoints_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr p_frenet_odom_;
  rclcpp::TimerBase::SharedPtr t_process_;
};

#endif  // PLANNING_PKG__FRENET_FRAME_NODE_HPP_
