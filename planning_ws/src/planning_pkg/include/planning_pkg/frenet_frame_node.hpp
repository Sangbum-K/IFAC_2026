// 원본 planner_pkg에 헤더가 누락되어 있어 cpp 구현부로부터 복원 작성함.
// odom(map 좌표)을 Frenet (s, d, vs, vd)로 변환해
// /car_state/frenet/odom (nav_msgs/Odometry 재활용: pose.x=s, pose.y=d)으로 발행.
// odom_mode: sim_gt=/ego_racecar/odom, real=/pf/pose/odom
#ifndef PLANNING_PKG__FRENET_FRAME_NODE_HPP_
#define PLANNING_PKG__FRENET_FRAME_NODE_HPP_

#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdint>
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
    double unwrapped_s{0.0};
    double track_psi{0.0};
    double kappa{0.0};
    int segment_index{-1};
    bool valid{false};
  };

  void CallbackOdom(const nav_msgs::msg::Odometry::SharedPtr msg);
  void CallbackGlobalWaypoints(const f1tenth_planning_msgs::msg::WpntArray::SharedPtr msg);
  void Run();

  bool ProcessFrenetConversion(
    const nav_msgs::msg::Odometry & odom_copy,
    const std::vector<std::vector<double>> & trajectory_copy,
    double track_length);
  FrenetCoordinates CartesianToFrenet(
    double x, double y, double vehicle_theta,
    const std::vector<std::vector<double>> & trajectory,
    double track_length);
  bool CartesianVelocityToFrenet(
    double vx, double vy, double vehicle_theta,
    const FrenetCoordinates & frenet_coords,
    double & s_dot, double & d_dot);
  void PublishFrenetOdometry(
    const builtin_interfaces::msg::Time & timestamp,
    const FrenetCoordinates & frenet_coords,
    double s_dot, double d_dot);

  // Parameters
  double loop_rate_hz_{40.0};
  std::string odom_mode_;   // "sim_gt" | "real"
  double odom_receive_timeout_sec_{0.20};
  double odom_source_timeout_sec_{0.20};
  double odom_future_tolerance_sec_{0.05};
  int projection_search_segments_{40};
  double projection_reacquire_distance_m_{2.0};
  double projection_heading_weight_m2_{0.25};
  double projection_progress_weight_{0.02};
  double min_frenet_jacobian_{0.20};

  // State
  nav_msgs::msg::Odometry current_odom_;
  bool has_odom_{false};
  std::chrono::steady_clock::time_point odom_arrival_time_{};
  int64_t odom_source_stamp_ns_{0};
  int64_t last_published_odom_stamp_ns_{0};
  std::vector<std::vector<double>> global_trajectory_;
  bool has_trajectory_{false};
  double track_length_{0.0};
  int last_segment_index_{0};
  bool has_projection_{false};
  double last_unwrapped_s_{0.0};
  double last_track_psi_{0.0};
  std::atomic<bool> projection_reset_requested_{false};
  std::mutex mutex_odom_;
  std::mutex mutex_trajectory_;

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr s_odom_;
  rclcpp::Subscription<f1tenth_planning_msgs::msg::WpntArray>::SharedPtr s_global_waypoints_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr p_frenet_odom_;
  rclcpp::TimerBase::SharedPtr t_process_;
};

#endif  // PLANNING_PKG__FRENET_FRAME_NODE_HPP_
