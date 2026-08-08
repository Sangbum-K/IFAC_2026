// 원본 planner_pkg에 헤더가 누락되어 있어 cpp 구현부로부터 복원 작성함.
// 현재 s 주변에서 global waypoints를 80개 슬라이스해 /local_waypoints로 발행.
// 회피: spliner_node가 발행하는 /planner/avoidance/otwpnts가 유효(비어있지
// 않고 신선)하면 로컬 윈도우의 해당 s 구간을 회피 궤적으로 치환(splice)한다
// — ForzaETH(unicorn) 스택의 state_machine splice 역할.
#ifndef PLANNING_PKG__LOCAL_TRAJECTORY_NODE_HPP_
#define PLANNING_PKG__LOCAL_TRAJECTORY_NODE_HPP_

#include <chrono>
#include <array>
#include <cmath>
#include <cstdint>
#include <limits>
#include <mutex>
#include <string>
#include <vector>

#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"

#include "f1tenth_planning_msgs/msg/wpnt.hpp"
#include "f1tenth_planning_msgs/msg/wpnt_array.hpp"

class LocalTrajectoryNode : public rclcpp::Node
{
public:
  explicit LocalTrajectoryNode(
    const std::string & node_name,
    const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  ~LocalTrajectoryNode() override;

private:
  void CallbackGlobalWaypoints(const f1tenth_planning_msgs::msg::WpntArray::SharedPtr msg);
  void CallbackFrenetOdom(const nav_msgs::msg::Odometry::SharedPtr msg);
  void CallbackAvoidanceWaypoints(const f1tenth_planning_msgs::msg::WpntArray::SharedPtr msg);
  int FindClosestWaypointIndex(
    double s_coordinate,
    const std::vector<f1tenth_planning_msgs::msg::Wpnt> & global_waypoints,
    double track_length);
  void Run();
  void PublishLocalWaypoints(
    const std::vector<f1tenth_planning_msgs::msg::Wpnt> & global_waypoints_copy,
    const nav_msgs::msg::Odometry & frenet_odom_copy);
  // 회피 궤적이 유효하면 로컬 윈도우의 겹치는 s 구간을 치환한다.
  // false는 입력 궤적이 비정상임을 뜻하며 이 주기의 local path를 발행하지 않는다.
  // replaced_out 에는 실제로 치환된 웨이포인트 수가 담긴다 (pitwall 텔레메트리용).
  bool SpliceAvoidanceWaypoints(
    std::vector<f1tenth_planning_msgs::msg::Wpnt> & local_wpnts,
    const std::vector<f1tenth_planning_msgs::msg::Wpnt> & avoidance_wpnts,
    double track_length,
    int & replaced_out);

  // Parameters
  double loop_rate_hz_{40.0};
  int num_local_waypoints_{80};
  double avoidance_timeout_sec_{0.5};
  bool require_avoidance_heartbeat_{true};
  double frenet_odom_timeout_sec_{0.20};
  double frenet_odom_source_timeout_sec_{0.20};
  double future_stamp_tolerance_sec_{0.05};
  // frenet_frame/min_frenet_jacobian 과 같은 값이어야 한다. 예전엔 여기만
  // 리터럴 0.20 이라, frenet 쪽만 내리면 frenet 은 계속 발행하는데 local 이
  // 거부해 /local_waypoints 가 끊겼다 (원인 노드를 특정하기 어려운 정지).
  double min_frenet_jacobian_{0.20};
  double max_avoidance_step_m_{1.0};
  double minimum_horizon_sec_{1.5};
  double minimum_braking_decel_mps2_{2.5};
  double minimum_stop_buffer_m_{1.5};

  // State
  std::vector<f1tenth_planning_msgs::msg::Wpnt> global_waypoints_;
  bool has_global_waypoints_{false};
  nav_msgs::msg::Odometry current_frenet_odom_;
  bool has_frenet_odom_{false};
  std::chrono::steady_clock::time_point frenet_odom_arrival_time_{};
  int64_t last_frenet_source_stamp_ns_{0};
  std::vector<f1tenth_planning_msgs::msg::Wpnt> avoidance_waypoints_;
  bool has_avoidance_message_{false};
  std::chrono::steady_clock::time_point avoidance_arrival_time_{};
  int64_t avoidance_source_stamp_ns_{0};
  int64_t last_avoidance_source_stamp_ns_{0};
  std::mutex mutex_global_waypoints_;
  std::mutex mutex_frenet_odom_;
  std::mutex mutex_avoidance_;

  rclcpp::Subscription<f1tenth_planning_msgs::msg::WpntArray>::SharedPtr s_global_waypoints_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr s_frenet_odom_;
  rclcpp::Subscription<f1tenth_planning_msgs::msg::WpntArray>::SharedPtr s_avoidance_waypoints_;
  rclcpp::Publisher<f1tenth_planning_msgs::msg::WpntArray>::SharedPtr p_local_waypoints_;
  rclcpp::TimerBase::SharedPtr t_publish_;
};

#endif  // PLANNING_PKG__LOCAL_TRAJECTORY_NODE_HPP_
