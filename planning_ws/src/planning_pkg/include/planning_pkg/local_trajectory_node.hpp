// 원본 planner_pkg에 헤더가 누락되어 있어 cpp 구현부로부터 복원 작성함.
// 현재 s 주변에서 global waypoints를 80개 슬라이스해 /local_waypoints로 발행.
// 회피: spliner_node가 발행하는 /planner/avoidance/otwpnts가 유효(비어있지
// 않고 신선)하면 로컬 윈도우의 해당 s 구간을 회피 궤적으로 치환(splice)한다
// — ForzaETH(unicorn) 스택의 state_machine splice 역할.
#ifndef PLANNING_PKG__LOCAL_TRAJECTORY_NODE_HPP_
#define PLANNING_PKG__LOCAL_TRAJECTORY_NODE_HPP_

#include <cmath>
#include <limits>
#include <mutex>
#include <string>
#include <vector>

#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"

#include "ae_hyu_msgs/msg/wpnt.hpp"
#include "ae_hyu_msgs/msg/wpnt_array.hpp"

class LocalTrajectoryNode : public rclcpp::Node
{
public:
  explicit LocalTrajectoryNode(
    const std::string & node_name,
    const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  ~LocalTrajectoryNode() override;

private:
  void CallbackGlobalWaypoints(const ae_hyu_msgs::msg::WpntArray::SharedPtr msg);
  void CallbackFrenetOdom(const nav_msgs::msg::Odometry::SharedPtr msg);
  void CallbackAvoidanceWaypoints(const ae_hyu_msgs::msg::WpntArray::SharedPtr msg);
  int FindClosestWaypointIndex(
    double s_coordinate,
    const std::vector<ae_hyu_msgs::msg::Wpnt> & global_waypoints);
  void Run();
  void PublishLocalWaypoints(
    const std::vector<ae_hyu_msgs::msg::Wpnt> & global_waypoints_copy,
    const nav_msgs::msg::Odometry & frenet_odom_copy);
  // 회피 궤적이 유효하면 로컬 윈도우의 겹치는 s 구간을 치환
  void SpliceAvoidanceWaypoints(
    std::vector<ae_hyu_msgs::msg::Wpnt> & local_wpnts,
    const std::vector<ae_hyu_msgs::msg::Wpnt> & avoidance_wpnts,
    double track_length);

  // Parameters
  double loop_rate_hz_{40.0};
  int num_local_waypoints_{80};
  double avoidance_timeout_sec_{0.5};

  // State
  std::vector<ae_hyu_msgs::msg::Wpnt> global_waypoints_;
  bool has_global_waypoints_{false};
  nav_msgs::msg::Odometry current_frenet_odom_;
  bool has_frenet_odom_{false};
  std::vector<ae_hyu_msgs::msg::Wpnt> avoidance_waypoints_;
  rclcpp::Time avoidance_stamp_{0, 0, RCL_ROS_TIME};
  int last_closest_index_{0};
  std::mutex mutex_global_waypoints_;
  std::mutex mutex_frenet_odom_;
  std::mutex mutex_avoidance_;

  rclcpp::Subscription<ae_hyu_msgs::msg::WpntArray>::SharedPtr s_global_waypoints_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr s_frenet_odom_;
  rclcpp::Subscription<ae_hyu_msgs::msg::WpntArray>::SharedPtr s_avoidance_waypoints_;
  rclcpp::Publisher<ae_hyu_msgs::msg::WpntArray>::SharedPtr p_local_waypoints_;
  rclcpp::TimerBase::SharedPtr t_publish_;
};

#endif  // PLANNING_PKG__LOCAL_TRAJECTORY_NODE_HPP_
