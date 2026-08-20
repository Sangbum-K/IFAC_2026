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
#include "std_msgs/msg/float32.hpp"

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
  void CallbackDynSpeedFloor(const std_msgs::msg::Float32::SharedPtr msg);
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
  // ego_d 는 진단 로그 전용이다 (2026-08-14). 스플라이스가 선두를 못 덮는
  // 것 자체는 정상일 수 있고(회피 경로가 앞에서 시작), 해로운 경우는
  // "차는 라인을 벗어나 있는데 따라갈 경로는 라인 위" 하나뿐이라
  // 그 판정에 차의 현재 d 가 필요하다. 동작에는 쓰지 않는다.
  bool SpliceAvoidanceWaypoints(
    std::vector<f1tenth_planning_msgs::msg::Wpnt> & local_wpnts,
    const std::vector<f1tenth_planning_msgs::msg::Wpnt> & avoidance_wpnts,
    double track_length,
    int & replaced_out,
    double ego_d);

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
  // 회피 경로가 빈 배열로 바뀐 뒤 직전 속도 상한을 유지할 시간 [s].
  // 0 이면 홀드 없음(옛 fail-open). 근거는 cpp 의 declare_parameter 주석.
  double avoidance_loss_hold_sec_{0.5};
  // 홀드 상한의 바닥 [m/s]. 0 = 비활성 (선언부 주석 참조).
  double avoidance_loss_hold_floor_{1.5};
  // spliner 가 보내는 동적 속도 바닥에 걸 상한 [m/s]. 위 상수 바닥의 검증
  // 상한(3.0)과 같은 역할이며, 인지가 상대 속도를 크게 잘못 보고해도
  // 홀드가 통째로 무효가 되지 않게 한다.
  double avoidance_loss_hold_floor_max_{3.0};

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
  // 회피 상실 홀드용: 직전 **비어있지 않은** 회피 경로의 선두 속도와 그 시각.
  double last_avoidance_head_speed_{-1.0};
  std::chrono::steady_clock::time_point last_avoidance_nonempty_time_{};
  bool has_avoidance_head_speed_{false};
  // spliner /planner/avoidance/speed_floor 의 최신 수신값 (콜백이 쓴다).
  double dyn_speed_floor_rx_{0.0};
  // 위 값을 **회피 경로가 아직 살아 있던 시점에** 걸어 둔 것. 홀드는 경로가
  // 사라진 뒤에 도는데, 그때 상대도 같이 사라져 0 이 흘러오면 바닥이 없어진다.
  // 그래서 선두 속도(last_avoidance_head_speed_)와 **같은 순간에 같이** 건다.
  double last_dyn_speed_floor_{0.0};
  std::mutex mutex_global_waypoints_;
  std::mutex mutex_frenet_odom_;
  std::mutex mutex_avoidance_;
  std::mutex mutex_dyn_speed_floor_;

  rclcpp::Subscription<f1tenth_planning_msgs::msg::WpntArray>::SharedPtr s_global_waypoints_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr s_frenet_odom_;
  rclcpp::Subscription<f1tenth_planning_msgs::msg::WpntArray>::SharedPtr s_avoidance_waypoints_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr s_dyn_speed_floor_;
  rclcpp::Publisher<f1tenth_planning_msgs::msg::WpntArray>::SharedPtr p_local_waypoints_;
  rclcpp::TimerBase::SharedPtr t_publish_;
};

#endif  // PLANNING_PKG__LOCAL_TRAJECTORY_NODE_HPP_
