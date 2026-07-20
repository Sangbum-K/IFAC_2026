// Task 4: 벽 충돌 / 진행 불능(stuck) 상태 판정 및 복구 감독 노드.
//
// 책임 분리 (REAL_CAR_PLANNING_TASKS.md):
//  - Planning(이 노드): stuck/충돌 판정, 복구용 후진·재정렬 "요청"과 정지 요청 생성
//  - Control(control_node): 최종 속도·조향, 후진 하드 리밋, E-stop 우선 처리
//  - Safety: /safety/emergency_stop=true는 항상 최우선 -> 즉시 STOP
//
// 출력 계약 (control_node가 구독):
//  - /planner/recovery/state       std_msgs/String  : 현재 상태 문자열 (진단/RViz)
//  - /planner/recovery/target_speed std_msgs/Float64: 요청 속도 [m/s]
//      DRIVING          -> NaN (override 없음, Control은 무시)
//      COLLISION_STOP/STOP/STUCK_DETECTED -> 0.0
//      RECOVER_REVERSE  -> -recovery_reverse_speed_mps (Control이 자체 리밋 재적용)
//      RECOVER_ALIGN    -> +align_speed_mps
//  - STOP은 래치 상태: /planner/recovery/reset (std_msgs/Bool true)로만 해제.
//
// 상태 전이는 작업명세서 Task 4 다이어그램과 동일:
// DRIVING -> STUCK_DETECTED / COLLISION_STOP
// STUCK_DETECTED -> RECOVER_REVERSE(후방 안전 시) / STOP
// RECOVER_REVERSE -> RECOVER_ALIGN / STOP
// RECOVER_ALIGN -> DRIVING / STOP
#ifndef PLANNING_PKG__RECOVERY_SUPERVISOR_NODE_HPP_
#define PLANNING_PKG__RECOVERY_SUPERVISOR_NODE_HPP_

#include <cmath>
#include <mutex>
#include <string>
#include <vector>

#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/string.hpp"
#include "visualization_msgs/msg/marker.hpp"

#include "f1tenth_planning_msgs/msg/obstacle_array.hpp"
#include "f1tenth_planning_msgs/msg/wpnt_array.hpp"

class RecoverySupervisorNode : public rclcpp::Node
{
public:
  explicit RecoverySupervisorNode(
    const std::string & node_name,
    const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  enum class State {
    DRIVING, STUCK_DETECTED, COLLISION_STOP,
    RECOVER_REVERSE, RECOVER_ALIGN, STOP
  };
  static const char * StateName(State s);

  void CallbackFrenetOdom(const nav_msgs::msg::Odometry::SharedPtr msg);
  void CallbackPoseOdom(const nav_msgs::msg::Odometry::SharedPtr msg);
  void CallbackLocalWaypoints(const f1tenth_planning_msgs::msg::WpntArray::SharedPtr msg);
  void CallbackObstacles(const f1tenth_planning_msgs::msg::ObstacleArray::SharedPtr msg);
  void CallbackEStop(const std_msgs::msg::Bool::SharedPtr msg);
  void CallbackReset(const std_msgs::msg::Bool::SharedPtr msg);
  void Run();

  // 판정 함수
  bool IsStuck(double now_sec);                 // 전진 명령 대비 s 진행량 부족
  bool IsBoundaryViolation() const;             // 트랙 경계 여유 < 임계값
  bool IsRearSafe() const;                      // 후방 rear_check 구간 장애물 없음
  bool IsHeadingAligned() const;                // 차량 yaw vs local path psi
  bool InputsStale(double now_sec) const;       // odom/perception timeout

  void TransitionTo(State next, const std::string & reason);
  void PublishOutputs();

  // Parameters (real_car.yaml에서 보수값 override)
  double loop_rate_hz_{20.0};
  double stuck_timeout_sec_{1.0};
  double stuck_min_progress_m_{0.10};
  double min_forward_cmd_mps_{0.2};
  double boundary_stop_margin_m_{0.15};
  // 경계 위반이 이 시간 이상 "지속"돼야 COLLISION_STOP (통합 시 추가).
  // v15 raceline은 코너 벽 여유 ~0.3m + 추적오차 ~0.2m라 순간 스파이크가
  // 정상 주행에서도 발생 — 1사이클 오탐으로 정지하면 안 된다.
  double boundary_persist_sec_{0.2};
  double rear_check_dist_m_{1.5};
  double recovery_reverse_speed_mps_{0.5};
  double recovery_max_reverse_distance_m_{0.5};
  double recovery_timeout_sec_{4.0};
  int recovery_max_attempts_{2};
  double align_tolerance_rad_{0.35};
  double align_speed_mps_{0.5};
  double odom_timeout_sec_{0.5};
  double perception_timeout_sec_{0.5};

  // State
  State state_{State::DRIVING};
  int recovery_attempts_{0};
  double stuck_since_sec_{-1.0};
  double boundary_since_sec_{-1.0};
  double stuck_ref_s_{0.0};
  double recover_start_sec_{0.0};
  double recover_start_s_{0.0};
  bool estop_{false};
  bool reset_requested_{false};

  double cur_s_{0.0};
  double cur_d_{0.0};
  double cur_vs_{0.0};
  double cur_yaw_{0.0};
  f1tenth_planning_msgs::msg::WpntArray local_wpnts_;
  f1tenth_planning_msgs::msg::ObstacleArray obstacles_;
  rclcpp::Time frenet_stamp_{0, 0, RCL_ROS_TIME};
  rclcpp::Time pose_stamp_{0, 0, RCL_ROS_TIME};
  rclcpp::Time obstacles_stamp_{0, 0, RCL_ROS_TIME};
  bool has_frenet_{false};
  bool has_local_{false};
  double gb_max_s_{1e9};
  std::mutex mutex_;

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr s_frenet_odom_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr s_pose_odom_;
  rclcpp::Subscription<f1tenth_planning_msgs::msg::WpntArray>::SharedPtr s_local_wpnts_;
  rclcpp::Subscription<f1tenth_planning_msgs::msg::ObstacleArray>::SharedPtr s_obstacles_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr s_estop_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr s_reset_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr p_state_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr p_target_speed_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr p_marker_;
  rclcpp::TimerBase::SharedPtr t_run_;
};

#endif  // PLANNING_PKG__RECOVERY_SUPERVISOR_NODE_HPP_
