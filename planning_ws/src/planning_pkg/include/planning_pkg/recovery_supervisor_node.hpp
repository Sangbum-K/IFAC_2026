// Task 4: 벽 충돌 / 진행 불능(stuck) 상태 판정 및 복구 감독 노드.
//
// 책임 분리 (REAL_CAR_PLANNING_TASKS.md):
//  - Planning(이 노드): stuck/충돌 판정, 복구용 후진·재정렬 "요청"과 정지 요청 생성
//  - Control(control_node): 최종 속도·조향, 후진 하드 리밋, E-stop 우선 처리
//  - Safety: /safety/emergency_stop=true는 항상 최우선 -> 즉시 STOP
//
// 단독 노드의 `recovery/enabled` 기본값은 false다. planner_stack의 안전
// 프로파일은 true로 기동하고 control과 아래 state/target lease를 함께 소비한다.
//
// control_node는 아래 state/target_speed를 한 쌍의 lease로 소비한다. 한쪽이
// 끊기거나 오래되거나 계약과 맞지 않으면 fail-closed로 정지한다.
//
// 출력 계약:
//  - /planner/recovery/state       std_msgs/String  : 현재 상태 문자열 (진단/RViz)
//  - /planner/recovery/target_speed std_msgs/Float64: 요청 속도 [m/s]
//      WAITING_FOR_INPUTS -> 0.0
//      DRIVING          -> NaN (override 없음, Control은 무시)
//      COLLISION_STOP/STOP/STUCK_DETECTED -> 0.0
//      RECOVER_REVERSE  -> -recovery_reverse_speed_mps
//      RECOVER_STOP     -> 0.0 (방향 전환 전 실제 정지 dwell)
//      RECOVER_ALIGN    -> +align_speed_mps
// 후진은 전용 조향 경로가 없으므로 기본적으로 비활성이다. Planning과 Control의
// allow_reverse를 둘 다 명시적으로 켜고 후방 coverage를 확인한 경우에만 전달된다.
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
    WAITING_FOR_INPUTS, DRIVING, STUCK_DETECTED, COLLISION_STOP,
    RECOVER_REVERSE, RECOVER_STOP, RECOVER_ALIGN, STOP
  };
  static const char * StateName(State s);

  void CallbackFrenetOdom(const nav_msgs::msg::Odometry::SharedPtr msg);
  void CallbackPoseOdom(const nav_msgs::msg::Odometry::SharedPtr msg);
  void CallbackGlobalWaypoints(const f1tenth_planning_msgs::msg::WpntArray::SharedPtr msg);
  void CallbackLocalWaypoints(const f1tenth_planning_msgs::msg::WpntArray::SharedPtr msg);
  void CallbackObstacles(const f1tenth_planning_msgs::msg::ObstacleArray::SharedPtr msg);
  void CallbackEStop(const std_msgs::msg::Bool::SharedPtr msg);
  void CallbackReset(const std_msgs::msg::Bool::SharedPtr msg);
  void Run();

  // 판정 함수
  bool IsStuck(double now_sec);                 // 전진 명령 대비 s 진행량 부족
  bool IsCurrentBoundaryViolation() const;      // 현재 footprint가 안전 경계를 침범
  bool IsBoundaryViolation() const;             // 제동거리 내 경로의 footprint 여유 부족
  bool IsRearSafe(double now_sec) const;         // 후방 coverage + 장애물 없음
  bool IsFrontSafe(double now_sec) const;        // 정렬 중 전방 장애물 없음
  bool IsHeadingAligned() const;                // 차량 yaw vs local path psi
  bool InputsReady(
    const rclcpp::Time & now, std::string * reason = nullptr) const;
  bool StampFresh(
    const rclcpp::Time & now, const rclcpp::Time & receipt_stamp,
    const rclcpp::Time & source_stamp, double timeout_sec) const;
  bool IsStoppedForDwell(double now_sec);
  double StoppingDistance() const;

  void TransitionTo(State next, const std::string & reason);
  void PublishOutputs();

  // Parameters (real_car.yaml에서 보수값 override)
  double loop_rate_hz_{20.0};
  double stuck_timeout_sec_{1.0};
  double stuck_min_progress_m_{0.10};
  double min_forward_cmd_mps_{0.2};
  double boundary_stop_margin_m_{0.15};
  double boundary_persist_sec_{0.2};
  double vehicle_width_m_{0.30};
  double vehicle_length_m_{0.55};
  double boundary_reaction_time_sec_{0.20};
  double boundary_braking_decel_mps2_{4.0};
  bool require_boundary_horizon_{true};
  double rear_check_dist_m_{1.5};
  double front_check_dist_m_{1.5};
  bool allow_reverse_{false};
  bool rear_coverage_confirmed_{false};
  double recovery_reverse_speed_mps_{0.5};
  double recovery_max_reverse_distance_m_{0.5};
  double recovery_timeout_sec_{4.0};
  int recovery_max_attempts_{2};
  double align_tolerance_rad_{0.35};
  double align_speed_mps_{0.5};
  double odom_timeout_sec_{0.5};
  double perception_timeout_sec_{0.5};
  double local_wpnts_timeout_sec_{0.5};
  double source_stamp_future_tolerance_sec_{0.1};
  bool require_source_stamps_{true};
  double readiness_dwell_sec_{0.2};
  double stopped_speed_threshold_mps_{0.05};
  double stopped_dwell_sec_{0.2};

  // State
  State state_{State::WAITING_FOR_INPUTS};
  int recovery_attempts_{0};
  double stuck_since_sec_{-1.0};
  double boundary_since_sec_{-1.0};
  double ready_since_sec_{-1.0};
  double stopped_since_sec_{-1.0};
  double stuck_ref_s_{0.0};
  double recover_start_sec_{0.0};
  double recover_start_s_{0.0};
  bool estop_{false};
  bool reset_requested_{false};

  double cur_s_{0.0};
  double cur_d_{0.0};
  double cur_vs_{0.0};
  double cur_yaw_{0.0};
  double cur_speed_norm_{0.0};
  f1tenth_planning_msgs::msg::WpntArray local_wpnts_;
  f1tenth_planning_msgs::msg::ObstacleArray obstacles_;
  rclcpp::Time frenet_receipt_stamp_{0, 0, RCL_ROS_TIME};
  rclcpp::Time frenet_source_stamp_{0, 0, RCL_ROS_TIME};
  rclcpp::Time pose_receipt_stamp_{0, 0, RCL_ROS_TIME};
  rclcpp::Time pose_source_stamp_{0, 0, RCL_ROS_TIME};
  rclcpp::Time obstacles_receipt_stamp_{0, 0, RCL_ROS_TIME};
  rclcpp::Time obstacles_source_stamp_{0, 0, RCL_ROS_TIME};
  rclcpp::Time local_wpnts_receipt_stamp_{0, 0, RCL_ROS_TIME};
  rclcpp::Time local_wpnts_source_stamp_{0, 0, RCL_ROS_TIME};
  bool has_frenet_{false};
  bool has_pose_{false};
  bool has_global_{false};
  bool has_local_{false};
  bool has_obstacles_{false};
  // 트랙 한 바퀴 길이. /global_waypoints에서 (마지막 s + 간격)으로 받는다 —
  // 로컬 윈도우 s의 러닝 맥스로 추정하면 첫 랩 동안 실제보다 작아서
  // wrap 계산(IsRearSafe, 진행량)이 틀린다.
  double track_len_{0.0};
  std::mutex mutex_;

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr s_frenet_odom_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr s_pose_odom_;
  rclcpp::Subscription<f1tenth_planning_msgs::msg::WpntArray>::SharedPtr s_global_wpnts_;
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
