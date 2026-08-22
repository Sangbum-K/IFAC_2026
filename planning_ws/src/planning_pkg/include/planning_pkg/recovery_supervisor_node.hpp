// Task 4: 벽 충돌 / 진행 불능(stuck) 상태 판정 및 복구 감독 노드.
//
// 책임 분리 (REAL_CAR_PLANNING_TASKS.md):
//  - Planning(이 노드): stuck/충돌 판정, 복구용 후진 "요청"과 정지 요청 생성
//  - Control(control_node): 최종 속도·조향, 후진 하드 리밋, E-stop 우선 처리
//  - Safety: /safety/emergency_stop=true는 항상 최우선 -> 즉시 STOP
//
// 단독 노드의 `recovery/enabled` 기본값은 false다. planner_stack의 launch가
// recovery:=auto(기본, mapping 제외 on)로 기동하고 control과 아래 state/target
// lease를 함께 소비한다.
//
// control_node는 아래 state/target_speed를 한 쌍의 lease로 소비한다. 한쪽이
// 끊기거나 오래되거나 계약과 맞지 않으면 fail-closed로 정지한다.
//
// 출력 계약:
//  - /planner/recovery/state       std_msgs/String  : 현재 상태 문자열 (진단/RViz)
//  - /planner/recovery/target_speed std_msgs/Float64: 요청 속도 [m/s]
//      WAITING_FOR_INPUTS -> 0.0
//      DRIVING          -> NaN (override 없음, Control은 무시)
//      RECOVER_REVERSE  -> -recovery_reverse_speed_mps
//      RECOVER_STOP     -> 0.0 (방향 전환 전 실제 정지 dwell)
//      STOP             -> 0.0 (래치)
//
// 발동 정책 (2026-08-22, run_0822_191159 실차 근거로 축소):
//  이 노드는 "완전히 박았을 때"만 개입한다. 세 조건을 모두 만족해야 후진한다:
//    ① IsStuck            — 전진 명령(head vx >= min_forward_cmd)인데
//                           stuck_timeout 동안 s 진행이 없음
//    ② IsFrontalWallContact — 노즈가 코리도 경계에 닿음
//                           (nose_margin <= wall_contact_margin_m)
//    ③ IsStoppedForDwell  — 실제 평면 속도가 dwell 동안 0 근처
//  run_0822_191159 검증: 진짜 벽 접촉은 nose_margin -0.00/-0.38m, 오탐 사례는
//  0.31m(코너 감속)과 0.55m(출발 스톨)이라 0.05m 임계로 분리된다.
//
//  ⚠ 삭제된 경로 — 되살리지 말 것:
//   - 경계 예측 정지(예전 COLLISION-STOP 경로): 레이스라인
//     에이펙스 여유 min(d_left,d_right)=0.445m 를 부호 없는 횡오차와 비교해
//     정상 주행을 오탐했다. 차의 실제 여유는 0.84m 이상이었는데도 3회 모두
//     트립했고, 그때 control 의 publish_stop 이 조향까지 0으로 꺾어(코너에서
//     16.6° -> 0°, 3.3 m/s) 차를 바깥 벽으로 직진시켰다. 즉 리커버리가
//     충돌을 만들고 그 충돌을 복구했다.
//   - 후진 완료 후의 정렬(ALIGN) 단계: 복귀 조향/속도는 정상 체인
//     (local_trajectory 최근접 s 재생성 + control L1)이 담당한다.
//
//  후진은 후방 확인 없이 직진으로 recovery_max_reverse_distance_m 만큼 한다
//  (LiDAR 후방 미커버 — 상대차/사람이 있는 세션에서는 allow_reverse를 끌 것).
//  시도는 무제한이되 "같은 지점 연속" recovery_max_attempts 회 실패 시 STOP
//  래치. DRIVING에서 progress_reset_m 이상 전진하면 카운터가 리셋된다.
//
// 상태 전이:
// DRIVING -> RECOVER_REVERSE (①②③ 모두, attempts++)
//         -> STOP (allow_reverse=false 킬스위치 / 연속 실패 한도 초과)
// RECOVER_REVERSE -> RECOVER_STOP (유클리드 후진거리 도달 또는 timeout)
// RECOVER_STOP -> DRIVING (정지 dwell)
//  - STOP은 래치 상태: /planner/recovery/reset (std_msgs/Bool true)로만 해제.
//  - E-stop과 입력 stale은 어느 상태에서든 STOP 래치 (불변).
#ifndef PLANNING_PKG__RECOVERY_SUPERVISOR_NODE_HPP_
#define PLANNING_PKG__RECOVERY_SUPERVISOR_NODE_HPP_

#include <mutex>
#include <string>

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
    WAITING_FOR_INPUTS, DRIVING, RECOVER_REVERSE, RECOVER_STOP, STOP
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
  // 노즈(전방 half_length 투영)가 코리도 경계에 닿았는지 = "완전히 박았다".
  // 후진의 필수 게이트다 (run_0822_191159: 진짜 접촉 -0.00m vs 오탐 0.31/0.55m).
  bool IsFrontalWallContact(double * nose_margin, double * heading_err) const;
  bool InputsReady(
    const rclcpp::Time & now, std::string * reason = nullptr) const;
  bool StampFresh(
    const rclcpp::Time & now, const rclcpp::Time & receipt_stamp,
    const rclcpp::Time & source_stamp, double timeout_sec) const;
  bool IsStoppedForDwell(double now_sec);

  void TransitionTo(State next, const std::string & reason, double now_sec);
  void PublishOutputs();

  // Parameters (real_car.yaml에서 보수값 override)
  double loop_rate_hz_{20.0};
  double stuck_timeout_sec_{1.0};
  double stuck_min_progress_m_{0.10};
  double min_forward_cmd_mps_{0.2};
  double vehicle_width_m_{0.30};
  double vehicle_length_m_{0.55};
  double wall_contact_margin_m_{0.05};
  bool allow_reverse_{false};
  double recovery_reverse_speed_mps_{0.5};
  double recovery_max_reverse_distance_m_{0.5};
  double recovery_timeout_sec_{4.0};
  int recovery_max_attempts_{3};
  double progress_reset_m_{3.0};
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
  double ready_since_sec_{-1.0};
  double stopped_since_sec_{-1.0};
  double stuck_ref_s_{0.0};
  double recover_start_sec_{0.0};
  double recover_start_x_{0.0};
  double recover_start_y_{0.0};
  // 마지막 복구 시작 지점의 s. DRIVING에서 여기서부터 progress_reset_m 이상
  // 전진하면 연속 실패 카운터가 리셋된다. -1 = 미복구 상태.
  double last_recover_s_{-1.0};
  bool estop_{false};
  bool reset_requested_{false};

  double cur_s_{0.0};
  double cur_d_{0.0};
  double cur_x_{0.0};
  double cur_y_{0.0};
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
  // wrap 계산(진행량)이 틀린다.
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
