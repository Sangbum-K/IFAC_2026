// ForzaETH(unicorn-racing-stack) planner/spliner/spliner_node.py의 C++ 포팅.
// 장애물(ObstacleArray) 주변에 pre/post-apex 7점을 잡아 자연 3차 스플라인으로
// d(s) 회피 궤적을 만들고 /planner/avoidance/otwpnts로 발행한다.
// 회피 불가(트랙 경계 근접, 사이드 스위치 불가) 시 빈 배열을 발행하며,
// local_trajectory_node가 이를 로컬 윈도우에 병합(splice)한다.
#ifndef PLANNING_PKG__SPLINER_NODE_HPP_
#define PLANNING_PKG__SPLINER_NODE_HPP_

#include <cmath>
#include <mutex>
#include <string>
#include <vector>

#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

#include "ae_hyu_msgs/msg/obstacle.hpp"
#include "ae_hyu_msgs/msg/obstacle_array.hpp"
#include "ae_hyu_msgs/msg/wpnt.hpp"
#include "ae_hyu_msgs/msg/wpnt_array.hpp"

class SplinerNode : public rclcpp::Node
{
public:
  explicit SplinerNode(
    const std::string & node_name,
    const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  void CallbackObstacles(const ae_hyu_msgs::msg::ObstacleArray::SharedPtr msg);
  void CallbackPredictedObstacles(const ae_hyu_msgs::msg::ObstacleArray::SharedPtr msg);
  void CallbackFrenetOdom(const nav_msgs::msg::Odometry::SharedPtr msg);
  void CallbackGlobalWaypoints(const ae_hyu_msgs::msg::WpntArray::SharedPtr msg);
  void Run();

  // ForzaETH spliner 핵심 로직
  std::vector<ae_hyu_msgs::msg::Obstacle> FilterObstacles(
    const std::vector<ae_hyu_msgs::msg::Obstacle> & obstacles) const;
  ae_hyu_msgs::msg::Obstacle PredictObstacleMovement(
    ae_hyu_msgs::msg::Obstacle obs) const;
  // 반환: {side("left"/"right"), d_apex}
  std::pair<std::string, double> MoreSpace(
    const ae_hyu_msgs::msg::Obstacle & obs, int gb_idx) const;
  // 해당 사이드의 apex d 계산: 목표는 장애물 가장자리 + evasion_dist지만,
  // corridor(장애물-벽)가 좁으면 여유를 장애물/벽 양쪽에 균등 배분한
  // 지점으로 축소 (벽에 최대한 붙이는 방식은 추적오차로 벽 접촉)
  double ApexFor(
    const std::string & side,
    const ae_hyu_msgs::msg::Obstacle & obs, int gb_idx) const;
  // forced_side: ""이면 MoreSpace가 결정, "left"/"right"면 해당 사이드 강제
  // (이중장애물 재계획 시 사용)
  bool DoSpline(
    const ae_hyu_msgs::msg::Obstacle & closest_obs,
    ae_hyu_msgs::msg::WpntArray & out_wpnts,
    visualization_msgs::msg::MarkerArray & out_mrks,
    const std::string & forced_side = "");

  // --- 이중장애물 재계획 / 안전 정지 (mina Task 2 병합) ---
  // 후보 경로를 lookahead 내 "모든" 장애물과 (s,d) 충돌 검사.
  // exclude_id: 회피/추종 대상 장애물 자신은 제외 (경로가 의도적으로
  // 그 주변/뒤를 지나므로). 반환 true=충돌 없음, false=충돌(blocking_s에
  // 막은 장애물의 s_start).
  bool CheckPathCollision(
    const std::vector<ae_hyu_msgs::msg::Wpnt> & path,
    const std::vector<ae_hyu_msgs::msg::Obstacle> & obstacles,
    int32_t exclude_id, double * blocking_s) const;
  // lookahead 내 전체 장애물 (raceline 근접 여부 무관 — 경로 검증용)
  std::vector<ae_hyu_msgs::msg::Obstacle> ObstaclesInLookahead(
    const std::vector<ae_hyu_msgs::msg::Obstacle> & obstacles) const;
  // 사이를 통과할 수 없을 만큼 가까운 장애물들(s 간격 < cluster_gap_s,
  // d corridor < cluster_gap_d)을 하나의 (s,d) 박스로 병합.
  // 대각선 이중장애물을 개별로 보면 최근접 기준 고정 폭 회피만 시도하다
  // 트랙경계 abort로 포기한다 — 병합해야 "바깥으로 크게 도는" 해가 나온다.
  std::vector<ae_hyu_msgs::msg::Obstacle> ClusterObstacles(
    std::vector<ae_hyu_msgs::msg::Obstacle> obstacles) const;
  // 좌·우 회피와 trailing까지 모두 막혔을 때: 차단 지점 앞 stop_margin에서
  // vx=0이 되는 정지 경로 생성 (최후 폴백)
  void BuildStopWpnts(
    double blocking_s,
    ae_hyu_msgs::msg::WpntArray & out_wpnts,
    visualization_msgs::msg::MarkerArray & out_mrks);

  // 회피 불가 시 폴백: raceline 유지 + 장애물 뒤 trailing_gap에서
  // 장애물 속도까지 감속하는 궤적 (ForzaETH 상태머신 TRAILING 대응)
  bool BuildTrailing(
    const ae_hyu_msgs::msg::Obstacle & closest_obs,
    ae_hyu_msgs::msg::WpntArray & out_wpnts,
    visualization_msgs::msg::MarkerArray & out_mrks);

  // 추월 섹터: 회피 기하(벽 여유 >= ot_min_clearance)가 성립하는 s 구간을
  // raceline에서 자동 산출. 섹터 안에서만 회피를 시도하고(밖은 TRAILING 대기)
  // 섹터 안에서는 경계 마진 완화 + 속도 부스트로 공격적으로 추월한다.
  void ComputeOtSectors();
  bool InOtSector(double s) const;
  void PublishSectorMarkers();

  // (s, d) -> (x, y). 균일 간격 global waypoints 기반 선형 보간.
  std::pair<double, double> FrenetToCartesian(double s, double d) const;

  // 자연 3차 스플라인: 절점 (xs, ys)를 지나는 곡선을 query 지점들에서 평가
  static std::vector<double> NaturalCubicSpline(
    const std::vector<double> & xs, const std::vector<double> & ys,
    const std::vector<double> & query);

  visualization_msgs::msg::Marker MakeWpntMarker(
    double x, double y, double v, int id) const;

  // Parameters (ForzaETH dyn_spliner_tuner.cfg 기본값)
  double loop_rate_hz_{20.0};
  double lookahead_m_{10.0};
  double evasion_dist_{0.65};
  double obs_traj_tresh_{0.3};
  double spline_bound_mindist_{0.2};
  std::vector<double> pre_apex_dists_{-4.0, -3.0, -1.5};
  std::vector<double> post_apex_dists_{2.0, 3.0, 4.0};
  double fixed_pred_time_{0.15};
  double outside_spline_scale_{1.75};
  double inside_speed_scale_{0.9};
  double spline_resolution_{0.1};
  double side_switch_d_tresh_{0.25};
  double obs_timeout_sec_{0.5};
  bool use_prediction_{true};   // Predictive Spliner: 예측 장애물이 원시 관측을 대체
  double trailing_gap_m_{1.5};
  double trailing_decel_{3.0};
  double cmd_speed_scale_{1.0};
  bool trailing_active_{false};
  // 이중장애물 재계획 파라미터 (mina Task 2)
  double path_check_margin_{0.15};   // 경로-장애물 충돌 판정 횡방향 여유 [m]
  double cluster_gap_s_m_{3.0};      // 이 s 간격 이내 장애물은 병합 후보 [m]
  double cluster_gap_d_m_{0.8};      // 사이 corridor가 이보다 좁으면 병합 [m]
  // 벽이 좁아 evasion_dist를 다 못 챙길 때 apex를 벽 마진 안으로 줄여서라도
  // 회피를 시도하는 최소 장애물 간격 (차 반폭 ~0.15 + 여유)
  double evasion_dist_min_{0.35};
  // apex가 벽 마진으로 축소(squeeze)된 회피는 이 배율로 감속 — 좁은
  // corridor를 섹터 부스트 속도로 지나면 추적오차(속도비례)로 벽에 닿는다
  double squeeze_speed_scale_{0.35};
  double stop_margin_m_{1.0};        // 정지 폴백 시 차단 지점 앞 안전거리 [m]
  double stop_ramp_dist_m_{2.5};     // 정지점 앞 최종 선형 감속 구간 길이 [m]
  bool stop_active_{false};

  // 추월 섹터 파라미터
  bool sector_gating_{true};
  double ot_min_clearance_{0.95};
  double ot_sector_min_len_m_{8.0};
  double ot_bound_mindist_{0.25};
  double ot_speed_scale_{1.1};
  std::vector<double> ot_sectors_param_;                 // 수동 지정 (flat s0,s1,...)
  std::vector<std::pair<double, double>> ot_sectors_;    // [s_start, s_end] (wrap 허용)
  bool in_ot_sector_{false};
  // 진행 중인 정적 회피 커밋 (경로 끝까지 유지 — 도중 소멸 시 벽 충돌)
  ae_hyu_msgs::msg::WpntArray committed_wpnts_;
  visualization_msgs::msg::MarkerArray committed_mrks_;
  int32_t committed_id_{-1};
  double committed_start_s_{-1.0};
  double committed_until_s_{-1.0};
  // DoSpline이 이번 사이클에 쓸 값 (섹터 안: 공격 모드 / 밖: 기본값)
  double active_bound_mindist_{0.2};
  double active_speed_scale_{1.0};

  // State
  ae_hyu_msgs::msg::ObstacleArray obstacles_;
  rclcpp::Time obstacles_stamp_{0, 0, RCL_ROS_TIME};
  ae_hyu_msgs::msg::ObstacleArray predicted_obstacles_;
  rclcpp::Time predicted_stamp_{0, 0, RCL_ROS_TIME};
  std::vector<ae_hyu_msgs::msg::Wpnt> gb_wpnts_;
  double gb_vmax_{0.0};
  double gb_max_s_{0.0};
  double wpnt_dist_{0.1};
  double cur_s_{0.0};
  double cur_d_{0.0};
  double cur_vs_{0.0};
  bool has_obstacles_{false};
  bool has_gb_wpnts_{false};
  bool has_frenet_odom_{false};
  std::string last_ot_side_{""};
  std::mutex mutex_obstacles_;
  std::mutex mutex_gb_wpnts_;
  std::mutex mutex_frenet_odom_;

  rclcpp::Subscription<ae_hyu_msgs::msg::ObstacleArray>::SharedPtr s_obstacles_;
  rclcpp::Subscription<ae_hyu_msgs::msg::ObstacleArray>::SharedPtr s_predicted_obstacles_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr s_frenet_odom_;
  rclcpp::Subscription<ae_hyu_msgs::msg::WpntArray>::SharedPtr s_global_waypoints_;
  rclcpp::Publisher<ae_hyu_msgs::msg::WpntArray>::SharedPtr p_evasion_wpnts_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr p_markers_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr p_sector_markers_;
  rclcpp::TimerBase::SharedPtr t_run_;
};

#endif  // PLANNING_PKG__SPLINER_NODE_HPP_
