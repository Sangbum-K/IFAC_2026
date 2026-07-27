// ForzaETH(unicorn-racing-stack) planner/spliner/spliner_node.py의 C++ 포팅.
// 장애물(ObstacleArray) 주변에 pre/post-apex 7점을 잡아 자연 3차 스플라인으로
// d(s) 회피 궤적을 만들고 /planner/avoidance/otwpnts로 발행한다.
// 회피 불가(트랙 경계 근접, 사이드 스위치 불가) 시 빈 배열을 발행하며,
// local_trajectory_node가 이를 로컬 윈도우에 병합(splice)한다.
#ifndef PLANNING_PKG__SPLINER_NODE_HPP_
#define PLANNING_PKG__SPLINER_NODE_HPP_

#include <cmath>
#include <deque>
#include <map>
#include <mutex>
#include <string>
#include <vector>

#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

#include "f1tenth_planning_msgs/msg/obstacle.hpp"
#include "f1tenth_planning_msgs/msg/obstacle_array.hpp"
#include "f1tenth_planning_msgs/msg/wpnt.hpp"
#include "f1tenth_planning_msgs/msg/wpnt_array.hpp"

class SplinerNode : public rclcpp::Node
{
public:
  explicit SplinerNode(
    const std::string & node_name,
    const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  void CallbackObstacles(const f1tenth_planning_msgs::msg::ObstacleArray::SharedPtr msg);
  void CallbackPredictedObstacles(const f1tenth_planning_msgs::msg::ObstacleArray::SharedPtr msg);
  void CallbackFrenetOdom(const nav_msgs::msg::Odometry::SharedPtr msg);
  void CallbackGlobalWaypoints(const f1tenth_planning_msgs::msg::WpntArray::SharedPtr msg);
  void Run();

  // ForzaETH spliner 핵심 로직
  std::vector<f1tenth_planning_msgs::msg::Obstacle> FilterObstacles(
    const std::vector<f1tenth_planning_msgs::msg::Obstacle> & obstacles) const;
  f1tenth_planning_msgs::msg::Obstacle PredictObstacleMovement(
    f1tenth_planning_msgs::msg::Obstacle obs) const;
  // 반환: {side("left"/"right"), d_apex}
  std::pair<std::string, double> MoreSpace(
    const f1tenth_planning_msgs::msg::Obstacle & obs, int gb_idx) const;
  // 해당 사이드의 apex d 계산: 목표는 장애물 가장자리 + evasion_dist지만,
  // corridor(장애물-벽)가 좁으면 여유를 장애물/벽 양쪽에 균등 배분한
  // 지점으로 축소 (벽에 최대한 붙이는 방식은 추적오차로 벽 접촉)
  double ApexFor(
    const std::string & side,
    const f1tenth_planning_msgs::msg::Obstacle & obs, int gb_idx) const;
  bool DoSpline(
    const f1tenth_planning_msgs::msg::Obstacle & closest_obs,
    f1tenth_planning_msgs::msg::WpntArray & out_wpnts,
    visualization_msgs::msg::MarkerArray & out_mrks);

  // ── 라인 스위치 (기본 maneuver, rule-based) ─────────────────────────────
  // 오프셋 레인(±apex)으로 smoothstep ease-on→hold→ease-back. 최적화 없음:
  // 사이드는 MoreSpace(빈 공간)+히스테리시스, 오프셋은 ApexFor로 정하고 고정
  // 블렌드로 진입/복귀한다. 우선/반대 사이드 오프셋이 모두 막히면 Run()이
  // DoSpline(free-form)을 "호출"한다 — 그게 폴백이다.
  bool TryLaneSwitch(
    const f1tenth_planning_msgs::msg::Obstacle & closest_obs,
    const std::vector<f1tenth_planning_msgs::msg::Obstacle> & all_ahead,
    f1tenth_planning_msgs::msg::WpntArray & out_wpnts,
    visualization_msgs::msg::MarkerArray & out_mrks);
  // 목표 오프셋 d_hold로 ease-on→hold→ease-back 궤적 생성 + 충돌/경계 검증.
  // closest_obs는 d_hold(=ApexFor)로 이미 클리어런스 확보 → 충돌검사에서 제외,
  // '다른' 장애물이 이 레인을 막는지만 본다. 막히면 false.
  bool BuildLaneSwitch(
    double d_hold,
    const f1tenth_planning_msgs::msg::Obstacle & closest_obs,
    const std::vector<f1tenth_planning_msgs::msg::Obstacle> & all_ahead,
    f1tenth_planning_msgs::msg::WpntArray & out_wpnts,
    visualization_msgs::msg::MarkerArray & out_mrks);

  // 경로를 lookahead 내 장애물들과 (s,d) 충돌 검사 (정적 커밋 유효성 검증,
  // SP 레인체인지 검증에 사용). exclude_id: 회피/추종 대상 장애물 자신은
  // 제외 (경로가 의도적으로 그 주변/뒤를 지나므로).
  // 반환 true=충돌 없음, false=충돌(blocking_s에 막은 장애물의 s_start).
  bool CheckPathCollision(
    const std::vector<f1tenth_planning_msgs::msg::Wpnt> & path,
    const std::vector<f1tenth_planning_msgs::msg::Obstacle> & obstacles,
    int32_t exclude_id, double * blocking_s) const;
  // lookahead 내 전체 장애물 (raceline 근접 여부 무관 — 경로 검증용)
  std::vector<f1tenth_planning_msgs::msg::Obstacle> ObstaclesInLookahead(
    const std::vector<f1tenth_planning_msgs::msg::Obstacle> & obstacles) const;
  // 사이를 통과할 수 없을 만큼 가까운 장애물들(s 간격 < cluster_gap_s,
  // d corridor < cluster_gap_d)을 하나의 (s,d) 박스로 병합.
  // 대각선 이중장애물을 개별로 보면 최근접 기준 고정 폭 회피만 시도하다
  // 트랙경계 abort로 포기한다 — 병합해야 "바깥으로 크게 도는" 해가 나온다.
  std::vector<f1tenth_planning_msgs::msg::Obstacle> ClusterObstacles(
    std::vector<f1tenth_planning_msgs::msg::Obstacle> obstacles) const;

  // 정적/동적 재분류 (unicorn perception/multi_tracking.py 이식):
  // 관측 이력의 위치 std 투표(min_std/max_std) + 정지 재분류(vs_reset)로
  // perception의 is_static 플래그를 덮어쓴다. 파킹한 상대를 동적으로 두면
  // 예측 유령/섹터 게이팅/커밋 비활성이 겹쳐 회피가 무너진다.
  void ClassifyObstacles(std::vector<f1tenth_planning_msgs::msg::Obstacle> & obstacles);

  // 회피 불가 시 폴백: raceline 유지 + 장애물 뒤 trailing_gap에서
  // 장애물 속도까지 감속하는 궤적 (ForzaETH 상태머신 TRAILING 대응)
  bool BuildTrailing(
    const f1tenth_planning_msgs::msg::Obstacle & closest_obs,
    f1tenth_planning_msgs::msg::WpntArray & out_wpnts,
    visualization_msgs::msg::MarkerArray & out_mrks);

  // 추월 섹터: 회피 기하(벽 여유 >= ot_min_clearance)가 성립하고 코너가
  // 아닌(|kappa| <= ot_max_kappa) s 구간을 raceline에서 자동 산출.
  // 섹터 안에서만 회피를 시도하고(밖은 TRAILING 대기) 섹터 안에서는 경계
  // 마진 완화 + 속도 부스트로 공격적으로 추월한다. 코너를 제외하는 이유:
  // 코너 아웃라인 추월은 경로가 길어져 페이스 우위가 상쇄된다 — 코너에선
  // 바짝 붙고 직선에서 속도 차로 제치는 것이 정석.
  void ComputeOtSectors();
  bool InOtSector(double s) const;
  void PublishSectorMarkers();

  // SP(최단거리) 추월 라인 레인체인지: 동적 상대 추월 시 IQP 오프셋
  // 스플라인 대신 SP 라인으로 갈아타 통과 구간을 지나고 복귀한다 —
  // 코너 진입 인사이드 찌르기 등 레이싱다운 추월 기하가 나온다.
  void CallbackSpWaypoints(const f1tenth_planning_msgs::msg::WpntArray::SharedPtr msg);
  void BuildSpMapping();   // SP 라인을 IQP frenet (d_sp(s), vx_sp(s))로 변환
  bool BuildSpLaneChange(
    const f1tenth_planning_msgs::msg::Obstacle & closest_obs,
    const std::vector<f1tenth_planning_msgs::msg::Obstacle> & all_ahead,
    f1tenth_planning_msgs::msg::WpntArray & out_wpnts,
    visualization_msgs::msg::MarkerArray & out_mrks);
  // 추월 마무리: 상대가 아직 옆이면(opp_beside) 간격이 벌어질 때까지,
  // 코너 안이면 코너 출구까지 SP 인코스를 유지한 뒤 raceline(아웃코스)으로
  // 복귀. v_floor(wpnt 단위)는 나란히 달리는 동안의 추월 페이스 하한.
  // 복귀 완료 시 false 반환
  bool BuildSpExit(
    bool opp_beside, double v_floor,
    f1tenth_planning_msgs::msg::WpntArray & out_wpnts,
    visualization_msgs::msg::MarkerArray & out_mrks);
  // s_from부터 raceline 복귀 블렌드 창(blend_len)이 코너(|kappa| >
  // ot_max_kappa)와 겹치지 않게 되는 최소 추가 SP 유지 거리 (최대 25m).
  // "인코스 진입 - 아웃코스 탈출"의 복귀 지연을 담당한다
  double SpHoldExtension(double s_from, double blend_len) const;

  // (s, d) -> (x, y). 균일 간격 global waypoints 기반 선형 보간.
  std::pair<double, double> FrenetToCartesian(double s, double d) const;

  // --- 고속 회피용 기하/속도 예산 ---------------------------------------
  // 진폭 amplitude[m]의 smoothstep 횡이동을 속도 v[m/s]에서 수행할 때,
  // 피크 횡가속이 blend_lat_acc_max_ 이내가 되는 최소 거리 [m].
  //   d(t) = A*s(w), w = t/L, s(w) = 3w^2-2w^3
  //   |d''|_max = 6*A*v^2/L^2  ->  L >= v*sqrt(6A/a_max)
  // 거리를 상수로 두면 횡가속이 v^2로 커져 그립을 넘는다 (6 m/s에서 1.5g 실측 계산).
  double BlendDist(double amplitude, double speed) const;

  // raceline에서 d만큼 오프셋된 경로의 실제 곡률.
  // 곡률 중심 쪽으로 붙으면 반경이 줄어 더 빨리 감속해야 한다.
  //   kappa' = kappa / (1 - d*kappa)      (d>0 = 진행방향 왼쪽)
  static double OffsetKappa(double kappa, double d);

  // 회피 궤적의 속도 프로파일을 물리적으로 성립하게 만든다:
  //  1) 각 점의 오프셋 곡률로 횡그립 상한 v <= sqrt(path_lat_acc_max/|kappa'|)
  //  2) 뒤에서 앞으로 역방향 패스로 종방향 감속 한계 적용
  //     v[i] <= sqrt(v[i+1]^2 + 2*path_decel_max*ds)
  // 직선에서는 아무것도 깎지 않고(가속 유지), 코너 진입 전에만 크게 감속한다.
  void ApplySpeedProfile(std::vector<f1tenth_planning_msgs::msg::Wpnt> & wpnts) const;

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
  bool use_trailing_{true};     // false면 회피 불가 시 감속/정지(trailing) 안 함
  double trailing_gap_m_{2.5};
  double trailing_decel_{2.5};
  double cmd_speed_scale_{1.0};
  // 고속 회피 기하/속도 예산 (BlendDist / ApplySpeedProfile)
  double blend_lat_acc_max_{3.0};   // 횡 전환에 배정할 그립 [m/s^2]
  double blend_dist_max_m_{8.0};    // 전환 거리 상한 [m]
  double path_lat_acc_max_{7.0};    // 오프셋 경로 곡률 -> 속도 상한용 그립 [m/s^2]
  double path_decel_max_{6.0};      // 회피 궤적 역방향 감속 한계 [m/s^2]
  bool trailing_active_{false};
  // 라인 스위치 (기본 maneuver)
  bool use_lane_switch_{true};       // false면 기존처럼 DoSpline을 1순위로
  double lane_blend_out_m_{5.0};     // raceline 복귀 블렌드 길이 [m]
  bool spline_fallback_active_{false};  // "모든 레인 막힘 -> 스플라인" 상승엣지 이벤트용
  // 이중장애물 재계획 파라미터 (mina Task 2)
  double path_check_margin_{0.15};   // 경로-장애물 충돌 판정 횡방향 여유 [m]
  double cluster_gap_s_m_{8.0};      // 이 s 간격 이내 장애물은 병합 후보 [m]
  double cluster_gap_d_m_{0.8};      // 사이 corridor가 이보다 좁으면 병합 [m]
  // 벽이 좁아 evasion_dist를 다 못 챙길 때 apex를 벽 마진 안으로 줄여서라도
  // 회피를 시도하는 최소 장애물 간격 (차 반폭 ~0.15 + 여유)
  double evasion_dist_min_{0.35};
  // apex가 벽 마진으로 축소(squeeze)된 회피는 이 배율로 감속 — 좁은
  // corridor를 섹터 부스트 속도로 지나면 추적오차(속도비례)로 벽에 닿는다
  double squeeze_speed_scale_{0.35};
  // 공격성 파라미터: 접근 구간은 감속하지 않고 장애물 직전에서만 제동한다
  double squeeze_brake_dist_m_{6.0};      // squeeze 감속을 시작하는 s_front 앞 거리 [m]
  double trailing_engage_dist_m_{12.0};   // trailing 감속을 시작하는 정지점 앞 거리 [m]
  double dyn_pass_speed_ratio_{1.4};     // 동적 상대 추월 중 최소 속도 = 상대 x 이 배율

  // 정적/동적 분류 파라미터 (unicorn opponent_tracker_params.yaml 기본값)
  int cls_min_nb_meas_{5};       // 분류 시작 최소 관측 수
  double cls_min_std_{0.16};     // std_s, std_d 둘 다 미만이면 정적 표 [m]
  double cls_max_std_{0.2};      // 하나라도 초과하면 정적 표 리셋 [m]
  double cls_vs_reset_{0.1};     // 최근 |vs| 평균이 미만이면 정지 재분류 [m/s]
  static constexpr size_t kClsWindow{20};     // 위치 std 계산 관측 창
  static constexpr size_t kClsVsWindow{10};   // vs 평균 관측 창 (unicorn vs_list 10)
  struct ObsClsTrack {
    std::deque<double> s_hist;   // s_ref 기준 unwrap된 s 관측
    std::deque<double> d_hist;
    std::deque<double> vs_hist;
    double s_ref{0.0};
    int static_count{0};
    int total_count{0};
    bool has_flag{false};        // 분류 확정 여부 (미확정이면 perception 플래그 유지)
    bool static_flag{false};
    rclcpp::Time last_seen{0, 0, RCL_ROS_TIME};
  };
  std::map<int32_t, ObsClsTrack> cls_tracks_;

  // 추월 섹터 파라미터
  bool sector_gating_{true};
  double ot_min_clearance_{0.95};
  double ot_max_kappa_{0.1};         // 섹터 판정 최대 곡률 [rad/m] (0.1 = 반경 10m)
  double ot_sector_min_len_m_{8.0};
  double ot_bound_mindist_{0.25};
  double ot_speed_scale_{1.1};
  std::vector<double> ot_sectors_param_;                 // 수동 지정 (flat s0,s1,...)
  std::vector<std::pair<double, double>> ot_sectors_;    // [s_start, s_end] (wrap 허용)
  bool in_ot_sector_{false};
  // 진행 중인 정적 회피 커밋 (경로 끝까지 유지 — 도중 소멸 시 벽 충돌)
  f1tenth_planning_msgs::msg::WpntArray committed_wpnts_;
  visualization_msgs::msg::MarkerArray committed_mrks_;
  int32_t committed_id_{-1};
  double committed_start_s_{-1.0};
  double committed_until_s_{-1.0};
  // DoSpline이 이번 사이클에 쓸 값 (섹터 안: 공격 모드 / 밖: 기본값)
  double active_bound_mindist_{0.2};
  double active_speed_scale_{1.0};

  // SP 레인체인지 상태
  bool sp_lane_change_{true};
  // 추월 완료 판정: 상대 앞머리가 ego 뒤로 이 간격만큼 벌어져야 복귀 [m]
  double lc_pass_clear_m_{4.0};
  std::vector<f1tenth_planning_msgs::msg::Wpnt> sp_raw_;   // SP 라인 원본 (자체 s 좌표)
  std::vector<double> sp_d_;    // gb 인덱스별 SP 라인의 IQP frenet 횡 오프셋
  std::vector<double> sp_vx_;   // gb 인덱스별 SP 라인 속도
  bool has_sp_{false};          // 매핑 완료 여부
  bool lc_active_{false};

  // State
  f1tenth_planning_msgs::msg::ObstacleArray obstacles_;
  rclcpp::Time obstacles_stamp_{0, 0, RCL_ROS_TIME};
  f1tenth_planning_msgs::msg::ObstacleArray predicted_obstacles_;
  rclcpp::Time predicted_stamp_{0, 0, RCL_ROS_TIME};
  std::vector<f1tenth_planning_msgs::msg::Wpnt> gb_wpnts_;
  double gb_vmax_{0.0};
  // 트랙 한 바퀴 길이 = 마지막 wpnt s + 점 간격. 마지막 s가 아니라 반드시
  // '+ 간격'이어야 한다 — local_trajectory_node / perception 브릿지가 쓰는
  // 모듈러스와 같은 값이라야 wrap 근처에서 s가 어긋나지 않는다.
  double track_len_{0.0};
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

  rclcpp::Subscription<f1tenth_planning_msgs::msg::ObstacleArray>::SharedPtr s_obstacles_;
  rclcpp::Subscription<f1tenth_planning_msgs::msg::ObstacleArray>::SharedPtr s_predicted_obstacles_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr s_frenet_odom_;
  rclcpp::Subscription<f1tenth_planning_msgs::msg::WpntArray>::SharedPtr s_global_waypoints_;
  rclcpp::Subscription<f1tenth_planning_msgs::msg::WpntArray>::SharedPtr s_sp_waypoints_;
  rclcpp::Publisher<f1tenth_planning_msgs::msg::WpntArray>::SharedPtr p_evasion_wpnts_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr p_markers_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr p_sector_markers_;
  rclcpp::TimerBase::SharedPtr t_run_;
};

#endif  // PLANNING_PKG__SPLINER_NODE_HPP_
