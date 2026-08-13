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
#include "std_msgs/msg/bool.hpp"
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
    const f1tenth_planning_msgs::msg::Obstacle & obs) const;
  // 해당 사이드의 apex d 계산: 목표는 장애물 가장자리 + evasion_dist지만,
  // corridor(장애물-벽)가 좁으면 여유를 장애물/벽 양쪽에 균등 배분한
  // 지점으로 축소 (벽에 최대한 붙이는 방식은 추적오차로 벽 접촉)
  double ApexFor(
    const std::string & side,
    const f1tenth_planning_msgs::msg::Obstacle & obs) const;

  // ── 연쇄 회피 그룹 ──────────────────────────────────────────────────────
  // closest 하나만 보고 사이드/오프셋을 정하면, 그 회피가 끝나기 전에 다음
  // 장애물이 closest 가 되면서 **같은 상황에 대해 답이 뒤집힌다**.
  //   run_0810_233521 재생 실측: 인카운터 71건 중 50건(70%)이 "차가 이미
  //   라인 밖(|d|>0.2)"인 상태에서 시작했고, side_switch_blocked 가 난
  //   28건 중 27건(96%)이 그 경우였다. 직전 인카운터와의 간격 p50 0.04초 —
  //   별개 사건이 아니라 한 번의 회피 도중 closest 가 갈린 것이다.
  // 그래서 "closest 의 복귀 블렌드 안에 들어오는 장애물"을 연쇄적으로 모아
  // 한 덩어리로 보고, 사이드와 오프셋을 그 덩어리 전체로 정한다.
  // 병합(ClusterObstacles)과 다르다: 여기서는 s 구간을 합치지 않으므로
  // MinWallInSpan 이 사이 구간의 최소 벽폭까지 훑는 부작용이 없다.
  std::vector<f1tenth_planning_msgs::msg::Obstacle> EvasionGroup(
    const f1tenth_planning_msgs::msg::Obstacle & closest,
    const std::vector<f1tenth_planning_msgs::msg::Obstacle> & all_ahead) const;
  // 그룹 전체를 한 오프셋으로 비켜 가는 d. clears_all 이 false면 그 사이드로는
  // 전부를 한 번에 통과할 수 없다 (지그재그가 필요 — 이 티어의 범위 밖).
  double GroupApex(
    const std::string & side,
    const std::vector<f1tenth_planning_msgs::msg::Obstacle> & group,
    bool * clears_all) const;

  bool DoSpline(
    const f1tenth_planning_msgs::msg::Obstacle & closest_obs,
    const std::vector<f1tenth_planning_msgs::msg::Obstacle> & all_ahead,
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
  // closest_obs를 포함한 모든 장애물과 차량 footprint/도착시간을 검증한다.
  // 목표 오프셋 계산이 성공했더라도 실제 sampled path가 안전하지 않으면 false.
  // hold_through_s: 오프셋을 유지해야 하는 마지막 s (그룹의 끝 장애물).
  //   기본은 closest_obs.s_end 지만, 복귀 블렌드 구간에 다른 장애물이 있으면
  //   그 뒤까지 hold 를 늘려 **복귀하다 들이받는 것**을 막는다. 늘리지 않으면
  //   경로가 CheckPathCollision 에서 거부되고, 그 다음 사이클에 뒤 장애물이
  //   closest 가 되면서 사이드가 뒤집힌다 (EvasionGroup 주석의 27건).
  bool BuildLaneSwitch(
    double d_hold,
    const f1tenth_planning_msgs::msg::Obstacle & closest_obs,
    double hold_through_s,
    const std::vector<f1tenth_planning_msgs::msg::Obstacle> & all_ahead,
    f1tenth_planning_msgs::msg::WpntArray & out_wpnts,
    visualization_msgs::msg::MarkerArray & out_mrks);

  // 커밋된 정적 회피 경로를 **무효화할 자격이 있는** 장애물만 남긴다.
  // 관측 이력(cls_tracks_)이 commit_break_min_obs_ 이상인 트랙만 통과시킨다.
  // 갓 뜬 1~2프레임짜리 유령이 이미 성립한 회피를 파기하고 정지 궤적으로
  // 떨어뜨리는 것을 막는다 (0807 run_164819 인과 3중 확인).
  // 재계획 경로의 검증에는 쓰지 않는다 — 거기서는 fail-closed가 맞다.
  std::vector<f1tenth_planning_msgs::msg::Obstacle> StableObstacles(
    const std::vector<f1tenth_planning_msgs::msg::Obstacle> & obstacles) const;

  // 경로를 lookahead 내 장애물들과 (s,d,t) 충돌 검사 (정적 커밋 유효성,
  // 모든 회피 후보 검증에 사용). 차량 반길이/반폭, 장애물 분산, 경로
  // 도착시각에서의 등속 이동을 포함하며 목표 장애물도 절대 제외하지 않는다.
  // 반환 true=충돌 없음, false=충돌(blocking_s에 막은 장애물의 s_start).
  bool CheckPathCollision(
    const std::vector<f1tenth_planning_msgs::msg::Wpnt> & path,
    const std::vector<f1tenth_planning_msgs::msg::Obstacle> & obstacles,
    double * blocking_s) const;
  // lookahead 내 전체 장애물 (raceline 근접 여부 무관 — 경로 검증용)
  std::vector<f1tenth_planning_msgs::msg::Obstacle> ObstaclesInLookahead(
    const std::vector<f1tenth_planning_msgs::msg::Obstacle> & obstacles) const;
  // 사이를 통과할 수 없을 만큼 가까운 장애물들(s 간격 < cluster_gap_s,
  // d corridor < cluster_gap_d)을 하나의 (s,d) 박스로 병합.
  // 대각선 이중장애물을 개별로 보면 최근접 기준 고정 폭 회피만 시도하다
  // 트랙경계 abort로 포기한다 — 병합해야 "바깥으로 크게 도는" 해가 나온다.
  // (non-const: 병합 보류 사유를 THROTTLE 로그로 남긴다. Node::get_clock()의
  //  const 오버로드는 ConstSharedPtr을 돌려줘 RCLCPP_*_THROTTLE에 못 넣는다.)
  std::vector<f1tenth_planning_msgs::msg::Obstacle> ClusterObstacles(
    std::vector<f1tenth_planning_msgs::msg::Obstacle> obstacles);

  // 정적/동적 재분류 (unicorn perception/multi_tracking.py 이식):
  // 관측 이력의 위치 std 투표(min_std/max_std) + 정지 재분류(vs_reset)로
  // perception의 is_static 플래그를 덮어쓴다. 파킹한 상대를 동적으로 두면
  // 예측 유령/섹터 게이팅/커밋 비활성이 겹쳐 회피가 무너진다.
  void ClassifyObstacles(std::vector<f1tenth_planning_msgs::msg::Obstacle> & obstacles);

  // 회피 불가 시 폴백: raceline 유지 + 장애물 뒤 trailing_gap에서
  // 장애물 속도까지 감속하는 궤적 (ForzaETH 상태머신 TRAILING 대응)
  //
  // allow_stop=false면 목표 속도를 trailing_retry_speed_ 아래로 내리지
  // 않는다. Tier1/2가 "조향 기하 하한 초과"가 아닌 일시적 사유로 실패한
  // 경우에 쓴다 — 정지가 정당하지 않고, 감속만 해도 다음 사이클의
  // BlendDist가 줄어 회피가 성립한다.
  // Tier 2c: 원호 이탈 (escape arc).
  // Tier1(라인스위치)/Tier2(스플라인)가 **조향 기하 하한**으로 거부했을 때만
  // 시도하는 마지막 회피 수단. 둘 다 "평행 복귀"를 전제한 smoothstep 이라
  // 6A/L² 을 요구하는데, 여기서는 평행 복귀를 포기하고 한쪽으로만 꺾어
  // 2A/L² 로 비킨다 (필요 거리 58%). 사람이 급하면 핸들만 꺾는 것과 같다.
  //
  // 전제 3가지 — 하나라도 깨지면 정지가 더 안전하다:
  //  (1) 정적 + 지금 실제로 보이는(is_visible) 장애물일 것. 외삽 유령에
  //      대고 급조향하면 멀쩡한 라인에서 벽으로 튄다.
  //  (2) 비키는 쪽에 실제 통과폭이 있을 것 (벽 여유 검사).
  //  (3) 남은 헤딩을 풀 복귀 구간이 계획 안에 들어올 것.
  // 곡률은 "최대 조향"이 아니라 조향 한계와 **그립 한계 중 작은 쪽**을 쓴다 —
  // 3 m/s 이상에서는 그립이 구속이라 최대 조향을 넣으면 그냥 밀린다.
  bool BuildEscapeArc(
    const f1tenth_planning_msgs::msg::Obstacle & closest_obs,
    const std::vector<f1tenth_planning_msgs::msg::Obstacle> & all_ahead,
    f1tenth_planning_msgs::msg::WpntArray & out_wpnts,
    visualization_msgs::msg::MarkerArray & out_mrks);

  bool BuildTrailing(
    const f1tenth_planning_msgs::msg::Obstacle & closest_obs,
    f1tenth_planning_msgs::msg::WpntArray & out_wpnts,
    visualization_msgs::msg::MarkerArray & out_mrks,
    bool allow_stop = true);

  // 추월 섹터: 회피 기하(벽 여유 >= ot_min_clearance)가 성립하고 코너가
  // 아닌(|kappa| <= ot_max_kappa) s 구간을 raceline에서 자동 산출.
  // 섹터 안에서만 **동적 상대** 회피를 시도하고(밖은 TRAILING 대기),
  // 섹터 안에서는 ot_speed_scale 배율로 속도를 올린다.
  // (예전 주석의 "경계 마진 완화"는 실재하지 않는다 — ot_bound_mindist 가
  //  spline_bound_mindist 와 같은 값이었고 모든 소비처가
  //  RequiredBoundaryMargin() 하한에 먹혀 완화가 성립한 적이 없어 제거됐다.
  //  정적 장애물은 섹터와 무관하게 항상 회피를 시도한다.)
  // 코너를 제외하는 이유:
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
    const std::vector<f1tenth_planning_msgs::msg::Obstacle> & all_ahead,
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
  // lat_acc <= 0 이면 blend_lat_acc_max_(또는 마찰원 잔여)를 쓴다.
  // kappa_track: 그 구간 레이스라인 곡률 [1/m] (BlendGeomFloor로 전달).
  // s_from: 블렌드 시작 s [m]. >= 0 이면 BlendGeomFloor 가 스칼라 근사 대신
  //   지점별 검사를 쓴다. 음수면 "s 문맥 없음"(기존 보수식).
  double BlendDist(double amplitude, double speed, double lat_acc = -1.0,
                   double kappa_track = 0.0, double s_from = -1.0) const;

  // 속도와 무관한 조향 기하 하한 [m]. smoothstep 횡이동 amplitude의 피크
  // 곡률 6A/L²이 조향 한계를 넘지 않는 최소 L. 가용 거리가 이보다 짧으면
  // "어떤 속도로도 회피 불가"이고, 그때만 정지가 정당하다.
  //
  // kappa_track: 그 구간에서 레이스라인 자체가 이미 쓰고 있는 곡률 [1/m].
  // 조향은 트랙 곡률과 기동 곡률을 **합쳐서** 내야 하므로 실제 조건은
  //   6A/L² + kappa_track <= 0.8*kappa_steer_max
  // 이다. 이 항을 빼면 헤어핀에서 하한을 과소평가한다 — 0807 run_164216
  // (kappa 0.74, A 0.49)의 참 하한은 1.80m가 아니라 2.23~2.45m였고, 그
  // 차이만큼 "성립한다고 판정했지만 실제로는 못 도는" 경로가 나왔다.
  //
  // one_sided: 한쪽으로만 꺾어 "비키기만" 하는 원호 기동. 평행 복귀를 하는
  // smoothstep 은 피크 곡률이 6A/L² 인데, 한쪽 원호는 2A/L² 이라 같은 곡률로
  // **3배**의 횡이동을 낸다. 즉 필요 거리가 1/sqrt(3) = 58% 로 준다.
  // rosbag_0811 기하불가 기각 표본에 대입하면 LaneSwitch 기각의 48% /
  // FreeSpline 기각의 67% 가 이 모델에서는 성립한다.
  // 대신 기동 끝에서 헤딩이 kappa*L 만큼 남으므로(3.8 m/s, A=0.46 에서 41도)
  // 반드시 복귀 블렌드와 세트로만 써야 한다 — BuildEscapeArc 참조.
  // s_from >= 0 이면 **지점별** 검사로 하한을 구한다 (아래 BlendGeomFeasible).
  // 스칼라 식은 `max|kappa_track|`(구간 최대)과 `6A/L²`(기동 피크)을 더하는데,
  // 이 둘은 **같은 지점에서 일어나지 않는다** — smoothstep 의 |d''| 는 블렌드
  // 양 끝에서 피크고 중앙에서 0 이다. 게다가 부호를 버려서, 코너 **바깥**으로
  // 비키는 기동(= 코너 조향을 푸는 방향)까지 더하는 것으로 친다.
  //   my_track0811 s=19.88, A=-0.44(우측), L=2.40 실측 재구성:
  //     스칼라 가정 피크 1.191 (= 0.733 + 0.458) -> 한계 1.042 초과 -> 거부
  //     실제 경로 피크 0.836 @ s=22.08 (한계의 80%) -> 통과
  //   kappa 피크 지점(s=20.11)의 참 조향 요구는 0.351 인데 1.191 로 봤다(3.4배).
  // 트랙 곡률과 기동 곡률이 실제로 **더해지는** 경우(코너 안쪽 회피, 그리고
  // 어떤 블렌드든 복귀하는 후반부)에는 지점별 결과가 스칼라 식과 같아진다 —
  // 즉 이 경로는 더 느슨해지지 않는다. 0807 run_164216 회귀는 유지된다.
  double BlendGeomFloor(double amplitude, double kappa_track = 0.0,
                        bool one_sided = false, double s_from = -1.0) const;

  // 길이 L 의 블렌드가 조향 한계 안에 들어오는가 — 지점별·부호 유지 검사.
  //   |kappa_track(s) + d''(s)/geom_floor_margin_| <= kappa_steer_max
  // 근사 여유(geom_floor_margin_)는 "실제 샘플 피크가 smoothstep 근사보다
  // 크다"를 흡수하는 것이므로 **기동 항에만** 건다 (스칼라 식과 같은 이유).
  // 상쇄 크레딧은 (1-geom_floor_kappa_ratio_)*kappa_steer_max 로 제한한다 —
  // 레이스라인 kappa 는 d=0 기준이고 자차는 추종 오차만큼 벗어나 있으므로
  // "코너가 대신 돌아준다"를 무한정 믿지 않는다.
  bool BlendGeomFeasible(double amplitude, double blend_len, double s_from,
                         bool one_sided) const;

  // 계획이 달성 가능하다고 보는 최대 곡률 [1/m].
  //   steer_kappa_scale_ * tan(|delta_max|) / wheelbase
  // 회피 가부를 결정하는 모든 조향 게이트(BlendGeomFloor / ApplySpeedProfile /
  // SP 라인 로드 검증)가 이 하나를 쓴다. 게이트마다 식을 따로 쓰면 한 곳만
  // 완화됐을 때 "여기선 되는데 저기선 안 되는" 모순이 생긴다.
  double SteerKappaMax() const;

  // 가용 거리 available 안에서 amplitude만큼 횡이동할 수 있는 최대 속도 [m/s].
  // 기하 하한을 못 채우면 0(= 어떤 속도로도 불가), 횡이동이 0에 가까우면 무한대.
  // 회피 가부를 "지금 속도에서 되나?"(이진 거부)가 아니라 "되는 속도는
  // 얼마인가?"로 바꾸는 함수다 — 정적 장애물은 속도가 자유 변수이므로
  // 감속으로 성립시킬 수 있는데도 매번 Tier3(정지)로 떨어지고 있었다.
  // s_from >= 0 이면 기하 가부를 "최소 하한 계산 후 비교"가 아니라 **실제로
  // 쓸 길이(= available)에서 지점별로** 판정한다. 블렌드는 available 을 그대로
  // 쓰므로 이쪽이 근사 없이 정확하다.
  double BlendSpeedCap(double amplitude, double available,
                       double lat_acc = -1.0, double kappa_track = 0.0,
                       double s_from = -1.0) const;

  // 경로 웨이포인트의 **기하부**를 채운다 (vx_mps 는 호출부 몫).
  // 2026-08-13 이전에는 이 11개 대입이 5곳에 복제돼 있었다
  // (BuildLaneSwitch / DoSpline / BuildEscapeArc / BuildSpLaneChange /
  //  BuildSpExit). 다섯 벌 모두 완전히 동일했지만, 복제라는 사실 때문에
  //  d_left/d_right 부호 규약을 한 곳만 고치면 조용히 어긋날 수 있었다.
  // vx_mps 는 사이트마다 규칙이 실제로 다르므로(추월 페이스 하한, 그립 캡,
  // sp 라인 속도 등) 일부러 여기서 정하지 않는다 — 그 차이는 복제 표류가
  // 아니라 의도된 것이고, 호출부에 남아 있어야 읽힌다.
  f1tenth_planning_msgs::msg::Wpnt MakeBaseWpnt(
      int idx, double s, double d, double x, double y, int gb_idx) const;

  // [s_from, s_to] 구간 레이스라인 |kappa|의 최대값 [1/m]. BlendGeomFloor에
  // 넘길 트랙 곡률항. 최대값을 쓰는 이유는 기하 하한이 구간 내 가장 조향이
  // 빡빡한 지점에서 성립해야 하기 때문이다.
  // d_offset != 0 이면 그 오프셋 경로의 유효 곡률(OffsetKappa) 기준 최대값.
  // 0이면 레이스라인 곡률 그대로.
  double BlendKappaMax(double s_from, double s_to,
                       double d_offset = 0.0) const;

  // "회피 중이라 사이드를 유지해야 하는가". |cur_d_|가 문턱을 넘는 것만으로는
  // 부족하다 — 그 d의 **부호가 last_ot_side_와 같아야** 실제로 그 사이드로
  // 회피 중인 것이다. 0807 run_164216에서 차가 오른쪽(d=-0.27)으로 밀려
  // 있는데 last_ot_side_="left"라 왼쪽을 고집해 회피가 전패했다.
  // (문턱 0.20m는 정상 추종오차보다 작아 |d| 조건만으로는 38~96%의 시간
  //  동안 참이 된다 — 사실상 상시 라치였다.)
  //
  // 2026-08-08 추가: |cur_d_| 조건만으로는 **접근 구간**(아직 라인 위라
  // |d|가 작음)에서 히스테리시스가 전혀 없었다. 0808 run_185049에서 같은
  // 장애물의 관측 박스 폭이 0.28~1.25m로 흔들리며 left/right 판정이
  // 프레임마다 뒤집혔고, 그 뒤집힘이 통과 직후 "사이드 전환"으로 이어져
  // 로컬 경로를 반대편으로 점프시켰다(control 추종 백스톱 경로오차 0.5m).
  // 그래서 "같은 장애물을 계속 회피하는 중"이면 |d|와 무관하게 유지한다.
  // 장애물 id는 detector에서 초당 1.7개꼴로 갈리므로 id가 아니라 s 근접으로
  // 판정한다. 유지는 **선호**일 뿐이고, 유지 사이드가 막히면 두 호출부가
  // 모두 반대 사이드로 폴백하므로 데드락이 되지 않는다.
  bool SideKeepActive(const f1tenth_planning_msgs::msg::Obstacle & obs) const;

  // "지금 회피 중이라 커밋된 사이드가 유효한가" = last_ot_side_ 의 수명.
  //
  // 2026-08-12 추가 (mina 8fd816c). CommitOtSide 는 세팅만 하고 **어디에서도
  // 지우지 않았다** — 첫 회피 이후 last_ot_side_ 가 주행이 끝날 때까지 남았고,
  // 위 주석이 이미 지적한 "|d| 조건은 정상 추종오차 안이라 38~96% 시간 동안
  // 참"과 맞물려 사이드 유지가 사실상 영구 라치가 됐다.
  //   rosbag0812 run_0812_013630: 장애물 없는 정상 주행에서도 직선 |d| p95
  //   0.26 > side_switch_d_tresh 0.20 이라 반대편이 1.1m 비어 있어도 후보를
  //   만들지 않고 trailing 으로 떨어졌다.
  //
  // CommitOtSide 는 회피 경로를 실제로 만든 사이클마다 호출되므로, 스탬프가
  // 오래됐다는 것은 곧 "지금 회피 중이 아니다" 이다.
  //
  // ⚠️ 이 수명은 **진입 시점의 첫 사이드 결정**만 고친다. 회피가 시작되면
  // 커밋이 매 사이클 갱신되므로 항상 신선하고, 그 구간의 오판은 아래
  // side_clears 양보(TryLaneSwitch)와 DoSpline 의 가로지르기 허용이 담당한다.
  // run_0812_013630 실측: 장애물 접근 53회 중 |d|가 0.20을 처음 넘는 순간에
  // 커밋이 낡아 있던 건 3회(115.7/143.5/152.1s)뿐이고, 관통 사고 4건은 전부
  // age=0.00 이었다. 두 수정은 겹치지 않는다.
  bool OtSideCommitFresh() const;

  // last_ot_side_ 와 사이드 잠금(대상 s + 시각)을 **함께** 갱신한다.
  // 따로 갱신하면 잠금이 옛 장애물을 가리킨 채 남아, 다음 장애물의 사이드를
  // 엉뚱하게 고정한다.
  void CommitOtSide(
    const std::string & side,
    const f1tenth_planning_msgs::msg::Obstacle & obs);

  // 두 장애물을 하나의 박스로 합치면 그 s 구간에서 통과 가능한 d가 하나도
  // 남지 않는가. 남지 않으면 그 병합은 거의 항상 벽 리턴/유령과 실물이
  // 합쳐진 산물이다 — 사전 필터는 낙관적이어야 하고 실제 가부는
  // CheckPathCollision이 판정하므로 그런 병합은 하지 않는다.
  bool MergeBlocksTrack(
    const f1tenth_planning_msgs::msg::Obstacle & a,
    const f1tenth_planning_msgs::msg::Obstacle & b,
    double s_start, double s_end) const;

  // 회피 거부 사유 누적 카운터. 거부 로그는 전부 THROTTLE(2000ms)인데 루프는
  // 30Hz라 최대 1/60만 남는다 — 0808 run_185049 분석에서 "left가 왜 실패했나"가
  // 대부분 지워져 있었다. 카운터는 전수를 세고 /pitwall/reject_<사유>로 낸다.
  // const 경로(ClusterObstacles 등)에서도 세야 하므로 const + mutable 카운터.
  void NoteReject(const char * reason) const;
  // 회피 목표(d_hold/d_apex)를 "재시도 중 유지할 값"으로 기록한다.
  void NoteRetryTarget(double d_target);
  void PublishRejectCounts();

  // ── 사이클 트레이스 (trace_cycles_) ──────────────────────────────────
  // 거부 로그가 전부 THROTTLE(2000ms)이고 카운터는 누적값만 주므로,
  // "장애물까지 몇 m에서 / 무슨 분류로 / 어느 Tier가 / 왜 실패했나"의
  // **시계열**이 어디에도 남지 않는다. 회피가 늦는 원인을 가르려면
  //   (a) 인지가 늦게 잡는가  (b) 정적 분류가 늦는가
  //   (c) 성립했다가 커밋이 깨지는가  (d) 진짜 기하가 안 되는가
  // 를 구별해야 하는데, 넷 다 이 한 줄이 있으면 즉시 갈린다.
  // 장애물이 있는 사이클에만 찍히므로 평시에는 조용하다.
  void TraceCycle(
    const f1tenth_planning_msgs::msg::Obstacle * closest,
    bool committed, bool try_evade, bool in_sector,
    const char * outcome) const;

  // [s_from, s_to] 구간에서 횡 전환에 쓸 수 있는 그립 [m/s²].
  // 마찰원 잔여 sqrt(path_lat_acc_max² - (kappa_ref·v_ref)²)의 구간 최소값.
  // 직선에서는 레이스라인이 횡그립을 거의 안 쓰므로 예산 전체를 쓸 수 있고,
  // 코너에서는 남는 만큼만 쓴다. blend_use_spare_grip_=false면 고정값 사용.
  double BlendLatAcc(double s_from, double s_to) const;

  // 회피 기하를 짤 때 쓰는 계획 속도 [m/s]. 실측 속도가 아니라
  // max(|cur_vs_|, plan_speed_floor_)다 — v=0이면 BlendDist가 0을 돌려주고
  // 절점 스케일이 1.0으로 붕괴해, 정지한 차는 회피 기하 자체를 만들 수 없다
  // (기하곡률이 조향 한계를 넘어 ApplySpeedProfile이 조용히 거부). 차는 회피
  // 지점에 정지 상태로 도달하지 않으므로 "그때 낼 속도"로 기하를 짜야 한다.
  // 제동/속도 프로파일 등 실측 속도가 필요한 곳에는 쓰지 말 것.
  double PlanSpeed() const;

  // [s_from, s_to] 구간(랩 wrap 허용)에서 가장 좁은 좌/우 벽 여유.
  // apex를 한 점의 벽으로 정하면 그 구간 다른 지점에서 경계에 걸려 클램프되고,
  // 클램프가 장애물 간격을 침범하면 회피가 통째로 포기된다 (run_0806_155621).
  void MinWallInSpan(
    double s_from, double s_to, double & min_d_left, double & min_d_right) const;

  // CheckPathCollision이 이 장애물에 대해 실제로 요구하는 횡 간격 [m].
  // 회피 목표(evasion_dist)가 이 값보다 작으면 계획한 회피를 자기 검증기가
  // 매번 충돌로 거부한다 — 정적 회피가 통째로 죽어 있던 원인이다
  // (run_0806_155621: 목표 0.35 vs 요구 0.50).
  double RequiredObstacleClearance(
    const f1tenth_planning_msgs::msg::Obstacle & obs) const;

  // 관측 분산 -> footprint 팽창량 [m] (3σ), max_uncertainty_m_로 클램프.
  // pos_var는 위치 분산, vel_var는 속도 분산, t는 그 속도가 작용하는 시간.
  double UncertaintyMargin(double pos_var, double vel_var, double t) const;

  // raceline에서 d만큼 오프셋된 경로의 실제 곡률.
  // 곡률 중심 쪽으로 붙으면 반경이 줄어 더 빨리 감속해야 한다.
  //   kappa' = kappa / (1 - d*kappa)      (d>0 = 진행방향 왼쪽)
  static double OffsetKappa(double kappa, double d);

  // 회피 궤적의 속도 프로파일을 물리적으로 성립하게 만든다:
  //  1) 각 점의 오프셋 곡률로 횡그립 상한 v <= sqrt(path_lat_acc_max/|kappa'|)
  //  2) 차량 조향각/조향률 한계 적용
  //  3) friction-circle을 반영한 역방향 감속과 순방향 가속 한계 적용
  //  4) 현재 속도/계획 지연/jerk로 곡률 상한 도달 가능성 검증
  //  5) 변경된 속도로 ax를 다시 계산
  // require_current_feasible=false는 충돌 회피보다 최대 제동 명령 발행이
  // 우선인 trailing에만 사용한다. 반환 false면 현재 상태에서 물리적으로
  // 도달할 수 없는 회피 경로다.
  // steer_reject_ratio_override > 0 이면 곡률 기각 배율을 그 값으로 바꾼다
  // (escape 원호 전용 — escape_steer_kappa_reject_ratio_ 주석 참조).
  // 밴드를 넘긴 샘플만 살아나는 것이 아니라, 밴드 **안**의 샘플은 그대로
  // sqrt(a_lat/k) 횡가속 캡을 받아 속도가 강하게 눌린다는 점에 주의.
  bool ApplySpeedProfile(
    std::vector<f1tenth_planning_msgs::msg::Wpnt> & wpnts,
    bool require_current_feasible = true,
    double steer_reject_ratio_override = -1.0) const;

  // local_trajectory의 modulo splice는 한 바퀴 이상 경로를 표현할 수 없다.
  bool PlanSpanAllowed(double span) const;
  double RequiredBoundaryMargin(double requested_margin) const;
  bool WithinTrackBoundary(int gb_idx, double d, double requested_margin) const;

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
  // outside 판정(코너 바깥쪽인가)에 필요한 최소 |곡률 합| [1/m].
  // apex 뒤 2m(20점) 곡률 합의 **부호만** 보면 직선(합≈0)에서 노이즈로
  // 좌/우가 프레임마다 뒤집히고, 뒤집힐 때마다 절점 간격이
  // outside_spline_scale_(1.75배)만큼 점프한다. 데드밴드 아래에서는
  // "바깥쪽 없음"으로 보고 배율을 적용하지 않는다.
  // 1.0 = 2m 구간 헤딩 변화 0.1 rad (평균 곡률 0.05, 반경 20m).
  static constexpr double kOutsideKappaSumMin{1.0};
  // DoSpline 절점 스케일 상한. 조향 기하 하한이 이 배율을 넘게 요구하면
  // "만들어 놓고 나중에 거부"가 아니라 여기서 기하 불성립으로 반환한다.
  static constexpr double kMaxKnotScale{3.0};
  double spline_resolution_{0.1};
  double side_switch_d_tresh_{0.25};
  double obs_timeout_sec_{0.5};
  bool use_prediction_{true};   // Predictive Spliner: 예측 장애물이 원시 관측을 대체
  double trailing_gap_m_{2.5};
  double trailing_decel_{2.5};
  double cmd_speed_scale_{1.0};
  // 고속 회피 기하/속도 예산 (BlendDist / ApplySpeedProfile)
  double blend_lat_acc_max_{3.0};   // 횡 전환에 배정할 그립 [m/s^2]
  double blend_dist_max_m_{8.0};
  // 횡 전환 그립 예산을 고정값 대신 마찰원 잔여로 계산할지 (BlendLatAcc).
  // 실차에서 문제가 생기면 false로 즉시 되돌릴 수 있게 파라미터로 둔다.
  bool blend_use_spare_grip_{true};    // 전환 거리 상한 [m]
  double path_lat_acc_max_{7.0};    // 오프셋 경로 곡률 -> 속도 상한용 그립 [m/s^2]
  // 레이스라인 그립 일관성 캡: 오프셋으로 곡률이 커진 구간에서, 그 지점의
  // 레이스라인이 실제로 쓰던 횡가속(kappa*v^2)을 넘지 않도록 제한한다.
  // inside_speed_scale 같은 고정 배율을 물리 기반으로 대체하는 장치.
  bool offset_grip_cap_{true};
  double path_decel_max_{6.0};      // 회피 궤적 역방향 감속 한계 [m/s^2]
  double path_accel_max_{3.0};      // 회피 궤적 순방향 가속 한계 [m/s^2]
  double path_jerk_max_{8.0};       // 현재 상태 도달 가능성/ax 변화 한계 [m/s^3]
  // "지금 최대로 밟으면 어디까지 줄일 수 있나"(reachable)를 푸는 **플랜트**
  // 제동 능력 [m/s^2]. path_decel_max_ 와 목적이 다르다 — 저쪽은 경로 정형용
  // 상한이고 이쪽은 차가 실제로 낼 수 있는 감속이다. 섞으면 ApplySpeedProfile
  // 의 그립 게이트가 낙관 쪽으로 틀린다 (params.yaml 주석 참조).
  double plant_decel_mps2_{2.38};   // = deployment_profiles.A_DEC_PLANNED
  double planning_delay_sec_{0.10}; // 인지-계획-제어 지연 보수치 [s]
  double vehicle_half_length_m_{0.275};
  double vehicle_half_width_m_{0.15};
  double boundary_tracking_margin_m_{0.05};
  double collision_sigma_{2.0};
  // 정적 장애물의 시간 성장 불확실성 상한 [s]. 브릿지가 모든 장애물에
  // vs_var/vd_var를 상수(0.25)로 붙이는데, CheckPathCollision/FilterObstacles가
  // is_static에도 도착시각 t를 그대로 곱하면 3σ_d ≈ 1.5t가 된다 — 정지 출발
  // (도달 ~2s)이면 요구 여유가 3m로 트랙 폭을 넘어 어떤 정적 회피도 검증을
  // 통과할 수 없다 (run_0806_155621). 정적 물체는 시간이 지나도 안 움직이므로
  // 시간 성장분을 이 값으로 자른다 (분류 오류 대비 완충만 남김).
  double static_time_uncertainty_cap_{0.3};
  // 관측 불확실성 팽창(3σ)의 상한 [m].
  // 도입 근거(과거): 브릿지가 속도 미확정 첫 프레임에 vs_var/vd_var를 "클립
  // 한계의 제곱"(max_abs_vs²=100, max_abs_vd²=16)으로 붙여 3σ가 종방향 9.0m /
  // 횡방향 3.6m가 됐고, 폭 1.1~2.1m 트랙에서 좌우 어느 쪽도 통과 불가가 되어
  // 매번 Tier3 TRAILING(v_target=0)으로 떨어졌다 (rosbag_0806 4런 68회).
  // ⚠️ 그 브릿지 동작은 같은 커밋(b934d0e)에서 이미 폐기됐다 — 지금은
  //    init_vel_std(0.5)² = 0.25를 쓴다. 즉 이 캡이 막던 9.0m/3.6m 케이스는
  //    더 이상 발생하지 않고, 지금 잘리는 것은 정상 케이스다:
  //    브릿지 position_var_floor=0.01 -> 3σ_d = 3*0.1 = 0.30 > 0.25.
  //    결과적으로 collision_sigma를 올려도 여기서 잘려 효과가 없다.
  //    (두 값을 함께 봐야 한다 — params.yaml의 collision_sigma 항목 참조.)
  double max_uncertainty_m_{0.25};
  // 회피 기하 계획에 쓰는 최소 속도 [m/s] — PlanSpeed() 참조
  double plan_speed_floor_{1.5};
  bool trailing_active_{false};
  // "정적 장애물 정지" 이벤트 상승엣지용 (Tier3 정적/동적 분리)
  bool static_stop_active_{false};
  // 이 속도 이하에서만 크리프 탈출(진입 블렌드가 hold 구간까지 연장)을
  // 허용한다 [m/s]. 주행 중 허용하면 장애물 옆에서 횡전환이 끝나는
  // 위험한 기하가 나온다 — 정지 라치를 푸는 용도로만 쓴다.
  double creep_escape_speed_{0.6};
  // 라인 스위치 (기본 maneuver)
  bool use_lane_switch_{true};       // false면 기존처럼 DoSpline을 1순위로
  double lane_blend_out_m_{5.0};     // raceline 복귀 블렌드 길이 [m]
  // ── 조기 라인 스위치 ──────────────────────────────────────────────────
  // BuildLaneSwitch의 진입 블렌드는 "그립상 최소 전환거리 x 1.2"이고
  // blend_start가 hold_start에서 딱 그만큼만 뒤로 물러나므로, 회피는 항상
  // **최소 거리에서 최대 횡가속**으로 실행된다. 늦게 시작하는 근거는
  // run_0806_155621 — 긴 진입 블렌드가 장애물과 무관한 상류의 좁은 구간을
  // 오프셋 상태로 통과해 그 벽에 걸렸다 — 인데, 그 근거는 "상류가 좁을 때"만
  // 성립한다. 상류가 넓고 곧으면 늦출 이유가 없다.
  //   0808 run_185049: 장애물 s=31.7 앞 직선(kappa~0, 좌벽 1.07m)이 4.7m
  //   비어 있는데 횡이동은 마지막 1.0~2.5m에서 일어났고, 6랩 중 4랩에서
  //   조향이 하드웨어 한계(20.6도)에 포화하고 rate까지 clip됐다.
  // 이 값보다 곡률이 작고, 진입 구간 내내 목표 오프셋이 벽 안에 들어가면
  // 가용 거리를 다 써서 완만하게 옮긴다. 피크 횡가속은 6A/L²이므로 L을
  // 1.8배로 늘리면 3분의 1이 된다. 0이면 이 완화를 끈다(옛 동작).
  double early_switch_kappa_max_{0.10};   // [rad/m] 0.10 = 반경 10m
  bool spline_fallback_active_{false};  // "모든 레인 막힘 -> 스플라인" 상승엣지 이벤트용
  // ── 원호 이탈 (BuildEscapeArc) ──────────────────────────────────────────
  bool escape_arc_enabled_{true};
  // 원호 기동에 배정할 횡가속 [m/s²]. blend_lat_acc_max_(3.0)가 아니라 더
  // 큰 값을 쓰는 이유: 이건 정상 주행의 전환이 아니라 마지막 회피 수단이고,
  // 여기서 아끼면 그 다음은 정지다. path_lat_acc_max_(7.0) 이하로 둘 것.
  double escape_arc_lat_acc_{5.5};
  // 이 속도 미만에서는 원호를 시도하지 않는다. 저속에서는 이미 정상 회피가
  // 성립하고(그립 곡률이 v² 로 커진다), 굳이 헤딩을 남길 이유가 없다.
  double escape_arc_min_speed_{1.2};
  // 기동 끝에 남는 헤딩 상한 [rad]. 이보다 크면 복귀가 벽으로 간다.
  double escape_arc_max_heading_{0.70};
  bool escape_arc_active_{false};       // 상승엣지 이벤트 + 컨트롤러 게이팅용
  // "회피 포기(트랙경계)" 상승엣지 이벤트용. 이 사유는 매 사이클 재발생하므로
  // (30Hz x 8초 = 207회 실측) 엣지에서만 발행하지 않으면 pitwall 로그가 묻힌다.
  bool boundary_abort_active_{false};   // 직전 사이클에 경계 abort가 있었나
  bool boundary_abort_now_{false};      // 이번 사이클 DoSpline이 경계 abort했나
  // 이중장애물 재계획 파라미터 (mina Task 2)
  double path_check_margin_{0.15};   // 경로-장애물 충돌 판정 횡방향 여유 [m]
  double cluster_gap_s_m_{8.0};      // 이 s 간격 이내 장애물은 병합 후보 [m]
  double cluster_gap_d_m_{0.8};      // 사이 corridor가 이보다 좁으면 병합 [m]
  // (evasion_dist_min_ 제거 — 2026-08-10)
  // "벽이 좁을 때 apex를 여기까지 줄여서라도 시도한다"는 하한이었으나,
  // 모든 소비처가 max(evasion_dist_min_, RequiredObstacleClearance(obs)) 로
  // 쓰는데 후자가 항상 더 컸다. 브릿지가 position_var_floor=0.01 /
  // velocity_var_floor=0.25 를 상수로 붙여 UncertaintyMargin 이 사실상 고정이라
  // RequiredObstacleClearance = max(path_check_margin, 차반폭) + σ 는
  // 실차에서 0.35~0.40 으로 굳어 있고, evasion_dist_min_(0.20)은 max 의
  // 승자가 될 수 없었다. 회피 가부의 실제 하한은 RequiredObstacleClearance
  // 하나이므로 그것만 남긴다.
  // squeeze 감속(apex가 벽 마진으로 축소된 회피를 배율로 감속) 제거됨.
  // 좁은 corridor에서의 속도 제한은 오프셋 곡률 기반 물리 캡
  // (OffsetKappa + path_lat_acc_max + offset_grip_cap)이 담당한다.
  // 주의: 벽 여유(RequiredBoundaryMargin)는 아직 속도와 무관한 고정값이므로
  // 추적오차의 속도 의존분은 현재 어디에서도 보상하지 않는다.
  double trailing_engage_dist_m_{8.0};    // trailing 감속을 시작하는 정지점 앞 거리 [m]
  // 속도 명령 -> 실제 감속 사이의 구동계 지연 [s]. 정지 프로파일이
  // v²/(2a)만 보면 이 지연 동안 굴러간 거리만큼 정지점을 넘는다
  // (rosbag_0806 실측: trailing_gap 1.5m 대비 0.24~0.68m에 정지).
  double trailing_lag_sec_{0.3};
  // 정지 프로파일 램프의 최대 감속률 [m/s^2]. 계단 대신 현재 속도에서
  // 이 비율로 내려오는 곡선을 하한으로 깐다 (params.yaml에 근거).
  double trailing_ramp_decel_{2.0};
  // 기하 하한 초과가 **아닌** 사유로 회피가 실패했을 때의 목표 속도 [m/s].
  double trailing_retry_speed_{1.5};
  // ApexFor의 목표 간격을 검증기 요구치보다 이만큼 더 준다 [m].
  // 동률(목표 == 충돌박스 경계)이면 수 mm 오차로 매 사이클 자기 거부한다.
  double apex_clearance_eps_m_{0.05};
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
  // 커밋된 회피를 파기할 수 있는 최소 관측 수 (StableObstacles).
  int commit_break_min_obs_{3};
  // coasting(is_visible=false) 장애물에게 커밋 파기 권한을 줄 것인가.
  // false(기본)면 외삽만으로 성립한 회피를 깨지 못한다 — rosbag_0811 에서
  // 두 콘 사이 committed_break 가 전부 근거 없는 coasting 트랙 때문이었고,
  // 그때마다 회피가 트레일링(급감속)으로 떨어졌다.
  // 여전히 **경로 계획의 입력**으로는 쓰인다 (가림 중인 실물을 잊지 않도록).
  // 막는 것은 "이미 성립한 회피를 뒤집는" 권한뿐이다.
  bool commit_break_requires_visible_{true};

  // 추월 섹터 파라미터
  bool sector_gating_{true};
  double ot_min_clearance_{0.95};
  double ot_max_kappa_{0.1};         // 섹터 판정 최대 곡률 [rad/m] (0.1 = 반경 10m)
  double ot_sector_min_len_m_{8.0};
  // 섹터 안 회피 속도 배율. **1.0 초과 = 부스트**이며 상한은 gb_vmax_ 다.
  // 2026-08-10 이전에는 생성자 검증이 1.0 초과를 거부해서, 헤더/파라미터
  // 주석이 약속한 "섹터 내 공격 모드"가 원리적으로 성립할 수 없었다
  // (배포 yaml 3개 모두 1.0 = 항등). 이제 허용하되 기본값은 1.0 유지.
  double ot_speed_scale_{1.0};
  std::vector<double> ot_sectors_param_;                 // 수동 지정 (flat s0,s1,...)
  std::vector<std::pair<double, double>> ot_sectors_;    // [s_start, s_end] (wrap 허용)
  // "직전 사이클에 회피를 시도했는가"의 상승엣지 래치.
  // 이름이 in_ot_sector_ 였지만 정적 장애물은 섹터 밖에서도 참이라 섹터
  // 소속과 무관하다 — 실제 의미로 개명했다.
  bool evade_attempt_active_{false};
  // 진행 중인 정적 회피 커밋 (경로 끝까지 유지 — 도중 소멸 시 벽 충돌)
  f1tenth_planning_msgs::msg::WpntArray committed_wpnts_;
  visualization_msgs::msg::MarkerArray committed_mrks_;
  double committed_start_s_{-1.0};
  double committed_until_s_{-1.0};
  /// 커밋을 만들게 한 장애물의 뒷 가장자리 s. 커밋 조기 해제(새 장애물이
  /// 커밋 경로 밖에 나타났을 때)가 "아직 그 장애물 옆을 지나는 중"인지
  /// 구별하는 데 쓴다 — 기동 중에 커밋을 풀면 raceline 전속 경로로
  /// 순간 전환돼 반대편 벽을 친다(커밋 도입 근거와 같은 실측).
  double committed_obs_end_s_{-1.0};
  // DoSpline이 이번 사이클에 쓸 값 (섹터 안: 공격 모드 / 밖: 기본값)
  double active_speed_scale_{1.0};

  // SP 레인체인지 상태
  bool sp_lane_change_{true};
  // 추월 완료 판정: 상대 앞머리가 ego 뒤로 이 간격만큼 벌어져야 복귀 [m]
  double lc_pass_clear_m_{4.0};
  std::vector<f1tenth_planning_msgs::msg::Wpnt> sp_raw_;   // SP 라인 원본 (자체 s 좌표)
  std::vector<double> sp_d_;    // gb 인덱스별 SP 라인의 IQP frenet 횡 오프셋
  std::vector<double> sp_vx_;   // gb 인덱스별 SP 라인 속도
  bool has_sp_{false};          // 매핑 완료 여부
  bool sp_mapping_attempted_{false};
  double sp_mapping_max_residual_m_{0.5};
  double vehicle_wheelbase_m_{0.33};
  double vehicle_max_steering_rad_{0.3578};
  double vehicle_max_steering_rate_radps_{2.0};
  // 계획 게이트가 쓰는 "실제로 달성되는 곡률 / kinematic 자전거모델 예측"
  // 비율. 1.0이면 tan(delta_max)/L 그대로 (예전 동작).
  // vehicle_max_steering_rad_ 는 steer_calib 로 측정·검증된 서보 기구 한계라
  // 건드리면 안 되므로(TUNING_STATUS §4, CLAUDE.md), 타이어 그립 향상처럼
  // "같은 조향각으로 더 돈다"를 반영할 때는 이 배율만 올린다.
  double steer_kappa_scale_{1.0};
  // BlendGeomFloor 의 smoothstep 근사 여유. 실제 샘플 피크 곡률이 근사식
  // 6A/L² 보다 큰 것을 흡수한다. 1.0에 가까울수록 하한이 짧아진다(= 완화).
  double geom_floor_margin_{0.8};
  // 트랙 곡률이 조향 예산을 거의 다 먹었을 때 남겨두는 최소 비율.
  // 이 아래로 내려가면 하한이 발산해 로그가 inf 로 찍히고 상위 로직이
  // "가용 대비 얼마나 모자란가"를 표현할 수 없다. 올리면 헤어핀에서
  // 하한이 짧아진다(= 완화).
  double geom_floor_kappa_ratio_{0.15};
  // ApplySpeedProfile 이 경로를 통째로 버리는 곡률 문턱의 배율.
  // 1.0이면 조향 한계를 1e-3 이라도 넘는 샘플 하나에 경로 전체가 죽는다.
  // 1.0보다 크면 그 밴드 안의 샘플은 살리되 횡가속 캡으로 속도만 눌린다.
  double steer_kappa_reject_ratio_{1.0};
  // escape 원호 전용 곡률 기각 배율 (2026-08-12).
  // escape 경로는 포물선 원호가 끝나는 지점에서 hold(기울기 0)로 급전환하며
  // 잔여 헤딩 2A/L 을 한 점에서 꺾는다. 그 접합부 곡률이 1.14~1.77 로 나와
  // 통상 배율(1.15 -> 문턱 1.303)에 걸려 **escape 가 한 번도 발행되지 못했다**
  // (2026-08-12 시뮬: escape_speed_profile 91, escape_active 0).
  // 이 배율을 크게 주면 꺾인 경로가 그대로 발행된다 — 차가 그 꺾임을 못 따라가
  // **밖으로 밀리는(언더스티어) 거동을 실제로 관측**하기 위한 것이다.
  // 근본 해결은 접합부에 기울기 완화 구간을 넣어 꺾임 자체를 없애는 것이고,
  // 그러면 접합 곡률이 0.785 로 떨어져 이 배율이 다시 필요 없어진다.
  double escape_steer_kappa_reject_ratio_{2.0};
  // 위 두 완화(require_current_feasible 면제 + 곡률 밴드 확대)를 켜는 스위치.
  // **기본 false = 커밋된 동작 그대로.** true 로 켜면 escape 원호가 자기
  // 접합부 꺾임을 안은 채로 발행되고, 차가 그걸 못 따라가 밖으로 밀리는
  // 거동을 관측할 수 있다. 관측 전용이며 실차에 켠 채로 올리면 안 된다.
  // 근본 해결(접합부 기울기 완화)이 들어가면 이 스위치는 삭제 대상이다.
  bool escape_relax_for_observation_{false};
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
  // 사이드 유지 잠금 (SideKeepActive 주석 참조). detector가 장애물 id를
  // 자주 갈아치우므로 id가 아니라 "그때 그 장애물이 있던 s"로 동일성을
  // 판정하고, 오래된 잠금은 시간으로 만료시킨다.
  double ot_side_obs_s_{-1.0};
  rclcpp::Time ot_side_stamp_{0, 0, RCL_ROS_TIME};
  bool has_ot_side_obs_{false};
  static constexpr double kOtSideLockDistM{1.5};   // 같은 장애물로 볼 s 오차
  static constexpr double kOtSideLockSec{1.0};     // 잠금 유효 시간
  // 커밋된 사이드 자체의 유효 시간 (OtSideCommitFresh). kOtSideLockSec 과 같은
  // 1.0s 라 (1) 같은-장애물 잠금의 동작은 바뀌지 않고, (2) |cur_d_| 기반
  // 유지만 만료를 얻는다. 1.0s = spliner/obs_timeout_sec(0.30)의 3배 —
  // 인지 드롭아웃으로 몇 사이클 회피 생성이 끊겨도 유지되고(그보다 긴 공백은
  // 어차피 상위에서 perception_stale fail-safe 로 처리된다), 장애물을 통과한
  // 뒤에는 확실히 풀린다 (4~6 m/s 에서 1.0s = 4~6 m 주행).
  // ⚠️ 미실측 판단값 — 다음 백에서 회피 중 경로생성 공백을 재서 확정할 것.
  static constexpr double kOtSideCommitValidSec{1.0};
  // 연쇄 회피 그룹 상한. 콘이 줄지어 있어도 hold 가 무한정 늘어나지 않게
  // 막는다 (구간 상한은 별도로 lookahead_m_ 가 잡는다).
  static constexpr size_t kEvasionGroupMax{4};
  // 거부 사유 카운터 (NoteReject/PublishRejectCounts).
  mutable std::map<std::string, int> reject_counts_;
  rclcpp::Time reject_publish_stamp_{0, 0, RCL_ROS_TIME};
  // 이번 사이클에 쌓인 거부 사유(발생 순서, 쉼표 구분). TraceCycle이 소비하고
  // Run() 시작에서 비운다. NoteReject가 const 경로에서도 불리므로 mutable.
  mutable std::string cycle_rejects_;
  bool trace_cycles_{true};
  // 이번 사이클에 Tier1/Tier2가 "조향 기하 하한을 못 채움"(= 어떤 속도로도
  // 불가)으로 거부했는가. 매 Run() 시작에 false로 리셋한다.
  //
  // Trailing(v_target=0 정지 궤적)은 **이 경우에만** 정당하다. 예전 코드는
  // `if (!evasion_valid)` 로 실패 사유를 구분하지 않아,
  // 충돌검사 거부·벽 축소·"이미 장애물 구간 내부"·스팬 초과 같은 일시적
  // 사유에도 전 점 vx=0 궤적을 발행했다 (주석 :935-950은 원래부터 사유를
  // 구분하겠다고 선언하고 있었다 — 코드가 주석을 따라간다).
  // vx=0은 MAP 컨트롤러에서 조향 0을 유발하므로(control_node.cpp:1057)
  // 이 구분이 안전에 직결된다.
  bool lane_switch_geom_infeasible_{false};
  bool free_spline_geom_infeasible_{false};
  // 회피가 **일시적으로** 실패했을 때 트레일링이 유지할 횡 목표 (d_hold /
  // d_apex). Tier1/Tier2가 목표를 계산해 장애물 여유는 만족했지만(clears)
  // 속도/충돌 검증에서 떨어진 경우에만 기록한다.
  //
  // 예전에는 실패하면 트레일링 d램프가 무조건 raceline(d=0)으로 되돌렸다.
  // 그러면 다음 사이클의 횡이동 잔량(d_hold - cur_d_)이 줄지 않아
  // BlendSpeedCap/BlendGeomFloor가 계속 같은 값을 돌려주는데, 그 사이
  // 가용 거리만 v*dt씩 말라 회피가 영원히 성립하지 않는다.
  //   run_0809_231809 4랩 실측: 52.51 "속도 프로파일 불성립"(캡 5.21 <
  //   자차 5.33)으로 거부된 뒤 d가 -0.07~0.14를 벗어나지 못했고, 가용은
  //   3.67 -> 1.15 m로 줄어 52.90에는 기하 하한(1.56)마저 못 채웠다.
  //   같은 시점에 d가 0.30이었다면 잔량 0.17 -> 하한 1.06 < 가용 1.15로
  //   성립했다.
  // 기하 하한 초과(allow_stop)나 perception 비상정지에서는 예전대로 복귀한다.
  double retry_d_target_{0.0};
  rclcpp::Time retry_d_stamp_{0, 0, RCL_ROS_TIME};
  bool has_retry_d_{false};
  static constexpr double kRetryDHoldSec{0.7};  // 목표 유효 시간
  std::mutex mutex_obstacles_;
  std::mutex mutex_gb_wpnts_;

  rclcpp::Subscription<f1tenth_planning_msgs::msg::ObstacleArray>::SharedPtr s_obstacles_;
  rclcpp::Subscription<f1tenth_planning_msgs::msg::ObstacleArray>::SharedPtr s_predicted_obstacles_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr s_frenet_odom_;
  rclcpp::Subscription<f1tenth_planning_msgs::msg::WpntArray>::SharedPtr s_global_waypoints_;
  rclcpp::Subscription<f1tenth_planning_msgs::msg::WpntArray>::SharedPtr s_sp_waypoints_;
  rclcpp::Publisher<f1tenth_planning_msgs::msg::WpntArray>::SharedPtr p_evasion_wpnts_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr p_markers_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr p_sector_markers_;
  // 원호 이탈 중임을 컨트롤러에 알린다. 이 기동은 설계상 조향이 포화 근처로
  // 가므로, control 의 추종 백스톱(조향포화 -> 감속)이 그대로 걸리면 회피와
  // 싸운다. 컨트롤러는 이 신호가 true 인 동안 포화 분기만 건너뛴다.
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr p_escape_active_;
  rclcpp::TimerBase::SharedPtr t_run_;
};

#endif  // PLANNING_PKG__SPLINER_NODE_HPP_
