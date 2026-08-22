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
#include "std_msgs/msg/float32.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

#include "planning_pkg/steer_grip_model.hpp"

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
  void CallbackFrenetOdom(const nav_msgs::msg::Odometry::SharedPtr msg);
  void CallbackGlobalWaypoints(const f1tenth_planning_msgs::msg::WpntArray::SharedPtr msg);
  void Run();

  // ForzaETH spliner 핵심 로직
  std::vector<f1tenth_planning_msgs::msg::Obstacle> FilterObstacles(
    const std::vector<f1tenth_planning_msgs::msg::Obstacle> & obstacles) const;
  // 반환: {side("left"/"right"), d_apex}
  // 사이드 선택. MoreSpaceRaw 가 이번 프레임 관측만으로 고르고, MoreSpace 가
  // 그 위에 **의도 히스테리시스**를 얹는다 (intent_side_ 멤버 주석 참조).
  // 히스테리시스 문턱이 0 이면 둘은 완전히 같다 = 종전 동작.
  std::pair<std::string, double> MoreSpaceRaw(
    const f1tenth_planning_msgs::msg::Obstacle & obs) const;
  std::pair<std::string, double> MoreSpace(
    const f1tenth_planning_msgs::msg::Obstacle & obs) const;
  // 이번 사이클에 고른 사이드를 "의도"로 남긴다. BuildLaneSwitch 성공과
  // 무관하게 매 사이클 찍힌다 — 그것이 last_ot_side_ 와의 차이다.
  void NoteSideIntent(const std::string & side,
                      const f1tenth_planning_msgs::msg::Obstacle & obs);
  // 해당 사이드의 apex d 계산: 목표는 장애물 가장자리 + evasion_dist지만,
  // corridor(장애물-벽)가 좁으면 여유를 장애물/벽 양쪽에 균등 배분한
  // 지점으로 축소 (벽에 최대한 붙이는 방식은 추적오차로 벽 접촉)
  double ApexFor(
    const std::string & side,
    const f1tenth_planning_msgs::msg::Obstacle & obs) const;

  // ── 기동 전체의 벽 여유 ────────────────────────────────────────────────
  // apex 를 잡는 MinWallInSpan 창은 hold 구간(장애물 앞뒤 0.5m)뿐이다.
  // 그런데 차는 그 뒤로 복귀 블렌드만큼 더 오프셋을 물고 간다 — 장애물
  // **직후에 좁아지는** 코너에서는 hold 구간이 넓어도 복귀 구간이 벽이다.
  //   run_0815_191056 s≈27.6 (헤어핀2 출구): hold 구간 우벽 0.84m 라
  //   MoreSpace 가 right 를 골랐는데, 1.2m 뒤 우벽이 0.47m 로 줄어 복귀가
  //   조향 기하로 불가능했다. 9랩 중 5랩이 이 선택을 했고 매 랩 벽예비가
  //   0.02~0.13m 까지 떨어지다 마지막 랩에 2.9초 벽 접촉으로 끝났다.
  //   같은 장애물을 left 로 가면 벽예비 +0.10m 에 복귀도 성립한다.
  // 그래서 사이드 선택에는 진입 블렌드 + hold + 복귀 블렌드 **전 구간**의
  // 최악 벽예유를 쓴다. apex 클램프 자체는 그대로 hold 구간 기준이다
  // (복귀 구간까지 최소 벽폭으로 clamp 하면 apex 가 사라져 회피가 죽는다 —
  //  좁은 것은 그 지점이지 스팬 전체가 아니다. TryLaneSwitch 의 지점별
  //  클램프와 같은 근거).
  //
  // reachable: 복귀 블렌드가 벽 수축을 조향 한계 안에서 따라갈 수 있는가.
  //   각 지점의 |d| 상한까지 내려오는 데 필요한 피크 곡률 6*Δd/L² 을
  //   SteerKappaMax() 와 비교한다. false 면 그 사이드는 "계획은 그려지지만
  //   차가 그릴 수 없는" 경로다.
  // 반환: 최악 벽예유 [m] (차체 반폭 포함. 음수면 차체가 벽 안).
  double ManeuverWorstMargin(
    double apex, const f1tenth_planning_msgs::msg::Obstacle & obs,
    bool * reachable = nullptr) const;

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
  // d_hold 를 유지하는 구간에 걸치는 **다른** 장애물이 그 오프셋을 막는가.
  // TryLaneSwitch 의 clears 판정이 closest_obs 하나만 봐서 생긴 구멍을 막는다
  // (run_0818_205422 25.31s 충돌 — 그 함수의 주석 참조). 막는 장애물이 있으면
  // 그 박스를 *blocker 로 돌려준다. 사이드는 묻지 않는다: d_hold 가 박스 왼쪽
  // 이든 오른쪽이든 요구 간격 밖이기만 하면 이 오프셋은 그것을 지나갈 수 있다.
  bool HoldClearsOthers(
    double d_hold,
    const f1tenth_planning_msgs::msg::Obstacle & closest_obs,
    double hold_through_s,
    const std::vector<f1tenth_planning_msgs::msg::Obstacle> & all_ahead,
    f1tenth_planning_msgs::msg::Obstacle * blocker) const;

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

  // 이 트랙을 몇 프레임 봤는가 (cls_tracks_ 의 관측 이력 길이).
  // 세 게이트(정지 권한 / 커밋 승격 / 커밋 파기)가 모두 이 값을 문턱과
  // 비교하므로 조회를 한 곳에 둔다 — 예전에는 같은 lookup 이 세 번 복사돼
  // 있었다. cls_tracks_ 는 클러스터 **전** 원시 id 로 채워지는데
  // ClusterObstacles 가 병합 시 min(a.id, b.id) 를 남기므로 이 조회는
  int ObsSeen(int32_t id) const;

  // 이 id 가 spliner 분류 이력에서 한 번이라도 동적이었나 (ObsClsTrack::ever_dynamic).
  // 모르는 id 면 false — 처음 보는 트랙에 마진을 주지 않는다.
  bool WasEverDynamic(int32_t id) const;

  // 이 id 가 **실제로** ever_moved_min_disp_m_ 이상 움직인 것이 관측됐나.
  // 분류기 판단(WasEverDynamic)이 아니라 관측된 변위다 — 정지 권한처럼
  // 안전에 직결되는 판단에만 이쪽을 쓴다 (ObsClsTrack::max_disp 주석 참조).
  bool WasEverMoved(int32_t id) const;

  // ── 동적 상대 종방향 추종 ──────────────────────────────────────────────
  // 상대 뒤 dyn_follow_gap_m_ 를 유지하기 위한 **속도 상한** [m/s].
  // 무한대를 돌려주면 "개입하지 않는다" = 상대를 무시하고 레이스라인 속도로
  // 간다. 이 함수는 **종방향만** 정한다 — 횡(회피)은 정적 파이프라인 몫이고,
  // 그래서 "회피가 최우선, 추종은 속도만"이 구조적으로 성립한다.
  // 2026-08-23 로 동적 상대에 대한 횡 기동이 전부 폐기됐으므로, 이 함수와
  // BuildTrailing 이 **동적 상대에 대한 플래너 출력의 전부**다.
  //
  // 개입 시점이 왜 gap 그 자체가 아닌가:
  //   목표 간격에서만 반응하면 접근속도 Δv 를 지울 거리가 없다. 실차
  //   plant_decel 2.38 기준 Δv=3 m/s 를 지우는 데만 1.89 m 가 필요한데
  //   목표 간격은 1.0 m 다. 그래서 개입 조건을
  //     gap <= gap_target + Δv²/(2a) + v·T
  //   로 둔다. 정속 추종(Δv≈0)에서는 거의 gap_target 에서 열리므로 "1.0 m
  //   밖은 신경 쓰지 않는다"는 계약이 그대로 지켜지고, 급접근일 때만 미리
  //   열린다.
  //
  // 반환값의 두 항:
  //   (1) 비례 추종  v_opp + kp·(gap - gap_target)  — 멀면 가속, 가까우면 감속
  //   (2) 안전 상한  sqrt(v_opp² + 2a·room)         — 상대가 지금 밟아도
  //       gap_min 안쪽으로 들어가지 않는 속도. room 에서 반응지연 주행분을
  //       먼저 뺀다 (BuildTrailing 의 reachable_v 와 같은 형태).
  // 둘 중 작은 쪽을 쓴다. (2)가 상대 급정거에 대한 **1 사이클(33 ms) 반응**
  // 이다 — 분류기가 STATIC 으로 뒤집는 데는 최소 0.67 s 가 걸리므로 그보다
  // 20배 빠르고, 그 뒤에 정적 파이프라인이 정지/우회를 맡는다.
  double OpponentFollowCap(
    const f1tenth_planning_msgs::msg::Obstacle & opp) const;

  // 관측 창(cls_tracks_.vs_hist, 최대 kClsVsWindow)의 vs 평균.
  // 트랙 이력이 없으면 fallback 을 그대로 돌려준다. BuildTrailing 의 목표
  // 속도에서 단일 프레임 노이즈(실측 std 0.43~0.45 m/s)를 걷어내는 용도.
  // ⚠️ vs_hist 는 ClassifyObstacles 가 **정적 덮어쓰기 전** 원시 vs 를 쌓는다.
  //    정적 장애물은 그 뒤 obs.vs=0 이 되므로 이 함수를 태우지 말 것.
  double ObsVsMean(int32_t id, double fallback) const;

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
  // blocking_obs 를 주면 그 최근접 차단 장애물을 가리키게 한다 (obstacles
  // 안의 원소를 가리키므로 그 벡터보다 오래 쓰면 안 된다). 커밋 꼬리 절단이
  // 다음 기동의 횡 진폭을 이 장애물에서 직접 뽑는 데 쓴다 — 상수 진폭으로
  // 재계획 여유를 잡으면 고속에서 모자란다(BlendDist 가 v 에 정비례).
  // contact_slack 을 주면 그 최근접 차단점에서 **차체 가장자리와 장애물
  // 가장자리 사이의 실제 거리**를 담는다 (양수 = 아직 안 닿는다). 충돌 박스는
  // 차 반폭 + 관측 분산이라, "박스에 들어갔다"와 "차가 닿는다" 사이에 분산만큼
  // 여유가 있다. 커밋 파기 확정이 그 둘을 구별하는 데 쓴다.
  bool CheckPathCollision(
    const std::vector<f1tenth_planning_msgs::msg::Wpnt> & path,
    const std::vector<f1tenth_planning_msgs::msg::Obstacle> & obstacles,
    double * blocking_s,
    const f1tenth_planning_msgs::msg::Obstacle ** blocking_obs = nullptr,
    double * contact_slack = nullptr) const;
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

  // check_obs: 완성된 트레일링 경로를 이 장애물들과 충돌검사한다. 트레일링의
  // 횡 램프는 종전까지 **벽만**(WithinTrackBoundary) 검사했고 장애물은 보지
  // 않았다 — "현재 d 유지"가 장애물 정중앙인 것을 아무도 몰랐다(7:23 접촉).
  // nullptr 이면 종전 동작(검사 없음).
  bool BuildTrailing(
    const f1tenth_planning_msgs::msg::Obstacle & closest_obs,
    f1tenth_planning_msgs::msg::WpntArray & out_wpnts,
    visualization_msgs::msg::MarkerArray & out_mrks,
    bool allow_stop = true,
    const std::vector<f1tenth_planning_msgs::msg::Obstacle> * check_obs =
      nullptr);

  // 추월 섹터: 회피 기하(벽 여유 >= ot_min_clearance)가 성립하고 코너가
  // 아닌(|kappa| <= ot_max_kappa) s 구간을 raceline에서 자동 산출.
  // ⚠️ 2026-08-23 현재 이 섹터는 **진단·시각화 전용**이다. 동적 회피를
  //    폐기하면서 이 섹터가 게이팅하던 대상이 사라졌고(정적 회피는 원래부터
  //    섹터와 무관하게 항상 시도한다), 마지막 소비처였던 직선 인코스 추월도
  //    같이 없앴다. 남겨 두는 것은 마커와 TraceCycle 의 in_sector 열이
  //    "여기가 직선이었다"를 백 분석에서 계속 읽히게 하기 위해서다.
  void ComputeOtSectors();
  bool InOtSector(double s) const;
  void PublishSectorMarkers();

  // ── 추월 기동은 없다 (2026-08-23) ─────────────────────────────────────
  // 여기 있던 TryStraightOvertake(직선 인코스 레이트브레이크)를 폐기했다.
  // 그전에 폐기한 SP 레인체인지·준정적 추월과 같은 이유다: 동적 상대의
  // 상태 추정(vs)이 노이즈와 같은 크기라 횡 기동을 매 사이클 다시 풀게
  // 되고, 커밋을 걸어도 churn 이 남아 접촉으로 이어졌다.
  // **동적 상대에 대한 플래너 출력은 이제 속도 하나뿐이다** —
  // OpponentFollowCap(경로가 있을 때의 캡) 또는 BuildTrailing(없을 때의
  // 제동 프로파일). 횡 경로 생성(BuildLaneSwitch / DoSpline / BuildEscapeArc)
  // 은 전부 정적 장애물 전용이다.

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
  // speed >= 0 이면 그 속도의 조향 능력으로, 음수면 PlanSpeed() 로 판정한다.
  double BlendGeomFloor(double amplitude, double kappa_track = 0.0,
                        bool one_sided = false, double s_from = -1.0,
                        double speed = -1.0) const;

  // 길이 L 의 블렌드가 조향 한계 안에 들어오는가 — 지점별·부호 유지 검사.
  //   |kappa_track(s) + d''(s)/geom_floor_margin_| <= kappa_steer_max
  // 근사 여유(geom_floor_margin_)는 "실제 샘플 피크가 smoothstep 근사보다
  // 크다"를 흡수하는 것이므로 **기동 항에만** 건다 (스칼라 식과 같은 이유).
  // 상쇄 크레딧은 (1-geom_floor_kappa_ratio_)*kappa_steer_max 로 제한한다 —
  // 레이스라인 kappa 는 d=0 기준이고 자차는 추종 오차만큼 벗어나 있으므로
  // "코너가 대신 돌아준다"를 무한정 믿지 않는다.
  // speed >= 0 이면 그 속도에서의 조향 능력(SteerKappaMaxAt)으로 판정한다.
  // 음수면 PlanSpeed() 를 쓴다.
  bool BlendGeomFeasible(double amplitude, double blend_len, double s_from,
                         bool one_sided, double speed = -1.0) const;

  // ── 조향 한계는 두 개다 (2026-08-23) ──────────────────────────────────
  // (1) SteerKappaKinematic() — 순수 기하:
  //       steer_kappa_scale_ * tan(|delta_max|) / wheelbase   (속도 무관)
  //     "조향각 자체가 모자라다"는 판정이고, 감속으로 살릴 수 없다.
  //     경로를 **통째로 버리는** 하드 기각은 전부 이 값을 쓴다 — 그래야
  //     기존 기각 문턱이 한 톨도 안 움직인다.
  // (2) SteerKappaMaxAt(v) — 그 속도에서 **실제로** 낼 수 있는 곡률:
  //       min( (1), 조향 LUT 표의 a_lat(v)/v^2 )
  //     계획이 "차가 낼 수 있는 것"만 약속하게 하는 값이고, 넘으면 감속으로
  //     살릴 수 있다. 회피 기동의 기하 게이트(BlendGeomFeasible/Floor/
  //     SpeedCap)와 속도 프로파일 캡이 이 값을 쓴다.
  //     표가 비어 있으면 (2) == (1) 이라 종전과 완전히 동일하다.
  //     근거·표·되돌리는 법: steer_grip_model.hpp 머리주석.
  double SteerKappaKinematic() const;
  double SteerKappaMaxAt(double speed) const;
  // 종전 이름 유지 — PlanSpeed() 기준 (2) 를 돌려준다.
  double SteerKappaMax() const;

  // 가용 거리 available 안에서 amplitude만큼 횡이동할 수 있는 최대 속도 [m/s].
  // 기하 하한을 못 채우면 0(= 어떤 속도로도 불가), 횡이동이 0에 가까우면 무한대.
  // 회피 가부를 "지금 속도에서 되나?"(이진 거부)가 아니라 "되는 속도는
  // 얼마인가?"로 바꾸는 함수다 — 정적 장애물은 속도가 자유 변수이므로
  // 감속으로 성립시킬 수 있는데도 매번 Tier3(정지)로 떨어지고 있었다.
  // s_from >= 0 이면 기하 가부를 "최소 하한 계산 후 비교"가 아니라 **실제로
  // 쓸 길이(= available)에서 지점별로** 판정한다. 블렌드는 available 을 그대로
  // 쓰므로 이쪽이 근사 없이 정확하다.
  // 2026-08-23: 조향 능력이 속도의 함수가 되면서 이 함수는 **후보 속도에서**
  // 기하를 판정한다. 지금 속도로 판정하면 "빠를 땐 무조건 0(정지)"이 되는데,
  // 이 함수의 존재 이유가 바로 그 이진 거부를 없애는 것이었다.
  double BlendSpeedCap(double amplitude, double available,
                       double lat_acc = -1.0, double kappa_track = 0.0,
                       double s_from = -1.0) const;

  // 경로 웨이포인트의 **기하부**를 채운다 (vx_mps 는 호출부 몫).
  // 2026-08-13 이전에는 이 11개 대입이 5곳에 복제돼 있었다
  // (BuildLaneSwitch / DoSpline / BuildEscapeArc 및 지금은 제거된 SP 빌더
  //  둘). 다섯 벌 모두 완전히 동일했지만, 복제라는 사실 때문에
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

  /// ClusterObstacles 가 이번 사이클에 모은 봉쇄 근거(corridor_block_evidence_)
  /// 로 정지 권한 래치를 갱신하고 /planner/corridor_blocked 로 낸다.
  /// 계획 사이클마다 ClusterObstacles **직후** 정확히 1회 호출한다.
  void UpdateCorridorBlockLatch();

  // 회피 거부 사유 누적 카운터. 거부 로그는 전부 THROTTLE(2000ms)인데 루프는
  // 30Hz라 최대 1/60만 남는다 — 0808 run_185049 분석에서 "left가 왜 실패했나"가
  // 대부분 지워져 있었다. 카운터는 전수를 세고 /pitwall/reject_<사유>로 낸다.
  // const 경로(ClusterObstacles 등)에서도 세야 하므로 const + mutable 카운터.
  void NoteReject(const char * reason) const;

  // 거부가 **아닌** 사건을 센다 — 성공(ok_*), 폴백 발행(trailing), 중간 마커,
  // veto 기록(cluster_merge_blocks_track).
  //
  // ⚠️ 2026-08-19 신설. 그 전에는 이것들도 NoteReject 로 세어서
  // `/pitwall/reject_trailing` 같은 이름으로 나갔고, 읽는 사람이 전부
  // "거부 횟수"로 오독했다 (run_0818 진단 두 건이 같은 함정에 빠졌다).
  // 실제로 `NoteReject("trailing")` 은 "Trailing fallback ..." 성공 로그 직후
  // `return true` 바로 앞에 있다 — **발행 성공 카운터**다.
  // 이쪽은 `/pitwall/outcome_*` 로 나가고, reject_* 에는 진짜 거부만 남는다.
  void NoteOutcome(const char * outcome) const;
  // 직전 ApplySpeedProfile 실패의 원인 위치를 사람이 읽는 문자열로. 곡률
  // 한계였으면 "곡률 idx=i/n(p%) k=... >한계 ...", 아니면 "도달불가(감속)".
  std::string SpeedRejectDetail() const;
  // 회피 목표(d_hold/d_apex)를 "재시도 중 유지할 값"으로 기록한다.
  void NoteRetryTarget(double d_target);
  void PublishRejectCounts();
  /// 커밋 경로를 release_s 에서 잘라 낸다 (꼬리가 blocking_s 의 장애물과
  /// 충돌할 때). 웨이포인트·기하 상한·마커를 함께 줄이고 committed_until_s_
  /// 를 새 끝으로 옮겨, 커밋이 그 자리에서 path_end 로 정상 만료되게 한다.
  /// 남는 점이 너무 적으면(기동조차 못 덮으면) 아무것도 하지 않는다 —
  /// 그 경우 호출부의 조건이 이미 성립하지 않으므로 도달하지 않는다.
  void TruncateCommitAt(double release_s, double blocking_s);

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

  // 자차의 경로 접선 속도 [m/s] = hypot(cur_vs_, cur_vd_).
  //
  // ApplySpeedProfile 의 전방 가속 엔벨로프와 성립성 시뮬레이션의 시작
  // 속도는 "차가 지금 실제로 내고 있는 속도"여야 한다. 예전에는 |cur_vs_|
  // (frenet ṡ = s-투영)를 썼는데, 회피 중에는 횡이동분이 통째로 빠진다:
  //   run_0822_172617 실측 — 회피 중 ṡ/|v| 중앙 0.917 (p10 0.756),
  //   회피 없을 때 0.991. |d| 중앙 0.43 m.
  // 그 결과 경로 머리 속도가 낮은 ṡ 에 앵커되고(머리 vx 가 cur_vs_ ±5 %
  // 안인 사이클 65.5 %), committed_hold(회피 사이클의 67 %)의 1 m 앞
  // 계획속도가 레이스라인의 0.63 에 머물렀다 — 그립 캡(중앙 5.31)과
  // 조향률 캡은 전혀 걸리지 않은 채로. **횡으로 크게 피할수록 플래너가
  // 자차를 느리다고 보고 더 세게 제동을 지시하는** 구조였다.
  //
  // 낮은 값을 적어 보내는 것은 컨트롤러 쪽에서도 금지 방향이다 — 조향이
  // waypoint vx 로 계산되므로 (계획/실제)² 배로 덜 꺾는다 (ApplySpeedProfile
  // require_current_feasible 블록 주석의 0807 코스아웃 메커니즘).
  double CurSpeed() const;

  // 블렌드를 **지나는 동안** 차가 실제로 낼 속도 [m/s].
  //
  // BlendDist(L = v*sqrt(6A/a_lat)) 는 속도 하나를 받는데, 지금까지 그 자리에
  // PlanSpeed()(= 계획 시점의 현재 속도)가 들어갔다. 가속 중인 직선에서는
  // 블렌드가 끝날 때의 속도가 훨씬 커서, 그 길이로 실제 낸 피크 횡가속이
  // 예산을 크게 넘는다 (a_lat 은 v²에 비례하는데 L 은 옛 v 로 잡혀 있다).
  //   run_0815_191056 t=109.5~110.4 (s=32.3->35.6, A=0.67):
  //     계획 시점 v=3.5 -> L=3.3m. 블렌드 끝 실측 v=5.9 m/s.
  //     실제 피크 a_lat = 6*A*v²/L² = 12.8 m/s² = 예산 8.83 의 1.46배.
  //     -> 조향 -14~-18도, 계획 대비 횡오차 +0.22m 로 벌어진 채 복귀에
  //        들어가 오차가 +0.57m 까지 커지며 좌벽 접촉 (s≈1.5).
  // 그래서 레이스라인 목표속도(구간 최대)와 구동계 가속으로 도달 가능한
  // 속도 중 작은 쪽을 쓰되, PlanSpeed() 아래로는 내려가지 않는다 — 결과적으로
  // 블렌드는 길어지기만 한다(= 완만해지기만 한다).
  // span 은 블렌드 길이 추정치. 길이<->속도가 서로를 참조하므로 호출부에서
  // 한 번 반복해 수렴시킨다.
  double BlendPlanSpeed(double s_from, double span) const;

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
  // 곡률 기각 배율(steer_kappa_reject_ratio_)의 밴드를 넘긴 샘플만 살아나는
  // 것이 아니라, 밴드 **안**의 샘플은 그대로 sqrt(a_lat/k) 횡가속 캡을 받아
  // 속도가 강하게 눌린다는 점에 주의.
  // out_geom_ceiling: 기하 전용 속도 상한(곡률·조향레이트·역방향 감속까지만
  //   적용, cur_vs_ 무관)을 채워 돌려준다. 커밋 경로가 이것을 보관한다.
  // in_geom_ceiling: 주면 위 기하 단계를 통째로 건너뛰고 이 값에서 시작한다.
  //   커밋 재생 전용 — 이미 처리된 경로에 그냥 재실행하면 OffsetKappa 가
  //   이중 적용되고(kappa_radpm 이 결과로 덮여 있다) base vx 도 이미 깎여
  //   있어 속도가 절대 회복되지 않는다.
  // ⚠️ 실패(false)로 빠져도 wpnts 는 **일부만** 갱신된 상태가 아니다:
  //    곡률 기각 검사를 먼저 전수로 돌고, 통과한 뒤에야 kappa_radpm/vx_mps 를
  //    쓴다. 반환값을 무시하는 호출부(BuildTrailing)가 있으므로 이 순서가
  //    지켜져야 한다.
  // steer_reject_idx_out: 주면 **소프트 모드**가 된다 — 곡률이 조향 기각
  //   밴드를 넘어도 경로를 버리지 않고, 첫 위반 인덱스를 여기 넣은 뒤
  //   sqrt(a_lat/k) 횡가속 캡을 전 구간에 그대로 적용한다 (위반 없으면 -1).
  //   버릴 상위 티어가 없는 호출부(BuildTrailing)를 위한 것이다.
  //
  //   왜 필요한가 (2026-08-20, run_0820_201714 4:45):
  //     하드 기각은 "경로를 버려라"인데 트레일링은 마지막 티어라 버릴 것이
  //     없어 반환값을 무시하고 그대로 발행한다. 그러면 **곡률/그립 캡이
  //     하나도 안 걸린 채** 나가고, 남는 수동 제동 곡선은 reachable_v 바닥에
  //     눌려 선두가 현재 속도와 같다 = 캡이 없는 것과 같다. 그 결과 기동이
  //     급할수록(곡률이 클수록) 더 빨리 가는 역전이 생긴다.
  //     실측: s=34.92 에서 k=1.48>1.30 으로 기각되자 발행 경로 속도가
  //     2.4 -> 5.7 m/s 로 튀었고 명령이 4.45 -> 5.45 로 되올라가 5.4 m/s 로
  //     장애물에 접촉했다. 같은 자리 직전 랩(4.25 m/s 진입)은 통과했다.
  //     런 전체 3499 사이클 중 230(6.6%)이 이 상태였고 사유는 전부 곡률이다.
  //   ⚠️ 유한하지 않은 곡률은 소프트 모드에서도 하드 기각이다 — sqrt(a/k) 가
  //      NaN 이 되어 속도를 오염시킨다.
  bool ApplySpeedProfile(
    std::vector<f1tenth_planning_msgs::msg::Wpnt> & wpnts,
    bool require_current_feasible = true,
    std::vector<double> * out_geom_ceiling = nullptr,
    const std::vector<double> * in_geom_ceiling = nullptr,
    int * steer_reject_idx_out = nullptr) const;

  // local_trajectory의 modulo splice는 한 바퀴 이상 경로를 표현할 수 없다.
  bool PlanSpanAllowed(double span) const;
  // 계획 경로와 벽 사이에 남겨 두는 추적오차 예비 [m].
  //
  // 이 값은 오래 고정 0.10 이었다. run_0815_191056 실측(4571 프레임)에서
  // **계획 대비 벽 방향** 횡오차 p90 은
  //   v<2  0.05 / 2~3  0.09 / 3~4  0.08 / 4~5  0.21 / 5~9  0.22 [m]
  // 로, 4 m/s 를 넘으면 예비가 실제 오차의 절반이었다. 같은 로그에서
  // 발행 경로의 벽예비는 s=31~37 (5~6.7 m/s 구간)에서 정확히 0.100 에
  // 붙어 있었다 — 지점별 클램프가 경로를 이 값까지 밀어붙이므로, 이
  // 파라미터가 곧 고속 구간의 실제 여유였다.
  // 그래서 v0 위로 선형 증가시킨다. v0 이하에서는 예전과 완전히 같다
  // (좁은 저속 코너의 회피 성립률을 건드리지 않기 위한 것 — 저속에서는
  //  실측 오차도 작다).
  double TrackingReserve(double speed) const;

  // dist_ahead 앞 지점에 도달했을 때의 속도 [m/s]. 감속 크레딧
  // (boundary_arrival_decel_mps2_) 이 0 이면 PlanSpeed() 와 같다.
  double ArrivalSpeed(double dist_ahead) const;
  // 벽 여유를 **그 지점을 지날 때** 속도로 재는 형태. 근거는 .cpp 주석.
  // dist_ahead = 0 이면 RequiredBoundaryMargin() 과 완전히 같다.
  double RequiredBoundaryMarginAt(
    double requested_margin, double dist_ahead) const;
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
  // "레이스라인에서 유의미하게 벗어났는가" 판정 문턱 [m] (DoSpline 경계검사).
  // spline_resolution_(종방향 샘플 간격)과 우연히 같은 0.1 이었으나 차원이
  // 다르다 — 한쪽을 바꾸면 다른 쪽이 조용히 따라가는 것을 끊는다.
  static constexpr double kOffRacelineEpsM{0.1};
  // d(s) 스플라인 샘플 간격 [m]
  double spline_resolution_{0.1};
  double side_switch_d_tresh_{0.25};
  double obs_timeout_sec_{0.5};
  double trailing_gap_m_{2.5};
  double trailing_decel_{2.5};
  // ── 접촉 임박 전용 제동 감속도 (2026-08-22) ────────────────────────────
  // trailing_decel_ 은 "상대를 따라가는" 곡선이라 올리면 추종이 늦게
  // 제동한다. 반면 램프가 장애물과 충돌하는 것이 확정된 분기는 정반대로
  // **차의 실제 제동 능력**을 써야 한다. 두 값을 분리한다.
  //   run_0822_172617 실측 제동 능력: 0.3s 창 최대 4.19 m/s² (브레이크 7A),
  //   상시 3.3~3.9 달성. 3.0 은 그 아래로 잡은 값이다.
  // 이 값이 낮으면 5001줄 주석이 말한 그대로 — 제동 곡선이 정지 곡선이
  // 아니라 reachable_v 에 눌린 "가속 금지" 캡으로만 동작한다.
  double trailing_brake_decel_{3.0};
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
  // 추적오차 예비의 속도 의존분 (TrackingReserve). ref 이하에서는 기여 0 —
  // gain=0 으로 두면 예전(고정값) 동작과 완전히 같다.
  double boundary_tracking_speed_ref_{3.5};   // [m/s] 이 속도까지는 고정값
  double boundary_tracking_speed_gain_{0.0};  // [s] 초과 속도당 추가 예비
  double boundary_tracking_margin_max_{0.05}; // [m] 예비 상한
  // 벽 여유를 "그 지점에 도달했을 때" 속도로 재기 위한 감속 크레딧 [m/s^2].
  // 0 = 예전(현재 속도) 동작과 비트 단위로 동일. 상한은 plant_decel_mps2_ —
  // 그 위는 브레이크 성능을 실제보다 높게 주장하는 것이라 예비가 얇아진다.
  double boundary_arrival_decel_mps2_{0.0};
  // 트레일링 횡 램프를 장애물과도 대조할 것인가. false = 예전 동작(벽만).
  // 개입 문턱은 **차체 접촉**이고 되감기 목표는 계획 마진이다 — 근거는
  // BuildTrailing 의 ramp_obstacle_slack 주석 (301/664 사이클).
  bool trailing_ramp_obstacle_check_{false};
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
  // 동적 상대 추종 캡이 걸려 있는가 (pitwall 상승엣지용).
  bool follow_active_{false};
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
  // 원호 곡률 예산 계산의 속도 하한 [m/s] (kappa_grip = a_lat / v²).
  // ⚠️ 2026-08-17 부터 이 값은 **거부 게이트가 아니다** — run_0817_145548
  //    에서 v=0.48 로 기던 차의 유일한 성립 회피(원호)가 이 게이트로 죽어
  //    13.8초 정지가 됐다. "저속이면 정상 회피가 성립한다"는 전제는 조향
  //    곡률 한계(속도 무관)를 놓친 것이었다. BuildEscapeArc 주석 참조.
  double escape_arc_min_speed_{1.2};
  // 기동 끝에 남는 헤딩 상한 [rad]. 이보다 크면 복귀가 벽으로 간다.
  double escape_arc_max_heading_{0.70};
  bool escape_arc_active_{false};       // 상승엣지 이벤트 + 컨트롤러 게이팅용
  // ── 정지 워치독 (2026-08-17, run_0817_150059/_145548) ───────────────────
  // 정지가 이만큼 이어지면 escape 의 is_visible 요구를 해제한다. 근거:
  // 150059 에서 기억 주입(is_visible=false) 유령 앞 정지가 14초 이어졌다 —
  // 정지 T초를 버틴 정적 장애물이 유령이면 브릿지 스캔 카빙이 이미 지웠어야
  // 하므로, 남아 있는 것은 실물로 취급해도 안전하다. is_static 요구와
  // 기하/충돌 검증은 그대로다 — 풀리는 것은 가시성 하나다.
  double standstill_watchdog_sec_{2.0};
  double standstill_speed_eps_{0.20};     // 이 미만이면 "정지"로 센다 [m/s]
  rclcpp::Time standstill_since_;
  bool standstill_since_valid_{false};
  bool standstill_override_{false};
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
  // 벽 여유(RequiredBoundaryMargin)의 추적오차 항은 2026-08-16 부터 속도
  // 의존이다 — TrackingReserve() 주석에 실측 근거가 있다.
  double trailing_engage_dist_m_{8.0};    // trailing 감속을 시작하는 정지점 앞 거리 [m]
  // ── 동적 상대 추종 (2026-08-22) ─────────────────────────────────────────
  // 유지 목표 간격 [m]. obs.s_start - cur_s_ 기준이다 — 실차 접촉이 이 값
  // 0.44 에서 관측됐으므로(run_0819_211107 89.51s) 1.5 는 실제 차체 여유
  // 약 1.06 m 에 해당한다.
  // ⚠️ 이 간격은 상대의 **급정거**를 완전히 막지 못한다. 반응지연 0.4 s
  //    (인지 65 ms + 계획 33 ms + trailing_lag 0.3) 와 제동 2.38 기준
  //    무접촉 필요 간격은 3 m/s 에서 1.59 m, 5 m/s 에서 3.09 m 다.
  //    1.5 m 는 2.9 m/s 이하에서만 완전 방어이고 그 위에서는 접촉 속도를
  //    낮추는 역할이다 — 직선 추월 창(7.9 m)을 죽이지 않기 위한 의도된
  //    트레이드다. 근본 방어는 위 안전항 + 정적 파이프라인 핸드오프다.
  double dyn_follow_gap_m_{1.0};
  // 안전항이 절대 침범하지 않는 간격 [m]. 실측 접촉 거리 0.44 위에 둔다.
  double dyn_follow_gap_min_m_{0.6};
  // 비례 추종 이득 [1/s]. 간격 오차 1 m 당 속도 명령 차이.
  double dyn_follow_kp_{1.0};
  // 속도 명령 -> 실제 감속 사이의 구동계 지연 [s]. 정지 프로파일이
  // v²/(2a)만 보면 이 지연 동안 굴러간 거리만큼 정지점을 넘는다
  // (rosbag_0806 실측: trailing_gap 1.5m 대비 0.24~0.68m에 정지).
  double trailing_lag_sec_{0.3};
  // 정지 프로파일 램프의 최대 감속률 [m/s^2]. 계단 대신 현재 속도에서
  // 이 비율로 내려오는 곡선을 하한으로 깐다 (params.yaml에 근거).
  double trailing_ramp_decel_{2.0};
  // 기하 하한 초과가 **아닌** 사유로 회피가 실패했을 때의 목표 속도 [m/s].
  double trailing_retry_speed_{1.5};
  // ── 크리프 바닥: 완전 정지를 두지 않는다 (2026-08-22) ──────────────────
  // 정지 권한이 열려도 목표를 0 이 아니라 이 값으로 둔다. 근거는 운영
  // 요구다 — 접촉하더라도 스택이 죽지 않고, 장애물만 치우면 스스로 다시
  // 진행해야 한다. 0 을 명령하면 (a) 래치가 evade_committed 를 닫아
  // 컨트롤러의 발진 하한(map/speed_lookahead_min_m)까지 닫히는 v=0
  // 자기유지 고정점에 들어가고, (b) 전 점 vx=0 은 MAP 컨트롤러의 조향을
  // 죽인다(0807 코스아웃). 낮은 크리프는 두 함정을 모두 피하면서
  // 접촉 속도를 제동 능력으로 눌러 준다.
  double trailing_creep_speed_{0.5};
  // 횡 목표가 축소/포기된 트레일링(= 실제로 비켜 가지 못하는 상태)의 속도
  // 바닥. trailing_retry_speed_ 는 "정적 장애물과 접촉할 때의 속도"라
  // 올리면 BlendDist(v에 정비례)가 커져 횡 램프가 안 들어가고, 포기 빈도가
  // 같이 오른다(실측 v<1.5 0.2% -> v1.5~2.5 10% -> v>3.0 31%). 그래서
  // 계획이 온전할 때만 높은 값을 쓰고 깎였으면 이 값으로 내린다.
  // 불변식: 0 <= degraded <= trailing_retry_speed_.
  double trailing_retry_speed_degraded_{1.5};
  // 트레일링 목표 속도에 관측 창 평균을 쓸 것인가 (params.yaml 주석 참조).
  bool trailing_vs_filter_{true};
  // ApexFor의 목표 간격을 검증기 요구치보다 이만큼 더 준다 [m].
  // 동률(목표 == 충돌박스 경계)이면 수 mm 오차로 매 사이클 자기 거부한다.
  double apex_clearance_eps_m_{0.05};

  // 정적/동적 분류 파라미터 (unicorn opponent_tracker_params.yaml 기본값)
  int cls_min_nb_meas_{5};       // 분류 시작 최소 관측 수
  double cls_min_std_{0.16};     // std_s, std_d 둘 다 미만이면 정적 표 [m]
  double cls_max_std_{0.2};      // 하나라도 초과하면 정적 표 리셋 [m]
  double cls_vs_reset_{0.1};     // 최근 |vs| 평균이 미만이면 정지 재분류 [m/s]
  // "멈춘 상대" 재분류에 요구하는 **연속 사이클 수**.
  //
  // 문턱만 올리는 방식은 폐기했다 — run_0818 두 백 실측에서, 실제로 움직이는
  // 트랙도 10프레임 이동평균이 문턱 아래로 잠깐씩 내려간다:
  //     문턱 0.1 -> 이동 트랙 오분류  4% / 9%
  //     문턱 0.2 ->                  16% / 18%
  //     문턱 0.3 ->                  26% / 36%
  // 그런데 그 하강은 **일시적**이다 (구간 길이 p50 3~4 프레임, 최대 14~23).
  // 진짜 멈춘 차는 계속 아래에 머무르므로 지속 조건이 둘을 깨끗이 가른다:
  //     문턱 0.2 + 지속 10 -> 오분류 3% / 0%
  //     문턱 0.2 + 지속 20 -> 오분류 0% / 0%
  int cls_vs_hold_{10};
  // ── 재등록 시 분류 이력 승계 (#15) ────────────────────────────────────
  // 조우 도중 같은 물체에 새 id 가 붙으면 관측 이력이 0 이 되어 분류가
  // 정적으로 되돌아가고 커밋이 파기된다. 방금 사라진 **동적** 트랙의
  // 외삽 위치와 이 거리 안이면 이력을 그대로 물려받는다.
  // 0 이면 기능 off. 정적 트랙은 대상이 아니다(콘끼리 이력이 새는 것 방지).
  double cls_inherit_dist_m_{0.6};
  double cls_inherit_timeout_sec_{0.5};
  // WasEverMoved 문턱 [m]. 0 이면 기능이 꺼진다(= 정지 권한이 is_static 하나로
  // 결정되던 종전 동작). 실측 근거는 ObsClsTrack::max_disp 주석.
  double ever_moved_min_disp_m_{0.5};
  // ── 회랑 봉쇄 정지 권한 (2026-08-22, run_0822_000310) ──────────────────
  // MergeBlocksTrack 의 "합치면 통과 가능한 d 가 하나도 없다" 판정은 지금까지
  // **병합 취소 사유로만** 쓰이고 버려졌다. run_0822_000310 t=112.3~113.9 에서
  // 그 판정이 99회(사실상 매 사이클) 떴는데도 자차는 BuildTrailing 의 바닥
  // (trailing_retry_speed 2.5 -> degraded 1.5)에 걸려 잔해 쪽으로 계속 굴러갔다.
  //
  // 이 래치는 그 판정을 **정지 권한** 하나로만 승격한다. 비상정지가 아니고
  // 횡 목표(d_goal)도 건드리지 않는다 — allow_stop 은 원래 속도 결정이다
  // (BuildTrailing 의 d_goal 주석 참조). 그래서 오검출의 대가가 "필요 이상
  // 감속"이지 "이유 없는 급정지"가 아니다. [[planning-full-block-estop-0821]]
  // 이 계산한 랩당 0.83회 오정지는 **정지 트리거**로 쓸 때의 값이다.
  bool corridor_block_stop_enabled_{true};
  // 근거 구간이 시작될 때 이 속도 이상이어야 래치가 무장한다. 트레일링으로
  // 이미 기어가는 중이면 바닥을 풀어 줄 이유가 없다 — run_0822_000310 의
  // 오발화 버스트 2건(t≈36-38, 88-90)이 전부 헤어핀에서 상대를 <=2.5 m/s 로
  // 정상 추종하던 구간이었고, 사고 버스트만 3.4 m/s 였다.
  double corridor_block_stop_min_speed_{3.0};
  // 단발 유령 제거용 디바운스. 6프레임(30Hz 기준 0.2s)보다 짧게 잡되
  // 1프레임은 반드시 거른다.
  double corridor_block_stop_hold_sec_{0.15};
  // 근거가 이만큼 끊기면 래치를 푼다. 속도 게이트와 달리 **해제에는 속도
  // 조건이 없다** — 래치가 걸려 차가 느려지면 속도 게이트가 스스로 래치를
  // 풀어 바닥이 되살아나는 자기모순이 생긴다.
  double corridor_block_stop_release_sec_{0.5};
  // 이번 사이클에 "구성원 전원이 한 번도 안 움직인" 봉쇄 병합이 있었나.
  // ClusterObstacles 가 세우고 UpdateCorridorBlockLatch 가 소비한다.
  bool corridor_block_evidence_{false};
  bool corridor_block_run_valid_{false};   // 연속 근거 구간이 진행 중인가
  bool corridor_block_run_armed_{false};   // 그 구간 시작 시 속도 게이트 통과
  rclcpp::Time corridor_block_run_since_{0, 0, RCL_ROS_TIME};
  rclcpp::Time corridor_block_last_evidence_{0, 0, RCL_ROS_TIME};
  bool corridor_block_latched_{false};
  static constexpr size_t kClsWindow{20};     // 위치 std 계산 관측 창
  static constexpr size_t kClsVsWindow{10};   // vs 평균 관측 창 (unicorn vs_list 10)
  struct ObsClsTrack {
    std::deque<double> s_hist;   // s_ref 기준 unwrap된 s 관측
    std::deque<double> d_hist;
    std::deque<double> vs_hist;
    double s_ref{0.0};
    int static_count{0};
    int total_count{0};
    // vs 평균이 cls_vs_reset 아래로 **연속** 유지된 사이클 수 (아래 cls_vs_hold).
    int vs_low_run{0};
    bool has_flag{false};        // 분류 확정 여부 (미확정이면 perception 플래그 유지)
    bool static_flag{false};
    // 이 트랙이 한 번이라도 동적으로 분류된 적이 있나.
    // perception 의 Track::ever_dynamic 과 같은 개념인데, 그 필드는 메시지에
    // 실려 오지 않으므로 spliner 가 자기 분류 이력으로 따로 기억한다.
    // 쓰임: 분류가 뒤집혀도 흔들리지 않는 단조 키가 필요한 곳.
    bool ever_dynamic{false};
    // 관측된 **최대 변위** [m] (s_ref 기준 절댓값이라 왕복해도 줄지 않는다).
    // ever_dynamic 과 목적이 다르다: 저쪽은 분류기의 판단이고 이쪽은 원시
    // 관측이다. 짧게 흔들린 콘이 DYNAMIC 으로 각인되는 일이 실제로 있었으므로
    // (실차 콘 max_disp 최대 0.244 m / 시뮬 0.422 m, 문턱 0.5), 정지 권한처럼
    // 안전에 직결되는 판단은 분류 결과가 아니라 이 값으로 건다.
    double max_disp{0.0};
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
  // 커밋을 **만들 수 있는** 최소 관측 수. 생성자 검증이
  // commit_min_obs_ >= commit_break_min_obs_ 를 강제한다 — 그래야
  // "만들 수는 있는데 깰 수는 없는" 트랙이 존재하지 않는다.
  // 회피 경로 생성 자체는 이 게이트 앞에 있으므로 회피 반응은 늦지 않는다.
  int commit_min_obs_{7};
  // 이 속도 미만이면 커밋을 만들지 않는다. 정지 중 생성된 커밋은 머리
  // 속도가 vx=0 으로 굳고(회피상실 홀드가 전 구간 0 m/s 캡을 건다),
  // pos < span 이 영원히 참이라 path_end 로도 만료되지 않는다.
  double commit_min_speed_{0.3};
  // 전방에 아무 장애물도 없는 상태가 이만큼 이어지면 커밋을 해제한다.
  // 0 이면 비활성(종전 동작). 판정은 관측수/가시성 게이트를 거치지 않은
  // 원시 목록으로 한다 — stable_ahead 로 하면 실물이 잠깐 가려진 것만으로
  // 커밋이 풀려서 0811 에서 막은 coasting fail-open 이 되살아난다.
  double commit_release_empty_sec_{1.0};
  // 전방 목록이 마지막으로 비어 있지 않았던 시각 (위 판정용).
  rclcpp::Time last_obs_ahead_time_{0, 0, RCL_ROS_TIME};
  /// 꼬리 충돌로 커밋을 잘라 낼 때, 차단점 앞에 남겨 둘 재계획 여유 [m].
  /// 커밋은 여기서 path_end 로 만료되고 그 자리에서 티어가 다시 돈다 —
  /// 이 값이 "다음 장애물을 몇 미터 앞에서 처음 계획하는가"를 정한다.
  /// 실효값은 max(이 값, BlendDist(차단 장애물 진폭, BlendPlanSpeed) +
  /// 차 반길이) 라 속도·진폭이 커지면 자동으로 늘어난다.
  /// 0 이면 꼬리 절단 비활성(종전 동작).
  double commit_tail_release_lead_m_{4.0};

  // 추월 섹터 파라미터 (진단·시각화 전용 — ComputeOtSectors 주석 참조)
  bool sector_gating_{true};
  double ot_min_clearance_{0.95};
  double ot_max_kappa_{0.1};         // 섹터 판정 최대 곡률 [rad/m] (0.1 = 반경 10m)
  double ot_sector_min_len_m_{8.0};
  // 섹터 안 회피 속도 배율. **1.0 초과 = 부스트**이며 상한은 gb_vmax_ 다.
  // 2026-08-10 이전에는 생성자 검증이 1.0 초과를 거부해서, 헤더/파라미터
  // 주석이 약속한 "섹터 내 공격 모드"가 원리적으로 성립할 수 없었다
  // (배포 yaml 3개 모두 1.0 = 항등). 이제 허용하되 기본값은 1.0 유지.
  std::vector<double> ot_sectors_param_;                 // 수동 지정 (flat s0,s1,...)
  std::vector<std::pair<double, double>> ot_sectors_;    // [s_start, s_end] (wrap 허용)
  // "직전 사이클에 회피를 시도했는가"의 상승엣지 래치.
  // 이름이 in_ot_sector_ 였지만 정적 장애물은 섹터 밖에서도 참이라 섹터
  // 소속과 무관하다 — 실제 의미로 개명했다.
  bool evade_attempt_active_{false};
  // 진행 중인 정적 회피 커밋 (경로 끝까지 유지 — 도중 소멸 시 벽 충돌)
  f1tenth_planning_msgs::msg::WpntArray committed_wpnts_;
  // 커밋 경로의 기하 전용 속도 상한. 재생 때 현재 cur_vs_ 로 전방 가속
  // envelope 만 다시 풀기 위해 보관한다 (비어 있으면 종전대로 동결값 사용).
  std::vector<double> committed_geom_ceiling_;
  // ApplySpeedProfile 이 마지막으로 성공했을 때의 기하 상한. 커밋 시점에
  // 크기가 맞으면 committed_geom_ceiling_ 로 옮긴다.
  mutable std::vector<double> last_geom_ceiling_;
  /// 커밋 생성 시점의 그립 예산(active_lat_budget_). 커밋 재생은 기하 상한만
  /// 재사용하고 **전방 가속 envelope 는 현재 예산으로 다시 푼다**. 그 사이
  /// 정적/동적 분류가 뒤집혀 예산이 바뀌면 같은 기하에서 속도가 점프하므로
  /// 커밋이 사는 동안 얼려 둔다. 음수 = 미설정(종전 동작).
  double committed_lat_budget_{-1.0};
  visualization_msgs::msg::MarkerArray committed_mrks_;
  double committed_start_s_{-1.0};
  double committed_until_s_{-1.0};
  /// 커밋을 만들게 한 장애물의 뒷 가장자리 s. 커밋 조기 해제(새 장애물이
  /// 커밋 경로 밖에 나타났을 때)가 "아직 그 장애물 옆을 지나는 중"인지
  /// 구별하는 데 쓴다 — 기동 중에 커밋을 풀면 raceline 전속 경로로
  /// 순간 전환돼 반대편 벽을 친다(커밋 도입 근거와 같은 실측).
  double committed_obs_end_s_{-1.0};
  /// 커밋 파기 확정에 필요한 연속 충돌 사이클 수. 1 이면 종전 동작
  /// (한 번 걸리면 즉시 파기). 차체 접촉(contact_slack <= 0)은 이 값과
  /// 무관하게 항상 즉시 파기하므로, 이 지연이 적용되는 것은 "분산 여유만
  /// 갉아먹은" 얕은 충돌뿐이다.
  int commit_break_confirm_cycles_{2};
  /// 커밋 경로에서 자차가 이만큼 벗어나면 파기한다 [m]. 0 이면 기능 off
  /// (= 종전 동작: 경로/장애물만 보고 자차 이탈은 안 본다).
  /// 0.40 = run_0822_000933 실측으로 정상 회피(p99 0.188 / max 0.532)와
  /// 사고 구간(p50 0.331 / max 1.006)을 가르는 값. 정상 구간 발화 1 사이클.
  double commit_break_lat_dev_m_{0.40};
  /// 위 이탈이 이만큼 **연속**돼야 파기한다. 3 = 30 Hz 기준 0.1 s.
  /// 정상 구간의 단발 1 사이클을 걸러내는 것이 목적이다.
  int commit_break_lat_dev_cycles_{3};
  /// 현재 커밋에서 횡이탈이 연속으로 관측된 사이클 수.
  int commit_lat_dev_run_{0};
  /// 현재 커밋에서 충돌이 연속으로 관측된 사이클 수 (위 판정용).
  int commit_collision_run_{0};
  /// 이 커밋이 꼬리 충돌로 한 번이라도 잘렸는가. 만료 사유를 구분하는 데만
  /// 쓴다 (path_end vs 꼬리 절단에 의한 계획적 만료).
  bool committed_tail_truncated_{false};
  // 이번 사이클에 실제로 쓸 그립 예산 [m/s^2]. ApplySpeedProfile(곡률 캡·
  // 마찰원 종그립·reachable 성립성)과 BlendLatAcc 가 path_lat_acc_max_ 대신
  // 이 값을 읽는다. Run() 이 매 사이클 path_lat_acc_max_ 로 되돌린다.
  // (2026-08-22: 동적 전용 상향 분기는 제거됐다 — 회피가 정적 전용이 되어
  //  나눌 대상이 없다. 예산 자체의 상향은 9단계에서 한 값으로 한다.)
  double active_lat_budget_{7.0};

  double vehicle_wheelbase_m_{0.33};
  double vehicle_max_steering_rad_{0.3220};
  double vehicle_max_steering_rate_radps_{2.0};
  // 계획 게이트가 쓰는 "실제로 달성되는 곡률 / kinematic 자전거모델 예측"
  // 비율. 1.0이면 tan(delta_max)/L 그대로 (예전 동작).
  // vehicle_max_steering_rad_ 는 steer_calib 로 측정·검증된 서보 기구 한계라
  // 건드리면 안 되므로(TUNING_STATUS §4, CLAUDE.md), 타이어 그립 향상처럼
  // "같은 조향각으로 더 돈다"를 반영할 때는 이 배율만 올린다.
  double steer_kappa_scale_{1.0};
  // 속도별 조향 능력 표 (steer_grip_model.hpp). 비어 있으면 종전 동작.
  planning_pkg::SteerGripModel steer_grip_;
  // 회피 사이드를 갈아타려면 반대쪽이 이만큼 더 나아야 한다 [m].
  // 0 이면 히스테리시스 없음 = 종전 동작.
  double side_switch_hysteresis_m_{0.0};
  // 트레일링에서 **접촉을 피할 횡 후보가 하나도 없을 때**의 속도 바닥 [m/s].
  // 음수면 비활성 = 종전 동작(trailing_retry_speed_degraded_ 까지만 내려감).
  // ⚠️ 0 으로 두지 말 것 — 정적 장애물 앞 완전 정지는 스스로 못 푸는 라치다
  //    (d809831 "정적 장애물의 정지를 폐지한다"). 목적은 정지가 아니라
  //    접촉 에너지(v^2) 를 줄이는 것이다: 2.5 -> 0.8 m/s 면 1/10 이 된다.
  double trailing_contact_floor_mps_{-1.0};
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
  // (escape_steer_kappa_reject_ratio_ / escape_relax_for_observation_ 제거)
  // 두 노브는 "원호 끝 -> hold 급전환이 잔여 헤딩을 한 점에서 꺾어 접합부
  // 곡률이 1.14~1.77 로 튄다"를 우회하려고 넣은 관측용 완화였다. 그 꺾임은
  // BuildEscapeArc 의 기울기 완화 구간(원호 0.4D + ease 0.6D, 두 구간 피크가
  // 모두 5A/D²)이 들어가면서 없어졌고, 배포 yaml 4개 어디에도 스위치가 없어
  // 완화가 켜진 적이 한 번도 없다 — 즉 reject_ratio 쪽은 읽힌 적조차 없다.

  // State
  f1tenth_planning_msgs::msg::ObstacleArray obstacles_;
  rclcpp::Time obstacles_stamp_{0, 0, RCL_ROS_TIME};
  std::vector<f1tenth_planning_msgs::msg::Wpnt> gb_wpnts_;
  // 파생 캐시(ot_sectors_)가 어느 레이아웃으로 만들어졌는지. gb_wpnts_ 가
  // 다른 크기로 재발행되면 섹터가 옛 s 좌표를 가리킨 채 남으므로, 레이아웃이
  // 바뀌면 파생물을 무효화한다. (gb 인덱스로 직접 첨자되던 SP 캐시는
  // 2026-08-22 에 제거됐다 — 그쪽은 out-of-bounds 위험까지 있었다.)
  size_t gb_layout_size_{0};
  double gb_layout_len_{0.0};
  double gb_vmax_{0.0};
  // 트랙 한 바퀴 길이 = 마지막 wpnt s + 점 간격. 마지막 s가 아니라 반드시
  // '+ 간격'이어야 한다 — local_trajectory_node / perception 브릿지가 쓰는
  // 모듈러스와 같은 값이라야 wrap 근처에서 s가 어긋나지 않는다.
  double track_len_{0.0};
  double wpnt_dist_{0.1};
  double cur_s_{0.0};
  double cur_d_{0.0};
  double cur_vs_{0.0};
  // frenet ḋ [m/s]. cur_vs_ 는 s-투영이라 회피 중(횡이동 중)에는 실제
  // 속도보다 작다 — run_0822_172617 실측 ṡ/|v| 중앙 0.917, p10 0.756.
  // 속도 앵커가 필요한 곳은 CurSpeed()( = hypot(ṡ, ḋ) )를 쓸 것.
  double cur_vd_{0.0};
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
  // ── 회피 **의도** 사이드 잠금 (2026-08-23) ────────────────────────────
  // last_ot_side_ 는 BuildLaneSwitch 가 **성공했을 때만** 찍힌다. 그런데
  // run_0823_043254 에서 lane_switch 성공은 574 사이클 중 14 회뿐이었고
  // (044748 은 1766 중 43), 나머지 전부가 trailing 이었다 — 즉 SideKeepActive
  // 의 히스테리시스가 정작 필요한 구간에서 **한 번도 켜지지 않았다**.
  // 그래서 "고른 쪽"을 성공 여부와 무관하게 여기 남기고, 반대쪽이
  // side_switch_hysteresis_m 이상 더 나을 때만 갈아탄다.
  //   왜 문턱이 필요한가: 같은 백에서 좌/우 개폐를 가르는 여유 마진은
  //   중앙 4.3 cm (043254) / 7.9 cm (044748) 인데, 같은 정적 장애물 하나의
  //   관측 중심이 접근하는 동안 표준편차 3.4~5.6 cm, 범위 14~20 cm 로
  //   흔들린다. 즉 매 프레임 다시 고르면 판정이 노이즈에 지배된다 —
  //   실제로 조우 1 건당 개방 사이드가 평균 1.2 회 뒤집혔고, 그 뒤집힘이
  //   90.7 s / 124.5 s / 133.0 s / 152.0 s 충돌 직전에 전부 있었다.
  std::string intent_side_{""};
  double intent_obs_s_{-1.0};
  rclcpp::Time intent_stamp_{0, 0, RCL_ROS_TIME};
  bool has_intent_side_{false};
  // 연쇄 회피 그룹 상한. 콘이 줄지어 있어도 hold 가 무한정 늘어나지 않게
  // 막는다 (구간 상한은 별도로 lookahead_m_ 가 잡는다).
  static constexpr size_t kEvasionGroupMax{4};
  // 거부 사유 카운터 (NoteReject/PublishRejectCounts).
  mutable std::map<std::string, int> reject_counts_;
  // 거부가 아닌 사건 카운터 (NoteOutcome). reject_ 와 섞지 않는다 — 위 주석 참조.
  mutable std::map<std::string, int> outcome_counts_;
  rclcpp::Time reject_publish_stamp_{0, 0, RCL_ROS_TIME};
  // 이번 사이클에 쌓인 거부 사유(발생 순서, 쉼표 구분). TraceCycle이 소비하고
  // Run() 시작에서 비운다. NoteReject가 const 경로에서도 불리므로 mutable.
  mutable std::string cycle_rejects_;
  // ApplySpeedProfile 이 곡률 한계로 경로를 버렸을 때 **어느 점**이 걸렸는지.
  // 거부 로그는 호출자(TryLaneSwitch/DoSpline/...)에 있는데 사유가 "기하곡률>
  // 조향한계 또는 감속 불가"로 뭉뚱그려져 있어, 머리 곡률 수정이 실제로
  // 먹었는지 백에서 확인할 방법이 없었다. const 경로라 mutable.
  mutable int speed_reject_idx_{-1};
  mutable int speed_reject_n_{0};
  mutable double speed_reject_kappa_{0.0};
  mutable double speed_reject_limit_{0.0};
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
  static constexpr double kRetryDHoldSec{0.7};  // 목표 유효 시간 (정적/준정적)
  std::mutex mutex_obstacles_;
  std::mutex mutex_gb_wpnts_;

  rclcpp::Subscription<f1tenth_planning_msgs::msg::ObstacleArray>::SharedPtr s_obstacles_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr s_frenet_odom_;
  rclcpp::Subscription<f1tenth_planning_msgs::msg::WpntArray>::SharedPtr s_global_waypoints_;
  rclcpp::Publisher<f1tenth_planning_msgs::msg::WpntArray>::SharedPtr p_evasion_wpnts_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr p_markers_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr p_sector_markers_;
  // 원호 이탈 중임을 컨트롤러에 알린다. 이 기동은 설계상 조향이 포화 근처로
  // 가므로, control 의 추종 백스톱(조향포화 -> 감속)이 그대로 걸리면 회피와
  // 싸운다. 컨트롤러는 이 신호가 true 인 동안 포화 분기만 건너뛴다.
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr p_escape_active_;
  /// 회랑 봉쇄 래치 상태. 소비처는 없고 관측용이다 — 백에서 발화율(랩당)을
  /// 재려면 이 토픽 하나만 보면 된다. corridor_block_stop_enabled_ 가
  /// false 여도 근거 판정은 그대로 돌아 계측이 끊기지 않는다.
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr p_corridor_blocked_;
  /// 이번 사이클에 **차를 세워 둘 의도가 없는가**. 컨트롤러의 정지 발진
  /// 하한(map/speed_lookahead_min_m) 게이트다.
  ///   evasion_valid && (!used_trailing || allow_stop 이 false 인 트레일링)
  ///
  /// ApplySpeedProfile 은 경로 머리 속도를 cur_vs_ 에 앵커하므로 정지 중
  /// 회피 경로의 첫 점은 vx=0 이 된다. 컨트롤러의 속도 lookahead 는
  /// 거리가 아니라 v x 0.25 초라 v=0 에서 0 m 가 되고, 결국 그 0 을 읽어
  /// 명령 0 -> v=0 이 스스로를 유지한다 (run_0822_001734 실측: 정지+회피
  /// 1773 사이클 중 87.5 % 가 계획 선두속도 0, 회피가 없으면 0.7 %).
  ///
  /// 처음에는 trailing 을 통째로 제외했다("회피 못 해서 따라가며 감속"
  /// 이므로). 그 판이 실차에서 정확히 반대로 작동했다 — run_0822_143207 /
  /// _144702 의 정지 교착 4 에피소드(19.1 / 49.4 / 72.8 / 50.5 s, 두 백
  /// 375.9 s 의 51 %) 는 **전부** trailing 사이클이라 게이트가 닫혀 있었다:
  ///   명령 0 이 99.7~99.8 %, 계획 선두속도 중앙 0.00,
  ///   evade_committed=true 는 0.5~3.3 % 뿐.
  /// 네 에피소드 모두 사람이 차를 옮겨서 풀었다.
  ///
  /// 갈라야 하는 것은 "trailing 인가"가 아니라 **"세울 의도인가"**다. 그
  /// 판단은 플래너가 이미 allow_stop 으로 내리고 있다:
  ///   allow_stop=false (처음부터 정적인 콘) -> BuildTrailing 이 목표속도에
  ///     trailing_retry_speed_ 바닥을 깐다. 즉 "감속하되 서지는 않는다"가
  ///     플래너의 명시된 의도인데, ApplySpeedProfile 의 0 앵커가 그 의도를
  ///     매 사이클 0 으로 덮어써서 영구 교착이 된다. 위 4 에피소드의
  ///     정지 사이클 중 88.7 % / 99.7 % 가 이 상태였다
  ///     (trailing_retry_clamp_raised 가 그 사이클에서만 오른다).
  ///   allow_stop=true (동적 상대, 달리다 멈춘 상대, 비상정지 합성 장애물)
  ///     -> 진짜 정지 계획이므로 종전대로 게이트를 닫는다.
  ///
  /// 두 백 오프라인 재검증 (정지 사이클 2439 / 1314):
  ///   게이트 열림 4.0 % -> 88.9 %,  2.7 % -> 99.7 %
  ///   명령 0    94.8 % -> 12.0 %,  97.0 % ->  0.5 %
  ///   명령 중앙  0.00 -> 1.25 m/s,  0.00 -> 1.39 m/s
  ///
  /// ⚠️ 남는 노출 — 새로 발진하는 사이클을 **차체 횡중첩**으로 걸러 세면:
  ///   144702 는 1274 사이클 전부가 장애물 옆에 이미 비켜 서 있었다
  ///     (정면 노출 0). 나가는 것이 유일하게 옳은 동작이다.
  ///   143207 은 71.8 % 가 정면이지만 간격 중앙 3.42 m 이고, 발진속도
  ///     1.27 m/s 의 정지거리(0.78 m) 안쪽은 1.3 %, 0.30 m 안쪽은 0.6 %.
  ///   (게이트만 보고 세면 각각 20.1 % / 41.5 % 로 나오는데, 그 대부분은
  ///    옆으로 비켜 선 장애물이다 — 이 수를 쓰지 말 것.)
  /// 이 거래를 받는 근거는 (a) 정적 장애물에 완전 정지를 주지 않는 것이
  /// 이미 확정된 설계이고(0806 정지 라치 방지), (b) 대안이 사람이 차를
  /// 들어 옮기는 것뿐이기 때문이다. 실차에서 콘 접촉이 관측되면 여기가
  /// 아니라 BuildTrailing 의 바닥(trailing_retry_speed_)을 손볼 것 —
  /// 게이트를 다시 닫으면 교착이 그대로 돌아온다.
  ///
  /// 매 사이클 발행한다 — 구독자가 마지막 true 에 붙어 있지 않게
  /// (p_escape_active_ 와 같은 fail-closed 처리).
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr p_evade_committed_;

  /// "이번 사이클 회피/추종 대상이 없고, **마지막 정적 대상은 이미 지나쳤다**".
  /// local_trajectory 의 회피상실 홀드(avoidance_loss_hold) 게이트다.
  ///
  /// 홀드는 "장애물을 잃은 것이 가속 신호가 아니다"를 위한 것인데, 실측
  /// (run_0822_172617)으로 홀드 활성 29.0 s 의 64.7 % 가 마지막 대상 뒤끝을
  /// 이미 중앙 4.63 m 지난 상태였다 — 조우가 기하적으로 끝났는데 1 초 동안
  /// gb 의 78 % 캡을 유지했고, 33 % 사이클에서는 실제 감속을 강제했다.
  /// 반대로 대상이 **앞에 있는 채로** 사라진 경우(인지 플리커, 15.5 %)는
  /// last_static_target_end_s_ 를 아직 지나지 않았으므로 이 신호가 false 로
  /// 남아 홀드가 종전대로 방어한다.
  ///
  /// 매 사이클 발행 (p_escape_active_ 와 같은 fail-closed 계약). 구독자는
  /// 신선도(age)를 함께 확인한다 — 노드가 죽으면 신호가 멎어 홀드가
  /// 종전 동작으로 돌아간다.
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr p_target_passed_;
  /// 마지막으로 회피 티어가 겨눴던(또는 지나침으로 방출한) 정적 장애물의
  /// s_end [wrap]. -1 = 아직 없음. 재시작 없이는 리셋하지 않는다 — 통과
  /// 판정(Mod 반 바퀴)이 자차가 반 트랙 이상 벗어나면 스스로 false 가 된다.
  double last_static_target_end_s_{-1.0};
  rclcpp::TimerBase::SharedPtr t_run_;
};

#endif  // PLANNING_PKG__SPLINER_NODE_HPP_
