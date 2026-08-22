/*
 * 클러스터링 모듈
 * - LaserScan 포인트는 이미 각도순으로 정렬되어 있으므로, 인접한 두 포인트 사이
 *   거리가 임계값 이하면 같은 클러스터로 묶는 breakpoint(연속성) 기반
 *   클러스터링을 사용한다. 일반 DBSCAN 대비 O(N)으로 훨씬 가볍다.
 * - 임계값은 고정이 아니라 센서 거리에 비례해 커지는 adaptive breakpoint
 *   (Borges & Aldon / Forza ETH 계열). 멀수록 포인트 간격이 벌어지는 걸 보정한다.
 * - **ranges를 points와 따로 받는 이유**: 임계값의 r은 "센서 원점 기준 거리"여야
 *   한다. 노드는 클러스터링 전에 points를 map frame으로 옮기므로, points만 받아서
 *   |point|를 쓰면 그건 map 원점 기준 거리가 되어 임계값이 통째로 틀어진다
 *   (트랙 위 위치에 따라 같은 장애물이 병합되거나 사라진다 — 실측 확인).
 */
#ifndef PERCEPTION_PKG__CLUSTERING_HPP_
#define PERCEPTION_PKG__CLUSTERING_HPP_

#include <vector>

#include "perception_pkg/types.hpp"

namespace perception_pkg
{

/// d_max에 곱하는 보수화 계수. 원식(Borges & Aldon)에는 없고, 이 스택에서
/// 과병합보다 과분할을 선호해 절반으로 조여뒀던 값이다. 3*sigma 노이즈 항까지
/// 같이 줄어들므로 실효 lambda는 설정값보다 다소 작게 동작한다.
///
/// 2026-08-18: 0.5 -> 1.0 (원식 복귀). run_0817_194156 전수 분석에서 과분할이
/// 인지 출력을 망치는 주범으로 확정됐다:
///   · 미관측(is_visible=0) 전환 447건 중 47%가 raw /scan 이 그 물체를 8빔 이상
///     또렷하게 보고 있는 중(R p50 3.3m)에 일어났다 = 센서가 아니라 클러스터링
///   · t=135.99 사례: 빔 11.75~21.25도의 **연속된 한 표면**(이웃 점 간격 최대
///     0.051m)이 r=0.22 본체와 r=0.06 파편으로 갈렸다. 그 거리에서 d_max 가
///     0.053m 라 실제 간격 0.051m 과 사실상 같았고, 프레임마다 붙었다 갈라진다
///   · 갈라진 파편은 새 ID 로 스폰되고(신규 트랙의 19%가 기존 트랙 0.45m 안,
///     그중 82%가 반경<0.10m), 짝을 잃은 쪽은 0.80s 코스팅으로 계속 발행된다
///     -> 발행 인스턴스의 21.5%가 0.45m 안에 짝을 가진 중복이었다
/// 1.0 이면 같은 2.4m 에서 문턱이 0.053 -> 0.107m 가 된다. 과병합 상한은
/// max_cluster_radius_m가 따로 지키므로 벽 덩어리는 기본적으로 통째로 폐기된다.
///
/// ⚠️ 이 값을 되돌리려면 real_car.yaml 의 min_cluster_size 근거 주석("SAFETY_FACTOR
///    0.5 의 과분할이 겹치므로")도 같이 다시 볼 것 — 그 8 은 과분할 전제 위의 값이다.
inline constexpr double kSafetyFactor = 1.0;

struct Cluster
{
  std::vector<Point2> points;
  Point2 centroid{{0.0, 0.0}};
  /// 중심에서 가장 먼 포인트까지 거리 [m]
  double radius{0.0};
};

/// 각도순으로 정렬된 포인트를 adaptive breakpoint 기준으로 클러스터링한다.
///
/// 인접 포인트 i-1, i를 같은 물체로 볼 최대 거리:
///     d_max = (r * sin(dphi) / sin(lambda - dphi) + 3*sigma) * kSafetyFactor
/// r은 두 포인트의 센서 거리 평균이다.
///
/// ranges는 points와 1:1 대응하는 **센서 원점 기준 거리** [m]여야 한다.
/// 길이가 다르면 std::invalid_argument를 던진다.
std::vector<Cluster> adaptive_cluster_points(
  const std::vector<Point2> & points,
  const std::vector<double> & ranges,
  double angle_increment,
  double lambda_angle,
  double sigma,
  int min_cluster_size);


/// 반경 게이트를 넘긴 클러스터를 최대 간격에서 쪼갠다 (실험용 opt-in).
///
/// 근거: run_0818 두 백에서 `max_cluster_radius_m 0.35` 거부가 초당 0.30~0.32회
/// 로 꾸준했는데, F1TENTH 상대차 반경이 0.31 이라 **경계선**이다. 벽 fringe 와
/// 병합되면 0.35 를 넘고, 지금 동작은 그 덩어리를 통째로 지워 안에 있던 실물도
/// 같이 사라진다. 실측 프레임 비율이 33.8% 밖에 안 되는 주요 후보다.
///
/// points 가 각도순이라는 성질을 이용해 인접 점 간 거리가 가장 큰 곳에서
/// 나누고, 각 조각에 대해 재귀적으로 반복한다. 조각이 min_points 미만이면
/// 버린다 — 쪼개다 파편만 남는 것을 막기 위한 것이고, 하나도 남지 않으면
/// 호출자가 원본을 지우는 종전 동작으로 돌아가면 된다.
///
/// max_depth 로 재귀를 제한한다(벽처럼 정말 큰 덩어리에서 무한 분할 방지).
///
/// 프로덕션 노드는 원본이 초대형이라는 정보가 조각에서 사라지는 문제 때문에
/// 기본적으로 이 함수를 호출하지 않는다. `cluster_split_enabled=true`인 오프라인
/// A/B에서만 사용한다.
std::vector<Cluster> split_oversized_cluster(
  const Cluster & cluster, double max_radius, std::size_t min_points,
  int max_depth = 4);

}  // namespace perception_pkg

#endif  // PERCEPTION_PKG__CLUSTERING_HPP_
