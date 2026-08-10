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
/// 과병합보다 과분할을 선호해 절반으로 조인 값이다. 3*sigma 노이즈 항까지
/// 같이 줄어들므로 실효 lambda는 설정값보다 다소 작게 동작한다.
inline constexpr double kSafetyFactor = 0.5;

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

}  // namespace perception_pkg

#endif  // PERCEPTION_PKG__CLUSTERING_HPP_
