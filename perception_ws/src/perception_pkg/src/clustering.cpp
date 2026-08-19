#include "perception_pkg/clustering.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <stdexcept>

namespace perception_pkg
{
namespace
{

Cluster make_cluster(const std::vector<Point2> & points, std::size_t begin, std::size_t end)
{
  Cluster c;
  c.points.assign(points.begin() + static_cast<long>(begin), points.begin() + static_cast<long>(end));
  double sx = 0.0;
  double sy = 0.0;
  for (const auto & p : c.points) {
    sx += p[0];
    sy += p[1];
  }
  // 역수 곱이 아니라 나눗셈이어야 한다 — sum*(1/n)은 sum/n과 마지막 비트가
  // 달라지고, 그 1 ULP가 프레임을 거치며 클러스터 분할/트랙 ID를 갈라놓는다
  // (파이썬 원본 대비 A/B 재생에서 실제로 관측됨).
  const double n = static_cast<double>(c.points.size());
  c.centroid = {{sx / n, sy / n}};
  double r2max = 0.0;
  for (const auto & p : c.points) {
    const double dx = p[0] - c.centroid[0];
    const double dy = p[1] - c.centroid[1];
    r2max = std::max(r2max, dx * dx + dy * dy);
  }
  c.radius = std::sqrt(r2max);
  return c;
}

}  // namespace

std::vector<Cluster> adaptive_cluster_points(
  const std::vector<Point2> & points,
  const std::vector<double> & ranges,
  double angle_increment,
  double lambda_angle,
  double sigma,
  int min_cluster_size)
{
  std::vector<Cluster> clusters;
  const std::size_t n = points.size();
  const std::size_t min_size =
    static_cast<std::size_t>(std::max(1, min_cluster_size));

  if (n < min_size) {
    return clusters;
  }
  if (ranges.size() != n) {
    throw std::invalid_argument("ranges must be the same length as points");
  }

  const double dphi = angle_increment;
  // lambda <= dphi면 sin이 0 이하가 되어 d_max가 음수/발산한다. 양수로 클램프해
  // "전부 병합"으로 떨어뜨린다 — 전부 분할(=장애물 실명)보다 안전한 실패 방향이다.
  // 정상 설정에서는 걸리지 않는다 (노드가 기동 시 lambda > dphi를 검증).
  const double denom = std::max(std::sin(lambda_angle - dphi), 1e-6);
  const double sin_dphi = std::sin(dphi);

  std::size_t seg_begin = 0;
  for (std::size_t i = 1; i < n; ++i) {
    const double r_pair = 0.5 * (ranges[i - 1] + ranges[i]);
    const double d_max = (r_pair * sin_dphi / denom + 3.0 * sigma) * kSafetyFactor;
    const double dx = points[i][0] - points[i - 1][0];
    const double dy = points[i][1] - points[i - 1][1];
    if (std::sqrt(dx * dx + dy * dy) > d_max) {
      if (i - seg_begin >= min_size) {
        clusters.push_back(make_cluster(points, seg_begin, i));
      }
      seg_begin = i;
    }
  }
  if (n - seg_begin >= min_size) {
    clusters.push_back(make_cluster(points, seg_begin, n));
  }
  return clusters;
}


std::vector<Cluster> split_oversized_cluster(
  const Cluster & cluster, double max_radius, std::size_t min_points,
  int max_depth)
{
  std::vector<Cluster> out;
  if (cluster.points.size() < min_points) {
    return out;
  }
  if (max_depth <= 0 || cluster.radius <= max_radius ||
    cluster.points.size() < 2U * min_points)
  {
    // 더 못 쪼개거나 쪼갤 필요가 없다. 게이트를 통과할 때만 채택한다.
    if (cluster.radius <= max_radius) {
      out.push_back(cluster);
    }
    return out;
  }
  // 인접 점 간 거리가 가장 큰 곳에서 나눈다. 양쪽이 모두 min_points 를
  // 채울 수 있는 위치만 후보로 본다.
  std::size_t cut = 0;
  double widest = -1.0;
  for (std::size_t i = min_points; i + min_points <= cluster.points.size(); ++i) {
    const double dx = cluster.points[i][0] - cluster.points[i - 1][0];
    const double dy = cluster.points[i][1] - cluster.points[i - 1][1];
    const double gap = std::sqrt(dx * dx + dy * dy);
    if (gap > widest) {
      widest = gap;
      cut = i;
    }
  }
  if (cut == 0) {
    return out;
  }
  const Cluster a = make_cluster(cluster.points, 0, cut);
  const Cluster b = make_cluster(cluster.points, cut, cluster.points.size());
  for (const auto & part : {a, b}) {
    auto sub = split_oversized_cluster(part, max_radius, min_points, max_depth - 1);
    out.insert(out.end(), sub.begin(), sub.end());
  }
  return out;
}

}  // namespace perception_pkg
