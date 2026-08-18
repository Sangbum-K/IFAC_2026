#ifndef PERCEPTION_PKG__CLUSTER_FILTERS_HPP_
#define PERCEPTION_PKG__CLUSTER_FILTERS_HPP_

namespace perception_pkg
{

/// Keep points only when the map lookup succeeded and the wall distance meets the threshold.
[[nodiscard]] inline constexpr bool passes_wall_distance_gate(
  bool distance_available, double wall_distance, double min_wall_distance) noexcept
{
  return distance_available && wall_distance >= min_wall_distance;
}

/// Accept clusters at the radius boundary; this preserves the node's `radius > max` rejection.
[[nodiscard]] inline constexpr bool passes_cluster_radius_gate(
  double radius, double max_radius) noexcept
{
  return !(radius > max_radius);
}

}  // namespace perception_pkg

#endif  // PERCEPTION_PKG__CLUSTER_FILTERS_HPP_
