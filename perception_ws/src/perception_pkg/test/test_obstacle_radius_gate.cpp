#include <gtest/gtest.h>

#include <cstddef>
#include <cstdint>
#include <fstream>
#include <limits>
#include <stdexcept>
#include <string>
#include <vector>

#include <nlohmann/json.hpp>

#include "perception_pkg/cluster_filters.hpp"
#include "perception_pkg/clustering.hpp"
#include "perception_pkg/preprocessing.hpp"

namespace perception_pkg
{
namespace
{

using Json = nlohmann::json;

Json load_fixture()
{
  std::ifstream stream(RADIUS_GATE_FIXTURE_PATH);
  if (!stream) {
    throw std::runtime_error(
            std::string("failed to open radius-gate fixture: ") + RADIUS_GATE_FIXTURE_PATH);
  }
  return Json::parse(stream);
}

std::vector<Point2> read_points(const Json & fixture)
{
  std::vector<Point2> points;
  for (const auto & row : fixture.at("source_points_xy_m_map")) {
    points.push_back(Point2{{row.at(0).get<double>(), row.at(1).get<double>()}});
  }
  return points;
}

TEST(ObstacleRadiusGateRegression, Run0809WallFilteredClusterIsAccepted)
{
  const Json fixture = load_fixture();
  std::vector<Point2> points = read_points(fixture);
  std::vector<double> ranges =
    fixture.at("source_laser_ranges_m").get<std::vector<double>>();
  const std::vector<int> beam_indices =
    fixture.at("source_beam_indices").get<std::vector<int>>();
  const std::vector<double> wall_distances =
    fixture.at("wall_distance_m").get<std::vector<double>>();

  const auto source_size = fixture.at("source_size").get<std::size_t>();
  ASSERT_EQ(source_size, 21u);
  ASSERT_EQ(points.size(), source_size);
  ASSERT_EQ(ranges.size(), source_size);
  ASSERT_EQ(beam_indices.size(), source_size);
  ASSERT_EQ(wall_distances.size(), source_size);

  const double wall_dist_min = fixture.at("wall_dist_min_m").get<double>();
  std::vector<uint8_t> keep_mask;
  keep_mask.reserve(wall_distances.size());
  std::vector<int> retained_beam_indices;
  for (std::size_t i = 0; i < wall_distances.size(); ++i) {
    const bool keep = passes_wall_distance_gate(true, wall_distances[i], wall_dist_min);
    keep_mask.push_back(static_cast<uint8_t>(keep));
    if (keep) {
      retained_beam_indices.push_back(beam_indices[i]);
    }
  }

  EXPECT_EQ(
    retained_beam_indices,
    fixture.at("retained_beam_indices").get<std::vector<int>>());
  apply_mask(points, ranges, keep_mask);
  ASSERT_EQ(points.size(), fixture.at("retained_size").get<std::size_t>());
  ASSERT_EQ(points.size(), 19u);
  ASSERT_EQ(ranges.size(), points.size());

  const std::vector<Cluster> clusters = adaptive_cluster_points(
    points, ranges,
    fixture.at("angle_increment_rad").get<double>(),
    fixture.at("lambda_angle_rad").get<double>(),
    fixture.at("sigma_m").get<double>(),
    fixture.at("min_cluster_size").get<int>());
  ASSERT_EQ(clusters.size(), 1u);
  const Cluster & cluster = clusters.front();
  ASSERT_EQ(cluster.points.size(), 19u);

  const auto expected_centroid =
    fixture.at("post_filter_centroid_xy_m").get<std::vector<double>>();
  ASSERT_EQ(expected_centroid.size(), 2u);
  EXPECT_DOUBLE_EQ(cluster.centroid[0], expected_centroid[0]);
  EXPECT_DOUBLE_EQ(cluster.centroid[1], expected_centroid[1]);
  EXPECT_DOUBLE_EQ(cluster.radius, fixture.at("post_filter_radius_m").get<double>());

  const double max_radius = fixture.at("max_cluster_radius_m").get<double>();
  EXPECT_EQ(fixture.at("expected").get<std::string>(), "keep");
  EXPECT_GT(fixture.at("source_radius_m").get<double>(), max_radius);
  EXPECT_FALSE(
    passes_cluster_radius_gate(fixture.at("source_radius_m").get<double>(), max_radius));
  EXPECT_LT(cluster.radius, max_radius);
  EXPECT_TRUE(passes_cluster_radius_gate(cluster.radius, max_radius));
}

TEST(ObstacleRadiusGateRegression, WallDistancePredicatePreservesBoundaryAndInvalidSemantics)
{
  constexpr double threshold = 0.25;
  const double nan = std::numeric_limits<double>::quiet_NaN();
  const double inf = std::numeric_limits<double>::infinity();

  EXPECT_FALSE(passes_wall_distance_gate(true, 0.24, threshold));
  EXPECT_TRUE(passes_wall_distance_gate(true, threshold, threshold));
  EXPECT_TRUE(passes_wall_distance_gate(true, 0.26, threshold));
  EXPECT_FALSE(passes_wall_distance_gate(true, nan, threshold));
  EXPECT_TRUE(passes_wall_distance_gate(true, inf, threshold));
  EXPECT_FALSE(passes_wall_distance_gate(true, -inf, threshold));
  EXPECT_FALSE(passes_wall_distance_gate(false, inf, threshold));
}

TEST(ObstacleRadiusGateRegression, RadiusPredicatePreservesBoundaryAndInvalidSemantics)
{
  constexpr double max_radius = 0.35;
  const double nan = std::numeric_limits<double>::quiet_NaN();
  const double inf = std::numeric_limits<double>::infinity();

  EXPECT_TRUE(passes_cluster_radius_gate(0.34, max_radius));
  EXPECT_TRUE(passes_cluster_radius_gate(max_radius, max_radius));
  EXPECT_FALSE(passes_cluster_radius_gate(0.36, max_radius));
  EXPECT_TRUE(passes_cluster_radius_gate(nan, max_radius));
  EXPECT_FALSE(passes_cluster_radius_gate(inf, max_radius));
  EXPECT_TRUE(passes_cluster_radius_gate(-inf, max_radius));
}

}  // namespace
}  // namespace perception_pkg
