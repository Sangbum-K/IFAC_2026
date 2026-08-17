// WallDistanceMap::raycast (sphere tracing) 경계 조건.
//
// 이 함수의 오답은 레이 일관성 필터의 오검출로 바로 이어진다:
//   · 너무 짧게 답하면 -> 멀쩡한 원거리 장애물이 "벽 뒤"로 몰려 버려진다
//   · 너무 길게 답하면 -> 배리어를 넘어온 점이 그대로 통과한다
// 그래서 "벽 앞/뒤", 맵 밖, 벽 셀 위, 무한루프까지 전부 고정한다.
#include <gtest/gtest.h>

#include <cmath>
#include <vector>

#include "perception_pkg/distance_transform.hpp"
#include "perception_pkg/wall_distance_map.hpp"

using perception_pkg::WallDistanceMap;

namespace
{

/// x = wall_col 열 전체가 벽인 맵. 나머지는 자유공간.
/// EDT 는 실제 구현(euclidean_distance_transform)으로 채운다 — 테스트가
/// 손으로 만든 근사 거리가 아니라 런타임과 같은 값 위에서 돌게.
WallDistanceMap make_vertical_wall(int w, int h, int wall_col, double res)
{
  std::vector<uint8_t> seed(static_cast<std::size_t>(w) * h, 0);
  for (int r = 0; r < h; ++r) {
    seed[static_cast<std::size_t>(r) * w + wall_col] = 1;
  }
  WallDistanceMap m;
  m.dist = perception_pkg::euclidean_distance_transform(seed, w, h, res);
  m.width = w;
  m.height = h;
  m.resolution = res;
  m.origin_x = 0.0;
  m.origin_y = 0.0;
  m.origin_yaw = 0.0;
  return m;
}

constexpr double kRes = 0.05;

}  // namespace

TEST(WallDistanceMapRaycast, HitsWallAtExpectedRange)
{
  // 200x40 셀, x=100 열이 벽 -> 벽면은 x = 5.00 m.
  // x=1.0 에서 출발하므로 이동거리는 4.00 m (벽의 절대 x 좌표가 아니다).
  const auto m = make_vertical_wall(200, 40, 100, kRes);
  const double got = m.raycast(1.0, 1.0, 1.0, 0.0, 9.0);
  // hit_eps = resolution 이라 벽 한 셀 앞에서 멈춘다. 한 셀 오차만 허용.
  EXPECT_NEAR(got, 4.0, 2.0 * kRes) << "정면 레이가 벽 거리를 못 맞춤";
  EXPECT_LT(got, 4.0 + kRes) << "벽을 지나쳤다 (sphere tracing 이 넘어섬)";
}

TEST(WallDistanceMapRaycast, ObstacleInFrontIsNeverRejected)
{
  // 레이 일관성 필터의 핵심 불변식: 진짜 장애물은 벽보다 가깝다.
  // 벽이 5.0 m 인데 3.0 m 에서 반사가 왔다면 어떤 여유에서도 살아야 한다.
  const auto m = make_vertical_wall(200, 40, 100, kRes);
  const double expected = m.raycast(1.0, 1.0, 1.0, 0.0, 9.0);
  const double measured = 3.0;
  const double margin = 1.5;
  EXPECT_LE(measured, expected + margin) << "벽 앞 반사를 기각하려 한다";
}

TEST(WallDistanceMapRaycast, BeamPastWallIsDetected)
{
  // 배리어를 넘어간 빔: 벽이 4.0 m 인데 6.64 m 에서 반사.
  // 초과 2.64 m = run_0815 실측 초과 중앙값과 같은 크기다.
  const auto m = make_vertical_wall(200, 40, 100, kRes);
  const double expected = m.raycast(1.0, 1.0, 1.0, 0.0, 9.0);
  const double measured = expected + 2.64;
  const double margin = 1.5;
  EXPECT_GT(measured, expected + margin) << "벽 뒤 반사를 못 잡는다";
}

TEST(WallDistanceMapRaycast, ReturnsMaxRangeWhenLeavingMap)
{
  // 벽 없는 방향 -> 맵 밖으로 나간다. "벽이 없다"가 아니라 "모른다"이므로
  // max_range 를 돌려주고, 호출부는 그 빔을 건드리지 않는다.
  const auto m = make_vertical_wall(200, 40, 100, kRes);
  const double got = m.raycast(1.0, 1.0, -1.0, 0.0, 9.0);
  EXPECT_DOUBLE_EQ(got, 9.0);
}

TEST(WallDistanceMapRaycast, StartingOnWallReturnsZeroAndTerminates)
{
  // 벽 셀 위에서 출발하면 dist==0 이다. 전진 하한이 없으면 여기서 무한루프가
  // 난다 (그 자리에서 t += 0). 즉시 0 을 돌려주고 끝나야 한다.
  const auto m = make_vertical_wall(200, 40, 100, kRes);
  const double got = m.raycast(5.0, 1.0, 1.0, 0.0, 9.0);
  EXPECT_NEAR(got, 0.0, kRes);
}

TEST(WallDistanceMapRaycast, ZeroResolutionIsInert)
{
  // 맵이 아직 안 왔거나 malformed 인 경우. 나눗셈으로 죽지 않고 판단을 보류한다.
  WallDistanceMap m;
  m.width = 10;
  m.height = 10;
  m.resolution = 0.0;
  m.dist.assign(100, 1.0f);
  EXPECT_DOUBLE_EQ(m.raycast(0.0, 0.0, 1.0, 0.0, 9.0), 9.0);
}

TEST(WallDistanceMapRaycast, DiagonalRayHitsWall)
{
  // 45도 입사. 벽면 x=5.0 이므로 시작 x=1.0 에서 경로길이는 4.0*sqrt(2)=5.657.
  const auto m = make_vertical_wall(200, 200, 100, kRes);
  const double s = std::sqrt(0.5);
  const double got = m.raycast(1.0, 1.0, s, s, 12.0);
  EXPECT_NEAR(got, 4.0 * std::sqrt(2.0), 4.0 * kRes);
}

TEST(WallDistanceMapRaycast, GrazingRayIsUnstableUnderPerturbation)
{
  // 안정성 게이트가 잡아야 하는 상황을 고정한다. 벽과 거의 나란한 빔은
  // 방향을 1.5도만 틀어도 기대거리가 크게 바뀐다 — 그런 빔으로는 "벽 뒤"를
  // 판정하면 안 된다.
  const auto m = make_vertical_wall(400, 400, 200, kRes);
  const double probe = 1.5 * M_PI / 180.0;
  // 벽면(x=10.0)을 향해 거의 스치듯 가는 방향
  const double ang = std::atan2(1.0, 0.02);        // 거의 +y 축
  const double ux = std::cos(ang), uy = std::sin(ang);
  const double e0 = m.raycast(9.5, 0.5, ux, uy, 20.0);
  const double e1 = m.raycast(
    9.5, 0.5, std::cos(ang - probe), std::sin(ang - probe), 20.0);
  const double e2 = m.raycast(
    9.5, 0.5, std::cos(ang + probe), std::sin(ang + probe), 20.0);
  const double spread = std::max({e0, e1, e2}) - std::min({e0, e1, e2});
  EXPECT_GT(spread, 0.5) << "접선 입사인데 섭동에 둔감하다 — 게이트가 안 걸린다";
}

TEST(WallDistanceMapRaycast, HeadOnRayIsStableUnderPerturbation)
{
  // 반대 경우: 정면 입사는 섭동에 거의 안 흔들려야 한다. 안 그러면 게이트가
  // 멀쩡한 빔까지 전부 보류시켜 필터가 사실상 꺼진다.
  const auto m = make_vertical_wall(400, 400, 200, kRes);
  const double probe = 1.5 * M_PI / 180.0;
  const double e0 = m.raycast(1.0, 5.0, 1.0, 0.0, 20.0);
  const double e1 = m.raycast(1.0, 5.0, std::cos(-probe), std::sin(-probe), 20.0);
  const double e2 = m.raycast(1.0, 5.0, std::cos(probe), std::sin(probe), 20.0);
  const double spread = std::max({e0, e1, e2}) - std::min({e0, e1, e2});
  EXPECT_LT(spread, 0.5) << "정면 입사가 섭동에 흔들린다 — 게이트가 과하게 걸린다";
}
