#include <gtest/gtest.h>

#include <cmath>
#include <vector>

#include "perception_pkg/track_filter.hpp"
#include "perception_pkg/tracker.hpp"

namespace perception_pkg
{
namespace
{

Cluster make_cluster(double x, double y, double radius = 0.20)
{
  Cluster cluster;
  cluster.centroid = Point2{{x, y}};
  cluster.radius = radius;
  return cluster;
}

Tracker make_tracker(
  double reacquire_timeout_sec = 0.20,
  double frenet_reid_timeout_sec = 1.25,
  double frenet_progress_slack_m = 0.80)
{
  // primary gate=0.45m, active-dynamic recovery gate=min(0.7, 0.45*1.6)=0.7m
  return Tracker(
    0.45, 1, 0.70, 1.0, 10, 4.0, 1.6, reacquire_timeout_sec,
    frenet_reid_timeout_sec, 0.45, frenet_progress_slack_m, 0.30);
}

TrackBounds make_corner_bounds()
{
  // (0,0)에서 +x로 가다 (1,0) 부근에서 +y로 꺾이는 L자 레이스라인.
  // project()는 가까운 waypoint의 psi를 쓰므로, 코너 뒤 후보는 같은 d=0이고
  // s만 증가한다.
  return TrackBounds({
      Waypoint{0.0, 0.0, 0.0, 1.0, 1.0, 0.0},
      Waypoint{1.0, 0.0, 1.0, 1.0, 1.0, 0.0},
      Waypoint{1.0, 1.0, 2.0, 1.0, 1.0, 1.57079632679489661923},
      Waypoint{1.0, 2.0, 3.0, 1.0, 1.0, 1.57079632679489661923},
      Waypoint{1.0, 3.0, 4.0, 1.0, 1.0, 1.57079632679489661923}});
}

TEST(TrackerReacquisition, KeepsDynamicIdAndResetsStaleVelocity)
{
  auto tracker = make_tracker();
  auto tracks = tracker.step({make_cluster(0.0, 0.0)}, 0.0);
  ASSERT_EQ(tracks.size(), 1u);
  tracks.front()->ever_dynamic = true;
  tracks.front()->is_dynamic = true;

  // y로 0.6m 꺾여 primary gate 밖이지만 마지막 실측점에서는 recovery gate 안이다.
  tracks = tracker.step({make_cluster(0.0, 0.60)}, 0.05);
  ASSERT_EQ(tracks.size(), 1u);
  EXPECT_EQ(tracks.front()->id, 0);
  EXPECT_EQ(tracks.front()->misses, 0);
  EXPECT_NEAR(tracks.front()->kf.x(), 0.0, 1e-9);
  EXPECT_NEAR(tracks.front()->kf.y(), 0.60, 1e-9);
  EXPECT_NEAR(tracks.front()->kf.vx(), 0.0, 1e-9);
  EXPECT_NEAR(tracks.front()->kf.vy(), 0.0, 1e-9);
}

TEST(TrackerReacquisition, NeverUsesRecoveryForStaticTrack)
{
  auto tracker = make_tracker();
  auto tracks = tracker.step({make_cluster(0.0, 0.0)}, 0.0);
  ASSERT_EQ(tracks.size(), 1u);

  tracks = tracker.step({make_cluster(0.0, 0.60)}, 0.05);
  ASSERT_EQ(tracks.size(), 2u);
  EXPECT_EQ(tracks[0]->id, 0);
  EXPECT_EQ(tracks[0]->misses, 1);
  EXPECT_EQ(tracks[1]->id, 1);
}

TEST(TrackerReacquisition, RefusesAmbiguousDynamicCandidate)
{
  auto tracker = make_tracker();
  // 두 후보는 gating_distance(0.45m)보다 멀어야 한다 — 그보다 가까우면
  // tracker가 "쪼개진 한 물체"로 보고 두 번째 ID를 만들지 않는다
  // (tracker.cpp 9번 파편 스폰 억제). 여기서 검증하려는 것은 그게 아니라
  // "한 cluster에 active dynamic 후보가 둘이면 재연결하지 않는다"이다.
  auto tracks = tracker.step(
    {make_cluster(-0.30, 0.0), make_cluster(0.30, 0.0)}, 0.0);
  ASSERT_EQ(tracks.size(), 2u);
  for (auto & track : tracks) {
    track->ever_dynamic = true;
    track->is_dynamic = true;
  }

  // 하나의 cluster에 active dynamic 후보가 두 개면 안전하게 재연결하지 않는다.
  tracks = tracker.step({make_cluster(0.0, 0.60)}, 0.05);
  ASSERT_EQ(tracks.size(), 3u);
  EXPECT_EQ(tracks[0]->misses, 1);
  EXPECT_EQ(tracks[1]->misses, 1);
  EXPECT_EQ(tracks[2]->id, 2);
}

TEST(TrackerReacquisition, ReidentifiesTrackOnTheRetirementFrame)
{
  auto tracker = make_tracker(0.0);  // active recovery off: exercise retired re-ID ordering.
  auto tracks = tracker.step({make_cluster(0.0, 0.0)}, 0.0);
  ASSERT_EQ(tracks.size(), 1u);
  tracks.front()->ever_dynamic = true;
  tracks.front()->is_dynamic = true;

  tracks = tracker.step({}, 0.05);
  ASSERT_EQ(tracks.size(), 1u);
  EXPECT_EQ(tracks.front()->misses, 1);

  // primary gate 밖의 cluster: miss=2에서 lifecycle이 먼저 retire하고,
  // same-frame retired re-ID가 id=0을 복원한다.
  tracks = tracker.step({make_cluster(0.0, 0.60)}, 0.10);
  ASSERT_EQ(tracks.size(), 1u);
  EXPECT_EQ(tracks.front()->id, 0);
  EXPECT_TRUE(tracks.front()->ever_dynamic);
  EXPECT_TRUE(tracks.front()->is_dynamic);
}

TEST(TrackerReacquisition, FrenetReidentifiesUniqueDynamicTrackAfterCornerFovGap)
{
  // 1.4m track progress는 strict CV Re-ID(.7m) 밖이지만, L자 코너를 따라
  // 같은 차선(d=0)으로 재등장했다. Frenet 보조 경로만 같은 ID를 허용한다.
  auto tracker = make_tracker(0.0, 1.25, 1.60);
  const TrackBounds bounds = make_corner_bounds();
  auto tracks = tracker.step({make_cluster(0.0, 0.0)}, 0.0, &bounds);
  ASSERT_EQ(tracks.size(), 1u);
  tracks.front()->ever_dynamic = true;
  tracks.front()->is_dynamic = true;
  tracks.front()->dynamic_streak = 4;
  tracks.front()->confidence = 1.0;

  tracks = tracker.step({make_cluster(0.2, 0.0)}, 0.05, &bounds);
  ASSERT_EQ(tracks.size(), 1u);
  tracks = tracker.step({}, 0.10, &bounds);
  ASSERT_EQ(tracks.size(), 1u);
  tracks = tracker.step({}, 0.15, &bounds);  // timeout=1 -> retired
  EXPECT_TRUE(tracks.empty());

  tracks = tracker.step({make_cluster(1.0, 0.6)}, 0.20, &bounds);
  ASSERT_EQ(tracks.size(), 1u);
  EXPECT_EQ(tracks.front()->id, 0);
  EXPECT_TRUE(tracks.front()->ever_dynamic);
  EXPECT_TRUE(tracks.front()->is_dynamic);
  EXPECT_EQ(tracks.front()->misses, 0);
  EXPECT_NEAR(tracks.front()->kf.vx(), 0.0, 1e-9);
  EXPECT_NEAR(tracks.front()->kf.vy(), 0.0, 1e-9);
  EXPECT_DOUBLE_EQ(tracks.front()->confidence, 1.0);
}

TEST(TrackerReacquisition, FrenetReidRejectsAmbiguousDynamicReentry)
{
  auto tracker = make_tracker(0.0, 1.25, 1.60);
  const TrackBounds bounds = make_corner_bounds();
  auto tracks = tracker.step({make_cluster(0.0, 0.0)}, 0.0, &bounds);
  ASSERT_EQ(tracks.size(), 1u);
  tracks.front()->ever_dynamic = true;
  tracks.front()->is_dynamic = true;

  tracks = tracker.step({make_cluster(0.2, 0.0)}, 0.05, &bounds);
  tracks = tracker.step({}, 0.10, &bounds);
  tracks = tracker.step({}, 0.15, &bounds);
  EXPECT_TRUE(tracks.empty());

  // 후보가 두 개면 어느 쪽도 기존 dynamic ID를 받지 않는다.
  // 간격은 gating_distance(0.45m)보다 넓게 둔다 — 더 가까우면 파편 스폰 억제가
  // 두 번째 ID를 막아 이 테스트의 전제(후보 2개)가 성립하지 않는다.
  // (1.0,0.10): s=1.0 d=0.10, (1.0,0.6): s=1.6 d=0 — 둘 다 Frenet 후보 조건 안.
  tracks = tracker.step({make_cluster(1.0, 0.6), make_cluster(1.0, 0.10)}, 0.20, &bounds);
  ASSERT_EQ(tracks.size(), 2u);
  EXPECT_EQ(tracks[0]->id, 1);
  EXPECT_EQ(tracks[1]->id, 2);
}

}  // namespace
}  // namespace perception_pkg
