#include <gtest/gtest.h>

#include "perception_pkg/classifier.hpp"

namespace perception_pkg
{
namespace
{

Cluster make_cluster(double x, double y)
{
  Cluster cluster;
  cluster.centroid = Point2{{x, y}};
  cluster.radius = 0.20;
  return cluster;
}

TEST(ClassifierVisibility, CoastingFramesDoNotConfirmDynamic)
{
  Track track(0, make_cluster(0.0, 0.0), 0.0, 10);
  track.dynamic_streak = 1;

  // v_thresh=0이면 기존 구현은 KF coast만으로도 moving vote를 더해 n_confirm=2를
  // 채웠다. 미관측 프레임은 streak을 그대로 유지해야 한다.
  EXPECT_FALSE(classify_track(track, false, 0.0, 2, 2, 0.10, 0.20, 3));
  EXPECT_EQ(track.dynamic_streak, 1);
  EXPECT_EQ(track.static_streak, 0);
  EXPECT_FALSE(track.is_dynamic);
  EXPECT_FALSE(track.ever_dynamic);
}

TEST(ClassifierVisibility, CoastingFramesDoNotReleaseConfirmedDynamic)
{
  Track track(0, make_cluster(0.0, 0.0), 0.0, 10);
  track.is_dynamic = true;
  track.ever_dynamic = true;
  track.dynamic_streak = 3;
  track.confidence = 0.75;

  for (int i = 0; i < 5; ++i) {
    EXPECT_TRUE(classify_track(track, false, 1.0, 2, 2, 0.10, 0.20, 3));
  }
  EXPECT_EQ(track.static_streak, 0);
  EXPECT_EQ(track.dynamic_streak, 3);
  EXPECT_DOUBLE_EQ(track.confidence, 0.75);

  // 이후 실제 저속 관측만 dynamic_release회 들어오면 정상적으로 해제된다.
  for (int i = 0; i < 3; ++i) {
    classify_track(track, true, 1.0, 2, 2, 0.10, 0.20, 3);
  }
  EXPECT_FALSE(track.is_dynamic);
}

}  // namespace
}  // namespace perception_pkg
