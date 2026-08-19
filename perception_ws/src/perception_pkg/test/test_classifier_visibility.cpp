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
  // 창은 이제 프레임이 아니라 **초**다 (2026-08-19). pos_hist 창 1.0 s.
  Track track(0, make_cluster(0.0, 0.0), 0.0, 1.0);
  track.dynamic_time = 0.10;
  track.last_meas_dt = 0.05;

  // v_thresh=0이면 예전 구현은 KF coast만으로도 moving vote를 더해 문턱을
  // 채웠다. 미관측 프레임은 적립 시간을 그대로 유지해야 한다.
  EXPECT_FALSE(classify_track(track, false, 0.0, 0.10, 0.10, 0.10, 0.20, 0.15));
  EXPECT_DOUBLE_EQ(track.dynamic_time, 0.10);
  EXPECT_DOUBLE_EQ(track.static_time, 0.0);
  EXPECT_FALSE(track.is_dynamic);
  EXPECT_FALSE(track.ever_dynamic);
}

TEST(ClassifierVisibility, CoastingFramesDoNotReleaseConfirmedDynamic)
{
  Track track(0, make_cluster(0.0, 0.0), 0.0, 1.0);
  track.is_dynamic = true;
  track.ever_dynamic = true;
  track.dynamic_time = 0.15;
  track.last_meas_dt = 0.05;
  track.confidence = 0.75;

  for (int i = 0; i < 5; ++i) {
    EXPECT_TRUE(classify_track(track, false, 1.0, 0.10, 0.10, 0.10, 0.20, 0.15));
  }
  EXPECT_DOUBLE_EQ(track.static_time, 0.0);
  EXPECT_DOUBLE_EQ(track.dynamic_time, 0.15);
  EXPECT_DOUBLE_EQ(track.confidence, 0.75);

  // 이후 실제 저속 관측이 t_dynamic_release(0.15 s) 만큼 쌓이면 해제된다.
  // 관측당 0.05 s 이므로 세 번.
  for (int i = 0; i < 3; ++i) {
    classify_track(track, true, 1.0, 0.10, 0.10, 0.10, 0.20, 0.15);
  }
  EXPECT_FALSE(track.is_dynamic);
}

TEST(ClassifierVisibility, ConfirmTimeIsIndependentOfMeasurementRate)
{
  // 이 테스트가 2026-08-19 수정의 핵심 계약이다 — 같은 0.30 s 동안 움직임
  // 근거가 이어지면, 측정이 촘촘하든(20 ms) 성기든(100 ms) 똑같이 확정된다.
  // 예전에는 문턱이 프레임 수라서 성긴 쪽이 5배 오래 걸렸다.
  for (double dt : {0.02, 0.10}) {
    Track track(0, make_cluster(0.0, 0.0), 0.0, 3.0);
    track.last_meas_dt = dt;
    double elapsed = 0.0;
    // v_thresh=0 이라 모든 측정 프레임이 moving 투표.
    while (!track.is_dynamic && elapsed < 1.0) {
      classify_track(track, true, 0.0, 0.30, 1.0, 0.10, 0.20, 5.0);
      elapsed += dt;
    }
    EXPECT_TRUE(track.is_dynamic) << "dt=" << dt;
    // 문턱 0.30 s 를 넘긴 첫 측정에서 확정 — 초과분은 dt 하나를 넘지 않는다.
    EXPECT_GE(elapsed, 0.30);
    EXPECT_LE(elapsed, 0.30 + dt + 1e-9);
  }
}

}  // namespace
}  // namespace perception_pkg
