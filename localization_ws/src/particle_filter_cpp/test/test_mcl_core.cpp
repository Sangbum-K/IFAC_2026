// Copyright 2026 IFAC F1TENTH Team
// SPDX-License-Identifier: MIT

#include <cmath>
#include <vector>

#include "gtest/gtest.h"
#include "particle_filter_cpp/mcl_core.hpp"

namespace pf = particle_filter_cpp;

TEST(PoseMath, ComposeInverseRoundTrip)
{
  const pf::Pose2d pose{1.2, -0.7, 2.9};
  const auto identity = pf::compose(pf::inverse(pose), pose);
  EXPECT_NEAR(identity.x, 0.0, 1.0e-12);
  EXPECT_NEAR(identity.y, 0.0, 1.0e-12);
  EXPECT_NEAR(identity.yaw, 0.0, 1.0e-12);
}

TEST(OdomBuffer, InterpolatesAcrossYawBranchCut)
{
  pf::OdomBuffer buffer(8);
  ASSERT_TRUE(buffer.append({100, {0.0, 0.0, 3.10}, 1.0, 0.0}));
  ASSERT_TRUE(buffer.append({200, {2.0, 4.0, -3.10}, 3.0, 2.0}));
  const auto sample = buffer.lookup(150, 1.0);
  ASSERT_TRUE(sample.has_value());
  EXPECT_NEAR(sample->pose.x, 1.0, 1.0e-12);
  EXPECT_NEAR(sample->pose.y, 2.0, 1.0e-12);
  EXPECT_NEAR(std::abs(sample->pose.yaw), M_PI, 0.05);
  EXPECT_NEAR(sample->linear_speed, 2.0, 1.0e-12);
}

TEST(ImuBuffer, TrapezoidalIntegrationUsesExactBoundaries)
{
  pf::ImuBuffer buffer(8);
  ASSERT_TRUE(buffer.append({1000000000LL, 1.0, 0.1, 0.2}));
  ASSERT_TRUE(buffer.append({1100000000LL, 3.0, 0.2, 0.4}));
  ASSERT_TRUE(buffer.append({1200000000LL, 5.0, 0.3, 0.6}));
  const auto result = buffer.integrate(1050000000LL, 1150000000LL, 0.2);
  ASSERT_TRUE(result.valid);
  EXPECT_NEAR(result.delta_yaw, 0.3, 1.0e-12);
  EXPECT_EQ(result.sample_count, 3U);
}

TEST(QualityTracker, UsesConsecutiveHysteresis)
{
  pf::QualityTracker tracker(0.7, 0.4, 2, 3);
  EXPECT_EQ(tracker.state(), pf::QualityState::kStartup);
  tracker.update(0.8);
  tracker.update(0.8);
  EXPECT_EQ(tracker.state(), pf::QualityState::kStartup);
  tracker.update(0.8);
  EXPECT_EQ(tracker.state(), pf::QualityState::kOk);
  tracker.update(0.1);
  EXPECT_EQ(tracker.state(), pf::QualityState::kDegraded);
  tracker.update(0.1);
  EXPECT_EQ(tracker.state(), pf::QualityState::kLost);
}

TEST(SlipDetector, ImmediateEntryAndEvidenceBasedRecovery)
{
  pf::SlipDetector detector(0.15, 0.15, 1.0, 0.5, 2, 2, 2.0);
  auto update = detector.update(
    {0.0, 0.0, 0.0}, 0.0, true, {0.35, 0.0, 0.0}, true);
  EXPECT_TRUE(update.active);
  EXPECT_TRUE(update.entered);
  update = detector.update(
    {0.0, 0.0, 0.0}, 0.0, true, {0.0, 0.0, 0.0}, true);
  EXPECT_TRUE(update.active);
  update = detector.update(
    {0.0, 0.0, 0.0}, 0.0, true, {0.0, 0.0, 0.0}, true);
  EXPECT_FALSE(update.active);
  EXPECT_TRUE(update.recovered);
}

TEST(SlipDetector, DisabledScanMatchClearsLatchedSlip)
{
  pf::SlipDetector detector(0.15, 0.15, 1.0, 0.5, 2, 2, 2.0);
  const auto entered = detector.update(
    {0.0, 0.0, 0.0}, 0.0, true, {0.35, 0.0, 0.0}, true);
  ASSERT_TRUE(entered.active);

  const auto disabled = detector.disabled_update();
  EXPECT_FALSE(disabled.active);
  EXPECT_FALSE(disabled.evidence_valid);

  const auto next = detector.update(
    {0.0, 0.0, 0.0}, 0.0, true, {0.0, 0.0, 0.0}, true);
  EXPECT_FALSE(next.active);
}

TEST(ScanMatcher, RecoversSmallTranslation)
{
  std::vector<float> angles;
  std::vector<float> previous;
  std::vector<float> current;
  for (int index = -20; index <= 20; ++index) {
    const float angle = static_cast<float>(index) * 0.03F;
    angles.push_back(angle);
    previous.push_back(3.0F + 0.2F * std::sin(static_cast<float>(index) * 0.4F));
    current.push_back(previous.back());
  }
  const auto result = pf::estimate_scan_motion(
    previous, current, angles, angles, {}, 10.0, 0.4, 0.9, 4, 12);
  ASSERT_TRUE(result.valid);
  EXPECT_NEAR(result.action.x, 0.0, 1.0e-6);
  EXPECT_NEAR(result.action.y, 0.0, 1.0e-6);
  EXPECT_NEAR(result.action.yaw, 0.0, 1.0e-6);
}

TEST(MclCore, DeterministicSeedProducesStableInitialization)
{
  pf::MclParameters parameters;
  parameters.particle_count = 16;
  parameters.max_range_px = 20;
  parameters.random_seed = 809;
  pf::MclCore first(parameters);
  pf::MclCore second(parameters);
  first.initialize_pose({1.0, 2.0, 0.5}, 0.5, 0.4);
  second.initialize_pose({1.0, 2.0, 0.5}, 0.5, 0.4);
  ASSERT_EQ(first.particles().size(), second.particles().size());
  for (std::size_t index = 0; index < first.particles().size(); ++index) {
    EXPECT_DOUBLE_EQ(first.particles()[index].x, second.particles()[index].x);
    EXPECT_DOUBLE_EQ(first.particles()[index].y, second.particles()[index].y);
    EXPECT_DOUBLE_EQ(first.particles()[index].yaw, second.particles()[index].yaw);
  }
}
