// Copyright 2026 IFAC F1TENTH Team
// SPDX-License-Identifier: MIT

#include <cmath>
#include <stdexcept>
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

TEST(OdomBuffer, RejectsSmallOutOfOrderWithoutMutationAndRecovers)
{
  constexpr int64_t second_ns = 1000000000LL;
  pf::OdomBuffer buffer(8);
  ASSERT_TRUE(buffer.append({10 * second_ns, {0.0, 0.0, 0.0}, 1.0, 0.0}));
  ASSERT_TRUE(buffer.append({10 * second_ns + second_ns / 5, {2.0, 4.0, 0.0}, 3.0, 2.0}));

  const auto size_before_reject = buffer.size();
  const auto reset_count_before_reject = buffer.time_reset_count();
  const auto latest_before_reject = buffer.latest();
  const auto lookup_before_reject = buffer.lookup(10 * second_ns + second_ns / 10, 1.0);
  ASSERT_TRUE(latest_before_reject.has_value());
  ASSERT_TRUE(lookup_before_reject.has_value());

  EXPECT_FALSE(buffer.append(
      {10 * second_ns + second_ns / 10, {99.0, 98.0, 0.5}, 97.0, 96.0}));
  EXPECT_EQ(buffer.size(), size_before_reject);
  EXPECT_EQ(buffer.time_reset_count(), reset_count_before_reject);

  const auto latest_after_reject = buffer.latest();
  const auto lookup_after_reject = buffer.lookup(10 * second_ns + second_ns / 10, 1.0);
  ASSERT_TRUE(latest_after_reject.has_value());
  ASSERT_TRUE(lookup_after_reject.has_value());
  EXPECT_EQ(latest_after_reject->stamp_ns, latest_before_reject->stamp_ns);
  EXPECT_DOUBLE_EQ(latest_after_reject->pose.x, latest_before_reject->pose.x);
  EXPECT_DOUBLE_EQ(latest_after_reject->pose.y, latest_before_reject->pose.y);
  EXPECT_DOUBLE_EQ(latest_after_reject->pose.yaw, latest_before_reject->pose.yaw);
  EXPECT_DOUBLE_EQ(latest_after_reject->linear_speed, latest_before_reject->linear_speed);
  EXPECT_DOUBLE_EQ(latest_after_reject->angular_speed, latest_before_reject->angular_speed);
  EXPECT_EQ(lookup_after_reject->stamp_ns, lookup_before_reject->stamp_ns);
  EXPECT_DOUBLE_EQ(lookup_after_reject->pose.x, lookup_before_reject->pose.x);
  EXPECT_DOUBLE_EQ(lookup_after_reject->pose.y, lookup_before_reject->pose.y);
  EXPECT_DOUBLE_EQ(lookup_after_reject->pose.yaw, lookup_before_reject->pose.yaw);
  EXPECT_DOUBLE_EQ(lookup_after_reject->linear_speed, lookup_before_reject->linear_speed);
  EXPECT_DOUBLE_EQ(lookup_after_reject->angular_speed, lookup_before_reject->angular_speed);

  ASSERT_TRUE(buffer.append(
      {10 * second_ns + 3 * second_ns / 10, {4.0, 8.0, 0.0}, 5.0, 4.0}));
  EXPECT_EQ(buffer.size(), size_before_reject + 1U);
  const auto recovered = buffer.lookup(10 * second_ns + second_ns / 4, 1.0);
  ASSERT_TRUE(recovered.has_value());
  EXPECT_DOUBLE_EQ(recovered->pose.x, 3.0);
  EXPECT_DOUBLE_EQ(recovered->pose.y, 6.0);
  EXPECT_DOUBLE_EQ(recovered->pose.yaw, 0.0);
  EXPECT_DOUBLE_EQ(recovered->linear_speed, 4.0);
  EXPECT_DOUBLE_EQ(recovered->angular_speed, 3.0);
}

TEST(OdomBuffer, EqualTimestampReplacesLatestSample)
{
  constexpr int64_t second_ns = 1000000000LL;
  pf::OdomBuffer buffer(8);
  ASSERT_TRUE(buffer.append({10 * second_ns, {0.0, 0.0, 0.0}, 1.0, 0.0}));
  ASSERT_TRUE(buffer.append({10 * second_ns + second_ns / 5, {2.0, 4.0, 0.0}, 3.0, 2.0}));
  const auto size_before_replace = buffer.size();

  ASSERT_TRUE(buffer.append(
      {10 * second_ns + second_ns / 5, {7.0, 8.0, 0.5}, 9.0, 10.0}));
  EXPECT_EQ(buffer.size(), size_before_replace);
  EXPECT_EQ(buffer.time_reset_count(), 0U);
  const auto latest = buffer.latest();
  ASSERT_TRUE(latest.has_value());
  EXPECT_EQ(latest->stamp_ns, 10 * second_ns + second_ns / 5);
  EXPECT_DOUBLE_EQ(latest->pose.x, 7.0);
  EXPECT_DOUBLE_EQ(latest->pose.y, 8.0);
  EXPECT_DOUBLE_EQ(latest->pose.yaw, 0.5);
  EXPECT_DOUBLE_EQ(latest->linear_speed, 9.0);
  EXPECT_DOUBLE_EQ(latest->angular_speed, 10.0);
}

TEST(OdomBuffer, ResetsAtOneSecondBackwardBoundary)
{
  constexpr int64_t second_ns = 1000000000LL;
  pf::OdomBuffer buffer(8);
  ASSERT_TRUE(buffer.append({10 * second_ns, {0.0, 0.0, 0.0}, 1.0, 0.0}));
  ASSERT_TRUE(buffer.append({10 * second_ns + second_ns / 5, {2.0, 4.0, 0.0}, 3.0, 2.0}));

  const int64_t reset_stamp = 9 * second_ns + second_ns / 5;
  ASSERT_TRUE(buffer.append({reset_stamp, {8.0, 16.0, 0.0}, 7.0, 6.0}));
  EXPECT_EQ(buffer.size(), 1U);
  EXPECT_EQ(buffer.time_reset_count(), 1U);
  EXPECT_FALSE(buffer.lookup(10 * second_ns, 1.0).has_value());
  const auto reset_sample = buffer.lookup(reset_stamp, 1.0);
  ASSERT_TRUE(reset_sample.has_value());
  EXPECT_DOUBLE_EQ(reset_sample->pose.x, 8.0);
  EXPECT_DOUBLE_EQ(reset_sample->pose.y, 16.0);
  EXPECT_DOUBLE_EQ(reset_sample->linear_speed, 7.0);
  EXPECT_DOUBLE_EQ(reset_sample->angular_speed, 6.0);

  ASSERT_TRUE(buffer.append(
      {reset_stamp + second_ns / 10, {10.0, 20.0, 0.0}, 9.0, 8.0}));
  const auto recovered = buffer.lookup(reset_stamp + second_ns / 20, 1.0);
  ASSERT_TRUE(recovered.has_value());
  EXPECT_DOUBLE_EQ(recovered->pose.x, 9.0);
  EXPECT_DOUBLE_EQ(recovered->pose.y, 18.0);
  EXPECT_DOUBLE_EQ(recovered->pose.yaw, 0.0);
  EXPECT_DOUBLE_EQ(recovered->linear_speed, 8.0);
  EXPECT_DOUBLE_EQ(recovered->angular_speed, 7.0);
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

TEST(MclCore, RestoreMotionOnlyRestoresSnapshotAndAppliesActionOnce)
{
  pf::MclParameters parameters;
  parameters.particle_count = 3;
  parameters.random_seed = 809;
  pf::MclCore core(parameters);

  pf::Proposal sensor_proposal;
  sensor_proposal.particles = {
    {100.0, 101.0, 0.75},
    {200.0, 201.0, 1.00},
    {300.0, 301.0, 1.25}};
  sensor_proposal.prior_weights = {0.2, 0.3, 0.5};
  core.accept(sensor_proposal, {8.0, 1.0, 1.0});
  ASSERT_DOUBLE_EQ(core.particles()[0].x, 100.0);
  ASSERT_DOUBLE_EQ(core.weights()[0], 0.8);

  const std::vector<pf::Particle> prior_particles{
    {1.0, 2.0, 0.0},
    {-1.0, 4.0, 0.0},
    {3.0, -2.0, 0.0}};
  const std::vector<double> prior_weights{2.0, 3.0, 5.0};
  core.restore_motion_only(prior_particles, prior_weights, {2.0, -1.0, 0.25});

  const std::vector<pf::Particle> expected_particles{
    {3.0, 1.0, 0.25},
    {1.0, 3.0, 0.25},
    {5.0, -3.0, 0.25}};
  const std::vector<double> expected_weights{0.2, 0.3, 0.5};
  ASSERT_EQ(core.particles().size(), expected_particles.size());
  ASSERT_EQ(core.weights().size(), expected_weights.size());
  for (std::size_t index = 0; index < expected_particles.size(); ++index) {
    EXPECT_DOUBLE_EQ(core.particles()[index].x, expected_particles[index].x);
    EXPECT_DOUBLE_EQ(core.particles()[index].y, expected_particles[index].y);
    EXPECT_NEAR(core.particles()[index].yaw, expected_particles[index].yaw, 1.0e-12);
    EXPECT_DOUBLE_EQ(core.weights()[index], expected_weights[index]);
  }
}

TEST(MclCore, RestoreMotionOnlyRejectsInvalidSnapshotWithoutMutation)
{
  pf::MclParameters parameters;
  parameters.particle_count = 3;
  parameters.random_seed = 809;
  pf::MclCore core(parameters);
  const auto particles_before = core.particles();
  const auto weights_before = core.weights();

  EXPECT_THROW(
    core.restore_motion_only({}, {}, {2.0, -1.0, 0.25}),
    std::invalid_argument);
  EXPECT_THROW(
    core.restore_motion_only(
      {{1.0, 2.0, 0.0}, {-1.0, 4.0, 0.0}, {3.0, -2.0, 0.0}},
      {0.5, 0.5}, {2.0, -1.0, 0.25}),
    std::invalid_argument);

  ASSERT_EQ(core.particles().size(), particles_before.size());
  ASSERT_EQ(core.weights().size(), weights_before.size());
  for (std::size_t index = 0; index < particles_before.size(); ++index) {
    EXPECT_DOUBLE_EQ(core.particles()[index].x, particles_before[index].x);
    EXPECT_DOUBLE_EQ(core.particles()[index].y, particles_before[index].y);
    EXPECT_DOUBLE_EQ(core.particles()[index].yaw, particles_before[index].yaw);
    EXPECT_DOUBLE_EQ(core.weights()[index], weights_before[index]);
  }
}

TEST(MclCore, RestoreMotionOnlyUsesUniformWeightsWhenSnapshotWeightsCannotNormalize)
{
  pf::MclParameters parameters;
  parameters.particle_count = 3;
  parameters.random_seed = 809;
  pf::MclCore core(parameters);

  core.restore_motion_only(
    {{1.0, 2.0, 0.0}, {-1.0, 4.0, 0.0}, {3.0, -2.0, 0.0}},
    {0.0, 0.0, 0.0}, {});

  ASSERT_EQ(core.weights().size(), 3U);
  for (const double weight : core.weights()) {
    EXPECT_DOUBLE_EQ(weight, 1.0 / 3.0);
  }
}
