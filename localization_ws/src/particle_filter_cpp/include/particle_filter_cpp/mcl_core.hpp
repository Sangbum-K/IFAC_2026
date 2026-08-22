// Copyright 2026 IFAC F1TENTH Team
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.

#ifndef PARTICLE_FILTER_CPP__MCL_CORE_HPP_
#define PARTICLE_FILTER_CPP__MCL_CORE_HPP_

#include <array>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <mutex>
#include <optional>
#include <random>
#include <string>
#include <vector>

namespace particle_filter_cpp
{

struct Pose2d
{
  double x{0.0};
  double y{0.0};
  double yaw{0.0};
};

struct Particle
{
  double x{0.0};
  double y{0.0};
  double yaw{0.0};
};

struct OdomSample
{
  int64_t stamp_ns{0};
  Pose2d pose{};
  double linear_speed{0.0};
  double angular_speed{0.0};
};

struct ImuSample
{
  int64_t stamp_ns{0};
  double yaw_rate{0.0};
  double acceleration_x{0.0};
  double acceleration_y{0.0};
};

struct ImuIntegration
{
  bool valid{false};
  double delta_yaw{0.0};
  double max_abs_yaw_rate{0.0};
  double max_horizontal_acceleration{0.0};
  std::size_t sample_count{0};
};

double wrap_angle(double angle);
Pose2d compose(const Pose2d & lhs, const Pose2d & rhs);
Pose2d inverse(const Pose2d & pose);
double translation_norm(const Pose2d & pose);

class OdomBuffer
{
public:
  explicit OdomBuffer(std::size_t capacity);
  bool append(const OdomSample & sample);
  std::optional<OdomSample> lookup(int64_t stamp_ns, double max_gap_sec) const;
  std::optional<OdomSample> latest() const;
  void clear();
  std::size_t size() const;
  std::size_t capacity() const;
  std::size_t time_reset_count() const;

private:
  std::size_t capacity_;
  mutable std::mutex mutex_;
  std::deque<OdomSample> samples_;
  std::size_t time_reset_count_{0};
};

class ImuBuffer
{
public:
  explicit ImuBuffer(std::size_t capacity);
  bool append(const ImuSample & sample);
  ImuIntegration integrate(
    int64_t start_ns, int64_t end_ns, double max_gap_sec) const;
  void clear();
  std::size_t size() const;
  std::size_t capacity() const;
  std::size_t time_reset_count() const;

private:
  static std::optional<ImuSample> interpolate(
    const std::deque<ImuSample> & samples, int64_t stamp_ns,
    int64_t max_gap_ns);

  std::size_t capacity_;
  mutable std::mutex mutex_;
  std::deque<ImuSample> samples_;
  std::size_t time_reset_count_{0};
};

enum class QualityState
{
  kStartup,
  kDegraded,
  kOk,
  kLost,
};

const char * quality_state_name(QualityState state);

class QualityTracker
{
public:
  QualityTracker(
    double ok_fraction, double lost_fraction, int bad_to_lost,
    int good_to_recover);
  bool update(double inlier_fraction);
  void force_lost();
  void reset();
  QualityState state() const;
  double ok_fraction() const;
  int consecutive_good() const;
  int consecutive_lost() const;

private:
  double ok_fraction_;
  double lost_fraction_;
  int bad_to_lost_;
  int good_to_recover_;
  QualityState state_{QualityState::kStartup};
  int consecutive_good_{0};
  int consecutive_lost_{0};
};

struct InnovationGuardUpdate
{
  bool tripped{false};
  Pose2d accumulated{};
  double span_sec{0.0};
  double translation_coherence{0.0};
  double yaw_coherence{0.0};
  std::size_t sample_count{0};
};

class CumulativeInnovationGuard
{
public:
  CumulativeInnovationGuard(
    double window_sec, double minimum_span_sec,
    double maximum_translation_m, double maximum_yaw_rad,
    double minimum_coherence);
  InnovationGuardUpdate update(
    int64_t stamp_ns, const Pose2d & correction, bool suspicious);
  void reset();

private:
  struct Sample
  {
    int64_t stamp_ns{0};
    Pose2d correction{};
  };

  int64_t window_ns_;
  int64_t minimum_span_ns_;
  double maximum_translation_m_;
  double maximum_yaw_rad_;
  double minimum_coherence_;
  std::deque<Sample> samples_;
};

class ReadinessTracker
{
public:
  void receive_initial_pose();
  void accept_initial_pose();
  void clear_initial_pose();
  void reset_quality();
  bool update(QualityState state);
  bool ready() const;
  bool initial_pose_received() const;
  bool initial_pose_accepted() const;

private:
  bool initial_pose_received_{false};
  bool initial_pose_accepted_{false};
  bool ready_{false};
};

struct SlipUpdate
{
  bool active{false};
  bool entered{false};
  bool recovered{false};
  bool evidence_valid{false};
  double score{0.0};
  double translation_innovation_m{0.0};
  double scan_odom_yaw_innovation_rad{0.0};
  double odom_imu_yaw_innovation_rad{0.0};
  double scan_imu_yaw_innovation_rad{0.0};
};

class SlipDetector
{
public:
  SlipDetector(
    double translation_threshold_m, double yaw_threshold_rad,
    double enter_score, double recover_score, int enter_updates,
    int recover_updates, double immediate_score);
  SlipUpdate update(
    const Pose2d & odom_action, double imu_delta_yaw, bool imu_valid,
    const Pose2d & scan_action, bool scan_valid);
  SlipUpdate disabled_update();
  void reset();

private:
  double translation_threshold_m_;
  double yaw_threshold_rad_;
  double enter_score_;
  double recover_score_;
  double immediate_score_;
  int enter_updates_;
  int recover_updates_;
  bool active_{false};
  int consecutive_slip_{0};
  int consecutive_recovery_{0};
};

struct ScanMotionEstimate
{
  bool valid{false};
  Pose2d action{};
  double rmse_m{0.0};
  double inlier_fraction{0.0};
};

ScanMotionEstimate estimate_scan_motion(
  const std::vector<float> & previous_ranges,
  const std::vector<float> & current_ranges,
  const std::vector<float> & previous_angles,
  const std::vector<float> & current_angles,
  const Pose2d & initial_action,
  double maximum_range_m,
  double max_correspondence_distance_m,
  double trim_fraction,
  int iterations,
  int min_points);

struct BeamSelection
{
  std::vector<uint8_t> mask;
  double skipped_fraction{0.0};
  bool excessive_skip{false};
  bool fallback_to_all_beams{false};
  std::size_t selected_count{0};
  double selected_angle_span_rad{0.0};
  double best_particle_inlier_fraction{0.0};
  double weighted_inlier_fraction{0.0};
};

double selected_beam_angle_span(
  const std::vector<float> & angles, const std::vector<uint8_t> & mask);

struct MclParameters
{
  std::size_t particle_count{2000};
  double motion_dispersion_x{0.05};
  double motion_dispersion_y{0.025};
  double motion_dispersion_theta{0.08};
  double motion_noise_min_scale{0.08};
  double motion_noise_reference_translation_m{0.25};
  double motion_noise_reference_yaw_rad{0.15};
  double resample_ess_ratio{0.50};
  bool beam_skip_enabled{true};
  double beam_skip_distance_m{0.20};
  double beam_skip_min_particle_fraction{0.30};
  double beam_skip_max_fraction{0.50};
  bool beam_skip_short_only{true};
  double squash_factor{2.2};
  double map_resolution{0.05};
  int max_range_px{200};
  double z_short{0.01};
  double z_max{0.07};
  double z_rand{0.12};
  double z_hit{0.75};
  double sigma_hit{4.0};
  int random_seed{-1};
};

struct Proposal
{
  std::vector<Particle> particles;
  std::vector<double> prior_weights;
  bool resampled{false};
  double prior_ess_ratio{1.0};
};

struct SensorEvaluation
{
  bool valid{false};
  std::vector<double> posterior_weights;
  BeamSelection beams;
};

class MclCore
{
public:
  explicit MclCore(const MclParameters & parameters);
  void initialize_pose(const Pose2d & pose, double xy_sigma, double yaw_sigma);
  void initialize_global(const std::vector<Pose2d> & permissible_poses);
  Proposal prepare_proposal(const Pose2d & action, double noise_multiplier);
  SensorEvaluation evaluate_sensor(
    const Proposal & proposal,
    const std::vector<float> & observed_ranges,
    const std::vector<float> & predicted_ranges,
    bool allow_beam_skip_fallback = true,
    const std::vector<uint8_t> & required_beam_mask = {}) const;
  void accept(Proposal proposal, std::vector<double> posterior_weights);
  void restore_motion_only(
    const std::vector<Particle> & particles,
    const std::vector<double> & weights,
    const Pose2d & action);
  Pose2d expected_pose() const;
  std::array<double, 9> covariance_3x3() const;
  double effective_sample_size_ratio() const;
  const std::vector<Particle> & particles() const;
  const std::vector<double> & weights() const;
  std::vector<float> particle_query_array(const Proposal & proposal) const;

private:
  static double effective_sample_size(const std::vector<double> & weights);
  static bool normalize(std::vector<double> & weights);
  std::vector<std::size_t> systematic_resample(
    const std::vector<double> & weights);
  void apply_motion(
    std::vector<Particle> & particles, const Pose2d & action,
    double noise_multiplier);
  void build_sensor_log_table();

  MclParameters parameters_;
  std::vector<Particle> particles_;
  std::vector<double> weights_;
  std::vector<float> sensor_log_table_;
  mutable std::mt19937_64 rng_;
};

}  // namespace particle_filter_cpp

#endif  // PARTICLE_FILTER_CPP__MCL_CORE_HPP_
