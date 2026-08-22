// Copyright 2026 IFAC F1TENTH Team
// SPDX-License-Identifier: MIT

#include "particle_filter_cpp/mcl_core.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <numeric>
#include <stdexcept>
#include <utility>

namespace particle_filter_cpp
{
namespace
{
constexpr double kPi = 3.14159265358979323846;
constexpr int64_t kResetThresholdNs = 1000000000LL;
constexpr double kTinyWeight = 1.0e-300;

template<typename T>
T lerp(const T lhs, const T rhs, const double ratio)
{
  return static_cast<T>(lhs + (rhs - lhs) * ratio);
}

bool finite_pose(const Pose2d & pose)
{
  return std::isfinite(pose.x) && std::isfinite(pose.y) &&
         std::isfinite(pose.yaw);
}

struct Point2d
{
  double x{0.0};
  double y{0.0};
};

Point2d transform_point(const Point2d & point, const Pose2d & pose)
{
  const double cosine = std::cos(pose.yaw);
  const double sine = std::sin(pose.yaw);
  return {
    pose.x + cosine * point.x - sine * point.y,
    pose.y + sine * point.x + cosine * point.y};
}

double squared_distance(const Point2d & lhs, const Point2d & rhs)
{
  const double dx = lhs.x - rhs.x;
  const double dy = lhs.y - rhs.y;
  return dx * dx + dy * dy;
}

double quantile(std::vector<double> values, const double fraction)
{
  if (values.empty()) {
    return std::numeric_limits<double>::infinity();
  }
  std::sort(values.begin(), values.end());
  const double position = fraction * static_cast<double>(values.size() - 1U);
  const auto lower = static_cast<std::size_t>(std::floor(position));
  const auto upper = static_cast<std::size_t>(std::ceil(position));
  if (lower == upper) {
    return values[lower];
  }
  return lerp(values[lower], values[upper], position - static_cast<double>(lower));
}
}  // namespace

double wrap_angle(const double angle)
{
  return std::atan2(std::sin(angle), std::cos(angle));
}

Pose2d compose(const Pose2d & lhs, const Pose2d & rhs)
{
  const double cosine = std::cos(lhs.yaw);
  const double sine = std::sin(lhs.yaw);
  return {
    lhs.x + cosine * rhs.x - sine * rhs.y,
    lhs.y + sine * rhs.x + cosine * rhs.y,
    wrap_angle(lhs.yaw + rhs.yaw)};
}

Pose2d inverse(const Pose2d & pose)
{
  const double cosine = std::cos(pose.yaw);
  const double sine = std::sin(pose.yaw);
  return {
    -cosine * pose.x - sine * pose.y,
    sine * pose.x - cosine * pose.y,
    wrap_angle(-pose.yaw)};
}

double translation_norm(const Pose2d & pose)
{
  return std::hypot(pose.x, pose.y);
}

OdomBuffer::OdomBuffer(const std::size_t capacity)
: capacity_(capacity)
{
  if (capacity < 2U) {
    throw std::invalid_argument("odom buffer capacity must be at least 2");
  }
}

bool OdomBuffer::append(const OdomSample & sample)
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (!samples_.empty()) {
    const int64_t newest = samples_.back().stamp_ns;
    if (sample.stamp_ns == newest) {
      samples_.back() = sample;
      return true;
    }
    if (sample.stamp_ns < newest) {
      if (newest - sample.stamp_ns >= kResetThresholdNs) {
        samples_.clear();
        ++time_reset_count_;
      } else {
        return false;
      }
    }
  }
  samples_.push_back(sample);
  while (samples_.size() > capacity_) {
    samples_.pop_front();
  }
  return true;
}

std::optional<OdomSample> OdomBuffer::lookup(
  const int64_t stamp_ns, const double max_gap_sec) const
{
  const int64_t max_gap_ns = static_cast<int64_t>(max_gap_sec * 1.0e9);
  std::lock_guard<std::mutex> lock(mutex_);
  if (samples_.empty() || stamp_ns < samples_.front().stamp_ns ||
    stamp_ns > samples_.back().stamp_ns)
  {
    return std::nullopt;
  }
  const auto upper = std::lower_bound(
    samples_.begin(), samples_.end(), stamp_ns,
    [](const OdomSample & sample, const int64_t value) {
      return sample.stamp_ns < value;
    });
  if (upper != samples_.end() && upper->stamp_ns == stamp_ns) {
    return *upper;
  }
  if (upper == samples_.begin() || upper == samples_.end()) {
    return std::nullopt;
  }
  const auto lower = std::prev(upper);
  const int64_t gap_ns = upper->stamp_ns - lower->stamp_ns;
  if (gap_ns <= 0 || gap_ns > max_gap_ns) {
    return std::nullopt;
  }
  const double ratio = static_cast<double>(stamp_ns - lower->stamp_ns) /
    static_cast<double>(gap_ns);
  const double yaw_delta = wrap_angle(upper->pose.yaw - lower->pose.yaw);
  return OdomSample{
    stamp_ns,
    {
      lerp(lower->pose.x, upper->pose.x, ratio),
      lerp(lower->pose.y, upper->pose.y, ratio),
      wrap_angle(lower->pose.yaw + ratio * yaw_delta)},
    lerp(lower->linear_speed, upper->linear_speed, ratio),
    lerp(lower->angular_speed, upper->angular_speed, ratio)};
}

std::optional<OdomSample> OdomBuffer::latest() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return samples_.empty() ? std::nullopt : std::optional<OdomSample>(samples_.back());
}

void OdomBuffer::clear()
{
  std::lock_guard<std::mutex> lock(mutex_);
  samples_.clear();
}

std::size_t OdomBuffer::size() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return samples_.size();
}

std::size_t OdomBuffer::capacity() const {return capacity_;}

std::size_t OdomBuffer::time_reset_count() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return time_reset_count_;
}

ImuBuffer::ImuBuffer(const std::size_t capacity)
: capacity_(capacity)
{
  if (capacity < 2U) {
    throw std::invalid_argument("IMU buffer capacity must be at least 2");
  }
}

bool ImuBuffer::append(const ImuSample & sample)
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (!samples_.empty()) {
    const int64_t newest = samples_.back().stamp_ns;
    if (sample.stamp_ns == newest) {
      samples_.back() = sample;
      return true;
    }
    if (sample.stamp_ns < newest) {
      if (newest - sample.stamp_ns >= kResetThresholdNs) {
        samples_.clear();
        ++time_reset_count_;
      } else {
        return false;
      }
    }
  }
  samples_.push_back(sample);
  while (samples_.size() > capacity_) {
    samples_.pop_front();
  }
  return true;
}

std::optional<ImuSample> ImuBuffer::interpolate(
  const std::deque<ImuSample> & samples, const int64_t stamp_ns,
  const int64_t max_gap_ns)
{
  const auto upper = std::lower_bound(
    samples.begin(), samples.end(), stamp_ns,
    [](const ImuSample & sample, const int64_t value) {
      return sample.stamp_ns < value;
    });
  if (upper != samples.end() && upper->stamp_ns == stamp_ns) {
    return *upper;
  }
  if (upper == samples.begin() || upper == samples.end()) {
    return std::nullopt;
  }
  const auto lower = std::prev(upper);
  const int64_t gap_ns = upper->stamp_ns - lower->stamp_ns;
  if (gap_ns <= 0 || gap_ns > max_gap_ns) {
    return std::nullopt;
  }
  const double ratio = static_cast<double>(stamp_ns - lower->stamp_ns) /
    static_cast<double>(gap_ns);
  return ImuSample{
    stamp_ns,
    lerp(lower->yaw_rate, upper->yaw_rate, ratio),
    lerp(lower->acceleration_x, upper->acceleration_x, ratio),
    lerp(lower->acceleration_y, upper->acceleration_y, ratio)};
}

ImuIntegration ImuBuffer::integrate(
  const int64_t start_ns, const int64_t end_ns, const double max_gap_sec) const
{
  if (end_ns < start_ns || max_gap_sec <= 0.0) {
    return {};
  }
  const int64_t max_gap_ns = static_cast<int64_t>(max_gap_sec * 1.0e9);
  std::lock_guard<std::mutex> lock(mutex_);
  if (samples_.empty() || start_ns < samples_.front().stamp_ns ||
    end_ns > samples_.back().stamp_ns)
  {
    return {};
  }
  auto start = interpolate(samples_, start_ns, max_gap_ns);
  auto end = interpolate(samples_, end_ns, max_gap_ns);
  if (!start || !end) {
    return {};
  }
  std::vector<ImuSample> points;
  points.reserve(samples_.size() + 2U);
  points.push_back(*start);
  for (const auto & sample : samples_) {
    if (sample.stamp_ns > start_ns && sample.stamp_ns < end_ns) {
      points.push_back(sample);
    }
  }
  if (end_ns > start_ns) {
    points.push_back(*end);
  }
  ImuIntegration result;
  result.valid = true;
  result.sample_count = points.size();
  for (const auto & point : points) {
    result.max_abs_yaw_rate = std::max(result.max_abs_yaw_rate, std::abs(point.yaw_rate));
    result.max_horizontal_acceleration = std::max(
      result.max_horizontal_acceleration,
      std::hypot(point.acceleration_x, point.acceleration_y));
  }
  for (std::size_t index = 1; index < points.size(); ++index) {
    const auto & lower = points[index - 1U];
    const auto & upper = points[index];
    const int64_t gap_ns = upper.stamp_ns - lower.stamp_ns;
    if (gap_ns <= 0 || gap_ns > max_gap_ns) {
      return {};
    }
    result.delta_yaw += 0.5 * (lower.yaw_rate + upper.yaw_rate) *
      (static_cast<double>(gap_ns) / 1.0e9);
  }
  return result;
}

void ImuBuffer::clear()
{
  std::lock_guard<std::mutex> lock(mutex_);
  samples_.clear();
}

std::size_t ImuBuffer::size() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return samples_.size();
}

std::size_t ImuBuffer::capacity() const {return capacity_;}

std::size_t ImuBuffer::time_reset_count() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return time_reset_count_;
}

const char * quality_state_name(const QualityState state)
{
  switch (state) {
    case QualityState::kStartup: return "STARTUP";
    case QualityState::kDegraded: return "DEGRADED";
    case QualityState::kOk: return "OK";
    case QualityState::kLost: return "LOST";
  }
  return "STARTUP";
}

QualityTracker::QualityTracker(
  const double ok_fraction, const double lost_fraction,
  const int bad_to_lost, const int good_to_recover)
: ok_fraction_(ok_fraction),
  lost_fraction_(lost_fraction),
  bad_to_lost_(bad_to_lost),
  good_to_recover_(good_to_recover)
{
  if (!(0.0 <= lost_fraction && lost_fraction < ok_fraction && ok_fraction <= 1.0) ||
    bad_to_lost <= 0 || good_to_recover <= 0)
  {
    throw std::invalid_argument("invalid localization quality parameters");
  }
}

bool QualityTracker::update(double inlier_fraction)
{
  if (!std::isfinite(inlier_fraction)) {
    inlier_fraction = 0.0;
  }
  inlier_fraction = std::clamp(inlier_fraction, 0.0, 1.0);
  const QualityState previous = state_;
  if (inlier_fraction >= ok_fraction_) {
    consecutive_lost_ = 0;
    ++consecutive_good_;
    if (consecutive_good_ >= good_to_recover_) {
      state_ = QualityState::kOk;
    }
  } else if (inlier_fraction < lost_fraction_) {
    consecutive_good_ = 0;
    ++consecutive_lost_;
    if (consecutive_lost_ >= bad_to_lost_) {
      state_ = QualityState::kLost;
    } else if (state_ != QualityState::kLost) {
      state_ = QualityState::kDegraded;
    }
  } else {
    consecutive_good_ = 0;
    consecutive_lost_ = 0;
    if (state_ != QualityState::kLost) {
      state_ = QualityState::kDegraded;
    }
  }
  return state_ != previous;
}

void QualityTracker::force_lost()
{
  state_ = QualityState::kLost;
  consecutive_good_ = 0;
  consecutive_lost_ = bad_to_lost_;
}

void QualityTracker::reset()
{
  state_ = QualityState::kStartup;
  consecutive_good_ = 0;
  consecutive_lost_ = 0;
}

QualityState QualityTracker::state() const {return state_;}
double QualityTracker::ok_fraction() const {return ok_fraction_;}
int QualityTracker::consecutive_good() const {return consecutive_good_;}
int QualityTracker::consecutive_lost() const {return consecutive_lost_;}

CumulativeInnovationGuard::CumulativeInnovationGuard(
  const double window_sec, const double minimum_span_sec,
  const double maximum_translation_m, const double maximum_yaw_rad,
  const double minimum_coherence)
: window_ns_(static_cast<int64_t>(window_sec * 1.0e9)),
  minimum_span_ns_(static_cast<int64_t>(minimum_span_sec * 1.0e9)),
  maximum_translation_m_(maximum_translation_m),
  maximum_yaw_rad_(maximum_yaw_rad),
  minimum_coherence_(minimum_coherence)
{
  if (window_sec <= 0.0 || minimum_span_sec < 0.0 || minimum_span_sec > window_sec ||
    maximum_translation_m <= 0.0 || maximum_yaw_rad <= 0.0 ||
    minimum_coherence <= 0.0 || minimum_coherence > 1.0)
  {
    throw std::invalid_argument("invalid cumulative innovation guard parameters");
  }
}

InnovationGuardUpdate CumulativeInnovationGuard::update(
  const int64_t stamp_ns, const Pose2d & correction, const bool suspicious)
{
  InnovationGuardUpdate result;
  if (!suspicious || stamp_ns <= 0 || !finite_pose(correction)) {
    reset();
    return result;
  }
  if (!samples_.empty() && stamp_ns <= samples_.back().stamp_ns) {
    reset();
  }
  samples_.push_back({stamp_ns, correction});
  while (!samples_.empty() && stamp_ns - samples_.front().stamp_ns > window_ns_) {
    samples_.pop_front();
  }

  double translation_path_m = 0.0;
  double yaw_path_rad = 0.0;
  for (const Sample & sample : samples_) {
    result.accumulated = compose(result.accumulated, sample.correction);
    translation_path_m += translation_norm(sample.correction);
    yaw_path_rad += std::abs(wrap_angle(sample.correction.yaw));
  }
  result.sample_count = samples_.size();
  if (samples_.size() >= 2U) {
    result.span_sec = static_cast<double>(
      samples_.back().stamp_ns - samples_.front().stamp_ns) / 1.0e9;
  }
  if (translation_path_m > 0.0) {
    result.translation_coherence =
      std::min(1.0, translation_norm(result.accumulated) / translation_path_m);
  }
  if (yaw_path_rad > 0.0) {
    result.yaw_coherence =
      std::min(1.0, std::abs(result.accumulated.yaw) / yaw_path_rad);
  }
  if (samples_.size() < 2U ||
    samples_.back().stamp_ns - samples_.front().stamp_ns < minimum_span_ns_)
  {
    return result;
  }
  const bool translation_trip =
    translation_norm(result.accumulated) >= maximum_translation_m_ &&
    result.translation_coherence >= minimum_coherence_;
  const bool yaw_trip =
    std::abs(result.accumulated.yaw) >= maximum_yaw_rad_ &&
    result.yaw_coherence >= minimum_coherence_;
  result.tripped = translation_trip || yaw_trip;
  return result;
}

void CumulativeInnovationGuard::reset()
{
  samples_.clear();
}

void ReadinessTracker::receive_initial_pose()
{
  initial_pose_received_ = true;
  initial_pose_accepted_ = false;
  ready_ = false;
}

void ReadinessTracker::accept_initial_pose()
{
  initial_pose_received_ = true;
  initial_pose_accepted_ = true;
  ready_ = false;
}

void ReadinessTracker::clear_initial_pose()
{
  initial_pose_received_ = false;
  initial_pose_accepted_ = false;
  ready_ = false;
}

void ReadinessTracker::reset_quality() {ready_ = false;}

bool ReadinessTracker::update(const QualityState state)
{
  const bool previous = ready_;
  if (!initial_pose_accepted_ || state == QualityState::kStartup ||
    state == QualityState::kLost)
  {
    ready_ = false;
  } else if (state == QualityState::kOk) {
    ready_ = true;
  }
  return previous != ready_;
}

bool ReadinessTracker::ready() const {return ready_;}
bool ReadinessTracker::initial_pose_received() const {return initial_pose_received_;}
bool ReadinessTracker::initial_pose_accepted() const {return initial_pose_accepted_;}

SlipDetector::SlipDetector(
  const double translation_threshold_m, const double yaw_threshold_rad,
  const double enter_score, const double recover_score,
  const int enter_updates, const int recover_updates,
  const double immediate_score)
: translation_threshold_m_(translation_threshold_m),
  yaw_threshold_rad_(yaw_threshold_rad),
  enter_score_(enter_score),
  recover_score_(recover_score),
  immediate_score_(immediate_score),
  enter_updates_(enter_updates),
  recover_updates_(recover_updates)
{
  if (translation_threshold_m <= 0.0 || yaw_threshold_rad <= 0.0 ||
    recover_score < 0.0 || recover_score >= enter_score ||
    immediate_score < enter_score || enter_updates <= 0 || recover_updates <= 0)
  {
    throw std::invalid_argument("invalid slip detector parameters");
  }
}

void SlipDetector::reset()
{
  active_ = false;
  consecutive_slip_ = 0;
  consecutive_recovery_ = 0;
}

SlipUpdate SlipDetector::disabled_update()
{
  reset();
  return {};
}

SlipUpdate SlipDetector::update(
  const Pose2d & odom_action, const double imu_delta_yaw,
  const bool imu_valid, const Pose2d & scan_action, const bool scan_valid)
{
  SlipUpdate result;
  std::vector<double> components;
  if (scan_valid) {
    result.translation_innovation_m = std::hypot(
      scan_action.x - odom_action.x, scan_action.y - odom_action.y);
    result.scan_odom_yaw_innovation_rad = std::abs(
      wrap_angle(scan_action.yaw - odom_action.yaw));
    components.push_back(result.translation_innovation_m / translation_threshold_m_);
    components.push_back(result.scan_odom_yaw_innovation_rad / yaw_threshold_rad_);
  }
  if (imu_valid) {
    result.odom_imu_yaw_innovation_rad = std::abs(
      wrap_angle(odom_action.yaw - imu_delta_yaw));
    components.push_back(result.odom_imu_yaw_innovation_rad / yaw_threshold_rad_);
    if (scan_valid) {
      result.scan_imu_yaw_innovation_rad = std::abs(
        wrap_angle(scan_action.yaw - imu_delta_yaw));
      components.push_back(result.scan_imu_yaw_innovation_rad / yaw_threshold_rad_);
    }
  }
  result.evidence_valid = !components.empty();
  result.score = components.empty() ? 0.0 :
    *std::max_element(components.begin(), components.end());
  if (active_) {
    if (scan_valid && imu_valid && result.score <= recover_score_) {
      ++consecutive_recovery_;
      if (consecutive_recovery_ >= recover_updates_) {
        active_ = false;
        result.recovered = true;
        consecutive_recovery_ = 0;
      }
    } else {
      consecutive_recovery_ = 0;
    }
    consecutive_slip_ = 0;
  } else if (result.evidence_valid) {
    if (result.score >= immediate_score_) {
      active_ = true;
      result.entered = true;
      consecutive_slip_ = 0;
    } else if (result.score >= enter_score_) {
      ++consecutive_slip_;
      if (consecutive_slip_ >= enter_updates_) {
        active_ = true;
        result.entered = true;
        consecutive_slip_ = 0;
      }
    } else {
      consecutive_slip_ = 0;
    }
    consecutive_recovery_ = 0;
  }
  result.active = active_;
  return result;
}

ScanMotionEstimate estimate_scan_motion(
  const std::vector<float> & previous_ranges,
  const std::vector<float> & current_ranges,
  const std::vector<float> & previous_angles,
  const std::vector<float> & current_angles,
  const Pose2d & initial_action,
  const double maximum_range_m,
  const double max_correspondence_distance_m,
  const double trim_fraction,
  const int iterations,
  const int min_points)
{
  ScanMotionEstimate failure;
  failure.action = initial_action;
  failure.rmse_m = std::numeric_limits<double>::infinity();
  if (previous_ranges.size() != current_ranges.size() ||
    previous_ranges.size() != previous_angles.size() ||
    current_ranges.size() != current_angles.size() || !finite_pose(initial_action) ||
    maximum_range_m <= 0.0 || max_correspondence_distance_m <= 0.0 ||
    trim_fraction <= 0.0 || trim_fraction > 1.0 || iterations <= 0 || min_points < 3)
  {
    return failure;
  }
  const double usable_limit = maximum_range_m * 0.995;
  std::vector<Point2d> target;
  std::vector<Point2d> source;
  for (std::size_t index = 0; index < previous_ranges.size(); ++index) {
    const double previous = previous_ranges[index];
    if (std::isfinite(previous) && previous > 0.05 && previous < usable_limit) {
      target.push_back({
        previous * std::cos(previous_angles[index]),
        previous * std::sin(previous_angles[index])});
    }
    const double current = current_ranges[index];
    if (std::isfinite(current) && current > 0.05 && current < usable_limit) {
      source.push_back({
        current * std::cos(current_angles[index]),
        current * std::sin(current_angles[index])});
    }
  }
  if (target.size() < static_cast<std::size_t>(min_points) ||
    source.size() < static_cast<std::size_t>(min_points))
  {
    return failure;
  }

  Pose2d pose = initial_action;
  const double max_squared = max_correspondence_distance_m *
    max_correspondence_distance_m;
  for (int iteration = 0; iteration < iterations; ++iteration) {
    std::vector<Point2d> transformed;
    std::vector<std::size_t> nearest;
    std::vector<double> distances;
    transformed.reserve(source.size());
    nearest.reserve(source.size());
    distances.reserve(source.size());
    for (const auto & point : source) {
      const Point2d moved = transform_point(point, pose);
      transformed.push_back(moved);
      double best = std::numeric_limits<double>::infinity();
      std::size_t best_index = 0;
      for (std::size_t target_index = 0; target_index < target.size(); ++target_index) {
        const double candidate = squared_distance(moved, target[target_index]);
        if (candidate < best) {
          best = candidate;
          best_index = target_index;
        }
      }
      nearest.push_back(best_index);
      distances.push_back(std::sqrt(best));
    }
    std::vector<double> in_range_distances;
    for (std::size_t index = 0; index < distances.size(); ++index) {
      if (distances[index] * distances[index] <= max_squared) {
        in_range_distances.push_back(distances[index]);
      }
    }
    if (in_range_distances.size() < static_cast<std::size_t>(min_points)) {
      return failure;
    }
    const double trim_limit = quantile(in_range_distances, trim_fraction);
    std::vector<std::size_t> selected;
    for (std::size_t index = 0; index < distances.size(); ++index) {
      if (distances[index] <= max_correspondence_distance_m &&
        distances[index] <= trim_limit)
      {
        selected.push_back(index);
      }
    }
    if (selected.size() < static_cast<std::size_t>(min_points)) {
      return failure;
    }
    Point2d source_center;
    Point2d target_center;
    for (const auto index : selected) {
      source_center.x += transformed[index].x;
      source_center.y += transformed[index].y;
      target_center.x += target[nearest[index]].x;
      target_center.y += target[nearest[index]].y;
    }
    const double count = static_cast<double>(selected.size());
    source_center.x /= count;
    source_center.y /= count;
    target_center.x /= count;
    target_center.y /= count;
    double h00 = 0.0;
    double h01 = 0.0;
    double h10 = 0.0;
    double h11 = 0.0;
    for (const auto index : selected) {
      const double sx = transformed[index].x - source_center.x;
      const double sy = transformed[index].y - source_center.y;
      const double tx = target[nearest[index]].x - target_center.x;
      const double ty = target[nearest[index]].y - target_center.y;
      h00 += sx * tx;
      h01 += sx * ty;
      h10 += sy * tx;
      h11 += sy * ty;
    }
    const double angle = std::atan2(h01 - h10, h00 + h11);
    const double cosine = std::cos(angle);
    const double sine = std::sin(angle);
    const Pose2d delta{
      target_center.x - (cosine * source_center.x - sine * source_center.y),
      target_center.y - (sine * source_center.x + cosine * source_center.y),
      angle};
    pose = compose(delta, pose);
  }

  std::vector<double> accepted_distances;
  std::size_t in_range_count = 0;
  std::vector<double> all_distances;
  all_distances.reserve(source.size());
  for (const auto & point : source) {
    const auto moved = transform_point(point, pose);
    double best = std::numeric_limits<double>::infinity();
    for (const auto & target_point : target) {
      best = std::min(best, squared_distance(moved, target_point));
    }
    const double distance = std::sqrt(best);
    all_distances.push_back(distance);
    if (distance <= max_correspondence_distance_m) {
      ++in_range_count;
      accepted_distances.push_back(distance);
    }
  }
  if (accepted_distances.size() < static_cast<std::size_t>(min_points)) {
    return failure;
  }
  const double trim_limit = quantile(accepted_distances, trim_fraction);
  double squared_sum = 0.0;
  std::size_t selected_count = 0;
  for (const double distance : all_distances) {
    if (distance <= max_correspondence_distance_m && distance <= trim_limit) {
      squared_sum += distance * distance;
      ++selected_count;
    }
  }
  if (selected_count < static_cast<std::size_t>(min_points)) {
    return failure;
  }
  return {
    true,
    pose,
    std::sqrt(squared_sum / static_cast<double>(selected_count)),
    static_cast<double>(in_range_count) / static_cast<double>(source.size())};
}

MclCore::MclCore(const MclParameters & parameters)
: parameters_(parameters),
  particles_(parameters.particle_count),
  weights_(parameters.particle_count, 1.0 / static_cast<double>(parameters.particle_count))
{
  if (parameters_.particle_count == 0U || parameters_.map_resolution <= 0.0 ||
    parameters_.max_range_px <= 0 || parameters_.squash_factor <= 0.0 ||
    parameters_.resample_ess_ratio <= 0.0 || parameters_.resample_ess_ratio > 1.0)
  {
    throw std::invalid_argument("invalid MCL parameters");
  }
  if (parameters_.random_seed < 0) {
    std::random_device device;
    std::seed_seq seed{device(), device(), device(), device()};
    rng_.seed(seed);
  } else {
    rng_.seed(static_cast<std::mt19937_64::result_type>(parameters_.random_seed));
  }
  build_sensor_log_table();
}

void MclCore::initialize_pose(
  const Pose2d & pose, const double xy_sigma, const double yaw_sigma)
{
  std::normal_distribution<double> xy_noise(0.0, xy_sigma);
  std::normal_distribution<double> yaw_noise(0.0, yaw_sigma);
  for (auto & particle : particles_) {
    particle.x = pose.x + xy_noise(rng_);
    particle.y = pose.y + xy_noise(rng_);
    particle.yaw = wrap_angle(pose.yaw + yaw_noise(rng_));
  }
  std::fill(weights_.begin(), weights_.end(), 1.0 / static_cast<double>(weights_.size()));
}

void MclCore::initialize_global(const std::vector<Pose2d> & permissible_poses)
{
  if (permissible_poses.empty()) {
    throw std::invalid_argument("global initialization needs free map cells");
  }
  std::uniform_int_distribution<std::size_t> index_distribution(
    0U, permissible_poses.size() - 1U);
  std::uniform_real_distribution<double> yaw_distribution(0.0, 2.0 * kPi);
  for (auto & particle : particles_) {
    const auto & source = permissible_poses[index_distribution(rng_)];
    particle = {source.x, source.y, yaw_distribution(rng_)};
  }
  std::fill(weights_.begin(), weights_.end(), 1.0 / static_cast<double>(weights_.size()));
}

bool MclCore::normalize(std::vector<double> & weights)
{
  double total = 0.0;
  for (const double weight : weights) {
    if (!std::isfinite(weight) || weight < 0.0) {
      return false;
    }
    total += weight;
  }
  if (!std::isfinite(total) || total <= kTinyWeight) {
    return false;
  }
  for (auto & weight : weights) {
    weight /= total;
  }
  return true;
}

double MclCore::effective_sample_size(const std::vector<double> & weights)
{
  std::vector<double> normalized = weights;
  if (!normalize(normalized)) {
    return 0.0;
  }
  double squared_sum = 0.0;
  for (const double weight : normalized) {
    squared_sum += weight * weight;
  }
  return squared_sum > 0.0 ? 1.0 / squared_sum : 0.0;
}

std::vector<std::size_t> MclCore::systematic_resample(
  const std::vector<double> & weights)
{
  std::vector<double> normalized = weights;
  if (!normalize(normalized)) {
    throw std::runtime_error("cannot resample invalid particle weights");
  }
  std::vector<double> cumulative(normalized.size());
  std::partial_sum(normalized.begin(), normalized.end(), cumulative.begin());
  cumulative.back() = 1.0;
  std::uniform_real_distribution<double> start_distribution(
    0.0, 1.0 / static_cast<double>(normalized.size()));
  const double start = start_distribution(rng_);
  std::vector<std::size_t> indices(normalized.size());
  std::size_t source = 0;
  for (std::size_t index = 0; index < indices.size(); ++index) {
    const double position = start + static_cast<double>(index) /
      static_cast<double>(indices.size());
    while (source + 1U < cumulative.size() && position >= cumulative[source]) {
      ++source;
    }
    indices[index] = source;
  }
  return indices;
}

void MclCore::apply_motion(
  std::vector<Particle> & particles, const Pose2d & action,
  const double noise_multiplier)
{
  const double normalized_motion =
    translation_norm(action) / parameters_.motion_noise_reference_translation_m +
    std::abs(action.yaw) / parameters_.motion_noise_reference_yaw_rad;
  double noise_scale = std::clamp(
    std::sqrt(normalized_motion), parameters_.motion_noise_min_scale, 2.0);
  noise_scale *= std::max(1.0, noise_multiplier);
  std::normal_distribution<double> noise_x(
    0.0, parameters_.motion_dispersion_x * noise_scale);
  std::normal_distribution<double> noise_y(
    0.0, parameters_.motion_dispersion_y * noise_scale);
  std::normal_distribution<double> noise_yaw(
    0.0, parameters_.motion_dispersion_theta * noise_scale);
  for (auto & particle : particles) {
    const double cosine = std::cos(particle.yaw);
    const double sine = std::sin(particle.yaw);
    particle.x += cosine * action.x - sine * action.y + noise_x(rng_);
    particle.y += sine * action.x + cosine * action.y + noise_y(rng_);
    particle.yaw = wrap_angle(particle.yaw + action.yaw + noise_yaw(rng_));
  }
}

Proposal MclCore::prepare_proposal(
  const Pose2d & action, const double noise_multiplier)
{
  std::vector<double> normalized = weights_;
  if (!normalize(normalized)) {
    throw std::runtime_error("particle weights are invalid");
  }
  Proposal proposal;
  proposal.prior_ess_ratio = effective_sample_size(normalized) /
    static_cast<double>(normalized.size());
  if (proposal.prior_ess_ratio < parameters_.resample_ess_ratio) {
    const auto indices = systematic_resample(normalized);
    proposal.particles.reserve(indices.size());
    for (const auto index : indices) {
      proposal.particles.push_back(particles_[index]);
    }
    proposal.prior_weights.assign(
      indices.size(), 1.0 / static_cast<double>(indices.size()));
    proposal.resampled = true;
  } else {
    proposal.particles = particles_;
    proposal.prior_weights = std::move(normalized);
  }
  apply_motion(proposal.particles, action, noise_multiplier);
  return proposal;
}

void MclCore::build_sensor_log_table()
{
  const int width = parameters_.max_range_px + 1;
  sensor_log_table_.assign(static_cast<std::size_t>(width * width), 0.0F);
  std::vector<double> column(static_cast<std::size_t>(width));
  for (int predicted = 0; predicted < width; ++predicted) {
    double normalizer = 0.0;
    for (int observed = 0; observed < width; ++observed) {
      const double residual = static_cast<double>(observed - predicted);
      double probability = parameters_.z_hit *
        std::exp(-(residual * residual) /
        (2.0 * parameters_.sigma_hit * parameters_.sigma_hit)) /
        (parameters_.sigma_hit * std::sqrt(2.0 * kPi));
      if (observed < predicted) {
        probability += 2.0 * parameters_.z_short *
          static_cast<double>(predicted - observed) / static_cast<double>(predicted);
      }
      if (observed == parameters_.max_range_px) {
        probability += parameters_.z_max;
      }
      if (observed < parameters_.max_range_px) {
        probability += parameters_.z_rand /
          static_cast<double>(parameters_.max_range_px);
      }
      column[static_cast<std::size_t>(observed)] = probability;
      normalizer += probability;
    }
    for (int observed = 0; observed < width; ++observed) {
      const double probability = column[static_cast<std::size_t>(observed)] / normalizer;
      sensor_log_table_[static_cast<std::size_t>(observed * width + predicted)] =
        static_cast<float>(std::log(std::max(probability, 1.0e-12)));
    }
  }
}

double selected_beam_angle_span(
  const std::vector<float> & angles, const std::vector<uint8_t> & mask)
{
  if (angles.size() != mask.size()) {
    return 0.0;
  }
  double minimum = std::numeric_limits<double>::infinity();
  double maximum = -std::numeric_limits<double>::infinity();
  for (std::size_t index = 0; index < angles.size(); ++index) {
    if (mask[index] != 0U && std::isfinite(angles[index])) {
      minimum = std::min(minimum, static_cast<double>(angles[index]));
      maximum = std::max(maximum, static_cast<double>(angles[index]));
    }
  }
  return std::isfinite(minimum) && std::isfinite(maximum) ? maximum - minimum : 0.0;
}

SensorEvaluation MclCore::evaluate_sensor(
  const Proposal & proposal,
  const std::vector<float> & observed_ranges,
  const std::vector<float> & predicted_ranges,
  const bool allow_beam_skip_fallback,
  const std::vector<uint8_t> & required_beam_mask) const
{
  SensorEvaluation result;
  const std::size_t particle_count = proposal.particles.size();
  const std::size_t beam_count = observed_ranges.size();
  if (particle_count == 0U || beam_count == 0U ||
    proposal.prior_weights.size() != particle_count ||
    predicted_ranges.size() != particle_count * beam_count ||
    (!required_beam_mask.empty() && required_beam_mask.size() != beam_count))
  {
    return result;
  }
  std::vector<double> prior = proposal.prior_weights;
  if (!normalize(prior)) {
    return result;
  }
  std::vector<int> observed_px(beam_count);
  std::vector<int> predicted_px(predicted_ranges.size());
  for (std::size_t beam = 0; beam < beam_count; ++beam) {
    observed_px[beam] = std::clamp(
      static_cast<int>(std::nearbyint(observed_ranges[beam] / parameters_.map_resolution)),
      0, parameters_.max_range_px);
  }
  for (std::size_t index = 0; index < predicted_ranges.size(); ++index) {
    predicted_px[index] = std::clamp(
      static_cast<int>(std::nearbyint(predicted_ranges[index] / parameters_.map_resolution)),
      0, parameters_.max_range_px);
  }
  const int distance_px = static_cast<int>(std::round(
    parameters_.beam_skip_distance_m / parameters_.map_resolution));
  std::vector<double> agreement(beam_count, 0.0);
  std::vector<double> particle_inlier(particle_count, 0.0);
  result.beams.mask.assign(beam_count, 1U);
  if (!required_beam_mask.empty()) {
    result.beams.mask = required_beam_mask;
  }
  for (std::size_t particle = 0; particle < particle_count; ++particle) {
    for (std::size_t beam = 0; beam < beam_count; ++beam) {
      const int predicted = predicted_px[particle * beam_count + beam];
      const bool inlier = std::abs(predicted - observed_px[beam]) <= distance_px;
      const bool eligible = parameters_.beam_skip_short_only ?
        predicted <= observed_px[beam] + distance_px : inlier;
      if (eligible) {
        agreement[beam] += prior[particle];
      }
    }
  }
  if (parameters_.beam_skip_enabled) {
    std::size_t required_count = 0;
    std::size_t selected_count = 0;
    for (std::size_t beam = 0; beam < beam_count; ++beam) {
      const bool required = required_beam_mask.empty() || required_beam_mask[beam] != 0U;
      if (required) {
        ++required_count;
      }
      result.beams.mask[beam] = static_cast<uint8_t>(
        required && agreement[beam] >= parameters_.beam_skip_min_particle_fraction);
      if (result.beams.mask[beam]) {
        ++selected_count;
      }
    }
    result.beams.skipped_fraction = required_count == 0U ? 1.0 :
      1.0 - static_cast<double>(selected_count) / static_cast<double>(required_count);
    result.beams.excessive_skip = selected_count == 0U ||
      result.beams.skipped_fraction > parameters_.beam_skip_max_fraction;
    result.beams.fallback_to_all_beams =
      result.beams.excessive_skip && allow_beam_skip_fallback;
    if (result.beams.fallback_to_all_beams) {
      for (std::size_t beam = 0; beam < beam_count; ++beam) {
        result.beams.mask[beam] = static_cast<uint8_t>(
          required_beam_mask.empty() || required_beam_mask[beam] != 0U);
      }
    }
  }
  result.beams.selected_count = static_cast<std::size_t>(std::count(
    result.beams.mask.begin(), result.beams.mask.end(), static_cast<uint8_t>(1U)));
  if (result.beams.selected_count == 0U) {
    return result;
  }
  std::vector<double> log_weights(particle_count, 0.0);
  const int table_width = parameters_.max_range_px + 1;
  result.beams.best_particle_inlier_fraction = 0.0;
  result.beams.weighted_inlier_fraction = 0.0;
  for (std::size_t particle = 0; particle < particle_count; ++particle) {
    std::size_t inlier_count = 0;
    double log_weight = 0.0;
    for (std::size_t beam = 0; beam < beam_count; ++beam) {
      if (!result.beams.mask[beam]) {
        continue;
      }
      const int predicted = predicted_px[particle * beam_count + beam];
      const int observed = observed_px[beam];
      log_weight += sensor_log_table_[static_cast<std::size_t>(
        observed * table_width + predicted)];
      if (std::abs(predicted - observed) <= distance_px) {
        ++inlier_count;
      }
    }
    log_weights[particle] = log_weight / parameters_.squash_factor;
    particle_inlier[particle] = static_cast<double>(inlier_count) /
      static_cast<double>(result.beams.selected_count);
    result.beams.best_particle_inlier_fraction = std::max(
      result.beams.best_particle_inlier_fraction, particle_inlier[particle]);
    result.beams.weighted_inlier_fraction += prior[particle] * particle_inlier[particle];
  }
  const double maximum = *std::max_element(log_weights.begin(), log_weights.end());
  if (!std::isfinite(maximum)) {
    return result;
  }
  result.posterior_weights.resize(particle_count);
  for (std::size_t particle = 0; particle < particle_count; ++particle) {
    result.posterior_weights[particle] = prior[particle] *
      std::exp(log_weights[particle] - maximum);
  }
  result.valid = normalize(result.posterior_weights);
  return result;
}

void MclCore::accept(Proposal proposal, std::vector<double> posterior_weights)
{
  if (proposal.particles.size() != posterior_weights.size() ||
    !normalize(posterior_weights))
  {
    throw std::invalid_argument("invalid posterior");
  }
  particles_ = std::move(proposal.particles);
  weights_ = std::move(posterior_weights);
}

void MclCore::restore_motion_only(
  const std::vector<Particle> & particles,
  const std::vector<double> & weights,
  const Pose2d & action)
{
  if (particles.size() != parameters_.particle_count || weights.size() != particles.size()) {
    throw std::invalid_argument("invalid motion-only snapshot");
  }
  particles_ = particles;
  weights_ = weights;
  if (!normalize(weights_)) {
    std::fill(weights_.begin(), weights_.end(), 1.0 / static_cast<double>(weights_.size()));
  }
  for (auto & particle : particles_) {
    const double cosine = std::cos(particle.yaw);
    const double sine = std::sin(particle.yaw);
    particle.x += cosine * action.x - sine * action.y;
    particle.y += sine * action.x + cosine * action.y;
    particle.yaw = wrap_angle(particle.yaw + action.yaw);
  }
}

Pose2d MclCore::expected_pose() const
{
  Pose2d mean;
  double sine_yaw = 0.0;
  double cosine_yaw = 0.0;
  for (std::size_t index = 0; index < particles_.size(); ++index) {
    mean.x += weights_[index] * particles_[index].x;
    mean.y += weights_[index] * particles_[index].y;
    sine_yaw += weights_[index] * std::sin(particles_[index].yaw);
    cosine_yaw += weights_[index] * std::cos(particles_[index].yaw);
  }
  mean.yaw = std::atan2(sine_yaw, cosine_yaw);
  return mean;
}

std::array<double, 9> MclCore::covariance_3x3() const
{
  std::array<double, 9> covariance{};
  const Pose2d mean = expected_pose();
  for (std::size_t index = 0; index < particles_.size(); ++index) {
    const std::array<double, 3> residual{
      particles_[index].x - mean.x,
      particles_[index].y - mean.y,
      wrap_angle(particles_[index].yaw - mean.yaw)};
    for (std::size_t row = 0; row < 3U; ++row) {
      for (std::size_t column = 0; column < 3U; ++column) {
        covariance[row * 3U + column] += weights_[index] *
          residual[row] * residual[column];
      }
    }
  }
  return covariance;
}

double MclCore::effective_sample_size_ratio() const
{
  return effective_sample_size(weights_) / static_cast<double>(weights_.size());
}

const std::vector<Particle> & MclCore::particles() const {return particles_;}
const std::vector<double> & MclCore::weights() const {return weights_;}

std::vector<float> MclCore::particle_query_array(const Proposal & proposal) const
{
  std::vector<float> queries(proposal.particles.size() * 3U);
  for (std::size_t index = 0; index < proposal.particles.size(); ++index) {
    queries[index * 3U] = static_cast<float>(proposal.particles[index].x);
    queries[index * 3U + 1U] = static_cast<float>(proposal.particles[index].y);
    queries[index * 3U + 2U] = static_cast<float>(proposal.particles[index].yaw);
  }
  return queries;
}

}  // namespace particle_filter_cpp
