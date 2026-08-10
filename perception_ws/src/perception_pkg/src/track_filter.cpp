#include "perception_pkg/track_filter.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>
#include <stdexcept>

namespace perception_pkg
{
namespace
{

bool all_finite(const std::vector<double> & v)
{
  return std::all_of(v.begin(), v.end(), [](double x) {return std::isfinite(x);});
}

}  // namespace

TrackBounds::TrackBounds(const std::vector<Waypoint> & wpnts)
{
  const std::size_t n = wpnts.size();
  if (n < 3) {
    throw std::invalid_argument("need at least 3 waypoints");
  }
  x_.reserve(n);
  y_.reserve(n);
  s_.reserve(n);
  d_left_.reserve(n);
  d_right_.reserve(n);
  psi_.reserve(n);
  for (const auto & w : wpnts) {
    x_.push_back(w.x);
    y_.push_back(w.y);
    s_.push_back(w.s);
    d_left_.push_back(w.d_left);
    d_right_.push_back(w.d_right);
    psi_.push_back(w.psi);
  }
  if (!all_finite(x_) || !all_finite(y_) || !all_finite(s_) ||
    !all_finite(d_left_) || !all_finite(d_right_) || !all_finite(psi_))
  {
    throw std::invalid_argument("waypoints contain non-finite values");
  }
  // s는 단조증가여야 normalize_s(둘레 wrap)가 성립한다.
  for (std::size_t i = 1; i < n; ++i) {
    if (!(s_[i] > s_[i - 1])) {
      throw std::invalid_argument("waypoint s_m must be strictly increasing");
    }
  }
  // 폭이 0이면 필터가 모든 포인트를 지운다 -> 장애물 실명.
  for (std::size_t i = 0; i < n; ++i) {
    if (d_left_[i] <= 0.0 || d_right_[i] <= 0.0) {
      throw std::invalid_argument("waypoint d_left/d_right must be positive");
    }
  }
  length_ = s_.back();
  if (!(length_ > 0.0)) {
    throw std::invalid_argument("track length must be positive");
  }
}

void TrackBounds::project(
  const std::vector<Point2> & points,
  std::vector<std::size_t> & idx_out,
  std::vector<double> & s_out,
  std::vector<double> & d_out) const
{
  const std::size_t n = points.size();
  const std::size_t m = x_.size();
  idx_out.resize(n);
  s_out.resize(n);
  d_out.resize(n);

  for (std::size_t i = 0; i < n; ++i) {
    const double px = points[i][0];
    const double py = points[i][1];
    std::size_t best = 0;
    double best_d2 = std::numeric_limits<double>::infinity();
    for (std::size_t j = 0; j < m; ++j) {
      const double dx = px - x_[j];
      const double dy = py - y_[j];
      const double d2 = dx * dx + dy * dy;
      if (d2 < best_d2) {
        best_d2 = d2;
        best = j;
      }
    }
    const double ex = px - x_[best];
    const double ey = py - y_[best];
    const double c = std::cos(psi_[best]);
    const double s = std::sin(psi_[best]);
    idx_out[i] = best;
    d_out[i] = -s * ex + c * ey;
    s_out[i] = s_[best] + (c * ex + s * ey);
  }
}

double TrackBounds::project_s(const Point2 & point) const
{
  std::vector<std::size_t> idx;
  std::vector<double> s;
  std::vector<double> d;
  project(std::vector<Point2>{point}, idx, s, d);
  return s.front();
}

double TrackBounds::normalize_s(double ds) const
{
  // 파이썬 np.mod와 같은 부호 규약: 결과는 항상 [0, length_)
  double m = std::fmod(ds, length_);
  if (m < 0.0) {
    m += length_;
  }
  return (m > length_ * 0.5) ? m - length_ : m;
}

std::vector<uint8_t> TrackBounds::on_track_mask(
  const std::vector<Point2> & points, const Point2 & sensor_xy,
  double inflation, double max_ahead, double max_behind) const
{
  std::vector<uint8_t> mask(points.size(), 0);
  if (points.empty()) {
    return mask;
  }
  std::vector<std::size_t> idx;
  std::vector<double> s;
  std::vector<double> d;
  project(points, idx, s, d);
  const double car_s = project_s(sensor_xy);

  for (std::size_t i = 0; i < points.size(); ++i) {
    const double ds = normalize_s(s[i] - car_s);
    // 폭이 inflation보다 좁은 구간에서 경계가 뒤집혀 전부 지워지는 것을 막는다.
    const double left = std::max(d_left_[idx[i]] - inflation, 1e-3);
    const double right = std::max(d_right_[idx[i]] - inflation, 1e-3);
    mask[i] = (d[i] < left && d[i] > -right && ds < max_ahead && ds > -max_behind) ? 1 : 0;
  }
  return mask;
}

}  // namespace perception_pkg
