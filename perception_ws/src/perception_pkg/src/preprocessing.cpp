#include "perception_pkg/preprocessing.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>

namespace perception_pkg
{

std::vector<float> median_filter_1d(const std::vector<float> & arr, int kernel)
{
  std::vector<float> out = arr;
  const int n = static_cast<int>(arr.size());
  const int half = kernel / 2;
  if (half < 1 || n <= 2 * half) {
    return out;
  }

  // 윈도 하나당 유한값만 모아 중앙값을 낸다. 원소 수가 짝수면 가운데 두 값의
  // 평균 (numpy median과 동일).
  std::vector<float> win;
  win.reserve(static_cast<std::size_t>(kernel));
  for (int i = 0; i + kernel <= n; ++i) {
    win.clear();
    for (int k = 0; k < kernel; ++k) {
      const float v = arr[static_cast<std::size_t>(i + k)];
      if (std::isfinite(v)) {
        win.push_back(v);
      }
    }
    if (win.empty()) {
      continue;   // 전부 비유한값인 윈도는 원본을 유지한다
    }
    const std::size_t m = win.size();
    std::nth_element(win.begin(), win.begin() + static_cast<long>(m / 2), win.end());
    float median = win[m / 2];
    if (m % 2 == 0) {
      // 아래쪽 중앙값 = 앞 절반의 최대값
      const float lower = *std::max_element(win.begin(), win.begin() + static_cast<long>(m / 2));
      median = 0.5f * (lower + median);
    }
    out[static_cast<std::size_t>(i + half)] = median;
  }
  return out;
}

ScanPoints preprocess_scan(
  const std::vector<float> & ranges,
  double angle_min, double angle_increment,
  double range_min, double range_max,
  double fov_min, double fov_max,
  int median_kernel)
{
  const std::size_t n = ranges.size();

  // 1) median filter로 스파이크(순간 튀는 값) 제거
  const bool use_median =
    median_kernel >= 3 && median_kernel % 2 == 1 &&
    n > static_cast<std::size_t>(median_kernel);
  std::vector<float> smoothed;
  if (use_median) {
    smoothed = median_filter_1d(ranges, median_kernel);
  }
  const std::vector<float> & src = use_median ? smoothed : ranges;

  ScanPoints out;
  out.points.reserve(n);
  out.ranges.reserve(n);

  // 2) range / FOV 유효성 마스크
  for (std::size_t i = 0; i < n; ++i) {
    const double r = static_cast<double>(src[i]);
    if (!std::isfinite(r) || r < range_min || r > range_max) {
      continue;
    }
    const double a = angle_min + static_cast<double>(i) * angle_increment;
    if (a < fov_min || a > fov_max) {
      continue;
    }
    out.points.push_back({{r * std::cos(a), r * std::sin(a)}});
    out.ranges.push_back(r);
  }
  return out;
}

Rot2 rotation_matrix_2d_from_quaternion(double qx, double qy, double qz, double qw)
{
  const double yaw = std::atan2(
    2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz));
  const double c = std::cos(yaw);
  const double s = std::sin(yaw);
  return Rot2{{c, -s, s, c}};
}

void transform_points(std::vector<Point2> & points, double tx, double ty, const Rot2 & rot)
{
  for (auto & p : points) {
    const double x = rot[0] * p[0] + rot[1] * p[1] + tx;
    const double y = rot[2] * p[0] + rot[3] * p[1] + ty;
    p[0] = x;
    p[1] = y;
  }
}

std::vector<uint8_t> polygon_mask(
  const std::vector<Point2> & points, const std::vector<Point2> & polygon)
{
  std::vector<uint8_t> mask(points.size(), 1);
  if (points.empty() || polygon.size() < 3) {
    return mask;   // 폴리곤이 없거나 부적합하면 필터 미적용
  }

  // Ray casting 기반 point-in-polygon 테스트.
  std::fill(mask.begin(), mask.end(), 0);
  const std::size_t m = polygon.size();
  for (std::size_t e = 0; e < m; ++e) {
    const double ax = polygon[e][0];
    const double ay = polygon[e][1];
    const double bx = polygon[(e + 1) % m][0];
    const double by = polygon[(e + 1) % m][1];
    const double denom = (by - ay) + 1e-12;
    for (std::size_t i = 0; i < points.size(); ++i) {
      const double x = points[i][0];
      const double y = points[i][1];
      if ((ay > y) != (by > y)) {
        const double x_int = (bx - ax) * (y - ay) / denom + ax;
        if (x < x_int) {
          mask[i] ^= 1u;
        }
      }
    }
  }
  return mask;
}

void apply_mask(
  std::vector<Point2> & points, std::vector<double> & ranges,
  const std::vector<uint8_t> & mask)
{
  std::size_t out = 0;
  const std::size_t n = std::min(points.size(), mask.size());
  for (std::size_t i = 0; i < n; ++i) {
    if (mask[i]) {
      points[out] = points[i];
      ranges[out] = ranges[i];
      ++out;
    }
  }
  points.resize(out);
  ranges.resize(out);
}

}  // namespace perception_pkg
