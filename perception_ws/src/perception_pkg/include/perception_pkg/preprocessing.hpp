/*
 * 스캔 전처리 — 책임 분리
 * --------------------------------
 * preprocess_scan()은 순수하게 "센서 프레임 기준 필터링 + xy 변환"만 담당한다.
 * laser -> target(map) frame 변환은 transform_points()가, 경계 폴리곤 필터는
 * polygon_mask()가 따로 맡는다. 이렇게 나눠야 boundary_polygon을 map frame에
 * 딱 한 번만 정의하면 되고, 차량이 어디에 있든 항상 올바르게 동작한다.
 * (laser frame 고정 폴리곤은 차량이 움직이면 의미 자체가 성립하지 않는다.)
 */
#ifndef PERCEPTION_PKG__PREPROCESSING_HPP_
#define PERCEPTION_PKG__PREPROCESSING_HPP_

#include <array>
#include <cstdint>
#include <vector>

#include "perception_pkg/types.hpp"

namespace perception_pkg
{

/// 행 우선(row-major) 2x2 회전행렬: {r00, r01, r10, r11}
using Rot2 = std::array<double, 4>;

struct ScanPoints
{
  /// (N) 유효 포인트 [x, y] — laser frame 기준
  std::vector<Point2> points;
  /// (N) 각 포인트의 **센서 원점 기준** 거리 [m].
  /// adaptive 클러스터링 임계값이 이 값을 쓴다. points는 이후 map frame으로
  /// 옮겨지므로 |point|로는 복원할 수 없어 따로 들고 다닌다.
  std::vector<double> ranges;
};

/// LaserScan.ranges -> 필터링된 (x, y) 포인트 (laser frame 기준).
///
/// median_kernel : 스파이크 노이즈 제거용 median filter 커널 크기 (홀수, >=3일 때만 적용)
/// fov_min/max   : 관심 각도 범위 [rad]
ScanPoints preprocess_scan(
  const std::vector<float> & ranges,
  double angle_min, double angle_increment,
  double range_min, double range_max,
  double fov_min, double fov_max,
  int median_kernel);

/// 가장자리는 그대로 두고 내부 구간만 median filter 적용.
/// 윈도 안의 비유한값(inf = 미반사, nan)은 제외하고 중앙값을 낸다.
/// 윈도가 전부 비유한값이면 원본을 유지한다.
std::vector<float> median_filter_1d(const std::vector<float> & arr, int kernel);

/// 쿼터니언에서 2D(z축 회전만) 회전행렬을 추출.
/// 라이다/차량은 평면 위에서만 회전하므로 yaw만 뽑아 쓰면 충분하다.
Rot2 rotation_matrix_2d_from_quaternion(double qx, double qy, double qz, double qw);

/// points를 (translation, rotation)으로 정의된 변환에 따라 다른 좌표계로 옮긴다
/// (예: laser frame -> map frame). 제자리(in-place) 변환.
void transform_points(std::vector<Point2> & points, double tx, double ty, const Rot2 & rot);

/// polygon(points와 같은 frame) 내부에 있는 포인트의 마스크.
///
/// **마스크를 돌려주는 이유**: 호출부는 points와 point_ranges를 1:1로 유지해야
/// 한다. points만 잘라 돌려주면 클러스터링이 길이 불일치로 죽는다.
/// 폴리곤이 비었거나 부적합하면 전부 통과(1)시킨다.
std::vector<uint8_t> polygon_mask(
  const std::vector<Point2> & points, const std::vector<Point2> & polygon);

/// mask가 0인 원소를 points/ranges에서 함께 제거한다 (1:1 대응 유지).
void apply_mask(
  std::vector<Point2> & points, std::vector<double> & ranges,
  const std::vector<uint8_t> & mask);

}  // namespace perception_pkg

#endif  // PERCEPTION_PKG__PREPROCESSING_HPP_
