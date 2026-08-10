/*
 * 정확 유클리드 거리변환(EDT) — scipy.ndimage.distance_transform_edt 대체.
 *
 * 파이썬 판은 scipy에 의존했고, scipy가 없는 환경에서는 벽 필터가 통째로
 * 꺼졌다(fail-open). C++에서는 Felzenszwalb & Huttenlocher(2012)의 분리형
 * O(N) 알고리즘을 직접 담아 그 조건부 비활성 경로 자체를 없앤다.
 * 결과값은 scipy와 동일한 **정확한** 유클리드 거리다(근사 chamfer 아님).
 */
#ifndef PERCEPTION_PKG__DISTANCE_TRANSFORM_HPP_
#define PERCEPTION_PKG__DISTANCE_TRANSFORM_HPP_

#include <cstdint>
#include <vector>

namespace perception_pkg
{

/// seed(=0 거리 기준점)까지의 유클리드 거리에 scale을 곱해 돌려준다.
///
/// seed  : (height*width) row-major 마스크. 0이 아닌 셀이 거리 0의 기준점.
///         (scipy 관례로 옮기면 distance_transform_edt(input) 에서 input==0 인 셀)
/// scale : 셀 -> 미터 환산 계수(맵 resolution).
/// 반환  : (height*width) 거리 [m]. seed가 하나도 없으면 큰 유한값으로 채운다.
///
/// **scale을 안에서 곱하는 이유**: `sqrt`를 float으로 접은 뒤 다시 곱하면
/// 반올림이 두 번 일어나 scipy(`edt * res` 를 float64로 계산 후 한 번만 축소)와
/// 마지막 비트가 갈린다. 그 1 ULP가 wall_dist_min 경계의 포인트 하나를 살리거나
/// 죽여 클러스터 분할 -> 트랙 ID까지 발산시킨다 (원본 대비 A/B 재생에서 실측).
std::vector<float> euclidean_distance_transform(
  const std::vector<uint8_t> & seed, int width, int height, double scale);

}  // namespace perception_pkg

#endif  // PERCEPTION_PKG__DISTANCE_TRANSFORM_HPP_
