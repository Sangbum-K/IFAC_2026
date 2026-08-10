// 인지 파이프라인 공용 기본 타입.
#ifndef PERCEPTION_PKG__TYPES_HPP_
#define PERCEPTION_PKG__TYPES_HPP_

#include <array>

namespace perception_pkg
{

/// 2D 좌표 [x, y]. 프레임은 사용처가 정한다(laser 또는 target_frame).
using Point2 = std::array<double, 2>;

}  // namespace perception_pkg

#endif  // PERCEPTION_PKG__TYPES_HPP_
