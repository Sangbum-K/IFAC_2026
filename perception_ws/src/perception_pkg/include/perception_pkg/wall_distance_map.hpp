// /map 에서 만든 셀별 최근접 벽거리 테이블과 그 위의 레이캐스트.
//
// obstacle_detector_node.cpp 안에 있던 것을 헤더로 뺐다 — raycast() 가
// sphere tracing 이라 경계 조건(맵 밖·벽 셀 위·resolution 0)이 여럿이고,
// 그 하나하나가 레이 일관성 필터의 오검출로 직결되므로 단위 테스트가 필요하다.
#ifndef PERCEPTION_PKG__WALL_DISTANCE_MAP_HPP_
#define PERCEPTION_PKG__WALL_DISTANCE_MAP_HPP_

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <vector>

namespace perception_pkg
{

/// /map에서 만든 셀별 최근접 벽거리 테이블. 한 번 만들면 불변이라
/// shared_ptr로 통째로 교체해 on_scan과 cb_map 사이의 경합을 없앤다.
struct WallDistanceMap
{
  std::vector<float> dist;   ///< row-major, [m]
  int width{0};
  int height{0};
  double origin_x{0.0};
  double origin_y{0.0};
  double origin_yaw{0.0};
  double resolution{0.0};

  /// 셀 인덱스를 벗어나면 false. 맵 밖 포인트는 "열린 공간"으로 취급하지 않는다.
  bool distance_at(double x, double y, double & out) const
  {
    const double dx = x - origin_x;
    const double dy = y - origin_y;
    const double c = std::cos(origin_yaw);
    const double s = std::sin(origin_yaw);
    const long col = static_cast<long>(std::floor((c * dx + s * dy) / resolution));
    const long row = static_cast<long>(std::floor((-s * dx + c * dy) / resolution));
    if (row < 0 || row >= height || col < 0 || col >= width) {
      return false;
    }
    out = static_cast<double>(dist[static_cast<std::size_t>(row) * static_cast<std::size_t>(width) +
        static_cast<std::size_t>(col)]);
    return true;
  }

  /// (x0,y0)에서 단위벡터 (ux,uy) 방향 첫 벽까지 거리 [m].
  ///
  /// 점유격자를 따로 들고 다니지 않고 EDT 위에서 sphere tracing 을 한다:
  /// dist 는 "가장 가까운 벽까지"이므로 그만큼 전진해도 **어느 방향으로도**
  /// 벽을 지나칠 수 없다. 열린 공간에서는 한 걸음이 크고 벽에 가까워질수록
  /// 잘게 줄어들어, 고정 간격 행진(8m/2.5cm = 320회)보다 훨씬 적은 반복으로
  /// 같은 답에 수렴한다 (실측 열린 구간 10~30회).
  ///
  /// 맵 밖으로 나가면 max_range 를 돌려준다 — "벽이 없다"가 아니라 "모른다"이고,
  /// 호출부(레이 일관성 필터)는 max_range 를 받으면 그 빔을 건드리지 않는다.
  double raycast(double x0, double y0, double ux, double uy, double max_range) const
  {
    if (!(resolution > 0.0)) {
      return max_range;
    }
    // 한 셀 안쪽까지 오면 벽에 닿은 것으로 본다. 같은 값을 전진 하한으로도
    // 써서 dist==0(벽 셀 위) 일 때 제자리걸음하는 경우를 막는다.
    const double hit_eps = resolution;
    const int max_steps = static_cast<int>(max_range / resolution) + 8;
    double t = 0.0;
    for (int i = 0; i < max_steps && t < max_range; ++i) {
      double d = 0.0;
      if (!distance_at(x0 + ux * t, y0 + uy * t, d)) {
        return max_range;                    // 맵 밖 — 판단하지 않는다
      }
      if (d <= hit_eps) {
        return t;
      }
      t += std::max(d, hit_eps);
    }
    return max_range;
  }
};

}  // namespace perception_pkg

#endif  // PERCEPTION_PKG__WALL_DISTANCE_MAP_HPP_
