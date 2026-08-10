/*
 * 트랙 경계(Frenet) 기반 스캔 포인트 필터 — ForzaETH race_stack의
 * `Detect::laserPointOnTrack()` (perception/src/detect.cpp) 이식.
 *
 * ## 왜 맵 EDT 필터만으로 부족한가
 * 벽거리 필터는 "맵 벽까지의 거리"만 본다. 그런데 스캔-맵 정합 오차(run_0805
 * 실측 p99 0.70m, max 1.20m) 때문에 벽 리턴이 문턱 바로 바깥에 남는 fringe가
 * 항상 생기고, 문턱을 올리면 코리도(주행 가능 폭 0.96~2.42m, 중앙값 1.36m)를
 * 통째로 먹는다. 즉 벽거리 하나로는 못 끝낸다.
 *
 * ForzaETH는 여기에 **직교하는 두 번째 기준**을 쓴다: 포인트를 레이스라인 기준
 * Frenet (s, d)로 바꿔서
 *
 *   1) `d`가 그 지점의 트랙 폭(d_left/d_right, 웨이포인트마다 다름) 밖이면 버린다.
 *      - 벽거리와 달리 "주행 코리도 안이냐"를 직접 묻는다. 코리도는 맵 벽보다
 *        매끄러워서 정합 오차에 덜 민감하다.
 *   2) `s`가 자차보다 `max_ahead`m 넘게 앞이면 버린다 (ForzaETH
 *      `max_viewing_distance`).
 *      - 이게 이 트랙(29.6m 루프, 11x9m 실내)에서 특히 크게 먹힌다. 루프 반대편
 *        벽은 **유클리드 거리로는 가깝지만**(range_max 12m 안에 다 들어온다)
 *        s로는 반 바퀴 밖이다. 단순 range_max 축소로는 못 자르는 걸 s 게이트가
 *        자른다 — rosbag_0801 재생에서 레이스라인을 막는 유령의 s는 p50 8.0m /
 *        p90 11.3m로, 실효 인지거리(~4.5m) 밖에 몰려 있었다.
 *
 * `/global_waypoints`를 못 받은 동안은 필터를 적용하지 않는다(fail-open).
 * 여기서 스캔을 버리면 장애물 스트림이 끊겨 spliner가 fail-safe 정지한다.
 */
#ifndef PERCEPTION_PKG__TRACK_FILTER_HPP_
#define PERCEPTION_PKG__TRACK_FILTER_HPP_

#include <cstdint>
#include <vector>

#include "perception_pkg/types.hpp"

namespace perception_pkg
{

/// `/global_waypoints`(WpntArray)의 한 점에서 필터가 실제로 쓰는 항목만 추린 것.
/// 메시지 타입 의존을 노드 쪽에만 두기 위해 평범한 구조체로 받는다.
struct Waypoint
{
  double x;
  double y;
  double s;
  double d_left;
  double d_right;
  double psi;
};

/// 전역 레이스라인에서 만든 Frenet 투영 + 트랙 폭 테이블.
class TrackBounds
{
public:
  /// 부적합한 웨이포인트(3점 미만 / 비유한값 / s 비단조 / 폭 <= 0 / 길이 <= 0)이면
  /// std::invalid_argument를 던진다.
  explicit TrackBounds(const std::vector<Waypoint> & wpnts);

  /// (N,2) map 좌표 -> (최근접 웨이포인트 idx, s, d).
  /// d는 진행방향 기준 **왼쪽이 양수** (ETH 관례, d_left와 같은 부호).
  void project(
    const std::vector<Point2> & points,
    std::vector<std::size_t> & idx_out,
    std::vector<double> & s_out,
    std::vector<double> & d_out) const;

  /// 점 하나에 대한 s만 (자차 위치 투영용).
  double project_s(const Point2 & point) const;

  /// 둘레 wrap을 고려한 [-L/2, L/2) 구간 s 차이.
  double normalize_s(double ds) const;

  /// 트랙 코리도 안 + 자차 전방 시야창 안인 포인트의 마스크.
  ///
  /// sensor_xy  : 자차(라이다) map 좌표 — s 원점
  /// inflation  : 트랙 경계를 안쪽으로 줄이는 여유 [m] (ForzaETH boundaries_inflation)
  /// max_ahead  : 자차보다 이만큼 앞(s)까지만 본다 [m]
  /// max_behind : 자차보다 이만큼 뒤(s)까지만 본다 [m]
  std::vector<uint8_t> on_track_mask(
    const std::vector<Point2> & points, const Point2 & sensor_xy,
    double inflation, double max_ahead, double max_behind) const;

  std::size_t size() const {return x_.size();}
  double length() const {return length_;}

private:
  std::vector<double> x_, y_, s_, d_left_, d_right_, psi_;
  double length_{0.0};
};

}  // namespace perception_pkg

#endif  // PERCEPTION_PKG__TRACK_FILTER_HPP_
