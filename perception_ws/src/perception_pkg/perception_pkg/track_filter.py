"""
트랙 경계(Frenet) 기반 스캔 포인트 필터 — ForzaETH race_stack의
`Detect::laserPointOnTrack()` (perception/src/detect.cpp) 이식.

## 왜 맵 EDT 필터만으로 부족한가
`obstacle_detector_node.wall_point_mask()`는 "맵 벽까지의 거리"만 본다.
그런데 스캔-맵 정합 오차(run_0805 실측 p99 0.70m, max 1.20m) 때문에 벽 리턴이
문턱 바로 바깥에 남는 fringe가 항상 생기고, 문턱을 올리면 코리도(주행 가능
폭 0.96~2.42m, 중앙값 1.36m)를 통째로 먹는다. 즉 벽거리 하나로는 못 끝낸다.

ForzaETH는 여기에 **직교하는 두 번째 기준**을 쓴다: 포인트를 레이스라인 기준
Frenet (s, d)로 바꿔서

  1) `d`가 그 지점의 트랙 폭(d_left/d_right, 웨이포인트마다 다름) 밖이면 버린다.
     - 벽거리와 달리 "주행 코리도 안이냐"를 직접 묻는다. 코리도는 맵 벽보다
       매끄러워서 정합 오차에 덜 민감하다.
  2) `s`가 자차보다 `max_ahead`m 넘게 앞이면 버린다 (ForzaETH
     `max_viewing_distance`).
     - 이게 이 트랙(29.6m 루프, 11x9m 실내)에서 특히 크게 먹힌다. 루프 반대편
       벽은 **유클리드 거리로는 가깝지만**(range_max 12m 안에 다 들어온다)
       s로는 반 바퀴 밖이다. 단순 range_max 축소로는 못 자르는 걸 s 게이트가
       자른다 — rosbag_0801 재생에서 레이스라인을 막는 유령의 s는 p50 8.0m /
       p90 11.3m로, 실효 인지거리(~4.5m) 밖에 몰려 있었다.

`/global_waypoints`를 못 받은 동안은 필터를 적용하지 않는다(fail-open).
여기서 스캔을 버리면 장애물 스트림이 끊겨 spliner가 fail-safe 정지한다.
"""
import numpy as np


class TrackBounds:
    """`/global_waypoints`(WpntArray)에서 만든 Frenet 투영 + 트랙 폭 테이블."""

    __slots__ = ('x', 'y', 's', 'd_left', 'd_right', 'psi', 'length', 'n')

    def __init__(self, wpnts):
        n = len(wpnts)
        if n < 3:
            raise ValueError('need at least 3 waypoints')
        self.x = np.fromiter((w.x_m for w in wpnts), dtype=np.float64, count=n)
        self.y = np.fromiter((w.y_m for w in wpnts), dtype=np.float64, count=n)
        self.s = np.fromiter((w.s_m for w in wpnts), dtype=np.float64, count=n)
        self.d_left = np.fromiter((w.d_left for w in wpnts),
                                  dtype=np.float64, count=n)
        self.d_right = np.fromiter((w.d_right for w in wpnts),
                                   dtype=np.float64, count=n)
        self.psi = np.fromiter((w.psi_rad for w in wpnts),
                               dtype=np.float64, count=n)
        arrays = (self.x, self.y, self.s, self.d_left, self.d_right, self.psi)
        if not all(np.all(np.isfinite(a)) for a in arrays):
            raise ValueError('waypoints contain non-finite values')
        # s는 단조증가여야 normalize_s(둘레 wrap)가 성립한다.
        if not np.all(np.diff(self.s) > 0.0):
            raise ValueError('waypoint s_m must be strictly increasing')
        # 폭이 0이면 필터가 모든 포인트를 지운다 -> 장애물 실명.
        if np.any(self.d_left <= 0.0) or np.any(self.d_right <= 0.0):
            raise ValueError('waypoint d_left/d_right must be positive')
        self.length = float(self.s[-1])
        if not self.length > 0.0:
            raise ValueError('track length must be positive')
        self.n = n

    def project(self, points):
        """(N,2) map 좌표 -> (최근접 웨이포인트 idx, s, d).

        d는 진행방향 기준 **왼쪽이 양수** (ETH 관례, d_left와 같은 부호).
        """
        dx = points[:, 0][:, None] - self.x[None, :]
        dy = points[:, 1][:, None] - self.y[None, :]
        idx = np.argmin(dx * dx + dy * dy, axis=1)
        ex, ey = dx[np.arange(len(points)), idx], dy[np.arange(len(points)), idx]
        psi = self.psi[idx]
        c, s = np.cos(psi), np.sin(psi)
        d = -s * ex + c * ey
        s_along = c * ex + s * ey
        return idx, self.s[idx] + s_along, d

    def normalize_s(self, ds):
        """둘레 wrap을 고려한 [-L/2, L/2) 구간 s 차이."""
        ds = np.mod(ds, self.length)
        return np.where(ds > self.length * 0.5, ds - self.length, ds)

    def on_track_mask(self, points, sensor_xy, inflation,
                      max_ahead, max_behind):
        """트랙 코리도 안 + 자차 전방 시야창 안인 포인트의 bool 마스크.

        Parameters
        ----------
        points     : (N, 2) map 프레임 좌표
        sensor_xy  : (2,) 자차(라이다) map 좌표 — s 원점
        inflation  : 트랙 경계를 안쪽으로 줄이는 여유 [m]. 경계 근처 벽 리턴을
                     확실히 빼기 위한 것 (ForzaETH `boundaries_inflation`).
        max_ahead  : 자차보다 이만큼 앞(s)까지만 본다 [m]
        max_behind : 자차보다 이만큼 뒤(s)까지만 본다 [m]
        """
        if len(points) == 0:
            return np.zeros(0, dtype=bool)
        idx, s, d = self.project(points)
        car_s = self.project(np.asarray(sensor_xy, dtype=np.float64)
                             .reshape(1, 2))[1][0]
        ds = self.normalize_s(s - car_s)
        # 폭이 inflation보다 좁은 구간에서 경계가 뒤집혀 전부 지워지는 것을 막는다.
        left = np.maximum(self.d_left[idx] - inflation, 1e-3)
        right = np.maximum(self.d_right[idx] - inflation, 1e-3)
        return ((d < left) & (d > -right)
                & (ds < max_ahead) & (ds > -max_behind))
