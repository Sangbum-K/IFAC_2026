"""Frenet frame utilities.

frenet_frame_node.cpp의 Cartesian<->Frenet 변환을 미러링하고,
d(s) 프로파일로 주어진 회피 경로를 controller가 소비하는 전체 필드
(x, y, psi, kappa, d_left, d_right)로 복원하는 유틸을 제공한다.

좌표 규약 (Wpnt.msg와 동일):
  s: raceline 진행 방향 호길이 [m], 트랙 길이로 wrap
  d: raceline 기준 횡 오프셋 [m], 진행 방향 왼쪽이 +
"""

import numpy as np
from scipy.spatial import cKDTree


class FrenetFrame:
    def __init__(self, traj):
        """traj: main.py의 build_trajectory 출력 (raceline) dict."""
        self.xy = traj["xy"]
        self.s = traj["s"]
        self.psi = traj["psi"]
        self.kappa = traj["kappa"]
        self.vx = traj["vx"]
        self.d_left = traj["d_left"]
        self.d_right = traj["d_right"]
        self.n = len(self.s)
        seg_last = float(np.hypot(*(self.xy[0] - self.xy[-1])))
        self.track_len = float(self.s[-1]) + seg_last
        self.tree = cKDTree(self.xy)
        # 진행 방향 왼쪽(+d) 단위 법선
        self.normal = np.column_stack([-np.sin(self.psi), np.cos(self.psi)])

    # ------------------------------------------------------------------ s 유틸
    def wrap_s(self, s):
        return np.mod(s, self.track_len)

    def s_diff(self, s_a, s_b):
        """s_a - s_b를 트랙 wrap 고려해서 [-L/2, L/2)로 반환 (앞이면 +)."""
        return (s_a - s_b + 0.5 * self.track_len) % self.track_len \
            - 0.5 * self.track_len

    def idx_of_s(self, s):
        """s -> 가장 가까운 raceline 인덱스 (균일 간격 가정, ds=0.1)."""
        ds = self.track_len / self.n
        return np.mod(np.round(np.asarray(s) / ds).astype(int), self.n)

    # ------------------------------------------------------- cart <-> frenet
    def cart2frenet(self, xy_pos):
        """(x, y) -> (s, d). frenet_frame_node의 최근접+접선투영과 동일 방식."""
        _, i = self.tree.query(xy_pos)
        t_hat = np.array([np.cos(self.psi[i]), np.sin(self.psi[i])])
        e = np.asarray(xy_pos) - self.xy[i]
        s = self.wrap_s(self.s[i] + float(e @ t_hat))
        d = float(t_hat[0] * e[1] - t_hat[1] * e[0])  # cross(t, e): 왼쪽 +
        return s, d

    def frenet2cart(self, s, d):
        """(s, d) 배열 -> (N,2) map 좌표. raceline 점 + d*법선."""
        i = self.idx_of_s(s)
        return self.xy[i] + np.asarray(d)[:, None] * self.normal[i]

    # ------------------------------------------------- 회피 경로 필드 복원
    def path_from_d_profile(self, s_grid, d_profile):
        """s_grid 위의 d(s) 프로파일 -> controller가 쓰는 전체 필드 재계산.

        반환 dict 키는 Wpnt.msg 필드명과 동일 (id, ax_mps2 제외).
        psi/kappa는 offset 경로 기하에서 수치 미분으로 다시 구한다
        (controller가 feedforward로 kappa_radpm, psi_rad를 사용하므로 필수).
        """
        i = self.idx_of_s(s_grid)
        xy = self.xy[i] + d_profile[:, None] * self.normal[i]
        dx = np.gradient(xy[:, 0])
        dy = np.gradient(xy[:, 1])
        psi = np.arctan2(dy, dx)
        dl = np.hypot(dx, dy)
        # 헤딩 변화율 -> 곡률 (unwrap으로 ±pi 점프 방지)
        kappa = np.gradient(np.unwrap(psi)) / np.maximum(dl, 1e-9)
        # 트랙 경계 여유: +d로 이동하면 왼쪽 여유는 줄고 오른쪽 여유는 는다
        return {
            "s_m": self.wrap_s(s_grid),
            "d_m": d_profile,
            "x_m": xy[:, 0],
            "y_m": xy[:, 1],
            "d_left": self.d_left[i] - d_profile,
            "d_right": self.d_right[i] + d_profile,
            "psi_rad": psi,
            "kappa_radpm": kappa,
        }

    def raceline_v_at(self, s):
        return self.vx[self.idx_of_s(s)]
