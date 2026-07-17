"""상대 차량 모델 + 관측(perception) 모사.

상대는 raceline(d=0)을 speed_factor 배율 속도로 정확히 따라간다고 가정.
관측은 ae_hyu_msgs/Obstacle.msg 필드명 그대로의 dict로 나가므로,
실차에서는 이 부분만 perception/tracking 노드 출력으로 갈아끼우면 된다.
노이즈/지연 파라미터는 params.yaml의 opponent.noise.* 에서 튜닝.
"""

import collections

import numpy as np


class Opponent:
    def __init__(self, frenet_frame, params, s_start, rng):
        self.ff = frenet_frame
        self.p = params["opponent"]
        self.veh = params["vehicle"]
        self.rng = rng
        self.s = float(s_start)
        self.v = 0.0
        # 관측 지연 모사용 상태 이력 (t, s, v)
        self._hist = collections.deque(maxlen=200)

    def step(self, dt):
        """raceline 속도 * speed_factor로 s 전진 (경로 추종은 이상적이라 가정)."""
        v_ref = float(self.ff.raceline_v_at(np.array([self.s]))[0])
        self.v = v_ref * self.p["speed_factor"]
        self.s = self.ff.wrap_s(self.s + self.v * dt)

    def pose(self):
        """map 좌표 (x, y). 시각화/충돌판정용 참값."""
        return self.ff.frenet2cart(np.array([self.s]), np.array([0.0]))[0]

    def observe(self, ego_s, t_now):
        """ego 관점 관측 -> Obstacle.msg 필드 dict 리스트 (0개 or 1개).

        detection_range 밖이면 미검출(빈 리스트) — 실차 LiDAR 검출 한계 모사.
        noise.enabled면 가우시안 노이즈 + latency 적용.
        """
        self._hist.append((t_now, self.s, self.v))
        gap = abs(self.ff.s_diff(self.s, ego_s))
        if gap > self.p["detection_range_m"]:
            return []

        s_obs, v_obs = self.s, self.v
        nz = self.p["noise"]
        if nz["enabled"]:
            # latency: latency_s 전의 상태를 지금 관측한 것으로 사용
            t_delayed = t_now - nz["latency_s"]
            for th, sh, vh in self._hist:
                if th >= t_delayed:
                    s_obs, v_obs = sh, vh
                    break
            s_obs = self.ff.wrap_s(s_obs + self.rng.normal(0, nz["s_std_m"]))
            v_obs = max(v_obs + self.rng.normal(0, nz["vs_std_mps"]), 0.0)
        d_obs = (self.rng.normal(0, nz["d_std_m"])
                 if nz["enabled"] else 0.0)

        half_len = 0.5 * self.veh["length_m"]
        return [{
            "id": 0,
            "s_center": s_obs, "d_center": d_obs,
            "s_start": self.ff.wrap_s(s_obs - half_len),
            "s_end": self.ff.wrap_s(s_obs + half_len),
            "d_left": d_obs + 0.5 * self.veh["width_m"],
            "d_right": d_obs - 0.5 * self.veh["width_m"],
            "vs": v_obs, "vd": 0.0,
            # 분산 필드 (Obstacle.msg의 *_var): planner가 불확실성 비례로
            # 마진을 키우는 데 사용. 실차에선 tracking 필터의 공분산이 들어옴
            "s_var": nz["s_std_m"] ** 2 if nz["enabled"] else 0.0,
            "d_var": nz["d_std_m"] ** 2 if nz["enabled"] else 0.0,
            "vs_var": nz["vs_std_mps"] ** 2 if nz["enabled"] else 0.0,
            "is_static": False,
        }]
