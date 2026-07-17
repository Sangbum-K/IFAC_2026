"""두 local planner의 공통 기반.

입출력 인터페이스 (ROS 그래프와 1:1 대응, 실차 포팅 시 그대로 유지):
  입력  ego state : frenet odom (s, d, vs) + map pose   <- frenet_frame_node
  입력  obstacles : Obstacle.msg 필드의 dict 리스트      <- perception/tracking
                    (s_center, d_center, vs, vd, s_start, s_end, is_static)
  출력  local wpnts: Wpnt.msg 필드의 dict-of-arrays 80개  -> crazy_controller
                    (id, s_m, d_m, x_m, y_m, d_right, d_left,
                     psi_rad, kappa_radpm, vx_mps, ax_mps2)
"""

import numpy as np

G = 9.81


class LocalPlannerBase:
    def __init__(self, frenet_frame, params):
        self.ff = frenet_frame
        self.veh = params["vehicle"]
        self.lp = params["local_planner"]
        self.ds_out = self.lp["output_ds_m"]
        self.n_out = self.lp["output_num_waypoints"]
        self.horizon = self.lp["planning_horizon_m"]
        # planner 내부 s 그리드 간격: 출력보다 성기게 (계산량 절약, NX 예산)
        self.ds_plan = 0.25
        self.n_plan = int(self.horizon / self.ds_plan)
        self.last_plan = None  # 직전 계획 (hysteresis/채터링 억제용)

    # ------------------------------------------------------------- 충돌 판정
    def collision_margins(self, v_ego):
        """속도 의존 종방향 마진. 빠를수록 길게 잡는다."""
        s_m = (self.veh["length_m"] + self.lp["safety_s_base_m"]
               + self.lp["safety_s_gain_s"] * v_ego)
        d_m = self.veh["width_m"] + self.lp["safety_d_m"]
        return s_m, d_m

    def check_collision_st(self, s_path, d_path, t_path, obstacles, v_ego):
        """시공간 충돌 체크 (관측 불확실성 반영).

        경로 위 각 점의 도달 시각 t_path에서 상대(등속 예측)의 s를 구해
        s-d 사각형 마진과 겹치면 충돌. 반환: (충돌여부, 최소 clearance).
        clearance는 마진 대비 비율(1=꼭 마진만큼, 클수록 안전)로,
        비용함수의 w_safety 항에 들어간다.

        마진은 Obstacle.msg의 분산 필드(s_var/d_var/vs_var)만큼 커진다:
          종방향 sigma(t) = sqrt(s_var + vs_var * t^2)  — 등속 외삽 시
          속도 추정 오차가 t에 비례해 누적되는 것을 반영. 먼 미래의
          예측일수록 덜 믿는다는 뜻이고, 이게 없으면 노이즈 낀 관측을
          t_horizons 끝(3.5s)까지 그대로 신뢰해 위치 오차 ~1.4m를
          마진 없이 지나치게 된다. perception이 분산을 안 주면(=키 없음)
          마진 확장은 0으로, 기존 동작과 동일.
        """
        s_margin, d_margin = self.collision_margins(v_ego)
        k_sig = self.lp["uncertainty_gain"]
        sig_cap = self.lp["uncertainty_margin_max_m"]
        min_clear = np.inf
        for ob in obstacles:
            s_obs_t = ob["s_center"] + ob["vs"] * t_path      # 등속 예측
            d_obs_t = ob["d_center"] + ob["vd"] * t_path
            sig_s = np.sqrt(ob.get("s_var", 0.0)
                            + ob.get("vs_var", 0.0) * t_path ** 2)
            sig_d = np.sqrt(ob.get("d_var", 0.0))
            s_m_t = s_margin + np.minimum(k_sig * sig_s, sig_cap)
            d_m_t = d_margin + min(k_sig * sig_d, sig_cap)
            ds = np.abs(self.ff.s_diff(s_path, s_obs_t))
            dd = np.abs(d_path - d_obs_t)
            # 마진으로 정규화한 시공간 거리 (max-norm)
            clear = np.max(np.stack([ds / s_m_t, dd / d_m_t]), axis=0)
            min_clear = min(min_clear, float(clear.min()))
            if min_clear < 1.0:
                return True, min_clear
        return False, min_clear

    def _bound_need(self):
        """경계에서 확보해야 하는 최소 거리 = 차 반폭 + 추가 여유.

        주의: raceline CSV의 d_left/d_right는 생성 단계에서 이미
        margin(기본 0.15m)이 차감된 값이라, 여기 track_margin_m은
        '그 위에 더 얹는' 여유다. 과하게 잡으면 좁은 구간에서
        회피 후보가 전멸하고 follow 모드에 갇힌다.
        """
        return 0.5 * self.veh["width_m"] + self.lp["track_margin_m"]

    def check_track_bounds(self, s_path, d_path):
        """트랙 경계 침범 체크 (차 반폭 + 여유 포함). True면 위반."""
        i = self.ff.idx_of_s(s_path)
        need = self._bound_need()
        left_ok = (self.ff.d_left[i] - d_path) >= need    # +d = 왼쪽으로 이동
        right_ok = (self.ff.d_right[i] + d_path) >= need
        return not bool(np.all(left_ok & right_ok))

    def available_corridor(self, s_grid):
        """호라이즌 창에서 좌/우로 실제 쓸 수 있는 최대 offset.

        후보 offset을 절대값으로 두면 좁은 트랙에서 후보가 전부
        경계 위반으로 죽는다. 대신 '창 내 최소 가용폭'을 재서
        후보를 그 비율로 깐다 (트랙 폭 적응형 lattice).
        반환: (avail_left>=0, avail_right>=0) [m]
        """
        i = self.ff.idx_of_s(s_grid)
        need = self._bound_need()
        avail_l = max(float(np.min(self.ff.d_left[i])) - need, 0.0)
        avail_r = max(float(np.min(self.ff.d_right[i])) - need, 0.0)
        return avail_l, avail_r

    # ------------------------------------------------------ 속도 재프로파일
    def velocity_profile_on_path(self, kappa, ds, v0, v_cap):
        """회피 경로용 속도 프로파일 (raceline과 같은 friction-circle 방식).

        v0(현재 속도)에서 출발하는 forward pass + 감속 backward pass.
        v_cap: 상한 (raceline vx나 추종 속도 등 상황별 상한을 넘겨받음).
        """
        a_lat_max = self.veh["mu"] * G
        n = len(kappa)
        v = np.minimum(np.sqrt(a_lat_max / np.maximum(np.abs(kappa), 1e-9)),
                       v_cap)
        v[0] = min(v[0], max(v0, 0.1))
        for i in range(n - 1):  # forward (가속 한계)
            a_lat = v[i] ** 2 * abs(kappa[i])
            a_ok = self.veh["a_acc_max_mps2"] * np.sqrt(
                max(0.0, 1.0 - (a_lat / a_lat_max) ** 2))
            v[i + 1] = min(v[i + 1], np.sqrt(v[i] ** 2 + 2 * a_ok * ds))
        for i in range(n - 2, -1, -1):  # backward (감속 한계)
            a_lat = v[i + 1] ** 2 * abs(kappa[i + 1])
            a_ok = self.veh["a_dec_max_mps2"] * np.sqrt(
                max(0.0, 1.0 - (a_lat / a_lat_max) ** 2))
            v[i] = min(v[i], np.sqrt(v[i + 1] ** 2 + 2 * a_ok * ds))
        return v

    def time_along_path(self, v, ds):
        """속도 프로파일 -> 각 점 도달 시각 (사다리꼴 적분)."""
        v_mid = 0.5 * (v[:-1] + v[1:])
        return np.concatenate([[0.0], np.cumsum(ds / np.maximum(v_mid, 0.1))])

    # ------------------------------------------------------------ 출력 포맷
    def format_output(self, s_grid, d_profile, v_profile):
        """planner 내부 그리드 -> controller용 80 x ds=0.1 윈도우로 리샘플.

        local_trajectory_node 출력과 동일 규격이라 crazy_controller를
        수정 없이 그대로 물릴 수 있다.
        """
        s_rel = np.arange(self.n_out) * self.ds_out          # 0 ~ 7.9 m
        s_rel_grid = np.arange(len(s_grid)) * (s_grid[1] - s_grid[0]) \
            if len(s_grid) > 1 else np.zeros(1)
        d_out = np.interp(s_rel, s_rel_grid, d_profile)
        v_out = np.interp(s_rel, s_rel_grid, v_profile)
        s_out = self.ff.wrap_s(s_grid[0] + s_rel)
        fields = self.ff.path_from_d_profile(s_out, d_out)
        ax_out = np.gradient(v_out ** 2) / (2 * self.ds_out)  # v dv/ds
        fields.update({
            "id": np.arange(self.n_out),
            "vx_mps": v_out,
            "ax_mps2": ax_out,
        })
        return fields
