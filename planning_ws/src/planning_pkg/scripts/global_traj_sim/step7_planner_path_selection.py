"""방식 1: Path Selection (lattice) local planner.

원리:
  raceline을 기준으로 좌우 d-offset 후보 경로 N개를 깔고(quintic 전환 후
  offset 유지), 트랙 경계/시공간 충돌로 걸러낸 뒤 비용 최소 후보를 선택.

장점: 계산 싸고 결정적, 상대 추정 노이즈에 강건 (Jetson NX에 부담 없음)
단점: 해상도가 후보 개수에 묶임, hysteresis 없으면 후보 간 진동

튜닝 순서 권장:
  1. transition_length_m  — 회피가 굼뜨면 줄이고, 조향 포화(서보 한계)면 늘림
  2. w_safety / safety_d_m — 접촉 위험 조절
  3. w_switch             — 좌우 후보 왔다갔다(채터링)하면 올림
  4. w_deviation          — 추월 후 라인 복귀가 느리면 올림
"""

import numpy as np

from step6_planner_base import LocalPlannerBase


def quintic_blend(x):
    """0->1 quintic smoothstep (양끝 1,2차 미분 0). x는 [0,1] 클립."""
    x = np.clip(x, 0.0, 1.0)
    return x ** 3 * (10.0 - 15.0 * x + 6.0 * x ** 2)


class PathSelectionPlanner(LocalPlannerBase):
    name = "path_selection"

    def __init__(self, frenet_frame, params):
        super().__init__(frenet_frame, params)
        p = self.lp["lattice"]
        self.p = p
        # 후보를 '가용 코리도 비율'로 정의 (-1=우측 한계, +1=좌측 한계).
        # 실제 offset[m]은 매 replan마다 available_corridor로 환산 ->
        # 좁은 구간에서도 후보가 살아있고, 넓은 구간에선 크게 벌린다
        self.fractions = np.linspace(-1.0, 1.0, p["n_candidates"])
        self.prev_target = 0.0  # hysteresis 기준 (직전 선택된 목표 offset)

    def plan(self, ego, obstacles):
        """ego: dict(s, d, v). obstacles: Obstacle 필드 dict 리스트.

        반환: (local_wpnts dict, debug dict)
        """
        p = self.p
        s0, d0, v0 = ego["s"], ego["d"], ego["v"]
        s_grid = s0 + np.arange(self.n_plan) * self.ds_plan
        s_rel = np.arange(self.n_plan) * self.ds_plan

        # 현재 d -> 목표 d 전환 프로파일 (모든 후보 공통 모양)
        blend = quintic_blend(s_rel / p["transition_length_m"])

        # 후보 offset = 코리도 비율 x 가용폭 (d_max_m로 상한)
        avail_l, avail_r = self.available_corridor(s_grid)
        d_targets = np.where(self.fractions >= 0,
                             self.fractions * min(avail_l, p["d_max_m"]),
                             self.fractions * min(avail_r, p["d_max_m"]))
        d_targets = np.unique(np.round(d_targets, 3))

        best = None
        candidates_dbg = []
        for d_t in d_targets:
            d_prof = d0 + (d_t - d0) * blend

            # --- 하드 제약 1: 트랙 경계 ---
            if self.check_track_bounds(s_grid, d_prof):
                candidates_dbg.append((d_t, np.inf, "bounds"))
                continue

            # --- 경로 기하 + 속도 프로파일 ---
            geo = self.ff.path_from_d_profile(s_grid, d_prof)
            v_cap = self.ff.raceline_v_at(s_grid)  # raceline 속도를 상한으로
            v_prof = self.velocity_profile_on_path(
                geo["kappa_radpm"], self.ds_plan, v0, v_cap)
            t_prof = self.time_along_path(v_prof, self.ds_plan)

            # --- 하드 제약 2: 시공간 충돌 ---
            hit, clear = self.check_collision_st(
                s_grid, d_prof, t_prof, obstacles, v0)
            if hit:
                candidates_dbg.append((d_t, np.inf, "collision"))
                continue

            # --- 비용 (가중치는 params.yaml lattice.* 참고) ---
            v_loss = float(np.mean(v_cap - v_prof))            # 속도 손실
            deviation = abs(d_t)                                # 라인 이탈
            lat_acc = float(np.max(v_prof ** 2 * np.abs(geo["kappa_radpm"])))
            switch = abs(d_t - self.prev_target)                # 선택 변경
            safety = 1.0 / max(clear, 1e-3)                     # 근접 페널티
            cost = (p["w_progress"] * v_loss
                    + p["w_deviation"] * deviation
                    + p["w_smoothness"] * lat_acc / (self.veh["mu"] * 9.81)
                    + p["w_safety"] * safety
                    + p["w_switch"] * switch)
            candidates_dbg.append((d_t, cost, "ok"))
            if best is None or cost < best["cost"]:
                best = {"cost": cost, "d_t": d_t, "d_prof": d_prof,
                        "v_prof": v_prof, "clear": clear}

        if best is None:
            # 모든 후보가 막힘 -> 추종 모드: 현재 offset 유지 + 앞차 속도로 감속
            # (ACC식 fallback. follow_gap_s 시간 간격 유지)
            d_prof = np.full(self.n_plan, d0)
            v_follow = self._follow_speed(s0, v0, obstacles)
            geo = self.ff.path_from_d_profile(s_grid, d_prof)
            v_prof = self.velocity_profile_on_path(
                geo["kappa_radpm"], self.ds_plan, v0,
                np.full(self.n_plan, v_follow))
            best = {"d_t": d0, "d_prof": d_prof, "v_prof": v_prof,
                    "clear": 0.0, "cost": np.inf}
            mode = "follow"
        else:
            mode = "overtake" if abs(best["d_t"]) > 0.05 else "global"

        self.prev_target = best["d_t"]
        out = self.format_output(s_grid, best["d_prof"], best["v_prof"])
        dbg = {"mode": mode, "d_target": best["d_t"],
               "clearance": best["clear"], "candidates": candidates_dbg}
        return out, dbg

    def _follow_speed(self, s0, v0, obstacles):
        """가장 가까운 전방 상대에 맞춘 추종 속도 (간단 ACC)."""
        v_f = self.veh["v_max_mps"]
        for ob in obstacles:
            gap = self.ff.s_diff(ob["s_center"], s0)
            if gap > 0:  # 전방
                # 목표: gap을 follow_gap_s * v 로 유지. 부족하면 감속
                desired_gap = self.p["follow_gap_s"] * max(ob["vs"], 0.5)
                v_f = min(v_f, max(ob["vs"] + 0.8 * (gap - desired_gap), 0.0))
        return v_f
