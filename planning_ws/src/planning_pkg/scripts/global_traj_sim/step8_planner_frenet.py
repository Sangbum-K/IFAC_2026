"""방식 2: Frenet Optimal Trajectory (Werling 스타일) local planner.

원리 (Werling et al., "Optimal Trajectory Generation for Dynamic Street
Scenarios in a Frenet Frame", ICRA 2010):
  횡방향  d(t): quintic,  (d0, d0', d0'') -> (d_target, 0, 0), T in t_horizons
  종방향  s(t): quartic,  velocity-keeping (종단 s 자유, 종단 속도 지정)
  모든 (lat x lon) 조합에 대해 jerk/시간/종단편차 비용 + 시공간 충돌 체크
  -> 최소 비용의 "시간 파라미터화된" 궤적 선택

장점: 추월 '타이밍'까지 계획 (언제 옆을 지나는지) -> 동적 상대에 정교
단점: 계산량 큼, 상대 등속 예측이 틀리면 자주 재계획 필요

튜닝 순서 권장:
  1. t_horizons_s — 회피가 늦으면 짧은 T 추가, 조향이 과격하면 긴 T 추가
  2. k_safety / safety 마진 — 근접 통과 허용도
  3. k_jerk vs k_time — 부드러움 vs 기민함 트레이드오프
  4. d_targets 간격 — 좁은 트랙이면 0.2m 간격으로 촘촘히

Jetson NX 예산 메모:
  후보 수 = len(d_targets) x len(t_horizons) x n_v_targets (기본 7x3x3=63).
  전 구간 numpy 벡터화라 Python에서도 후보당 ~수십 us. C++ 포팅 시 40Hz 여유.
"""

import numpy as np

from step6_planner_base import LocalPlannerBase


def quintic_coeffs(x0, dx0, ddx0, xT, dxT, ddxT, T):
    """경계조건 6개를 만족하는 5차 다항식 계수 (a0..a5)."""
    a0, a1, a2 = x0, dx0, 0.5 * ddx0
    T2, T3, T4, T5 = T ** 2, T ** 3, T ** 4, T ** 5
    A = np.array([[T3, T4, T5],
                  [3 * T2, 4 * T3, 5 * T4],
                  [6 * T, 12 * T2, 20 * T3]])
    b = np.array([xT - (a0 + a1 * T + a2 * T2),
                  dxT - (a1 + 2 * a2 * T),
                  ddxT - 2 * a2])
    a3, a4, a5 = np.linalg.solve(A, b)
    return np.array([a0, a1, a2, a3, a4, a5])


def quartic_coeffs(x0, dx0, ddx0, dxT, ddxT, T):
    """종단 위치 자유(velocity keeping) 4차 다항식 계수 (a0..a4)."""
    a0, a1, a2 = x0, dx0, 0.5 * ddx0
    T2, T3 = T ** 2, T ** 3
    A = np.array([[3 * T2, 4 * T3],
                  [6 * T, 12 * T2]])
    b = np.array([dxT - (a1 + 2 * a2 * T),
                  ddxT - 2 * a2])
    a3, a4 = np.linalg.solve(A, b)
    return np.array([a0, a1, a2, a3, a4, 0.0])


def poly_eval(coeffs, t, der=0):
    """다항식/도함수 평가 (coeffs: 저차부터)."""
    c = np.array(coeffs, dtype=float)
    for _ in range(der):
        c = c[1:] * np.arange(1, len(c))
    return np.polyval(c[::-1], t)


class FrenetPlanner(LocalPlannerBase):
    name = "frenet"

    def __init__(self, frenet_frame, params):
        super().__init__(frenet_frame, params)
        self.p = self.lp["frenet"]
        self.prev_d_end = 0.0     # 직전 계획 종단 d (채터링 억제)
        # 직전 계획의 미분 상태를 다음 계획 초기조건으로 이어붙임
        # (실차에서는 planner 출력 궤적 위의 현재 시점 상태를 쓰는 것과 동일)
        self.d_dot0 = 0.0
        self.d_ddot0 = 0.0

    def _block_penalty(self, s0, v0, d_t, obstacles):
        """상대가 이 종단 offset(d_t)의 코리도를 막고 있는 정도 -> 비용 [0, k_block].

        '막혔다/안막혔다' 이진 판정 대신 연속 가중치를 쓴다. 노이즈 낀 관측
        (ob["vs"], ob["d_center"])에 하드 threshold(if 문)를 걸면, 값이 경계를
        넘나들 때마다 replan(20Hz)마다 판정이 뒤집혀 후보 선택이 계속
        바뀌는 '플리커'가 생긴다 — 이게 노이즈 조건 H2H 레이스에서 frenet이
        코너마다 시간을 잃은 주원인이었다(path_selection의 1/clearance
        연속비용은 이 문제가 없었다). 여기서도 같은 방식으로, 판정 경계
        주변에 soft_band 폭의 선형 램프를 둬서 노이즈 한 스텝 정도의
        흔들림은 비용에 완만하게만 반영되게 한다.
        """
        p = self.p
        _, d_margin = self.collision_margins(v0)
        total = 0.0
        for ob in obstacles:
            gap = self.ff.s_diff(ob["s_center"], s0)
            if not (0.0 < gap < self.horizon):
                continue
            # 상대가 나보다 느릴수록 1에 가까움 (block_speed_soft_band_mps 폭 램프)
            speed_w = np.clip(
                (v0 + p["block_speed_margin_mps"] - ob["vs"])
                / p["block_speed_soft_band_mps"], 0.0, 1.0)
            if speed_w <= 0.0:
                continue
            # d_t가 상대 코리도에 가까울수록 1에 가까움 (block_d_soft_band_m 폭 램프)
            d_gap = abs(d_t - ob["d_center"])
            corridor_w = np.clip(
                (d_margin + p["block_d_soft_band_m"] - d_gap)
                / p["block_d_soft_band_m"], 0.0, 1.0)
            total += speed_w * corridor_w
        return p["k_block"] * total

    def plan(self, ego, obstacles):
        p = self.p
        s0, d0, v0 = ego["s"], ego["d"], ego["v"]
        s_scan = s0 + np.arange(self.n_plan) * self.ds_plan  # 공용 호라이즌 그리드

        # 종방향 목표 속도 후보: {호라이즌 내 raceline 최저속, 앞차 속도,
        # 현재 위치 raceline 속도} 범위를 커버해야 한다.
        # 현재 위치 속도만 쓰면 코너 진입 시 감속 후보가 없어 전 후보가
        # a_lat 기각 -> 비상제동에 빠진다 (코너 옆 추월 실패의 주원인)
        v_race = float(self.ff.raceline_v_at(np.array([s0]))[0])
        v_min_ahead = float(np.min(self.ff.raceline_v_at(s_scan)))
        v_set = {min(v_race, self.veh["v_max_mps"]), v_min_ahead}
        for ob in obstacles:
            if self.ff.s_diff(ob["s_center"], s0) > 0:
                v_set.add(np.clip(ob["vs"], 0.5, self.veh["v_max_mps"]))
        v_targets = np.unique(np.concatenate(
            [np.linspace(min(v_set), max(v_set), p["n_v_targets"])]))

        # 종단 d 후보: 코리도 비율 -> 실제 offset[m] 환산 (lattice와 동일 방식,
        # 좁은 구간에서 후보 전멸 방지). 비율은 params frenet.d_target_fractions
        avail_l, avail_r = self.available_corridor(s_scan)
        fr = np.asarray(p["d_target_fractions"])
        d_targets = np.unique(np.round(
            np.where(fr >= 0, fr * avail_l, fr * avail_r), 3))

        best = None
        n_evaluated = 0
        # 기각 사유 카운터 (튜닝 시 dbg로 확인: 어떤 제약이 후보를 죽이는지)
        rej = {"v_range": 0, "acc": 0, "bounds": 0, "collision": 0, "ok": 0}
        for T in p["t_horizons_s"]:
            t = np.arange(0.0, T + 1e-9, p["dt_eval_s"])
            for d_t in d_targets:
                cl = quintic_coeffs(d0, self.d_dot0, self.d_ddot0,
                                    d_t, 0.0, 0.0, T)
                d_traj = poly_eval(cl, t)
                d_jerk2 = float(np.trapezoid(poly_eval(cl, t, 3) ** 2, t))
                # d_t/obstacles에만 의존 -> v_t 루프 밖에서 한 번만 계산
                block_pen = self._block_penalty(s0, v0, d_t, obstacles)
                for v_t in v_targets:
                    n_evaluated += 1
                    cs = quartic_coeffs(0.0, v0, 0.0, v_t, 0.0, T)
                    s_traj = s0 + poly_eval(cs, t)
                    vs_traj = poly_eval(cs, t, 1)
                    s_jerk2 = float(np.trapezoid(poly_eval(cs, t, 3) ** 2, t))

                    # --- 하드 제약: 속도/가속/경계/횡가속 ---
                    if vs_traj.min() < -0.1 or \
                       vs_traj.max() > self.veh["v_max_mps"] + p["v_slack_mps"]:
                        rej["v_range"] += 1
                        continue
                    acc = poly_eval(cs, t, 2)
                    if np.abs(acc).max() > (self.veh["a_dec_max_mps2"]
                                            + p["acc_slack_mps2"]):
                        rej["acc"] += 1
                        continue
                    if self.check_track_bounds(s_traj, d_traj):
                        rej["bounds"] += 1
                        continue
                    # 횡가속(a_lat) 하드 리젝트는 일부러 두지 않는다: quartic
                    # vs_traj는 "속도 유지" 궤적이라 코너 앞에서 미리 감속하지
                    # 않고, 게다가 라인 곡률(raceline kappa)을 근사로 썼었다.
                    # 그 결과 코너 접근 시 사실상 전 후보가 리젝트되어 매번
                    # follow 폴백에 빠졌었다(실측: 코너 구간 replan의 rej
                    # 사유 99.8%가 a_lat) — H2H 레이스에서 frenet이 상대 없이
                    # 혼자 달려도 path_selection보다 랩당 ~3초 느렸던 진짜
                    # 원인이 여기였다(이전의 k_block/v_cap 수정은 이 문제를
                    # 잡지 못했다). 실제 실행 속도는 아래에서 항상
                    # velocity_profile_on_path로 이 후보의 '실제' 오프셋
                    # 경로 곡률(geo, path_from_d_profile 산출) 기준
                    # friction-circle 상한으로 다시 씌워지므로, 여기서
                    # vs_traj를 그대로 안전 판정에 쓰지 않아도 실행 단계에서
                    # 안전하게 재보정된다.

                    # --- 하드 제약: 시공간 충돌 (t가 이미 있으므로 직접) ---
                    hit, clear = self.check_collision_st(
                        s_traj, d_traj, t, obstacles, v0)
                    if hit:
                        rej["collision"] += 1
                        continue
                    rej["ok"] += 1

                    # --- 비용 (params.yaml frenet.* 가중치) ---
                    # k_block 항(block_pen)이 상대가 막은 코리도에 남는 후보를
                    # 연속적으로 밀어내, 옆 코리도로 빠지는 1단계 기동이 이기게 함
                    cost = (p["k_jerk"] * (d_jerk2 + s_jerk2)
                            + p["k_time"] * T
                            + p["k_d"] * d_t ** 2
                            + p["k_v"] * (v_t - v_race) ** 2
                            + p["k_safety"] / max(clear, 1e-3)
                            + p["k_switch"] * abs(d_t - self.prev_d_end)
                            + block_pen)
                    if best is None or cost < best["cost"]:
                        best = {"cost": cost, "T": T, "cl": cl, "cs": cs,
                                "t": t, "s": s_traj, "d": d_traj,
                                "v": vs_traj, "d_t": d_t, "clear": clear}

        if best is None:
            # 유효 후보 없음 -> follow 폴백: 현재 offset 유지, 곡률/앞차에 맞춘
            # 감속 프로파일. (급제동 폴백은 코너에서 flicker를 유발해 폐기.
            # 실차라면 behavior state AVOID -> FOLLOW로 내리는 지점)
            s_grid = s_scan
            d_prof = np.full(self.n_plan, d0)
            geo = self.ff.path_from_d_profile(s_grid, d_prof)
            v_cap = self.ff.raceline_v_at(s_grid).copy()
            for ob in obstacles:  # 앞차 속도를 상한에 반영 (간단 ACC)
                if 0 < self.ff.s_diff(ob["s_center"], s0) < self.horizon:
                    v_cap = np.minimum(v_cap, max(ob["vs"], 0.5))
            v_prof = self.velocity_profile_on_path(
                geo["kappa_radpm"], self.ds_plan, v0, v_cap)
            out = self.format_output(s_grid, d_prof, v_prof)
            self.d_dot0 = self.d_ddot0 = 0.0
            # follow 중 유지하는 offset을 hysteresis 기준으로 갱신해둬야
            # 다음 유효 계획의 k_switch 항이 오래된 기준과 비교하지 않는다
            self.prev_d_end = d0
            return out, {"mode": "follow", "d_target": d0,
                         "clearance": 0.0, "n_eval": n_evaluated, "rej": rej}

        # --- 시간 궤적 -> 공간 그리드 변환 (출력 규격은 s 기반 윈도우) ---
        s_rel = best["s"] - s0
        # 단조 증가 보장 (저속에서 수치상 정체 방지)
        s_rel = np.maximum.accumulate(s_rel + 1e-6 * np.arange(len(s_rel)))
        s_grid = s_scan
        grid_s_rel = np.arange(self.n_plan) * self.ds_plan
        d_prof = np.interp(grid_s_rel, s_rel, best["d"],
                           right=best["d"][-1])  # 궤적 끝 이후는 offset 유지
        v_prof = np.interp(grid_s_rel, s_rel, best["v"], right=best["v"][-1])
        geo = self.ff.path_from_d_profile(s_grid, d_prof)
        # v_cap: 궤적이 실제로 커버하는 구간(0~s_rel[-1])은 충돌검증된 vs_traj
        # 그대로(안전 마진 유지), 그 너머는 raceline 속도로 캡을 풀어 자연
        # 가속 회복을 허용한다.
        # 주의(수정 이력): 예전엔 np.maximum(v_prof, best["v"][-1])로 풀어주려
        # 했으나, v_prof 자체가 이미 궤적 끝에서 best["v"][-1]로 수렴하는
        # 프로파일이라 max()가 사실상 항상 best["v"][-1]로 평가돼 호라이즌
        # 전체가 그 값으로 눌렸다 (직선에서 시종 raceline 속도를 못 내는 원인 —
        # H2H 레이스에서 frenet이 상대 없이 혼자 달려도 path_selection보다
        # 랩당 ~3초 느렸던 것이 이 버그 때문이었다).
        n_cov = int(np.searchsorted(grid_s_rel, s_rel[-1], side="right"))
        v_cap = np.where(np.arange(self.n_plan) < n_cov,
                        v_prof, self.ff.raceline_v_at(s_grid))
        v_prof = self.velocity_profile_on_path(
            geo["kappa_radpm"], self.ds_plan, v0, v_cap)

        # 다음 replan의 초기 미분 상태 = 이번 궤적의 replan 주기 시점 상태
        t_next = 1.0 / self.lp["replan_hz"]
        self.d_dot0 = float(poly_eval(best["cl"], t_next, 1))
        self.d_ddot0 = float(poly_eval(best["cl"], t_next, 2))
        self.prev_d_end = best["d_t"]

        out = self.format_output(s_grid, d_prof, v_prof)
        # mode는 dbg/HUD 표시 전용 (제어 로직에는 영향 없음) -> 하드 threshold라도
        # 노이즈로 문자열이 깜빡이는 정도라 플리커 우려 없음
        in_front = any(0.0 < self.ff.s_diff(ob["s_center"], s0) < self.horizon
                      for ob in obstacles)
        mode = ("overtake" if (in_front or abs(best["d_t"]) > p["mode_threshold_m"])
               else "global")
        dbg = {"mode": mode, "d_target": best["d_t"],
               "clearance": best["clear"], "T": best["T"],
               "n_eval": n_evaluated, "rej": rej}
        return out, dbg
