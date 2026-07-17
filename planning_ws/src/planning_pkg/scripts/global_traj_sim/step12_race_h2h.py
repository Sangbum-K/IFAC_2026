"""실전형 H2H 레이스: frenet 차량 vs path_selection 차량 동시 출발.

simulate_h2h.py(느린 더미 상대 추월 테스트)와 달리, 두 차 모두 완전한
에이전트다: 각자 자기 planner + pure pursuit + bicycle 플랜트를 갖고,
상대를 노이즈 낀 관측(Obstacle.msg 규격)으로 보며 같은 raceline 속도
한계로 달린다. 속도 우위가 없으므로 승부는 순수하게 planner의
회피/추월/복귀 품질에서 갈린다.

실행 예:
  .venv/bin/python planning_ws/global_traj_sim/race_h2h.py --seed 0 --animate
  .venv/bin/python planning_ws/global_traj_sim/race_h2h.py --series 10
  .venv/bin/python planning_ws/global_traj_sim/race_h2h.py --no-noise --laps 3
"""

import argparse
import collections
import time
from pathlib import Path

import numpy as np
import yaml
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

from step5_frenet import FrenetFrame
from step10_simulate_h2h import (EgoVehicle, PurePursuitTracker, default_raceline,
                          load_params, load_raceline_csv)
from step7_planner_path_selection import PathSelectionPlanner
from step8_planner_frenet import FrenetPlanner

HERE = Path(__file__).resolve().parent
PLANNERS = {"path_selection": PathSelectionPlanner, "frenet": FrenetPlanner}
COLORS = {"path_selection": "#1f77b4", "frenet": "#d62728"}


class NoisyObserver:
    """상대 차량 관측 모사 (perception/tracking 노드 자리).

    opponent.py와 동일한 노이즈 모델(가우시안 + latency + 검출 범위)이지만,
    고정 raceline 주행이 아닌 실제 차량 상태(s, d, v)를 관측한다.
    """

    def __init__(self, frenet_frame, params, rng):
        self.ff = frenet_frame
        self.p = params["opponent"]
        self.veh = params["vehicle"]
        self.rng = rng
        self._hist = collections.deque(maxlen=200)  # latency용 (t, s, d, v)

    def observe(self, target, own_s, t_now):
        """target: dict(s, d, v). 반환: Obstacle 필드 dict 리스트 (0~1개)."""
        self._hist.append((t_now, target["s"], target["d"], target["v"]))
        if abs(self.ff.s_diff(target["s"], own_s)) > self.p["detection_range_m"]:
            return []

        s_o, d_o, v_o = target["s"], target["d"], target["v"]
        nz = self.p["noise"]
        if nz["enabled"]:
            t_delayed = t_now - nz["latency_s"]
            for th, sh, dh, vh in self._hist:
                if th >= t_delayed:
                    s_o, d_o, v_o = sh, dh, vh
                    break
            s_o = self.ff.wrap_s(s_o + self.rng.normal(0, nz["s_std_m"]))
            d_o = d_o + self.rng.normal(0, nz["d_std_m"])
            v_o = max(v_o + self.rng.normal(0, nz["vs_std_mps"]), 0.0)

        half_len = 0.5 * self.veh["length_m"]
        return [{
            "id": 0,
            "s_center": s_o, "d_center": d_o,
            "s_start": self.ff.wrap_s(s_o - half_len),
            "s_end": self.ff.wrap_s(s_o + half_len),
            "d_left": d_o + 0.5 * self.veh["width_m"],
            "d_right": d_o - 0.5 * self.veh["width_m"],
            "vs": v_o, "vd": 0.0,
            # 분산 필드 (Obstacle.msg의 *_var): planner의 불확실성 비례 마진용
            "s_var": nz["s_std_m"] ** 2 if nz["enabled"] else 0.0,
            "d_var": nz["d_std_m"] ** 2 if nz["enabled"] else 0.0,
            "vs_var": nz["vs_std_mps"] ** 2 if nz["enabled"] else 0.0,
            "is_static": False,
        }]


class Agent:
    """차량 + planner + 추종기 + 관측기 한 세트 (= 실차 한 대의 스택)."""

    def __init__(self, name, ff, params, s0, d0, rng):
        self.name = name
        self.ff = ff
        i0 = ff.idx_of_s(np.array([s0]))[0]
        xy = ff.frenet2cart(np.array([s0]), np.array([d0]))[0]
        self.vehicle = EgoVehicle(params["vehicle"], xy[0], xy[1],
                                  float(ff.psi[i0]), v=0.5)
        self.planner = PLANNERS[name](ff, params)
        self.tracker = PurePursuitTracker(params["vehicle"])
        self.observer = NoisyObserver(ff, params, rng)
        self.s, self.d = float(s0), float(d0)
        self.s_total = 0.0            # 누적 진행 거리 (랩 wrap 무관)
        self.lap_times = []
        self._lap_start_t = 0.0
        self._prev_s = float(s0)
        self.wpnts = None
        self.dbg = {}
        self.plan_times = []

    def frenet_state(self):
        return {"s": self.s, "d": self.d, "v": self.vehicle.v}

    def replan(self, other, t):
        obstacles = self.observer.observe(other.frenet_state(), self.s, t)
        t0 = time.perf_counter()
        self.wpnts, self.dbg = self.planner.plan(self.frenet_state(), obstacles)
        self.plan_times.append(time.perf_counter() - t0)

    def step(self, dt, t):
        steer, v_cmd = self.tracker.control(self.vehicle, self.wpnts)
        self.vehicle.step(steer, v_cmd, dt)
        self.s, self.d = self.ff.cart2frenet([self.vehicle.x, self.vehicle.y])
        ds = self.ff.s_diff(self.s, self._prev_s)
        self.s_total += max(ds, 0.0)  # 후진성 노이즈는 진행으로 안 침
        # 랩 판정: s wrap 통과
        if self._prev_s > 0.8 * self.ff.track_len and self.s < 0.2 * self.ff.track_len:
            self.lap_times.append(t - self._lap_start_t)
            self._lap_start_t = t
        self._prev_s = self.s


def run_race(params, raceline_csv, seed=0, laps=None, noise=None):
    """1회 레이스 실행 -> (log, metrics, ff)."""
    params = yaml.safe_load(yaml.safe_dump(params))  # deep copy
    if noise is not None:
        params["opponent"]["noise"]["enabled"] = noise
    race_p = params["race"]
    if laps:
        race_p["laps"] = laps

    traj = load_raceline_csv(raceline_csv)
    ff = FrenetFrame(traj)
    rng = np.random.default_rng(seed)

    # 스타트 그리드: 시드로 위치/선두를 랜덤화 (트랙 구간 편향 제거)
    s_grid = float(rng.uniform(0, ff.track_len))
    front_first = bool(rng.integers(0, 2))  # 누가 앞 그리드인지
    names = ["frenet", "path_selection"]
    if front_first:
        names.reverse()
    agents = [
        Agent(names[0], ff, params, s_grid, +race_p["stagger_d_m"],
              np.random.default_rng(seed * 2 + 1)),
        Agent(names[1], ff, params,
              ff.wrap_s(s_grid - race_p["stagger_s_m"]),
              -race_p["stagger_d_m"], np.random.default_rng(seed * 2 + 2)),
    ]

    dt = params["sim"]["dt_s"]
    replan_every = max(int(round(
        1.0 / params["local_planner"]["replan_hz"] / dt)), 1)
    goal = race_p["laps"] * ff.track_len
    hyst = race_p["lead_change_hysteresis_m"]

    log = {k: [] for k in ("t", "ax", "ay", "bx", "by", "atheta", "btheta",
                           "gap", "dist", "av", "bv")}
    t, step_i = 0.0, 0
    result, winner = "timeout", None
    lead_changes, lead_sign = 0, 0

    while t < race_p["max_time_s"]:
        a, b = agents
        # 재계획: 두 에이전트의 주기를 반주기 엇갈리게 (동기화 아티팩트 방지)
        if step_i % replan_every == 0 or a.wpnts is None:
            a.replan(b, t)
        if (step_i + replan_every // 2) % replan_every == 0 or b.wpnts is None:
            b.replan(a, t)
        a.step(dt, t)
        b.step(dt, t)

        dist = float(np.hypot(a.vehicle.x - b.vehicle.x,
                              a.vehicle.y - b.vehicle.y))
        gap = a.s_total - b.s_total  # +면 agents[0] 리드
        # 리드 교대 카운트 (히스테리시스로 노이즈 출렁임 제외)
        sign = 1 if gap > hyst else (-1 if gap < -hyst else 0)
        if sign != 0 and lead_sign != 0 and sign != lead_sign:
            lead_changes += 1
        if sign != 0:
            lead_sign = sign

        if dist < params["sim"]["collision_dist_m"]:
            result = "collision"
            break
        finished = [ag for ag in agents if ag.s_total >= goal]
        if finished:
            result = "finish"
            winner = max(agents, key=lambda ag: ag.s_total).name
            break

        log["t"].append(t)
        log["ax"].append(a.vehicle.x); log["ay"].append(a.vehicle.y)
        log["bx"].append(b.vehicle.x); log["by"].append(b.vehicle.y)
        log["atheta"].append(a.vehicle.theta)
        log["btheta"].append(b.vehicle.theta)
        log["av"].append(a.vehicle.v); log["bv"].append(b.vehicle.v)
        log["gap"].append(gap); log["dist"].append(dist)
        t += dt
        step_i += 1

    if result == "timeout" and winner is None:
        winner = max(agents, key=lambda ag: ag.s_total).name

    log = {k: np.asarray(v) for k, v in log.items()}
    log["names"] = [ag.name for ag in agents]
    a, b = agents
    metrics = {
        "result": result,
        "winner": winner if result != "collision" else None,
        "margin_m": abs(a.s_total - b.s_total),
        "lead_changes": lead_changes,
        "min_dist_m": float(log["dist"].min()) if len(log["dist"]) else np.nan,
        "race_time_s": t,
        "noise": params["opponent"]["noise"]["enabled"],
        "seed": seed,
        "front_grid": names[0],
    }
    for ag in agents:
        metrics[f"{ag.name}_mean_v"] = float(np.mean(
            log["av" if ag is a else "bv"])) if len(log["t"]) else np.nan
        metrics[f"{ag.name}_laps"] = len(ag.lap_times)
        # 첫 랩은 그리드 위치 -> 피니시라인 부분 랩이므로 기록에서 제외
        full_laps = ag.lap_times[1:]
        metrics[f"{ag.name}_best_lap_s"] = (min(full_laps)
                                            if full_laps else np.nan)
        metrics[f"{ag.name}_plan_ms"] = 1e3 * float(np.mean(ag.plan_times))
    return log, metrics, ff


def plot_race(log, metrics, ff, out_path):
    a_name, b_name = log["names"]
    fig, axes = plt.subplots(1, 2, figsize=(15, 7),
                             gridspec_kw={"width_ratios": [1.4, 1]})
    ax = axes[0]
    ax.plot(ff.xy[:, 0], ff.xy[:, 1], "--", color="gray", lw=0.5, alpha=0.5)
    ax.plot(log["ax"], log["ay"], color=COLORS[a_name], lw=1.0, label=a_name)
    ax.plot(log["bx"], log["by"], color=COLORS[b_name], lw=1.0, label=b_name)
    ax.plot(log["ax"][0], log["ay"][0], "^", color=COLORS[a_name], ms=10)
    ax.plot(log["bx"][0], log["by"][0], "^", color=COLORS[b_name], ms=10)
    ax.set_aspect("equal"); ax.legend()
    ax.set_title(f"race: winner={metrics['winner']} "
                 f"(margin {metrics['margin_m']:.1f} m, "
                 f"{metrics['lead_changes']} lead changes)")

    ax2 = axes[1]
    ax2.plot(log["t"], log["gap"], "k-", lw=1.0)
    ax2.axhline(0, color="gray", lw=0.5)
    ax2.fill_between(log["t"], log["gap"], 0, where=log["gap"] > 0,
                     color=COLORS[a_name], alpha=0.25, label=f"{a_name} leads")
    ax2.fill_between(log["t"], log["gap"], 0, where=log["gap"] < 0,
                     color=COLORS[b_name], alpha=0.25, label=f"{b_name} leads")
    ax2.set_xlabel("t [s]"); ax2.set_ylabel("progress gap [m]")
    ax2.legend(); ax2.grid(alpha=0.3)
    fig.tight_layout()
    fig.savefig(out_path, dpi=140)
    plt.close(fig)


def animate_race(log, metrics, ff, params, out_path, fps=20, speed=3.0):
    """두 차 중간점 follow-cam GIF (simulate_h2h.animate_run의 2차량판)."""
    from matplotlib import animation
    from matplotlib.patches import Polygon
    from simulate_h2h import _car_corners

    a_name, b_name = log["names"]
    dt = float(log["t"][1] - log["t"][0])
    step = max(int(round(speed / (fps * dt))), 1)
    frames = range(0, len(log["t"]), step)
    veh = params["vehicle"]

    fig, ax = plt.subplots(figsize=(8, 8))
    left = ff.xy + ff.d_left[:, None] * ff.normal
    right = ff.xy - ff.d_right[:, None] * ff.normal
    for bnd in (left, right):
        bnd = np.vstack([bnd, bnd[:1]])
        ax.plot(bnd[:, 0], bnd[:, 1], "k-", lw=1.0, alpha=0.6)
    ax.plot(ff.xy[:, 0], ff.xy[:, 1], "--", color="gray", lw=0.7, alpha=0.6)
    ax.set_aspect("equal")
    ax.set_title(f"{a_name} vs {b_name} · winner: {metrics['winner']}")

    trails, patches = {}, {}
    for name, kx in ((a_name, "a"), (b_name, "b")):
        trails[kx], = ax.plot([], [], "-", color=COLORS[name], lw=1.2,
                              alpha=0.8)
        patches[kx] = Polygon(np.zeros((4, 2)), closed=True,
                              fc=COLORS[name], ec="k", lw=0.5, zorder=5,
                              label=name)
        ax.add_patch(patches[kx])
    hud = ax.text(0.02, 0.98, "", transform=ax.transAxes, va="top",
                  fontsize=10, family="monospace",
                  bbox=dict(fc="white", alpha=0.75, ec="none"))
    ax.legend(loc="upper right", fontsize=9)

    def draw(i):
        for kx, xk, yk, tk in (("a", "ax", "ay", "atheta"),
                               ("b", "bx", "by", "btheta")):
            trails[kx].set_data(log[xk][:i + 1], log[yk][:i + 1])
            patches[kx].set_xy(_car_corners(
                log[xk][i], log[yk][i], log[tk][i],
                veh["length_m"], veh["width_m"]))
        cx = 0.5 * (log["ax"][i] + log["bx"][i])
        cy = 0.5 * (log["ay"][i] + log["by"][i])
        w = max(14.0, 0.6 * log["dist"][i] + 8.0)  # 벌어지면 줌아웃
        ax.set_xlim(cx - w, cx + w)
        ax.set_ylim(cy - w, cy + w)
        hud.set_text(f"t    {log['t'][i]:5.1f} s (x{speed:.0f})\n"
                     f"gap  {log['gap'][i]:+5.1f} m (+ = {a_name[:6]})\n"
                     f"dist {log['dist'][i]:5.2f} m")
        return list(trails.values()) + list(patches.values()) + [hud]

    anim = animation.FuncAnimation(fig, draw, frames=frames, blit=False)
    anim.save(out_path, writer=animation.PillowWriter(fps=fps), dpi=80)
    plt.close(fig)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--raceline", default=None)
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--laps", type=int, default=None)
    ap.add_argument("--no-noise", action="store_true",
                    help="관측 노이즈 끄기 (기본은 켬 = 현실적 조건)")
    ap.add_argument("--series", type=int, default=0,
                    help="N개 시드로 연속 레이스 -> 승수 집계")
    ap.add_argument("--plot", action="store_true")
    ap.add_argument("--animate", action="store_true")
    ap.add_argument("--anim-speed", type=float, default=3.0)
    args = ap.parse_args()

    params = load_params()
    csv = Path(args.raceline) if args.raceline else default_raceline()
    noise = False if args.no_noise else True

    if args.series > 0:
        wins = {"frenet": 0, "path_selection": 0}
        rows = []
        for seed in range(args.series):
            _, m, _ = run_race(params, csv, seed=seed, laps=args.laps,
                               noise=noise)
            if m["winner"]:
                wins[m["winner"]] += 1
            rows.append(m)
            print(f"seed {seed}: {m['result']:<9} winner={str(m['winner']):<15}"
                  f" margin={m['margin_m']:6.1f}m lead_changes={m['lead_changes']}"
                  f" min_dist={m['min_dist_m']:.2f}m front_grid={m['front_grid']}")
        n_coll = sum(1 for r in rows if r["result"] == "collision")
        print(f"\n=== {args.series} races (noise={noise}) ===")
        print(f"frenet {wins['frenet']}승 : path_selection "
              f"{wins['path_selection']}승 (충돌 {n_coll}회)")
        for name in PLANNERS:
            best = [r[f"{name}_best_lap_s"] for r in rows
                    if np.isfinite(r.get(f"{name}_best_lap_s", np.nan))]
            if best:
                print(f"{name}: best lap {min(best):.2f}s, "
                      f"mean best {np.mean(best):.2f}s")
        return

    log, metrics, ff = run_race(params, csv, seed=args.seed, laps=args.laps,
                                noise=noise)
    for k, v in metrics.items():
        print(f"  {k}: {v}")
    out_dir = HERE / "outputs"
    if args.plot:
        plot_race(log, metrics, ff, out_dir / f"race_seed{args.seed}.png")
        print(f"  plot: {out_dir / f'race_seed{args.seed}.png'}")
    if args.animate:
        animate_race(log, metrics, ff, params,
                     out_dir / f"race_seed{args.seed}.gif",
                     speed=args.anim_speed)
        print(f"  animation: {out_dir / f'race_seed{args.seed}.gif'}")


if __name__ == "__main__":
    main()
