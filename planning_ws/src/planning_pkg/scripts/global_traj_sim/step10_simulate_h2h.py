"""Head-to-Head 시뮬레이션: local planner + pure pursuit + kinematic bicycle.

파이프라인 (ROS 그래프 미러링):
  raceline CSV 로드           <- global_trajectory_node과 동일한 CSV
  frenet 변환                 <- frenet_frame_node
  local planner (본 실험 대상) <- local_trajectory_node 자리에 들어갈 신규 계층
  pure pursuit               <- crazy_controller (pp 모드, 동일 L1 스케줄)
  bicycle plant              <- f1tenth_gym 대응 (조향각속도 한계 포함)

실행 예:
  .venv/bin/python planning_ws/global_traj_sim/simulate_h2h.py \
      --planner path_selection --opp-speed 0.75 --plot
  .venv/bin/python planning_ws/global_traj_sim/simulate_h2h.py \
      --planner frenet --noise --seed 3
"""

import argparse
import time
from pathlib import Path

import numpy as np
import yaml
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

from step5_frenet import FrenetFrame
from step9_opponent import Opponent
from step7_planner_path_selection import PathSelectionPlanner
from step8_planner_frenet import FrenetPlanner

HERE = Path(__file__).resolve().parent

PLANNERS = {"path_selection": PathSelectionPlanner, "frenet": FrenetPlanner}


def load_params(path=HERE / "params.yaml"):
    with open(path) as f:
        return yaml.safe_load(f)


def load_raceline_csv(csv_path):
    """global_trajectory_node이 읽는 것과 동일한 9열 CSV -> traj dict."""
    d = np.genfromtxt(csv_path, delimiter=";", comments="#")
    return {"s": d[:, 0], "xy": d[:, 1:3], "d_right": d[:, 3],
            "d_left": d[:, 4], "psi": d[:, 5], "kappa": d[:, 6],
            "vx": d[:, 7], "ax": d[:, 8]}


def default_raceline():
    cands = sorted((HERE / "outputs").glob("Spielberg_map_*mu*.csv"))
    if not cands:
        raise FileNotFoundError(
            "raceline CSV가 없습니다. 먼저 main.py로 생성하세요: "
            ".venv/bin/python planning_ws/global_traj_sim/main.py --map Spielberg_map")
    return cands[0]


class EgoVehicle:
    """kinematic bicycle + 순정 서보의 조향각속도 한계 반영."""

    def __init__(self, veh_params, x, y, theta, v=0.3):
        self.p = veh_params
        self.x, self.y, self.theta, self.v = x, y, theta, v
        self.steer = 0.0

    def step(self, steer_cmd, v_cmd, dt):
        # 서보 속도 한계: 명령이 커도 max_steer_rate 이상 못 돌린다
        d_steer = np.clip(steer_cmd - self.steer,
                          -self.p["max_steer_rate_radps"] * dt,
                          self.p["max_steer_rate_radps"] * dt)
        self.steer = np.clip(self.steer + d_steer,
                             -self.p["max_steer_rad"], self.p["max_steer_rad"])
        dv = np.clip(v_cmd - self.v, -self.p["a_dec_max_mps2"] * dt,
                     self.p["a_acc_max_mps2"] * dt)
        self.v = max(self.v + dv, 0.0)
        self.theta += self.v / self.p["wheelbase_m"] * np.tan(self.steer) * dt
        self.x += self.v * np.cos(self.theta) * dt
        self.y += self.v * np.sin(self.theta) * dt


class PurePursuitTracker:
    """crazy_controller pp 모드 미러: L1 = clip(m_l1*v + q_l1, clip범위)."""

    def __init__(self, veh_params, m_l1=0.3, q_l1=0.4,
                 t_clip_min=0.8, t_clip_max=5.0):
        self.L = veh_params["wheelbase_m"]
        self.max_steer = veh_params["max_steer_rad"]
        self.m_l1, self.q_l1 = m_l1, q_l1
        self.clip = (t_clip_min, t_clip_max)

    def control(self, ego, wpnts):
        """wpnts: planner 출력 (x_m, y_m, vx_mps 사용)."""
        pos = np.array([ego.x, ego.y])
        pts = np.column_stack([wpnts["x_m"], wpnts["y_m"]])
        l1 = np.clip(self.m_l1 * ego.v + self.q_l1, *self.clip)
        dist = np.hypot(*(pts - pos).T)
        ahead = np.nonzero(dist >= l1)[0]
        j = ahead[0] if len(ahead) else len(pts) - 1
        dx, dy = pts[j] - pos
        alpha = np.arctan2(dy, dx) - ego.theta
        alpha = np.arctan2(np.sin(alpha), np.cos(alpha))
        ld = max(dist[j], 1e-3)
        steer = np.clip(np.arctan2(2 * self.L * np.sin(alpha), ld),
                        -self.max_steer, self.max_steer)
        return steer, float(wpnts["vx_mps"][j])


def run_h2h(planner_name, params, raceline_csv, opp_speed=None,
            noise=None, seed=0, start_gap=None):
    """단일 H2H 시나리오 실행 -> 로그/메트릭 dict."""
    params = yaml.safe_load(yaml.safe_dump(params))  # deep copy (배치 안전)
    if opp_speed is not None:
        params["opponent"]["speed_factor"] = opp_speed
    if noise is not None:
        params["opponent"]["noise"]["enabled"] = noise
    if start_gap is not None:
        params["sim"]["start_gap_m"] = start_gap

    traj = load_raceline_csv(raceline_csv)
    ff = FrenetFrame(traj)
    rng = np.random.default_rng(seed)
    sim_p, veh_p = params["sim"], params["vehicle"]

    # 시작 배치: seed로 상대 시작 s를 트랙 곳곳에 랜덤화 (구간 편향 제거)
    s_opp0 = float(rng.uniform(0, ff.track_len))
    s_ego0 = ff.wrap_s(s_opp0 - sim_p["start_gap_m"])
    i0 = ff.idx_of_s(np.array([s_ego0]))[0]
    ego = EgoVehicle(veh_p, *ff.xy[i0], float(ff.psi[i0]),
                     v=float(traj["vx"][i0] * 0.5))
    opp = Opponent(ff, params, s_opp0, rng)
    planner = PLANNERS[planner_name](ff, params)
    tracker = PurePursuitTracker(veh_p)

    dt = sim_p["dt_s"]
    replan_every = max(int(round(1.0 / params["local_planner"]["replan_hz"] / dt)), 1)
    log = {k: [] for k in ("t", "x", "y", "theta", "v", "s", "d", "opp_s",
                           "opp_x", "opp_y", "gap", "dist", "mode", "steer")}
    wpnts, dbg = None, {}
    plan_times = []
    t = 0.0
    step_i = 0
    result = "timeout"
    t_overtake = np.nan
    ahead_since = None

    while t < sim_p["max_time_s"]:
        s_ego, d_ego = ff.cart2frenet([ego.x, ego.y])

        # --- planner (replan_hz 주기) ---
        if step_i % replan_every == 0 or wpnts is None:
            obstacles = opp.observe(s_ego, t)
            t0 = time.perf_counter()
            wpnts, dbg = planner.plan(
                {"s": s_ego, "d": d_ego, "v": ego.v}, obstacles)
            plan_times.append(time.perf_counter() - t0)

        # --- controller + plant (매 스텝) ---
        steer_cmd, v_cmd = tracker.control(ego, wpnts)
        ego.step(steer_cmd, v_cmd, dt)
        opp.step(dt)

        # --- 판정 ---
        opp_xy = opp.pose()
        dist = float(np.hypot(ego.x - opp_xy[0], ego.y - opp_xy[1]))
        gap = ff.s_diff(s_ego, opp.s)  # +면 ego가 앞
        if dist < sim_p["collision_dist_m"]:
            result = "collision"
            break
        if gap > sim_p["overtake_hold_m"]:
            if ahead_since is None:
                ahead_since = t
            if t - ahead_since > 1.0:  # 1초 유지하면 추월 확정
                result = "overtake"
                t_overtake = ahead_since
                break
        else:
            ahead_since = None

        log["t"].append(t); log["x"].append(ego.x); log["y"].append(ego.y)
        log["theta"].append(ego.theta)
        log["v"].append(ego.v); log["s"].append(s_ego); log["d"].append(d_ego)
        log["opp_s"].append(opp.s)
        log["opp_x"].append(opp_xy[0]); log["opp_y"].append(opp_xy[1])
        log["gap"].append(gap); log["dist"].append(dist)
        log["mode"].append(dbg.get("mode", "?"))
        log["steer"].append(ego.steer)
        t += dt
        step_i += 1

    log = {k: (np.asarray(v) if k != "mode" else v) for k, v in log.items()}
    steer = log["steer"]
    metrics = {
        "planner": planner_name,
        "result": result,
        "t_overtake_s": float(t_overtake),
        "min_dist_m": float(log["dist"].min()) if len(log["dist"]) else np.nan,
        "mean_v_mps": float(log["v"].mean()) if len(log["v"]) else np.nan,
        # 조향각 변화율 RMS: 서보 부담/승차감 프록시 (실차 튜닝 시 참고)
        "steer_rate_rms_radps": float(np.sqrt(np.mean(
            (np.diff(steer) / dt) ** 2))) if len(steer) > 1 else np.nan,
        "plan_time_mean_ms": 1e3 * float(np.mean(plan_times)),
        "plan_time_max_ms": 1e3 * float(np.max(plan_times)),
        "seed": seed,
        "opp_speed": params["opponent"]["speed_factor"],
        "noise": params["opponent"]["noise"]["enabled"],
    }
    return log, metrics, ff


def plot_run(log, metrics, ff, out_path):
    fig, axes = plt.subplots(1, 2, figsize=(15, 7),
                             gridspec_kw={"width_ratios": [1.5, 1]})
    ax = axes[0]
    ax.plot(ff.xy[:, 0], ff.xy[:, 1], "k--", lw=0.5, alpha=0.5,
            label="raceline")
    pts = ax.scatter(log["x"], log["y"], c=log["t"], cmap="plasma", s=3,
                     label="ego")
    ax.plot(log["opp_x"], log["opp_y"], "c-", lw=1.5, alpha=0.7,
            label="opponent")
    ax.plot(log["x"][0], log["y"][0], "g^", ms=10, label="ego start")
    fig.colorbar(pts, ax=ax, label="t [s]")
    ax.set_aspect("equal"); ax.legend()
    ax.set_title(f"{metrics['planner']}: {metrics['result']} "
                 f"(t={metrics['t_overtake_s']:.1f}s, "
                 f"min dist {metrics['min_dist_m']:.2f}m)")

    ax2 = axes[1]
    ax2.plot(log["t"], log["gap"], label="s-gap (ego-opp) [m]")
    ax2.plot(log["t"], log["dist"], label="euclid dist [m]")
    ax2.plot(log["t"], log["d"], label="ego d [m]")
    ax2.plot(log["t"], log["v"], label="ego v [m/s]", alpha=0.6)
    ax2.axhline(0, color="k", lw=0.5)
    ax2.set_xlabel("t [s]"); ax2.legend(); ax2.grid(alpha=0.3)
    fig.tight_layout()
    fig.savefig(out_path, dpi=140)
    plt.close(fig)


def _car_corners(x, y, theta, length, width):
    """차량 사각형 4꼭짓점 (map 좌표). 애니메이션 렌더용."""
    c, s = np.cos(theta), np.sin(theta)
    hl, hw = 0.5 * length, 0.5 * width
    local = np.array([[hl, hw], [hl, -hw], [-hl, -hw], [-hl, hw]])
    rot = np.array([[c, -s], [s, c]])
    return local @ rot.T + [x, y]


def animate_run(log, metrics, ff, params, out_path,
                fps=20, speed=1.0, window_m=14.0):
    """후처리 재생: 시뮬 로그를 follow-cam GIF로 저장.

    speed: 재생 배속 (2.0 = 실시간의 2배). fps와 함께 프레임 수를 결정:
      프레임 간격 = speed / (fps * dt). 파일이 크면 speed를 올리거나 fps를 내릴 것.
    window_m: ego 중심 카메라의 반경 [m]. 넓히면 코스 맥락이, 좁히면 차간
      상호작용이 잘 보인다.
    """
    from matplotlib import animation
    from matplotlib.patches import Polygon

    dt = float(log["t"][1] - log["t"][0])
    step = max(int(round(speed / (fps * dt))), 1)
    frames = range(0, len(log["t"]), step)
    veh = params["vehicle"]

    fig, ax = plt.subplots(figsize=(8, 8))
    # 트랙 경계: raceline ± (d_left/d_right) 법선 오프셋으로 복원
    left = ff.xy + ff.d_left[:, None] * ff.normal
    right = ff.xy - ff.d_right[:, None] * ff.normal
    for b in (left, right):
        b = np.vstack([b, b[:1]])  # 폐곡선 닫기
        ax.plot(b[:, 0], b[:, 1], "k-", lw=1.0, alpha=0.6)
    ax.plot(ff.xy[:, 0], ff.xy[:, 1], "--", color="gray", lw=0.7, alpha=0.6)
    ax.set_aspect("equal")
    ax.set_title(f"{metrics['planner']} · opp {metrics['opp_speed']:.2f}x · "
                 f"{metrics['result']}")

    ego_trail, = ax.plot([], [], "-", color="#d62728", lw=1.2)
    opp_trail, = ax.plot([], [], "-", color="#1f77b4", lw=1.2, alpha=0.7)
    ego_patch = Polygon(np.zeros((4, 2)), closed=True, fc="#d62728",
                        ec="k", lw=0.5, zorder=5, label="ego")
    opp_patch = Polygon(np.zeros((4, 2)), closed=True, fc="#1f77b4",
                        ec="k", lw=0.5, zorder=5, label="opponent")
    ax.add_patch(ego_patch)
    ax.add_patch(opp_patch)
    hud = ax.text(0.02, 0.98, "", transform=ax.transAxes, va="top",
                  fontsize=10, family="monospace",
                  bbox=dict(fc="white", alpha=0.75, ec="none"))
    ax.legend(loc="upper right", fontsize=9)

    opp_i = ff.idx_of_s(np.asarray(log["opp_s"]))
    opp_psi = ff.psi[opp_i]

    def draw(i):
        ego_trail.set_data(log["x"][:i + 1], log["y"][:i + 1])
        opp_trail.set_data(log["opp_x"][:i + 1], log["opp_y"][:i + 1])
        ego_patch.set_xy(_car_corners(log["x"][i], log["y"][i],
                                      log["theta"][i],
                                      veh["length_m"], veh["width_m"]))
        opp_patch.set_xy(_car_corners(log["opp_x"][i], log["opp_y"][i],
                                      opp_psi[i],
                                      veh["length_m"], veh["width_m"]))
        ax.set_xlim(log["x"][i] - window_m, log["x"][i] + window_m)
        ax.set_ylim(log["y"][i] - window_m, log["y"][i] + window_m)
        hud.set_text(f"t    {log['t'][i]:5.1f} s (x{speed:.0f})\n"
                     f"v    {log['v'][i]:5.2f} m/s\n"
                     f"gap  {log['gap'][i]:+5.1f} m\n"
                     f"mode {log['mode'][i]}")
        return ego_trail, opp_trail, ego_patch, opp_patch, hud

    anim = animation.FuncAnimation(fig, draw, frames=frames, blit=False)
    anim.save(out_path, writer=animation.PillowWriter(fps=fps), dpi=80)
    plt.close(fig)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--planner", choices=list(PLANNERS), default="path_selection")
    ap.add_argument("--raceline", default=None, help="raceline CSV 경로")
    ap.add_argument("--opp-speed", type=float, default=None)
    ap.add_argument("--noise", action="store_true", default=None)
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--plot", action="store_true")
    ap.add_argument("--animate", action="store_true",
                    help="후처리 재생 GIF 저장 (follow-cam)")
    ap.add_argument("--anim-speed", type=float, default=2.0,
                    help="재생 배속 (기본 2배속)")
    ap.add_argument("--anim-fps", type=int, default=20)
    args = ap.parse_args()

    params = load_params()
    csv = Path(args.raceline) if args.raceline else default_raceline()
    log, metrics, ff = run_h2h(args.planner, params, csv,
                               opp_speed=args.opp_speed, noise=args.noise,
                               seed=args.seed)
    for k, v in metrics.items():
        print(f"  {k}: {v}")
    if args.plot:
        out = HERE / "outputs" / f"h2h_{args.planner}_seed{args.seed}.png"
        plot_run(log, metrics, ff, out)
        print(f"  plot: {out}")
    if args.animate:
        out = HERE / "outputs" / f"h2h_{args.planner}_seed{args.seed}.gif"
        animate_run(log, metrics, ff, params, out,
                    fps=args.anim_fps, speed=args.anim_speed)
        print(f"  animation: {out}")


if __name__ == "__main__":
    main()
