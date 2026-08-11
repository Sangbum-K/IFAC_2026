"""Global trajectory generation + simulation on a track map.

이 스크립트는 planning_pkg 안의 오프라인 개발 도구다 (.venv로 실행, colcon
빌드 대상 아님). 맵을 읽어 raceline/centerline/walls CSV를 만들어
planning_pkg/config/ 아래에 저장하면, 런타임 노드(planning_node)와
RViz 시각화 노드(raceline_viz_node)가 그걸 읽는다.

경로는 planning_pkg 루트 기준으로 자동 해석된다 (팀 워크스페이스 어디에
두어도 동작). 맵은 워크스페이스 공용 폴더 <ws>/maps 에서 읽는다 —
mapping.launch.py(mapp)가 저장하는 그 폴더라 복사가 필요 없다.
대회 맵은 --maps-dir 로 조직위 맵 폴더를 가리키면 된다.

Usage (planning_pkg 안에서 .venv python으로):
    python scripts/global_traj_sim/step4_main.py --map Spielberg_map
"""

import argparse
import sys
from pathlib import Path

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

from step1_track import Track, extract_centerline
from step2_global_trajectory import (build_trajectory, optimize_raceline_iqp,
                                     roll_to_start, write_csv)
from step3_simulate import PurePursuitSim

HERE = Path(__file__).resolve().parent
# planning_pkg 루트를 위로 탐색 (scripts/global_traj_sim/ -> planning_pkg/)
PKG_ROOT = next((p for p in HERE.parents if p.name == "planning_pkg"),
                HERE.parent.parent)
# <ws>/planning_ws/src/planning_pkg -> <ws> (deployment_profiles.py와 같은 규칙)
WORKSPACE_ROOT = PKG_ROOT.parents[2]
# 워크스페이스 공용 맵 폴더 = 맵의 단일 위치. mapping.launch.py(mapp)가 저장하고
# localization / raceline_tuner / sim_race launch가 전부 여기를 읽는다.
# (planning_pkg/maps 사본은 폐지 — 같은 이름 맵이 두 벌이면 어느 쪽이 쓰였는지
#  조용히 갈린다.) 대회 맵은 --maps-dir 로 가리킨다.
MAPS_DIR = WORKSPACE_ROOT / "maps"
PLANNER_CSV_DIR = PKG_ROOT / "config" / "global_trajectory"  # raceline 산출물
PLANNER_CENTERLINE_DIR = PKG_ROOT / "config" / "centerline"  # centerline/walls 산출물


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument(
        "--allow-unsafe-legacy", action="store_true",
        help="검증 전 CSV를 쓰는 구 파이프라인을 명시적으로 허용 (비권장)")
    ap.add_argument("--map", default="Spielberg_map")
    ap.add_argument("--maps-dir", default=str(MAPS_DIR),
                    help="맵 yaml/png 폴더 (대회 맵 폴더로 교체 가능)")
    ap.add_argument("--raceline", choices=["center", "iqp", "sp"], default="iqp",
                    help="center=중앙선 그대로, iqp=최소곡률 최적화 (TUM 스타일), "
                         "sp=최단거리 (추월 라인용)")
    ap.add_argument("--no-sp", action="store_true",
                    help="iqp 생성 시 SP 추월 라인 CSV 동시 생성 끄기")
    ap.add_argument("--safe-dist", type=float, default=0.30,
                    help="IQP에서 벽과 확보할 거리 (차 반폭+여유)")
    ap.add_argument("--v-max", type=float, default=7.0)
    ap.add_argument("--mu", type=float, default=0.85)
    ap.add_argument("--margin", type=float, default=0.15)
    ap.add_argument("--laps", type=int, default=1)
    ap.add_argument("--out", default=str(HERE / "outputs"))
    args = ap.parse_args()
    if not args.allow_unsafe_legacy:
        raise SystemExit(
            "step4_main.py는 검증 전에 CSV를 저장하는 구형 도구라 비활성화됐습니다.\n"
            "안전 게이트가 포함된 build_map.py를 사용하세요:\n"
            f"  python {HERE / 'build_map.py'} {args.map}\n"
            "정말 구형 결과 재현이 필요할 때만 --allow-unsafe-legacy를 추가하세요.")
    maps_dir = Path(args.maps_dir)

    out_dir = Path(args.out)
    out_dir.mkdir(parents=True, exist_ok=True)

    # 1) track + centerline ------------------------------------------------
    print(f"[1/4] Loading map '{args.map}' and extracting centerline ...")
    track = Track(maps_dir / f"{args.map}.yaml")
    center = extract_centerline(track)
    boundaries = center.pop("boundaries")
    center = roll_to_start(center, start_xy=(0.0, 0.0))
    print(f"      centerline: {len(center['s'])} points, "
          f"track length {center['s'][-1]:.1f} m")

    # wall boundary CSV (RViz 3D 벽 시각화 노드가 읽음: x; y; wall_id)
    outer_b, inner_b = boundaries
    walls = np.vstack([
        np.column_stack([outer_b, np.zeros(len(outer_b))]),     # wall_id 0
        np.column_stack([inner_b, np.ones(len(inner_b))]),      # wall_id 1
    ])
    walls_name = f"{args.map}_walls.csv"
    for dest in (out_dir / walls_name, PLANNER_CENTERLINE_DIR / walls_name):
        Path(dest).parent.mkdir(parents=True, exist_ok=True)
        np.savetxt(dest, walls, delimiter="; ", fmt="%.7f",
                   header="x_m; y_m; wall_id(0=outer,1=inner)")
    print(f"      wrote wall boundaries ({walls_name})")

    # 2) global trajectory -------------------------------------------------
    if args.raceline == "iqp":
        print("[2/4] IQP minimum-curvature raceline + velocity profile ...")
        geo = optimize_raceline_iqp(center, boundaries,
                                    safe_dist_m=args.safe_dist)
        geo = roll_to_start(geo, start_xy=(0.0, 0.0))
    elif args.raceline == "sp":
        print("[2/4] SP shortest-path raceline + velocity profile ...")
        geo = optimize_raceline_iqp(center, boundaries,
                                    safe_dist_m=args.safe_dist,
                                    objective="length")
        geo = roll_to_start(geo, start_xy=(0.0, 0.0))
    else:
        print("[2/4] Building velocity profile (centerline raceline) ...")
        geo = center
    traj = build_trajectory(geo, v_max=args.v_max, mu=args.mu,
                            margin=args.margin)
    tag = {"iqp": "iqp_", "sp": "sp_"}.get(args.raceline, "")
    csv_name = (f"{args.map}_{tag}max{args.v_max:.2f}ms_"
                f"margin{args.margin:.2f}m_mu{args.mu:.2f}.csv")
    for dest in (out_dir / csv_name, PLANNER_CSV_DIR / csv_name):
        write_csv(traj, dest, header_note=f"generated by global_traj_sim ({args.map})")
        print(f"      wrote {dest}")

    # 추월 라인: 메인(iqp) 생성 시 SP 라인도 함께 저장
    # (UNICORN이 global_waypoints.json에 IQP+SP 둘 다 담는 것과 같은 구조.
    #  H2H lane-change 플래너가 추월 차선으로 사용할 예정)
    if args.raceline == "iqp" and not args.no_sp:
        print("      generating SP overtake line ...")
        geo_sp = optimize_raceline_iqp(center, boundaries,
                                       safe_dist_m=args.safe_dist,
                                       objective="length")
        geo_sp = roll_to_start(geo_sp, start_xy=(0.0, 0.0))
        traj_sp = build_trajectory(geo_sp, v_max=args.v_max, mu=args.mu,
                                   margin=args.margin)
        sp_name = (f"{args.map}_sp_max{args.v_max:.2f}ms_"
                   f"margin{args.margin:.2f}m_mu{args.mu:.2f}.csv")
        for dest in (out_dir / sp_name, PLANNER_CSV_DIR / sp_name):
            write_csv(traj_sp, dest,
                      header_note=f"generated by global_traj_sim ({args.map}, SP overtake line)")
            print(f"      wrote {dest}")
        sp_len = traj_sp["s"][-1]
        iqp_len = traj["s"][-1]
        print(f"      경로 길이: IQP {iqp_len:.1f} m vs SP {sp_len:.1f} m "
              f"({iqp_len - sp_len:+.1f} m)")

    # planner_visualizer_node용 centerline CSV (config/centerline/에서 로드,
    # 파란 반투명 LINE_STRIP로 RViz에 표시됨. x;y 2열이면 충분)
    # 주의: raceline이 iqp여도 여기엔 항상 '진짜 중앙선'을 저장한다
    center_name = f"{args.map}_centerline.csv"
    PLANNER_CENTERLINE_DIR.mkdir(parents=True, exist_ok=True)
    for dest in (out_dir / center_name, PLANNER_CENTERLINE_DIR / center_name):
        np.savetxt(dest, center["xy"], delimiter="; ", fmt="%.7f",
                   header="x_m; y_m")
        print(f"      wrote {dest}")

    # 3) closed-loop simulation --------------------------------------------
    print(f"[3/4] Simulating {args.laps} lap(s) (pure pursuit + kinematic bicycle) ...")
    sim = PurePursuitSim(traj)
    log = sim.run(n_laps=args.laps)
    lap_time = log["t"][-1]
    print(f"      lap time: {lap_time / max(log['laps'], 1):.2f} s per lap, "
          f"v_mean {log['v'].mean():.2f} m/s, v_max {log['v'].max():.2f} m/s, "
          f"|lat err| mean {np.abs(log['d_err']).mean():.3f} m / "
          f"max {np.abs(log['d_err']).max():.3f} m")

    # 4) plots ---------------------------------------------------------------
    print("[4/4] Rendering plots ...")
    outer, inner = boundaries

    fig, ax = plt.subplots(figsize=(11, 8))
    ax.plot(*outer.T, "k-", lw=1)
    ax.plot(*inner.T, "k-", lw=1)
    if args.raceline == "iqp":  # 비교용: 중앙선을 회색 점선으로
        ax.plot(center["xy"][:, 0], center["xy"][:, 1], "--", color="gray",
                lw=0.8, alpha=0.7, label="centerline")
    pts = ax.scatter(traj["xy"][:, 0], traj["xy"][:, 1], c=traj["vx"],
                     cmap="viridis", s=4, label=f"{args.raceline} raceline (vx)")
    ax.plot(log["x"], log["y"], "r-", lw=0.8, alpha=0.8, label="driven path")
    ax.plot(traj["xy"][0, 0], traj["xy"][0, 1], "b*", ms=14, label="start (s=0)")
    fig.colorbar(pts, ax=ax, label="vx [m/s]")
    ax.set_aspect("equal"); ax.legend(loc="upper right")
    ax.set_title(f"{args.map}: global trajectory & simulated lap "
                 f"({lap_time / max(log['laps'],1):.2f} s/lap)")
    ax.set_xlabel("x [m]"); ax.set_ylabel("y [m]")
    fig.tight_layout()
    fig.savefig(out_dir / f"{args.map}_trajectory.png", dpi=150)

    fig, axes = plt.subplots(3, 1, figsize=(11, 9), sharex=True)
    axes[0].plot(log["s"], log["v"], label="v actual")
    axes[0].plot(log["s"], log["v_ref"], "--", label="v reference")
    axes[0].set_ylabel("v [m/s]"); axes[0].legend()
    axes[1].plot(log["s"], log["d_err"])
    axes[1].axhline(0, color="k", lw=0.5)
    axes[1].set_ylabel("lateral error [m]")
    axes[2].plot(log["s"], np.degrees(log["steer"]))
    axes[2].set_ylabel("steer [deg]"); axes[2].set_xlabel("s [m]")
    axes[0].set_title(f"{args.map}: closed-loop tracking")
    fig.tight_layout()
    fig.savefig(out_dir / f"{args.map}_tracking.png", dpi=150)

    print(f"      saved plots to {out_dir}")


if __name__ == "__main__":
    main()
