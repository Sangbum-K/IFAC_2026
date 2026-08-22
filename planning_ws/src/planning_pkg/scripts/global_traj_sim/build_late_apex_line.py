#!/usr/bin/env python3
"""my_track0822_4용 직선 중심 late-apex 레이스라인 생성기."""

import argparse
from pathlib import Path
import sys

import numpy as np

# 현재 실차 YAML의 0.380/0.322 rad를 이 프로세스의 배포 검증 모델에만 반영한다.
# 공용 설정 파일은 사용자의 진행 중 변경이라 건드리지 않는다.
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import deployment_profiles as deployment
deployment.MAX_LEFT_STEER = 0.3800
deployment.MAX_RIGHT_STEER = 0.3220
deployment.MAX_STEER = min(deployment.MAX_LEFT_STEER, deployment.MAX_RIGHT_STEER)

import build_map as bm


MAP_NAME = "my_track0822_4"
CSV_NAME = "my_track0822_4_iqp_max9.00ms_margin0.08m_mu1.40.csv"
MU_PROFILE = 1.40

# CCW 진행. 꼭짓점 사이를 직선으로 달리고 각 꼭짓점만 원호로 잇는다.
VERTICES = np.array([
    [3.60, -0.08],
    [3.60, 4.90],
    [-2.00, 3.65],
    [-10.65, 4.90],
    [-10.65, -0.08],
])
RADII = np.full(len(VERTICES), 1.18)


def rounded_polygon(vertices, radii, step=0.025):
    """폐폴리곤의 직선은 그대로 두고 꼭짓점만 접하는 원호로 치환한다."""
    vertices = np.asarray(vertices, dtype=float)
    radii = np.broadcast_to(np.asarray(radii, dtype=float), len(vertices))
    fillets = []
    for i, (point, radius) in enumerate(zip(vertices, radii)):
        incoming = point - vertices[i - 1]
        outgoing = vertices[(i + 1) % len(vertices)] - point
        incoming /= np.linalg.norm(incoming)
        outgoing /= np.linalg.norm(outgoing)
        turn = np.arctan2(np.cross(incoming, outgoing),
                          np.dot(incoming, outgoing))
        tangent = radius * np.tan(abs(turn) / 2.0)
        tangent = min(tangent,
                      0.45 * np.linalg.norm(point - vertices[i - 1]),
                      0.45 * np.linalg.norm(vertices[(i + 1) % len(vertices)] - point))
        effective_radius = tangent / max(np.tan(abs(turn) / 2.0), 1e-9)
        entry = point - tangent * incoming
        exit_ = point + tangent * outgoing
        normal = np.array([-incoming[1], incoming[0]]) * np.sign(turn)
        center = entry + effective_radius * normal
        start_angle = np.arctan2(*(entry - center)[::-1])
        count = max(3, int(np.ceil(abs(turn) * effective_radius / step)))
        angles = start_angle + np.linspace(0.0, turn, count, endpoint=True)
        arc = center + effective_radius * np.column_stack([np.cos(angles), np.sin(angles)])
        fillets.append((entry, exit_, arc))

    pieces = []
    for i, (_, exit_, arc) in enumerate(fillets):
        pieces.append(arc[:-1])
        next_entry = fillets[(i + 1) % len(fillets)][0]
        length = np.linalg.norm(next_entry - exit_)
        count = max(2, int(np.ceil(length / step)))
        pieces.append(np.linspace(exit_, next_entry, count, endpoint=False))
    return np.vstack(pieces)


def uniform_geometry(grid, spacing=bm.DS_OUT):
    raw = rounded_polygon(VERTICES, RADII)
    closed = np.vstack([raw, raw[0]])
    segment = np.linalg.norm(np.diff(closed, axis=0), axis=1)
    cumulative = np.concatenate([[0.0], np.cumsum(segment)])
    length = cumulative[-1]
    count = max(16, int(round(length / spacing)))
    target = np.linspace(0.0, length, count, endpoint=False)
    xy = np.column_stack([
        np.interp(target, cumulative, closed[:, axis]) for axis in range(2)
    ])

    previous = np.roll(xy, 1, axis=0)
    following = np.roll(xy, -1, axis=0)
    psi = np.arctan2(following[:, 1] - previous[:, 1],
                     following[:, 0] - previous[:, 0])
    incoming = xy - previous
    outgoing = following - xy
    chord = following - previous
    cross = incoming[:, 0] * outgoing[:, 1] - incoming[:, 1] * outgoing[:, 0]
    denominator = (np.linalg.norm(incoming, axis=1)
                   * np.linalg.norm(outgoing, axis=1)
                   * np.linalg.norm(chord, axis=1))
    kappa = 2.0 * cross / np.maximum(denominator, 1e-12)
    s = np.concatenate([[0.0], np.cumsum(np.linalg.norm(np.diff(xy, axis=0), axis=1))])
    a_lo, a_hi = grid.corridor_interval(xy, psi)
    return {
        "s": s,
        "xy": xy,
        "psi": psi,
        "kappa": kappa,
        "d_left": np.maximum(a_hi, 0.0),
        "d_right": np.maximum(-a_lo, 0.0),
    }


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--check", action="store_true", help="검증만 하고 CSV는 쓰지 않음")
    args = parser.parse_args()

    outer, inner, resolution, origin, raw_ring, skeleton_ring, height = bm.load_track(MAP_NAME)
    grid = bm.Grid(raw_ring, resolution, origin, height, skeleton_ring=skeleton_ring)
    center = bm.roll_to_start(bm.extract_centerline(grid), orientation="ccw")
    geo = bm.roll_to_start(uniform_geometry(grid), orientation="ccw")
    trajectory = bm.build_trajectory(geo, mu=MU_PROFILE)

    ok, messages = bm.verify_line("LATE-APEX", trajectory, grid)
    for message in messages:
        print(message)
    straight = np.sum(np.r_[np.diff(trajectory["s"]), bm.DS_OUT]
                      [np.abs(trajectory["kappa"]) < 0.04])
    print(f"      직선(|kappa|<0.04): {straight:.2f} / {trajectory['s'][-1]:.2f} m")
    conservative_radius = 0.33 / np.tan(0.3035)
    line_radius = 1.0 / np.max(np.abs(trajectory["kappa"]))
    if line_radius < conservative_radius:
        ok = False
        print(f"      [FAIL] 보수 조향 0.3035 rad 기준 R_min "
              f"{line_radius:.2f} < {conservative_radius:.2f} m")
    if not ok:
        bm._save_plots(MAP_NAME, trajectory, center, outer, inner, "ccw",
                       float(trajectory["psi"][0]), grid=grid,
                       suffix="_late_apex_FAILED")
        raise SystemExit("static validation failed")

    profiles, advisory = deployment.selected_controller_profiles("real")
    sim_ok, sim_messages, log, failed_log = bm.simulate_controller_profiles(
        "LATE-APEX", trajectory, grid, bm.LAPS, profiles,
        advisory_profiles=advisory)
    for message in sim_messages:
        print(message)
    walls = np.vstack([
        np.column_stack([outer, np.zeros(len(outer))]),
        np.column_stack([inner, np.ones(len(inner))]),
    ])
    bm._save_plots(MAP_NAME, trajectory, center, outer, inner, "ccw",
                   float(trajectory["psi"][0]), grid=grid,
                   log=failed_log or log,
                   suffix="_late_apex_CHECK" if args.check else "")
    if not sim_ok:
        raise SystemExit("closed-loop validation failed")
    if args.check:
        return

    bm.commit_generation_set(
        MAP_NAME, CSV_NAME, trajectory, center, walls, "ccw",
        sp_name="my_track0822_4_sp_max9.00ms_margin0.08m_mu1.40.csv",
        sp_traj=None, sp_disabled_reason="custom late-apex main line",
        validation={
            "geometry": "straight segments with circular late-apex corners",
            "mu_profile": MU_PROFILE,
            "gate_target": "real",
            "gate_profiles": [profile["name"] for profile in profiles],
            "advisory_profiles": [profile["name"] for profile in advisory],
            "backstop": deployment.backstop_metrics(
                log, deployment.controller_profile(log["profile_name"])),
        })
    print(Path(bm.CSV_DIR) / CSV_NAME)


if __name__ == "__main__":
    main()
