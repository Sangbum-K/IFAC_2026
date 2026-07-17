"""두 local planner의 Monte Carlo 배치 비교.

시나리오 축:
  planner      x  상대 속도 비율  x  관측 노이즈  x  시드(시작 위치 랜덤)
  (2)             (0.6/0.75/0.9)    (off/on)       (N회)

메트릭: 추월 성공률, 충돌률, 추월 소요시간, 최소 거리(안전),
        조향 변화율 RMS(승차감/서보 부담), planner 계산시간(NX 예산)

실행:
  .venv/bin/python planning_ws/global_traj_sim/compare.py --seeds 5
  .venv/bin/python planning_ws/global_traj_sim/compare.py --seeds 3 --jobs 4
"""

import argparse
import csv
import itertools
from multiprocessing import Pool
from pathlib import Path

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

from step10_simulate_h2h import (PLANNERS, default_raceline, load_params, run_h2h)

HERE = Path(__file__).resolve().parent

OPP_SPEEDS = [0.6, 0.75, 0.9]
NOISES = [False, True]


def _one_run(job):
    planner, opp_speed, noise, seed, csv_path = job
    params = load_params()
    try:
        _, m, _ = run_h2h(planner, params, csv_path, opp_speed=opp_speed,
                          noise=noise, seed=seed)
    except Exception as e:  # 한 런 실패가 배치 전체를 죽이지 않게
        m = {"planner": planner, "result": f"error:{e}", "seed": seed,
             "opp_speed": opp_speed, "noise": noise}
    return m


def aggregate(rows):
    """(planner, opp_speed, noise) 그룹별 요약 통계."""
    out = []
    key = lambda r: (r["planner"], r["opp_speed"], r["noise"])
    for k, grp in itertools.groupby(sorted(rows, key=key), key=key):
        grp = list(grp)
        n = len(grp)
        succ = [r for r in grp if r["result"] == "overtake"]
        coll = [r for r in grp if r["result"] == "collision"]
        t_ovt = [r["t_overtake_s"] for r in succ]
        out.append({
            "planner": k[0], "opp_speed": k[1], "noise": k[2], "n": n,
            "success_rate": len(succ) / n,
            "collision_rate": len(coll) / n,
            "t_overtake_mean_s": float(np.mean(t_ovt)) if t_ovt else np.nan,
            "min_dist_mean_m": float(np.nanmean(
                [r.get("min_dist_m", np.nan) for r in grp])),
            "steer_rate_rms_mean": float(np.nanmean(
                [r.get("steer_rate_rms_radps", np.nan) for r in grp])),
            "plan_ms_mean": float(np.nanmean(
                [r.get("plan_time_mean_ms", np.nan) for r in grp])),
        })
    return out


def plot_summary(summary, out_path):
    fig, axes = plt.subplots(1, 3, figsize=(16, 5))
    colors = {"path_selection": "#1f77b4", "frenet": "#d62728"}
    width = 0.35
    for ni, noise in enumerate(NOISES):
        for mi, (metric, label) in enumerate([
                ("success_rate", "overtake success rate"),
                ("t_overtake_mean_s", "time to overtake [s]"),
                ("collision_rate", "collision rate")]):
            ax = axes[mi]
            for pi, planner in enumerate(PLANNERS):
                vals = [next((r[metric] for r in summary
                              if r["planner"] == planner
                              and r["opp_speed"] == sp
                              and r["noise"] == noise), np.nan)
                        for sp in OPP_SPEEDS]
                x = np.arange(len(OPP_SPEEDS)) + (pi - 0.5) * width
                hatch = "//" if noise else None
                ax.bar(x, vals, width * 0.9, color=colors[planner],
                       alpha=0.9 if not noise else 0.45, hatch=hatch,
                       label=f"{planner}{' +noise' if noise else ''}"
                             if mi == 0 else None)
            ax.set_xticks(np.arange(len(OPP_SPEEDS)))
            ax.set_xticklabels([f"opp {s:.2f}x" for s in OPP_SPEEDS])
            ax.set_title(label)
            ax.grid(axis="y", alpha=0.3)
    axes[0].legend(fontsize=8)
    axes[0].set_ylim(0, 1.05)
    axes[2].set_ylim(0, 1.05)
    fig.suptitle("H2H local planner comparison (Spielberg)")
    fig.tight_layout()
    fig.savefig(out_path, dpi=140)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--seeds", type=int, default=5, help="조건당 반복 횟수")
    ap.add_argument("--jobs", type=int, default=4)
    ap.add_argument("--raceline", default=None)
    args = ap.parse_args()

    csv_path = Path(args.raceline) if args.raceline else default_raceline()
    jobs = [(pl, sp, nz, seed, csv_path)
            for pl in PLANNERS for sp in OPP_SPEEDS for nz in NOISES
            for seed in range(args.seeds)]
    print(f"running {len(jobs)} scenarios with {args.jobs} workers ...")
    with Pool(args.jobs) as pool:
        rows = pool.map(_one_run, jobs)

    out_dir = HERE / "outputs"
    raw_path = out_dir / "h2h_compare_raw.csv"
    with open(raw_path, "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=sorted({k for r in rows for k in r}))
        w.writeheader()
        w.writerows(rows)

    summary = aggregate(rows)
    sum_path = out_dir / "h2h_compare_summary.csv"
    with open(sum_path, "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=list(summary[0]))
        w.writeheader()
        w.writerows(summary)

    hdr = (f"{'planner':<16}{'opp':>5}{'noise':>7}{'n':>4}{'succ%':>8}"
           f"{'coll%':>8}{'t_ovt[s]':>10}{'mindist':>9}{'steerRMS':>10}"
           f"{'plan[ms]':>10}")
    print("\n" + hdr)
    print("-" * len(hdr))
    for r in summary:
        print(f"{r['planner']:<16}{r['opp_speed']:>5.2f}"
              f"{str(r['noise']):>7}{r['n']:>4}"
              f"{100*r['success_rate']:>7.0f}%{100*r['collision_rate']:>7.0f}%"
              f"{r['t_overtake_mean_s']:>10.2f}{r['min_dist_mean_m']:>9.2f}"
              f"{r['steer_rate_rms_mean']:>10.3f}{r['plan_ms_mean']:>10.2f}")

    plot_summary(summary, out_dir / "h2h_compare.png")
    print(f"\nwrote {raw_path}\n      {sum_path}\n      "
          f"{out_dir / 'h2h_compare.png'}")


if __name__ == "__main__":
    main()
