#!/usr/bin/env python3
"""두 사고의 기전 확정.
   (1) s=26~31 헤어핀2 출구: 계획 d(s) 프로파일이 우벽 수축 구간에서 남기는 예비.
   (2) t=108~112 직선 회피: 계획 d(t) 대 실제 d(t) — 오버슈트인가 추종 지연인가.
"""
import pickle
from pathlib import Path

import numpy as np

P = Path(__file__).parent
D = pickle.load(open(P / "data.pkl", "rb"))
S = np.load(P / "scanclear.npy")
gw = D["global_wpnts"][0][1]
gs = gw[:, 0]
A_l, A_r = np.abs(gw[:, 5]), np.abs(gw[:, 4])
ds = gs[1] - gs[0]
L = gs[-1] + ds
fre = D["/car_state/frenet/odom"]
cmd = D["/autonomy/ackermann_cmd"]
HALF = 0.15


def wl(s_):
    return A_l[(np.round((np.asarray(s_) % L) / ds).astype(int)) % len(gw)]


def wr(s_):
    return A_r[(np.round((np.asarray(s_) % L) / ds).astype(int)) % len(gw)]


# ---------- (1) 랩별로, 자차가 s=27.0 을 지날 때 발행된 계획의 d(s) ----------
print("=== (1) s=27.0 통과 시점의 발행 계획 d(s) 와 벽 예비 (랩별) ===")
print("    각 칸 = 계획 d @ 그 s / 그 지점 벽예비(= 벽 - |d| - 0.15)")
grid = np.arange(27.0, 31.01, 0.5)
print("  t     " + "".join(f"  s={g:<9.1f}" for g in grid))
seen_t = []
for t, w in D["local_wpnts"]:
    cs = np.interp(t, fre[:, 0], fre[:, 1])
    if not (26.9 <= cs <= 27.1):
        continue
    if seen_t and t - seen_t[-1] < 2.0:
        continue
    seen_t.append(t)
    rel = (w[:, 0] - cs + L / 2) % L - L / 2
    line = f"{t:6.1f}  "
    for g in grid:
        k = np.abs(w[:, 0] - g).argmin()
        if abs(w[k, 0] - g) > 0.3:
            line += f"{'--':>12s}"
            continue
        d = w[k, 1]
        wall = wl(g) if d >= 0 else wr(g)
        line += f" {d:+.2f}/{wall - abs(d) - HALF:+.2f} "
    print(line)

print("\n  실제 주행 d(s) (같은 랩들):")
print("  lap   " + "".join(f"  s={g:<9.1f}" for g in grid))
cuts = np.where(np.diff(fre[:, 0]) > 900)[0]
for li, t0 in enumerate(seen_t):
    line = f"{t0:6.1f}  "
    for g in grid:
        m = (fre[:, 0] >= t0 - 1) & (fre[:, 0] <= t0 + 12)
        if not m.any():
            line += f"{'--':>12s}"
            continue
        sub = fre[m]
        k = np.abs(sub[:, 1] - g).argmin()
        if abs(sub[k, 1] - g) > 0.25:
            line += f"{'--':>12s}"
            continue
        d = sub[k, 2]
        wall = wl(g) if d >= 0 else wr(g)
        line += f" {d:+.2f}/{wall - abs(d) - HALF:+.2f} "
    print(line)

# ---------- (2) 두 번째 사고 구간 ----------
print("\n=== (2) t=108.0~112.0  계획 d vs 실제 d (직선 회피 -> 좌벽 접촉) ===")
print("    t     s     d_act  d_plan   err  |  좌벽  예비(실제) | steer  v_act v_cmd")
plan_t, plan_d = [], []
for t, w in D["local_wpnts"]:
    if not (107.5 <= t <= 112.5) or len(w) < 2:
        continue
    cs = np.interp(t, fre[:, 0], fre[:, 1])
    rel = (w[:, 0] - cs + L / 2) % L - L / 2
    k = np.abs(rel).argmin()
    plan_t.append(t)
    plan_d.append(w[k, 1])
plan_t, plan_d = np.array(plan_t), np.array(plan_d)
m = (fre[:, 0] >= 108.0) & (fre[:, 0] <= 112.0)
for i in np.where(m)[0][::4]:
    t, s_, d_, v = fre[i, 0], fre[i, 1], fre[i, 2], fre[i, 4]
    dp = np.interp(t, plan_t, plan_d) if len(plan_t) else np.nan
    wall = wl(s_) if d_ >= 0 else wr(s_)
    print(f"{t:8.2f} {s_:6.2f} {d_:+6.2f} {dp:+6.2f} {d_-dp:+6.2f}  | "
          f"{wl(s_):5.2f} {wall - abs(d_) - HALF:+6.2f} | "
          f"{np.degrees(np.interp(t, cmd[:,0], cmd[:,1])):+6.1f} {v:5.2f} "
          f"{np.interp(t, cmd[:,0], cmd[:,2]):5.2f}")

# ---------- (3) 계획 예비의 s 분포 : 어디서 0 에 붙는가 ----------
print("\n=== (3) 발행 계획이 남기는 벽 예비의 s별 최소값 (전 랩) ===")
best = {}
for t, w in D["local_wpnts"]:
    if len(w) < 2:
        continue
    i = (np.round((w[:, 0] % L) / ds).astype(int)) % len(gw)
    wall = np.where(w[:, 1] >= 0, A_l[i], A_r[i])
    mar = wall - np.abs(w[:, 1]) - HALF
    for sv, mv in zip(w[:, 0], mar):
        b = int(sv)
        if b not in best or mv < best[b]:
            best[b] = mv
print("  s : 계획 최소예비")
for b in sorted(best):
    if best[b] < 0.15:
        print(f" {b:3d} : {best[b]:+.3f}   {'#' * int(max(0, 40 - best[b] * 200))}")
