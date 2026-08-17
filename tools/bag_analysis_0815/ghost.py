#!/usr/bin/env python3
"""지목 시각에 실제로 '유령 장애물'이 생겼는지 + 그게 지면 히트인지 판정.

판정법:
  1) /perception/obstacles(_frenet) 스트림에서 해당 시각의 장애물 등장/소멸 추적
  2) 맵 레이캐스트 vs 실측 스캔 -> '맵보다 짧게 돌아온' 빔 추출
  3) 짧은 빔들이 방위 전반에 퍼지고 거리가 거의 일정하면 지면 히트
     (LiDAR 높이 h=0.1374 m -> 피치 th 에서 지면까지 r = h/sin(th))
"""
import pickle
from pathlib import Path

import numpy as np

D = pickle.load(open(Path(__file__).parent / "data.pkl", "rb"))
pf = D["/pf/pose/odom"]              # t x y yaw v
fre = D["/car_state/frenet/odom"]
MAP = D["map"]
H_LIDAR = 0.1374
X_LIDAR = 0.28

grid = MAP["data"].reshape(MAP["h"], MAP["w"])
occ = grid > 50            # 점유
res, ox, oy = MAP["res"], MAP["ox"], MAP["oy"]
print(f"map {MAP['w']}x{MAP['h']} res={res} origin=({ox:.2f},{oy:.2f}) "
      f"occupied={occ.sum()}")


def raycast(x0, y0, ang, rmax=10.0):
    """맵에서 (x0,y0) 에서 ang 방향 첫 점유 셀까지 거리."""
    step = res * 0.5
    n = int(rmax / step)
    ca, sa = np.cos(ang), np.sin(ang)
    ds = np.arange(1, n + 1) * step
    xs = ((x0 + ca * ds) - ox) / res
    ys = ((y0 + sa * ds) - oy) / res
    ix = xs.astype(np.int32)
    iy = ys.astype(np.int32)
    good = (ix >= 0) & (ix < MAP["w"]) & (iy >= 0) & (iy < MAP["h"])
    hit = np.zeros(n, bool)
    hit[good] = occ[iy[good], ix[good]]
    k = np.argmax(hit) if hit.any() else -1
    return ds[k] if k >= 0 else rmax


def analyze_scan(ts, amin, ainc, r):
    """한 스캔에 대해 맵 대비 짧은 빔 통계."""
    i = np.searchsorted(pf[:, 0], ts)
    i = min(max(i, 0), len(pf) - 1)
    px, py, pyaw = pf[i, 1], pf[i, 2], pf[i, 3]
    lx = px + X_LIDAR * np.cos(pyaw)
    ly = py + X_LIDAR * np.sin(pyaw)

    n = len(r)
    ang = amin + ainc * np.arange(n)
    ok = np.isfinite(r) & (r > 0.05) & (r < 10.0)
    idx = np.where(ok & (np.abs(ang) < np.radians(100)))[0]
    if len(idx) < 20:
        return None
    idx = idx[::2]                       # 추가 데시메이션
    exp = np.array([raycast(lx, ly, pyaw + ang[j]) for j in idx])
    meas = r[idx]
    short = meas < exp - 0.30            # 맵 벽보다 30 cm 이상 앞에서 반사
    return dict(t=ts, ang=ang[idx], meas=meas, exp=exp, short=short,
                n_short=int(short.sum()), n=len(idx))


def report(t0, t1, label, stride=2):
    print(f"\n===== {label}  [{t0}, {t1}] =====")
    print("   t    짧은빔  방위폭°  거리 med  거리 std  피치추정°  최원거리")
    for ts, amin, ainc, r in D["scan"]:
        if not (t0 <= ts <= t1):
            continue
        a = analyze_scan(ts, amin, ainc, r)
        if a is None:
            continue
        if a["n_short"] == 0:
            print(f"{ts:6.2f}      0")
            continue
        sm = a["meas"][a["short"]]
        sa = np.degrees(a["ang"][a["short"]])
        span = sa.max() - sa.min()
        med = np.median(sm)
        # 지면 히트라면 r = h/sin(pitch)
        pitch = np.degrees(np.arcsin(np.clip(H_LIDAR / max(med, 1e-3), 0, 1)))
        print(f"{ts:6.2f} {a['n_short']:6d} {span:8.1f} {med:9.2f} {sm.std():9.2f} "
              f"{pitch:9.2f} {a['meas'].max():9.2f}")


# ---- 장애물 스트림 ----
print("\n########## 장애물 스트림 (frenet) ##########")


def obs_report(t0, t1, label):
    print(f"\n--- {label} ---")
    prev = set()
    for ts, lst in D["obs_frenet"]:
        if not (t0 <= ts <= t1):
            continue
        cur = {o["id"] for o in lst}
        new = cur - prev
        gone = prev - cur
        if new or gone or not lst:
            det = "  ".join(
                f"#{o['id']}{'S' if o['is_static'] else 'D'}"
                f"@s{o['s_center']:.1f},d{o['d_center']:+.2f}"
                f",sz{o['size']:.2f}{'' if o['is_visible'] else ',INV'}"
                for o in lst)
            print(f"{ts:6.2f} new={sorted(new)} gone={sorted(gone)} | {det}")
        prev = cur


for lbl, a, b in (("34.3", 32.5, 35.5), ("46.6", 44.5, 47.5),
                  ("60.6", 58.5, 61.5), ("105-108", 104.0, 108.5)):
    obs_report(a, b, lbl)

print("\n\n########## 맵 대비 짧은 빔 (지면 히트 판정) ##########")
for lbl, a, b in (("34.3 지목", 33.2, 34.8), ("46.6 지목", 45.6, 47.0),
                  ("60.6 지목", 59.9, 61.2), ("정상 주행 기준(20-21s)", 20.0, 21.0)):
    report(a, b, lbl)
