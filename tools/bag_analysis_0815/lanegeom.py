#!/usr/bin/env python3
"""d=0 기준선이 무엇인지 + 라인스위치가 요구하는 실제 횡이동량 + 플래너 주기."""
import pickle
from pathlib import Path

import numpy as np

D = pickle.load(open(Path(__file__).parent / "data.pkl", "rb"))
gw = D["global_wpnts"][0][1]     # s d x y d_r d_l psi kappa vx ax
s, d_r, d_l, kap, vx = gw[:, 0], gw[:, 4], gw[:, 5], gw[:, 7], gw[:, 8]
w = np.abs(d_r) + np.abs(d_l)
# 기준선이 중앙선 대비 얼마나 치우쳤나 (+ = 왼쪽으로 치우침)
off = (np.abs(d_l) - np.abs(d_r)) / 2.0
# 중앙선은 기준선에서 d = +off 위치. (d_l 이 크면 왼쪽에 공간이 많다
# = 기준선이 오른쪽 벽에 붙어 있다 = 중앙선은 왼쪽(+d) 에 있다)

print("=== d=0 기준선 정체 ===")
print(f"n={len(gw)} s_max={s[-1]:.2f}m  폭 med {np.median(w):.2f} "
      f"min {w.min():.2f} max {w.max():.2f}")
print(f"|d_left| med {np.median(np.abs(d_l)):.2f}  |d_right| med {np.median(np.abs(d_r)):.2f}")
print(f"기준선의 중앙선 대비 치우침: med {np.median(off):+.3f} m  "
      f"mean {off.mean():+.3f}  min {off.min():+.3f} max {off.max():+.3f}")
print(f"  |치우침| p50 {np.percentile(np.abs(off),50):.3f}  "
      f"p90 {np.percentile(np.abs(off),90):.3f}  max {np.abs(off).max():.3f} m")
print(f"  중앙선이면 치우침 0. 관측 |치우침| 평균 {np.abs(off).mean():.3f} m "
      f"= 반폭({np.median(w)/2:.2f}m)의 {100*np.abs(off).mean()/(np.median(w)/2):.0f}%")
print("  -> d=0 은 중앙선이 아니라 레이스라인 (좌우 비대칭)")

# 직선 / 코너 분리
straight = np.abs(kap) < 0.15
print(f"\n직선(|kappa|<0.15) {straight.sum()}/{len(s)}점, "
      f"코너 {(~straight).sum()}점")
for m, lbl in ((straight, "직선"), (~straight, "코너")):
    print(f"  {lbl}: 치우침 med {np.median(off[m]):+.3f} "
          f"|치우침| med {np.median(np.abs(off[m])):.3f} p90 {np.percentile(np.abs(off[m]),90):.3f}  "
          f"폭 med {np.median(w[m]):.2f}")

print("\n=== s별 (2m 빈): 기준선 위치와 좌/우 가용 ===")
print("   s    kappa   폭   |d_L| |d_R|  치우침  안쪽전환필요")
for a in np.arange(0, s[-1], 2.0):
    m = (s >= a) & (s < a + 2)
    if not m.any():
        continue
    # '인코스'(공간 많은 쪽)로 옮길 때 필요한 횡이동 = 그쪽 벽까지 - 안전여유
    need_L = np.median(np.abs(d_l[m])) - 0.40
    need_R = np.median(np.abs(d_r[m])) - 0.40
    inner = max(need_L, need_R)
    print(f"{a:5.1f} {np.median(kap[m]):+7.3f} {np.median(w[m]):5.2f} "
          f"{np.median(np.abs(d_l[m])):5.2f} {np.median(np.abs(d_r[m])):5.2f} "
          f"{np.median(off[m]):+7.3f} {inner:11.2f}")

# ---- 실제 회피 경로가 요구한 횡이동 ----
print("\n=== 실제 발행된 회피경로(otwpnts)의 횡이동 ===")
ex = []
for ts, arr in D["otwpnts"]:
    if len(arr) < 3:
        continue
    dd = arr[:, 1]
    ex.append((ts, dd.min(), dd.max(), dd.max() - dd.min(), np.abs(dd).max(),
               arr[-1, 0] - arr[0, 0]))
E = np.array(ex)
print(f"발행 {len(E)}회 / 전체 {len(D['otwpnts'])}")
if len(E):
    print(f"  경로 내 d 진폭(max-min): med {np.median(E[:,3]):.2f} "
          f"p90 {np.percentile(E[:,3],90):.2f} max {E[:,3].max():.2f} m")
    print(f"  |d| 최대:            med {np.median(E[:,4]):.2f} "
          f"p90 {np.percentile(E[:,4],90):.2f} max {E[:,4].max():.2f} m")
    print(f"  경로 s 길이:          med {np.median(E[:,5]):.1f} m")

# 자차가 실제로 낸 |d|
fre = D["/car_state/frenet/odom"]
print(f"\n자차 실제 |d|: med {np.median(np.abs(fre[:,2])):.2f} "
      f"p90 {np.percentile(np.abs(fre[:,2]),90):.2f} max {np.abs(fre[:,2]).max():.2f} m")

# ---- 플래너 주기 (연산 시간 병목 여부) ----
print("\n=== 노드 발행 주기 (연산 병목 진단) ===")
for key, name in (("otwpnts", "/planner/avoidance/otwpnts"),
                  ("local_wpnts", "/local_waypoints")):
    ts = np.array([t for t, _ in D[key]])
    dt = np.diff(ts) * 1000
    print(f"{name:34s} n={len(ts):5d} rate {len(ts)/(ts[-1]-ts[0]):5.1f}Hz  "
          f"dt med {np.median(dt):5.1f} p95 {np.percentile(dt,95):6.1f} "
          f"p99 {np.percentile(dt,99):6.1f} max {dt.max():7.1f} ms")
for topic in ("/perception/obstacles", "/perception/obstacles_frenet"):
    key = "obs_perc" if topic.endswith("obstacles") else "obs_frenet"
    ts = np.array([t for t, _ in D[key]])
    dt = np.diff(ts) * 1000
    print(f"{topic:34s} n={len(ts):5d} rate {len(ts)/(ts[-1]-ts[0]):5.1f}Hz  "
          f"dt med {np.median(dt):5.1f} p95 {np.percentile(dt,95):6.1f} "
          f"p99 {np.percentile(dt,99):6.1f} max {dt.max():7.1f} ms")
sc = np.array([t for t, _, _, _ in D["scan"]])
dt = np.diff(sc) * 1000
print(f"{'/scan':34s} n={len(sc):5d} rate {len(sc)/(sc[-1]-sc[0]):5.1f}Hz  "
      f"dt med {np.median(dt):5.1f} p95 {np.percentile(dt,95):6.1f} "
      f"p99 {np.percentile(dt,99):6.1f} max {dt.max():7.1f} ms")
for name in ("/ackermann_cmd", "/autonomy/ackermann_cmd"):
    a = D[name]
    dt = np.diff(a[:, 0]) * 1000
    print(f"{name:34s} n={len(a):5d} rate {len(a)/(a[-1,0]-a[0,0]):5.1f}Hz  "
          f"dt med {np.median(dt):5.1f} p95 {np.percentile(dt,95):6.1f} "
          f"p99 {np.percentile(dt,99):6.1f} max {dt.max():7.1f} ms")

# 인지 -> 회피경로 반영 지연
print("\n=== 지연: 장애물 첫 관측 -> 회피경로 발행 ===")
seen = {}
for ts, lst in D["obs_frenet"]:
    for o in lst:
        seen.setdefault(o["id"], ts)
ot_t = np.array([t for t, a in D["otwpnts"] if len(a) >= 3])
lat = []
for oid, t0 in sorted(seen.items(), key=lambda x: x[1]):
    nxt = ot_t[ot_t >= t0]
    if len(nxt):
        lat.append(nxt[0] - t0)
lat = np.array(lat)
if len(lat):
    print(f"  n={len(lat)}  med {1000*np.median(lat):.0f} ms  "
          f"p90 {1000*np.percentile(lat,90):.0f}  max {1000*lat.max():.0f} ms")
