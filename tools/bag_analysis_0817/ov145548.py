#!/usr/bin/env python3
"""run_0817_145548 개요: 랩·정지·수동개입·충돌 후보 타임라인."""
import pickle
import numpy as np

D = pickle.load(open("d145548.pkl", "rb"))

od = D["/odom"]
fr = D["/car_state/frenet/odom"]
ack = D["/ackermann_cmd"]
tel = D["/teleop"]
imu = D["/sensors/imu/raw"]

print("=== 랩 경계 (frenet s 되감김) ===")
prev = None
for t, s in fr[:, [0, 1]]:
    if prev is not None and prev - s > 19:
        print(f"  t={t:7.2f}  lap++")
    prev = s

print("\n=== 정지 구간 (v_odom<0.05 가 1.5s 이상) ===")
v = od[:, 4]
stopped = v < 0.05
runs = []
i = 0
while i < len(od):
    if stopped[i]:
        j = i
        while j < len(od) and stopped[j]:
            j += 1
        if od[j - 1, 0] - od[i, 0] > 1.5:
            k = np.argmin(np.abs(fr[:, 0] - od[i, 0]))
            runs.append((od[i, 0], od[j - 1, 0], fr[k, 1]))
        i = j
    else:
        i += 1
for a, b, s in runs:
    print(f"  {a:7.2f} ~ {b:7.2f}  ({b-a:5.1f}s)  s={s:.2f}")

print("\n=== teleop 개입 ===")
if len(tel):
    # teleop 메시지가 나온 구간
    gaps = np.where(np.diff(tel[:, 0]) > 1.0)[0]
    seg_starts = [0] + list(gaps + 1)
    seg_ends = list(gaps) + [len(tel) - 1]
    for a, b in zip(seg_starts, seg_ends):
        print(f"  {tel[a,0]:7.2f} ~ {tel[b,0]:7.2f}  n={b-a+1}  "
              f"v[{tel[a:b+1,2].min():.2f},{tel[a:b+1,2].max():.2f}]")

print("\n=== 충돌 후보 (|ax| 급증 or |gyro_z| 급증) ===")
t_imu = imu[:, 0]
axl = imu[:, 5]
for i in range(1, len(imu)):
    if abs(axl[i]) > 8.0:
        # 앞뒤 0.3초에 이미 보고했으면 스킵
        print(f"  t={t_imu[i]:7.2f}  ax={axl[i]:+6.2f}  wz={imu[i,3]:+5.2f}")

print("\n=== 랩 기억 이벤트 (rosout) ===")
for t, lvl, name, msg in D["rosout"]:
    if "기억" in msg or "poison" in msg.lower() or "carv" in msg.lower():
        print(f"{t:7.2f} [{lvl}] {msg[:160]}")
