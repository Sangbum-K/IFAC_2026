#!/usr/bin/env python3
"""실측 그립/슬립 포락선. '빠를 때 조향하면 미끄러지는가'를 데이터로 판정.

방법:
  a_lat_meas = v * omega          (요레이트 x 속도 = 실제 구심가속도)
  omega_kin  = v * tan(delta)/L   (자전거 모델, 슬립 0 가정)
  응답비 R   = omega_meas/omega_kin  --> 1보다 작으면 언더스티어(전륜 슬립)
R이 a_lat과 함께 꺾이는 지점이 실제 그립 한계다.
"""
import pathlib
import pickle

import numpy as np

D = pickle.load(open(str(pathlib.Path(__file__).parent / "data.pkl"), "rb"))
L = 0.33
MAX_STEER = 0.36
G = 9.80665


def smooth(x, n):
    if n <= 1:
        return x
    k = np.ones(n) / n
    return np.convolve(x, k, mode="same")


def main():
    imu = D["/sensors/imu/raw"]          # t,yaw,wx,wy,wz,ax,ay,az  (wz deg/s, a in g)
    ack = D["/ackermann_cmd"]            # t, steer, speed
    spd = D["/monitor/current_speed"]    # t, v
    esc = D["/planner/escape_active"]

    ti = imu[:, 0]
    wz = np.radians(imu[:, 4])           # rad/s
    ay = imu[:, 6] * G                   # m/s^2
    az = imu[:, 7] * G

    print("=== IMU 위생 점검 ===")
    print(f"az 중앙값 {np.median(az):.2f} m/s^2  (중력 9.81 기대) -> "
          f"{'가속도 단위 g 확정' if 8 < np.median(az) < 11 else '주의: 축/단위 불명'}")
    print(f"wz  |p99| {np.percentile(np.abs(wz),99):.2f} rad/s "
          f"= {np.degrees(np.percentile(np.abs(wz),99)):.0f} deg/s")

    # 시간 정렬: IMU 격자에 속도·조향을 보간
    v = np.interp(ti, spd[:, 0], spd[:, 1])
    dcmd = np.interp(ti, ack[:, 0], ack[:, 1])
    esc_i = np.interp(ti, esc[:, 0], esc[:, 1]) > 0.5 if len(esc) else np.zeros(len(ti), bool)

    # 평활 (IMU 250Hz 급, 노이즈·노면 진동 제거: ~0.1s)
    n = max(1, int(0.10 / np.median(np.diff(ti))))
    wzs = smooth(wz, n)
    vs = smooth(v, n)
    ays = smooth(ay, n)

    a_lat_meas = vs * wzs                       # 구심가속도 [m/s^2]
    w_kin = vs * np.tan(dcmd) / L               # 슬립 0 예측 요레이트
    print(f"평활창 {n} 샘플 (~0.1s), 유효 샘플 {len(ti)}")

    # ay(가속도계)와 v*wz(운동학) 상호검증
    m = np.abs(a_lat_meas) > 1.0
    cc = np.corrcoef(ays[m], a_lat_meas[m])[0, 1]
    sl = np.polyfit(a_lat_meas[m], ays[m], 1)[0]
    print(f"교차검증 ay vs v*wz: 상관 {cc:.3f}, 기울기 {sl:.2f} "
          f"(1.0 근처면 두 경로가 일치)\n")

    # ---- 1) 조향각 대비 실제 달성 횡가속도 (속도대별) ----
    print("=== 1) 조향 명령 -> 실제 횡가속도 (속도 구간별) ===")
    print("여기서 요레이트가 조향에 비례해 계속 커지면 그립 여유 있음, "
          "평평해지면 한계 도달.\n")
    dbins = [(2, 4), (4, 6), (6, 8), (8, 10), (10, 12), (12, 14),
             (14, 16), (16, 18), (18, 20), (20, 21.1)]
    for vlo, vhi in [(1.5, 2.5), (2.5, 3.5), (3.5, 4.5), (4.5, 5.5), (5.5, 7.0)]:
        sel_v = (vs >= vlo) & (vs < vhi)
        if sel_v.sum() < 50:
            continue
        print(f"-- v {vlo}~{vhi} m/s  (n={sel_v.sum()}) --")
        print("   |δcmd|deg   n    a_lat실측(중앙)  요레이트실측  운동학예측  "
              "응답비R   R의 p25")
        for dlo, dhi in dbins:
            ad = np.degrees(np.abs(dcmd))
            s = sel_v & (ad >= dlo) & (ad < dhi) & (np.abs(w_kin) > 0.05)
            if s.sum() < 25:
                continue
            R = np.abs(wzs[s]) / np.abs(w_kin[s])
            print(f"   {dlo:5.1f}-{dhi:<4.1f} {s.sum():5d}   "
                  f"{np.median(np.abs(a_lat_meas[s])):8.2f}      "
                  f"{np.median(np.abs(wzs[s])):7.2f}     "
                  f"{np.median(np.abs(w_kin[s])):7.2f}    "
                  f"{np.median(R):6.2f}    {np.percentile(R,25):5.2f}")
        print()

    # ---- 2) 달성 횡가속도의 분포 = 실제 그립 포락선 ----
    print("=== 2) 실제 달성한 횡가속도 분포 (그립 포락선) ===")
    al = np.abs(a_lat_meas)
    for q in (50, 75, 90, 95, 99, 99.5, 99.9):
        print(f"   p{q:<5} {np.percentile(al, q):6.2f} m/s^2")
    print(f"   max   {al.max():6.2f} m/s^2")
    print(f"   >6.6 인 시간 비율: {100*np.mean(al>6.6):.1f}%   "
          f">7.5: {100*np.mean(al>7.5):.1f}%   >8.5: {100*np.mean(al>8.5):.1f}%")
    print()

    # ---- 3) 응답비가 꺾이는 지점 = 그립 한계 ----
    print("=== 3) 횡가속도별 조향 응답비 R (꺾이는 곳이 그립 한계) ===")
    print("   a_lat구간    n     R중앙   R_p25   R_p10   판정")
    edges = [0.5, 1.5, 2.5, 3.5, 4.5, 5.5, 6.5, 7.5, 8.5, 9.5, 11.0, 14.0]
    base = None
    for lo, hi in zip(edges[:-1], edges[1:]):
        s = (al >= lo) & (al < hi) & (np.abs(w_kin) > 0.15) & (vs > 1.5)
        if s.sum() < 40:
            continue
        R = np.abs(wzs[s]) / np.abs(w_kin[s])
        med = np.median(R)
        if base is None:
            base = med
        drop = 100 * (1 - med / base)
        flag = "선형" if drop < 12 else ("완만한 포화" if drop < 30 else "포화")
        print(f"   {lo:4.1f}-{hi:<5.1f} {s.sum():6d}  {med:6.2f}  "
              f"{np.percentile(R,25):6.2f}  {np.percentile(R,10):6.2f}   "
              f"{flag} (기준대비 {drop:+.0f}%)")
    print()

    # ---- 4) 조향 포화(≥85%) 구간에서 실제로 무슨 일이 있었나 ----
    print("=== 4) 조향 포화 구간(|δ|>=85% of 0.36rad = 17.6deg)의 실제 거동 ===")
    sat = np.abs(dcmd) >= 0.85 * MAX_STEER
    print(f"   포화 시간 비율 {100*np.mean(sat):.1f}%  ({sat.sum()} 샘플)")
    for vlo, vhi in [(1.5, 3.0), (3.0, 4.0), (4.0, 5.0), (5.0, 7.0)]:
        s = sat & (vs >= vlo) & (vs < vhi)
        if s.sum() < 20:
            continue
        R = np.abs(wzs[s]) / np.maximum(np.abs(w_kin[s]), 1e-3)
        print(f"   v {vlo}~{vhi}: n={s.sum():5d}  a_lat중앙 "
              f"{np.median(np.abs(a_lat_meas[s])):5.2f}  최대 "
              f"{np.abs(a_lat_meas[s]).max():5.2f}  R중앙 {np.median(R):.2f}")
    print()

    # ---- 5) 회피(escape) 중 vs 평상시 ----
    print("=== 5) escape_active 중 vs 평상시 ===")
    for nm, s in (("escape 중", esc_i), ("평상시", ~esc_i)):
        if s.sum() < 50:
            continue
        print(f"   {nm}: n={s.sum():6d}  v중앙 {np.median(vs[s]):4.2f}  "
              f"|δ|중앙 {np.degrees(np.median(np.abs(dcmd[s]))):5.2f}deg  "
              f"|δ|p99 {np.degrees(np.percentile(np.abs(dcmd[s]),99)):5.2f}deg  "
              f"a_lat p95 {np.percentile(np.abs(a_lat_meas[s]),95):5.2f}")
    print()

    # ---- 6) 조향각 히스토그램: 기계 한계까지 얼마나 남겼나 ----
    print("=== 6) 조향 명령 사용 분포 (기계 한계 21.07deg 대비) ===")
    ad = np.degrees(np.abs(dcmd))
    for q in (50, 90, 95, 99, 99.9):
        print(f"   p{q:<5} {np.percentile(ad,q):5.2f} deg "
              f"({100*np.percentile(ad,q)/21.07:.0f}% of 기계한계)")
    print(f"   max   {ad.max():5.2f} deg ({100*ad.max()/21.07:.0f}%)")
    print(f"   20.6deg(=0.36rad) 이상 시간비율 {100*np.mean(ad>=20.6):.2f}%")


if __name__ == "__main__":
    main()
