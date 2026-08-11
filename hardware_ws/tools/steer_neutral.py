#!/usr/bin/env python3
"""주행 rosbag 에서 **하드웨어가 실제로 직진하는 서보 위치 S\*** 를 역산한다.

왜 필요한가
-----------
`vesc.yaml` 의 `steering_angle_to_servo_offset` 은 "직진하려면 서보를 여기 두라"는
*명령*이고, S\* 는 그 차가 실제로 직진하는 서보 위치 = *하드웨어의 성질*이다.
캘리브레이션이 맞다는 건 offset == S\* 라는 뜻이다.

TUNING_STATUS.md §11 이 기록한 문제는 **S\* 가 런 사이에 최대 0.036 servo(2.5°)씩
뛴다**는 것이다. 한 런 안에서는 안정적인데 런 사이에만 튀므로 서서히 풀리는
헐거움이 아니라 이산적 기계 미끄러짐(서보 세이버 회전, 혼 스플라인 밀림,
타이로드/너클 재안착)의 서명이고, yaml 상수로는 못 쫓아간다.

이 스크립트는 §11 이 손으로 하던 계산을 재현한다. 판정 기준도 거기 있다
(★4: 연속 3런에서 S\* 가 현행 offset ±0.005 안).

방법
----
1. `/commands/servo/position` 과 `/ackermann_cmd` 를 시간정렬해
   `servo = gain*delta + offset` 을 최소제곱으로 적합 -> **그 런에 실제로 돌고
   있던** gain/offset 을 백에서 직접 얻는다 (yaml 을 믿지 않는다).
2. 요레이트 wz 와 속도 v 로 wz/v 대 delta 를 Theil-Sen 적합하고, wz=0 이 되는
   delta_0 를 구한다. 직진 조향 명령이 곧 중립의 어긋남이다.
3. S\* = gain*delta_0 + offset_실행.

⚠️ `/sensors/imu/raw` 의 gyro 는 VESC 드라이버가 **deg/s** 로 낸다
   (vesc.yaml 의 imu_angular_velocity_scale = pi/180 이 그 용도). rad/s 로 바꿔 쓴다.

사용법
------
    python3 ~/steer_neutral.py <bag_dir> [<bag_dir> ...]

여러 백을 주면 런 사이 점프를 바로 표로 보여준다 — 그게 이 문제의 핵심이므로
한 번에 3런을 넘기는 것을 권장한다.
"""

import glob
import math
import os
import sqlite3
import sys

import numpy as np

# --- ROS 메시지 역직렬화 -----------------------------------------------------
try:
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message
except ImportError:
    sys.exit("ROS 2 환경을 먼저 소싱할 것 (zsh: source /opt/ros/humble/setup.zsh)")

# wz/v 대 delta 적합에 쓸 구간. 큰 조향은 비선형(타이어 슬립)이라 제외하고,
# 저속은 wz/v 가 0/0 으로 발산하므로 제외한다. §11 과 같은 값.
DELTA_ABS_MAX = 0.12
SPEED_MIN = 2.5
# |lat_err| 교차검증 문턱 — 추종이 나쁜 구간이 추정을 오염시켰는지 본다.
LAT_ERR_MAX = 0.15
# 런 안 안정성을 보는 슬라이딩 윈도.
WINDOW_SEC = 15.0
DEG2RAD = math.pi / 180.0


def read_bag(bag_dir):
    """bag_dir 의 모든 .db3 에서 필요한 토픽을 (시각, 값) 배열로 뽑는다."""
    files = sorted(glob.glob(os.path.join(bag_dir, "*.db3")))
    if not files:
        raise SystemExit(f"{bag_dir}: .db3 가 없다")

    want = {
        "/commands/servo/position": "servo",
        "/ackermann_cmd": "delta",
        "/sensors/imu/raw": "wz",
        "/pf/pose/odom": "speed",
        "/pitwall/lat_err": "lat_err",
    }
    out = {k: ([], []) for k in want.values()}

    for path in files:
        con = sqlite3.connect(path)
        topics = {}          # topic_id -> (key, msg_class)
        for tid, name, type_str in con.execute(
                "select id, name, type from topics"):
            if name in want:
                topics[tid] = (want[name], get_message(type_str))
        if not topics:
            con.close()
            continue
        ids = ",".join(str(i) for i in topics)
        for tid, stamp, data in con.execute(
                f"select topic_id, timestamp, data from messages "
                f"where topic_id in ({ids}) order by timestamp"):
            key, cls = topics[tid]
            msg = deserialize_message(bytes(data), cls)
            if key == "servo":
                val = msg.data
            elif key == "delta":
                val = msg.drive.steering_angle
            elif key == "wz":
                # VESC 드라이버가 deg/s 로 낸다 -> rad/s
                val = msg.angular_velocity.z * DEG2RAD
            elif key == "speed":
                t = msg.twist.twist.linear
                val = math.hypot(t.x, t.y)
            else:
                val = msg.data
            out[key][0].append(stamp * 1e-9)
            out[key][1].append(val)
        con.close()

    return {k: (np.asarray(t), np.asarray(v)) for k, (t, v) in out.items()}


def resample(t_ref, t_src, v_src, max_gap=0.05):
    """t_ref 시각에 가장 가까운 t_src 표본을 붙인다. max_gap 초과는 NaN."""
    if len(t_src) == 0:
        return np.full_like(t_ref, np.nan, dtype=float)
    idx = np.searchsorted(t_src, t_ref)
    idx = np.clip(idx, 1, len(t_src) - 1)
    left, right = t_src[idx - 1], t_src[idx]
    pick = np.where(np.abs(t_ref - left) <= np.abs(right - t_ref), idx - 1, idx)
    out = v_src[pick].astype(float)
    out[np.abs(t_src[pick] - t_ref) > max_gap] = np.nan
    return out


def theil_sen(x, y, max_pairs=200000, seed=0):
    """Theil-Sen 기울기/절편. 쌍이 너무 많으면 무작위 표본으로 근사."""
    n = len(x)
    rng = np.random.default_rng(seed)
    if n * (n - 1) // 2 <= max_pairs:
        i, j = np.triu_indices(n, k=1)
    else:
        i = rng.integers(0, n, max_pairs)
        j = rng.integers(0, n, max_pairs)
        keep = i != j
        i, j = i[keep], j[keep]
    dx = x[j] - x[i]
    ok = np.abs(dx) > 1e-9
    slope = np.median((y[j][ok] - y[i][ok]) / dx[ok])
    intercept = np.median(y - slope * x)
    return slope, intercept


def analyse(bag_dir):
    data = read_bag(bag_dir)
    t_d, delta = data["delta"]
    if len(t_d) < 100:
        raise SystemExit(f"{bag_dir}: /ackermann_cmd 표본이 너무 적다")

    servo = resample(t_d, *data["servo"])
    wz = resample(t_d, *data["wz"])
    speed = resample(t_d, *data["speed"])
    lat = resample(t_d, *data["lat_err"], max_gap=0.2)

    # --- 1) 이 런에 실제로 돌고 있던 gain/offset 을 백에서 복원 --------------
    m = np.isfinite(servo) & np.isfinite(delta)
    A = np.column_stack([delta[m], np.ones(m.sum())])
    (gain_eff, offset_eff), *_ = np.linalg.lstsq(A, servo[m], rcond=None)
    resid = servo[m] - (gain_eff * delta[m] + offset_eff)
    resid_rms = float(np.sqrt(np.mean(resid ** 2)))

    # --- 2) wz=0 이 되는 조향 명령 delta_0 -----------------------------------
    def fit(mask, seed=0, min_n=200):
        x, y = delta[mask], wz[mask] / speed[mask]
        if len(x) < min_n:
            return None
        s, b = theil_sen(x, y, seed=seed)
        if abs(s) < 1e-9:
            return None
        return -b / s, len(x)

    base = (np.isfinite(wz) & np.isfinite(speed) & np.isfinite(delta)
            & (np.abs(delta) < DELTA_ABS_MAX) & (speed > SPEED_MIN))
    got = fit(base)
    if got is None:
        raise SystemExit(
            f"{bag_dir}: |delta|<{DELTA_ABS_MAX}, v>{SPEED_MIN} 표본이 부족하다. "
            "직선 구간을 더 오래 주행한 백이 필요하다.")
    delta0, n_used = got
    s_star = gain_eff * delta0 + offset_eff

    # 추종 좋은 구간만으로 교차검증
    clean = base & np.isfinite(lat) & (np.abs(lat) < LAT_ERR_MAX)
    got_clean = fit(clean)
    s_star_clean = (gain_eff * got_clean[0] + offset_eff) if got_clean else None

    # 부트스트랩 불확실도
    boots = []
    idx = np.flatnonzero(base)
    rng = np.random.default_rng(0)
    for k in range(40):
        pick = rng.choice(idx, len(idx), replace=True)
        mask = np.zeros_like(base)
        mask[np.unique(pick)] = True
        g = fit(mask, seed=k)
        if g:
            boots.append(gain_eff * g[0] + offset_eff)
    boot_sd = float(np.std(boots)) if len(boots) > 5 else float("nan")

    # --- 3) 런 안 안정성: 15초 슬라이딩 윈도 --------------------------------
    t0 = t_d[base][0]
    wins = []
    start = t0
    while start < t_d[base][-1] - WINDOW_SEC:
        w = base & (t_d >= start) & (t_d < start + WINDOW_SEC)
        # 좁은 트랙이라 |delta|<0.12 & v>2.5 표본이 드물다. 윈도에는 본 적합보다
        # 낮은 문턱을 쓰되, 너무 낮추면 윈도 sd 가 잡음이 되므로 60 이 하한이다.
        g = fit(w, min_n=60)
        if g:
            wins.append(gain_eff * g[0] + offset_eff)
        start += WINDOW_SEC
    win_sd = float(np.std(wins)) if len(wins) >= 2 else float("nan")

    return dict(
        bag=os.path.basename(os.path.normpath(bag_dir)),
        gain_eff=gain_eff, offset_eff=offset_eff, resid_rms=resid_rms,
        delta0=delta0, s_star=s_star, n_used=n_used,
        s_star_clean=s_star_clean, boot_sd=boot_sd,
        win_sd=win_sd, n_win=len(wins), wins=wins,
        lat_p50=float(np.nanmedian(np.abs(lat))) if np.isfinite(lat).any() else float("nan"),
        duration=float(t_d[-1] - t_d[0]),
    )


def main(argv):
    if len(argv) < 2:
        sys.exit(__doc__)
    results = []
    for bag in argv[1:]:
        try:
            r = analyse(bag)
        except SystemExit as e:
            print(f"[skip] {e}")
            continue
        results.append(r)
        print(f"\n=== {r['bag']}  ({r['duration']:.0f} s) ===")
        print(f"  백에서 복원한 실행 캘리브레이션 : gain {r['gain_eff']:+.4f} / "
              f"offset {r['offset_eff']:.4f}  (잔차 rms {r['resid_rms']:.5f})")
        print(f"  직진이 되는 조향 명령 delta_0   : {r['delta0']:+.4f} rad "
              f"({math.degrees(r['delta0']):+.2f} deg)   표본 {r['n_used']}")
        print(f"  ** 하드웨어 실제 중립 S*        : {r['s_star']:.4f} **")
        if r['s_star_clean'] is not None:
            print(f"     (|lat_err|<{LAT_ERR_MAX} 만: {r['s_star_clean']:.4f} "
                  f"— 전체값과 가까우면 추종 오염 아님)")
        print(f"  부트스트랩 불확실도             : ±{r['boot_sd']:.4f}")
        print(f"  런 안 안정성 ({WINDOW_SEC:.0f}s 윈도 {r['n_win']}개) : "
              f"sd {r['win_sd']:.4f}")
        if r['wins']:
            # 계단처럼 한 번 튀고 유지되면 주행 중 미끄러짐, 들쭉날쭉하면 추정 잡음.
            print("     윈도별 S* : "
                  + " ".join(f"{w:.4f}" for w in r['wins']))
        print(f"  |lat_err| p50                   : {r['lat_p50']:.3f} m")

    if len(results) >= 2:
        ss = [r['s_star'] for r in results]
        spread = max(ss) - min(ss)
        print("\n=== 런 사이 비교 (이게 핵심) ===")
        for r in results:
            print(f"  {r['bag']:28s} S* {r['s_star']:.4f}")
        print(f"  최대-최소 : {spread:.4f} servo = "
              f"{spread/abs(results[0]['gain_eff'])*57.29578:.2f} deg")
        # 판정은 **추정 불확실도**(부트스트랩) 대비로 한다. 윈도 sd 는 표본이
        # 60~120 개뿐인 짧은 창에서 나오므로 추정 잡음이 부풀려져 있어, 그걸
        # 기준으로 삼으면 진짜 점프를 놓친다 (§11 도 부트스트랩과 비교했다).
        worst_boot = max(r['boot_sd'] for r in results if math.isfinite(r['boot_sd']))
        print(f"  런별 추정 불확실도(부트스트랩) 최대 : ±{worst_boot:.4f}")
        ratio = spread / worst_boot if worst_boot > 0 else float('inf')
        print(f"  => 런 사이 변동이 측정 정밀도의 {ratio:.0f}배")
        if ratio >= 5:
            print("     이산적 기계 미끄러짐 신호. yaml 상수로는 못 쫓아간다.")
        else:
            print("     측정 정밀도 수준. 이번 세트에서는 점프 징후 없음.")

    # ★4 판정 — 현행 yaml offset 기준 ±0.005
    if results:
        off = results[-1]['offset_eff']
        lo, hi = off - 0.005, off + 0.005
        print(f"\n=== 판정 (TUNING_STATUS §11 ★4) ===")
        print(f"  기준: 연속 3런의 S* 가 {off:.4f} ±0.005 = [{lo:.4f}, {hi:.4f}]")
        npass = 0
        for r in results:
            ok = lo <= r['s_star'] <= hi
            npass += ok
            verdict = "통과" if ok else f"벗어남 ({r['s_star'] - off:+.4f})"
            print(f"    {r['bag']:28s} S* {r['s_star']:.4f}  {verdict}")
        print(f"  => {npass}/{len(results)} 런 통과"
              + ("  — '고쳤다'고 말할 수 없다." if npass < 3 else "  — 기준 충족."))


if __name__ == "__main__":
    main(sys.argv)
