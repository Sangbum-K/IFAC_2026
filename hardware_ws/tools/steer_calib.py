#!/usr/bin/env python3
"""조향 캘리브레이션 계산기 (f1tenth_stack / vesc.yaml)

변환식:  servo = gain * delta(rad) + offset
자전거 모델: delta_actual = atan(wheelbase / R)

사용법
  # 1) 현재 설정으로 clip 한계가 어디인지 보기
  python3 steer_calib.py limits

  # 2) 좌/우 선회 반경 측정값으로 gain·offset 동시 피팅
  #    (좌회전 명령각[rad], 좌회전 원 지름[m], 우회전 명령각[rad], 우회전 원 지름[m])
  python3 steer_calib.py fit 0.20 3.00 -0.20 3.20

  # 3) 직선주행 쏠림으로 offset만 미세보정
  #    (주행거리[m], 좌측쏠림[m] — 오른쪽으로 쏠렸으면 음수)
  python3 steer_calib.py trim 10.0 0.5
"""
import math
import os
import sys

import yaml

# vesc.yaml 을 직접 읽는다. 상수를 손으로 베껴두면 튜닝 도중 반드시 어긋난다.
# realpath 를 쓰므로 ~/steer_calib.py 같은 심볼릭 링크로 실행해도 원본 위치를 찾는다.
VESC_YAML = os.path.normpath(os.path.join(
    os.path.dirname(os.path.realpath(__file__)),
    '..', 'src', 'f1tenth_stack', 'config', 'vesc.yaml'))

with open(VESC_YAML, 'r') as _f:
    _cfg = yaml.safe_load(_f)
_shared = _cfg['/**']['ros__parameters']

GAIN = float(_shared['steering_angle_to_servo_gain'])
OFFSET = float(_shared['steering_angle_to_servo_offset'])
SERVO_MIN = float(_shared['servo_min'])
SERVO_MAX = float(_shared['servo_max'])
WHEELBASE = float(_cfg['vesc_to_odom_node']['ros__parameters']['wheelbase'])


def rad2deg(r):
    return r * 180.0 / math.pi


def limits(gain=GAIN, offset=OFFSET):
    """servo clip 때문에 실제로 낼 수 있는 조향각 범위."""
    d_at_min = (SERVO_MIN - offset) / gain
    d_at_max = (SERVO_MAX - offset) / gain
    lo, hi = sorted([d_at_min, d_at_max])
    sym = min(abs(lo), abs(hi))
    print(f"  gain={gain:+.4f}  offset={offset:.4f}  servo=[{SERVO_MIN}, {SERVO_MAX}]")
    print(f"  포화 없는 조향각 범위 : {lo:+.4f} ~ {hi:+.4f} rad "
          f"({rad2deg(lo):+.1f}° ~ {rad2deg(hi):+.1f}°)")
    print(f"  좌우 대칭 최대 조향각 : ±{sym:.4f} rad (±{rad2deg(sym):.1f}°)")
    print(f"  -> control_pkg 의 vehicle/max_steering_angle_rad 를 {sym:.3f} 이하로 두세요.")
    asym = abs(abs(lo) - abs(hi))
    if asym > 0.02:
        centered = (SERVO_MIN + SERVO_MAX) / 2
        print(f"  ! 좌우 비대칭 {rad2deg(asym):.1f}° — offset {offset:.4f} 이 "
              f"clip 중심 {centered:.4f} 에서 벗어나 있습니다.")
    return sym


def fit(d_cmd_l, diam_l, d_cmd_r, diam_r):
    """좌/우 2점으로 gain·offset 동시 피팅."""
    # 측정된 실제 조향각 (부호는 명령 방향을 따름)
    d_act_l = math.copysign(math.atan(WHEELBASE / (diam_l / 2.0)), d_cmd_l)
    d_act_r = math.copysign(math.atan(WHEELBASE / (diam_r / 2.0)), d_cmd_r)
    # 그때 실제로 나간 servo 값 (현재 gain/offset 으로 계산된 값)
    servo_l = GAIN * d_cmd_l + OFFSET
    servo_r = GAIN * d_cmd_r + OFFSET

    for nm, s in (("좌", servo_l), ("우", servo_r)):
        if not (SERVO_MIN <= s <= SERVO_MAX):
            print(f"  !! {nm}회전 servo={s:.4f} 가 clip 범위 밖입니다. "
                  f"명령각을 줄여서 다시 측정하세요. 결과 무효.")
            return

    gain_new = (servo_l - servo_r) / (d_act_l - d_act_r)
    offset_new = servo_l - gain_new * d_act_l

    print(f"  좌: 명령 {d_cmd_l:+.4f} rad -> 실제 {d_act_l:+.4f} rad "
          f"({rad2deg(d_act_l):+.1f}°, servo {servo_l:.4f})")
    print(f"  우: 명령 {d_cmd_r:+.4f} rad -> 실제 {d_act_r:+.4f} rad "
          f"({rad2deg(d_act_r):+.1f}°, servo {servo_r:.4f})")
    print()
    print(f"  steering_angle_to_servo_gain  : {GAIN:+.4f} -> {gain_new:+.4f}")
    print(f"  steering_angle_to_servo_offset: {OFFSET:.4f} -> {offset_new:.4f}")
    print()
    print("  새 값 기준 한계:")
    limits(gain_new, offset_new)


def trim(dist, drift_left):
    """직선주행 쏠림 -> offset 미세보정. drift_left>0 이면 왼쪽으로 쏠린 것."""
    # 호의 반경 R ~ s^2/(2d) -> delta_err = atan(wheelbase/R)
    if abs(drift_left) < 1e-9:
        print("  쏠림 0 — 보정 불필요.")
        return
    R = dist ** 2 / (2.0 * abs(drift_left))
    d_err = math.copysign(math.atan(WHEELBASE / R), drift_left)
    offset_new = OFFSET - GAIN * d_err
    print(f"  {dist:.1f} m 주행 중 {abs(drift_left):.2f} m "
          f"{'좌' if drift_left > 0 else '우'}측 쏠림")
    print(f"  -> 실효 조향 오차 {d_err:+.4f} rad ({rad2deg(d_err):+.2f}°)")
    print(f"  steering_angle_to_servo_offset: {OFFSET:.4f} -> {offset_new:.4f}")



def arc(ab, bc, ca):
    """경로 위 세 점의 상호 거리로 선회 반경 역산 (외접원).

    원을 다 그릴 공간이 없을 때 쓴다. 호를 돌면서 바닥에 세 점(출발/중간/끝)을
    찍고 세 변의 길이만 재면 된다. 결과 지름을 fit 에 그대로 넣으면 됨.
    """
    s = (ab + bc + ca) / 2.0
    k2 = s * (s - ab) * (s - bc) * (s - ca)
    if k2 <= 0:
        print("  !! 세 거리가 삼각형을 이루지 않습니다. 측정값을 확인하세요.")
        return
    area = math.sqrt(k2)
    R = ab * bc * ca / (4.0 * area)
    d_act = math.atan(WHEELBASE / R)
    print(f"  세 점 거리: {ab:.3f} / {bc:.3f} / {ca:.3f} m")
    print(f"  선회 반경 R = {R:.3f} m   (지름 {2 * R:.3f} m)")
    print(f"  실제 조향각 = {d_act:.4f} rad ({rad2deg(d_act):.2f}°)")
    print(f"  -> fit 에 넣을 지름 값: {2 * R:.3f}")
    return 2 * R


def plan(servo_lo=SERVO_MIN, servo_hi=SERVO_MAX):
    """조향각별 예상 원 지름 — 측정에 필요한 공간을 미리 확인."""
    print(f"  gain={GAIN:+.4f} offset={OFFSET:.4f} servo=[{servo_lo}, {servo_hi}]")
    print(f"  {'delta[rad]':>11} {'deg':>7} {'servo(L)':>9} {'servo(R)':>9} {'지름[m]':>9}  판정")
    d = 0.10
    while d <= 0.45:
        sl = GAIN * d + OFFSET          # 좌회전(+delta)
        sr = GAIN * (-d) + OFFSET       # 우회전(-delta)
        diam = 2.0 * WHEELBASE / math.tan(d)
        ok = (servo_lo <= sl <= servo_hi) and (servo_lo <= sr <= servo_hi)
        print(f"  {d:11.2f} {rad2deg(d):7.1f} {sl:9.4f} {sr:9.4f} {diam:9.2f}  "
              f"{'OK' if ok else 'clip!'}")
        d += 0.05

if __name__ == '__main__':
    if len(sys.argv) < 2:
        print(__doc__)
        sys.exit(1)
    cmd = sys.argv[1]
    a = [float(x) for x in sys.argv[2:]]
    if cmd == 'limits':
        limits()
    elif cmd == 'fit':
        fit(*a)
    elif cmd == 'trim':
        trim(*a)
    elif cmd == 'arc':
        arc(*a)
    elif cmd == 'plan':
        plan(*a)
    else:
        print(__doc__)
        sys.exit(1)
