#!/usr/bin/env python3
"""배포 컨트롤러 프로파일 + 실차 종방향 plant — 런타임 YAML이 단일 기준.

왜 이 모듈이 따로 있나
----------------------
`build_map.py`(레이스라인 생성)와 `tuner/controller_safety_model.py`(라이브
튜너)는 같은 "배포 제어기 계약"으로 폐루프 검증을 돌려야 한다. 예전에는 두
파일이 각자 숫자를 베껴 적고 AST 계약 검사로 어긋남을 잡으려 했는데,
2026-08-05 시점에 **세 벌이 전부 서로 달랐다**:

    build_map.py          planner_speed_scale 0.97 / max_speed 6.0
    controller_safety…py  planner_speed_scale 0.83 / max_speed 5.0
    실제 런타임 YAML      velocity_scale     0.742 / max_speed 4.0

즉 CSV는 5.8 m/s 조건으로 검증받고, 튜너는 4.15 m/s 조건으로 판정하고, 차는
4.0 m/s로 달렸다. 베껴 적는 한 이 드리프트는 반드시 다시 생긴다.

그래서 사본을 없애고, 프로파일은 '어느 YAML의 어느 env를 읽을지'만 선언한다.
숫자는 launch가 읽는 바로 그 파일에서 온다.

의존성은 numpy + pyyaml 뿐이다. build_map은 map 생성용 venv(scipy/skimage/PIL/
matplotlib)에서, 튜너는 시스템 ROS Python에서 돌기 때문에 무거운 것을 넣으면
한쪽이 못 쓴다.
"""

from pathlib import Path

import numpy as np
import yaml

G = 9.81

# --- 경로 ---------------------------------------------------------------------
HERE = Path(__file__).resolve().parent                  # <planning_pkg>/scripts
PKG_ROOT = HERE.parent                                  # <planning_pkg>
WORKSPACE_ROOT = PKG_ROOT.parents[2]                    # <ws>
CONTROL_PKG_ROOT = WORKSPACE_ROOT / "control_ws" / "src" / "control_pkg"
VESC_YAML = (WORKSPACE_ROOT / "hardware_ws" / "src" / "f1tenth_stack" /
             "config" / "vesc.yaml")
STEERING_LUT_DIR = "control_ws/src/control_pkg/config/steering_lookup"

# --- 차량 실측 상수 (차 자체의 물리. 트랙·설정과 무관) ------------------------
# 런타임 YAML(control vehicle/*)이 이 값과 어긋나면 resolve 단계에서 죽는다 —
# '검증한 차'와 '달리는 차'가 다르면 PASS가 아무 것도 보장하지 않기 때문이다.
WHEELBASE = 0.33        # [m] 실차 실측 (hardware vesc.yaml wheelbase와 동일)
# [rad] 서보 기구의 방향별 배포 한계. hardware vesc.yaml 에서 직접 유도한다:
#   delta = (servo - offset) / gain, gain -0.8564 / offset 0.424
#   우 (0.70-0.424)/-0.8564 = -0.322279 rad (18.45°) <- 대칭 한계를 결정
#   좌 (0.05-0.424)/-0.8564 = +0.436712 rad (25.02°)
# 우측 0.3220은 servo_max 안쪽으로 조금 내린 배포값이다.
# 좌측은 servo_min에 여유를 둔 0.3800 rad(servo 0.0986)를 사용한다.
# ⚠️ 그래서 TUNING_STATUS §11 의 중립 튐(런 사이 최대 0.036 servo = 0.042 rad)을
#    흡수할 마진이 이제 없다. 중립이 우측으로 밀린 런에서는 최대 조향 근처 명령이
#    servo_max 에서 잘려 "가장 크게 꺾어야 할 때 안 꺾이는" 형태로 나타난다.
MAX_LEFT_STEER = 0.3800
MAX_RIGHT_STEER = 0.3220
# 경로 생성과 기존 폐루프 안전 모델은 아직 대칭이므로 구속 방향을 사용한다.
MAX_STEER = min(MAX_LEFT_STEER, MAX_RIGHT_STEER)
MAX_STEER_RATE = 2.0    # [rad/s] control 기본 steering-rate limit

# 종방향 plant 한계 — run_0805_2*·run_0806_1* 6개 백 실측 (2026-08-06 재계산,
# 속도 PID 포화 구간만: cmd-erpm 갭 > 0.5 m/s AND 모터 전류 > 15 A):
#   가속: 레이스 속도대(2.5~5.5 m/s)에서 p50 +1.2~1.3, p90 +1.7~1.8. 전류 중앙값
#     24~25 A — 0731의 2.2(p90 2.23)는 지금 조건에선 재현 안 됨(배터리·속도대 차이).
#     2.2로 계획하면 차가 프로파일에 못 붙어 코너 탈출마다 속도 오차만 쌓인다.
#   감속: 스로틀-오프(전류<3 A, 브레이크 0)만으로 p50 -1.5 (0731 실측과 일치).
# ⚠️ 이 둘은 '컨트롤러가 명령할 수 있는 기울기'가 아니라 '차가 실제로 낼 수 있는
#    가감속'이다. 명령 slew 한계(control vehicle/max_accel_mps2 5.5,
#    max_decel_mps2 4.0)는 런타임 YAML에서 따로 읽어 컨트롤러 단에만 적용한다.
#
# 2026-08-07 jaldallim 97813be 가 3.5 로 올렸던 것을 되돌린다 (c252707 -> 이 커밋).
# 3.5 는 실측이 아니라 외삽이었고, 그 커밋 본문도 "실차 투입 전 확인 필요"로
# 명시했다. 0805+0806 백 7개 재측정치(전류>15 A & 가속중, 표본 10796, 전류
# 중앙값 23.6 A)는:
#      통짜        p50 1.01 · p90 1.42 · p95 1.53 · max 2.90
#      속도대별 p90  2.5~3.0 1.50 | 3.0~3.5 1.48 | 3.5~4.0 1.14
#                   4.0~4.5 1.31 | 4.5~5.0 1.33 | 5.0~5.5 1.15
# 즉 3.5 는 p95 의 2.3배이고 단일 백 최대(2.90)보다도 높다. 그 값으로 생성한
# my_track0807 라인은 314점 중 89점(28%)에서 ax 가 정확히 3.50 으로 포화해
# 있었다 — 랩의 3분의 1을 차가 낸 적 없는 가속으로 계획한 셈이다.
# ⚠️ 이 상수는 계획 한계이자 폐루프 플랜트의 포화값이다(plant_accel_max_mps2).
#    올리면 시뮬 차도 같이 세지므로 게이트는 절대 못 잡고, 실차에서만 드러난다.
#    드러나는 방식도 나쁘다: control_node 가 조향 LUT 를 '계획 속도'로 조회하고
#    a_lat 도 v_plan² 로 계산하므로, 계획이 실제보다 빠르면 조향이
#    (v_plan/v_real)² 배로 과대 산출된다 (RACELINE_V_MAX 주석의 같은 실패 모드).
# 대가는 작다: my_track0806 동일 기하로 생성기가 직접 뽑은 두 CSV 비교에서
#    3.5 -> ax max 3.50 · v max 6.02 · 랩 8.51 s
#    1.8 -> ax max 1.80 · v max 5.48 · 랩 8.84 s   = 0.33 s (3.7%) 차이뿐이다.
# 가속은 배터리 상태에 크게 좌우된다(0731 p90 2.23). 08-07 백이 생기면 같은
# 방법으로 다시 재서 이 주석을 갱신할 것. 값을 바꾸면 my_track0806/0806_3/0807
# 궤적도 같이 재생성해야 한다.
A_ACC_MAX = 4.5         # 실제 가속 한계 [m/s²] (전류 포화 실측 p90. p50은 1.25)
A_DEC_MAX = 1.4         # 코스팅(무동력) 감속 [m/s²] — 실측 p50 1.5에 여유 0.1

# 브레이크 전류 1 A 가 만드는 '추가' 감속 [m/s² per A] — 코스팅 위에 더해진다.
# 2026-08-06 실측 (0805~0806 백, brake 활성 & amps≥0.9 구간):
#   풀브레이크(1 A) 감속 p50 = -1.9 ~ -2.0 (v 2~5 m/s), 같은 백의 스로틀-오프
#   p50 = -1.5 → 크레딧 실측 0.4~0.45 /A. 하한 잡아 0.35.
# 2026-08-07 (jaldallim 97813be): vesc.yaml ackermann_brake_max 를 5 A 로 올리면서
#   0.15 -> 0.12 로 재산정. 총량 0.12*5 = 0.6.
#   ⚠️ 5 A 에서의 총량은 실측이 아니라 선형 외삽이다(실측 근거는 1 A 기준
#   0.4~0.45). 브레이크 크레딧은 선형이 보장되지 않으므로(타이어 슬립·모터
#   특성) 5 A 백이 생기면 다시 잴 것.
BRAKE_DECEL_PER_AMP = 0.14

# 계획에 쓰는 브레이크 전류 [A]. vesc.yaml 의 ackermann_brake_max 와 같아야 한다
# (CLAUDE.md: vesc.yaml 이 hardware_ws 의 single source of truth).
# jaldallim 97813be 는 A_DEC_PLANNED 식에 5.0 을 상수로 박아 두었는데, 여기서는
# 이름 있는 상수로 유지한다 — 예전에 이 곱수가 1.0 으로 굳어 있던 탓에 vesc.yaml
# 만 올렸을 때 계획 제동이 1.75 -> 1.55 로 **오히려 약해진** 적이 있다
# (my_track0806_3 / my_track0807 매니페스트의 plant_decel_planned_mps2 = 1.55).
# (c252707 이 같은 커밋에서 vesc.yaml 을 brake_max 6.0 / ackermann_brake_max 5.0
#  으로 올려 이 값과 이미 정합이다. 예전 "vesc.yaml 이 3.0 이라 어긋난다" 경고는
#  그때 해소됐는데도 남아 있어, 없는 불일치를 고치려고 ackermann_brake_max 를
#  다시 만지도록 유도했다.)
# ⚠️ 남아 있는 진짜 주의점: 브레이크 크레딧(0.12 A^-1)은 실측이 아닌 외삽이다.
#
# ⚠️⚠️ 2026-08-11 확인: **위 문단이 선언한 계약이 지금 깨져 있다.**
#   vesc.yaml ackermann_brake_max = 5.0 인데 이 값은 7.0 이다. 즉 계획은
#   하드웨어가 낼 수 없는 전류를 전제로 A_DEC_PLANNED = 1.4 + 0.14*7.0 = 2.38
#   을 쓴다. 계약대로 5.0 이면 2.10 이 된다.
#   값을 바로 5.0 으로 내리지 않은 이유는 실측이 2.38 쪽을 지지하기 때문이다 —
#   rosbag_0811/run_0811_163606 (타이어 교체 후, odom/PF 비 0.99) 전 구간 감속
#   p50 1.33 / p90 2.05 / p99 2.31 m/s². 2.38 은 그 envelope 바로 위다.
#   대신 진짜 문제는 다른 데 있다: 같은 백에서
#     · 계획이 -2.03 m/s² 를 요구한 구간의 실제 달성은 -1.38 (p50)
#     · 그때 브레이크 전류는 p50 0.23 A / max 1.09 A — 5 A 의 5분의 1도 안 쓴다
#   ackermann_to_vesc 의 제동 진입 조건(brake_speed_error_gain 2.0,
#   brake_entry_speed_error 0.10)이 "명령보다 빨라야" 열리는데 속도오차 p50 이
#   +0.03 m/s 라 거의 안 열린다. 즉 A_DEC_PLANNED 를 몇으로 두든 실현되는 건
#   사실상 코스팅(A_DEC_MAX)이다.
#   → 정리 순서: (1) 이 상수를 5.0 으로 맞출지, vesc.yaml 을 7.0 으로 올릴지
#     결정하고 (2) 제동 진입 조건을 계획 ax 기반(brake_decel_threshold)으로
#     실제 열리게 만든 뒤 (3) 그 백으로 BRAKE_DECEL_PER_AMP 를 재측정한다.
#     지금 값을 그냥 5.0 으로 내리면 계획만 느려지고 실차 거동은 안 변한다.
#
# 2026-08-19 (1) 첫 실행: 5.0 으로 내렸다가, 같은 날 되돌림 — vesc.yaml
# ackermann_brake_max 를 7.0 으로 올리는 쪽을 택했다(하드웨어 브레이크 전류
# 상한 변경, 실차 검증 전까지는 "미검증"). (2) 도 같은 날 구현: ackermann_
# to_vesc 의 진입 조건을 speed_error(사후 오차) 대신 계획 자체(cmd.
# acceleration)로 바꾸고, 브레이크 전류에 피드포워드 항(요구 감속 - 코스팅)/
# BRAKE_DECEL_PER_AMP 를 추가했다 — 이 상수가 이제 그 피드포워드가 쓰는
# 실제 물리 상수다(전에는 계획 전용이라 실차 거동과 분리돼 있었다). (3)
# 여전히 미해결: 이 상수 자체(브레이크 크레딧 0.14 A^-1)는 1A 실측에서
# 외삽한 값 — 새 피드포워드가 실제로 여는지, 크레딧이 5A/7A 대역에서도
# 선형인지는 다음 실차 세션에서 재측정 전까지 가정이다.
BRAKE_AMPS_PLANNED = 7.0

# 레이스라인 생성 속도 상한. build_map.py 의 V_MAX 가 이 값을 그대로 쓴다
# (python_sim_tracker 프로파일이 cap 으로 쓰기 때문에 여기 있어야 한다).
#
# 2026-08-07 6.0 -> 7.0. ⚠️ 이건 '천장'이지 '목표'가 아니다 — 지금 트랙에서는
# 사실상 아무것도 바꾸지 않는다. my_track0806 기하로 V_MAX 만 스윕한 실측:
#     V_MAX  6.0 -> 프로파일 max 6.00 · 평균 4.38 · 랩 7.52 s (cap 도달 4/318 pt)
#     V_MAX  6.5 -> 프로파일 max 6.04 · 평균 4.38 · 랩 7.52 s (cap 도달 0 pt)
#     V_MAX 7~10 -> 전부 6.04 / 4.38 / 7.52 s 로 동일
# 즉 6.04 m/s 위로는 V_MAX 가 아니라 코너 탈출속도(mu 0.68 -> 최저 2.88 m/s) +
# 직선 길이 + A_ACC_MAX 가 묶는다. 전체 웨이포인트의 95%가 가감속 램프 위에
# 있고 정적 cap 에 붙어 있는 건 7% 뿐이다. 트랙이 길어지거나 코너가 완만해지면
# 그때 이 값이 실제로 풀린다 — 그 전까지는 클램프만 안 하는 안전 상한이다.
# ⚠️ 여기를 더 올려 '명령을 높여서 실속도를 얻는' 식은 이 스택에선 안 된다.
#    control_node.cpp calc_steering_angle 이 조향 LUT 를 **웨이포인트 계획
#    속도**(speed_for_lookup)로 조회하고 a_lat 도 v_plan² 로 계산한다. 계획이
#    실제보다 빠르면 조향이 (v_plan/v_real)² 배로 과대 산출된다.
#
# 2026-08-19 7.0 -> 8.5 -> 9.0(같은 날 저녁): control real_car.yaml
# vehicle/max_speed_mps 와 짝을 맞춤. LUT를 IFAC_realcar_v3(11백 풀링 보정 +
# 7~9.0 추정 확장)로 바꿔야 이 값이 controller_profile() 의 fail-closed LUT
# 범위 검사를 통과한다 — LUT 파일 주석 참조. ⚠️ 위 스윕(6.0~10 전부 7.52s 동일)
# 이 보여주듯 이 상수 자체는 이 트랙에서 랩타임을 거의 못 움직인다 — my_track0818
# 실측도 마찬가지(v_max 8.5여도 프로파일은 7 m/s대에서 자연히 묶임, 0% 클램프).
# 사용자 요청으로 "형식상" 9.0까지 열어둔 것 — 실제로 코너를 더 빠르게 도는 건
# MU_PROFILE/LAT_ERR_CORNER_S 쪽 문제다. 7.0 이상 구간은 이 차로 달려본 적이
# 없으므로 9.0 자체도 "이 속도까지 명령이 나갈 수 있다"이지 "이 속도에서
# 안전이 검증됐다"가 아니다.
RACELINE_V_MAX = 9.0

# 속도 프로파일(velocity_profile)이 계획에 쓰는 제동 한계 = 코스팅 + 브레이크
# 크레딧(BRAKE_AMPS_PLANNED, vesc.yaml 과 동기). 2026-08-19 기준
# 1.4 + 0.14*5.0 = 2.10 (옛 주석의 1.4+0.12*5.0=2.0 은 BRAKE_DECEL_PER_AMP가
# 0.12->0.14 로 재산정된 뒤 안 갱신된 드리프트였다). 실제로 그 전류가 물리는지는
# 폐루프 시뮬(VescLongitudinalPlant)이 본다.
A_DEC_PLANNED = A_DEC_MAX + BRAKE_DECEL_PER_AMP * BRAKE_AMPS_PLANNED


# --- 런타임 설정 로더 ---------------------------------------------------------
def _read_yaml(path):
    """설정 YAML 하나를 읽는다. 없거나 깨졌으면 즉시 죽는다 (fail-closed)."""
    path = Path(path)
    if not path.is_file():
        raise FileNotFoundError(f"런타임 설정 파일 없음: {path}")
    try:
        with open(path, "r", encoding="utf-8") as handle:
            data = yaml.safe_load(handle)
    except (OSError, yaml.YAMLError) as exc:
        raise ValueError(f"설정 YAML 파싱 실패: {path}: {exc}") from exc
    if data is None:
        return {}
    if not isinstance(data, dict):
        raise ValueError(f"설정 YAML 최상위가 매핑이 아님: {path}")
    return data


def _node_params(document, node):
    """ROS2 파라미터 문서에서 '/**' + '<node>' 의 ros__parameters 를 병합."""
    merged = {}
    for key in ("/**", node):
        section = document.get(key)
        if section is None:
            continue
        if not isinstance(section, dict):
            raise ValueError(f"'{key}' 섹션이 매핑이 아님")
        params = section.get("ros__parameters", {})
        if not isinstance(params, dict):
            raise ValueError(f"'{key}/ros__parameters' 가 매핑이 아님")
        merged.update(params)
    return merged


_RUNTIME_PARAM_CACHE = {}


def runtime_params(pkg_root, node, env):
    """launch 와 같은 순서로 params.yaml 위에 environments/<env>.yaml 을 얹는다.

    planner_stack.launch.py / control.launch.py 가 하는 것과 동일한 겹침이다.
    env=None 이면 params.yaml 기본값만 쓴다.
    """
    cache_key = (str(pkg_root), node, env)
    if cache_key in _RUNTIME_PARAM_CACHE:
        return _RUNTIME_PARAM_CACHE[cache_key]
    config_dir = Path(pkg_root) / "config"
    merged = _node_params(_read_yaml(config_dir / "params.yaml"), node)
    sources = [str(config_dir / "params.yaml")]
    if env:
        env_path = config_dir / "environments" / f"{env}.yaml"
        merged.update(_node_params(_read_yaml(env_path), node))
        sources.append(str(env_path))
    result = (merged, " + ".join(sources))
    _RUNTIME_PARAM_CACHE[cache_key] = result
    return result


def _require_number(params, key, source, positive=True):
    """런타임 파라미터를 숫자로 꺼낸다. 없거나 이상하면 죽는다."""
    if key not in params:
        raise ValueError(f"{source}: 필수 파라미터 '{key}' 없음")
    value = params[key]
    if isinstance(value, bool) or not isinstance(value, (int, float)) or \
            not np.isfinite(value):
        raise ValueError(f"{source}: '{key}' 가 유한 숫자가 아님 ({value!r})")
    value = float(value)
    if positive and value <= 0.0:
        raise ValueError(f"{source}: '{key}' 는 양수여야 함 ({value})")
    return value


def _require_string(params, key, source):
    if key not in params:
        raise ValueError(f"{source}: 필수 파라미터 '{key}' 없음")
    value = params[key]
    if not isinstance(value, str) or not value:
        raise ValueError(f"{source}: '{key}' 가 비어 있지 않은 문자열이 아님")
    return value


def vesc_brake_config():
    """ackermann_to_vesc 가 실제로 읽는 arbitration 임계값을 그대로 가져온다."""
    params = _node_params(_read_yaml(VESC_YAML), "ackermann_to_vesc_node")
    source = str(VESC_YAML)
    config = {
        "enabled": bool(params.get("brake_control_enabled", False)),
        # ⚠️ 드라이버의 brake_max 가 아니라 ackermann_brake_max 다 (53f2e44).
        #    전자는 모든 publisher에 걸리는 최종 드라이버 clamp이고, 실제
        #    명령 전류를 자르는 건 ackermann_to_vesc 의 ackermann_brake_max.
        "brake_max": _require_number(params, "ackermann_brake_max", source),
        "gain": _require_number(
            params, "brake_speed_error_gain", source, positive=False),
        "decel_threshold": _require_number(
            params, "brake_decel_threshold", source, positive=False),
        "entry_speed_error": _require_number(
            params, "brake_entry_speed_error", source, positive=False),
        "overspeed_error": _require_number(
            params, "brake_overspeed_error", source, positive=False),
        "release_speed_error": _require_number(
            params, "brake_release_speed_error", source, positive=False),
        "min_vehicle_speed": _require_number(
            params, "brake_min_vehicle_speed", source, positive=False),
    }
    # ackermann_to_vesc.cpp 생성자와 같은 유효성 계약. 여기서 죽는 설정은
    # 실차에서도 노드가 안 뜬다 — 시뮬만 통과시키면 안 된다.
    if config["release_speed_error"] >= config["entry_speed_error"]:
        raise ValueError(
            f"{source}: brake_release_speed_error 가 entry 이상 "
            "(ackermann_to_vesc 가 거부하는 설정)")
    return config


# --- 배포 프로파일 선언 -------------------------------------------------------
CONTROLLER_PROFILE_SOURCES = {
    # 실차: planning environments/<env>.yaml 의 velocity_scale +
    #       control  environments/<env>.yaml 의 vehicle/* · map/*.
    #       종방향 plant 는 VESC arbitration(vesc.yaml)을 그대로 모델링한다.
    "cpp_real_car": {
        "kind": "cpp_map_l1",
        "plant": "vesc",
        "planning_env": "real_car",
        "control_env": "real_car",
    },
    # 시뮬: control_pkg 를 안 쓴다. planning scripts/sim/waypoint_tracker_node.py
    #       가 컨트롤러이므로 planning params.yaml 의 tracker/* 를 읽는다.
    #       plant 는 gym 차량 모델이라 VESC arbitration이 없다 → ideal.
    "python_sim_tracker": {
        "kind": "python_pure_pursuit",
        "plant": "ideal",
        "planning_env": "sim",
        "control_env": None,
    },
}

# 어떤 프로파일이 CSV 승격을 '막을' 수 있는가(gate) vs 돌려서 보고만 하는가
# (advisory). 예전엔 두 프로파일이 모두 gate 라, 실차용 CSV인데 Python 시뮬
# 프로파일이 실패하면 실차 배포가 통째로 막혔다. 두 프로파일은 서로 다른
# 컨트롤러·다른 속도 스케일이라 한쪽 실패가 다른 쪽의 안전과 무관하다.
# advisory 도 반드시 실행되고 결과가 출력된다 — 조용히 건너뛰지 않는다.
GATE_PROFILE_SETS = {
    "real": {"gate": ("cpp_real_car",),
             "advisory": ("python_sim_tracker",)},
    "sim":  {"gate": ("python_sim_tracker",),
             "advisory": ("cpp_real_car",)},
    "all":  {"gate": ("cpp_real_car", "python_sim_tracker"),
             "advisory": ()},
}
DEFAULT_GATE_TARGET = "real"

_PROFILE_NUMERIC_FIELDS = (
    "planner_speed_scale", "controller_speed_scale", "max_speed_mps",
    "trajectory_validation_mu",
    "lookahead_gain", "lookahead_offset", "lookahead_min_m",
    "lookahead_max_m", "wheelbase_m", "max_steer_rad",
    "max_left_steer_rad", "max_right_steer_rad",
    "max_steer_rate_radps", "max_accel_mps2", "max_decel_mps2",
    "plant_accel_max_mps2", "plant_decel_max_mps2",
    "control_rate_hz", "num_local_waypoints",
)

# 모든 프로파일에 있지는 않지만 폐루프 판정을 실제로 바꾸는 값들. 서명에서
# 빠져 있으면 control lookahead / 조향 LUT / vesc 제동 중재만 바뀐 재검증이
# "같은 조건"으로 기록된다 (profile_signature docstring 의 계약 위반).
# speed_lookahead_s 는 조향 LUT 조회 속도를, steering_lut 는 LUT 파일 자체를,
# vesc 는 brake_max/gain/entry/overspeed/release 중재를 정한다.
_PROFILE_SIGNATURE_EXTRA_FIELDS = (
    "curvature_factor", "speed_lookahead_s", "steering_lut", "vesc",
    "backstop", "lat_err_coeff", "speed_lookahead_for_steer_s",
    "max_lateral_error_m", "max_curvature_reference",
    "l1_speed_uses_path_plan", "steer_plan_speed_min_mps",
    "acc_scaler_for_steer", "dec_scaler_for_steer",
    "start_scale_speed", "end_scale_speed", "downscale_factor",
    "velocity_steer_gain_max", "nearest_search_horizon_m",
)


def _steering_lut_max_speed(relative_path):
    """LUT 헤더 행의 마지막(최대) 속도 열을 읽는다.

    control_node.cpp의 로더가 헤더 단조증가를 이미 강제하므로 마지막 값이
    곧 최대다. CSV 전체를 파싱하는 build_map._load_steering_lut과 달리
    첫 줄만 읽어 build_map의 무거운 의존성(scipy/skimage/PIL) 없이 쓴다.
    """
    path = (WORKSPACE_ROOT / relative_path).resolve()
    if not path.is_file():
        raise FileNotFoundError(f"배포 steering LUT 없음: {path}")
    with open(path, "r", encoding="utf-8") as handle:
        header = handle.readline()
    values = [float(cell) for cell in header.strip().split(",")]
    if len(values) < 2:
        raise ValueError(f"steering LUT 헤더 열이 부족함: {path}")
    return values[-1]


def _resolve_cpp_real_car(name, source):
    """control_pkg / planning_pkg 의 배포 YAML에서 C++ MAP-L1 프로파일을 만든다."""
    planning, planning_src = runtime_params(
        PKG_ROOT, "global_trajectory_node", source["planning_env"])
    local_traj, local_src = runtime_params(
        PKG_ROOT, "local_trajectory_node", source["planning_env"])
    control, control_src = runtime_params(
        CONTROL_PKG_ROOT, "control", source["control_env"])
    lut_file = _require_string(control, "map/steering_lut_file", control_src)
    max_left_steer = _require_number(
        control, "vehicle/max_left_steering_angle_rad", control_src)
    max_right_steer = _require_number(
        control, "vehicle/max_right_steering_angle_rad", control_src)
    l1_speed_uses_path_plan = control.get("map/l1_speed_uses_path_plan")
    if not isinstance(l1_speed_uses_path_plan, bool):
        raise ValueError(
            f"{name}: {control_src} 의 map/l1_speed_uses_path_plan 이 bool이 아님")
    backstop_enabled = control.get("map/backstop_enabled")
    if not isinstance(backstop_enabled, bool):
        raise ValueError(
            f"{name}: {control_src} 의 map/backstop_enabled 가 bool이 아님")
    profile = {
        "planner_speed_scale": _require_number(
            planning, "global_trajectory/velocity_scale", planning_src),
        # 생성기의 MU_PROFILE보다 작으면 CSV 생성은 성공해도 실차 launch에서
        # global_trajectory_node가 즉시 거부한다. 프로파일 계약에 넣어 mkcsv가
        # 긴 최적화 전에 그 드리프트를 잡게 한다.
        "trajectory_validation_mu": _require_number(
            planning, "global_trajectory/validation_mu", planning_src),
        "controller_speed_scale": _require_number(
            control, "vehicle/speed_scale", control_src),
        "max_speed_mps": _require_number(
            control, "vehicle/max_speed_mps", control_src),
        # calc_l1_point(): Unicorn adaptive L1
        # t = m_l1*v + q_l1 - curvature_factor*mean(|kappa[i+10:i+20]|)*v^2.
        "lookahead_gain": _require_number(
            control, "map/m_l1", control_src, positive=False),
        "lookahead_offset": _require_number(
            control, "map/q_l1", control_src, positive=False),
        "curvature_factor": _require_number(
            control, "map/curvature_factor", control_src, positive=False),
        "lookahead_min_m": _require_number(
            control, "map/t_clip_min", control_src),
        "lookahead_max_m": _require_number(
            control, "map/t_clip_max", control_src),
        "speed_lookahead_s": _require_number(
            control, "map/speed_lookahead", control_src, positive=False),
        "lat_err_coeff": _require_number(
            control, "map/lat_err_coeff", control_src, positive=False),
        "speed_lookahead_for_steer_s": _require_number(
            control, "map/speed_lookahead_for_steer", control_src,
            positive=False),
        "max_lateral_error_m": _require_number(
            control, "map/max_lateral_error_m", control_src),
        "max_curvature_reference": _require_number(
            control, "map/max_curvature_reference", control_src),
        "l1_speed_uses_path_plan": l1_speed_uses_path_plan,
        "steer_plan_speed_min_mps": _require_number(
            control, "map/steer_plan_speed_min_mps", control_src,
            positive=False),
        "acc_scaler_for_steer": _require_number(
            control, "map/acc_scaler_for_steer", control_src,
            positive=False),
        "dec_scaler_for_steer": _require_number(
            control, "map/dec_scaler_for_steer", control_src,
            positive=False),
        "start_scale_speed": _require_number(
            control, "map/start_scale_speed", control_src, positive=False),
        "end_scale_speed": _require_number(
            control, "map/end_scale_speed", control_src),
        "downscale_factor": _require_number(
            control, "map/downscale_factor", control_src, positive=False),
        "velocity_steer_gain_max": _require_number(
            control, "map/velocity_steer_gain_max", control_src),
        "nearest_search_horizon_m": _require_number(
            control, "map/nearest_search_horizon_m", control_src),
        # 추종 백스톱 (control_node.cpp:1008-1093). 폐루프 시뮬과 사후
        # backstop_report()가 모두 같은 기준으로 적용/보고한다.
        "backstop": {
            "enabled": backstop_enabled,
            "path_error_m": _require_number(
                control, "map/backstop_path_error_m", control_src),
            "steer_sat_ratio": _require_number(
                control, "map/backstop_steer_sat_ratio", control_src),
            "sat_min_path_error_m": _require_number(
                control, "map/backstop_sat_min_path_error_m", control_src,
                positive=False),
            "steering_reference_rad": _require_number(
                control, "map/backstop_steering_reference_rad", control_src),
            "min_speed_mps": _require_number(
                control, "map/backstop_min_speed_mps", control_src,
                positive=False),
        },
        "wheelbase_m": _require_number(
            control, "vehicle/wheelbase_m", control_src),
        # Planning/폐루프 게이트는 아직 대칭 모델이므로 실제 좌우 중 작은
        # 방향을 사용한다. Control 런타임 clamp는 두 방향 값을 직접 사용한다.
        "max_steer_rad": min(max_left_steer, max_right_steer),
        "max_left_steer_rad": max_left_steer,
        "max_right_steer_rad": max_right_steer,
        "max_steer_rate_radps": _require_number(
            control, "vehicle/max_steering_rate_radps", control_src),
        # 컨트롤러의 속도 '명령' slew 한계 (control_node.cpp 의
        # previous_speed_command 리미터). 차가 실제로 낼 수 있는 가감속과 다르다.
        "max_accel_mps2": _require_number(
            control, "vehicle/max_accel_mps2", control_src),
        "max_decel_mps2": _require_number(
            control, "vehicle/max_decel_mps2", control_src),
        # 실측 plant 한계 — 명령이 아무리 가팔라도 차는 이 이상 못 낸다.
        "plant_accel_max_mps2": A_ACC_MAX,
        "plant_decel_max_mps2": A_DEC_MAX,
        "control_rate_hz": _require_number(control, "control_rate", control_src),
        "num_local_waypoints": _require_number(
            local_traj, "local_trajectory/num_local_waypoints", local_src),
        "steering_lut": f"{STEERING_LUT_DIR}/{lut_file}",
        "config_sources": f"{planning_src} + {control_src}",
    }
    # 실측 상수와 배포 YAML이 어긋나면 '검증한 차'와 '달리는 차'가 다르다.
    # 이게 바로 이 모듈이 존재하는 이유인 드리프트라 경고가 아니라 실패로 둔다.
    for field, measured, key in (
            ("wheelbase_m", WHEELBASE, "vehicle/wheelbase_m"),
            ("max_steer_rad", MAX_STEER,
             "min(vehicle/max_left_steering_angle_rad, "
             "vehicle/max_right_steering_angle_rad)"),
            ("max_steer_rate_radps", MAX_STEER_RATE,
             "vehicle/max_steering_rate_radps")):
        if abs(profile[field] - measured) > 1e-9:
            raise ValueError(
                f"{name}: {control_src} 의 {key}={profile[field]} 가 "
                f"deployment_profiles 실측 상수 {measured} 와 다름. 둘 중 어느 "
                "쪽이 맞는지 정하고 한 변경으로 맞출 것 (최소회전반경 판정이 "
                "실측 상수를 쓰므로 어긋난 채로는 검증 자체가 무의미하다)")
    for actual, measured, key in (
            (max_left_steer, MAX_LEFT_STEER,
             "vehicle/max_left_steering_angle_rad"),
            (max_right_steer, MAX_RIGHT_STEER,
             "vehicle/max_right_steering_angle_rad")):
        if abs(actual - measured) > 1e-9:
            raise ValueError(
                f"{name}: {control_src} 의 {key}={actual} 가 "
                f"deployment_profiles 실측 상수 {measured} 와 다름")
    return profile


def _resolve_python_sim_tracker(name, source):
    """scripts/sim/waypoint_tracker_node.py 가 읽는 tracker/* 를 그대로 쓴다."""
    del name
    planning, planning_src = runtime_params(
        PKG_ROOT, "global_trajectory_node", source["planning_env"])
    tracker, tracker_src = runtime_params(
        PKG_ROOT, "waypoint_tracker_node", source["planning_env"])
    local_traj, local_src = runtime_params(
        PKG_ROOT, "local_trajectory_node", source["planning_env"])
    max_steer = _require_number(
        tracker, "tracker/max_steer_rad", tracker_src)
    return {
        "planner_speed_scale": _require_number(
            planning, "global_trajectory/velocity_scale", planning_src),
        "trajectory_validation_mu": _require_number(
            planning, "global_trajectory/validation_mu", planning_src),
        "controller_speed_scale": _require_number(
            tracker, "tracker/speed_scale", tracker_src),
        # tracker 에는 속도 cap 파라미터가 없다 (wpnt 속도를 그대로 스케일).
        # 프로파일 속도 자체가 상한이므로 생성 v-max 를 cap 으로 둔다.
        "max_speed_mps": RACELINE_V_MAX,
        # lookahead = clip(t_lookahead * v, min, max) — offset 항이 없다.
        "lookahead_gain": _require_number(
            tracker, "tracker/t_lookahead", tracker_src),
        "lookahead_offset": 0.0,
        "lookahead_min_m": _require_number(
            tracker, "tracker/min_lookahead_m", tracker_src),
        "lookahead_max_m": _require_number(
            tracker, "tracker/max_lookahead_m", tracker_src),
        "wheelbase_m": _require_number(
            tracker, "tracker/wheelbase_m", tracker_src),
        "max_steer_rad": max_steer,
        "max_left_steer_rad": max_steer,
        "max_right_steer_rad": max_steer,
        "max_steer_rate_radps": _require_number(
            tracker, "tracker/max_steering_rate_radps", tracker_src),
        "max_accel_mps2": _require_number(
            tracker, "tracker/max_accel_mps2", tracker_src),
        "max_decel_mps2": _require_number(
            tracker, "tracker/max_decel_mps2", tracker_src),
        "plant_accel_max_mps2": A_ACC_MAX,
        "plant_decel_max_mps2": A_DEC_MAX,
        # tracker 는 odom 이벤트로 돈다. 명시적 루프 주기가 없으므로 같은 노드의
        # watchdog 주파수를 명령 주기 대용으로 쓴다.
        "control_rate_hz": _require_number(
            tracker, "tracker/safety_watchdog_hz", tracker_src),
        "num_local_waypoints": _require_number(
            local_traj, "local_trajectory/num_local_waypoints", local_src),
        "steering_lut": None,
        "config_sources": f"{planning_src} + {tracker_src}",
    }


_PROFILE_RESOLVERS = {
    "cpp_map_l1": _resolve_cpp_real_car,
    "python_pure_pursuit": _resolve_python_sim_tracker,
}


def controller_profile(name):
    """이름으로 선택한 배포 프로파일을 런타임 YAML에서 해석해 검증한다."""
    if not isinstance(name, str) or name not in CONTROLLER_PROFILE_SOURCES:
        raise ValueError(f"알 수 없는 controller profile: {name!r}")
    source = CONTROLLER_PROFILE_SOURCES[name]
    kind = source.get("kind")
    if kind not in _PROFILE_RESOLVERS:
        raise ValueError(f"{name}: 지원하지 않는 controller kind")
    if source.get("plant") not in ("vesc", "ideal"):
        raise ValueError(f"{name}: 지원하지 않는 종방향 plant")
    profile = _PROFILE_RESOLVERS[kind](name, source)
    profile["name"] = name
    profile["kind"] = kind
    profile["plant"] = source["plant"]
    if profile["plant"] == "vesc":
        profile["vesc"] = vesc_brake_config()
    for field in _PROFILE_NUMERIC_FIELDS:
        value = profile.get(field)
        if isinstance(value, bool) or not isinstance(value, (int, float)) or \
                not np.isfinite(value):
            raise ValueError(f"{name}: {field}가 유한 숫자가 아님")
    if not (0.0 < profile["planner_speed_scale"] <= 1.0) or \
            not (0.0 < profile["controller_speed_scale"] <= 1.0):
        raise ValueError(f"{name}: speed scale은 (0, 1]이어야 함")
    positive = (
        "max_speed_mps", "trajectory_validation_mu",
        "lookahead_min_m", "lookahead_max_m",
        "wheelbase_m", "max_steer_rad", "max_left_steer_rad",
        "max_right_steer_rad", "max_steer_rate_radps",
        "max_accel_mps2", "max_decel_mps2", "plant_accel_max_mps2",
        "plant_decel_max_mps2", "control_rate_hz",
    )
    if any(profile[field] <= 0.0 for field in positive):
        raise ValueError(f"{name}: 차량/제어 한계는 양수여야 함")
    if profile["trajectory_validation_mu"] > 1.5:
        raise ValueError(f"{name}: global trajectory validation mu는 1.5 이하여야 함")
    if profile["lookahead_max_m"] < profile["lookahead_min_m"]:
        raise ValueError(f"{name}: lookahead min/max가 뒤집힘")
    if profile["kind"] == "cpp_map_l1":
        extra_numeric = (
            "lat_err_coeff", "speed_lookahead_for_steer_s",
            "max_lateral_error_m", "max_curvature_reference",
            "steer_plan_speed_min_mps", "acc_scaler_for_steer",
            "dec_scaler_for_steer", "start_scale_speed", "end_scale_speed",
            "downscale_factor", "velocity_steer_gain_max",
            "nearest_search_horizon_m",
        )
        if any(not isinstance(profile.get(field), (int, float)) or
               isinstance(profile.get(field), bool) or
               not np.isfinite(profile[field]) for field in extra_numeric):
            raise ValueError(f"{name}: MAP-L1 추가 파라미터가 유한 숫자가 아님")
        if not (0.0 < profile["lookahead_gain"] <= 5.0) or \
                not (-5.0 <= profile["lookahead_offset"] <= 10.0) or \
                not (0.0 <= profile["curvature_factor"] <= 5.0) or \
                not (0.0 <= profile["speed_lookahead_s"] <= 2.0) or \
                not (0.0 <= profile["speed_lookahead_for_steer_s"] <= 2.0) or \
                not (0.0 <= profile["lat_err_coeff"] <= 1.0) or \
                not (0.0 <= profile["acc_scaler_for_steer"] <= 5.0) or \
                not (0.0 <= profile["dec_scaler_for_steer"] <= 5.0) or \
                not (0.0 <= profile["start_scale_speed"] <
                     profile["end_scale_speed"] <= 30.0) or \
                not (0.0 <= profile["downscale_factor"] <= 1.0) or \
                not (0.0 < profile["max_lateral_error_m"] <= 5.0) or \
                not (0.0 < profile["max_curvature_reference"] <= 5.0) or \
                not (1.0 <= profile["velocity_steer_gain_max"] <= 3.0) or \
                not (0.5 <= profile["nearest_search_horizon_m"] <= 20.0) or \
                not (0.0 <= profile["steer_plan_speed_min_mps"] <= 2.0):
            raise ValueError(f"{name}: MAP-L1 추가 파라미터 범위가 잘못됨")
        backstop = profile.get("backstop")
        if not isinstance(backstop, dict) or \
                not isinstance(backstop.get("enabled"), bool):
            raise ValueError(f"{name}: backstop 설정이 매핑/bool이 아님")
        backstop_numeric = (
            "path_error_m", "steer_sat_ratio", "sat_min_path_error_m",
            "steering_reference_rad", "min_speed_mps")
        if any(not isinstance(backstop.get(field), (int, float)) or
               isinstance(backstop.get(field), bool) or
               not np.isfinite(backstop[field]) for field in backstop_numeric):
            raise ValueError(f"{name}: backstop 파라미터가 유한 숫자가 아님")
        if not (0.0 < backstop["path_error_m"] <= 5.0) or \
                not (0.0 < backstop["steer_sat_ratio"] <= 1.0) or \
                not (0.0 <= backstop["sat_min_path_error_m"] <
                     backstop["path_error_m"]) or \
                not (0.0 < backstop["steering_reference_rad"] <=
                     profile["max_steer_rad"]) or \
                not (0.0 <= backstop["min_speed_mps"] <= 10.0):
            raise ValueError(f"{name}: backstop 파라미터 범위가 잘못됨")
    if profile["max_steer_rad"] >= 0.5 * np.pi:
        raise ValueError(f"{name}: max steer가 자전거 모델 범위를 벗어남")
    if abs(profile["max_steer_rad"] - min(
            profile["max_left_steer_rad"],
            profile["max_right_steer_rad"])) > 1e-12:
        raise ValueError(f"{name}: 대칭 planning 조향 한계가 좌우 구속값과 불일치")
    n_local = profile["num_local_waypoints"]
    if int(n_local) != n_local or n_local < 2:
        raise ValueError(f"{name}: num_local_waypoints는 2 이상의 정수여야 함")
    profile["num_local_waypoints"] = int(n_local)
    lut = profile.get("steering_lut")
    if profile["kind"] == "cpp_map_l1" and \
            (not isinstance(lut, str) or not lut):
        raise ValueError(f"{name}: C++ MAP/L1 steering LUT가 지정되지 않음")
    if profile["kind"] == "python_pure_pursuit" and lut is not None:
        raise ValueError(f"{name}: Python pure pursuit에 LUT를 지정할 수 없음")
    # 2026-08-19: LUT 범위 초과 질의는 control_node.cpp도 build_map._lookup_
    # steer_angle도 조용히 최근접 열을 재사용한다 (의도된 동작 — 실차에서
    # 갑자기 조향각 0을 던지는 것보다 낫다). 하지만 계획/런타임 속도 상한이
    # LUT가 실제로 실측한 범위를 넘어서면 그 재사용이 상시로 일어나는데, 그건
    # "LUT가 이 속도를 안다"는 게 아니라 "LUT의 가장 가까운 실측치로 조용히
    # 대체한다"는 뜻이다 — 위 wheelbase/steer_rad 드리프트 검사와 같은 fail-
    # closed 철학을 여기도 적용한다. 예: RACELINE_V_MAX 나 vehicle/max_speed_mps
    # 를 LUT 재적합 없이 7.0 위로 올리면(§TUNING_STATUS 11, "10 m/s면 LUT부터
    # 막힌다") 여기서 바로 죽어야지, 시뮬 결과가 조용히 7.0 m/s 열로 클램프된
    # 채 나와서는 안 된다.
    if lut is not None:
        lut_max_speed = _steering_lut_max_speed(lut)
        if profile["max_speed_mps"] > lut_max_speed + 1e-9:
            raise ValueError(
                f"{name}: max_speed_mps={profile['max_speed_mps']:.3f} 가 "
                f"steering LUT 실측 최대 속도 {lut_max_speed:.3f} m/s를 넘음 "
                f"({lut}). 이 범위 밖 질의는 조용히 최근접 열로 클램프되므로 "
                "LUT 재적합 없이는 이 프로파일로 검증할 수 없음")
    return profile


def selected_controller_profiles(target=DEFAULT_GATE_TARGET):
    """gate/advisory 프로파일을 해석해서 (gate, advisory) 튜플로 돌려준다.

    gate 가 비어 있으면 검증 없이 결과를 쓰게 되므로 허용하지 않는다.
    advisory 도 실행은 되고 결과가 보고된다 — 승격만 막지 않을 뿐이다.
    """
    if target not in GATE_PROFILE_SETS:
        raise ValueError(f"알 수 없는 검증 대상: {target!r}")
    selection = GATE_PROFILE_SETS[target]
    gate_names = tuple(selection["gate"])
    advisory_names = tuple(selection["advisory"])
    if not gate_names:
        raise ValueError(f"{target}: gate 프로파일이 비어 있음 — 검증 없이 저장 금지")
    all_names = gate_names + advisory_names
    if len(all_names) != len(set(all_names)):
        raise ValueError(f"{target}: gate/advisory 프로파일 이름이 중복됨")
    return (tuple(controller_profile(name) for name in gate_names),
            tuple(controller_profile(name) for name in advisory_names))


def gate_controller_profiles(target=DEFAULT_GATE_TARGET):
    """저장/승격을 막을 수 있는 프로파일만."""
    return selected_controller_profiles(target)[0]


def advisory_controller_profiles(target=DEFAULT_GATE_TARGET):
    """실행하고 보고하지만 저장/승격을 막지는 않는 프로파일."""
    return selected_controller_profiles(target)[1]


def profile_manifest(target=DEFAULT_GATE_TARGET):
    """해석된 프로파일 요약. 어떤 설정으로 검증했는지 기록/비교용."""
    gate, advisory = selected_controller_profiles(target)
    return {
        "gate_target": target,
        "gate_profiles": [profile["name"] for profile in gate],
        "advisory_profiles": [profile["name"] for profile in advisory],
        "profiles": {
            profile["name"]: {
                field: profile[field] for field in _PROFILE_NUMERIC_FIELDS
            } | {
                field: profile.get(field)
                for field in _PROFILE_SIGNATURE_EXTRA_FIELDS
            } | {"config_sources": profile["config_sources"],
                 "plant": profile["plant"]}
            for profile in gate + advisory
        },
        "vehicle": {
            "wheelbase_m": WHEELBASE,
            "max_steer_rad": MAX_STEER,
            "max_left_steer_rad": MAX_LEFT_STEER,
            "max_right_steer_rad": MAX_RIGHT_STEER,
            "max_steer_rate_radps": MAX_STEER_RATE,
            "plant_accel_mps2": A_ACC_MAX,
            "plant_coast_decel_mps2": A_DEC_MAX,
            "brake_decel_per_amp": BRAKE_DECEL_PER_AMP,
            "plant_decel_planned_mps2": A_DEC_PLANNED,
        },
    }


# --- 종방향 plant -------------------------------------------------------------
# 시뮬 로그 열. 생성기와 튜너가 같은 목록을 쓴다.
SIM_LOG_FIELDS = (
    # path_err 는 d_err(수직 성분)이 아니라 **최근접 waypoint 까지의 거리**다.
    # 추종 백스톱이 재는 값이 그것이라(control_node.cpp:1010) 같은 양으로
    # 기록해야 backstop_report() 가 실차 발화 조건과 같은 판정을 한다.
    "t", "x", "y", "theta", "v", "v_ref", "s", "d_err", "path_err", "steer",
    "a_long_cmd", "a_long_plant", "a_lat_cmd", "a_long", "a_lat",
    "combined_demand", "combined_accel", "tire_saturated",
    "brake_amps", "braking",
)


def backstop_metrics(log, profile):
    """폐루프 로그가 추종 백스톱을 얼마나 건드렸는지 재서 dict 로 돌려준다.

    왜 필요한가: LAT_ERR_CORNER_S(코너 폭 상한 기울기)를 고르는 기준이 문자
    그대로 "백스톱 임계 초과 여부"인데, 그 판정이 gate 가 아니라 build_map.py
    주석의 손으로 만든 표에만 있었다. 그래서 gate 는 여러 기울기를 전부 PASS
    로 통과시키면서 "그 구간 내내 안전망 위에 앉아 있다"는 사실은 어디에도
    남기지 않았다.

    이것은 시뮬에서 이미 적용된 백스톱을 같은 기준으로 사후 집계하는 지표다.
    낙폭은 당시 실속도 대비 즉시 계산한 cap 차이이며, 실제 명령에는 그 cap 뒤
    slew가 적용되므로 낙폭 합계를 랩타임 손실로 직접 환산하면 안 된다.

    반환: 백스톱 없는 프로파일이거나 비활성이면 None. 그 외에는 미발화라도
    dict 를 돌려준다 — 매니페스트에 "0 번 발화"가 남는 것과 항목 자체가 없는
    것은 다르다 (후자는 이 지표가 없던 시절의 CSV 와 구별이 안 된다).
    """
    backstop = profile.get("backstop")
    if not backstop or not backstop.get("enabled"):
        return None
    path_err = np.asarray(log["path_err"], dtype=float)
    speed = np.asarray(log["v"], dtype=float)
    max_steer = backstop["steering_reference_rad"]
    saturation = np.abs(np.asarray(log["steer"], dtype=float)) / max(
        max_steer, 1e-3)

    err_threshold = backstop["path_error_m"]
    min_speed = backstop["min_speed_mps"]
    headroom = speed - min_speed

    err_hit = path_err > err_threshold
    over_err = np.clip(
        (path_err - err_threshold) / max(err_threshold, 1e-3), 0.0, 1.0)
    # 조향포화 분기는 경로오차 조건과 AND 로 묶여 있다 (2026-08-12).
    sat_threshold = backstop["steer_sat_ratio"]
    sat_min_err = backstop["sat_min_path_error_m"]
    sat_err_ok = (sat_min_err <= 0.0) | (path_err > sat_min_err)
    sat_hit = (saturation > sat_threshold) & sat_err_ok
    over_sat = np.clip(
        (saturation - sat_threshold) / max(1.0 - sat_threshold, 1e-3),
        0.0, 1.0)

    # 두 분기가 각자 캡을 내고 낮은 쪽이 이긴다. 마지막에 하한으로 clamp.
    reduction = np.maximum(
        np.where(err_hit, over_err * headroom, 0.0),
        np.where(sat_hit, over_sat * headroom, 0.0))
    reduction = np.maximum(
        np.minimum(reduction, np.maximum(headroom, 0.0)), 0.0)
    triggered = err_hit | sat_hit
    return {
        "path_error_threshold_m": float(err_threshold),
        "steer_sat_threshold": float(sat_threshold),
        "triggered_steps": int(triggered.sum()),
        "path_error_steps": int(err_hit.sum()),
        "steer_sat_steps": int(sat_hit.sum()),
        "total_steps": int(len(path_err)),
        "path_error_max_m": float(path_err.max()),
        "speed_cut_max_mps": float(reduction.max()),
    }


def backstop_report(log, profile):
    """backstop_metrics() 를 gate 출력 한 줄로 만든다. 없으면 ""."""
    metrics = backstop_metrics(log, profile)
    if not metrics or not metrics["triggered_steps"]:
        return ""
    return (
        f"백스톱 {metrics['triggered_steps']} step"
        f"(경로오차 {metrics['path_error_steps']} / "
        f"조향포화 {metrics['steer_sat_steps']}, "
        f"최대 낙폭 {metrics['speed_cut_max_mps']:.2f} m/s, "
        f"경로오차 max {metrics['path_error_max_m']:.3f} m > "
        f"{metrics['path_error_threshold_m']:.2f})")


class VescLongitudinalPlant:
    """ackermann_to_vesc 의 DRIVE / COAST / BRAKE 중재 + 실측 종방향 응답.

    예전 모델은 `a = clip((v_cmd - v)/dt, -A_DEC, +A_ACC)` 한 줄이었다. 그러면
    "명령만 내리면 언제나 A_DEC 로 선다"는 뜻이라, 2026-08-04 에 들어간 제동
    코드가 실제로 언제 물리는지(brake_entry_speed_error / brake_min_vehicle_speed
    / 히스테리시스)를 전혀 검사하지 못한다. 여기서 그 상태기계를 그대로 돌린다.

    모드별 실제 가속도:
      DRIVE : erpm 속도 명령 → VESC 속도 PID. 30 A 전류 포화로 +A_ACC_MAX 상한.
              53f2e44 이후로는 speed_error 가 brake_entry 미만이면 오버스피드라도
              여기 남는다 (작은 오차는 속도 PID 가 직접 잡게 둔다).
      COAST : motor current 0 → 항력만. -A_DEC_MAX (실측 코스팅).
              speed_error >= brake_entry 인데 제동 진입 조건은 아닐 때.
      BRAKE : brake 전류 = clip(gain * speed_error, 0, ackermann_brake_max).
              항력 + BRAKE_DECEL_PER_AMP * amps. 크레딧 미실측이면 COAST 와 동일.
    """

    def __init__(self, config, accel_max, coast_decel):
        self.config = config
        self.accel_max = float(accel_max)
        self.coast_decel = float(coast_decel)
        self.braking = False

    def reset(self):
        self.braking = False

    def step(self, speed, speed_cmd, accel_cmd, dt):
        """(가속도 [m/s²], 모드, brake amps). +가 가속.

        VESC는 '속도'를 좇는다(erpm 명령 → 속도 PID). accel_cmd 는 컨트롤러가
        내보내는 drive.acceleration 으로, 제동 진입 판정에만 쓰인다 — 실제
        가속도는 목표 속도까지의 차이와 물리 한계에서 나온다.
        """
        if speed_cmd >= speed:
            # DRIVE — 속도 PID 가 올리는 쪽. 30 A 전류 포화가 실제 상한이고,
            # 목표를 넘겨 가속하지는 않는다.
            self.braking = False
            return min((speed_cmd - speed) / dt, self.accel_max), "drive", 0.0

        speed_error = speed - speed_cmd
        needed_decel = speed_error / dt
        config = self.config
        if not config["enabled"] or speed_cmd < 0.0 or speed < 0.0:
            # brake_control_enabled=false 이거나 후진이면 legacy erpm 경로.
            # 감속은 코스팅 항력뿐이다.
            self.braking = False
            return -min(needed_decel, self.coast_decel), "coast", 0.0

        above_brake_speed = speed >= config["min_vehicle_speed"]
        planned_deceleration = (
            accel_cmd <= -config["decel_threshold"] and
            speed_error >= config["entry_speed_error"])
        excessive_overspeed = speed_error >= config["overspeed_error"]
        if self.braking:
            self.braking = (
                above_brake_speed and
                speed_error > config["release_speed_error"])
        else:
            self.braking = above_brake_speed and (
                planned_deceleration or excessive_overspeed)

        if not self.braking:
            # 53f2e44: 작은 오버스피드(entry 미만)는 COAST 로 떨어뜨리지 않고
            # erpm 명령을 그대로 보내 속도 PID 가 잡게 둔다. 이 구간에서 차는
            # 목표 속도를 좇으므로 항력 한계까지만 감속한다.
            if speed_error < config["entry_speed_error"]:
                return (-min(needed_decel, self.coast_decel), "drive", 0.0)
            return -min(needed_decel, self.coast_decel), "coast", 0.0
        amps = min(config["brake_max"],
                   max(0.0, config["gain"] * speed_error))
        available = self.coast_decel + BRAKE_DECEL_PER_AMP * amps
        return -min(needed_decel, available), "brake", amps


class IdealLongitudinalPlant:
    """VESC 중재가 없는 plant (시뮬 gym 차량). 명령 속도를 실측 한계로 좇는다."""

    def __init__(self, accel_max, decel_max):
        self.accel_max = float(accel_max)
        self.decel_max = float(decel_max)

    def reset(self):
        pass

    def step(self, speed, speed_cmd, accel_cmd, dt):
        del accel_cmd
        accel = float(np.clip(
            (speed_cmd - speed) / dt, -self.decel_max, self.accel_max))
        return accel, "drive" if accel >= 0.0 else "coast", 0.0


def make_longitudinal_plant(profile):
    """프로파일 선언에 맞는 종방향 plant 를 만든다."""
    if profile["plant"] == "vesc":
        return VescLongitudinalPlant(
            profile["vesc"], profile["plant_accel_max_mps2"],
            profile["plant_decel_max_mps2"])
    return IdealLongitudinalPlant(
        profile["plant_accel_max_mps2"], profile["plant_decel_max_mps2"])


def profile_brake_max(profile):
    """제동 크레딧 상한 계산용 brake_max [A]. VESC plant 가 아니면 0."""
    return float(profile["vesc"]["brake_max"]) \
        if profile.get("plant") == "vesc" else 0.0
