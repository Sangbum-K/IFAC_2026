"""Fast, hardware-free checks for high-speed planning safety invariants."""

import ast
import math
import re
import sys
from pathlib import Path

import numpy as np
import pytest
import yaml


PKG = Path(__file__).resolve().parents[1]
WORKSPACE = PKG.parents[2]
PARAMS = PKG / "config" / "params.yaml"
TRAJECTORIES = PKG / "config" / "global_trajectory"
CSV_SPEED_RE = re.compile(r"_max([0-9.]+)ms_")
SIM_RACE_LAUNCH = PKG / "launch" / "sim_race.launch.py"

sys.path.insert(0, str(PKG / "scripts"))
import deployment_profiles as profiles  # noqa: E402


def _yaml(path):
    with path.open(encoding="utf-8") as stream:
        return yaml.safe_load(stream)


def _ros_params(document, node):
    return document[node]["ros__parameters"]


def _load_trajectory(path):
    rows = []
    with path.open(encoding="utf-8") as stream:
        for lineno, line in enumerate(stream, 1):
            line = line.strip()
            if not line or line.startswith("#"):
                continue
            fields = [field.strip() for field in line.split(";")]
            assert len(fields) == 9, f"{path}:{lineno}: expected 9 columns"
            rows.append([float(field) for field in fields])
    data = np.asarray(rows, dtype=float)
    assert data.ndim == 2 and data.shape[0] >= 8 and data.shape[1] == 9
    return data


def test_default_profile_is_fail_safe():
    cfg = _yaml(PARAMS)
    global_cfg = _ros_params(cfg, "global_trajectory_node")
    local = _ros_params(cfg, "local_trajectory_node")
    spliner = _ros_params(cfg, "spliner_node")
    recovery = _ros_params(cfg, "recovery_supervisor_node")
    tracker = _ros_params(cfg, "waypoint_tracker_node")

    assert global_cfg["global_trajectory/overtaking_csv_file"] == ""
    assert 0.0 < global_cfg["global_trajectory/min_velocity_scale"] <= \
        global_cfg["global_trajectory/velocity_scale"] <= \
        global_cfg["global_trajectory/max_velocity_scale"] <= 1.0
    # (spliner/use_trailing 삭제: 생성자가 false 를 무조건 true 로 되돌려
    #  스위치로 기능하지 않았다 — trailing 은 이제 항상 켜져 있다.)
    assert "spliner/use_trailing" not in spliner
    assert spliner["spliner/sp_lane_change"] is False
    # perception lease. 짧을수록 "빨리 알아챈다"가 아니다 — 초과하면 spliner가
    # fail-safe 정지 궤적(vx=0)을 발행하므로, detector 실측 발행 주기보다
    # 짧으면 정상 스트림에서도 프레임 사이마다 정지가 끼어든다.
    # run_0803_213313: 0.20 < detector 주기 0.246s -> 28.6초 정지 라치.
    # 상한은 "탐지 실패를 이만큼 안에는 알아챈다"는 안전 계약이므로 유지하되,
    # detector가 목표대로 >=15Hz가 되면 다시 조일 것.
    assert spliner["spliner/obs_timeout_sec"] <= 0.60
    assert spliner["spliner/ot_speed_scale"] <= 1.0
    assert local["local_trajectory/require_avoidance_heartbeat"] is True
    assert local["local_trajectory/avoidance_timeout_sec"] <= 0.25
    assert local["local_trajectory/frenet_odom_source_timeout_sec"] <= 0.25
    # recovery 체인은 현재 기본 비활성 (launch recovery:=true opt-in).
    # 다시 켤 때도 후진 계열은 검증 전까지 꺼져 있어야 한다.
    assert recovery["recovery/enabled"] is False
    assert recovery.get("recovery/allow_reverse", False) is False
    assert recovery.get("recovery/rear_coverage_confirmed", False) is False
    assert tracker["tracker/recovery_required"] is False
    assert tracker["tracker/recovery_allow_reverse"] is False
    # (tracker/internal_stuck_recovery_enabled 삭제: 자체 후진 상태머신을
    #  통째로 제거해 tracker 는 STOP 래치만 한다. 후진은 supervisor 몫.)
    assert "tracker/internal_stuck_recovery_enabled" not in tracker


def test_default_main_trajectory_semantics():
    cfg = _yaml(PARAMS)
    p = _ros_params(cfg, "global_trajectory_node")
    path = TRAJECTORIES / p["global_trajectory/trajectory_csv_file"]
    data = _load_trajectory(path)

    assert np.isfinite(data).all()
    assert np.all(data[:, 0] >= 0.0)
    assert np.all(data[:, 3:5] >= 0.0)
    assert np.all(data[:, 7] >= 0.0)
    ds = np.diff(data[:, 0])
    assert np.all(ds > 1e-5)
    assert ds.max() <= ds.min() * 1.25
    closure = np.linalg.norm(data[0, 1:3] - data[-1, 1:3])
    assert closure <= ds.max() * 2.5

    kappa_limit = math.tan(
        p["global_trajectory/validation_max_steer_rad"]
    ) / p["global_trajectory/validation_wheelbase_m"]
    assert np.abs(data[:, 6]).max() <= \
        kappa_limit * (1.0 + p["global_trajectory/validation_tolerance"])
    lateral_accel = data[:, 7] ** 2 * np.abs(data[:, 6])
    combined_accel = np.hypot(lateral_accel, data[:, 8])
    assert combined_accel.max() <= \
        p["global_trajectory/validation_mu"] * 9.81 * \
        (1.0 + p["global_trajectory/validation_tolerance"])
    numeric_ax = (data[1:, 7] ** 2 - data[:-1, 7] ** 2) / (2.0 * ds)
    assert np.abs(numeric_ax - data[:-1, 8]).max() <= \
        p["global_trajectory/validation_ax_tolerance"]


def test_environment_nominal_speed_matches_control_cap():
    cfg = _yaml(PARAMS)
    global_base = _ros_params(cfg, "global_trajectory_node")
    csv_name = global_base["global_trajectory/trajectory_csv_file"]
    match = CSV_SPEED_RE.search(csv_name)
    assert match, f"trajectory filename does not encode v_max: {csv_name}"
    csv_vmax = float(match.group(1))
    actual_csv_vmax = float(
        _load_trajectory(TRAJECTORIES / csv_name)[:, 7].max())

    # base(params.yaml) 프로파일도 검사한다. 예전엔 real_car/race_map 만 봐서
    # base 의 velocity_scale 이 v-max 6.0 시절 값(0.6667)으로 남아 sim control
    # cap 4.0 을 17% 초과하는 것을 못 잡았다 (integration stack env:=sim 경로).
    for env_name in ("sim", "sim_real_car", "real_car", "race_map"):
        # 오버레이에 velocity_scale 이 있으면 그것, 없으면 base(params.yaml).
        # 예전엔 env:=sim 만 "sim.yaml 이 비어 있다"는 전제로 base 를 직접 읽었는데,
        # 2026-08-12 에 sim.yaml 이 real_car 거동으로 채워지면서 그 전제가 깨졌다.
        # 이제 실제 파라미터 해석 순서(base <- 오버레이)를 그대로 따라간다.
        planning_scale = _env_profile_flat(PKG, env_name).get(
            "global_trajectory_node.global_trajectory/velocity_scale",
            global_base["global_trajectory/velocity_scale"])
        control_env_path = (
            WORKSPACE / "control_ws" / "src" / "control_pkg" /
            "config" / "environments" / f"{env_name}.yaml"
        )
        assert control_env_path.is_file(), control_env_path
        control_env = _yaml(control_env_path)
        control_cap = _ros_params(
            control_env, "control"
        )["vehicle/max_speed_mps"]
        control_scale = _ros_params(
            control_env, "control"
        )["vehicle/speed_scale"]
        # 계획 최고속이 control cap 을 넘으면 control 이 전 구간을 clamp 해서
        # (a) 속도 프로파일이 cap 상수로 뭉개지고 (b) 조향 LUT 를 wpnt 속도로
        # 조회하는 구조(control_node calc_steering_angle) 때문에 실제 필요조향의
        # 1.7~2.0 배가 나온다. 그걸 막는 게 이 단언이다.
        #
        # 2026-08-12: 예전엔 등식(csv_vmax*scale == cap)이었다. 08-10 타이어 교체로
        # real_car 가 velocity_scale 0.96 을 쓰면서 6.72 != 7.0 이 되어 깨졌는데,
        # real_car.yaml 은 그 7.0 을 "클램프가 걸리지 않는 안전 상한"으로
        # **의도적으로** 남긴다고 명시한다 (control real_car.yaml vehicle/max_speed_mps
        # 주석). 즉 낡은 쪽은 등식이지 설정이 아니다.
        # 등식이 실제로 지키던 사고(base velocity_scale 이 v-max 6.0 시절 값
        # 0.6667 로 남아 sim cap 4.0 을 17% 초과)는 이 부등식이 그대로 잡는다.
        assert csv_vmax * planning_scale <= control_cap + 0.02, env_name
        # 허용오차는 위 단언과 같은 0.02 m/s. 예전엔 1e-6 이라 race_map 의
        # velocity_scale 0.1429(= 1.0/7.0 을 소수 4자리로 **올림**한 값)가
        # 계획 최고속을 cap 보다 6.7e-5 m/s 넘기는 것을 실패로 봤다. 그건
        # 설정 오류가 아니라 4자리 반올림 잔차이고, 조향 중립이 런 사이 2.5°
        # 튀는 차에서 측정 가능한 양이 아니다 (TUNING_STATUS §11).
        # 이 단언이 실제로 잡아야 하는 것은 "cap 이 프로파일 상단을 잘라
        # 조향 LUT 조회가 뭉개지는" 수준의 초과이고, 그건 0.02 로 충분히 걸린다.
        assert actual_csv_vmax * planning_scale * control_scale <= \
            control_cap + 0.02, env_name


def test_real_profiles_require_exact_time_and_fresh_inputs():
    for env_name in ("real_car", "race_map"):
        env = _yaml(PKG / "config" / "environments" / f"{env_name}.yaml")
        bridge = _ros_params(env, "perception_obstacle_bridge_node")
        assert bridge["bridge/tf_allow_latest_fallback"] is False
        # detector 출력 지연 실측 p50 0.258 / p90 0.306 / p99 0.356s
        # (run_0803_213313). 0.25면 511프레임 중 508개가 거부돼 장애물
        # 스트림이 통째로 끊긴다 — 이 상한은 "신선한 입력만 쓴다"는 계약이지
        # 파이프라인을 굶기라는 뜻이 아니다. 좌표 자체는 detector가 scan
        # 정시각 TF로 변환하므로 age 완화가 위치 오차를 키우지 않는다.
        # detector on_scan 처리시간(~150ms)을 줄이면 다시 조일 것.
        assert bridge["bridge/max_source_age_sec"] <= 0.45
        assert bridge["bridge/tf_max_age_sec"] <= 0.10


def test_control_profiles_disable_reverse_and_supervisor_not_required():
    control_base = _ros_params(
        _yaml(WORKSPACE / "control_ws" / "src" / "control_pkg" /
              "config" / "params.yaml"),
        "control",
    )
    assert control_base["recovery/allow_reverse"] is False
    for env_name in ("real_car", "race_map", "sim"):
        control_env = _ros_params(
            _yaml(WORKSPACE / "control_ws" / "src" / "control_pkg" /
                  "config" / "environments" / f"{env_name}.yaml"),
            "control",
        )
        # recovery 체인 비활성 상태이므로 supervisor 부재로 fail-closed 하지 않는다.
        assert control_env["recovery/require_supervisor"] is False
        assert control_env["vehicle/speed_scale"] <= 1.0


def test_sim_controller_matches_planner_speed_model():
    """플래너가 명령에 곱하는 배율과 컨트롤러가 쓰는 배율이 같아야 한다.

    어긋나면 동적 상대 trailing 에서 상대보다 느리게 따라가 격차가 벌어진다.
    경로가 둘이라 따로 본다:
      (1) 파이썬 waypoint_tracker (sim_race tracker:=true). 통합 스택은 항상
          tracker:=false 라 레거시 경로고, tracker/speed_scale 0.30 은 드리프트가
          아니라 "pure-pursuit 이 이 트랙 R~1.4m 헤어핀을 오버슛 없이 따라가려면
          저속이어야 한다"는 그 컨트롤러 자체의 한계다 (params.yaml 주석).
          그래서 base 안에서만 짝을 본다.
      (2) C++ control_node (통합 스택의 실제 경로). env 오버레이가 양쪽을 다
          덮으므로 **실효값**끼리 봐야 한다 — 2026-08-12 이전엔 base 의 0.30 과
          control sim.yaml 의 0.30 을 비교해서, env:=sim 이 real_car 거동으로
          올라간 뒤에도 통과할 수 없는 형태였다.
    """
    planning = _ros_params(_yaml(PARAMS), "spliner_node")
    tracker = _ros_params(_yaml(PARAMS), "waypoint_tracker_node")
    assert math.isclose(
        planning["spliner/cmd_speed_scale"],
        tracker["tracker/speed_scale"],
        rel_tol=0.0, abs_tol=1e-9,
    )

    control_pkg = WORKSPACE / "control_ws" / "src" / "control_pkg"
    for env_name in ("sim", "sim_real_car", "real_car"):
        planner_scale = _env_profile_flat(PKG, env_name).get(
            "spliner_node.spliner/cmd_speed_scale",
            planning["spliner/cmd_speed_scale"])
        control_scale = _env_profile_flat(control_pkg, env_name)[
            "control.vehicle/speed_scale"]
        assert math.isclose(
            planner_scale, control_scale, rel_tol=0.0, abs_tol=1e-9,
        ), env_name


def test_perception_defaults_use_exact_source_time():
    perception = _ros_params(
        _yaml(WORKSPACE / "perception_ws" / "src" / "perception_pkg" /
              "config" / "params.yaml"),
        "obstacle_detector_node",
    )
    assert perception["require_source_stamp"] is True
    assert perception["tf_allow_latest_fallback"] is False
    assert perception["max_source_age_sec"] <= 0.25
    assert perception["tf_max_age_sec"] <= 0.10


def test_launch_does_not_auto_enable_sp():
    planner_launch = (
        PKG / "launch" / "planner_stack.launch.py"
    ).read_text(encoding="utf-8")
    sim_launch = (
        PKG / "launch" / "sim_race.launch.py"
    ).read_text(encoding="utf-8")
    assert "sp = ''" in planner_launch
    assert "sp = sp_candidate" not in planner_launch
    assert "overtaking_csv_file', default_value=''" in planner_launch
    assert "overtaking_csv_file', default_value=''" in sim_launch


def _sim_race_real_car_vehicle():
    """Read sim_race.launch.py's REAL_CAR_VEHICLE dict without executing it.

    The launch file cannot import ``deployment_profiles`` (that module lives in
    ``scripts/`` and is never installed into ``share/``), so the gym numbers are
    a hand-kept copy.  Parsing them here is what keeps the copy honest.
    """
    tree = ast.parse(
        SIM_RACE_LAUNCH.read_text(encoding="utf-8"),
        filename=str(SIM_RACE_LAUNCH))
    for node in ast.walk(tree):
        if not isinstance(node, ast.Assign):
            continue
        targets = [t.id for t in node.targets if isinstance(t, ast.Name)]
        if "REAL_CAR_VEHICLE" in targets:
            return ast.literal_eval(node.value)
    raise AssertionError(
        f"REAL_CAR_VEHICLE not found in {SIM_RACE_LAUNCH}")


def test_sim_gym_vehicle_matches_measured_plant():
    """gym 차량 물리(vehicle:=real_car)가 실측 상수와 어긋나면 실패.

    어긋나면 sim은 실차가 낼 수 없는 가속/그립으로 돌아서, 회피·정지거리
    검증이 통째로 낙관적이 된다 (2026-08-07 이전에 a_max가 2.2로 굳어 있어
    실차 3.5 대비 37% 느리게 가속했다).
    """
    vehicle = _sim_race_real_car_vehicle()
    assert vehicle["vehicle.a_max"] == pytest.approx(profiles.A_ACC_MAX)
    assert vehicle["vehicle.v_max"] == pytest.approx(profiles.RACELINE_V_MAX)
    assert vehicle["vehicle.s_max"] == pytest.approx(profiles.MAX_LEFT_STEER)
    assert vehicle["vehicle.s_min"] == pytest.approx(-profiles.MAX_RIGHT_STEER)
    # gym 차체는 recovery가 코리도 판정에 쓰는 치수와 같아야 한다.
    recovery = _ros_params(_yaml(PARAMS), "recovery_supervisor_node")
    assert vehicle["vehicle.width"] == pytest.approx(
        recovery["recovery/vehicle_width_m"])
    assert vehicle["vehicle.length"] == pytest.approx(
        recovery["recovery/vehicle_length_m"])

    # gym의 accl_constraints는 ±a_max 대칭이라 감속만 따로 못 준다.
    # control sim_real_car.yaml의 명령 slew가 그 역할을 하므로 여기서 함께 본다.
    control_sim = _ros_params(
        _yaml(WORKSPACE / "control_ws" / "src" / "control_pkg" / "config" /
              "environments" / "sim_real_car.yaml"),
        "control")
    assert control_sim["vehicle/max_accel_mps2"] == pytest.approx(
        profiles.A_ACC_MAX)
    assert control_sim["vehicle/max_decel_mps2"] == pytest.approx(
        profiles.A_DEC_PLANNED)
    assert control_sim["vehicle/max_left_steering_angle_rad"] == pytest.approx(
        profiles.MAX_LEFT_STEER)
    assert control_sim["vehicle/max_right_steering_angle_rad"] == pytest.approx(
        profiles.MAX_RIGHT_STEER)
    assert control_sim["vehicle/max_steering_rate_radps"] == pytest.approx(
        profiles.MAX_STEER_RATE)


# sim_real_car.yaml 이 real_car.yaml 과 **의도적으로** 다를 수 있는 키.
# 전부 배선(토픽·프레임·LUT)이고 거동 파라미터는 하나도 없다. 여기에 거동
# 파라미터를 추가하려면 "왜 sim에서 다른 값이어야 하는가"를 주석으로 남길 것.
SIM_REAL_CAR_WIRING_EXEMPT = {
    "planning": {
        # 둘 다 /pf/pose/odom 을 구독한다 (frenet_frame_node.cpp).
        "frenet_frame_node.frenet_frame/odom_mode",
        # 실차는 perception 팀 토픽, sim은 perception_pkg detector 출력.
        "perception_obstacle_bridge_node.bridge/input_topic",
    },
    "perception": {
        "obstacle_detector_node.scan_topic",
        "obstacle_detector_node.obstacle_topic",
        # sim에만 있는 노드 — virtual_obstacle_node와 마커 토픽이 겹쳐서 분리.
        "obstacle_viz_node.viz/input_topic",
        "obstacle_viz_node.viz/output_topic",
    },
    "control": {
        "control.ackermann_topic",
        # LUT는 하드웨어 한계가 아니라 타이어 모델이다. gym single-track에
        # 실차 Pacejka LUT를 물리면 실차와 무관한 추적오차가 생긴다.
        "control.map/steering_lut_file",
        # gym에는 VESC plant가 없어 명령단이 대신 차의 가감속/조향 한계를 건다.
        # 값 자체는 test_sim_gym_vehicle_matches_measured_plant 가 검사한다.
        "control.vehicle/max_accel_mps2",
        "control.vehicle/max_decel_mps2",
        "control.vehicle/max_steering_rate_radps",
    },
    "integration": {
        "integration.perception_topic",
        "integration.drive_topic",
    },
}

# env:=sim 이 real_car.yaml 과 **의도적으로** 다를 수 있는 키.
# sim_real_car 의 배선 예외에 더해, pf:=false(위치추정이 gym GT) 때문에 생기는
# 예외가 얹힌다 — 그 셋 말고는 env:=sim 도 실차와 같은 거동을 써야 한다.
SIM_WIRING_EXEMPT = {
    name: set(keys) for name, keys in SIM_REAL_CAR_WIRING_EXEMPT.items()
}
SIM_WIRING_EXEMPT["planning"] |= {
    # pf:=false 라 /pf/pose/odom 발행자가 없다 -> gym GT 를 쓴다.
    "recovery_supervisor_node.recovery/pose_odom_topic",
}
SIM_WIRING_EXEMPT["control"] |= {
    "control.odom_topic",
    # 게이트는 /pf/localization_ready 를 요구하는데 pf:=false 면 발행자가 없다.
    # 켜면 차가 영원히 출발하지 않으므로 env:=sim 에서는 반드시 꺼져 있어야 한다.
    "control.localization_gate/enabled",
}
SIM_WIRING_EXEMPT["perception"] |= {
    # odom 프레임 자체가 없으므로 time-travel lookup 을 쓸 수 없다.
    "obstacle_detector_node.tf_fixed_frame",
}
SIM_WIRING_EXEMPT["integration"] |= {"integration.odom_topic"}


SIM_REAL_CAR_PACKAGES = {
    "planning": PKG,
    "perception": WORKSPACE / "perception_ws" / "src" / "perception_pkg",
    "control": WORKSPACE / "control_ws" / "src" / "control_pkg",
    "integration": WORKSPACE / "integration_ws" / "src" / "integration_pkg",
}


def _env_profile_flat(pkg_root, env_name):
    """environments/<env>.yaml 을 '<노드>.<파라미터>' 평면 dict으로."""
    document = _yaml(Path(pkg_root) / "config" / "environments" /
                     f"{env_name}.yaml") or {}
    flat = {}
    for node, section in document.items():
        for key, value in (section or {}).get("ros__parameters", {}).items():
            flat[f"{node}.{key}"] = value
    return flat


def test_sim_real_car_profiles_mirror_real_car():
    """env:=sim_real_car 는 배선만 sim이고 거동은 실차와 같아야 한다.

    이 계약이 깨지면 "sim에선 됐는데 실차에선 안 되는" 원인이 파라미터인지
    물리인지 구분할 수 없게 된다. 실제로 bridge/wall_dist_min_m 이 real_car
    0.30 / sim 0.15 로 갈려 sim만 브릿지 벽필터가 사실상 꺼져 있었다.
    """
    for name, pkg_root in SIM_REAL_CAR_PACKAGES.items():
        real = _env_profile_flat(pkg_root, "real_car")
        sim = _env_profile_flat(pkg_root, "sim_real_car")
        exempt = SIM_REAL_CAR_WIRING_EXEMPT[name]
        drifted = sorted(
            key for key in set(real) | set(sim)
            if key not in exempt and real.get(key) != sim.get(key))
        assert not drifted, (
            f"{name}: sim_real_car.yaml 이 real_car.yaml 과 어긋남 -> "
            + ", ".join(
                f"{key} (real={real.get(key, '<absent>')!r}, "
                f"sim={sim.get(key, '<absent>')!r})" for key in drifted))


def test_sim_profiles_mirror_real_car():
    """env:=sim 도 배선(+pf:=false 파생)만 다르고 거동은 실차와 같아야 한다.

    2026-08-12 이전에는 sim.yaml 이 비어 있거나 저속 값이라, env:=sim 이
    velocity_scale 0.5714 / cmd_speed_scale 0.30 / lookahead 10 m 로 실차와
    전혀 다른 동작점에서 회피를 검증했다. 그 상태의 "sim에선 됐다"는 실차와
    대응되지 않는다. sim 과 sim_real_car 의 차이는 파라미터가 아니라 launch
    (pf / vehicle) 여야 한다.
    """
    for name, pkg_root in SIM_REAL_CAR_PACKAGES.items():
        real = _env_profile_flat(pkg_root, "real_car")
        sim = _env_profile_flat(pkg_root, "sim")
        exempt = SIM_WIRING_EXEMPT[name]
        drifted = sorted(
            key for key in set(real) | set(sim)
            if key not in exempt and real.get(key) != sim.get(key))
        assert not drifted, (
            f"{name}: sim.yaml 이 real_car.yaml 과 어긋남 -> "
            + ", ".join(
                f"{key} (real={real.get(key, '<absent>')!r}, "
                f"sim={sim.get(key, '<absent>')!r})" for key in drifted))


def test_static_cylinder_model_cannot_understate_extent():
    """정적 장애물 고정 원통이 extent 를 예측 불가능하게 줄이지 않는지.

    브릿지는 대역 안에서만 radius 를 고정한다. 그래서 **실제로 잃는 extent 의
    상한 = 대역 상한 - 고정값** 이고, 이 값이 회피 판정 마진(실측 6 cm)을
    크게 넘으면 "판정을 여는 방향"으로 밀린다. 근거는 rosbag run_0812_013630
    실측 — perception_obstacle_bridge_node.py 선언부 주석 참조.
    """
    for env_name in ("real_car", "sim_real_car", "sim"):
        # 벽 판정선은 base params.yaml 이 0.0(비활성)이고 env 에서 켜진다.
        perception = _ros_params(
            _yaml(WORKSPACE / "perception_ws" / "src" / "perception_pkg" /
                  "config" / "environments" / f"{env_name}.yaml"),
            "obstacle_detector_node",
        )
        env = _ros_params(
            _yaml(PKG / "config" / "environments" / f"{env_name}.yaml"),
            "perception_obstacle_bridge_node",
        )
        radius = env["bridge/static_cylinder_radius_m"]
        lo, hi = env["bridge/static_cylinder_band_m"]
        open_space = env["bridge/static_cylinder_open_space_m"]

        assert 0.0 < lo < hi, f"{env_name}: 대역이 뒤집혔다"
        assert lo <= radius <= hi, (
            f"{env_name}: 고정값이 대역 밖이면 적용 여부가 값 자체와 모순된다")
        # 최대 피해량. 0.15 여유는 차 반폭이고, 그 이상을 잃으면 벽까지의
        # 가짜 통로가 차폭에 근접한다.
        assert hi - radius <= 0.15, (
            f"{env_name}: 최대 extent 손실 {hi - radius:.3f}m 가 너무 크다")
        # detector 가 벽으로 거르는 크기를 넘어서까지 "콘"이라고 우기면 안 된다.
        max_cluster = perception["max_cluster_radius_m"]
        assert hi <= max_cluster, (
            f"{env_name}: 대역 상한 {hi} > detector 벽 판정선 {max_cluster}")
        # 벽 근처 클러스터는 벽 점에 오염돼 extent 가 실제로 커진다(실측
        # 중심 0~0.30 m 에서 radius p50 0.20, 그 바깥은 0.13).
        # 이 조건은 **중심 기준**이므로, 오염 경계를 덮으려면 고정 반지름
        # 이상이어야 한다 — 중심이 radius 만큼만 떨어져 있으면 표면이 벽에
        # 닿아 클러스터가 붙는다.
        assert open_space >= radius, (
            f"{env_name}: open_space {open_space} 가 고정 반지름 {radius} 미만 "
            "— 표면이 벽에 닿는 클러스터까지 고정하게 된다")


def test_bridge_reads_only_fields_that_exist_on_perception_obstacle():
    """브릿지가 **입력** 메시지에 없는 필드를 읽지 않는지.

    입력(f1tenth_perception_msgs/Obstacle)과 출력(f1tenth_planning_msgs)은
    이름이 비슷하고 필드가 다르다 — 입력엔 is_dynamic, 출력엔 is_static 이
    있다. 2026-08-12 고정 원통 커밋이 입력에서 src.is_static 을 읽어 **첫
    장애물 프레임에서** AttributeError 로 노드가 죽었고, 스트림이 끊기자
    spliner 가 fail-safe stop 을 발행해 차가 섰다. 위 파라미터 테스트들은
    YAML 숫자만 봐서 이걸 전혀 잡지 못했다.

    ROS 런타임 없이 .msg 정의와 소스 AST 만으로 본다.
    """
    msg_path = (WORKSPACE / "planning_ws" / "src" / "f1tenth_perception_msgs" /
                "msg" / "Obstacle.msg")
    fields = set()
    for line in msg_path.read_text(encoding="utf-8").splitlines():
        line = line.split("#", 1)[0].strip()
        if not line:
            continue
        parts = line.split()
        if len(parts) >= 2:
            fields.add(parts[1])
    assert "is_dynamic" in fields, "Obstacle.msg 파싱 실패 (테스트가 무력해짐)"

    source = (PKG / "scripts" / "planner" /
              "perception_obstacle_bridge_node.py")
    tree = ast.parse(source.read_text(encoding="utf-8"))
    # 입력 메시지를 담는 루프 변수. 이름이 바뀌면 아래 assert 가 알려준다.
    loop_var = "src"
    used = {
        node.attr
        for node in ast.walk(tree)
        if isinstance(node, ast.Attribute)
        and isinstance(node.value, ast.Name)
        and node.value.id == loop_var
    }
    assert used, (
        f"{source.name}: '{loop_var}.*' 접근이 하나도 없다 — 루프 변수 이름이 "
        "바뀌었으면 이 테스트도 같이 고칠 것")
    missing = sorted(used - fields)
    assert not missing, (
        f"{source.name}: 입력 Obstacle 에 없는 필드를 읽는다: {missing} "
        f"(정의된 필드: {sorted(fields)})")
