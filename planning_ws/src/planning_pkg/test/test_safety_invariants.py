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
    for env_name in ("sim", "real_car", "race_map"):
        if env_name == "sim":
            planning_scale = global_base["global_trajectory/velocity_scale"]
        else:
            planning_env = _yaml(
                PKG / "config" / "environments" / f"{env_name}.yaml")
            planning_scale = _ros_params(
                planning_env, "global_trajectory_node"
            )["global_trajectory/velocity_scale"]
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
        assert math.isclose(
            csv_vmax * planning_scale, control_cap,
            rel_tol=0.0, abs_tol=0.02,
        ), env_name
        assert actual_csv_vmax * planning_scale * control_scale <= \
            control_cap + 1e-6, env_name


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
    planning = _ros_params(_yaml(PARAMS), "spliner_node")
    tracker = _ros_params(_yaml(PARAMS), "waypoint_tracker_node")
    control_sim = _ros_params(
        _yaml(WORKSPACE / "control_ws" / "src" / "control_pkg" /
              "config" / "environments" / "sim.yaml"),
        "control",
    )
    assert math.isclose(
        planning["spliner/cmd_speed_scale"],
        tracker["tracker/speed_scale"],
        rel_tol=0.0, abs_tol=1e-9,
    )
    assert math.isclose(
        planning["spliner/cmd_speed_scale"],
        control_sim["vehicle/speed_scale"],
        rel_tol=0.0, abs_tol=1e-9,
    )


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
    assert vehicle["vehicle.s_max"] == pytest.approx(profiles.MAX_STEER)
    assert vehicle["vehicle.s_min"] == pytest.approx(-profiles.MAX_STEER)
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
    assert control_sim["vehicle/max_steering_angle_rad"] == pytest.approx(
        profiles.MAX_STEER)
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
        "control.vehicle/max_steering_angle_rad",
        "control.vehicle/max_steering_rate_radps",
    },
    "integration": {
        "integration.perception_topic",
        "integration.drive_topic",
    },
}

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
