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
    # SP 레인체인지는 2026-08-22 (2/8단계)에 통째로 제거됐다 — 파라미터가
    # 남아 있으면 declare 되지 않은 키라 기동이 거부된다.
    assert "spliner/sp_lane_change" not in spliner
    # perception lease. 짧을수록 "빨리 알아챈다"가 아니다 — 초과하면 spliner가
    # fail-safe 정지 궤적(vx=0)을 발행하므로, detector 실측 발행 주기보다
    # 짧으면 정상 스트림에서도 프레임 사이마다 정지가 끼어든다.
    # run_0803_213313: 0.20 < detector 주기 0.246s -> 28.6초 정지 라치.
    # 상한은 "탐지 실패를 이만큼 안에는 알아챈다"는 안전 계약이므로 유지하되,
    # detector가 목표대로 >=15Hz가 되면 다시 조일 것.
    assert spliner["spliner/obs_timeout_sec"] <= 0.60
    # 추월 속도 배율(ot_speed_scale)은 2026-08-22 (4/8단계)에 제거됐다.
    # 남아 있으면 declare 되지 않은 키라 기동이 거부된다.
    assert "spliner/ot_speed_scale" not in spliner
    assert local["local_trajectory/require_avoidance_heartbeat"] is True
    assert local["local_trajectory/avoidance_timeout_sec"] <= 0.25
    assert local["local_trajectory/frenet_odom_source_timeout_sec"] <= 0.25
    # recovery 체인 부활 (2026-08-22): 기본 활성. 노드 기동은 launch
    # (recovery:=auto)가, 내부 게이트는 이 값이 결정한다. 발동은 "완전히
    # 박았을 때"로 한정되고, allow_reverse가 운용 킬스위치다.
    assert recovery["recovery/enabled"] is True
    assert recovery["recovery/allow_reverse"] is True
    assert tracker["tracker/recovery_required"] is False
    assert tracker["tracker/recovery_allow_reverse"] is True
    # 재설계에서 삭제된 rear/align 계열 — 남아 있으면 declare 되지 않은 키라
    # 기동이 거부된다.
    for removed in (
        "recovery/rear_coverage_confirmed",
        "recovery/rear_check_dist_m",
        "recovery/front_check_dist_m",
        "recovery/align_tolerance_rad",
        "recovery/align_speed_mps",
        # 경계 예측 정지 계열 — run_0822_191159 에서 정상 주행을 오탐해 삭제.
        "recovery/boundary_stop_margin_m",
        "recovery/boundary_persist_sec",
        "recovery/boundary_reaction_time_sec",
        "recovery/boundary_braking_decel_mps2",
        "recovery/require_boundary_horizon",
        "recovery/resume_grace_sec",
    ):
        assert removed not in recovery, removed
    # 벽 접촉 임계: run_0822_191159 의 진짜 접촉(-0.00m)은 잡고 오탐(0.31m)은
    # 거르는 구간 안에 있어야 한다.
    assert -0.05 <= recovery["recovery/wall_contact_margin_m"] <= 0.15
    # 후진/복구 파라미터의 안전 범위. 후진 요청 속도는 control의 하드캡보다
    # 크면 잘려서 무의미해지므로 캡 이하를 강제한다.
    assert 0.3 <= recovery["recovery/recovery_max_reverse_distance_m"] <= 1.0
    assert 2 <= recovery["recovery/recovery_max_attempts"] <= 5
    assert recovery["recovery/progress_reset_m"] >= \
        2.0 * recovery["recovery/recovery_max_reverse_distance_m"]
    control_base = _ros_params(
        _yaml(WORKSPACE / "control_ws" / "src" / "control_pkg" /
              "config" / "params.yaml"),
        "control",
    )
    assert recovery["recovery/recovery_reverse_speed_mps"] <= \
        control_base["recovery/max_reverse_speed_mps"]
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


def test_control_profiles_disable_recovery_chain():
    control_base = _ros_params(
        _yaml(WORKSPACE / "control_ws" / "src" / "control_pkg" /
              "config" / "params.yaml"),
        "control",
    )
    # base는 벤치/단독 실행 fail-safe로 계속 꺼져 있다.
    assert control_base["recovery/allow_reverse"] is False
    assert control_base["recovery/require_supervisor"] is False
    # 2026-08-22 recovery 전면 비활성: launch가 supervisor를 띄우지 않으므로
    # 주행 env 4종은 lease를 요구하면 안 된다(요구하면 출발 자체가 막힌다).
    # 후진 소비도 함께 닫아 둔다 — 되살릴 때 둘 다 true로 되돌릴 것.
    for env_name in ("real_car", "race_map", "sim", "sim_real_car"):
        control_env = _ros_params(
            _yaml(WORKSPACE / "control_ws" / "src" / "control_pkg" /
                  "config" / "environments" / f"{env_name}.yaml"),
            "control",
        )
        assert control_env["recovery/require_supervisor"] is False, env_name
        assert control_env["recovery/allow_reverse"] is False, env_name
        assert control_env["vehicle/speed_scale"] <= 1.0, env_name


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


def test_launch_disables_recovery_by_default():
    """recovery 전면 비활성(2026-08-22): launch 기본이 false여야 한다.

    control env의 require_supervisor=false와 쌍이다. 한쪽만 되돌리면
    (기본 false + require true) 차가 출발조차 못 하는 조합이 된다.
    """
    planner_launch = (
        PKG / "launch" / "planner_stack.launch.py"
    ).read_text(encoding="utf-8")
    sim_launch = SIM_RACE_LAUNCH.read_text(encoding="utf-8")
    assert "'recovery', default_value='false'" in planner_launch
    # auto 경로 자체는 남겨 둔다 (recovery:=auto 로 되살릴 수 있어야 한다).
    assert "env != 'mapping'" in planner_launch
    # sim_race는 인자를 선언하고 planner_stack으로 통과시켜야 한다.
    assert "'recovery', default_value='false'" in sim_launch
    assert "'recovery': LaunchConfiguration('recovery')" in sim_launch


def test_recovery_supervisor_source_invariants():
    """상태머신 재설계(2026-08-22)의 소스 수준 회귀 방지.

    - rear-safe 게이트와 RECOVER_ALIGN은 삭제됐다 — 되살아나면 벽 크래시가
      다시 STOP 막다른 길이 된다.
    - 경계 예측 정지(COLLISION_STOP)는 삭제됐다 — run_0822_191159 에서 실제
      여유 0.84m 인데도 3회 모두 트립했고, 정지 시 조향이 0으로 꺾여 차를
      벽으로 보냈다.
    - 후진 타임아웃은 STOP 래치가 아니라 RECOVER_STOP(전진 재시도)이어야 한다.
    - 복귀 grace 파라미터는 declare 되어 있어야 한다.
    """
    src = (PKG / "src" / "recovery_supervisor_node.cpp").read_text(
        encoding="utf-8")
    hdr = (
        PKG / "include" / "planning_pkg" / "recovery_supervisor_node.hpp"
    ).read_text(encoding="utf-8")
    for banned in (
        "IsRearSafe", "RECOVER_ALIGN", "rear_coverage_confirmed",
        # 경계 예측 정지 — 되살리면 정상 주행이 다시 멈추고 조향이 끊긴다.
        "IsBoundaryViolation", "State::COLLISION_STOP", "StoppingDistance",
    ):
        assert banned not in src, banned
        assert banned not in hdr, banned
    assert re.search(
        r"State::RECOVER_STOP,\s*\"reverse timeout", src), \
        "reverse timeout은 RECOVER_STOP으로 가야 한다 (STOP 래치 회귀 금지)"
    assert 'declare_parameter("recovery/progress_reset_m"' in src
    assert 'declare_parameter("recovery/wall_contact_margin_m"' in src
    # 후진 진입은 벽 접촉 게이트를 반드시 통과해야 한다.
    assert re.search(r"!IsStuck\(now_sec\)\s*\|\|\s*!wall_contact", src), \
        "후진 진입에 벽 접촉 게이트가 없다"


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


def test_oversized_clusters_are_discarded_in_deployment_profiles():
    """벽 판정 원본은 조각으로 되살리지 않는다는 인지-플래너 계약.

    분할 뒤에는 원본의 반경/provenance가 사라져 브릿지와 spliner가 벽 fringe를
    정상 장애물로 구별할 수 없다. 따라서 실차 및 실차 대응 sim에서는 반드시
    원본 단위로 폐기해야 한다.
    """
    perception_pkg = WORKSPACE / "perception_ws" / "src" / "perception_pkg"
    detector_src = (perception_pkg / "src" / "obstacle_detector_node.cpp").read_text(
        encoding="utf-8")
    assert re.search(
        r'declare_parameter<bool>\(\s*"cluster_split_enabled"\s*,\s*false\s*\)',
        detector_src), "cluster_split_enabled 기본값은 fail-safe false여야 한다"
    assert re.search(
        r'bool\s+cluster_split_enabled_\s*\{false\}', detector_src), (
        "멤버 초기값도 fail-safe false여야 한다")

    base = _ros_params(_yaml(perception_pkg / "config" / "params.yaml"),
                       "obstacle_detector_node")
    assert base["cluster_split_enabled"] is False
    for env_name in ("real_car", "sim_real_car", "sim", "race_map"):
        env = _env_profile_flat(perception_pkg, env_name)
        enabled = env.get("obstacle_detector_node.cluster_split_enabled",
                          base["cluster_split_enabled"])
        assert enabled is False, (
            f"{env_name}: 초대형 클러스터를 분할하면 벽 fringe가 플래너로 새어 나간다")


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


def test_visible_static_wall_relaxation_stays_inside_detector_guard():
    """08-22 완화는 raw detector의 point-level wall guard 밖으로 안 나간다."""
    for env_name in ("real_car", "sim_real_car", "sim", "race_map"):
        perception = _ros_params(
            _yaml(WORKSPACE / "perception_ws" / "src" / "perception_pkg" /
                  "config" / "environments" / f"{env_name}.yaml"),
            "obstacle_detector_node",
        )
        bridge = _ros_params(
            _yaml(PKG / "config" / "environments" / f"{env_name}.yaml"),
            "perception_obstacle_bridge_node",
        )
        base = bridge["bridge/wall_dist_min_m"]
        slack = bridge["bridge/visible_static_wall_slack_m"]
        hold = bridge["bridge/visible_static_hold_sec"]

        assert 0.0 < slack <= base
        assert base - slack >= perception["wall_dist_min_m"], (
            f"{env_name}: bridge relaxation falls below detector map guard")
        assert 0.0 < hold <= min(1.0, bridge["bridge/track_forget_sec"])


def test_h2h_dynamic_shadow_is_explicit_and_wall_bounded():
    """H2H ghost containment must not become a generic static-object filter."""
    for env_name in ("real_car", "sim_real_car", "sim", "race_map"):
        bridge = _ros_params(
            _yaml(PKG / "config" / "environments" / f"{env_name}.yaml"),
            "perception_obstacle_bridge_node",
        )
        assert bridge["bridge/dynamic_static_shadow_enabled"] is True
        assert 0.05 <= bridge["bridge/dynamic_static_shadow_behind_m"] <= 0.25
        assert 2.18 <= bridge["bridge/dynamic_static_shadow_ahead_m"] <= 2.50
        assert 0.80 <= bridge["bridge/dynamic_static_shadow_d_m"] <= 1.00
        # run_0822_hth #105 last visible -> #110 ego pass was about 2.29 s.
        assert 2.30 <= bridge["bridge/dynamic_static_shadow_hold_sec"] <= 3.0
        assert bridge["bridge/use_map_filter"] is True
        assert bridge["bridge/wall_dist_min_m"] <= \
            bridge["bridge/dynamic_static_shadow_wall_max_m"] <= 0.60
        # The source's raw dynamic state is the H2H witness; a confidence reset
        # must not turn the actual opponent into the accompanying static ghost.
        assert bridge["bridge/force_low_confidence_dynamic_static"] is False


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


def test_low_speed_floors_stay_aligned():
    """저속 바닥 세 개가 같은 값이라는 주장을 코드가 지키는지 확인한다.

    2026-08-17 커밋(afb72d4)이 `avoidance_loss_hold_floor_mps` 를 신설하며
    "spliner/trailing_retry_speed_degraded, control 의 backstop_min_speed_mps
    와 같은 값으로 맞췄다"고 선언했다. 세 값은 서로 다른 파일에 있고 아무도
    묶어 두지 않아서, 하나만 튜닝하면 그 선언이 조용히 거짓이 된다.
    같은 값이어야 하는 이유는 셋이 같은 물리량이기 때문이다 — "차를 세우지
    않고 유지할 수 있는 최저 전진속도". 어긋나면 한 층이 다른 층보다 먼저
    0 에 가까워지고, 전 점 vx=0 이 MAP 조향을 죽이는 경로가 다시 열린다.
    """
    planning = _yaml(PARAMS)
    hold_floor = _ros_params(
        planning, "local_trajectory_node"
    )["local_trajectory/avoidance_loss_hold_floor_mps"]
    retry_degraded = _ros_params(
        planning, "spliner_node"
    )["spliner/trailing_retry_speed_degraded"]

    control_params = (
        WORKSPACE / "control_ws" / "src" / "control_pkg" /
        "config" / "params.yaml"
    )
    assert control_params.is_file(), control_params
    control = _ros_params(_yaml(control_params), "control")
    backstop_floor = control["map/backstop_min_speed_mps"]

    assert hold_floor == retry_degraded == backstop_floor, (
        "저속 바닥이 어긋났다: "
        f"avoidance_loss_hold_floor={hold_floor} "
        f"trailing_retry_speed_degraded={retry_degraded} "
        f"backstop_min_speed={backstop_floor}")

    # 2026-08-21 실차 채택값 1.6은 1.5 m/s trailing floor까지 구제하되
    # 그보다 높은 정상 제동 구간은 건드리지 않도록 딱 0.1 m/s만 연 값이다.
    # 옛 단언(< backstop_floor)은 이 의도된 1.6을 거부해 검증 자체가 막혔다.
    # 반대로 0.1보다 더 열리면 "계획이 무너진 경우만"이라는 전제를 벗어나
    # 정상 제동 구간 조향 게인을 바꾸므로 여기서 계속 막는다.
    steer_min = control["map/steer_plan_speed_min_mps"]
    assert steer_min == pytest.approx(backstop_floor + 0.1), (
        f"steer_plan_speed_min_mps={steer_min} 가 실증된 저속 구제 범위 "
        f"{backstop_floor + 0.1}와 다르다")


# --- sim <-> 실차 위치추정(PF) 파리티 ------------------------------------------
PF_PKG = WORKSPACE / "localization_ws" / "src" / "particle_filter_cpp"
PF_REAL_YAML = PF_PKG / "config" / "localize.yaml"
PF_SIM_YAML = PF_PKG / "config" / "localize_sim.yaml"
PF_NODE_SRC = PF_PKG / "src" / "particle_filter_node.cpp"

# sim 이 실차와 **의도적으로** 다른 PF 키. 전부 배선(토픽·프레임)이다.
PF_WIRING_EXEMPT = {
    "scan_topic",        # 실차 /scan, sim 은 가상장애물이 합성된 /scan_overlay
    "base_frame",        # gym 네임스페이스
    "laser_frame",
    "imu_topic",         # gym 에는 IMU 가 없다 (아래 주석 참조)
}


def _pf_declared_defaults():
    """particle_filter_node.cpp 의 declare_parameter<T>("name", default) 수집."""
    source = PF_NODE_SRC.read_text(encoding="utf-8")
    defaults = {}
    for match in re.finditer(
            r'declare_parameter\s*(?:<[^>]*>)?\s*\(\s*"([^"]+)"\s*,', source):
        index, depth, buffer = match.end(), 1, ""
        while index < len(source) and depth > 0:
            char = source[index]
            if char == "(":
                depth += 1
            elif char == ")":
                depth -= 1
                if depth == 0:
                    break
            buffer += char
            index += 1
        defaults[match.group(1)] = " ".join(buffer.split())
    assert len(defaults) > 50, "declare_parameter 수집 실패 — 정규식을 확인할 것"
    return defaults


def _as_scalar(text):
    literal = str(text).strip().strip('"').lower()
    if literal in ("true", "false"):
        return literal == "true"
    try:
        return float(literal)
    except ValueError:
        return literal


def test_sim_pf_profile_matches_real_car():
    """sim PF 의 **실효** 설정이 실차 localize.yaml 과 같아야 한다.

    localize_sim.yaml 은 실차 파일의 부분집합이라, 안 적은 키는 C++ 노드
    기본값으로 돈다. 그 기본값이 실차 값과 다르면 sim 은 다른 PF 를 돌리는
    셈인데 어느 파일을 봐도 드러나지 않는다.

    2026-08-18 실제 사고: deskew_enabled / ground_filter_enabled 의 노드
    기본값이 true 인데 실차만 false 를 명시하고 있었다. 이 둘이 켜져 있으면
    particle_filter_cpp 는 validate_supported_configuration() 에서 예외를
    던지고 **즉사한다** — PF 를 C++ 로 바꾼 뒤로 run_sim.sh 의 기본 프로파일
    (sim_real_car, pf:=true)에 위치추정이 아예 없었다.
    """
    defaults = _pf_declared_defaults()
    real = _ros_params(_yaml(PF_REAL_YAML), "particle_filter")
    sim = _ros_params(_yaml(PF_SIM_YAML), "particle_filter")

    drifted = []
    for key, real_value in real.items():
        if key in PF_WIRING_EXEMPT or key not in defaults:
            continue          # 노드가 선언하지 않는 키는 양쪽 다 죽은 값이다
        effective = sim[key] if key in sim else _as_scalar(defaults[key])
        if _as_scalar(effective) != _as_scalar(real_value):
            source = "localize_sim.yaml" if key in sim else "노드 기본값"
            drifted.append(f"{key} (real={real_value!r}, sim={effective!r} <- {source})")
    assert not drifted, (
        "sim PF 실효 설정이 실차와 어긋남 -> " + ", ".join(drifted))


def test_sim_pf_passes_supported_profile_gate():
    """PF 노드가 거부하는 조합을 sim 설정이 만들지 않는지 직접 확인.

    위 파리티 테스트와 겹쳐 보이지만 다른 것을 막는다: 실차 쪽 값이 바뀌면
    파리티는 계속 통과하면서 sim 만 기동 불가가 될 수 있다.
    """
    sim = _ros_params(_yaml(PF_SIM_YAML), "particle_filter")
    assert sim.get("ground_filter_enabled") is False
    assert sim.get("deskew_enabled") is False
    assert sim.get("range_method") == "glt"
    assert "refusing to ignore enabled geometry features" in \
        PF_NODE_SRC.read_text(encoding="utf-8"), \
        "PF 의 프로파일 게이트가 사라졌다면 이 테스트의 전제를 다시 볼 것"


# --- sim <-> 실차 LiDAR 기하 ---------------------------------------------------
# run_0817_194156 /scan 실측 (실차 Hokuyo). 이 셋이 인지의 '거리별 클러스터
# 점수'를 결정하므로, min_cluster_size 같은 실차 백 기반 문턱을 sim 에서
# 재현하려면 같아야 한다.
REAL_LIDAR_BEAMS = 1081
REAL_LIDAR_FOV_RAD = 4.712389          # = 3π/2, angle_min/max = ∓2.356194
REAL_LIDAR_ANGLE_INCREMENT = 0.00436332
GYM_BRIDGE_SRC = (WORKSPACE / "sim_ws" / "src" / "f1tenth_gym_ros" /
                  "f1tenth_gym_ros" / "gym_bridge.py")


def test_sim_gym_lidar_matches_real_scan():
    """sim LiDAR 가 실차 /scan 과 맞춰질 수 있는 만큼 맞춰져 있는지."""
    vehicle = _sim_race_real_car_vehicle()

    # 라이다 장착 위치는 hardware 실측 extrinsic 이 단일 기준이다.
    # 이건 f110_gym 에 lidar_dist 로 실제 전달되므로 물리에 반영된다.
    sensors = _yaml(WORKSPACE / "hardware_ws" / "src" / "f1tenth_stack" /
                    "config" / "sensors.yaml")
    base_to_laser_x = _ros_params(sensors, "sensor_extrinsics")["base_to_laser_x"]
    assert vehicle["scan_distance_to_base_link"] == pytest.approx(base_to_laser_x)

    # 빔 수/FOV 는 **덮으면 안 된다**. f110_gym 은 num_beams=1080 / fov=4.7 로
    # 하드코딩돼 있고 그 인자를 받는 경로가 없어서, gym_config 의 값을 바꾸면
    # 실제 레이와 다른 헤더를 발행하게 된다 (구독자가 복원하는 점이 통째로 틀어짐).
    assert "scan_beams" not in vehicle and "scan_fov" not in vehicle, (
        "scan_beams/scan_fov 를 덮으면 헤더만 바뀌고 실제 레이는 그대로다 — "
        "f110_gym 을 벤더링해서 고치지 않는 한 넣지 말 것")

    # gym_bridge 가 publish 하는 angle_increment 는 f110_gym 내부 빔 간격과
    # 같아야 한다 (ScanSimulator2D 는 fov/(num_beams-1) 로 쏜다). 예전엔
    # fov/num_beams 를 발행해서 마지막 빔 방향이 한 칸(≈0.25°) 틀어져 있었다.
    assert "scan_fov / (scan_beams - 1)" in \
        GYM_BRIDGE_SRC.read_text(encoding="utf-8"), \
        "gym_bridge 의 angle_inc 가 f110_gym 빔 간격과 어긋난다"

    # 남는 각분해능 차이는 실차 대비 0.5% 안이어야 한다. 인지의 클러스터 점수
    # 문턱(min_cluster_size 등)이 전부 실차 백에서 나왔으므로, 여기가 벌어지면
    # 같은 물체가 sim 에서 다른 점수로 잡힌다.
    gym = _ros_params(
        _yaml(WORKSPACE / "sim_ws" / "src" / "f1tenth_gym_ros" / "config" /
              "sim.yaml"), "bridge")
    sim_increment = gym["scan_fov"] / (gym["scan_beams"] - 1)
    assert sim_increment == pytest.approx(REAL_LIDAR_ANGLE_INCREMENT, rel=5e-3), (
        f"sim 각분해능 {sim_increment} 가 실차 {REAL_LIDAR_ANGLE_INCREMENT} 와 "
        "0.5% 넘게 다르다")


# ── 정적/동적 분리 불변식 (2026-08-19) ──────────────────────────────────────
# run_0818 수정에서 넣은 동적 전용 완화들은 전부 "정적 경로는 건드리지 않는다"를
# 전제로 승인됐다. 그 전제가 조용히 깨지면(게이트 한 줄이 사라지면) 오래 튜닝된
# 정적 회피 거동이 통째로 바뀌므로, 소스 수준에서 못 박아 둔다.
SPLINER_CPP = PKG / "src" / "spliner_node.cpp"


def _spliner_source():
    return SPLINER_CPP.read_text(encoding="utf-8")


def test_lat_budget_is_reset_every_cycle():
    """예산이 사이클을 넘어 새면 정적 경로가 동적 예산으로 계산된다."""
    src = _spliner_source()
    # active_speed_scale_ 이 정확히 이 리셋 누락으로 정적에 25% 부스트를
    # 흘린 전례가 있다 (그 주석이 spliner_node.cpp Run() 에 남아 있다).
    assert re.search(
        r"active_lat_budget_\s*=\s*path_lat_acc_max_;", src), (
        "사이클 시작 리셋(active_lat_budget_ = path_lat_acc_max_)이 없다"
    )
    # 대입은 장애물 분기 안에서만, 그리고 반드시 게이트를 거쳐야 한다.
    unconditional = re.findall(
        r"active_lat_budget_\s*=\s*dyn_path_lat_acc_max_\s*;", src)
    assert not unconditional, (
        "dyn_path_lat_acc_max_ 를 무조건 대입하는 곳이 생겼다: %r" % unconditional
    )


def test_committed_lat_budget_is_frozen():
    """커밋 재생은 생성 시점 예산으로 돌아야 한다 (분류가 뒤집혀도)."""
    src = _spliner_source()
    # 커밋 재생은 기하 상한만 재사용하고 전방 가속 envelope 는 **현재** 예산으로
    # 다시 푼다. 그 사이 예산이 바뀌면 같은 기하에서 속도가 점프한다.
    assert "committed_lat_budget_ = active_lat_budget_;" in src, (
        "커밋 생성 시 예산 동결이 사라졌다"
    )
    assert re.search(
        r"if \(committed_lat_budget_ > 0\.0\) \{\s*"
        r"active_lat_budget_ = committed_lat_budget_;", src, re.S), (
        "커밋 유지 사이클에 동결 예산 복원이 사라졌다"
    )
    # 커밋이 끝나면 반드시 풀려야 한다 (다음 커밋이 물려받으면 안 된다).
    assert src.count("committed_lat_budget_ = -1.0;") >= 2, (
        "커밋 만료/파기 시 예산 해제가 빠졌다"
    )


def test_speed_profile_reads_active_budget_only():
    """ApplySpeedProfile / BlendLatAcc 이 정적 예산을 직접 읽으면 분리가 무의미하다."""
    src = _spliner_source()
    # 곡률 캡 · 마찰원 종그립 차감 · reachable 성립성 · BlendLatAcc 네 곳이
    # 모두 active_lat_budget_ 를 읽어야 한다.
    for needle in (
        "if (active_lat_budget_ > 1e-3) {",
        "std::sqrt(active_lat_budget_ / k)",
        "std::clamp(lat / active_lat_budget_, 0.0, 1.0)",
        "k * simulated_v * simulated_v > active_lat_budget_",
        "const double budget = active_lat_budget_;",
    ):
        assert needle in src, f"예산 소비처가 정적 값으로 되돌아갔다: {needle}"
    # path_lat_acc_max_ 의 남은 용도는 설정/검증/리셋/게이트뿐이다.
    # 속도 프로파일 본문에서 다시 읽히면 분리가 깨진 것이다.
    body = src[src.index("bool SplinerNode::ApplySpeedProfile"):]
    assert "path_lat_acc_max_" not in body, (
        "ApplySpeedProfile 본문이 path_lat_acc_max_ 를 직접 읽는다"
    )


def test_entry_runway_has_no_closing_correction():
    """진입 활주로 접근속도 보정이 남아 있으면 안 된다.

    2026-08-22 (3/8단계): 이 티어는 정적 전용이 됐으므로 "상대가 다가오며
    활주로가 줄어든다"는 상황 자체가 없다. 보정 배율은 항상 >= 1 이라
    정적에 적용되면 회피 가부가 통째로 헐거워진다.
    """
    src = _spliner_source()
    assert re.search(
        r"const double entry_avail =\s*"
        r"std::max\(dist_front - vehicle_half_length_m_, 0\.0\);", src), (
        "entry_avail 이 단순 기하값이 아니다 — 접근속도 보정이 되살아났다"
    )
    assert "qs_min_closing_mps_" not in src, (
        "준정적 추월 잔재(qs_min_closing_mps_)가 남아 있다"
    )


def test_dynamic_obstacles_never_enter_the_evasion_tiers():
    """동적 상대는 회피(횡) 티어에 들어가지 않는다 — 2026-08-22 재작성의 핵심 계약.

    새 계약: 회피는 정적 전용, 동적은 종방향 추종(+7단계 직선 추월)만.
    이 게이트가 풀리면 폐기한 동적 회피가 뒷문으로 되살아난다.
    """
    src = _spliner_source()
    # ① 회피 시도 게이트가 정적 전용이다.
    assert "bool try_evade = closest.is_static;" in src, (
        "try_evade 가 정적 전용이 아니다 — 동적이 회피 티어로 들어간다"
    )
    # ② 폐기한 동적 기구가 되살아나지 않았다.
    code = "\n".join(ln for ln in src.splitlines()
                     if not ln.strip().startswith("//"))
    for gone in ("BuildSpLaneChange", "BuildSpExit", "QuasiStaticOvertake",
                 "IsPredictionTube", "PredictObstacleMovement",
                 "dyn_path_lat_acc_max_", "dyn_trailing_floor_",
                 "trailing_decel_dyn_", "dyn_pass_speed_ratio_",
                 "dyn_speed_floor_", "ot_speed_scale_"):
        assert gone not in code, f"폐기한 동적 기구가 되살아났다: {gone}"
    # ③ 커밋은 정적 전용이다 (동적에 커밋을 걸면 상대가 움직여도 얼어붙는다).
    assert re.search(
        r"const bool commit_candidate =\s*"
        r"evasion_valid && !used_trailing &&\s*"
        r"closest\.is_static &&", src), (
        "커밋 승격이 정적 전용이 아니다"
    )


def test_follow_cap_is_longitudinal_only():
    """동적 추종은 **속도만** 정한다 — 회피 경로의 횡을 건드리면 안 된다.

    "정적 회피가 최우선"이 코드 구조로 성립하는 근거가 이것이다. 캡이
    d_m 을 만지기 시작하면 회피 경로가 추종에 끌려간다.
    """
    src = _spliner_source()
    blk = src[src.index("동적 상대 종방향 추종 (2026-08-22, 6/8단계)"):]
    blk = blk[:blk.index("pitwall::Log(\"follow_active\"")]
    assert "w.vx_mps = cap_w;" in blk, "추종 캡이 vx_mps 를 누르지 않는다"
    for lateral in ("d_m", "d_left", "d_right", "x_m", "y_m"):
        assert f"w.{lateral}" not in blk, (
            f"추종 캡이 횡/위치 필드({lateral})를 건드린다 — 회피 우선이 깨진다"
        )
    # 회피 경로가 있으면 새로 만들지 않고 캡만 씌운다.
    assert "if (evasion_valid) {" in blk, (
        "회피 경로가 있을 때와 없을 때를 구분하지 않는다"
    )


def test_follow_cap_has_both_terms_and_a_physical_engage():
    """추종 캡의 비례항·안전항·개입 게이트가 전부 살아 있어야 한다."""
    src = _spliner_source()
    fn = src[src.index("double SplinerNode::OpponentFollowCap("):]
    fn = fn[:fn.index("\ndouble SplinerNode::ObsVsMean(")]
    # 개입 게이트: 목표 간격 + Δv²/(2a) + v·T
    assert "dyn_follow_gap_m_ + dv * dv / (2.0 * a) + v_ego * t_react" in fn, (
        "개입 거리가 속도차 의존이 아니다 — 급접근에서 제동 거리가 없다"
    )
    # 비례항
    assert "v_opp + dyn_follow_kp_ * (gap - dyn_follow_gap_m_)" in fn, (
        "비례 추종항이 없다 — 간격이 목표로 수렴하지 않는다"
    )
    # 안전항: 반응지연 주행분을 뺀 room 으로 sqrt(v_opp^2 + 2a·room)
    assert "gap - dyn_follow_gap_min_m_ - v_ego * t_react" in fn, (
        "안전항의 room 에서 반응지연 주행분을 빼지 않는다"
    )
    assert "std::sqrt(v_opp * v_opp + 2.0 * a * room)" in fn, "안전항이 없다"
    # 반응지연은 인지-계획 + 구동계 둘 다여야 한다.
    assert "planning_delay_sec_" in fn and "trailing_lag_sec_" in fn, (
        "반응지연이 한쪽만 반영됐다"
    )


def test_follow_gap_params_are_physical():
    """추종 간격 파라미터가 실측 접촉 거리와 lookahead 사이에 있어야 한다."""
    spliner = _ros_params(_yaml(PARAMS), "spliner_node")
    gap = spliner["spliner/dyn_follow_gap_m"]
    gap_min = spliner["spliner/dyn_follow_gap_min_m"]
    kp = spliner["spliner/dyn_follow_kp"]
    # 0.44 = run_0819_211107 89.51s 실측 접촉 거리 (obs.s_start - cur_s_).
    assert gap_min > 0.44, (
        f"안전 하한 {gap_min} 이 실측 접촉 거리 0.44 이하다 — 닿고 나서 막는다"
    )
    assert gap > gap_min, "목표 간격이 안전 하한보다 작다"
    assert gap <= spliner["spliner/lookahead_m"], "목표 간격이 lookahead 밖이다"
    assert 0.0 < kp <= 5.0


def test_straight_overtake_keeps_all_five_gates():
    """유일한 추월 기동의 발화 조건 5개가 전부 남아 있어야 한다."""
    src = _spliner_source()
    fn = src[src.index("bool SplinerNode::TryStraightOvertake("):]
    fn = fn[:fn.index("\nbool SplinerNode::BuildTrailing(")]
    # ② 직선 + 활주로
    assert "StraightEndAhead(cur_s_)" in fn, "직선 판정이 없다"
    assert 'NoteReject("ot_no_runway")' in fn, "활주로 검사가 없다"
    # ③ 직선 안 정적 장애물 0개
    assert 'NoteReject("ot_static_in_straight")' in fn, (
        "직선에 정적 장애물이 있어도 추월을 시도한다 — 회피 우선이 깨진다"
    )
    # ④ 간격 상한
    assert "gap > ot_max_gap_m_" in fn, "간격 상한 게이트가 없다"
    # ⑤ 벽/상대 여유
    assert "RequiredObstacleClearance(opp)" in fn, "상대 요구 간격을 안 본다"
    assert "MinWallInSpan(" in fn, "기동 구간 벽 여유를 안 본다"
    # 최종 검증 3종은 반드시 남아야 한다.
    for guard in ("WithinTrackBoundary(", "ApplySpeedProfile(",
                  "CheckPathCollision("):
        assert guard in fn, f"추월 경로 검증이 빠졌다: {guard}"
    # 횡이동은 상대 옆에 닿기 전에 끝나야 한다.
    assert "blend_in > must_finish_by" in fn, (
        "상대 옆에서 횡이동이 끝나는 기하를 막지 않는다"
    )


def test_overtake_suspends_the_follow_cap():
    """추월 중에는 추종 캡을 걸지 않는다 — 상대 속도에 묶이면 못 지나간다."""
    src = _spliner_source()
    assert "if (engaged && !ot_now) {" in src, (
        "추월 중에도 추종 캡이 걸린다 — 레이트 브레이크가 성립하지 않는다"
    )
    # 추월은 추종 중에만, 그리고 정적 회피 경로가 없을 때만 연다.
    assert "if (engaged && !evasion_valid &&" in src, (
        "추월이 추종/회피 우선 게이트를 안 거친다"
    )
    # 상승엣지 플래그는 매 사이클 갱신돼야 한다 (굳으면 다음 조우를 놓친다).
    assert "ot_active_ = ot_now;" in src, "ot_active_ 가 매 사이클 갱신되지 않는다"


def test_dospline_ego_anchor_survives_passing_the_apex():
    """DoSpline 의 ego 기준점이 apex 를 지나는 순간 한 바퀴 뒤로 감기면 안 된다.

    Mod 는 [0, len) 이므로 `s_apex - Mod(s_apex - cur_s_, L)` 은 **항상
    <= s_apex** 다. 자차가 apex 를 조금이라도 지나면 결과가 track_len 만큼
    음수로 감기고, 그러면
      · "이미 장애물 구간 내부" 거부  (>= s_front && <= s_back)
      · ego 재앵커                    (> knot_s.front())
    두 가드가 **둘 다 조용히 건너뛰어져** 경로가 자차 뒤(apex-4 m)에서
    시작하고, entry_avail_pre 가 한 바퀴만큼 부풀어 속도 캡도 안 걸린다.
    run_0822_000933 t=30.06 실측: 자차 s=5.71 인데 발행 경로가 s=1.65 에서
    시작, 그 지점 계획 d=+0.11 vs 자차 d=-0.40 (오차 0.51 m).
    """
    src = _spliner_source()
    # 주석은 옛 식을 근거로 인용하므로 코드 줄만 본다.
    code = "\n".join(ln for ln in src.splitlines()
                     if not ln.strip().startswith("//"))
    assert "s_apex - Mod(s_apex - cur_s_, track_len_)" not in code, (
        "DoSpline 의 ego 기준점이 다시 한 바퀴 감기는 식으로 돌아갔다"
    )
    # 부호 있는 최단 차이 형태여야 한다 — 두 곳(cur_unwrapped_pre / cur_unwrapped).
    n = len(re.findall(
        r"s_apex \+\s*\(Mod\(cur_s_ - s_apex \+ track_len_ / 2\.0, track_len_\)\s*-\s*"
        r"track_len_ / 2\.0\)\s*;", src, re.S))
    assert n == 2, f"부호 있는 최단 차이 형태가 2곳이 아니다: {n}곳"


def test_commit_breaks_when_ego_leaves_the_committed_path():
    """커밋 해제가 자차 횡이탈을 본다.

    해제 사유가 경로/장애물만 보면, 못 타는 경로를 경로 끝까지 재생한다.
    run_0822_000933 t=30.06~31.56: 경로오차 0.51 -> 1.01 m 로 벌어지는 1.5 초
    동안 committed_hold 유지, 속도 1.9 -> 0.00, 결국 장애물 소멸로만 풀림.
    """
    src = _spliner_source()
    assert "!lat_deviated &&" in src, "커밋 유지 조건에 횡이탈이 들어가 있지 않다"
    assert re.search(r"commit_lat_dev_run_ >= commit_break_lat_dev_cycles_", src), (
        "횡이탈이 연속 사이클 요구 없이 즉시 파기한다 — 단발 노이즈로 커밋이 깨진다"
    )
    assert 'NoteReject("commit_released_lat_deviation")' in src, "해제 사유가 계측되지 않는다"
    # 경로가 자차 s 를 안 덮으면 판정하지 않아야 한다 (그건 path_end 몫).
    assert "best <= 0.5" in src, "커밋 꼬리를 지난 경우를 이탈로 오판한다"


def test_commit_lat_dev_params_separate_normal_from_failure():
    """이탈 문턱이 정상 회피 오차 위, 사고 구간 중앙 아래에 있어야 한다."""
    spliner = _ros_params(_yaml(PARAMS), "spliner_node")
    thr = spliner["spliner/commit_break_lat_dev_m"]
    cyc = spliner["spliner/commit_break_lat_dev_cycles"]
    # run_0822_000933 실측: 정상 회피 p99 0.188 / max 0.532,
    #                        사고 구간 p50 0.331 / max 1.006.
    assert 0.20 <= thr <= 0.55, (
        f"문턱 {thr} 이 정상 회피 p99(0.188)와 사고 구간 p50(0.331) 사이를 "
        "벗어났다 — 오발화하거나 사고를 못 잡는다"
    )
    # 0.60 이상은 세 사건 중 하나(t=56)를 아예 못 잡는다.
    assert thr < 0.60, "문턱이 0.60 이상이면 사건 일부를 놓친다"
    assert 2 <= cyc <= 10, "연속 요구가 단발 노이즈를 못 거르거나 너무 늦다"


def test_ever_moved_stop_authority_is_paired_with_measured_speed():
    """정지 권한을 ever_moved 에 열어 주면 목표 속도도 관측값을 써야 한다.

    ClassifyObstacles 가 정적 트랙의 obs.vs 를 0 으로 **강제**하므로, allow_stop
    만 열고 obs_vs_eff 를 그대로 두면 코너에서 잠깐 느려져 STATIC 으로 뒤집힌
    상대를 "정지"로 읽고 목표 0 을 명령한다 = 급감속. 둘은 한 묶음이다.
    """
    src = _spliner_source()
    # (1) 정지 권한은 분류기(ever_dynamic)가 아니라 관측 변위(ever_moved)로 연다.
    assert re.search(
        r"const bool allow_stop =\s*"
        r"!closest\.is_static \|\| WasEverMoved\(closest\.id\);", src, re.S), (
        "allow_stop 이 WasEverMoved 를 쓰지 않는다 — 멈춘 상대 뒤에서 못 선다"
    )
    # 정지 권한에 분류기 판단(WasEverDynamic)을 쓰면 짧게 흔들린 콘까지
    # 정지 권한을 얻는다 (run_0820_235246: 39 트랙 중 3개가 그런 콘).
    # ⚠️ WasEverDynamic 자체는 그립 예산 키(1593줄)로 정당하게 쓰인다 —
    #    금지하는 것은 **allow_stop 문장에서** 쓰는 것뿐이다.
    assert "allow_stop =\n                    !closest.is_static || WasEverDynamic" \
        not in src, (
        "정지 권한이 관측 변위가 아니라 분류기 판단으로 열린다"
    )
    # (2) 그 짝: ever_moved 트랙은 obs.vs(=0 강제) 대신 관측 창 평균을 쓴다.
    assert re.search(
        r"const double obs_vs_eff =\s*"
        r"\(trailing_vs_filter_ && !emergency_stop &&\s*"
        r"\(!obs\.is_static \|\| obs_ever_moved\)\)\s*"
        r"\? ObsVsMean\(obs\.id, obs\.vs\) : obs\.vs;", src, re.S), (
        "ever_moved 트랙이 강제된 obs.vs=0 을 목표로 쓴다 — 잠깐 느려진 상대에 "
        "급감속한다 (allow_stop 변경과 반드시 같이 가야 한다)"
    )
    # (3) 복귀 기하가 없는 최악 분기도 같은 규약을 따른다.
    #     2026-08-22: 바닥이 0 -> trailing_creep_speed_ 로 바뀌었다. 완전 정지
    #     프로파일을 두지 않는다는 운영 요구(접촉해도 스택 유지, 장애물을
    #     치우면 자력 재개)라 0 리터럴은 이제 **금지**다.
    assert re.search(
        r"allow_stop\s*\?\s*\(obs\.is_static && obs_ever_moved\s*"
        r"\?\s*std::max\(obs_vs_eff, trailing_creep_speed_\)\s*"
        r":\s*trailing_creep_speed_\)", src, re.S), (
        "복귀 기하 없음 분기가 ever_moved 트랙에 관측 창 평균/크리프 대신 "
        "다른 값을 명령한다"
    )
    # (4) 문턱은 콘(실측 0.13~0.21 m)과 상대(p50 8.36 m) 사이에 있어야 한다.
    params = _ros_params(_yaml(PARAMS), "spliner_node")
    thr = params["spliner/ever_moved_min_disp_m"]
    assert 0.25 <= thr <= 2.0, (
        f"ever_moved_min_disp_m={thr} 가 콘 오분류(0.21 m)와 실제 상대 이동 "
        f"사이를 못 가른다"
    )



def test_no_complete_stop_profile():
    """완전 정지 프로파일 금지 (2026-08-22).

    운영 요구: 접촉하더라도 스택이 죽지 않고, 장애물만 치우면 스스로 다시
    진행해야 한다. 목표 속도 0 은 두 함정을 동시에 연다 —
      (a) 래치가 evade_committed 를 닫아 컨트롤러 발진 하한까지 닫히는
          v=0 자기유지 고정점 (사람이 개입해야 풀린다),
      (b) 전 점 vx=0 이 MAP 컨트롤러의 조향을 죽인다 (0807 코스아웃).
    접촉 속도를 낮추는 일은 바닥이 아니라 제동 능력이 한다.
    """
    src = _spliner_source()

    # (1) 트레일링 바닥은 allow_stop 여부와 무관하게 항상 존재한다.
    assert re.search(
        r"v_target_prof_actual = std::max\(\s*"
        r"v_target_prof_actual,\s*"
        r"allow_stop \? trailing_creep_speed_ : trailing_retry_speed_\);",
        src, re.S), (
        "정지 권한이 열렸을 때 바닥이 사라진다 — 완전 정지 프로파일이 부활했다"
    )

    # (2) 접촉이 확정된 분기는 추종용이 아니라 실제 제동 능력을 쓴다.
    assert "trailing_brake_decel_" in src, (
        "램프 충돌 제동이 추종용 감속도(trailing_decel)를 그대로 쓴다 — "
        "제동 곡선이 아니라 '가속 금지' 캡으로만 동작한다"
    )

    # (3) 바닥이 접촉을 보장할 때만 크리프로 내린다 (통상 트레일링은 유지).
    assert re.search(r"!allow_stop && floor_stop_dist\s*>\s*stop_dist", src), (
        "접촉 확정 크리프 전환이 물리식(바닥 정지거리 > 남은거리)이 아니거나 "
        "정지 권한 없는 정적 전용이 아니다 — 움직이는 상대 뒤에서 급감속한다"
    )

    # (4) 파라미터 불변식.
    params = _ros_params(_yaml(PARAMS), "spliner_node")
    creep = params["spliner/trailing_creep_speed"]
    retry = params["spliner/trailing_retry_speed"]
    decel = params["spliner/trailing_decel"]
    brake = params["spliner/trailing_brake_decel"]
    assert 0.0 < creep <= retry, (
        f"trailing_creep_speed={creep} 가 0 이거나 retry({retry})를 넘는다"
    )
    # 크리프는 접촉해도 스택이 사는 속도여야 한다 — 너무 높으면 그냥 들이받는다.
    assert creep <= 1.0, f"trailing_creep_speed={creep} 는 접촉 속도로 너무 높다"
    # 제동 능력은 실측(run_0822_172617: 0.3s 창 최대 4.19 m/s²) 아래.
    assert decel <= brake <= 4.2, (
        f"trailing_brake_decel={brake} 가 추종용({decel})보다 낮거나 "
        f"실측 제동 능력(4.19)을 넘는다"
    )

def test_corridor_block_verdict_reaches_stop_authority():
    """"합치면 트랙이 막힌다" 판정이 정지 권한까지 도달해야 한다.

    run_0822_000310 t=112.3~113.9: 그 판정이 99회(= 매 사이클) 떴는데도
    잔해는 새 id·is_static·이동이력 0 이라 allow_stop 이 열리지 않았고,
    자차는 degraded 1.5 m/s 바닥으로 계속 굴러가 벽까지 갔다. 판정을
    NoteOutcome 으로만 세고 버리면 그 사고가 그대로 재현된다.
    """
    src = _spliner_source()
    # (1) 판정 지점에서 래치 근거를 세운다.
    verdict = re.search(
        r'NoteOutcome\("cluster_merge_blocks_track"\);(.*?)'
        r"corridor_block_evidence_ = true;", src, re.S)
    assert verdict, "봉쇄 판정이 래치 근거로 이어지지 않는다 — 판정이 다시 버려진다"
    # 근거는 **회랑**의 성질이지 트랙 하나의 성질이 아니다. WasEverMoved 를
    # 걸면 정확히 반대로 동작한다 — run_0822_000310 재생 실측: 사고 버스트는
    # 93쌍 중 70쌍이 "움직인 구성원"을 가졌고(강체 잔해의 클러스터 재배분),
    # 오발화 버스트(헤어핀 콘)는 46/46 이 "안 움직임"이었다.
    verdict_code = re.sub(r"//[^\n]*", "", verdict.group(1))
    assert "WasEverMoved" not in verdict_code, (
        "봉쇄 근거에 WasEverMoved 게이트가 걸렸다 — 잔해를 거르고 콘을 통과시킨다"
    )
    # (2) 그 래치가 BuildTrailing 의 정지 권한 인자로 실제로 들어가야 한다.
    #     allow_stop 문장 자체는 건드리지 않는다(위 테스트가 고정한다).
    assert re.search(
        r"const bool stop_authority =\s*"
        r"allow_stop \|\| corridor_block_latched_;.*?"
        r"BuildTrailing\(closest, evasion_wpnts, mrks,\s*"
        r"stop_authority, &all_ahead\)", src, re.S), (
        "래치가 BuildTrailing 의 allow_stop 인자에 연결되지 않았다"
    )
    # (3) 속도 게이트는 근거 구간 **시작에서 한 번만** 본다. 매 사이클 보면
    #     래치로 차가 느려지는 순간 게이트가 스스로 래치를 풀어 바닥이
    #     되살아난다 (자기모순 -> 결국 들이받는다).
    assert re.search(
        r"if \(!corridor_block_run_valid_\) \{.*?"
        r"corridor_block_run_armed_ =\s*"
        r"!\(corridor_block_stop_min_speed_ > 0\.0\) \|\|\s*"
        r"std::abs\(cur_vs_\) >= corridor_block_stop_min_speed_;", src, re.S), (
        "속도 게이트가 근거 구간 시작이 아니라 매 사이클 평가된다"
    )
    # (4) 해제 조건에 속도가 끼면 (3)과 같은 자기모순이 된다.
    release = re.search(
        r"if \(corridor_block_latched_\) \{(.*?)\n        \}", src, re.S)
    assert release, "래치 해제 분기를 찾지 못했다"
    assert "cur_vs_" not in release.group(1), (
        "래치 해제가 속도를 본다 — 래치가 스스로를 푼다"
    )
    # (5) 문턱들.
    params = _ros_params(_yaml(PARAMS), "spliner_node")
    hold = params["spliner/corridor_block_stop_hold_sec"]
    rel = params["spliner/corridor_block_stop_release_sec"]
    vmin = params["spliner/corridor_block_stop_min_speed"]
    # 1프레임(30Hz=0.033s) 유령은 거르고, 물리 예산(봉쇄 확정 -> 정지
    # 데드라인 0.14~0.4s) 안에서 끝나야 한다.
    assert 0.05 <= hold <= 0.30, f"hold={hold} 가 유령/물리 예산 사이를 못 잡는다"
    assert rel >= hold, "해제가 hold 보다 짧으면 래치가 채터링한다"
    # 트레일링으로 이미 기어가는 중(<=2.5 m/s, 오발화 버스트 실측)이면
    # 바닥을 풀어 줄 이유가 없다. 레이스라인 최저속(4.6 m/s) 위로 올리면
    # 사고 버스트(3.4 m/s)를 놓친다.
    # trailing_retry_speed 아래에서는 바닥이 구속이 아니라 래치가 바꿀 것이
    # 없다(노드도 거부한다). 위로는 사고 버스트 시작 3.29 m/s 를 넘기면 안 된다.
    assert params["spliner/trailing_retry_speed"] <= vmin <= 3.2, (
        f"corridor_block_stop_min_speed={vmin} 가 오발화 버스트(v_max 2.02)와 "
        f"사고 버스트(v_start 3.29) 사이를 못 가른다"
    )
    assert params["spliner/corridor_block_stop_enabled"] is True, (
        "회랑 봉쇄 정지 권한이 꺼져 있다 — run_0822_000310 사고가 재현된다"
    )
