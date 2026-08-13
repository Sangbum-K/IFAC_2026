#!/usr/bin/env python3
"""Pure-numpy closed-loop safety model shared by the raceline tuner.

This module deliberately has no ROS, scipy, skimage, matplotlib, or PIL
dependency.  The tuner runs with the system ROS Python, while build_map.py is
normally run in a map-generation venv with heavier dependencies.

Deployment profile values are NOT duplicated here.  They used to be, and on
2026-08-05 all three copies disagreed: build_map.py had planner scale 0.97 /
cap 6.0, this file had 0.83 / 5.0, and the runtime YAML that the car actually
loads had 0.742 / 4.0.  ``assert_generator_profile_contract`` was supposed to
catch exactly that and was itself failing, which locked tuner saves.

Profiles now come from ``scripts/deployment_profiles.py``, which reads the same
YAML files that launch reads (numpy + pyyaml only, so both runtimes can import
it).  The AST contract below no longer compares constants -- it only pins the
generator's *algorithm* revision, which is still duplicated in this file.
"""

import ast
import hashlib
import json
import math
import sys
from pathlib import Path

import numpy as np

# scripts/ 를 import 경로에 넣는다. 이 모듈은 튜너 노드가 파일 경로로 직접
# 로드하므로 패키지 import 가 아니다.
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
# gate_controller_profiles / advisory_controller_profiles 는 여기서 안 쓰지만
# 재수출해야 한다 — raceline_tuner_node 가 이 모듈을 파일 경로로 로드해
# module.<name> 으로 부른다.
# (selected_controller_profiles 는 재수출에서 뺐다: 튜너가 부른 적이 없고
#  실제 소비자인 build_map.py / 테스트는 deployment_profiles 를 직접 import 한다.)
from deployment_profiles import (          # noqa: E402,F401
    BRAKE_DECEL_PER_AMP, DEFAULT_GATE_TARGET, SIM_LOG_FIELDS,
    advisory_controller_profiles, controller_profile,
    gate_controller_profiles, make_longitudinal_plant, profile_brake_max,
)
import deployment_profiles as _profiles     # noqa: E402

G = 9.81
MODEL_VERSION = 'tuner-controller-safety-2026-08-13-v5'
# Restricted-AST digest of the generator functions that must remain numerically
# paired with this lightweight copy.  Any generator algorithm edit locks tuner
# saves until this model and its parity test are deliberately updated.
# 2026-08-13 v5: the C++ MAP/L1 profile now mirrors Unicorn's adaptive L1:
# mean absolute curvature at local waypoint offsets [10, 20) subtracts a
# curvature_factor*kappa*v^2 term from the speed-based lookahead.  The generator
# and this lightweight copy were updated together and parity-tested.
GENERATOR_MODEL_AST_SHA256 = (
    '23c28a7714bda7251413bb017b48c44c5a868d4ce6ee4cf22c0dfcad3d85fc5a')
VEHICLE_HALF_WIDTH_M = 0.15
VEHICLE_HALF_LENGTH_M = 0.275


def profile_signature(target=DEFAULT_GATE_TARGET):
    """Digest of the *resolved* profile values actually used for validation.

    Because the values are read from the deployed YAML, this signature changes
    the moment someone retunes velocity_scale or the control speed cap -- which
    is the point: a tuner result recorded under one set of deployed limits must
    not be mistaken for one recorded under another.
    """
    encoded = json.dumps(
        _profiles.profile_manifest(target),
        sort_keys=True, separators=(',', ':')).encode('utf-8')
    return hashlib.sha256(encoded).hexdigest()


PROFILE_SIGNATURE = profile_signature()


def _restricted_ast_value(node):
    """Evaluate a numeric literal (optionally signed) from build_map.py.

    Originally this walked names/tuples/lists/dicts/string-addition so the
    generator's ``CONTROLLER_PROFILE_LIBRARY`` dict could be reconstructed and
    compared numerically.  2bb7e55 moved those profiles into
    ``deployment_profiles`` and deleted the comparison, leaving one caller that
    reads a single scalar (``MU``) -- so the extra branches were unreachable.
    """
    if isinstance(node, ast.Constant):
        return node.value
    if isinstance(node, ast.UnaryOp) and isinstance(
            node.op, (ast.UAdd, ast.USub)):
        value = _restricted_ast_value(node.operand)
        if not isinstance(value, (int, float)) or isinstance(value, bool):
            raise ValueError('unsupported unary operand')
        return value if isinstance(node.op, ast.UAdd) else -value
    raise ValueError(f'unsupported AST expression: {type(node).__name__}')


def assert_generator_profile_contract(build_map_path):
    """Fail if build_map.py's safety algorithm revision differs from this copy.

    Importing build_map.py is intentionally avoided: its map-generation
    dependencies are not ROS runtime dependencies.

    Profile *constants* are no longer compared -- both sides now import
    ``deployment_profiles`` and read the deployed YAML, so there is nothing left
    to drift.  What is still duplicated is the closed-loop algorithm below, and
    that is what this digest pins.  Also assert that the generator really does
    delegate its profiles, so a future edit cannot quietly reintroduce a copy.
    """
    path = Path(build_map_path)
    if not path.is_file():
        raise FileNotFoundError(f'generator profile source missing: {path}')
    source = path.read_text(encoding='utf-8')
    tree = ast.parse(source, filename=str(path))
    delegated = any(
        isinstance(statement, ast.ImportFrom) and
        statement.module == 'deployment_profiles'
        for statement in tree.body)
    if not delegated:
        raise ValueError(
            'generator no longer imports deployment_profiles: deployment '
            'constants would be duplicated again')
    for forbidden in ('CONTROLLER_PROFILE_LIBRARY', 'SIM_CONTROLLER_PROFILES'):
        for statement in tree.body:
            targets = (
                statement.targets if isinstance(statement, ast.Assign)
                else [statement.target]
                if isinstance(statement, ast.AnnAssign) else [])
            if any(isinstance(target, ast.Name) and target.id == forbidden
                   for target in targets):
                raise ValueError(
                    f'generator redefines {forbidden}: profile constants must '
                    'come from deployment_profiles only')
    implementation_names = {
        '_max_segment_endpoint_speed', 'velocity_profile',
        '_lookup_steer_angle', '_lookup_lateral_accel',
        'PurePursuitSim', 'verify_simulation',
    }
    implementation = [
        ast.dump(statement, include_attributes=False)
        for statement in tree.body
        if getattr(statement, 'name', None) in implementation_names
    ]
    found_names = {
        getattr(statement, 'name', None)
        for statement in tree.body
        if getattr(statement, 'name', None) in implementation_names
    }
    if found_names != implementation_names:
        raise ValueError(
            f'generator safety-model functions missing: '
            f'{sorted(implementation_names - found_names)}')
    implementation_signature = hashlib.sha256(
        '\n'.join(implementation).encode('utf-8')).hexdigest()
    if implementation_signature != GENERATOR_MODEL_AST_SHA256:
        raise ValueError(
            'generator/tuner safety algorithm revision differs: '
            f'{implementation_signature} != {GENERATOR_MODEL_AST_SHA256}')
    return PROFILE_SIGNATURE


def generator_physical_mu(build_map_path):
    """build_map.py 의 물리 마찰계수 MU (폐루프 검증의 타이어 한계).

    생성기는 속도 프로파일을 MU_PROFILE(낮은 계획 mu)로 풀고, 폐루프 추종
    검증은 MU(물리 한계)로 한다. 튜너가 계획 mu 하나로 둘 다 하면 생성기가
    승인한 CSV 를 자기가 다시 거부해 저장이 영구히 잠긴다.
    """
    path = Path(build_map_path)
    if not path.is_file():
        raise FileNotFoundError(f'generator profile source missing: {path}')
    tree = ast.parse(path.read_text(encoding='utf-8'), filename=str(path))
    value = None
    for statement in tree.body:
        targets = (
            statement.targets if isinstance(statement, ast.Assign)
            else [statement.target] if isinstance(statement, ast.AnnAssign)
            else [])
        if any(isinstance(target, ast.Name) and target.id == 'MU'
               for target in targets):
            value = _restricted_ast_value(statement.value)
    if not isinstance(value, (int, float)) or isinstance(value, bool) or \
            not math.isfinite(value) or not 0.0 < float(value) <= 1.5:
        raise ValueError(f'generator physical MU is missing or invalid: {path}')
    return float(value)


def _max_segment_endpoint_speed(
        base_speed, base_kappa, candidate_kappa, segment_ds,
        longitudinal_limit, tire_limit, current_high):
    """Largest candidate endpoint speed satisfying segment-worst tire demand."""
    base_sq = base_speed ** 2

    def feasible(candidate):
        candidate_sq = candidate ** 2
        longitudinal = (candidate_sq - base_sq) / (2.0 * segment_ds)
        lateral = max(
            base_sq * base_kappa,
            candidate_sq * candidate_kappa)
        return (
            longitudinal <= longitudinal_limit + 1e-12 and
            np.hypot(longitudinal, lateral) <= tire_limit + 1e-12)

    if feasible(current_high):
        return current_high
    lo, hi = base_speed, current_high
    for _ in range(60):
        mid = 0.5 * (lo + hi)
        if feasible(mid):
            lo = mid
        else:
            hi = mid
    return lo


def velocity_profile(s, kappa, v_max, mu, a_acc_max, a_dec_max,
                     segment_ds, speed_ceiling=None, max_iterations=500):
    """Closed-loop fixed-point projection with optional manual speed ceilings.

    ``a_acc_max`` / ``a_dec_max`` accept a scalar or a per-waypoint array; the
    generator uses arrays to narrow the longitudinal band inside corners
    (``A_ACC_CORNER``).  Each segment takes the *smaller* of its two endpoint
    limits, matching build_map.
    """
    if not all(np.isfinite(value) and value > 0.0 for value in (v_max, mu)):
        raise ValueError('velocity limits must be positive and finite')
    s = np.asarray(s, dtype=float)
    kappa = np.abs(np.asarray(kappa, dtype=float))
    segment_ds = np.asarray(segment_ds, dtype=float)
    count = len(s)
    if count < 3 or len(kappa) != count or len(segment_ds) != count or \
            not np.isfinite(s).all() or not np.isfinite(kappa).all() or \
            not np.isfinite(segment_ds).all() or np.any(segment_ds <= 1e-6):
        raise ValueError('closed-loop velocity input is malformed')
    accel_point = np.broadcast_to(
        np.asarray(a_acc_max, dtype=float), (count,)).astype(float)
    decel_point = np.broadcast_to(
        np.asarray(a_dec_max, dtype=float), (count,)).astype(float)
    if not np.isfinite(accel_point).all() or np.any(accel_point <= 0.0) or \
            not np.isfinite(decel_point).all() or np.any(decel_point <= 0.0):
        raise ValueError('velocity limits must be positive and finite')
    accel_segment = np.minimum(accel_point, np.roll(accel_point, -1))
    decel_segment = np.minimum(decel_point, np.roll(decel_point, -1))
    tire_limit = mu * G
    # Match build_map's 10 ppm reserve so 7-decimal CSV serialization cannot
    # turn an exactly-on-limit floating value into an over-limit value.
    projection_limit = tire_limit * (1.0 - 1e-5)
    lateral_cap = np.sqrt(
        projection_limit / np.maximum(kappa, 1e-12))
    v = np.minimum(lateral_cap, v_max)
    if speed_ceiling is not None:
        ceiling = np.asarray(speed_ceiling, dtype=float)
        if len(ceiling) != count or not np.isfinite(ceiling).all() or \
                np.any(ceiling < 0.0):
            raise ValueError('manual speed ceiling is malformed')
        v = np.minimum(v, ceiling)

    for _ in range(max_iterations):
        max_reduction = 0.0
        for index in range(count):
            following = (index + 1) % count
            if v[following] >= v[index]:
                safe = _max_segment_endpoint_speed(
                    v[index], kappa[index], kappa[following],
                    segment_ds[index], accel_segment[index], projection_limit,
                    v[following])
                max_reduction = max(max_reduction, v[following] - safe)
                v[following] = safe
            else:
                safe = _max_segment_endpoint_speed(
                    v[following], kappa[following], kappa[index],
                    segment_ds[index], decel_segment[index], projection_limit,
                    v[index])
                max_reduction = max(max_reduction, v[index] - safe)
                v[index] = safe
        if max_reduction < 1e-10:
            break
    else:
        raise RuntimeError('velocity profile projection did not converge')

    ax = closed_loop_ax(v, segment_ds)
    next_v = np.roll(v, -1)
    worst_lateral = np.maximum(
        v ** 2 * kappa,
        next_v ** 2 * np.roll(kappa, -1))
    combined = np.hypot(worst_lateral, ax)
    invalid = (
        (ax > accel_segment + 1e-8) |
        (ax < -decel_segment - 1e-8) |
        (combined > tire_limit + 1e-8))
    if np.any(invalid):
        bad = np.flatnonzero(invalid)
        raise RuntimeError(
            f'velocity projection failed at {bad[:8].tolist()}: '
            f'combined={combined[bad].max():.6f}, limit={tire_limit:.6f}')
    return v


def closed_loop_ax(speed, segment_ds):
    speed = np.asarray(speed, dtype=float)
    segment_ds = np.asarray(segment_ds, dtype=float)
    if len(speed) != len(segment_ds) or np.any(segment_ds <= 1e-6):
        raise ValueError('closed-loop acceleration input is malformed')
    return (
        np.roll(speed, -1) ** 2 - speed ** 2) / (2.0 * segment_ds)


def _load_steering_lut(workspace_root, relative_path):
    root = Path(workspace_root).resolve()
    path = (root / relative_path).resolve()
    try:
        path.relative_to(root)
    except ValueError as exc:
        raise ValueError(f'steering LUT escapes workspace: {path}') from exc
    if not path.is_file():
        raise FileNotFoundError(f'deployment steering LUT missing: {path}')
    table = np.loadtxt(path, delimiter=',')
    if table.ndim != 2 or table.shape[0] < 2 or table.shape[1] < 2:
        raise ValueError(f'invalid steering LUT shape: {path}')
    if not np.isfinite(table[0, 1:]).all() or \
            not np.isfinite(table[1:, 0]).all() or \
            np.any(np.sum(np.isfinite(table[1:, 1:]), axis=0) == 0):
        raise ValueError(f'invalid steering LUT values: {path}')
    return table


def _lookup_steer_angle(table, acceleration, velocity):
    sign = 1.0 if acceleration > 0.0 else -1.0
    acceleration = abs(float(acceleration))
    velocity_column = int(
        np.argmin(np.abs(table[0, 1:] - velocity))) + 1
    finite_rows = np.flatnonzero(
        np.isfinite(table[1:, velocity_column]) &
        np.isfinite(table[1:, 0])) + 1
    if len(finite_rows) == 0:
        raise ValueError('steering LUT selected column has no finite row')
    closest = int(np.argmin(
        np.abs(table[finite_rows, velocity_column] - acceleration)))
    if closest == 0:
        return 0.0
    if closest + 1 >= len(finite_rows):
        second = closest
    else:
        before, after = closest - 1, closest + 1
        before_error = abs(
            table[finite_rows[before], velocity_column] - acceleration)
        after_error = abs(
            table[finite_rows[after], velocity_column] - acceleration)
        second = before if before_error <= after_error else after
    first_row = finite_rows[closest]
    second_row = finite_rows[second]
    steering = float(table[first_row, 0])
    if first_row != second_row:
        first_accel = table[first_row, velocity_column]
        second_accel = table[second_row, velocity_column]
        if abs(second_accel - first_accel) > 1e-9:
            ratio = np.clip(
                (acceleration - first_accel) /
                (second_accel - first_accel), 0.0, 1.0)
            steering += ratio * (
                table[second_row, 0] - table[first_row, 0])
    return float(steering * sign)


def _lookup_lateral_accel(table, steering_angle, velocity):
    """Measured forward LUT response: (steer, velocity) -> lateral accel."""
    sign = 1.0 if steering_angle > 0.0 else -1.0
    steering = abs(float(steering_angle))
    velocity_abs = abs(float(velocity))
    velocity_column = int(
        np.argmin(np.abs(table[0, 1:] - velocity_abs))) + 1
    finite_rows = np.flatnonzero(
        np.isfinite(table[1:, velocity_column]) &
        np.isfinite(table[1:, 0])) + 1
    if len(finite_rows) == 0:
        raise ValueError('steering LUT selected column has no finite rows')
    steer_axis = table[finite_rows, 0]
    accel_axis = table[finite_rows, velocity_column]
    order = np.argsort(steer_axis)
    steer_axis = steer_axis[order]
    accel_axis = accel_axis[order]
    lateral = float(np.interp(
        steering, steer_axis, accel_axis,
        left=accel_axis[0], right=accel_axis[-1]))
    minimum_table_speed = float(np.min(table[0, 1:]))
    if velocity_abs < minimum_table_speed and minimum_table_speed > 0.0:
        lateral *= (velocity_abs / minimum_table_speed) ** 2
    return lateral * sign


class PurePursuitSim:
    """Selected deployed controller contract + friction-circle bicycle model."""

    def __init__(self, traj, profile_name, workspace_root, mu):
        self.profile = controller_profile(profile_name)
        self.traj = traj
        self.xy = np.asarray(traj['xy'], dtype=float)
        raw_vx = np.asarray(traj['vx'], dtype=float)
        self.vx_guidance_ref = (
            raw_vx * self.profile['planner_speed_scale'])
        self.vx_ref = np.minimum(
            self.vx_guidance_ref *
            self.profile['controller_speed_scale'],
            self.profile['max_speed_mps'])
        self.s_ref = np.asarray(traj['s'], dtype=float)
        self.count = len(self.xy)
        if self.count < 8 or len(self.s_ref) != self.count or \
                len(raw_vx) != self.count or not np.isfinite(self.xy).all():
            raise ValueError('simulation trajectory is malformed')
        self.track_len = float(
            self.s_ref[-1] + np.hypot(*(self.xy[0] - self.xy[-1])))
        self.wheelbase = self.profile['wheelbase_m']
        self.max_steer = self.profile['max_steer_rad']
        self.max_steer_rate = self.profile['max_steer_rate_radps']
        self.lookahead_gain = self.profile['lookahead_gain']
        self.lookahead_offset = self.profile['lookahead_offset']
        self.curvature_factor = self.profile.get('curvature_factor', 0.0)
        self.lookahead_clip = (
            self.profile['lookahead_min_m'],
            self.profile['lookahead_max_m'])
        self.kappa_ref = np.asarray(traj['kappa'], dtype=float)
        # Controller speed-command slew limits (runtime YAML), which are not
        # the same thing as what the car can physically do -- the plant below
        # owns that.  See deployment_profiles.VescLongitudinalPlant.
        self.a_acc_max = self.profile['max_accel_mps2']
        self.a_dec_max = self.profile['max_decel_mps2']
        self.plant = make_longitudinal_plant(self.profile)
        self.local_count = min(
            self.profile['num_local_waypoints'], self.count)
        self.dt = 1.0 / self.profile['control_rate_hz']
        if not np.isfinite(mu) or mu <= 0.0:
            raise ValueError('simulation mu must be positive and finite')
        self.tire_limit = float(mu * G)
        self.steering_lut = None
        if self.profile['kind'] == 'cpp_map_l1':
            self.steering_lut = _load_steering_lut(
                workspace_root, self.profile['steering_lut'])

    def _nearest_index(self, point):
        delta = self.xy - point
        return int(np.argmin(np.einsum('ij,ij->i', delta, delta)))

    def local_window(self, index):
        return (index + np.arange(self.local_count)) % self.count

    def lookahead_point(self, position, index, lookahead):
        window = self.local_window(index)
        if self.profile['kind'] == 'cpp_map_l1':
            segments = np.hypot(*np.diff(self.xy[window], axis=0).T)
            reached = np.flatnonzero(np.cumsum(segments) >= lookahead)
            target_index = (
                window[reached[0] + 1] if len(reached) else window[-1])
            return self.xy[target_index], int(target_index)
        distance = np.hypot(*(self.xy[window] - position).T)
        ahead = np.flatnonzero(distance >= lookahead)
        target_index = window[ahead[0]] if len(ahead) else window[-1]
        return self.xy[target_index], int(target_index)

    def _steering_target(
            self, alpha, target_distance, lookahead, guidance_speed):
        if self.profile['kind'] == 'cpp_map_l1':
            lateral_accel = (
                2.0 * guidance_speed ** 2 / max(lookahead, 1e-3) *
                np.sin(alpha))
            return _lookup_steer_angle(
                self.steering_lut, lateral_accel, guidance_speed)
        return np.arctan2(
            2.0 * self.wheelbase * np.sin(alpha), target_distance)

    def _speed_target(self, position, theta, speed, target_index):
        if self.profile['kind'] == 'cpp_map_l1':
            query = position + \
                speed * self.profile['speed_lookahead_s'] * np.array([
                    np.cos(theta), np.sin(theta)])
            speed_index = self._nearest_index(query)
        else:
            speed_index = target_index
        return float(self.vx_ref[speed_index])

    def run(self, requested_laps=1, max_time_sec=300.0):
        position = self.xy[0].copy()
        theta = float(self.traj['psi'][0])
        speed = 0.0
        previous_s = 0.0
        fields = SIM_LOG_FIELDS
        log = {key: [] for key in fields}
        elapsed = 0.0
        laps = 0
        steering = 0.0
        # control_node.cpp's last_speed_command_: the slew limiter runs on the
        # previous *command*, not on measured speed.
        speed_command = 0.0
        self.plant.reset()
        while elapsed < max_time_sec:
            index = self._nearest_index(position)
            current_s = self.s_ref[index]
            if previous_s > 0.8 * self.track_len and \
                    current_s < 0.2 * self.track_len:
                laps += 1
                if laps >= requested_laps:
                    break
            previous_s = current_s
            tangent = np.array([
                np.cos(self.traj['psi'][index]),
                np.sin(self.traj['psi'][index])])
            error = position - self.xy[index]
            d_error = tangent[0] * error[1] - tangent[1] * error[0]
            lookahead_min = self.lookahead_clip[0]
            if self.profile['kind'] == 'cpp_map_l1':
                lookahead_min = min(max(
                    lookahead_min, np.sqrt(2.0) * abs(d_error)),
                    self.lookahead_clip[1])
                local_window = self.local_window(index)
                preview = local_window[10:min(20, len(local_window))]
                preview_curvature = (
                    float(np.mean(np.abs(self.kappa_ref[preview])))
                    if len(preview) else 0.0)
                raw_lookahead = (
                    self.lookahead_gain * abs(speed) + self.lookahead_offset -
                    self.curvature_factor * preview_curvature * speed * speed)
            else:
                raw_lookahead = (
                    self.lookahead_gain * abs(speed) + self.lookahead_offset)
            lookahead = np.clip(
                raw_lookahead, lookahead_min, self.lookahead_clip[1])
            target, target_index = self.lookahead_point(
                position, index, lookahead)
            dx, dy = target - position
            alpha = np.arctan2(dy, dx) - theta
            alpha = np.arctan2(np.sin(alpha), np.cos(alpha))
            target_distance = max(np.hypot(dx, dy), 1e-3)
            steering_target = np.clip(
                self._steering_target(
                    alpha, target_distance, lookahead,
                    self.vx_guidance_ref[index]),
                -self.max_steer, self.max_steer)
            steering += np.clip(
                steering_target - steering,
                -self.max_steer_rate * self.dt,
                self.max_steer_rate * self.dt)

            # Stage 1: controller speed command + slew (control_node.cpp).
            speed_target = self._speed_target(
                position, theta, speed, target_index)
            previous_command = speed_command
            speed_command = previous_command + float(np.clip(
                speed_target - previous_command,
                -self.a_dec_max * self.dt, self.a_acc_max * self.dt))
            # What the car publishes as drive.acceleration; the VESC brake
            # entry test reads exactly this.
            a_long_command = float(np.clip(
                (speed_command - previous_command) / self.dt,
                -self.a_dec_max, self.a_acc_max))

            # Stage 2: longitudinal plant (VESC arbitration + measured limits).
            a_long_plant, plant_mode, brake_amps = self.plant.step(
                speed, speed_command, a_long_command, self.dt)

            if self.profile['kind'] == 'cpp_map_l1':
                a_lat_command = _lookup_lateral_accel(
                    self.steering_lut, steering, speed)
            else:
                a_lat_command = float(
                    speed ** 2 * np.tan(steering) / self.wheelbase)
            # Tire budget is what the car actually tries to do.  The controller
            # command slope does not load the tires if the plant cannot deliver.
            combined_demand = float(np.hypot(
                a_long_plant, a_lat_command))
            a_lat = float(np.clip(
                a_lat_command, -self.tire_limit, self.tire_limit))
            longitudinal_available = float(np.sqrt(max(
                self.tire_limit ** 2 - a_lat ** 2, 0.0)))
            a_long = float(np.clip(
                a_long_plant,
                -longitudinal_available, longitudinal_available))
            combined_accel = float(np.hypot(a_long, a_lat))
            saturated = combined_demand > self.tire_limit * (1.0 + 1e-9)

            next_speed = max(speed + a_long * self.dt, 0.0)
            yaw_rate = a_lat / speed if speed > 1e-6 else 0.0
            theta += yaw_rate * self.dt
            mean_speed = 0.5 * (speed + next_speed)
            position = position + mean_speed * self.dt * np.array([
                np.cos(theta), np.sin(theta)])
            speed = next_speed
            elapsed += self.dt

            values = (
                elapsed, position[0], position[1], theta, speed,
                speed_command, current_s + laps * self.track_len, d_error,
                steering, a_long_command, a_long_plant, a_lat_command,
                a_long, a_lat, combined_demand, combined_accel,
                float(saturated), brake_amps,
                1.0 if plant_mode == 'brake' else 0.0)
            if len(values) != len(fields):
                raise RuntimeError('simulation log column count mismatch')
            for key, value in zip(fields, values):
                log[key].append(value)
        output = {key: np.asarray(value) for key, value in log.items()}
        output.update({
            'laps': laps,
            'track_len': self.track_len,
            'profile_name': self.profile['name'],
            'tire_limit': self.tire_limit,
            'model_version': MODEL_VERSION,
            'profile_signature': PROFILE_SIGNATURE,
        })
        return output


def _footprint_free(grid, xy, psi):
    xy = np.atleast_2d(xy)
    psi = np.atleast_1d(psi)
    step = max(0.5 * float(grid.res), 0.01)
    n_lon = max(1, int(math.ceil(2.0 * VEHICLE_HALF_LENGTH_M / step)))
    n_lat = max(1, int(math.ceil(2.0 * VEHICLE_HALF_WIDTH_M / step)))
    longitudinal = np.linspace(
        -VEHICLE_HALF_LENGTH_M, VEHICLE_HALF_LENGTH_M, n_lon + 1)
    lateral = np.linspace(
        -VEHICLE_HALF_WIDTH_M, VEHICLE_HALF_WIDTH_M, n_lat + 1)
    cosine, sine = np.cos(psi), np.sin(psi)
    free = np.ones(len(xy), dtype=bool)
    for dl in longitudinal:
        for dd in lateral:
            points = xy + np.column_stack([
                dl * cosine - dd * sine,
                dl * sine + dd * cosine])
            free &= grid.is_free(points)
    return free


def verify_simulation(name, log, grid, requested_laps, profile_name, mu):
    """Fail-closed log, footprint, actuator, and tire-demand verification."""
    profile = controller_profile(profile_name)
    failures = []
    fields = SIM_LOG_FIELDS
    lengths = [len(log.get(field, ())) for field in fields]
    prefix = f'{name}/{profile_name}'
    if not lengths or min(lengths) == 0 or len(set(lengths)) != 1:
        return False, [f'{prefix}: [FAIL] simulation log is empty/misaligned']
    matrix = np.column_stack([log[field] for field in fields])
    if not np.isfinite(matrix).all():
        return False, [f'{prefix}: [FAIL] simulation log contains NaN/Inf']
    if log.get('profile_name') != profile_name or \
            log.get('model_version') != MODEL_VERSION or \
            log.get('profile_signature') != PROFILE_SIGNATURE:
        failures.append(f'{prefix}: [FAIL] simulation model metadata mismatch')
    tire_limit = float(mu * G)
    if not np.isfinite(log.get('tire_limit', np.nan)) or \
            abs(float(log['tire_limit']) - tire_limit) > 1e-9:
        failures.append(f'{prefix}: [FAIL] tire-limit metadata mismatch')
    if int(log.get('laps', 0)) < requested_laps:
        failures.append(
            f'{prefix}: [FAIL] completed {int(log.get("laps", 0))}/'
            f'{requested_laps} laps')
    dt = np.diff(log['t'])
    if len(dt) and (np.any(dt <= 0.0) or not np.isfinite(dt).all()):
        failures.append(f'{prefix}: [FAIL] log time is not monotonic')
    pose_xy = np.column_stack([log['x'], log['y']])
    bad_footprint = np.flatnonzero(
        ~_footprint_free(grid, pose_xy, log['theta']))
    if len(bad_footprint):
        first = int(bad_footprint[0])
        failures.append(
            f'{prefix}: [FAIL] tracking footprint intersects wall/unknown '
            f'({len(bad_footprint)} steps, first t={log["t"][first]:.2f}s)')
    if np.any(np.abs(log['steer']) > profile['max_steer_rad'] + 1e-6):
        failures.append(f'{prefix}: [FAIL] steering angle limit exceeded')
    if len(dt):
        steering_rate = np.abs(np.diff(log['steer']) / dt)
        if np.any(
                steering_rate >
                profile['max_steer_rate_radps'] * 1.001):
            failures.append(f'{prefix}: [FAIL] steering rate limit exceeded')
    if np.any(log['v'] < -1e-6) or \
            np.any(log['v'] > profile['max_speed_mps'] + 1e-6):
        failures.append(f'{prefix}: [FAIL] profile speed limit exceeded')
    reconstructed_demand = np.hypot(
        log['a_long_plant'], log['a_lat_cmd'])
    reconstructed_actual = np.hypot(log['a_long'], log['a_lat'])
    if not np.allclose(
            reconstructed_demand, log['combined_demand'],
            rtol=1e-9, atol=1e-9) or not np.allclose(
                reconstructed_actual, log['combined_accel'],
                rtol=1e-9, atol=1e-9):
        failures.append(f'{prefix}: [FAIL] acceleration log arithmetic mismatch')
    demand_bad = np.flatnonzero(
        log['combined_demand'] > tire_limit * (1.0 + 1e-9))
    saturated = log['tire_saturated'] > 0.5
    if not np.array_equal(
            saturated,
            log['combined_demand'] > tire_limit * (1.0 + 1e-9)):
        failures.append(f'{prefix}: [FAIL] tire saturation metadata mismatch')
    if len(demand_bad):
        failures.append(
            f'{prefix}: [FAIL] controller combined demand '
            f'{log["combined_demand"][demand_bad].max():.2f} > '
            f'mu*g {tire_limit:.2f} m/s²')
    if np.any(log['combined_accel'] > tire_limit * (1.0 + 1e-9)):
        failures.append(
            f'{prefix}: [FAIL] integrated combined acceleration exceeds tire limit')
    if np.any(log['a_long_cmd'] > profile['max_accel_mps2'] + 1e-9) or \
            np.any(log['a_long_cmd'] < -profile['max_decel_mps2'] - 1e-9):
        failures.append(f'{prefix}: [FAIL] longitudinal command cap exceeded')
    plant_decel_limit = (
        profile['plant_decel_max_mps2'] +
        BRAKE_DECEL_PER_AMP * profile_brake_max(profile))
    if np.any(log['a_long_plant'] > profile['plant_accel_max_mps2'] + 1e-9) or \
            np.any(log['a_long_plant'] < -plant_decel_limit - 1e-9):
        failures.append(
            f'{prefix}: [FAIL] plant acceleration exceeds measured limits '
            f'({log["a_long_plant"].min():.2f}~'
            f'{log["a_long_plant"].max():.2f} m/s², allowed '
            f'-{plant_decel_limit:.2f}~'
            f'+{profile["plant_accel_max_mps2"]:.2f})')
    return not failures, failures
