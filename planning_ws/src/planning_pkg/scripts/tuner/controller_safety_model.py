#!/usr/bin/env python3
"""Pure-numpy closed-loop safety model shared by the raceline tuner.

This module deliberately has no ROS, scipy, skimage, matplotlib, or PIL
dependency.  The tuner runs with the system ROS Python, while build_map.py is
normally run in a map-generation venv with heavier dependencies.

The profile manifest is duplicated intentionally to isolate those runtime
dependencies.  ``assert_generator_profile_contract`` parses build_map.py with a
restricted AST evaluator and fails if the two selected profile manifests drift.
"""

import ast
import copy
import hashlib
import json
import math
from pathlib import Path

import numpy as np

G = 9.81
MODEL_VERSION = 'tuner-controller-safety-2026-07-31-v1'
# Restricted-AST digest of the generator functions that must remain numerically
# paired with this lightweight copy.  Any generator algorithm edit locks tuner
# saves until this model and its parity test are deliberately updated.
GENERATOR_MODEL_AST_SHA256 = (
    '4214370c89f4badce24a2b9fade4219f87263cc0d3fb8c6549ab04f67689dbbe')
SELECTED_CONTROLLER_PROFILES = (
    'cpp_real_car',
    'python_sim_tracker',
)
VEHICLE_HALF_WIDTH_M = 0.15
VEHICLE_HALF_LENGTH_M = 0.275

# These values mirror the explicitly selected deployment profiles in
# build_map.py.  The AST contract check below makes any future drift fail closed.
CONTROLLER_PROFILE_LIBRARY = {
    'cpp_real_car': {
        'kind': 'cpp_map_l1',
        'planner_speed_scale': 0.60,
        'controller_speed_scale': 1.0,
        'max_speed_mps': 3.6,
        'lookahead_gain': 0.47,
        'lookahead_offset': -0.2,
        'lookahead_min_m': 0.5,
        'lookahead_max_m': 8.0,
        'wheelbase_m': 0.33,
        'max_steer_rad': 0.36,
        'max_steer_rate_radps': 2.0,
        'max_accel_mps2': 5.5,
        'max_decel_mps2': 7.0,
        'control_rate_hz': 40.0,
        'num_local_waypoints': 130,
        'steering_lut': (
            'control_ws/src/control_pkg/config/steering_lookup/'
            'NUC6_glc_pacejka_lookup_table.csv'),
    },
    'python_sim_tracker': {
        'kind': 'python_pure_pursuit',
        'planner_speed_scale': 0.6667,
        'controller_speed_scale': 0.30,
        'max_speed_mps': 6.0,
        'lookahead_gain': 0.5,
        'lookahead_offset': 0.0,
        'lookahead_min_m': 0.6,
        'lookahead_max_m': 1.2,
        'wheelbase_m': 0.33,
        'max_steer_rad': 0.36,
        'max_steer_rate_radps': 2.0,
        'max_accel_mps2': 5.5,
        'max_decel_mps2': 7.0,
        'control_rate_hz': 40.0,
        'num_local_waypoints': 130,
        'steering_lut': None,
    },
}

_PROFILE_NUMERIC_FIELDS = (
    'planner_speed_scale', 'controller_speed_scale', 'max_speed_mps',
    'lookahead_gain', 'lookahead_offset', 'lookahead_min_m',
    'lookahead_max_m', 'wheelbase_m', 'max_steer_rad',
    'max_steer_rate_radps', 'max_accel_mps2', 'max_decel_mps2',
    'control_rate_hz', 'num_local_waypoints',
)


def controller_profile(name):
    if not isinstance(name, str) or name not in CONTROLLER_PROFILE_LIBRARY:
        raise ValueError(f'unknown controller profile: {name!r}')
    profile = copy.deepcopy(CONTROLLER_PROFILE_LIBRARY[name])
    profile['name'] = name
    if profile.get('kind') not in ('cpp_map_l1', 'python_pure_pursuit'):
        raise ValueError(f'{name}: unsupported controller kind')
    for field in _PROFILE_NUMERIC_FIELDS:
        value = profile.get(field)
        if isinstance(value, bool) or not isinstance(value, (int, float)) or \
                not np.isfinite(value):
            raise ValueError(f'{name}: {field} is not a finite number')
    if not (0.0 < profile['planner_speed_scale'] <= 1.0) or \
            not (0.0 < profile['controller_speed_scale'] <= 1.0):
        raise ValueError(f'{name}: speed scales must be in (0, 1]')
    positive = (
        'max_speed_mps', 'lookahead_min_m', 'lookahead_max_m',
        'wheelbase_m', 'max_steer_rad', 'max_steer_rate_radps',
        'max_accel_mps2', 'max_decel_mps2', 'control_rate_hz',
    )
    if any(profile[field] <= 0.0 for field in positive):
        raise ValueError(f'{name}: vehicle/controller limits must be positive')
    if profile['lookahead_max_m'] < profile['lookahead_min_m']:
        raise ValueError(f'{name}: lookahead bounds are reversed')
    if profile['max_steer_rad'] >= 0.5 * np.pi:
        raise ValueError(f'{name}: max steer is outside bicycle-model range')
    count = profile['num_local_waypoints']
    if int(count) != count or count < 2:
        raise ValueError(f'{name}: num_local_waypoints must be an integer >= 2')
    profile['num_local_waypoints'] = int(count)
    lut = profile.get('steering_lut')
    if profile['kind'] == 'cpp_map_l1' and \
            (not isinstance(lut, str) or not lut):
        raise ValueError(f'{name}: steering LUT is missing')
    if profile['kind'] == 'python_pure_pursuit' and lut is not None:
        raise ValueError(f'{name}: Python tracker must not specify a LUT')
    return profile


def selected_controller_profiles():
    names = tuple(SELECTED_CONTROLLER_PROFILES)
    if not names or len(names) != len(set(names)):
        raise ValueError('selected controller profile list is empty or duplicated')
    return tuple(controller_profile(name) for name in names)


def profile_manifest():
    """Stable, test-visible manifest excluding derived ``name`` fields."""
    return {
        'model_version': MODEL_VERSION,
        'generator_model_ast_sha256': GENERATOR_MODEL_AST_SHA256,
        'selected_profiles': list(SELECTED_CONTROLLER_PROFILES),
        'profiles': {
            name: copy.deepcopy(CONTROLLER_PROFILE_LIBRARY[name])
            for name in SELECTED_CONTROLLER_PROFILES
        },
    }


def profile_signature():
    payload = {
        'selected_profiles': list(SELECTED_CONTROLLER_PROFILES),
        'profiles': {
            name: CONTROLLER_PROFILE_LIBRARY[name]
            for name in SELECTED_CONTROLLER_PROFILES
        },
    }
    encoded = json.dumps(
        payload, sort_keys=True, separators=(',', ':')).encode('utf-8')
    return hashlib.sha256(encoded).hexdigest()


PROFILE_SIGNATURE = profile_signature()


def _restricted_ast_value(node, values):
    """Evaluate only literals/names/containers/string-or-number addition."""
    if isinstance(node, ast.Constant):
        return node.value
    if isinstance(node, ast.Name) and node.id in values:
        return values[node.id]
    if isinstance(node, ast.Tuple):
        return tuple(_restricted_ast_value(item, values) for item in node.elts)
    if isinstance(node, ast.List):
        return [_restricted_ast_value(item, values) for item in node.elts]
    if isinstance(node, ast.Dict):
        return {
            _restricted_ast_value(key, values):
            _restricted_ast_value(value, values)
            for key, value in zip(node.keys, node.values)
        }
    if isinstance(node, ast.UnaryOp) and isinstance(
            node.op, (ast.UAdd, ast.USub)):
        value = _restricted_ast_value(node.operand, values)
        if not isinstance(value, (int, float)) or isinstance(value, bool):
            raise ValueError('unsupported unary operand')
        return value if isinstance(node.op, ast.UAdd) else -value
    if isinstance(node, ast.BinOp) and isinstance(node.op, ast.Add):
        return (
            _restricted_ast_value(node.left, values) +
            _restricted_ast_value(node.right, values))
    raise ValueError(f'unsupported AST expression: {type(node).__name__}')


def assert_generator_profile_contract(build_map_path):
    """Fail if build_map.py's selected deployment manifest differs.

    Importing build_map.py is intentionally avoided: its map-generation
    dependencies are not ROS runtime dependencies.
    """
    path = Path(build_map_path)
    if not path.is_file():
        raise FileNotFoundError(f'generator profile source missing: {path}')
    tree = ast.parse(path.read_text(encoding='utf-8'), filename=str(path))
    values = {}
    wanted = {
        'V_MAX', 'WHEELBASE', 'MAX_STEER', 'A_ACC_MAX', 'A_DEC_MAX',
        'MAX_STEER_RATE', 'N_LOCAL_WP', 'SIM_CONTROLLER_PROFILES',
        'CONTROLLER_PROFILE_LIBRARY',
    }
    for statement in tree.body:
        if not isinstance(statement, (ast.Assign, ast.AnnAssign)):
            continue
        targets = (
            statement.targets if isinstance(statement, ast.Assign)
            else [statement.target])
        if len(targets) != 1 or not isinstance(targets[0], ast.Name):
            continue
        name = targets[0].id
        if name not in wanted:
            continue
        try:
            values[name] = _restricted_ast_value(statement.value, values)
        except (KeyError, TypeError, ValueError) as exc:
            raise ValueError(
                f'cannot evaluate generator profile field {name}: {exc}') from exc
    if 'SIM_CONTROLLER_PROFILES' not in values or \
            'CONTROLLER_PROFILE_LIBRARY' not in values:
        raise ValueError('generator selected profile manifest is missing')
    generator_selected = tuple(values['SIM_CONTROLLER_PROFILES'])
    if generator_selected != tuple(SELECTED_CONTROLLER_PROFILES):
        raise ValueError(
            f'generator/tuner selected profiles differ: '
            f'{generator_selected} != {SELECTED_CONTROLLER_PROFILES}')
    generator_library = values['CONTROLLER_PROFILE_LIBRARY']
    for name in SELECTED_CONTROLLER_PROFILES:
        if name not in generator_library:
            raise ValueError(f'generator profile missing: {name}')
        if generator_library[name] != CONTROLLER_PROFILE_LIBRARY[name]:
            raise ValueError(
                f'generator/tuner profile constants differ for {name}')
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
    """Closed-loop fixed-point projection with optional manual speed ceilings."""
    values = (v_max, mu, a_acc_max, a_dec_max)
    if not all(np.isfinite(value) and value > 0.0 for value in values):
        raise ValueError('velocity limits must be positive and finite')
    s = np.asarray(s, dtype=float)
    kappa = np.abs(np.asarray(kappa, dtype=float))
    segment_ds = np.asarray(segment_ds, dtype=float)
    count = len(s)
    if count < 3 or len(kappa) != count or len(segment_ds) != count or \
            not np.isfinite(s).all() or not np.isfinite(kappa).all() or \
            not np.isfinite(segment_ds).all() or np.any(segment_ds <= 1e-6):
        raise ValueError('closed-loop velocity input is malformed')
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
                    segment_ds[index], a_acc_max, projection_limit,
                    v[following])
                max_reduction = max(max_reduction, v[following] - safe)
                v[following] = safe
            else:
                safe = _max_segment_endpoint_speed(
                    v[following], kappa[following], kappa[index],
                    segment_ds[index], a_dec_max, projection_limit, v[index])
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
        (ax > a_acc_max + 1e-8) |
        (ax < -a_dec_max - 1e-8) |
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
        self.lookahead_clip = (
            self.profile['lookahead_min_m'],
            self.profile['lookahead_max_m'])
        self.a_acc_max = self.profile['max_accel_mps2']
        self.a_dec_max = self.profile['max_decel_mps2']
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
            query = position + speed * 0.25 * np.array([
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
        fields = (
            't', 'x', 'y', 'theta', 'v', 'v_ref', 's', 'd_err', 'steer',
            'a_long_cmd', 'a_lat_cmd', 'a_long', 'a_lat',
            'combined_demand', 'combined_accel', 'tire_saturated')
        log = {key: [] for key in fields}
        elapsed = 0.0
        laps = 0
        steering = 0.0
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
            lookahead = np.clip(
                self.lookahead_gain * abs(speed) + self.lookahead_offset,
                lookahead_min, self.lookahead_clip[1])
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

            speed_command = self._speed_target(
                position, theta, speed, target_index)
            a_long_command = float(np.clip(
                (speed_command - speed) / self.dt,
                -self.a_dec_max, self.a_acc_max))
            if self.profile['kind'] == 'cpp_map_l1':
                a_lat_command = _lookup_lateral_accel(
                    self.steering_lut, steering, speed)
            else:
                a_lat_command = float(
                    speed ** 2 * np.tan(steering) / self.wheelbase)
            combined_demand = float(np.hypot(
                a_long_command, a_lat_command))
            a_lat = float(np.clip(
                a_lat_command, -self.tire_limit, self.tire_limit))
            longitudinal_available = float(np.sqrt(max(
                self.tire_limit ** 2 - a_lat ** 2, 0.0)))
            a_long = float(np.clip(
                a_long_command,
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
                steering, a_long_command, a_lat_command, a_long, a_lat,
                combined_demand, combined_accel, float(saturated))
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
    fields = (
        't', 'x', 'y', 'theta', 'v', 'v_ref', 's', 'd_err', 'steer',
        'a_long_cmd', 'a_lat_cmd', 'a_long', 'a_lat',
        'combined_demand', 'combined_accel', 'tire_saturated')
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
        log['a_long_cmd'], log['a_lat_cmd'])
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
    return not failures, failures
