"""Regression tests for the tuner's fail-closed, pure-numpy safety model."""

import ast
import importlib.util
import sys
from pathlib import Path

import numpy as np
import pytest


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
SCRIPTS_ROOT = PACKAGE_ROOT / 'scripts'
MODEL_PATH = (
    PACKAGE_ROOT / 'scripts' / 'tuner' / 'controller_safety_model.py')
BUILD_MAP_PATH = (
    PACKAGE_ROOT / 'scripts' / 'global_traj_sim' / 'build_map.py')
sys.path.insert(0, str(SCRIPTS_ROOT))
import deployment_profiles as profiles  # noqa: E402


def _load_model():
    spec = importlib.util.spec_from_file_location(
        'planning_tuner_safety_test_model', MODEL_PATH)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def _load_generator_safety_slice(steering_lut):
    """Execute only build_map's pure controller AST, without heavy imports.

    Deployment profiles are no longer defined in build_map -- they come from
    ``deployment_profiles``, which is dependency-light enough to import here.
    """
    tree = ast.parse(
        BUILD_MAP_PATH.read_text(encoding='utf-8'),
        filename=str(BUILD_MAP_PATH))
    assignments = {'G', 'MU', 'V_MAX'}
    definitions = {
        '_max_segment_endpoint_speed', 'velocity_profile',
        '_lookup_steer_angle', '_lookup_lateral_accel', 'PurePursuitSim',
    }
    selected = []
    for statement in tree.body:
        if isinstance(statement, ast.Assign) and \
                len(statement.targets) == 1 and \
                isinstance(statement.targets[0], ast.Name) and \
                statement.targets[0].id in assignments:
            selected.append(statement)
        elif getattr(statement, 'name', None) in definitions:
            selected.append(statement)

    class NumpyTree:
        def __init__(self, points):
            self.points = np.asarray(points, dtype=float)

        def query(self, point):
            delta = self.points - np.asarray(point, dtype=float)
            squared = np.einsum('ij,ij->i', delta, delta)
            index = int(np.argmin(squared))
            return float(np.sqrt(squared[index])), index

    namespace = {
        'np': np,
        'cKDTree': NumpyTree,
        '_load_steering_lut': lambda _path: steering_lut,
        'RACELINE_V_MAX': profiles.RACELINE_V_MAX,
        'SIM_LOG_FIELDS': profiles.SIM_LOG_FIELDS,
        'controller_profile': profiles.controller_profile,
        'make_longitudinal_plant': profiles.make_longitudinal_plant,
        'profile_brake_max': profiles.profile_brake_max,
    }
    module = ast.Module(body=selected, type_ignores=[])
    ast.fix_missing_locations(module)
    exec(compile(module, str(BUILD_MAP_PATH), 'exec'), namespace)
    return namespace


def test_profile_manifest_is_versioned_and_matches_generator():
    model = _load_model()
    assert model.MODEL_VERSION.startswith('tuner-controller-safety-')
    assert len(model.PROFILE_SIGNATURE) == 64
    assert len(model.GENERATOR_MODEL_AST_SHA256) == 64
    assert [p['name'] for p in model.gate_controller_profiles()] == \
        ['cpp_real_car']
    assert [p['name'] for p in model.advisory_controller_profiles()] == \
        ['python_sim_tracker']
    assert model.assert_generator_profile_contract(
        BUILD_MAP_PATH) == model.PROFILE_SIGNATURE


def test_generator_algorithm_drift_is_fail_closed(tmp_path):
    """A generator algorithm edit must lock tuner saves until reviewed."""
    model = _load_model()
    source = BUILD_MAP_PATH.read_text(encoding='utf-8')
    changed = source.replace(
        'tire_saturated = combined_demand > self.tire_limit * (1.0 + 1e-9)',
        'tire_saturated = combined_demand > self.tire_limit * (1.0 + 1e-8)',
        1)
    assert changed != source, 'algorithm anchor line not found'
    candidate = tmp_path / 'build_map.py'
    candidate.write_text(changed, encoding='utf-8')
    with pytest.raises(ValueError, match='safety algorithm revision differs'):
        model.assert_generator_profile_contract(candidate)


def test_generator_reintroducing_a_profile_copy_is_fail_closed(tmp_path):
    """The whole point of deployment_profiles: no second copy of the numbers."""
    model = _load_model()
    source = BUILD_MAP_PATH.read_text(encoding='utf-8')
    candidate = tmp_path / 'build_map.py'

    candidate.write_text(
        source + '\nCONTROLLER_PROFILE_LIBRARY = {}\n', encoding='utf-8')
    with pytest.raises(ValueError, match='CONTROLLER_PROFILE_LIBRARY'):
        model.assert_generator_profile_contract(candidate)

    candidate.write_text(
        source.replace('from deployment_profiles import', 'from os import', 1),
        encoding='utf-8')
    with pytest.raises(ValueError, match='deployment_profiles'):
        model.assert_generator_profile_contract(candidate)


def test_deployed_yaml_is_the_only_source_of_profile_speeds():
    """Resolved profiles must equal what launch would load, not a copy."""
    control, _ = profiles.runtime_params(
        profiles.CONTROL_PKG_ROOT, 'control', 'real_car')
    planning, _ = profiles.runtime_params(
        profiles.PKG_ROOT, 'global_trajectory_node', 'real_car')
    profile = profiles.controller_profile('cpp_real_car')
    assert profile['max_speed_mps'] == control['vehicle/max_speed_mps']
    assert profile['controller_speed_scale'] == control['vehicle/speed_scale']
    assert profile['planner_speed_scale'] == \
        planning['global_trajectory/velocity_scale']
    assert profile['trajectory_validation_mu'] == \
        planning['global_trajectory/validation_mu']


def test_cpp_profile_includes_every_deployed_map_l1_term():
    """The safety model must not silently omit a control_node MAP-L1 term."""
    control, _ = profiles.runtime_params(
        profiles.CONTROL_PKG_ROOT, 'control', 'real_car')
    profile = profiles.controller_profile('cpp_real_car')
    fields = {
        'lat_err_coeff': 'map/lat_err_coeff',
        'speed_lookahead_for_steer_s': 'map/speed_lookahead_for_steer',
        'max_lateral_error_m': 'map/max_lateral_error_m',
        'max_curvature_reference': 'map/max_curvature_reference',
        'l1_speed_uses_path_plan': 'map/l1_speed_uses_path_plan',
        'steer_plan_speed_min_mps': 'map/steer_plan_speed_min_mps',
        'acc_scaler_for_steer': 'map/acc_scaler_for_steer',
        'dec_scaler_for_steer': 'map/dec_scaler_for_steer',
        'start_scale_speed': 'map/start_scale_speed',
        'end_scale_speed': 'map/end_scale_speed',
        'downscale_factor': 'map/downscale_factor',
        'velocity_steer_gain_max': 'map/velocity_steer_gain_max',
        'nearest_search_horizon_m': 'map/nearest_search_horizon_m',
    }
    for profile_field, yaml_key in fields.items():
        assert profile[profile_field] == control[yaml_key]
    assert profile['backstop']['steering_reference_rad'] == \
        control['map/backstop_steering_reference_rad']


def test_real_footprint_is_advisory_but_sim_footprint_is_hard():
    """Only the empirically-miscalibrated VESC tracking footprint is advisory."""
    model = _load_model()

    class OccupiedGrid:
        res = 0.05

        @staticmethod
        def is_free(points):
            return np.zeros(len(points), dtype=bool)

    log = {
        field: np.zeros(2, dtype=float)
        for field in profiles.SIM_LOG_FIELDS
    }
    log['t'] = np.array([0.1, 0.2])
    log.update({
        'laps': 1,
        'tire_limit': 1.5 * model.G,
        'model_version': model.MODEL_VERSION,
        'profile_signature': model.PROFILE_SIGNATURE,
    })
    log['profile_name'] = 'cpp_real_car'
    ok, messages = model.verify_simulation(
        'probe', log, OccupiedGrid(), 1, 'cpp_real_car', 1.5)
    assert ok
    assert any('[warn]' in message and 'footprint' in message
               for message in messages)
    assert not any('[FAIL]' in message for message in messages)

    log['profile_name'] = 'python_sim_tracker'
    ok, messages = model.verify_simulation(
        'probe', log, OccupiedGrid(), 1, 'python_sim_tracker', 1.5)
    assert not ok
    assert any('[FAIL]' in message and 'footprint' in message
               for message in messages)


def test_vehicle_geometry_drift_between_yaml_and_measured_is_fail_closed(
        monkeypatch):
    """control YAML disagreeing with the measured car must not validate.

    A wheelbase mismatch means the closed-loop gate and the minimum-turn-radius
    check are describing a different car than the one that will drive the CSV.
    """
    monkeypatch.setattr(profiles, 'WHEELBASE', profiles.WHEELBASE + 0.02)
    with pytest.raises(ValueError, match='vehicle/wheelbase_m'):
        profiles.controller_profile('cpp_real_car')


def test_vesc_brake_arbitration_matches_ackermann_to_vesc():
    """The plant must reproduce the deployed DRIVE/COAST/BRAKE state machine."""
    config = profiles.vesc_brake_config()
    assert config['enabled'], 'brake_control_enabled is off in vesc.yaml'
    plant = profiles.VescLongitudinalPlant(
        config, profiles.A_ACC_MAX, profiles.A_DEC_MAX)
    dt = 1.0 / 40.0

    # Accelerating: current-saturation ceiling, never the raw command slope.
    accel, mode, amps = plant.step(3.0, 9.0, 5.5, dt)
    assert mode == 'drive' and amps == 0.0
    assert accel == pytest.approx(profiles.A_ACC_MAX)

    # Speed error below brake_entry_speed_error: the deployed code keeps the
    # erpm/speed-PID path active rather than cutting to coast (53f2e44).
    _, mode, amps = plant.step(3.0, 3.0 - 0.5 * config['entry_speed_error'],
                               -1.0, dt)
    assert mode == 'drive' and amps == 0.0

    # At/above entry but neither brake-entry condition met (no planned
    # deceleration, overspeed below the no-intent threshold): coast.
    coast_error = 0.5 * (
        config['entry_speed_error'] + config['overspeed_error'])
    assert config['entry_speed_error'] <= coast_error < \
        config['overspeed_error']
    plant.reset()
    _, mode, amps = plant.step(3.0, 3.0 - coast_error, 0.0, dt)
    assert mode == 'coast' and amps == 0.0

    # Planned deceleration + error past entry: brake engages with gain*error.
    error = config['entry_speed_error'] + 0.05
    _, mode, amps = plant.step(3.0, 3.0 - error, -1.0, dt)
    assert mode == 'brake'
    assert amps == pytest.approx(
        min(config['brake_max'], config['gain'] * error))

    # Hysteresis: stays engaged between release and entry thresholds.
    held = 0.5 * (config['release_speed_error'] + config['entry_speed_error'])
    _, mode, _ = plant.step(3.0, 3.0 - held, -1.0, dt)
    assert mode == 'brake'
    # Released below the release threshold, and since that error is also below
    # brake_entry the deployed code returns to the erpm/speed-PID path, not
    # coast (53f2e44 changed `speed_error > 0` to `>= brake_entry`).
    _, mode, _ = plant.step(
        3.0, 3.0 - 0.5 * config['release_speed_error'], -1.0, dt)
    assert mode == 'drive'

    # Below brake_min_vehicle_speed the deployed code coasts instead.
    _, mode, _ = plant.step(
        0.5 * config['min_vehicle_speed'], 0.0, -1.0, dt)
    assert mode == 'coast'


def test_velocity_projection_respects_manual_ceiling_and_wrap_segment():
    model = _load_model()
    count = 80
    s = np.arange(count, dtype=float) * 0.1
    segment_ds = np.full(count, 0.1)
    kappa = np.zeros(count)
    # Put curvature on both sides of the last→first wrap.
    kappa[:5] = 1.0
    kappa[-4:] = 0.9
    ceiling = np.full(count, 6.0)
    ceiling[20] = 1.0
    speed = model.velocity_profile(
        s, kappa, 6.0, 0.75, 5.5, 7.0, segment_ds,
        speed_ceiling=ceiling)
    ax = model.closed_loop_ax(speed, segment_ds)
    lateral = speed ** 2 * np.abs(kappa)
    worst_segment_lateral = np.maximum(lateral, np.roll(lateral, -1))
    assert np.isfinite(speed).all()
    assert np.all(speed <= ceiling + 1e-12)
    assert np.allclose(
        ax,
        (np.roll(speed, -1) ** 2 - speed ** 2) / (2.0 * segment_ds))
    assert np.max(np.hypot(worst_segment_lateral, ax)) <= \
        0.75 * model.G + 1e-8


def test_lightweight_velocity_and_controller_logs_match_generator():
    model = _load_model()
    workspace_root = PACKAGE_ROOT.parents[2]
    lut_path = (
        workspace_root /
        profiles.controller_profile('cpp_real_car')['steering_lut'])
    steering_lut = np.loadtxt(lut_path, delimiter=',')
    generator = _load_generator_safety_slice(steering_lut)

    count = 120
    radius = 3.0
    angle = np.linspace(0.0, 2.0 * np.pi, count, endpoint=False)
    xy = np.column_stack([radius * np.cos(angle), radius * np.sin(angle)])
    segment_ds = np.hypot(
        np.roll(xy[:, 0], -1) - xy[:, 0],
        np.roll(xy[:, 1], -1) - xy[:, 1])
    s = np.concatenate([[0.0], np.cumsum(segment_ds[:-1])])
    kappa = np.full(count, 1.0 / radius)
    generator_speed = generator['velocity_profile'](
        s, kappa, 6.0, 0.75, 5.5, 7.0, segment_ds=segment_ds)
    tuner_speed = model.velocity_profile(
        s, kappa, 6.0, 0.75, 5.5, 7.0, segment_ds)
    assert np.allclose(tuner_speed, generator_speed, rtol=0.0, atol=1e-12)

    trajectory = {
        's': s,
        'xy': xy,
        'd_right': np.full(count, 1.0),
        'd_left': np.full(count, 1.0),
        'psi': angle + 0.5 * np.pi,
        'kappa': kappa,
        'vx': np.minimum(tuner_speed, 2.5),
        'ax': np.zeros(count),
    }
    gate, advisory = profiles.selected_controller_profiles()
    for profile in gate + advisory:
        profile_name = profile['name']
        generator_log = generator['PurePursuitSim'](
            trajectory, profile_name, mu=0.75).run(
                n_laps=99, max_time=2.0)
        tuner_log = model.PurePursuitSim(
            trajectory, profile_name, workspace_root, mu=0.75).run(
                requested_laps=99, max_time_sec=2.0)
        for field in profiles.SIM_LOG_FIELDS:
            assert np.allclose(
                tuner_log[field], generator_log[field],
                rtol=1e-12, atol=1e-12), (profile_name, field)
