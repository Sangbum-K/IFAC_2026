"""Regression tests for the tuner's fail-closed, pure-numpy safety model."""

import ast
import importlib.util
from pathlib import Path

import numpy as np
import pytest


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
MODEL_PATH = (
    PACKAGE_ROOT / 'scripts' / 'tuner' / 'controller_safety_model.py')
BUILD_MAP_PATH = (
    PACKAGE_ROOT / 'scripts' / 'global_traj_sim' / 'build_map.py')
TUNER_NODE_PATH = (
    PACKAGE_ROOT / 'scripts' / 'tuner' / 'raceline_tuner_node.py')


def _load_model():
    spec = importlib.util.spec_from_file_location(
        'planning_tuner_safety_test_model', MODEL_PATH)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def _load_generator_safety_slice(steering_lut):
    """Execute only build_map's pure controller AST, without heavy imports."""
    tree = ast.parse(
        BUILD_MAP_PATH.read_text(encoding='utf-8'),
        filename=str(BUILD_MAP_PATH))
    assignments = {
        'G', 'MU', 'V_MAX', 'WHEELBASE', 'MAX_STEER', 'A_ACC_MAX',
        'A_DEC_MAX', 'MAX_STEER_RATE', 'N_LOCAL_WP',
        'SIM_CONTROLLER_PROFILES', 'CONTROLLER_PROFILE_LIBRARY',
        '_PROFILE_NUMERIC_FIELDS',
    }
    definitions = {
        '_max_segment_endpoint_speed', 'velocity_profile',
        'controller_profile', 'selected_controller_profiles',
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
    assert tuple(model.SELECTED_CONTROLLER_PROFILES) == (
        'cpp_real_car', 'python_sim_tracker')
    assert model.assert_generator_profile_contract(
        BUILD_MAP_PATH) == model.PROFILE_SIGNATURE


def test_generator_profile_drift_is_fail_closed(tmp_path):
    model = _load_model()
    changed = BUILD_MAP_PATH.read_text(encoding='utf-8').replace(
        '"planner_speed_scale": 0.60,',
        '"planner_speed_scale": 0.61,', 1)
    candidate = tmp_path / 'build_map.py'
    candidate.write_text(changed, encoding='utf-8')
    with pytest.raises(ValueError, match='profile constants differ'):
        model.assert_generator_profile_contract(candidate)


def test_default_wall_contract_accepts_unchanged_deployed_main_csv():
    """Prevent reintroducing the old 0.35 + margin double-reserve deadlock."""
    tree = ast.parse(
        TUNER_NODE_PATH.read_text(encoding='utf-8'),
        filename=str(TUNER_NODE_PATH))
    default_min_wall = None
    for call in (
            node for node in ast.walk(tree) if isinstance(node, ast.Call)):
        if isinstance(call.func, ast.Attribute) and \
                call.func.attr == 'declare_parameter' and \
                len(call.args) >= 2 and \
                isinstance(call.args[0], ast.Constant) and \
                call.args[0].value == 'tuner/min_wall_dist_m':
            default_min_wall = ast.literal_eval(call.args[1])
            break
    assert default_min_wall is not None
    model = _load_model()
    assert default_min_wall >= model.VEHICLE_HALF_WIDTH_M
    csv_path = (
        PACKAGE_ROOT / 'config' / 'global_trajectory' /
        'my_track_iqp_max6.00ms_margin0.08m_mu0.75.csv')
    trajectory = np.loadtxt(csv_path, delimiter=';', comments='#')
    assert np.min(trajectory[:, 3:5]) >= default_min_wall


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
        model.CONTROLLER_PROFILE_LIBRARY['cpp_real_car']['steering_lut'])
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
    fields = (
        't', 'x', 'y', 'theta', 'v', 'v_ref', 's', 'd_err', 'steer',
        'a_long_cmd', 'a_lat_cmd', 'a_long', 'a_lat',
        'combined_demand', 'combined_accel', 'tire_saturated')
    for profile_name in model.SELECTED_CONTROLLER_PROFILES:
        generator_log = generator['PurePursuitSim'](
            trajectory, profile_name, mu=0.75).run(
                n_laps=99, max_time=2.0)
        tuner_log = model.PurePursuitSim(
            trajectory, profile_name, workspace_root, mu=0.75).run(
                requested_laps=99, max_time_sec=2.0)
        for field in fields:
            assert np.allclose(
                tuner_log[field], generator_log[field],
                rtol=1e-12, atol=1e-12), (profile_name, field)
