# Copyright 2026 IFAC F1TENTH Team
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Tests for the cumulative PF bag-replay A/B profiles."""

import ast
import hashlib
import importlib.util
import json
import sys
from dataclasses import dataclass
from pathlib import Path
from typing import Optional

import numpy as np

import yaml


PROFILE_NAMES = (
    'variant0_no_ground_no_deskew',
    'variant0_no_ground_no_deskew_no_icp',
)
FEATURE_KEYS = (
    'rangelib_variant',
    'ground_filter_enabled',
    'deskew_enabled',
    'scan_match_enabled',
)


def _load_profiles():
    profile_dir = Path(__file__).parents[1] / 'config' / 'profiles'
    profiles = []
    for name in PROFILE_NAMES:
        with (profile_dir / (name + '.yaml')).open(encoding='utf-8') as stream:
            document = yaml.safe_load(stream)
        profiles.append(document['particle_filter']['ros__parameters'])
    return profiles


def _literal_assignment(path, name):
    tree = ast.parse(path.read_text(encoding='utf-8'), filename=str(path))
    for node in tree.body:
        if not isinstance(node, ast.Assign) or len(node.targets) != 1:
            continue
        target = node.targets[0]
        if isinstance(target, ast.Name) and target.id == name:
            return tuple(ast.literal_eval(node.value))
    raise AssertionError('%s is not assigned in %s' % (name, path))


def _load_replay_command_builder():
    """Load pure replay argv helpers without importing ROS bag modules."""
    repo_root = Path(__file__).resolve().parents[4]
    path = repo_root / 'tools/run_pf_ab_replay.py'
    tree = ast.parse(path.read_text(encoding='utf-8'), filename=str(path))
    selected = []
    names = {
        'REPLAY_TOPICS',
        'RECORD_TOPICS',
        'PASS_UPDATE_RATE_HZ',
        'sha256_bytes',
        'sha256_file',
        'stable_file_identity',
        'capture_input_bag_identity',
        'verify_input_bag_identity',
        'ros_double_literal',
        'laser_z_label',
        'build_replay_cases',
        'replay_topics_for_laser_z',
        'replay_topics',
        'initial_pose_command',
        'build_commands',
        'merge_driving_mode_analysis',
        'metric_summary',
        'value',
        'percent_value',
        'scaled_value',
        'render_summary',
    }
    for node in tree.body:
        if isinstance(node, ast.Assign):
            targets = {
                target.id for target in node.targets
                if isinstance(target, ast.Name)
            }
            if targets & names:
                selected.append(node)
        elif isinstance(node, ast.FunctionDef) and node.name in names:
            selected.append(node)
        elif isinstance(node, ast.ClassDef) and node.name == 'ReplayCase':
            selected.append(node)
    namespace = {
        'hashlib': hashlib,
        'json': json,
        'Path': Path,
        'RuntimeError': RuntimeError,
        'sys': sys,
        'dataclass': dataclass,
        'Optional': Optional,
    }
    module = ast.Module(body=selected, type_ignores=[])
    exec(compile(module, str(path), 'exec'), namespace)
    return namespace


def test_profiles_are_self_describing_and_change_one_feature_at_a_time():
    """Each cumulative overlay differs from its predecessor by one switch."""
    profiles = _load_profiles()

    for expected_name, profile in zip(PROFILE_NAMES, profiles):
        assert profile['pf_profile_label'] == expected_name
        assert set(FEATURE_KEYS).issubset(profile)

    for previous, current in zip(profiles, profiles[1:]):
        changed = [
            key for key in FEATURE_KEYS
            if previous[key] != current[key]
        ]
        assert len(changed) == 1


def test_profile_names_stay_synchronized_across_launch_and_replay():
    """The three entry points must expose the same ordered experiment set."""
    repo_root = Path(__file__).resolve().parents[4]
    assignments = (
        (
            repo_root / 'localization_ws/src/particle_filter_cpp/launch/'
            'localize_launch.py',
            'PF_AB_PROFILES',
        ),
        (
            repo_root / 'integration_ws/src/integration_pkg/launch/'
            'real_car.launch.py',
            'PF_AB_PROFILES',
        ),
        (repo_root / 'tools/run_pf_ab_replay.py', 'PROFILES'),
    )
    for path, assignment in assignments:
        assert _literal_assignment(path, assignment) == PROFILE_NAMES


def test_replay_commands_are_humble_safe_and_preserve_double_override():
    """Recorder flags and ROS parameter types must remain Humble-compatible."""
    helpers = _load_replay_command_builder()
    commands = helpers['build_commands'](
        Path('/input_bag'),
        Path('/result'),
        Path('/base.yaml'),
        Path('/profile.yaml'),
        Path('/map_server.py'),
        1.0,
        100.0,
        2.0,
        25.0,
        809,
    )
    _map_command, pf_command, record_command, play_command = commands

    assert 'max_update_rate_hz:=25.0' in pf_command
    assert 'random_seed:=809' in pf_command
    assert '--disable-keyboard-controls' not in record_command
    assert '--disable-keyboard-controls' in play_command


def test_recorded_pf_seed_skips_stale_initialpose_and_starts_at_seed():
    """Mid-session bags must not replay historical latched initial poses."""
    helpers = _load_replay_command_builder()
    commands = helpers['build_commands'](
        Path('/input_bag'), Path('/result'), Path('/base.yaml'),
        Path('/profile.yaml'), Path('/map_server.py'),
        1.0, 100.0, 2.0, 20.0, 809, 0.126, 3.47, True)
    play_command = commands[3]

    assert '/initialpose' not in play_command
    assert '/tf_static' not in play_command
    assert play_command[play_command.index('--start-offset') + 1] == '3.47'


def test_production_profile_preserves_20hz_guard_durations():
    """Update-count gates retain their validated duration after 15->20 Hz."""
    config = Path(__file__).parents[1] / 'config' / 'localize.yaml'
    with config.open(encoding='utf-8') as stream:
        profile = yaml.safe_load(stream)['particle_filter']['ros__parameters']

    assert profile['max_update_rate_hz'] == 20.0
    assert profile['quality_bad_updates_to_lost'] == 4
    assert profile['quality_good_updates_to_recover'] == 7
    assert profile['lost_recovery_stationary_updates'] == 20
    assert profile['slip_enter_updates'] == 3
    assert profile['slip_recover_updates'] == 7
    assert profile['max_consecutive_jump_rejects'] == 13
    assert profile['lost_recovery_close_range_m'] == 0.60
    assert profile['lost_recovery_max_close_fraction'] == 0.40
    assert profile['tracking_min_beams'] == 12
    assert profile['tracking_min_angle_span_rad'] == 1.57
    source = (Path(__file__).parents[1] / 'src' /
              'particle_filter_node.cpp').read_text(encoding='utf-8')
    assert 'scan.close_fraction <= lost_recovery_max_close_fraction_' in source
    assert '!innovation_fail_closed_ && !beam_fail_closed_' in source
    assert 'last_innovation_guard_.sample_count > 0U' in source


def test_replay_always_executes_native_cpp_runtime():
    """The replay command must not retain a Python PF fallback."""
    helpers = _load_replay_command_builder()
    commands = helpers['build_commands'](
        Path('/input_bag'), Path('/result'), Path('/base.yaml'),
        Path('/profile.yaml'), Path('/map_server.py'),
        1.0, 100.0, 2.0, 25.0, 809)

    assert commands[1][:4] == [
        'ros2', 'run', 'particle_filter_cpp',
        'particle_filter_cpp_node']


def test_laser_z_ab_replaces_recorded_static_tf_without_replaying_it():
    """Height A/B must change one TF edge without a duplicate publisher."""
    helpers = _load_replay_command_builder()
    cases = helpers['build_replay_cases'](
        ('variant0_no_ground_no_deskew',),
        (0.1100, 0.1374), 0.1100)

    assert [case.effective_base_to_laser_z_m for case in cases] == [
        0.1100, 0.1374]
    assert [case.label for case in cases] == [
        'variant0_no_ground_no_deskew__laser_z_0p1100m',
        'variant0_no_ground_no_deskew__laser_z_0p1374m',
    ]

    commands = helpers['build_commands'](
        Path('/input_bag'),
        Path('/result'),
        Path('/base.yaml'),
        Path('/profile.yaml'),
        Path('/map_server.py'),
        1.0,
        100.0,
        2.0,
        25.0,
        809,
        0.1374,
    )
    map_command, _pf_command, record_command, play_command = commands
    assert map_command[-2:] == ['--base-to-laser-z-m', '0.1374']
    assert '/tf_static' in record_command
    assert '/tf_static' not in play_command


def test_laser_z_override_changes_only_copied_direct_transform():
    """Recorded static TF messages must remain immutable and reusable."""
    from geometry_msgs.msg import TransformStamped

    repo_root = Path(__file__).resolve().parents[4]
    path = repo_root / 'tools/pf_replay_map_server.py'
    spec = importlib.util.spec_from_file_location(
        'pf_replay_map_server_test', path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)

    laser = TransformStamped()
    laser.header.frame_id = 'base_link'
    laser.child_frame_id = 'laser'
    laser.transform.translation.x = 0.27
    laser.transform.translation.z = 0.11
    laser.transform.rotation.w = 1.0
    imu = TransformStamped()
    imu.header.frame_id = 'base_link'
    imu.child_frame_id = 'vesc_imu'
    imu.transform.translation.x = -0.03
    imu.transform.rotation.w = 1.0

    modified, original_z = module.override_base_to_laser_z(
        [laser, imu], 0.1374)

    assert original_z == 0.11
    assert laser.transform.translation.z == 0.11
    assert module.find_direct_static_transform(
        modified).transform.translation.z == 0.1374
    assert modified[0].transform.translation.x == 0.27
    assert modified[1].transform.translation.x == -0.03
    assert modified[1].child_frame_id == 'vesc_imu'


def test_replay_identity_hashes_metadata_and_every_storage_file(tmp_path):
    """A result must be bound to bag bytes, not only a mutable path."""
    helpers = _load_replay_command_builder()
    bag = tmp_path / 'bag'
    bag.mkdir()
    (bag / 'metadata.yaml').write_text('storage_identifier: sqlite3\n')
    (bag / 'part_0.db3').write_bytes(b'first')
    (bag / 'part_1.db3').write_bytes(b'second')

    identity = helpers['capture_input_bag_identity'](bag)

    assert [item['path'] for item in identity['files']] == [
        'metadata.yaml', 'part_0.db3', 'part_1.db3']
    helpers['verify_input_bag_identity'](bag, identity)

    (bag / 'part_1.db3').write_bytes(b'changed')
    with np.testing.assert_raises_regex(
            RuntimeError, 'Input bag changed after preflight'):
        helpers['verify_input_bag_identity'](bag, identity)


def test_replay_summary_separates_runtime_from_autonomy_moving_quality():
    """The compact table cannot present whole-bag fit as autonomy quality."""
    helpers = _load_replay_command_builder()
    report = {
        'new': {
            'update_timing': {'count': 100, 'rate_hz': 16.0},
            'diagnostics': {
                'timing': {
                    'compute': {'p95': 20.0},
                    'age': {'p99': 5.0},
                },
                'counters': {'stale_drop': 0, 'worker_error': 0},
            },
            'map_fit_held_out_pre': {'within_0.10m_fraction': 0.50},
        },
    }
    mode_report = {
        'command_path': '/input.db3',
        'mux': {'matched_fraction': 0.99},
        'moving_fit': {'updates': 70},
        'moving_worst_intervals': [],
        'autonomy_moving_fit': {
            'updates': 60,
            'fit_10cm_mean': 0.90,
            'fit_10cm_p05': 0.70,
            'wall_p90_p95_m': 0.30,
            'bad_update_fraction': 0.10,
            'correction_translation_p95_m': 0.06,
            'correction_yaw_p95_deg': 1.2,
        },
        'autonomy_moving_worst_intervals': [],
    }

    merged = helpers['merge_driving_mode_analysis'](report, mode_report)
    row = helpers['metric_summary'](
        'test', merged, 25.0, 809,
        base_to_laser_z_m=0.1374,
        base_to_laser_z_source='override')
    rendered = helpers['render_summary']([row], 25.0, 809)

    assert row['held_out_pre_within_10cm_fraction'] == 0.50
    assert row['autonomy_moving_fit_10cm_mean'] == 0.90
    assert row['autonomy_moving_updates'] == 60
    assert row['base_to_laser_z_m'] == 0.1374
    assert 'WHOLE-REPLAY RUNTIME' in rendered
    assert 'AUTONOMY + MOVING QUALITY' in rendered
    assert 'LaserZcm' in rendered
    assert '13.74' in rendered
    assert '90.0%' in rendered
    assert '50.0%' not in rendered
