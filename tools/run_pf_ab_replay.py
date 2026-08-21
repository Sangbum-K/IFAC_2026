#!/usr/bin/env python3
"""Run controlled PF feature A/B replays against one real rosbag.

Only the recorded PF inputs and map are replayed in an isolated ROS domain.
No hardware, planning, perception, control, mux, or actuation node is launched.
Each profile gets its own sqlite3 output bag, logs, manifest, and no-GT report.
"""

import argparse
import hashlib
import json
import math
import os
import shlex
import shutil
import signal
import subprocess
import sys
import time
from dataclasses import dataclass
from datetime import datetime, timezone
from pathlib import Path
from typing import Optional

from pf_replay_map_server import find_direct_static_transform
from pf_replay_map_server import load_recorded_static_transforms
from pf_replay_map_server import storage_id

import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message


REPO_ROOT = Path(__file__).resolve().parents[1]
DEFAULT_BAG = REPO_ROOT / 'bags' / 'rosbag0809' / 'run_0809_231809'
DEFAULT_OUTPUT_ROOT = REPO_ROOT / 'bags' / 'pf_ab_replay'
PROFILES = (
    'variant0_no_ground_no_deskew',
    'variant0_no_ground_no_deskew_no_icp',
)
REPLAY_TOPICS = (
    '/map',
    '/tf_static',
    '/initialpose',
    '/scan',
    '/odom',
    '/sensors/imu/raw',
)
RECORD_TOPICS = REPLAY_TOPICS + (
    '/clock',
    '/pf/pose/odom',
    '/pf/viz/fake_scan',
    '/pf/viz/inferred_pose',
    '/pf/viz/particles',
    '/pf/localization_ready',
    '/pf/localization_quality',
    '/pf/initialpose_received',
    '/pf/initialpose_accepted',
    '/tf',
    '/rosout',
)
REQUIRED_TYPES = {
    '/map': 'nav_msgs/msg/OccupancyGrid',
    '/tf_static': 'tf2_msgs/msg/TFMessage',
    '/initialpose': 'geometry_msgs/msg/PoseWithCovarianceStamped',
    '/scan': 'sensor_msgs/msg/LaserScan',
    '/odom': 'nav_msgs/msg/Odometry',
    '/sensors/imu/raw': 'sensor_msgs/msg/Imu',
}
PF_READY_LOG = 'Finished initializing PF profile '
PASS_UPDATE_RATE_HZ = 15.0
MIN_RATE_CAP_HEADROOM_HZ = 5.0
DEFAULT_MAX_UPDATE_RATE_HZ = 25.0
DEFAULT_RANDOM_SEED = 809


@dataclass
class ManagedProcess:
    """A subprocess and the log stream that must remain open while it runs."""

    name: str
    process: subprocess.Popen
    log_stream: object
    command: list


@dataclass(frozen=True)
class WorktreeEvidence:
    """Git identity plus exact raw evidence captured before a run."""

    manifest: dict
    status_porcelain: bytes
    status_short: bytes
    diff_head: bytes


@dataclass(frozen=True)
class ReplayCase:
    """One PF profile and static-TF calibration executed as a unit."""

    label: str
    profile: str
    base_to_laser_z_override_m: Optional[float]
    effective_base_to_laser_z_m: float
    base_to_laser_z_source: str


def shell_join(command):
    """Render an argv list without executing through a shell."""
    return ' '.join(shlex.quote(str(item)) for item in command)


def ros_double_literal(value):
    """Serialize a ROS DOUBLE override without allowing integer inference."""
    literal = str(float(value))
    if '.' not in literal and 'e' not in literal.lower():
        literal += '.0'
    return literal


def laser_z_label(profile, height_m):
    """Build a deterministic filesystem-safe label for a LiDAR-z case."""
    encoded = ('%.4f' % height_m).replace('-', 'minus_').replace('.', 'p')
    return '%s__laser_z_%sm' % (profile, encoded)


def build_replay_cases(profiles, height_values, recorded_height_m):
    """Expand PF profiles or an ordered LiDAR-height comparison into runs."""
    profiles = tuple(profiles)
    if not height_values:
        return [
            ReplayCase(
                label=profile,
                profile=profile,
                base_to_laser_z_override_m=None,
                effective_base_to_laser_z_m=recorded_height_m,
                base_to_laser_z_source='recorded',
            )
            for profile in profiles
        ]

    heights = tuple(float(value) for value in height_values)
    if len(set(heights)) != len(heights):
        raise ValueError('LiDAR z comparison values must be unique')
    if len(heights) > 1 and len(profiles) != 1:
        raise ValueError(
            'multiple --base-to-laser-z-m values require exactly one '
            '--profiles entry so only calibration changes between runs')
    if len(heights) == 1:
        height_m = heights[0]
        return [
            ReplayCase(
                label=profile,
                profile=profile,
                base_to_laser_z_override_m=height_m,
                effective_base_to_laser_z_m=height_m,
                base_to_laser_z_source='override',
            )
            for profile in profiles
        ]
    return [
        ReplayCase(
            label=laser_z_label(profiles[0], height_m),
            profile=profiles[0],
            base_to_laser_z_override_m=height_m,
            effective_base_to_laser_z_m=height_m,
            base_to_laser_z_source='override',
        )
        for height_m in heights
    ]


def replay_topics_for_laser_z(base_to_laser_z_m):
    """Suppress recorded static TF when the map server republishes it."""
    return tuple(
        topic for topic in REPLAY_TOPICS
        if not (
            base_to_laser_z_m is not None and topic == '/tf_static'))


def replay_topics(base_to_laser_z_m, seed_from_recorded_pf=False):
    """Drop stale recorded initial poses when replay seeds from PF output."""
    return tuple(
        topic for topic in replay_topics_for_laser_z(base_to_laser_z_m)
        if not (seed_from_recorded_pf and topic == '/initialpose'))


def first_recorded_pf_seed(bag_path):
    """Return the first recorded map-frame base pose and its bag offset."""
    topic_types = read_topic_types(bag_path)
    message_name = topic_types.get('/pf/pose/odom')
    if message_name != 'nav_msgs/msg/Odometry':
        raise RuntimeError(
            '--seed-from-recorded-pf requires /pf/pose/odom; found %r'
            % message_name)
    message_type = get_message(message_name)
    metadata = rosbag2_py.Info().read_metadata(
        str(bag_path), storage_id(bag_path))
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(
            uri=str(bag_path), storage_id=storage_id(bag_path)),
        rosbag2_py.ConverterOptions('', ''),
    )
    while reader.has_next():
        topic, data, timestamp_ns = reader.read_next()
        if topic != '/pf/pose/odom':
            continue
        message = deserialize_message(data, message_type)
        orientation = message.pose.pose.orientation
        return {
            'start_offset_sec': max(
                0.0,
                (timestamp_ns - metadata.starting_time.nanoseconds) / 1e9),
            'x': float(message.pose.pose.position.x),
            'y': float(message.pose.pose.position.y),
            'z': float(orientation.z),
            'w': float(orientation.w),
        }
    raise RuntimeError('/pf/pose/odom contains no messages')


def initial_pose_command(seed):
    """Build one ROS CLI publication for a deterministic replay seed."""
    message = {
        'header': {'frame_id': 'map'},
        'pose': {'pose': {
            'position': {'x': seed['x'], 'y': seed['y'], 'z': 0.0},
            'orientation': {
                'x': 0.0, 'y': 0.0, 'z': seed['z'], 'w': seed['w']},
        }},
    }
    return [
        'ros2', 'topic', 'pub', '--once', '/initialpose',
        'geometry_msgs/msg/PoseWithCovarianceStamped',
        json.dumps(message, separators=(',', ':')),
    ]


def static_transform_manifest(transform):
    """Serialize one recorded static TF without relying on ROS repr text."""
    translation = transform.transform.translation
    rotation = transform.transform.rotation
    return {
        'parent_frame': transform.header.frame_id,
        'child_frame': transform.child_frame_id,
        'translation_xyz_m': [
            float(translation.x),
            float(translation.y),
            float(translation.z),
        ],
        'quaternion_xyzw': [
            float(rotation.x),
            float(rotation.y),
            float(rotation.z),
            float(rotation.w),
        ],
    }


def read_topic_types(bag_path):
    """Return rosbag topic types without deserializing its messages."""
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(
            uri=str(bag_path), storage_id=storage_id(bag_path)),
        rosbag2_py.ConverterOptions('', ''),
    )
    return {
        item.name: item.type for item in reader.get_all_topics_and_types()
    }


def validate_bag(bag_path):
    """Fail before starting ROS if an exact PF replay input is unavailable."""
    if not bag_path.exists():
        raise RuntimeError('Input bag does not exist: %s' % bag_path)
    topic_types = read_topic_types(bag_path)
    problems = []
    for topic, expected in REQUIRED_TYPES.items():
        actual = topic_types.get(topic)
        if actual != expected:
            problems.append('%s: expected %s, found %r' % (
                topic, expected, actual))
    if problems:
        raise RuntimeError(
            'Bag cannot reproduce the PF input:\n  ' + '\n  '.join(problems))
    return topic_types


def package_share(package):
    """Resolve an installed ROS package and explain the common source error."""
    result = subprocess.run(
        ['ros2', 'pkg', 'prefix', package, '--share'],
        check=False,
        text=True,
        capture_output=True,
    )
    if result.returncode != 0:
        raise RuntimeError(
            'ROS package %r is not visible. Build localization_ws and '
            'source '
            'its install/setup.bash before running this tool.\n%s'
            % (package, result.stderr.strip()))
    return Path(result.stdout.strip()).resolve()


def sha256_bytes(value):
    """Return a lowercase SHA256 digest for bytes."""
    return hashlib.sha256(value).hexdigest()


def sha256_file(path):
    """Hash a file without loading a potentially large artifact at once."""
    digest = hashlib.sha256()
    with Path(path).open('rb') as stream:
        while True:
            block = stream.read(1024 * 1024)
            if not block:
                break
            digest.update(block)
    return digest.hexdigest()


def stable_file_identity(path):
    """Hash one bag file and reject a concurrent write during the hash."""
    path = Path(path)
    before = path.stat()
    digest = sha256_file(path)
    after = path.stat()
    if (before.st_size, before.st_mtime_ns) != (
            after.st_size, after.st_mtime_ns):
        raise RuntimeError(
            'Input bag file changed while hashing: %s' % path)
    return {
        'size': after.st_size,
        'sha256': digest,
    }


def capture_input_bag_identity(bag_path):
    """Bind an experiment to metadata plus every rosbag storage file."""
    bag_path = Path(bag_path).resolve()
    if bag_path.is_dir():
        bag_is_directory = True
        storage_files = sorted({
            path.resolve()
            for suffix in ('*.db3', '*.mcap')
            for path in bag_path.rglob(suffix)
            if path.is_file()
        })
        files = []
        metadata = bag_path / 'metadata.yaml'
        if metadata.is_file():
            files.append(metadata.resolve())
        files.extend(storage_files)
    elif bag_path.is_file() and bag_path.suffix in ('.db3', '.mcap'):
        bag_is_directory = False
        storage_files = [bag_path]
        files = [bag_path]
    else:
        raise RuntimeError('Input is not a rosbag directory/storage file: %s'
                           % bag_path)
    if not storage_files:
        raise RuntimeError('Input bag contains no .db3 or .mcap files: %s'
                           % bag_path)

    entries = []
    for path in files:
        entry = stable_file_identity(path)
        entry['path'] = (
            str(path.relative_to(bag_path))
            if bag_is_directory else path.name)
        entries.append(entry)
    fingerprint_payload = json.dumps(
        entries, sort_keys=True, separators=(',', ':'),
        ensure_ascii=True).encode('utf-8')
    return {
        'format': 'rosbag-content-sha256-v1',
        'resolved_path': str(bag_path),
        'files': entries,
        'fingerprint_sha256': sha256_bytes(fingerprint_payload),
    }


def verify_input_bag_identity(bag_path, expected):
    """Rehash the input and fail if any byte changed since preflight."""
    actual = capture_input_bag_identity(bag_path)
    if actual['fingerprint_sha256'] != expected['fingerprint_sha256']:
        raise RuntimeError(
            'Input bag changed after preflight: expected %s, found %s'
            % (
                expected['fingerprint_sha256'],
                actual['fingerprint_sha256'],
            ))
    return actual


def git_output(arguments):
    """Run one read-only Git query and retain byte-exact output."""
    result = subprocess.run(
        ['git', *arguments], cwd=REPO_ROOT,
        check=False, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
    if result.returncode != 0:
        raise RuntimeError(
            'git %s failed: %s'
            % (' '.join(arguments), result.stderr.decode(
                'utf-8', errors='replace').strip()))
    return result.stdout


def untracked_entry(relative_bytes):
    """Hash one non-ignored untracked path without following symlink text."""
    relative = os.fsdecode(relative_bytes)
    path = REPO_ROOT / relative
    if path.is_symlink():
        target = os.readlink(path)
        payload = os.fsencode(target)
        kind = 'symlink'
    elif path.is_file():
        return {
            'path': relative,
            'kind': 'file',
            'size': path.stat().st_size,
            'sha256': sha256_file(path),
        }
    elif not path.exists():
        payload = b'missing'
        kind = 'missing'
    else:
        payload = ('mode:%o' % path.lstat().st_mode).encode('ascii')
        kind = 'special'
    return {
        'path': relative,
        'kind': kind,
        'size': len(payload),
        'sha256': sha256_bytes(payload),
    }


def capture_worktree_evidence():
    """Fingerprint HEAD plus every tracked change and untracked file body."""
    head = git_output(['rev-parse', 'HEAD']).decode('ascii').strip()
    status_porcelain = git_output([
        'status', '--porcelain=v1', '-z', '--untracked-files=all'])
    status_short = git_output([
        'status', '--short', '--untracked-files=all'])
    # HEAD includes every clean tracked file. This binary diff adds staged,
    # unstaged, deleted, renamed, and binary tracked changes to that identity.
    diff_head = git_output(['diff', '--binary', 'HEAD', '--'])
    untracked_raw = git_output([
        'ls-files', '--others', '--exclude-standard', '-z'])
    untracked = [
        untracked_entry(relative)
        for relative in sorted(filter(None, untracked_raw.split(b'\0')))
    ]
    identity = {
        'format': 'git-worktree-sha256-v1',
        'head': head,
        'dirty': bool(status_porcelain),
        'status_porcelain_sha256': sha256_bytes(status_porcelain),
        'diff_head_sha256': sha256_bytes(diff_head),
        'untracked': untracked,
    }
    canonical = json.dumps(
        identity, sort_keys=True, separators=(',', ':'),
        ensure_ascii=True).encode('utf-8')
    identity['fingerprint_sha256'] = sha256_bytes(canonical)
    return WorktreeEvidence(
        manifest=identity,
        status_porcelain=status_porcelain,
        status_short=status_short,
        diff_head=diff_head,
    )


def persist_worktree_evidence(session_dir, evidence):
    """Write the raw Git inputs used to calculate a session fingerprint."""
    directory = session_dir / 'worktree'
    directory.mkdir(parents=True, exist_ok=False)
    paths = {
        'status_porcelain_v1_z': directory / 'status_porcelain_v1_z.bin',
        'status_short': directory / 'status_short.txt',
        'diff_head_binary': directory / 'diff_head_binary.patch',
        'untracked_hashes': directory / 'untracked_hashes.json',
    }
    paths['status_porcelain_v1_z'].write_bytes(evidence.status_porcelain)
    paths['status_short'].write_bytes(evidence.status_short)
    paths['diff_head_binary'].write_bytes(evidence.diff_head)
    write_json(paths['untracked_hashes'], evidence.manifest['untracked'])
    return {
        name: {
            'path': str(path),
            'sha256': sha256_file(path),
        }
        for name, path in paths.items()
    }


def resolve_active_cpp_runtime(package_prefix):
    """Resolve and hash the installed C++ PF executable."""
    package_prefix = Path(package_prefix).resolve()
    executable = (
        package_prefix / 'lib' / 'particle_filter_cpp' /
        'particle_filter_cpp_node')
    if not executable.is_file():
        raise RuntimeError(
            'Installed C++ PF executable is missing: %s' % executable)
    if not os.access(executable, os.X_OK):
        raise RuntimeError(
            'Installed C++ PF executable is not executable: %s' % executable)
    digest = sha256_file(executable)
    return {
        'engine': 'cpp',
        'package_prefix': str(package_prefix),
        'entrypoint_path': str(executable),
        'entrypoint_sha256': digest,
        'python_tag': 'native-cpp',
        'egg_link_path': None,
        'egg_link_sha256': None,
        'import_spec_path': None,
        'active_particle_filter_path': str(executable),
        'active_particle_filter_sha256': digest,
        'active_package_files': [],
    }


def snapshot_file(source, destination):
    """Copy and hash one exact runtime input, rejecting a concurrent change."""
    source = Path(source)
    destination.parent.mkdir(parents=True, exist_ok=True)
    before = sha256_file(source)
    shutil.copy2(source, destination)
    after = sha256_file(source)
    copied = sha256_file(destination)
    if before != after or after != copied:
        raise RuntimeError(
            'Runtime artifact changed while snapshotting: %s' % source)
    return {
        'source_path': str(source),
        'source_resolved_path': str(source.resolve()),
        'snapshot_path': str(destination),
        'sha256': copied,
        'size': destination.stat().st_size,
    }


def snapshot_runtime_artifacts(result_dir, common, profile_config):
    """Freeze every YAML/tool/code artifact that can affect this PF run."""
    directory = result_dir / 'snapshot'
    runtime = common['pf_runtime']
    active_name = 'particle_filter_cpp_node'
    sources = {
        'base_config': (
            common['base_config'], directory / 'config' / 'localize.yaml'),
        'profile_config': (
            profile_config, directory / 'config' / 'profile.yaml'),
        'active_particle_filter': (
            runtime['active_particle_filter_path'],
            directory / 'code' / active_name),
        'run_pf_ab_replay': (
            Path(__file__).resolve(),
            directory / 'tools' / 'run_pf_ab_replay.py'),
        'pf_replay_map_server': (
            REPO_ROOT / 'tools' / 'pf_replay_map_server.py',
            directory / 'tools' / 'pf_replay_map_server.py'),
        'compare_pf_replay': (
            REPO_ROOT / 'tools' / 'compare_pf_replay.py',
            directory / 'tools' / 'compare_pf_replay.py'),
        'analyze_pf_bags': (
            REPO_ROOT / 'tools' / 'analyze_pf_bags.py',
            directory / 'tools' / 'analyze_pf_bags.py'),
        'particle_filter_entrypoint': (
            runtime['entrypoint_path'],
            directory / 'install' / 'particle_filter_entrypoint'),
    }
    if runtime['egg_link_path'] is not None:
        sources['particle_filter_egg_link'] = (
            runtime['egg_link_path'],
            directory / 'install' / 'particle-filter.egg-link')
    for item in runtime['active_package_files']:
        module_path = Path(item['path'])
        if module_path.resolve() == Path(
                runtime['active_particle_filter_path']).resolve():
            continue
        sources['active_pf_module_' + module_path.stem] = (
            module_path,
            directory / 'code' / 'particle_filter' / module_path.name)
    return {
        name: snapshot_file(source, destination)
        for name, (source, destination) in sources.items()
    }


def write_json(path, value):
    """Atomically update a small experiment metadata file."""
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_suffix(path.suffix + '.tmp')
    temporary.write_text(
        json.dumps(value, indent=2, ensure_ascii=False) + '\n',
        encoding='utf-8',
    )
    temporary.replace(path)


def start_process(name, command, log_path, environment):
    """Start an isolated process group and stream output to a label file."""
    log_path.parent.mkdir(parents=True, exist_ok=True)
    stream = log_path.open('w', encoding='utf-8')
    stream.write('$ ' + shell_join(command) + '\n\n')
    stream.flush()
    process = subprocess.Popen(
        [str(item) for item in command],
        stdout=stream,
        stderr=subprocess.STDOUT,
        env=environment,
        start_new_session=True,
    )
    return ManagedProcess(name, process, stream, command)


def stop_process(handle, timeout_sec=15.0):
    """SIGINT a complete process group so rosbag2 gets time to finalize."""
    if handle is None:
        return None
    process = handle.process
    if process.poll() is None:
        try:
            os.killpg(process.pid, signal.SIGINT)
        except ProcessLookupError:
            pass
        try:
            process.wait(timeout=timeout_sec)
        except subprocess.TimeoutExpired:
            try:
                os.killpg(process.pid, signal.SIGTERM)
            except ProcessLookupError:
                pass
            try:
                process.wait(timeout=5.0)
            except subprocess.TimeoutExpired:
                try:
                    os.killpg(process.pid, signal.SIGKILL)
                except ProcessLookupError:
                    pass
                process.wait(timeout=5.0)
    return_code = process.returncode
    handle.log_stream.close()
    return return_code


def wait_for_log(handle, log_path, text, timeout_sec):
    """Wait until a service/node explicitly reports readiness."""
    deadline = time.monotonic() + timeout_sec
    while time.monotonic() < deadline:
        return_code = handle.process.poll()
        if return_code is not None:
            raise RuntimeError(
                '%s exited before readiness (code %s); see %s'
                % (handle.name, return_code, log_path))
        try:
            if text in log_path.read_text(encoding='utf-8', errors='replace'):
                return
        except FileNotFoundError:
            pass
        time.sleep(0.2)
    raise RuntimeError(
        '%s did not report readiness within %.1f s; see %s'
        % (handle.name, timeout_sec, log_path))


def wait_for_playback(player, particle_filter):
    """Wait for playback while detecting an early PF crash immediately."""
    while True:
        player_code = player.process.poll()
        pf_code = particle_filter.process.poll()
        if pf_code is not None:
            raise RuntimeError(
                'particle_filter exited during playback (code %s)' % pf_code)
        if player_code is not None:
            if player_code != 0:
                raise RuntimeError(
                    'ros2 bag play failed with code %s' % player_code)
            player.log_stream.close()
            return
        time.sleep(0.5)


def run_analysis(
        analyzer_tool, mode_analyzer_tool, input_bag, output_bag,
        result_dir, settle_sec):
    """Run whole-bag runtime and source-command mode-filtered analyzers."""
    report_path = result_dir / 'analysis.json'
    command = [
        sys.executable,
        str(analyzer_tool),
        '--old', str(input_bag),
        '--new', str(output_bag),
        '--settle-sec', str(settle_sec),
        '--old-angle-step', '15',
        '--new-angle-step', '15',
        '--format', 'json',
        '--output', str(report_path),
    ]
    result = subprocess.run(
        command,
        cwd=REPO_ROOT,
        check=False,
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
    )
    (result_dir / 'analysis.log').write_text(
        '$ ' + shell_join(command) + '\n\n' + result.stdout,
        encoding='utf-8',
    )
    if result.returncode != 0:
        raise RuntimeError(
            'PF analyzer failed with code %s; see %s'
            % (result.returncode, result_dir / 'analysis.log'))
    report = json.loads(report_path.read_text(encoding='utf-8'))

    mode_report_path = result_dir / 'mode_analysis.json'
    mode_command = [
        sys.executable,
        str(mode_analyzer_tool),
        str(output_bag),
        '--command-bag', str(input_bag),
        '--settle-sec', str(settle_sec),
        '--angle-step', '15',
        '--held-out-stride', '10',
        '--json',
        '--output', str(mode_report_path),
    ]
    mode_result = subprocess.run(
        mode_command,
        cwd=REPO_ROOT,
        check=False,
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
    )
    (result_dir / 'mode_analysis.log').write_text(
        '$ ' + shell_join(mode_command) + '\n\n' + mode_result.stdout,
        encoding='utf-8',
    )
    if mode_result.returncode != 0:
        raise RuntimeError(
            'PF driving-mode analyzer failed with code %s; see %s'
            % (mode_result.returncode, result_dir / 'mode_analysis.log'))
    mode_reports = json.loads(
        mode_report_path.read_text(encoding='utf-8'))
    if len(mode_reports) != 1:
        raise RuntimeError(
            'PF driving-mode analyzer returned %d reports; expected one'
            % len(mode_reports))
    mode_report = mode_reports[0]
    report = merge_driving_mode_analysis(report, mode_report)
    write_json(report_path, report)
    return report


def merge_driving_mode_analysis(report, mode_report):
    """Attach exact source-command quality metrics to one replay report."""
    candidate = report['new']
    for key in (
            'moving_fit', 'moving_worst_intervals', 'mux',
            'autonomy_moving_fit',
            'autonomy_moving_worst_intervals'):
        candidate[key] = mode_report.get(key)
    candidate['driving_mode_filter'] = {
        'command_bag': mode_report.get('command_path'),
        'mode': 'autonomy',
        'minimum_speed_mps': 0.5,
        'maximum_command_age_ms': 100.0,
        'classification': (
            'exact ackermann_mux output-to-source message match'),
    }
    return report


def metric_summary(
        profile, report, max_update_rate_hz, random_seed,
        base_to_laser_z_m=None, base_to_laser_z_source=None):
    """Extract the fixed go/no-go metrics from one full analyzer report."""
    candidate = report['new']
    diagnostics = candidate['diagnostics']
    timing = diagnostics.get('timing', {})
    counters = diagnostics.get('counters', {})
    update_rate = candidate.get('update_timing', {}).get('rate_hz')
    compute_p95 = timing.get('compute', {}).get('p95')
    age_p99 = timing.get('age', {}).get('p99')
    stale = counters.get('stale_drop')
    worker_error = counters.get('worker_error')
    within_10cm = candidate.get(
        'map_fit_held_out_pre', {}).get('within_0.10m_fraction')
    mux = candidate.get('mux') or {}
    autonomy = candidate.get('autonomy_moving_fit') or {}
    thresholds_available = all(
        value is not None for value in (
            update_rate, age_p99, stale, worker_error))
    runtime_pass = bool(thresholds_available and all((
        update_rate >= PASS_UPDATE_RATE_HZ,
        age_p99 <= 100.0,
        stale == 0,
        worker_error == 0,
    )))
    return {
        'profile': profile,
        'base_to_laser_z_m': base_to_laser_z_m,
        'base_to_laser_z_source': base_to_laser_z_source,
        'updates': candidate.get('update_timing', {}).get('count'),
        'pf_update_rate_hz': update_rate,
        'compute_p95_ms': compute_p95,
        'correction_age_p99_ms': age_p99,
        'stale_drop': stale,
        'worker_error': worker_error,
        'held_out_pre_within_10cm_fraction': within_10cm,
        'mux_matched_fraction': mux.get('matched_fraction'),
        'autonomy_moving_updates': autonomy.get('updates'),
        'autonomy_moving_fit_10cm_mean': autonomy.get('fit_10cm_mean'),
        'autonomy_moving_fit_10cm_p05': autonomy.get('fit_10cm_p05'),
        'autonomy_moving_wall_p90_p95_m': autonomy.get(
            'wall_p90_p95_m'),
        'autonomy_moving_bad_update_fraction': autonomy.get(
            'bad_update_fraction'),
        'autonomy_moving_correction_translation_p95_m': autonomy.get(
            'correction_translation_p95_m'),
        'autonomy_moving_correction_yaw_p95_deg': autonomy.get(
            'correction_yaw_p95_deg'),
        'runtime_threshold_pass': runtime_pass,
        'replay_max_update_rate_hz': max_update_rate_hz,
        'replay_random_seed': random_seed,
    }


def value(value, digits=2):
    """Format one optional numeric metric for the summary table."""
    return 'N/A' if value is None else ('%.*f' % (digits, value))


def percent_value(metric, digits=1):
    """Format one optional ratio as a percentage."""
    return (
        'N/A' if metric is None
        else ('%.*f%%' % (digits, 100.0 * metric)))


def scaled_value(metric, scale, digits=2):
    """Scale and format one optional numeric metric."""
    return value(None if metric is None else scale * metric, digits)


def render_summary(rows, max_update_rate_hz, random_seed):
    """Render whole-run runtime and autonomy-moving quality separately."""
    runtime_headers = (
        'Profile', 'LaserZcm', 'PFHz', 'ComputeP95', 'AgeP99', 'Stale',
        'Error', 'Runtime',
    )
    runtime_rows = []
    quality_headers = (
        'Profile', 'LaserZcm', 'MuxMatch', 'AutoN', 'AutoFit',
        'AutoFitP05', 'AutoWallP95cm', 'AutoBad', 'AutoCorrP95cm',
        'AutoYawP95deg',
    )
    quality_rows = []
    for row in rows:
        runtime_rows.append((
            row['profile'],
            scaled_value(row.get('base_to_laser_z_m'), 100.0),
            value(row['pf_update_rate_hz']),
            value(row['compute_p95_ms']),
            value(row['correction_age_p99_ms']),
            row['stale_drop'] if row['stale_drop'] is not None else 'N/A',
            row['worker_error'] if row['worker_error'] is not None else 'N/A',
            'PASS' if row['runtime_threshold_pass'] else 'FAIL',
        ))
        quality_rows.append((
            row['profile'],
            scaled_value(row.get('base_to_laser_z_m'), 100.0),
            percent_value(row['mux_matched_fraction']),
            row['autonomy_moving_updates']
            if row['autonomy_moving_updates'] is not None else 'N/A',
            percent_value(row['autonomy_moving_fit_10cm_mean']),
            percent_value(row['autonomy_moving_fit_10cm_p05']),
            scaled_value(
                row['autonomy_moving_wall_p90_p95_m'], 100.0),
            percent_value(row['autonomy_moving_bad_update_fraction']),
            scaled_value(
                row['autonomy_moving_correction_translation_p95_m'],
                100.0),
            value(row['autonomy_moving_correction_yaw_p95_deg']),
        ))

    def table(headers, table_rows):
        all_rows = [headers] + table_rows
        widths = [
            max(len(str(row[index])) for row in all_rows)
            for index in range(len(headers))
        ]

        def line(row):
            return '  '.join(
                str(item).ljust(widths[index])
                for index, item in enumerate(row)).rstrip()

        return '\n'.join((
            line(headers),
            line(tuple('-' * width for width in widths)),
            *(line(row) for row in table_rows),
        ))

    return '\n'.join((
        'WHOLE-REPLAY RUNTIME',
        table(runtime_headers, runtime_rows),
        '',
        'AUTONOMY + MOVING QUALITY',
        table(quality_headers, quality_rows),
        '',
        'Replay override: max_update_rate_hz=%.1f, random_seed=%d'
        % (max_update_rate_hz, random_seed),
        'Runtime target: PFHz >= %.1f, AgeP99 <= 100 ms, stale/error = 0'
        % PASS_UPDATE_RATE_HZ,
        'Quality scope: exact mux source=autonomy, speed >= 0.5 m/s, '
        'command age <= 100 ms',
    ))


def build_commands(
        bag_path, result_dir, base_config, profile_config, map_server_tool,
        playback_rate, clock_hz, replay_delay, max_update_rate_hz,
        random_seed, base_to_laser_z_m=None, start_offset_sec=0.0,
        seed_from_recorded_pf=False):
    """Build argv lists once so dry-run and execution cannot diverge."""
    map_command = [
        sys.executable,
        str(map_server_tool),
        str(bag_path),
    ]
    if base_to_laser_z_m is not None:
        map_command.extend((
            '--base-to-laser-z-m',
            ros_double_literal(base_to_laser_z_m),
        ))
    pf_command = [
        'ros2', 'run', 'particle_filter_cpp', 'particle_filter_cpp_node',
        '--ros-args',
        '--params-file', str(base_config),
        '--params-file', str(profile_config),
        '-p', 'use_sim_time:=true',
        # Preserve the decimal point: ROS would parse "25" as INTEGER and
        # reject it because max_update_rate_hz is declared as DOUBLE.
        '-p', 'max_update_rate_hz:=%s' % ros_double_literal(
            max_update_rate_hz),
        '-p', 'random_seed:=%d' % random_seed,
    ]
    output_bag = result_dir / 'bag'
    record_command = [
        'ros2', 'bag', 'record',
        '-o', str(output_bag),
        '-s', 'sqlite3',
        '--use-sim-time',
        *RECORD_TOPICS,
    ]
    play_command = [
        'ros2', 'bag', 'play', str(bag_path),
        '--rate', str(playback_rate),
        '--clock', str(clock_hz),
        '--delay', str(replay_delay),
        '--read-ahead-queue-size', '5000',
        '--disable-keyboard-controls',
    ]
    if start_offset_sec > 0.0:
        play_command.extend((
            '--start-offset', ros_double_literal(start_offset_sec)))
    play_command.extend((
        '--topics', *replay_topics(
            base_to_laser_z_m, seed_from_recorded_pf)))
    return map_command, pf_command, record_command, play_command


def run_profile(args, common, case, domain_id, session_dir):
    """Execute and finalize one profile; all callers run this serially."""
    profile = case.profile
    run_label = case.label
    result_dir = session_dir / run_label
    result_dir.mkdir(parents=True, exist_ok=False)
    profile_config = common['profiles_dir'] / (profile + '.yaml')
    artifacts = snapshot_runtime_artifacts(
        result_dir, common, profile_config)
    replay_laser_z_m = (
        case.effective_base_to_laser_z_m
        if common['replay_seed'] else case.base_to_laser_z_override_m)
    commands = build_commands(
        common['bag_path'], result_dir,
        artifacts['base_config']['snapshot_path'],
        artifacts['profile_config']['snapshot_path'],
        artifacts['pf_replay_map_server']['snapshot_path'],
        args.playback_rate, args.clock_hz, args.replay_delay_sec,
        args.max_update_rate_hz, args.random_seed,
        replay_laser_z_m,
        common['replay_seed']['start_offset_sec']
        if common['replay_seed'] else 0.0,
        common['replay_seed'] is not None)
    replay_topic_names = replay_topics(
        replay_laser_z_m,
        common['replay_seed'] is not None)
    environment = os.environ.copy()
    ros_log_dir = result_dir / 'ros_logs'
    ros_log_dir.mkdir(parents=True, exist_ok=True)
    environment.update({
        'ROS_DOMAIN_ID': str(domain_id),
        'ROS_LOCALHOST_ONLY': '1',
        'ROS_LOG_DIR': str(ros_log_dir),
        'PYTHONUNBUFFERED': '1',
        'RMW_FASTRTPS_PUBLICATION_MODE': 'ASYNCHRONOUS',
    })
    manifest = {
        'profile': profile,
        'pf_engine': 'cpp',
        'run_label': run_label,
        'git_revision': common['worktree'].manifest['head'],
        'git_dirty': common['worktree'].manifest['dirty'],
        'worktree_fingerprint': common['worktree'].manifest,
        'worktree_evidence_files': common['worktree_evidence_files'],
        'input_bag': str(common['bag_path']),
        'input_bag_identity': common['input_bag_identity'],
        'output_bag': str(result_dir / 'bag'),
        'ros_domain_id': domain_id,
        'base_config': str(common['base_config']),
        'profile_config': str(profile_config),
        'executed_base_config': artifacts['base_config']['snapshot_path'],
        'executed_profile_config': (
            artifacts['profile_config']['snapshot_path']),
        'runtime_artifacts': artifacts,
        'active_pf_runtime': common['pf_runtime'],
        'recorded_base_to_laser_transform': common[
            'recorded_base_to_laser_transform'],
        'base_to_laser_z_override_m': case.base_to_laser_z_override_m,
        'effective_base_to_laser_z_m': (
            case.effective_base_to_laser_z_m),
        'base_to_laser_z_source': case.base_to_laser_z_source,
        'replay_topics': list(replay_topic_names),
        'recorded_pf_seed': common['replay_seed'],
        'record_topics': list(RECORD_TOPICS),
        'playback_rate': args.playback_rate,
        'clock_hz': args.clock_hz,
        'max_update_rate_hz': args.max_update_rate_hz,
        'random_seed': args.random_seed,
        'commands': [shell_join(command) for command in commands],
        'started_utc': datetime.now(timezone.utc).isoformat(),
        'status': 'running',
    }
    write_json(result_dir / 'manifest.json', manifest)
    print('\n[%s] domain=%d output=%s' % (
        run_label, domain_id, result_dir))
    print(
        '[%s] PF replay override: max_update_rate_hz=%.1f, random_seed=%d'
        % (run_label, args.max_update_rate_hz, args.random_seed))
    print(
        '[%s] base_link -> laser z: %.4f m (%s)'
        % (
            run_label,
            case.effective_base_to_laser_z_m,
            case.base_to_laser_z_source,
        ))
    print(
        '[%s] active C++ PF sha256=%s' % (
            run_label,
            artifacts['active_particle_filter']['sha256']))

    map_server = particle_filter = recorder = player = None
    try:
        map_server = start_process(
            'recorded map server', commands[0],
            result_dir / 'map_server.log', environment)
        wait_for_log(
            map_server, result_dir / 'map_server.log',
            'Serving recorded /map', args.startup_timeout_sec)

        particle_filter = start_process(
            'particle_filter', commands[1],
            result_dir / 'particle_filter.log', environment)
        wait_for_log(
            particle_filter, result_dir / 'particle_filter.log',
            PF_READY_LOG + profile, args.startup_timeout_sec)

        recorder = start_process(
            'rosbag recorder', commands[2],
            result_dir / 'record.log', environment)
        time.sleep(args.recorder_discovery_sec)
        if recorder.process.poll() is not None:
            raise RuntimeError(
                'ros2 bag record exited early; see %s'
                % (result_dir / 'record.log'))

        if common['replay_seed'] is not None:
            seed_result = subprocess.run(
                initial_pose_command(common['replay_seed']),
                check=False, text=True, stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT, env=environment, timeout=10.0)
            (result_dir / 'initial_pose.log').write_text(
                seed_result.stdout, encoding='utf-8')
            if seed_result.returncode != 0:
                raise RuntimeError(
                    'recorded PF seed publication failed; see %s'
                    % (result_dir / 'initial_pose.log'))

        player = start_process(
            'rosbag player', commands[3],
            result_dir / 'play.log', environment)
        wait_for_playback(player, particle_filter)
        player = None
        time.sleep(args.post_roll_sec)

        recorder_code = stop_process(recorder, timeout_sec=30.0)
        recorder = None
        if recorder_code not in (0, -signal.SIGINT):
            raise RuntimeError(
                'ros2 bag record finalized with code %s' % recorder_code)
        if not (result_dir / 'bag' / 'metadata.yaml').is_file():
            raise RuntimeError('Output rosbag metadata was not finalized')

        metrics = None
        if not args.skip_analysis:
            print('[%s] analyzing output...' % run_label)
            report = run_analysis(
                artifacts['compare_pf_replay']['snapshot_path'],
                artifacts['analyze_pf_bags']['snapshot_path'],
                common['bag_path'], result_dir / 'bag', result_dir,
                args.settle_sec)
            metrics = metric_summary(
                run_label, report, args.max_update_rate_hz,
                args.random_seed,
                base_to_laser_z_m=case.effective_base_to_laser_z_m,
                base_to_laser_z_source=case.base_to_laser_z_source)

        verified_bag_identity = verify_input_bag_identity(
            common['bag_path'], common['input_bag_identity'])

        manifest.update({
            'finished_utc': datetime.now(timezone.utc).isoformat(),
            'status': 'complete',
            'metrics': metrics,
            'input_bag_identity_verified_after_run': verified_bag_identity,
        })
        write_json(result_dir / 'manifest.json', manifest)
        return metrics
    except BaseException as error:
        manifest.update({
            'finished_utc': datetime.now(timezone.utc).isoformat(),
            'status': 'failed',
            'error': repr(error),
        })
        write_json(result_dir / 'manifest.json', manifest)
        raise
    finally:
        stop_process(player, timeout_sec=5.0)
        stop_process(recorder, timeout_sec=30.0)
        stop_process(particle_filter, timeout_sec=10.0)
        stop_process(map_server, timeout_sec=10.0)
        time.sleep(1.0)


def parse_arguments():
    """Parse replay controls without starting any ROS process."""
    parser = argparse.ArgumentParser(
        description=(
            'Replay one bag through isolated PF A/B profiles without '
            'launching hardware or control'),
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""examples:
  # Validate the installed profiles and exact commands; writes nothing.
  python3 tools/run_pf_ab_replay.py --dry-run

  # Run the production C++ profile.
  python3 tools/run_pf_ab_replay.py --profiles variant0_no_ground_no_deskew

  # Defaults shared by every profile: 25 Hz cap headroom and fixed seed 809.
  python3 tools/run_pf_ab_replay.py --max-update-rate-hz 25 --random-seed 809

  # Isolate scan-to-scan ICP on another bag.
  python3 tools/run_pf_ab_replay.py /path/to/bag \\
    --profiles variant0_no_ground_no_deskew \\
    variant0_no_ground_no_deskew_no_icp

  # Isolate recorded 11.00 cm versus measured 13.74 cm LiDAR scan height.
  python3 tools/run_pf_ab_replay.py --profiles variant0_no_ground_no_deskew \\
    --base-to-laser-z-m 0.1100 0.1374

  # Ignore stale latched /initialpose and start from recorded PF output.
  python3 tools/run_pf_ab_replay.py /path/to/mid-session-bag \
    --seed-from-recorded-pf

Build/source localization_ws first. Results are written below
bags/pf_ab_replay by default. Each profile contains bag/, manifest.json,
process logs, analysis.json, and mode_analysis.json. summary.txt separates
whole-replay runtime acceptance from localization quality measured only while
the original bag's final mux output is fresh autonomy and speed is >=0.5 m/s.
The tool rejects a rate cap below 20 Hz so a strict measured 15 Hz result is
not merely the configured limiter. Production localize.yaml is not modified.
""")
    parser.add_argument(
        'bag', nargs='?', default=str(DEFAULT_BAG),
        help='input rosbag2 directory (default: run_0809_231809)')
    parser.add_argument(
        '--profiles', nargs='+', choices=PROFILES, default=list(PROFILES),
        help='profiles to run serially in the given order (default: all)')
    parser.add_argument(
        '--base-to-laser-z-m', type=float, nargs='+',
        help=(
            'override recorded base_link -> laser static-TF z in metres; '
            'multiple ordered values form a height A/B and require exactly '
            'one PF profile'))
    parser.add_argument(
        '--output-root', default=str(DEFAULT_OUTPUT_ROOT),
        help='parent directory for timestamped replay sessions')
    parser.add_argument(
        '--domain-id', type=int, default=90,
        help='first isolated ROS_DOMAIN_ID; each profile uses the next ID')
    parser.add_argument('--playback-rate', type=float, default=1.0)
    parser.add_argument('--clock-hz', type=float, default=100.0)
    parser.add_argument(
        '--max-update-rate-hz', type=float,
        default=DEFAULT_MAX_UPDATE_RATE_HZ,
        help=(
            'common replay-only PF update cap (default: %.1f Hz; must be at '
            'least %.1f Hz to leave headroom above the %.1f Hz pass target)'
            % (
                DEFAULT_MAX_UPDATE_RATE_HZ,
                PASS_UPDATE_RATE_HZ + MIN_RATE_CAP_HEADROOM_HZ,
                PASS_UPDATE_RATE_HZ)),
    )
    parser.add_argument(
        '--random-seed', type=int, default=DEFAULT_RANDOM_SEED,
        help=(
            'common deterministic PF seed for every profile '
            '(default: %(default)s)'),
    )
    parser.add_argument(
        '--seed-from-recorded-pf', action='store_true',
        help=(
            'ignore recorded /initialpose, publish the first recorded '
            '/pf/pose/odom as the initial pose, and start playback there'))
    parser.add_argument('--replay-delay-sec', type=float, default=2.0)
    parser.add_argument('--recorder-discovery-sec', type=float, default=2.0)
    parser.add_argument('--post-roll-sec', type=float, default=3.0)
    parser.add_argument('--startup-timeout-sec', type=float, default=120.0)
    parser.add_argument('--settle-sec', type=float, default=10.0)
    parser.add_argument(
        '--skip-analysis', action='store_true',
        help='record only; do not run compare_pf_replay.py')
    parser.add_argument(
        '--dry-run', action='store_true',
        help='validate inputs/install and print commands without writing')
    return parser.parse_args()


def main():
    """Validate, run every requested profile, and summarize the results."""
    args = parse_arguments()
    valid_playback_timing = all((
        math.isfinite(args.playback_rate),
        math.isfinite(args.clock_hz),
        args.playback_rate > 0.0,
        args.clock_hz > 0.0,
    ))
    if not valid_playback_timing:
        raise SystemExit('playback rate and clock frequency must be positive')
    minimum_rate_cap = PASS_UPDATE_RATE_HZ + MIN_RATE_CAP_HEADROOM_HZ
    valid_rate_cap = all((
        math.isfinite(args.max_update_rate_hz),
        args.max_update_rate_hz >= minimum_rate_cap,
    ))
    if not valid_rate_cap:
        raise SystemExit(
            '--max-update-rate-hz must be at least %.1f Hz: the pass target '
            'is a measured %.1f Hz, so the configured cap needs %.1f Hz of '
            'headroom' % (
                minimum_rate_cap, PASS_UPDATE_RATE_HZ,
                MIN_RATE_CAP_HEADROOM_HZ))
    if args.random_seed < 0:
        raise SystemExit('--random-seed must be a non-negative integer')
    if args.base_to_laser_z_m is not None and any(
            not math.isfinite(value) or not 0.0 < value <= 2.0
            for value in args.base_to_laser_z_m):
        raise SystemExit(
            '--base-to-laser-z-m values must be finite and in (0, 2] m')
    if min(
            args.replay_delay_sec, args.recorder_discovery_sec,
            args.post_roll_sec, args.startup_timeout_sec,
            args.settle_sec) < 0.0:
        raise SystemExit('time arguments must be non-negative')
    if len(set(args.profiles)) != len(args.profiles):
        raise SystemExit('each profile may be selected only once')
    bag_path = Path(args.bag).expanduser().resolve()
    output_root = Path(args.output_root).expanduser().resolve()
    validate_bag(bag_path)
    replay_seed = (
        first_recorded_pf_seed(bag_path)
        if args.seed_from_recorded_pf else None)
    recorded_static_transforms = load_recorded_static_transforms(bag_path)
    recorded_base_to_laser = find_direct_static_transform(
        recorded_static_transforms)
    recorded_base_to_laser_manifest = static_transform_manifest(
        recorded_base_to_laser)
    recorded_base_to_laser_z_m = recorded_base_to_laser_manifest[
        'translation_xyz_m'][2]
    try:
        cases = build_replay_cases(
            args.profiles,
            args.base_to_laser_z_m,
            recorded_base_to_laser_z_m,
        )
    except ValueError as error:
        raise SystemExit(str(error))
    if args.domain_id < 0 or args.domain_id + len(cases) - 1 > 232:
        raise SystemExit('ROS domain IDs must remain between 0 and 232')
    input_bag_identity = capture_input_bag_identity(bag_path)
    share = package_share('particle_filter_cpp')
    base_config = share / 'config' / 'localize.yaml'
    profiles_dir = share / 'config' / 'profiles'
    missing = [
        path for path in (
            base_config,
            *(profiles_dir / (profile + '.yaml') for profile in args.profiles),
        ) if not path.is_file()
    ]
    if missing:
        raise RuntimeError(
            'Installed PF configs are incomplete; rebuild/source '
            'localization_ws:\n  %s'
            % '\n  '.join(str(path) for path in missing))
    worktree = capture_worktree_evidence()
    pf_runtime = resolve_active_cpp_runtime(share.parent.parent)

    stamp = datetime.now().strftime('%Y%m%d_%H%M%S')
    session_dir = output_root / ('%s__%s' % (bag_path.name, stamp))
    common = {
        'bag_path': bag_path,
        'base_config': base_config,
        'profiles_dir': profiles_dir,
        'worktree': worktree,
        'pf_runtime': pf_runtime,
        'pf_engine': 'cpp',
        'input_bag_identity': input_bag_identity,
        'recorded_base_to_laser_transform': (
            recorded_base_to_laser_manifest),
        'replay_seed': replay_seed,
    }

    print('PF A/B replay preflight OK')
    print('  input : %s' % bag_path)
    print(
        '  bag ID: %s (%d files)'
        % (
            input_bag_identity['fingerprint_sha256'],
            len(input_bag_identity['files']),
        ))
    print('  output: %s' % session_dir)
    print('  map   : recorded /map via /map_server/map (no local YAML)')
    print(
        '  TF    : recorded base_link -> laser z=%.4f m%s'
        % (
            recorded_base_to_laser_z_m,
            (
                '; ordered override=' + ' -> '.join(
                    '%.4f m' % case.effective_base_to_laser_z_m
                    for case in cases)
                if args.base_to_laser_z_m is not None else ''),
        ))
    print('  safety: isolated local domain; no hardware/control launch')
    print('          stop any existing vehicle ROS stack before execution')
    print(
        '  PF args: max_update_rate_hz=%.1f (> %.1f target), random_seed=%d'
        % (
            args.max_update_rate_hz, PASS_UPDATE_RATE_HZ,
            args.random_seed))
    if replay_seed is not None:
        print(
            '  seed   : recorded PF pose at +%.3fs (%.3f, %.3f)'
            % (
                replay_seed['start_offset_sec'], replay_seed['x'],
                replay_seed['y']))
    print(
        '  source : HEAD %s, dirty=%s, fingerprint=%s'
        % (
            worktree.manifest['head'],
            worktree.manifest['dirty'],
            worktree.manifest['fingerprint_sha256']))
    print(
        '  PF code: %s engine=cpp (sha256=%s, %s)'
        % (
            pf_runtime['active_particle_filter_path'],
            pf_runtime['active_particle_filter_sha256'],
            pf_runtime['python_tag']))
    print('  order : %s' % ' -> '.join(case.label for case in cases))

    if args.dry_run:
        for index, case in enumerate(cases):
            profile = case.profile
            result_dir = session_dir / case.label
            profile_config = profiles_dir / (profile + '.yaml')
            snapshot_dir = result_dir / 'snapshot'
            commands = build_commands(
                bag_path, result_dir,
                snapshot_dir / 'config' / 'localize.yaml',
                snapshot_dir / 'config' / 'profile.yaml',
                snapshot_dir / 'tools' / 'pf_replay_map_server.py',
                args.playback_rate, args.clock_hz, args.replay_delay_sec,
                args.max_update_rate_hz, args.random_seed,
                case.effective_base_to_laser_z_m
                if replay_seed else case.base_to_laser_z_override_m,
                replay_seed['start_offset_sec'] if replay_seed else 0.0,
                replay_seed is not None)
            print('\n[%s] ROS_DOMAIN_ID=%d' % (
                case.label, args.domain_id + index))
            print(
                '  base_link -> laser z      : %.4f m (%s)'
                % (
                    case.effective_base_to_laser_z_m,
                    case.base_to_laser_z_source,
                ))
            print('  snapshot source base   : %s' % base_config)
            print('  snapshot source profile: %s' % profile_config)
            for command in commands:
                print('  ' + shell_join(command))
            if replay_seed is not None:
                print('  ' + shell_join(initial_pose_command(replay_seed)))
        return 0

    session_dir.mkdir(parents=True, exist_ok=False)
    worktree_files = persist_worktree_evidence(session_dir, worktree)
    common['worktree_evidence_files'] = worktree_files
    session_manifest = {
        'input_bag': str(bag_path),
        'input_bag_identity': input_bag_identity,
        'git_revision': worktree.manifest['head'],
        'git_dirty': worktree.manifest['dirty'],
        'worktree_fingerprint': worktree.manifest,
        'worktree_evidence_files': worktree_files,
        'active_pf_runtime': pf_runtime,
        'pf_engine': 'cpp',
        'profiles': list(args.profiles),
        'recorded_base_to_laser_transform': (
            recorded_base_to_laser_manifest),
        'base_to_laser_z_values_requested_m': args.base_to_laser_z_m,
        'cases': [
            {
                'label': case.label,
                'profile': case.profile,
                'base_to_laser_z_override_m': (
                    case.base_to_laser_z_override_m),
                'effective_base_to_laser_z_m': (
                    case.effective_base_to_laser_z_m),
                'base_to_laser_z_source': case.base_to_laser_z_source,
            }
            for case in cases
        ],
        'max_update_rate_hz': args.max_update_rate_hz,
        'random_seed': args.random_seed,
        'recorded_pf_seed': replay_seed,
        'started_utc': datetime.now(timezone.utc).isoformat(),
        'status': 'running',
    }
    write_json(session_dir / 'manifest.json', session_manifest)
    rows = []
    failures = []
    try:
        for index, case in enumerate(cases):
            try:
                verify_input_bag_identity(bag_path, input_bag_identity)
                metrics = run_profile(
                    args, common, case, args.domain_id + index,
                    session_dir)
                verify_input_bag_identity(bag_path, input_bag_identity)
                if metrics is not None:
                    rows.append(metrics)
            except KeyboardInterrupt:
                raise
            except Exception as error:
                failures.append({
                    'profile': case.profile,
                    'run_label': case.label,
                    'error': repr(error),
                })
                print(
                    '[%s] FAILED: %s' % (case.label, error),
                    file=sys.stderr)
        session_end_bag_identity = None
        input_bag_unchanged = False
        input_bag_capture_error = None
        try:
            session_end_bag_identity = capture_input_bag_identity(bag_path)
            input_bag_unchanged = (
                session_end_bag_identity['fingerprint_sha256']
                == input_bag_identity['fingerprint_sha256'])
        except Exception as error:
            input_bag_capture_error = repr(error)
        if input_bag_capture_error is not None:
            failures.append({
                'profile': 'session',
                'error': (
                    'Could not verify input bag after the A/B session: %s'
                    % input_bag_capture_error),
            })
        elif not input_bag_unchanged:
            failures.append({
                'profile': 'session',
                'error': 'Input bag content changed during the A/B session',
            })
        if rows:
            summary_text = render_summary(
                rows, args.max_update_rate_hz, args.random_seed)
            (session_dir / 'summary.txt').write_text(
                summary_text + '\n', encoding='utf-8')
            write_json(session_dir / 'summary.json', rows)
            print('\n' + summary_text)
        session_manifest.update({
            'finished_utc': datetime.now(timezone.utc).isoformat(),
            'status': 'complete' if not failures else 'failed',
            'failures': failures,
            'summary': rows,
            'input_bag_identity_after_run': session_end_bag_identity,
            'input_bag_unchanged': input_bag_unchanged,
            'input_bag_identity_capture_error': input_bag_capture_error,
        })
        write_json(session_dir / 'manifest.json', session_manifest)
        print('\nResults: %s' % session_dir)
        return 1 if failures else 0
    except KeyboardInterrupt:
        session_manifest.update({
            'finished_utc': datetime.now(timezone.utc).isoformat(),
            'status': 'interrupted',
            'summary': rows,
        })
        write_json(session_dir / 'manifest.json', session_manifest)
        print(
            '\nInterrupted safely; finalized results remain in %s'
            % session_dir)
        return 130


if __name__ == '__main__':
    try:
        raise SystemExit(main())
    except RuntimeError as error:
        print('ERROR: %s' % error, file=sys.stderr)
        raise SystemExit(1)
