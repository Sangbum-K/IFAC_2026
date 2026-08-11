#!/usr/bin/env python3
"""Evaluate the shared 3D scan gate directly on recorded real-car bags.

⚠️ 2026-08-11 현재 이 스크립트는 그냥 돌지 않는다. localization_upgrade 를 main 에
머지할 때 perception 은 main 의 C++ 재작성본을 남기기로 했고, 이 스크립트가 쓰던
파이썬 perception_pkg(preprocessing / clustering)은 그때 사라졌다. 아래 임포트가
그래서 SystemExit 로 막힌다 — 고장이 아니라 의도된 상태다.

살리려면 둘 중 하나:
  1) 파이썬 perception 을 꺼내 옆에 둔다
     git show origin/localization_upgrade:perception_ws/src/perception_pkg/perception_pkg/preprocessing.py
     git show origin/localization_upgrade:perception_ws/src/perception_pkg/perception_pkg/clustering.py
  2) 지면필터/deskew 를 C++ obstacle_detector 로 포팅한 뒤 그쪽에 맞춰 다시 쓴다.

f1tenth_scan_geometry(지면필터·deskew 본체)는 머지에 그대로 남아 있으므로,
그 부분만 쓰는 분석은 이 스크립트 없이도 가능하다.
"""

import argparse
from dataclasses import dataclass, field
from pathlib import Path
import sys

import numpy as np


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'planning_ws/src/f1tenth_scan_geometry'))
sys.path.insert(0, str(ROOT / 'perception_ws/src/perception_pkg'))
sys.path.insert(0, str(ROOT / 'tools'))

from compare_pf_replay import read_bag  # noqa: E402
from compare_pf_replay import stamp_ns  # noqa: E402
from compare_pf_replay import text_table  # noqa: E402
from f1tenth_scan_geometry import base_orientation_from_imu  # noqa: E402
from f1tenth_scan_geometry import GroundFilterConfig  # noqa: E402
from f1tenth_scan_geometry import LevelReference  # noqa: E402
from f1tenth_scan_geometry import mask_ground_and_offplane  # noqa: E402
from f1tenth_scan_geometry import quaternion_from_rpy  # noqa: E402
from f1tenth_scan_geometry import quaternion_to_matrix  # noqa: E402
from f1tenth_scan_geometry.geometry import (  # noqa: E402
    level_relative_matrices,
    quaternion_conjugate,
    roll_pitch_from_matrices,
    slerp_quaternions,
)
try:
    from perception_pkg.preprocessing import sanitize_scan_ranges  # noqa: E402
    from perception_pkg.preprocessing import preprocess_scan  # noqa: E402
    from perception_pkg.clustering import (  # noqa: E402
        adaptive_component_provenance,
    )
except ModuleNotFoundError as exc:  # 위 docstring 참고 — 의도된 상태다
    raise SystemExit(
        f'{exc}\n\n'
        '파이썬 perception_pkg 는 2026-08-11 머지에서 제거됐다 '
        '(main 은 C++ obstacle_detector 를 쓴다). 이 스크립트를 되살리는 '
        '두 가지 방법은 파일 상단 docstring 에 적어 뒀다.') from exc


TOPICS = {'/scan', '/odom', '/sensors/imu/raw', '/tf_static'}
GRAVITY = 9.80665
GYRO_SCALE = np.pi / 180.0
PF_MAX_RANGE_M = 10.0


def resolve_bag(path):
    candidate = Path(path)
    if candidate.is_file():
        return candidate
    if candidate.is_dir():
        db3_files = sorted(candidate.glob('*.db3'))
        if len(db3_files) == 1:
            return candidate
    raise ValueError(f'{path} must be a bag directory or one .db3 file')


def quaternion_array(message):
    return np.array([
        message.x, message.y, message.z, message.w,
    ], dtype=np.float64)


def rpy_from_quaternion(quaternion):
    matrix = quaternion_to_matrix(quaternion)
    pitch = np.arctan2(
        -matrix[2, 0], np.hypot(matrix[0, 0], matrix[1, 0]))
    roll = np.arctan2(matrix[2, 1], matrix[2, 2])
    yaw = np.arctan2(matrix[1, 0], matrix[0, 0])
    return np.array([roll, pitch, yaw], dtype=np.float64)


def invert_transform(translation, quaternion):
    rotation = quaternion_to_matrix(quaternion)
    inverse_q = quaternion_conjugate(quaternion)
    return -(rotation.T @ translation), inverse_q


def recorded_laser_extrinsic(records):
    for _, message in records:
        for item in message.transforms:
            parent = item.header.frame_id.strip().lstrip('/')
            child = item.child_frame_id.strip().lstrip('/')
            translation = np.array([
                item.transform.translation.x,
                item.transform.translation.y,
                item.transform.translation.z,
            ], dtype=np.float64)
            quaternion = quaternion_array(item.transform.rotation)
            if parent == 'base_link' and child == 'laser':
                return translation, quaternion
            if parent == 'laser' and child == 'base_link':
                return invert_transform(translation, quaternion)
    raise RuntimeError('base_link <-> laser is missing from /tf_static')


class AttitudeInterpolator:
    def __init__(self, records, base_to_imu_quaternion):
        samples = {}
        frame_ids = set()
        for record_stamp, message in records:
            timestamp = stamp_ns(message.header.stamp) or int(record_stamp)
            imu_q = quaternion_array(message.orientation)
            norm = np.linalg.norm(imu_q)
            if not np.isfinite(norm) or norm < 1e-9:
                continue
            try:
                base_q = base_orientation_from_imu(
                    imu_q / norm, base_to_imu_quaternion)
            except ValueError:
                continue
            values = np.array([
                message.angular_velocity.x,
                message.angular_velocity.y,
                message.angular_velocity.z,
                message.linear_acceleration.x,
                message.linear_acceleration.y,
                message.linear_acceleration.z,
            ], dtype=np.float64)
            if not np.all(np.isfinite(values)):
                continue
            samples[timestamp] = (base_q, values)
            if message.header.frame_id.strip():
                frame_ids.add(message.header.frame_id.strip())
        self.stamps = np.array(sorted(samples), dtype=np.int64)
        self.quaternions = np.asarray(
            [samples[timestamp][0] for timestamp in self.stamps])
        self.values = np.asarray(
            [samples[timestamp][1] for timestamp in self.stamps])
        self.frame_ids = sorted(frame_ids)

    def at_many(self, timestamps, max_gap_sec=0.08):
        requested = np.asarray(timestamps, dtype=np.int64)
        if (requested.ndim != 1 or len(self.stamps) < 2
                or not len(requested)
                or requested.min() < self.stamps[0]
                or requested.max() > self.stamps[-1]):
            return None
        upper = np.searchsorted(self.stamps, requested, side='left')
        exact = self.stamps[upper] == requested
        lower = np.where(exact, upper, upper - 1)
        gaps = self.stamps[upper] - self.stamps[lower]
        if np.any((~exact) & (gaps > int(max_gap_sec * 1e9))):
            return None
        ratios = np.zeros(len(requested), dtype=np.float64)
        moving = ~exact
        ratios[moving] = (
            (requested[moving] - self.stamps[lower[moving]])
            / gaps[moving].astype(np.float64)
        )
        return slerp_quaternions(
            self.quaternions[lower], self.quaternions[upper], ratios)


def odom_speed_series(records):
    samples = {}
    for record_stamp, message in records:
        timestamp = stamp_ns(message.header.stamp) or int(record_stamp)
        speed = float(message.twist.twist.linear.x)
        if np.isfinite(speed):
            samples[timestamp] = speed
    stamps = np.array(sorted(samples), dtype=np.int64)
    speeds = np.array([samples[timestamp] for timestamp in stamps])
    return stamps, speeds


def estimate_level(attitudes, odom_records):
    odom_stamps, odom_speeds = odom_speed_series(odom_records)
    if len(odom_stamps) < 2:
        return None, None
    reference = LevelReference(
        min_samples=20,
        max_gyro_rad_s=0.12,
        gravity_mps2=GRAVITY,
        accel_tolerance_mps2=2.5,
    )
    speeds = np.interp(
        attitudes.stamps.astype(np.float64),
        odom_stamps.astype(np.float64), odom_speeds)
    for timestamp, quaternion, values, speed in zip(
            attitudes.stamps, attitudes.quaternions,
            attitudes.values, speeds):
        ready = reference.update(
            quaternion,
            np.linalg.norm(values[:3]) * GYRO_SCALE,
            np.linalg.norm(values[3:]) * GRAVITY,
            wheel_speed=speed,
        )
        if ready:
            return np.copy(reference.quaternion), int(timestamp)
    return None, None


def percentile_angles(values):
    degrees = np.degrees(np.abs(np.asarray(values, dtype=np.float64)))
    if not len(degrees):
        return {'p50': 0.0, 'p95': 0.0, 'p99': 0.0, 'max': 0.0}
    return {
        'p50': float(np.percentile(degrees, 50)),
        'p95': float(np.percentile(degrees, 95)),
        'p99': float(np.percentile(degrees, 99)),
        'max': float(np.max(degrees)),
    }


def attitude_diagnostics(attitudes, level_quaternion, odom_records):
    odom_stamps, odom_speeds = odom_speed_series(odom_records)
    speeds = np.interp(
        attitudes.stamps.astype(np.float64),
        odom_stamps.astype(np.float64), odom_speeds)
    relative = level_relative_matrices(
        attitudes.quaternions, level_quaternion)
    rolls, pitches = roll_pitch_from_matrices(relative)
    tilts = np.arccos(np.clip(relative[:, 2, 2], -1.0, 1.0))
    output = {}
    for name, mask in (
            ('all', np.ones(len(speeds), dtype=bool)),
            ('stationary', np.abs(speeds) <= 0.10),
            ('moving', np.abs(speeds) >= 0.50)):
        output[name] = {
            'samples': int(np.count_nonzero(mask)),
            'roll': percentile_angles(rolls[mask]),
            'pitch': percentile_angles(pitches[mask]),
            'tilt': percentile_angles(tilts[mask]),
        }
    return output


def sensor_inspections(attitudes, level_quaternion, odom_records,
                       scan_start, inspect_secs):
    odom_stamps, odom_speeds = odom_speed_series(odom_records)
    rows = []
    for target in inspect_secs:
        requested = scan_start + int(round(target * 1e9))
        index = int(np.argmin(np.abs(attitudes.stamps - requested)))
        timestamp = int(attitudes.stamps[index])
        relative = level_relative_matrices(
            attitudes.quaternions[index:index + 1], level_quaternion)
        roll, pitch = roll_pitch_from_matrices(relative)
        gyro = attitudes.values[index, :3] * GYRO_SCALE
        accel = attitudes.values[index, 3:] * GRAVITY
        speed = float(np.interp(
            timestamp, odom_stamps.astype(np.float64), odom_speeds))
        rows.append((
            float(target), (timestamp - scan_start) / 1e9, speed,
            float(np.degrees(roll[0])), float(np.degrees(pitch[0])),
            float(np.linalg.norm(gyro[:2])), float(gyro[2]),
            float(np.linalg.norm(accel[:2])), float(np.linalg.norm(accel)),
        ))
    return rows


def scan_stamp_and_duration(record_stamp, message):
    timestamp = stamp_ns(message.header.stamp) or int(record_stamp)
    duration = ((len(message.ranges) - 1) * float(message.time_increment)
                if message.time_increment > 0.0 else 0.0)
    if not 0.0 < duration <= 0.05:
        duration = 0.0
    return timestamp, duration


def pf_scan(message):
    ranges = np.asarray(message.ranges, dtype=np.float64)
    indices = np.arange(len(ranges), dtype=np.int64)
    selected_positions = np.arange(0, len(ranges), 15, dtype=np.int64)
    sensor_min = max(0.0, float(message.range_min))
    sensor_max = (float(message.range_max)
                  if np.isfinite(message.range_max) else PF_MAX_RANGE_M)
    mcl_valid = (
        np.isfinite(ranges)
        & (ranges >= sensor_min)
        & (ranges > 0.0)
        & (ranges <= sensor_max)
        & (ranges < 65.5)
        & (ranges < PF_MAX_RANGE_M * 0.995)
    )
    geometry_ranges = np.where(mcl_valid, ranges, np.nan)
    angles = (float(message.angle_min)
              + indices * float(message.angle_increment))
    return (geometry_ranges, angles, indices, mcl_valid[selected_positions],
            selected_positions)


def perception_scan(message):
    ranges, angles = sanitize_scan_ranges(
        message.ranges, message.angle_min, message.angle_increment,
        range_min=0.1, range_max=12.0,
        sensor_range_max=message.range_max,
    )
    fov = (
        (angles >= np.deg2rad(-80.0))
        & (angles <= np.deg2rad(80.0))
    )
    indices = np.flatnonzero(fov).astype(np.int64)
    selected = ranges[fov]
    return (selected, angles[fov], indices, np.isfinite(selected),
            np.arange(len(selected), dtype=np.int64))


def ground_config(profile):
    return GroundFilterConfig(
        ground_z_m=0.0,
        range_tolerance_m=0.08,
        range_tolerance_ratio=0.04,
        endpoint_z_tolerance_m=0.05,
        min_component_beams=6,
        min_component_angle_rad=np.deg2rad(4.0),
        min_component_span_m=0.30,
        max_planar_elevation_rad=np.deg2rad(4.0),
        degraded_offplane_fraction=0.55,
        degraded_ground_fraction=0.70,
        attitude_fallback_tilt_rad=np.deg2rad(4.0),
        attitude_ground_evidence_fraction=0.05,
        attitude_ground_evidence_min_beams=6,
        attitude_evidence_max_abs_angle_rad=np.deg2rad(80.0),
        degraded_valid_fraction=0.25,
        min_retained_beams=20,
    )


def maximum_run(mask, beam_indices):
    positions = np.flatnonzero(mask)
    if not len(positions):
        return 0
    nominal_step = (max(1, int(np.median(np.diff(beam_indices))))
                    if len(beam_indices) > 1 else 1)
    gaps = np.flatnonzero(
        (np.diff(positions) > 1)
        | (np.diff(beam_indices[positions])
           > nominal_step)) + 1
    return max(len(run) for run in np.split(positions, gaps))


@dataclass
class Accumulator:
    synced_frames: int = 0
    ground_frames: int = 0
    offplane_frames: int = 0
    degraded_frames: int = 0
    ground_dominated_frames: int = 0
    attitude_fallback_frames: int = 0
    ground_counts: list = field(default_factory=list)
    offplane_counts: list = field(default_factory=list)
    retained_counts: list = field(default_factory=list)
    top_events: list = field(default_factory=list)
    current_degraded_run: int = 0
    maximum_degraded_run: int = 0
    degraded_run_start_sec: float = 0.0
    maximum_degraded_span_sec: float = 0.0
    maximum_degraded_interval: tuple = ()
    current_degraded_reasons: set = field(default_factory=set)
    maximum_degraded_reasons: tuple = ()
    inspections: dict = field(default_factory=dict)

    def add(self, relative_sec, ground, valid_returns, profile, beam_indices,
            projection, inspect_secs=()):
        ground_mask = ground.ground_mask[projection]
        offplane_mask = ground.offplane_mask[projection]
        required = ground.keep_mask[projection]
        selected_indices = beam_indices[projection]
        ground_count = int(np.count_nonzero(ground_mask))
        offplane_count = int(np.count_nonzero(offplane_mask))
        degraded = ground.degraded
        if profile == 'PF':
            required = required | (~valid_returns & ~offplane_mask)
            retained = int(np.count_nonzero(required))
            if (ground.reason == 'too_few_retained'
                    and retained >= ground_config(profile).min_retained_beams):
                degraded = False
            if retained < ground_config(profile).min_retained_beams:
                degraded = True
        retained = int(np.count_nonzero(required))
        self.synced_frames += 1
        self.ground_frames += int(ground_count > 0)
        self.offplane_frames += int(offplane_count > 0)
        self.degraded_frames += int(degraded)
        if degraded and not self.current_degraded_run:
            self.degraded_run_start_sec = float(relative_sec)
            self.current_degraded_reasons.clear()
        if degraded:
            self.current_degraded_reasons.add(ground.reason)
        self.current_degraded_run = (
            self.current_degraded_run + 1 if degraded else 0)
        if degraded:
            span = float(relative_sec) - self.degraded_run_start_sec
            if (self.current_degraded_run > self.maximum_degraded_run
                    or span > self.maximum_degraded_span_sec):
                self.maximum_degraded_run = self.current_degraded_run
                self.maximum_degraded_span_sec = span
                self.maximum_degraded_interval = (
                    self.degraded_run_start_sec, float(relative_sec))
                self.maximum_degraded_reasons = tuple(sorted(
                    self.current_degraded_reasons))
        self.ground_dominated_frames += int(
            ground.reason == 'ground_dominated')
        self.attitude_fallback_frames += int(ground.attitude_fallback)
        self.ground_counts.append(ground_count)
        self.offplane_counts.append(offplane_count)
        self.retained_counts.append(retained)
        event = (
            ground_count, float(relative_sec),
            maximum_run(ground_mask, selected_indices), ground.reason,
        )
        self.top_events.append(event)
        self.top_events.sort(reverse=True)
        del self.top_events[5:]
        for target in inspect_secs:
            delta = abs(relative_sec - target)
            current = self.inspections.get(target)
            if current is None or delta < current[0]:
                self.inspections[target] = (
                    delta, float(relative_sec), ground_count, offplane_count,
                    retained, bool(degraded), ground.reason,
                    bool(ground.attitude_fallback),
                )

    def summary(self):
        total = max(self.synced_frames, 1)
        if not self.ground_counts:
            return {
                'synced': 0,
                'ground_frame_fraction': 0.0,
                'offplane_frame_fraction': 0.0,
                'degraded_fraction': 0.0,
                'ground_dominated_fraction': 0.0,
                'attitude_fallback_fraction': 0.0,
                'ground_p50': 0.0,
                'ground_p95': 0.0,
                'ground_max': 0,
                'offplane_p95': 0.0,
                'retained_p05': 0.0,
                'maximum_degraded_run': 0,
                'maximum_degraded_span_sec': 0.0,
                'maximum_degraded_interval': (),
                'maximum_degraded_reasons': (),
                'top_events': [],
                'inspections': [],
            }
        return {
            'synced': self.synced_frames,
            'ground_frame_fraction': self.ground_frames / total,
            'offplane_frame_fraction': self.offplane_frames / total,
            'degraded_fraction': self.degraded_frames / total,
            'ground_dominated_fraction': self.ground_dominated_frames / total,
            'attitude_fallback_fraction': (
                self.attitude_fallback_frames / total),
            'ground_p50': float(np.percentile(self.ground_counts, 50)),
            'ground_p95': float(np.percentile(self.ground_counts, 95)),
            'ground_max': int(max(self.ground_counts, default=0)),
            'offplane_p95': float(np.percentile(self.offplane_counts, 95)),
            'retained_p05': float(np.percentile(self.retained_counts, 5)),
            'maximum_degraded_run': self.maximum_degraded_run,
            'maximum_degraded_span_sec': self.maximum_degraded_span_sec,
            'maximum_degraded_interval': self.maximum_degraded_interval,
            'maximum_degraded_reasons': self.maximum_degraded_reasons,
            'top_events': list(self.top_events),
            'inspections': [
                (target,) + values[1:]
                for target, values in sorted(self.inspections.items())
            ],
        }


def component_diagnostics(message):
    points, ranges, indices, _ = preprocess_scan(
        message.ranges, message.angle_min, message.angle_increment,
        range_min=0.1, range_max=12.0,
        fov_min=np.deg2rad(-80.0), fov_max=np.deg2rad(80.0),
        median_kernel=3, sensor_range_max=message.range_max,
        return_metadata=True,
    )
    _, radii, sizes = adaptive_component_provenance(
        points, ranges, message.angle_increment,
        lambda_angle=np.deg2rad(10.0), sigma=0.015,
        beam_indices=indices)
    if not len(radii):
        return 0, 0.0, 0, 0
    largest = int(np.argmax(radii))
    return (
        len(radii), float(radii[largest]), int(sizes[largest]),
        int(np.count_nonzero(radii > 0.35)),
    )


def analyze(path, pitches_deg, sample_hz, imu_rpy_deg, inspect_secs=()):
    bag_path = resolve_bag(path)
    data = read_bag(bag_path, TOPICS)
    if not data['/scan'] or not data['/sensors/imu/raw'] or not data['/odom']:
        raise RuntimeError(f'{path} is missing scan, IMU, or odom data')

    laser_translation, recorded_laser_q = recorded_laser_extrinsic(
        data['/tf_static'])
    recorded_rpy = rpy_from_quaternion(recorded_laser_q)
    imu_q = quaternion_from_rpy(*np.deg2rad(imu_rpy_deg))
    attitudes = AttitudeInterpolator(data['/sensors/imu/raw'], imu_q)
    level_q, level_stamp = estimate_level(attitudes, data['/odom'])
    if level_q is None:
        raise RuntimeError(f'{path}: no 20-sample stationary IMU level reference')

    laser_quaternions = {
        pitch: quaternion_from_rpy(
            recorded_rpy[0], np.deg2rad(pitch), recorded_rpy[2])
        for pitch in pitches_deg
    }
    profiles = {'PF': pf_scan, 'Perception': perception_scan}
    accumulators = {
        (profile, pitch): Accumulator()
        for profile in profiles for pitch in pitches_deg
    }
    scan_start = scan_stamp_and_duration(*data['/scan'][0])[0]
    minimum_period_ns = int(1e9 / sample_hz) if sample_hz > 0.0 else 0
    last_sample_ns = -minimum_period_ns
    sampled = 0
    sync_misses = 0
    component_inspections = {}

    for record_stamp, message in data['/scan']:
        timestamp, duration = scan_stamp_and_duration(record_stamp, message)
        if timestamp - last_sample_ns < minimum_period_ns:
            continue
        last_sample_ns = timestamp
        sampled += 1
        reference_stamp = timestamp + int(round(0.5 * duration * 1e9))
        relative_sec = (reference_stamp - scan_start) / 1e9
        for target in inspect_secs:
            delta = abs(relative_sec - target)
            current = component_inspections.get(target)
            if current is None or delta < current[0]:
                component_inspections[target] = (
                    delta, relative_sec, *component_diagnostics(message))
        for profile, adapter in profiles.items():
            ranges, angles, indices, valid_returns, projection = adapter(message)
            if not len(ranges):
                continue
            beam_stamps = (
                timestamp
                + np.rint(indices * message.time_increment * 1e9).astype(np.int64)
                if duration > 0.0
                else np.full(len(indices), reference_stamp, dtype=np.int64)
            )
            base_quaternions = attitudes.at_many(beam_stamps)
            if base_quaternions is None:
                sync_misses += 1
                continue
            for pitch in pitches_deg:
                result = mask_ground_and_offplane(
                    ranges, angles, indices, base_quaternions, level_q,
                    laser_translation, laser_quaternions[pitch],
                    ground_config(profile),
                )
                accumulators[(profile, pitch)].add(
                    relative_sec, result, valid_returns, profile, indices,
                    projection, inspect_secs)

    return {
        'name': bag_path.name if bag_path.is_dir() else bag_path.stem,
        'scan_count': len(data['/scan']),
        'sampled': sampled,
        'sync_misses': sync_misses,
        'imu_frames': attitudes.frame_ids,
        'level_time_sec': (level_stamp - scan_start) / 1e9,
        'laser_xyz': laser_translation,
        'recorded_laser_rpy_deg': np.degrees(recorded_rpy),
        'attitude': attitude_diagnostics(
            attitudes, level_q, data['/odom']),
        'sensor_inspections': sensor_inspections(
            attitudes, level_q, data['/odom'], scan_start, inspect_secs),
        'component_inspections': [
            (target,) + values[1:]
            for target, values in sorted(component_inspections.items())
        ],
        'profiles': {
            profile: {
                pitch: accumulators[(profile, pitch)].summary()
                for pitch in pitches_deg
            }
            for profile in profiles
        },
    }


def percent(value):
    return f'{100.0 * value:.1f}%'


def render(result):
    rows = []
    for profile, pitches in result['profiles'].items():
        for pitch, summary in pitches.items():
            rows.append((
                profile, f'{pitch:+.2f}', summary['synced'],
                percent(summary['ground_frame_fraction']),
                f"{summary['ground_p50']:.0f}/{summary['ground_p95']:.0f}/"
                f"{summary['ground_max']}",
                percent(summary['offplane_frame_fraction']),
                f"{summary['offplane_p95']:.0f}",
                percent(summary['attitude_fallback_fraction']),
                percent(summary['degraded_fraction']),
                '%d/%.2fs' % (
                    summary['maximum_degraded_run'],
                    summary['maximum_degraded_span_sec']),
                f"{summary['retained_p05']:.0f}",
            ))
    event_rows = []
    degraded_rows = []
    for profile, pitches in result['profiles'].items():
        for pitch, summary in pitches.items():
            interval = summary['maximum_degraded_interval']
            if interval:
                degraded_rows.append((
                    profile, f'{pitch:+.2f}',
                    f'{interval[0]:.2f}', f'{interval[1]:.2f}',
                    summary['maximum_degraded_run'],
                    f"{summary['maximum_degraded_span_sec']:.2f}",
                    ','.join(summary['maximum_degraded_reasons']),
                ))
            for count, time_sec, run, reason in summary['top_events'][:2]:
                if count:
                    event_rows.append((
                        profile, f'{pitch:+.2f}', f'{time_sec:.2f}',
                        count, run, reason,
                    ))
    inspection_rows = []
    for profile, pitches in result['profiles'].items():
        for pitch, summary in pitches.items():
            for values in summary['inspections']:
                (target, actual, ground_count, offplane_count, retained,
                 degraded, reason, fallback) = values
                inspection_rows.append((
                    profile, f'{pitch:+.2f}', f'{target:.2f}',
                    f'{actual:.2f}', ground_count, offplane_count, retained,
                    str(int(degraded)), reason, str(int(fallback)),
                ))
    component_rows = [
        (f'{target:.2f}', f'{actual:.2f}', count, f'{radius:.3f}', size,
         over_limit)
        for target, actual, count, radius, size, over_limit
        in result['component_inspections']
    ]
    sensor_rows = [
        (f'{target:.2f}', f'{actual:.2f}', f'{speed:.2f}',
         f'{roll:.2f}', f'{pitch:.2f}', f'{gyro_xy:.3f}', f'{gyro_z:.3f}',
         f'{accel_xy:.2f}', f'{accel_norm:.2f}')
        for (target, actual, speed, roll, pitch, gyro_xy, gyro_z,
             accel_xy, accel_norm) in result['sensor_inspections']
    ]
    recorded_rpy = result['recorded_laser_rpy_deg']
    xyz = result['laser_xyz']
    imu_frame = ','.join(result['imu_frames']) or '(empty in bag)'
    attitude_rows = []
    for state, values in result['attitude'].items():
        attitude_rows.append((
            state, values['samples'],
            '%.1f/%.1f/%.1f' % (
                values['roll']['p50'], values['roll']['p95'],
                values['roll']['p99']),
            '%.1f/%.1f/%.1f' % (
                values['pitch']['p50'], values['pitch']['p95'],
                values['pitch']['p99']),
            '%.1f/%.1f/%.1f' % (
                values['tilt']['p50'], values['tilt']['p95'],
                values['tilt']['p99']),
        ))
    return '\n'.join((
        f"SCAN GEOMETRY BAG CHECK: {result['name']}",
        f"scans={result['scan_count']} sampled={result['sampled']} "
        f"profile_sync_miss={result['sync_misses']}",
        f"level_ready_at={result['level_time_sec']:.2f}s IMU_frame={imu_frame}",
        'recorded base<-laser xyz=(%.3f, %.3f, %.3f)m rpy=(%.2f, %.2f, %.2f)deg'
        % (*xyz, *recorded_rpy),
        '',
        'RAW AHRS RELATIVE ATTITUDE (p50/p95/p99 deg)',
        text_table(
            ('State', 'N', 'AbsRoll', 'AbsPitch', 'Tilt'), attitude_rows),
        '',
        'Pitch sweep is diagnostic only; enter a physically measured TF value.',
        text_table(
            ('Profile', 'PitchDeg', 'N', 'GroundFrames', 'Ground p50/p95/max',
             'OffplaneFrames', 'OffP95', 'AHRSFallback', 'Degraded',
             'MaxDropRun', 'RetainedP05'),
            rows,
        ),
        '',
        'LONGEST DEGRADED-SCAN INTERVALS',
        text_table(
            ('Profile', 'PitchDeg', 'StartSec', 'EndSec', 'Frames', 'SpanSec',
             'Reasons'),
            degraded_rows,
        ),
        '',
        'TOP EXPECTED-GROUND MATCHES',
        text_table(
            ('Profile', 'PitchDeg', 'BagSec', 'Beams', 'LongestRun', 'Reason'),
            event_rows,
        ),
        '',
        'REQUESTED-TIME INSPECTION',
        text_table(
            ('Profile', 'PitchDeg', 'Requested', 'Actual', 'Ground', 'Offplane',
             'Retained', 'Drop', 'Reason', 'Fallback'),
            inspection_rows,
        ),
        '',
        'RAW PERCEPTION COMPONENTS BEFORE MAP/TRACK FILTERS',
        text_table(
            ('Requested', 'Actual', 'Components', 'LargestRadius',
             'LargestSize', 'Radius>0.35m'),
            component_rows,
        ),
        '',
        'REQUESTED-TIME IMU/ODOM (SI units after VESC scaling)',
        text_table(
            ('Requested', 'Actual', 'Speed', 'RollDeg', 'PitchDeg',
             'GyroXY', 'GyroZ', 'AccelXY', 'AccelNorm'),
            sensor_rows,
        ),
    ))


def parse_csv_floats(value, expected=None):
    try:
        output = tuple(float(item.strip()) for item in value.split(','))
    except ValueError as error:
        raise argparse.ArgumentTypeError(str(error)) from error
    if not output or not np.all(np.isfinite(output)):
        raise argparse.ArgumentTypeError('values must be finite')
    if expected is not None and len(output) != expected:
        raise argparse.ArgumentTypeError(f'expected {expected} comma-separated values')
    return output


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('bags', nargs='+')
    parser.add_argument(
        '--pitch-deg', type=parse_csv_floats,
        default=parse_csv_floats('0,0.5,1.0,1.25,1.5,1.75,2.0'))
    parser.add_argument('--sample-hz', type=float, default=5.0)
    parser.add_argument(
        '--base-to-imu-rpy-deg',
        type=lambda value: parse_csv_floats(value, expected=3),
        default=(0.0, 0.0, 0.0))
    parser.add_argument(
        '--inspect-sec', type=parse_csv_floats, default=())
    args = parser.parse_args()
    if not 0.0 <= args.sample_hz <= 40.0:
        parser.error('--sample-hz must be 0 (all scans) or in (0, 40]')
    for bag in args.bags:
        print(render(analyze(
            bag, args.pitch_deg, args.sample_hz,
            args.base_to_imu_rpy_deg, args.inspect_sec)))


if __name__ == '__main__':
    main()
