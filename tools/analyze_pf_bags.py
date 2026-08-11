#!/usr/bin/env python3
"""Summarize localization health across one or more real ROS 2 bags."""

import argparse
import json
import math
from pathlib import Path

from compare_pf_replay import OdomInterpolator
from compare_pf_replay import TOPICS as PF_TOPICS
from compare_pf_replay import base_to_laser_pose
from compare_pf_replay import compose
from compare_pf_replay import correction_metrics
from compare_pf_replay import inverse
from compare_pf_replay import map_fit_metrics
from compare_pf_replay import nearest_by_stamp
from compare_pf_replay import new_update_poses
from compare_pf_replay import old_update_poses
from compare_pf_replay import output_timing
from compare_pf_replay import pose_2d
from compare_pf_replay import read_bag
from compare_pf_replay import replay_diagnostics
from compare_pf_replay import stamp_ns
from compare_pf_replay import text_table
from compare_pf_replay import update_timing

import numpy as np

from scipy.ndimage import distance_transform_edt


MUX_OUTPUT_TOPIC = '/ackermann_cmd'
MUX_SOURCE_TOPICS = {
    '/autonomy/ackermann_cmd': 'autonomy',
    '/teleop': 'manual',
    '/estop_cmd': 'estop',
}
COMMAND_TOPICS = {MUX_OUTPUT_TOPIC, *MUX_SOURCE_TOPICS}
COMMAND_MAX_AGE_SEC = 0.10


def resolve_bag(path):
    """Resolve a bag directory containing exactly one db3 file."""
    candidate = Path(path)
    if candidate.is_file():
        return candidate
    db3_files = sorted(candidate.glob('*.db3'))
    if len(db3_files) != 1:
        raise ValueError(f'{path} must contain exactly one db3 file')
    return db3_files[0]


def message_rate(records):
    """Calculate a header-timestamp message rate."""
    timestamps = np.array([
        stamp_ns(message.header.stamp) for _, message in records
    ], dtype=np.int64)
    if timestamps.size < 2:
        return 0.0
    timestamps = np.sort(timestamps[timestamps > 0])
    span = (timestamps[-1] - timestamps[0]) / 1e9
    return float((len(timestamps) - 1) / span) if span > 0.0 else 0.0


def nested(mapping, *keys, default=None):
    """Read a nested metric without obscuring missing bag data."""
    value = mapping
    for key in keys:
        if not isinstance(value, dict) or key not in value:
            return default
        value = value[key]
    return value


def ackermann_key(message):
    """Build an exact key preserved by ackermann_mux's message copy."""
    drive = message.drive
    return (
        stamp_ns(message.header.stamp),
        message.header.frame_id,
        float(drive.steering_angle),
        float(drive.steering_angle_velocity),
        float(drive.speed),
        float(drive.acceleration),
        float(drive.jerk),
    )


def classify_mux_outputs(data):
    """Identify which mux input produced each recorded vehicle command."""
    key_sources = {}
    for topic, source in MUX_SOURCE_TOPICS.items():
        for _, message in data.get(topic, []):
            key_sources.setdefault(ackermann_key(message), set()).add(source)

    outputs = []
    counts = {
        'autonomy': 0,
        'manual': 0,
        'estop': 0,
        'ambiguous': 0,
        'unknown': 0,
    }
    for record_stamp, message in data.get(MUX_OUTPUT_TOPIC, []):
        sources = key_sources.get(ackermann_key(message), set())
        if len(sources) == 1:
            source = next(iter(sources))
        elif sources:
            source = 'ambiguous'
        else:
            source = 'unknown'
        counts[source] += 1
        outputs.append((int(record_stamp), source))
    outputs.sort(key=lambda item: item[0])
    total = len(outputs)
    matched = counts['autonomy'] + counts['manual'] + counts['estop']
    return outputs, {
        'outputs': total,
        'matched': matched,
        'matched_fraction': matched / total if total else None,
        'source_counts': counts,
    }


def autonomous_moving_fit_summary(
        samples, mux_outputs, minimum_speed=0.5,
        maximum_command_age_sec=COMMAND_MAX_AGE_SEC):
    """Select moving samples whose final mux output came from autonomy."""
    if not mux_outputs:
        return {}, []
    output_stamps = np.array(
        [item[0] for item in mux_outputs], dtype=np.int64)
    maximum_age_ns = int(maximum_command_age_sec * 1e9)
    autonomous = []
    for sample in samples:
        timestamp = sample['timestamp_ns']
        index = int(
            np.searchsorted(output_stamps, timestamp, side='right') - 1)
        if index < 0:
            continue
        age_ns = timestamp - mux_outputs[index][0]
        if not 0 <= age_ns <= maximum_age_ns:
            continue
        if mux_outputs[index][1] == 'autonomy':
            autonomous.append(sample)
    return moving_fit_summary(autonomous, minimum_speed)


def map_fit_series(
        valid_updates, scan_records, map_message, settle_ns, laser_pose,
        excluded_angle_steps, held_out_stride, scan_adapter=None):
    """Calculate pre-correction held-out map fit for each PF update."""
    scan_items = []
    for _, scan in scan_records:
        if scan_adapter is None:
            timestamp = stamp_ns(scan.header.stamp)
            ranges = np.asarray(scan.ranges, dtype=np.float32)
            angles = (
                scan.angle_min
                + np.arange(len(ranges)) * scan.angle_increment)
        else:
            timestamp, ranges, angles = scan_adapter(scan)
        beam_indices = np.arange(len(ranges), dtype=np.int64)
        steps = (
            (excluded_angle_steps,)
            if isinstance(excluded_angle_steps, int)
            else tuple(excluded_angle_steps)
        )
        held_out = np.ones(len(ranges), dtype=bool)
        for angle_step in steps:
            held_out &= beam_indices % angle_step != 0
        selected = beam_indices[held_out][::held_out_stride]
        scan_items.append((
            timestamp,
            ranges[selected],
            angles[selected],
            float(scan.range_min),
            float(scan.range_max),
        ))
    scan_items.sort(key=lambda item: item[0])
    scan_stamps = np.array([item[0] for item in scan_items], dtype=np.int64)

    width = map_message.info.width
    height = map_message.info.height
    resolution = map_message.info.resolution
    occupied = np.asarray(map_message.data).reshape(height, width) >= 50
    wall_distance = distance_transform_edt(~occupied) * resolution
    inverse_origin = inverse(pose_2d(map_message.info.origin))
    samples = []

    for index, update in enumerate(valid_updates):
        timestamp, _, odom_to_base, _ = update
        if timestamp < settle_ns or index == 0:
            continue
        predicted_pose = compose(valid_updates[index - 1][3], odom_to_base)
        scan_index = nearest_by_stamp(scan_stamps, timestamp)
        scan_timestamp, ranges, angles, range_min, range_max = scan_items[
            scan_index]
        maximum_hit_range = min(range_max * 0.995, 9.95)
        valid = np.logical_and.reduce((
            np.isfinite(ranges),
            ranges > max(range_min, 0.05),
            ranges < maximum_hit_range,
        ))
        ranges = ranges[valid]
        angles = angles[valid]
        if ranges.size == 0:
            continue

        map_to_laser = compose(predicted_pose, laser_pose)
        endpoint_x = map_to_laser[0] + ranges * np.cos(
            map_to_laser[2] + angles)
        endpoint_y = map_to_laser[1] + ranges * np.sin(
            map_to_laser[2] + angles)
        cosine = math.cos(inverse_origin[2])
        sine = math.sin(inverse_origin[2])
        local_x = (
            inverse_origin[0] + cosine * endpoint_x - sine * endpoint_y)
        local_y = (
            inverse_origin[1] + sine * endpoint_x + cosine * endpoint_y)
        columns = np.floor(local_x / resolution).astype(np.int64)
        rows = np.floor(local_y / resolution).astype(np.int64)
        inside = np.logical_and.reduce((
            columns >= 0, columns < width, rows >= 0, rows < height,
        ))
        distances = wall_distance[rows[inside], columns[inside]]
        if distances.size < 5:
            continue

        previous_timestamp = valid_updates[index - 1][0]
        previous_odom = valid_updates[index - 1][2]
        delta_time = max((timestamp - previous_timestamp) / 1e9, 1e-6)
        odom_delta = compose(inverse(previous_odom), odom_to_base)
        correction = compose(inverse(predicted_pose), update[1])
        samples.append({
            'timestamp_ns': int(timestamp),
            'time_sec': float((timestamp - settle_ns) / 1e9),
            'scan_time_error_ms': abs(scan_timestamp - timestamp) / 1e6,
            'x': float(predicted_pose[0]),
            'y': float(predicted_pose[1]),
            'yaw_deg': float(np.degrees(predicted_pose[2])),
            'speed_mps': float(np.hypot(odom_delta[0], odom_delta[1])
                               / delta_time),
            'within_0.10m_fraction': float(np.mean(distances <= 0.10)),
            'wall_distance_p90_m': float(np.percentile(distances, 90)),
            'wall_distance_p95_m': float(np.percentile(distances, 95)),
            'outside_fraction': float(1.0 - np.mean(inside)),
            'correction_m': float(np.hypot(correction[0], correction[1])),
            'correction_yaw_deg': float(abs(np.degrees(correction[2]))),
        })
    return samples


def bad_intervals(samples):
    """Group persistent severe map-fit failures into time intervals."""
    groups = []
    current = []
    for sample in samples:
        bad = (
            sample['within_0.10m_fraction'] < 0.75
            or sample['wall_distance_p90_m'] > 0.20
        )
        contiguous = (
            current
            and sample['time_sec'] - current[-1]['time_sec'] <= 0.30
        )
        if bad:
            if current and not contiguous:
                groups.append(current)
                current = []
            current.append(sample)
        elif current:
            groups.append(current)
            current = []
    if current:
        groups.append(current)

    intervals = []
    for group in groups:
        worst = min(group, key=lambda item: item['within_0.10m_fraction'])
        intervals.append({
            'start_sec': group[0]['time_sec'],
            'end_sec': group[-1]['time_sec'],
            'duration_sec': group[-1]['time_sec'] - group[0]['time_sec'],
            'updates': len(group),
            'worst_x': worst['x'],
            'worst_y': worst['y'],
            'minimum_fit_fraction': min(
                item['within_0.10m_fraction'] for item in group),
            'maximum_wall_p90_m': max(
                item['wall_distance_p90_m'] for item in group),
            'median_speed_mps': float(np.median([
                item['speed_mps'] for item in group
            ])),
            'maximum_correction_m': max(
                item['correction_m'] for item in group),
            'maximum_correction_yaw_deg': max(
                item['correction_yaw_deg'] for item in group),
        })
    return sorted(
        intervals,
        key=lambda item: (
            item['minimum_fit_fraction'],
            -item['maximum_wall_p90_m'],
        ),
    )


def moving_fit_summary(samples, minimum_speed=0.5):
    """Summarize map fit only while the car is actually driving."""
    moving = [
        sample for sample in samples
        if sample['speed_mps'] >= minimum_speed
    ]
    if not moving:
        return {}, []
    fits = np.array([
        sample['within_0.10m_fraction'] for sample in moving
    ])
    wall_p90 = np.array([
        sample['wall_distance_p90_m'] for sample in moving
    ])
    bad = np.logical_or(fits < 0.75, wall_p90 > 0.20)
    return {
        'updates': len(moving),
        'fit_10cm_mean': float(np.mean(fits)),
        'fit_10cm_p05': float(np.percentile(fits, 5)),
        'wall_p90_p95_m': float(np.percentile(wall_p90, 95)),
        'correction_translation_p95_m': float(np.percentile([
            sample['correction_m'] for sample in moving
        ], 95)),
        'correction_yaw_p95_deg': float(np.percentile([
            sample['correction_yaw_deg'] for sample in moving
        ], 95)),
        'bad_update_fraction': float(np.mean(bad)),
    }, moving


def analyze(
        path, settle_sec, angle_step, held_out_stride,
        command_path=None):
    """Calculate no-ground-truth PF metrics for one recorded run.

    ``command_path`` may name the original driving bag when ``path`` is a PF
    replay output.  PF geometry and runtime then come from ``path``, while the
    exact ackermann_mux source classification comes from the immutable input
    bag.  Matching by the original ROS timestamps avoids replay scheduling or
    recorder latency changing the driving-mode label.
    """
    bag_path = resolve_bag(path)
    data_topics = PF_TOPICS if command_path else PF_TOPICS | COMMAND_TOPICS
    data = read_bag(bag_path, topics=data_topics)
    if command_path is None:
        command_bag_path = bag_path
        command_data = data
    else:
        command_bag_path = resolve_bag(command_path)
        command_data = read_bag(
            command_bag_path, topics=COMMAND_TOPICS)
    odom = OdomInterpolator(data['/odom'])
    settle_ns = int(odom.stamps[0] + settle_sec * 1e9)

    updates = new_update_poses(data)
    update_stats = update_timing(updates, data['/scan'])
    direct_match = (
        update_stats.get('exact_source_scan_stamps', 0)
        / max(update_stats.get('count', 0), 1)
    )
    timestamp_mode = 'direct'
    if direct_match < 0.90 and data['/pf/viz/fake_scan']:
        updates, _ = old_update_poses(data)
        update_stats = update_timing(updates, data['/scan'])
        timestamp_mode = 'fake-scan recovered'

    corrections, valid_updates = correction_metrics(
        updates, odom, settle_ns)
    map_message = data['/map'][0][1]
    laser_pose = base_to_laser_pose(data)
    excluded_steps = (angle_step,)
    held_out = map_fit_metrics(
        valid_updates,
        data['/scan'],
        map_message,
        settle_ns,
        laser_pose,
        predicted=True,
        excluded_angle_steps=excluded_steps,
        held_out_stride=held_out_stride,
    )
    post_fit = map_fit_metrics(
        valid_updates,
        data['/scan'],
        map_message,
        settle_ns,
        laser_pose,
        predicted=False,
        excluded_angle_steps=excluded_steps,
        held_out_stride=held_out_stride,
    )
    diagnostics = replay_diagnostics(data['/rosout'])
    counters = diagnostics['counters']
    offered = counters.get('offered', 0)
    fit_samples = map_fit_series(
        valid_updates,
        data['/scan'],
        map_message,
        settle_ns,
        laser_pose,
        (angle_step,),
        held_out_stride,
    )
    intervals = bad_intervals(fit_samples)
    moving_summary, moving_samples = moving_fit_summary(fit_samples)
    moving_intervals = bad_intervals(moving_samples)
    mux_outputs, mux_summary = classify_mux_outputs(command_data)
    autonomy_summary, autonomy_samples = autonomous_moving_fit_summary(
        fit_samples, mux_outputs)
    autonomy_intervals = bad_intervals(autonomy_samples)

    return {
        'name': bag_path.parent.name,
        'path': str(bag_path),
        'command_path': str(command_bag_path),
        'timestamp_mode': timestamp_mode,
        'scan_rate_hz': message_rate(data['/scan']),
        'odom_rate_hz': message_rate(data['/odom']),
        'pf_update_rate_hz': update_stats.get('rate_hz'),
        'scan_stamp_match_fraction': (
            update_stats.get('exact_source_scan_stamps', 0)
            / max(update_stats.get('count', 0), 1)
        ),
        'held_out_pre': held_out,
        'held_out_post': post_fit,
        'corrections': corrections,
        'output': output_timing(data['/pf/pose/odom'], data['/odom']),
        'runtime': diagnostics['timing'],
        'counters': counters,
        'overwrite_fraction': (
            counters.get('overwritten', 0) / offered if offered else None
        ),
        'jump_rejections': diagnostics['logged_jump_rejections'],
        'bad_update_fraction': (
            np.mean([
                sample['within_0.10m_fraction'] < 0.75
                or sample['wall_distance_p90_m'] > 0.20
                for sample in fit_samples
            ]) if fit_samples else None
        ),
        'worst_intervals': intervals[:10],
        'moving_fit': moving_summary,
        'moving_worst_intervals': moving_intervals[:10],
        'mux': mux_summary,
        'autonomy_moving_fit': autonomy_summary,
        'autonomy_moving_worst_intervals': autonomy_intervals[:10],
    }


def percent(value):
    """Format an optional ratio as a percentage."""
    return 'N/A' if value is None else f'{100.0 * value:.1f}%'


def number(value, digits=1):
    """Format an optional numeric metric."""
    return 'N/A' if value is None else f'{value:.{digits}f}'


def scaled_number(value, scale=100.0, digits=1):
    """Scale and format a metric while preserving missing values."""
    return number(None if value is None else scale * value, digits)


def render(results):
    """Render a compact cross-bag localization table."""
    quality_rows = []
    runtime_rows = []
    interval_rows = []
    for result in results:
        quality_rows.append((
            result['name'],
            number(result['scan_rate_hz']),
            number(result['pf_update_rate_hz']),
            percent(result['scan_stamp_match_fraction']),
            percent(nested(result, 'mux', 'matched_fraction')),
            str(nested(
                result, 'autonomy_moving_fit', 'updates', default=0)),
            percent(nested(
                result, 'autonomy_moving_fit', 'fit_10cm_mean')),
            percent(nested(
                result, 'autonomy_moving_fit', 'fit_10cm_p05')),
            scaled_number(nested(
                result, 'autonomy_moving_fit', 'wall_p90_p95_m')),
            percent(nested(
                result, 'autonomy_moving_fit', 'bad_update_fraction')),
            scaled_number(nested(
                result, 'autonomy_moving_fit',
                'correction_translation_p95_m')),
            number(nested(
                result, 'autonomy_moving_fit',
                'correction_yaw_p95_deg')),
        ))
        runtime_rows.append((
            result['name'],
            number(nested(result, 'runtime', 'compute', 'p95')),
            number(nested(result, 'runtime', 'turnaround', 'p95')),
            number(nested(result, 'runtime', 'age', 'p99')),
            percent(result['overwrite_fraction']),
            result['counters'].get('sync_drop', 'N/A'),
            result['counters'].get('stale_drop', 'N/A'),
            result['counters'].get('worker_error', 'N/A'),
            result['jump_rejections'],
        ))
        for interval in result['autonomy_moving_worst_intervals'][:3]:
            interval_rows.append((
                result['name'],
                number(interval['start_sec']),
                number(interval['duration_sec']),
                interval['updates'],
                f"{interval['worst_x']:.2f},{interval['worst_y']:.2f}",
                percent(interval['minimum_fit_fraction']),
                number(100.0 * interval['maximum_wall_p90_m']),
                number(interval['median_speed_mps']),
                number(100.0 * interval['maximum_correction_m']),
                number(interval['maximum_correction_yaw_deg']),
            ))

    return '\n'.join((
        'REAL-BAG PF LOCALIZATION SUMMARY',
        '',
        text_table(
            ('Run', 'ScanHz', 'PFHz', 'Stamp', 'MuxMatch', 'AutoN',
             'AutoFit', 'AutoFitP05', 'AutoWallP95cm', 'AutoBad',
             'AutoCorrP95cm', 'AutoYawP95deg'),
            quality_rows,
        ),
        '',
        text_table(
            ('Run', 'ComputeP95', 'TurnP95', 'AgeP99', 'Overwrite',
             'SyncDrop', 'Stale', 'Error', 'JumpReject'),
            runtime_rows,
        ),
        '',
        'WORST AUTONOMOUS-MOVING HELD-OUT MAP-FIT INTERVALS',
        text_table(
            ('Run', 'Start', 'Dur', 'N', 'WorstXY', 'MinFit', 'MaxP90cm',
             'Speed', 'MaxCorrCm', 'MaxYawDeg'),
            interval_rows,
        ),
    ))


def main():
    """Analyze supplied real-driving bags."""
    parser = argparse.ArgumentParser()
    parser.add_argument('bags', nargs='+')
    parser.add_argument('--settle-sec', type=float, default=10.0)
    parser.add_argument('--angle-step', type=int, default=10)
    parser.add_argument('--held-out-stride', type=int, default=10)
    parser.add_argument(
        '--command-bag',
        help=(
            'read /ackermann_cmd and mux source topics from this original '
            'driving bag while analyzing replay output bags'),
    )
    parser.add_argument('--json', action='store_true')
    parser.add_argument('--output', help='also write the rendered report')
    args = parser.parse_args()
    if args.settle_sec < 0.0:
        parser.error('--settle-sec must be non-negative')
    if args.angle_step <= 1 or args.held_out_stride <= 0:
        parser.error('angle step must exceed one and stride must be positive')

    results = [
        analyze(
            path,
            args.settle_sec,
            args.angle_step,
            args.held_out_stride,
            command_path=args.command_bag,
        )
        for path in args.bags
    ]
    report = json.dumps(results, indent=2) if args.json else render(results)
    print(report)
    if args.output:
        Path(args.output).write_text(report + '\n', encoding='utf-8')


if __name__ == '__main__':
    main()
