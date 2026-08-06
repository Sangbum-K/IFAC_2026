#!/usr/bin/env python3
"""Compare recorded particle-filter output before and after a bag replay."""

import argparse
import json
import math
import re
from pathlib import Path

import numpy as np

from rclpy.serialization import deserialize_message

import rosbag2_py

from rosidl_runtime_py.utilities import get_message

from scipy.ndimage import distance_transform_edt


TOPICS = {
    '/map',
    '/odom',
    '/pf/pose/odom',
    '/pf/viz/fake_scan',
    '/pf/viz/inferred_pose',
    '/rosout',
    '/scan',
    '/tf_static',
}

SYNC_PATTERN = re.compile(
    r'^PF sync: (?P<rate>[\d.]+)Hz, compute (?P<compute>[\d.]+)ms, '
    r'scan turnaround (?P<turnaround>[\d.]+)ms, correction-age '
    r'(?P<age>-?[\d.]+)ms, CPU-core (?P<cpu>[\d.]+)%,'
)
COUNTERS_PATTERN = re.compile(
    r'scans (?P<processed>\d+)/(?P<offered>\d+), latest-overwrite '
    r'(?P<overwritten>\d+) \((?P<overwrite_percent>\d+)%\), '
    r'sync/stale/order-drop (?P<sync_drop>\d+)/(?P<stale_drop>\d+)/'
    r'(?P<order_drop>\d+), worker-error (?P<worker_error>\d+), '
    r'odom-buffer (?P<buffer_used>\d+)/(?P<buffer_capacity>\d+)'
)


def stamp_ns(stamp):
    """Convert a ROS time message to integer nanoseconds."""
    return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)


def yaw_from_quaternion(quaternion):
    """Extract planar yaw from a quaternion message."""
    sin_yaw = 2.0 * (
        quaternion.w * quaternion.z + quaternion.x * quaternion.y)
    cos_yaw = 1.0 - 2.0 * (
        quaternion.y * quaternion.y + quaternion.z * quaternion.z)
    return math.atan2(sin_yaw, cos_yaw)


def pose_2d(pose):
    """Convert a ROS pose to an x, y, yaw NumPy array."""
    return np.array([
        pose.position.x,
        pose.position.y,
        yaw_from_quaternion(pose.orientation),
    ], dtype=np.float64)


def transform_2d(transform):
    """Convert a ROS transform to an x, y, yaw NumPy array."""
    return np.array([
        transform.translation.x,
        transform.translation.y,
        yaw_from_quaternion(transform.rotation),
    ], dtype=np.float64)


def wrap_angle(angle):
    """Wrap one angle to the [-pi, pi] interval."""
    return np.arctan2(np.sin(angle), np.cos(angle))


def compose(lhs, rhs):
    """Compose two planar rigid transforms."""
    cosine = math.cos(lhs[2])
    sine = math.sin(lhs[2])
    return np.array([
        lhs[0] + cosine * rhs[0] - sine * rhs[1],
        lhs[1] + sine * rhs[0] + cosine * rhs[1],
        wrap_angle(lhs[2] + rhs[2]),
    ])


def inverse(pose):
    """Invert one planar rigid transform."""
    cosine = math.cos(pose[2])
    sine = math.sin(pose[2])
    return np.array([
        -cosine * pose[0] - sine * pose[1],
        sine * pose[0] - cosine * pose[1],
        wrap_angle(-pose[2]),
    ])


def storage_id(path):
    """Infer the rosbag storage plugin from the supplied path."""
    bag_path = Path(path)
    if bag_path.suffix == '.db3':
        return 'sqlite3'
    if bag_path.suffix == '.mcap':
        return 'mcap'
    if bag_path.is_dir():
        if any(bag_path.glob('*.db3')):
            return 'sqlite3'
        if any(bag_path.glob('*.mcap')):
            return 'mcap'
    raise ValueError(
        f'Cannot infer rosbag storage for {path}; expected db3 or mcap data')


def read_bag(path):
    """Read only the topics needed for PF comparison."""
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(path), storage_id=storage_id(path)),
        rosbag2_py.ConverterOptions('', ''),
    )
    topic_types = {
        item.name: item.type for item in reader.get_all_topics_and_types()
    }
    message_types = {
        topic: get_message(topic_types[topic])
        for topic in TOPICS if topic in topic_types
    }
    output = {topic: [] for topic in TOPICS}

    while reader.has_next():
        topic, raw_data, record_stamp = reader.read_next()
        if topic not in message_types:
            continue
        message = deserialize_message(raw_data, message_types[topic])
        output[topic].append((record_stamp, message))
    return output


def stamped_poses(records, odometry=False):
    """Extract record time, header time, and planar pose tuples."""
    output = []
    for record_stamp, message in records:
        pose = message.pose.pose if odometry else message.pose
        output.append((
            record_stamp,
            stamp_ns(message.header.stamp),
            pose_2d(pose),
        ))
    return output


class OdomInterpolator:
    """Interpolate source odometry at arbitrary scan timestamps."""

    def __init__(self, records):
        """Build a sorted interpolation table from odometry messages."""
        poses = stamped_poses(records, odometry=True)
        poses.sort(key=lambda item: item[1])
        unique = {}
        for _, timestamp, pose in poses:
            unique[timestamp] = pose
        self.stamps = np.array(sorted(unique), dtype=np.int64)
        self.poses = np.array([unique[timestamp] for timestamp in self.stamps])
        self.unwrapped_yaw = np.unwrap(self.poses[:, 2])

    def at(self, timestamp):
        """Return a linearly interpolated pose, or None outside the table."""
        upper = int(np.searchsorted(self.stamps, timestamp, side='left'))
        if upper == 0:
            if timestamp == self.stamps[0]:
                return self.poses[0].copy()
            return None
        if upper == len(self.stamps):
            return None
        if self.stamps[upper] == timestamp:
            return self.poses[upper].copy()
        lower = upper - 1
        ratio = ((timestamp - self.stamps[lower]) /
                 float(self.stamps[upper] - self.stamps[lower]))
        result = self.poses[lower] + ratio * (
            self.poses[upper] - self.poses[lower])
        result[2] = wrap_angle(
            self.unwrapped_yaw[lower] + ratio * (
                self.unwrapped_yaw[upper] - self.unwrapped_yaw[lower]))
        return result


def nearest_record_index(records, record_stamp):
    """Find the record whose receipt time is closest to a target."""
    stamps = np.array([item[0] for item in records], dtype=np.int64)
    upper = int(np.searchsorted(stamps, record_stamp, side='left'))
    choices = [
        index for index in (upper - 1, upper)
        if 0 <= index < len(stamps)
    ]
    return min(
        choices,
        key=lambda index: abs(int(stamps[index]) - record_stamp),
    )


def old_update_poses(data):
    """Recover old PF state times from the paired fake-scan messages."""
    inferred = stamped_poses(data['/pf/viz/inferred_pose'])
    fake_scans = data['/pf/viz/fake_scan']
    updates = []
    pairing_error_ms = []
    for record_stamp, _, pose in inferred:
        index = nearest_record_index(fake_scans, record_stamp)
        fake_record_stamp, fake_scan = fake_scans[index]
        updates.append((stamp_ns(fake_scan.header.stamp), pose))
        pairing_error_ms.append(abs(fake_record_stamp - record_stamp) / 1e6)
    return updates, pairing_error_ms


def new_update_poses(data):
    """Read scan-stamped update poses produced by the synchronized PF."""
    return [
        (timestamp, pose)
        for _, timestamp, pose in stamped_poses(data['/pf/viz/inferred_pose'])
    ]


def base_to_laser_pose(data):
    """Read the base_link-to-laser extrinsic from the recorded static TF."""
    for _, message in data['/tf_static']:
        for transform in message.transforms:
            parent = transform.header.frame_id.lstrip('/')
            child = transform.child_frame_id.lstrip('/')
            pose = transform_2d(transform.transform)
            if parent == 'base_link' and child == 'laser':
                return pose
            if parent == 'laser' and child == 'base_link':
                return inverse(pose)
    raise RuntimeError('base_link <-> laser transform not found in /tf_static')


def percentile_summary(values, angular=False):
    """Return robust distribution statistics for JSON output."""
    values = np.asarray(values, dtype=np.float64)
    if values.size == 0:
        return {}
    if angular:
        values = np.degrees(values)
    return {
        'median': float(np.median(values)),
        'p95': float(np.percentile(values, 95)),
        'p99': float(np.percentile(values, 99)),
        'max': float(np.max(values)),
    }


def lag_one_correlation(values):
    """Measure whether consecutive corrections reverse direction."""
    values = np.asarray(values, dtype=np.float64)
    if (values.size < 3 or np.std(values[:-1]) < 1e-12
            or np.std(values[1:]) < 1e-12):
        return None
    return float(np.corrcoef(values[:-1], values[1:])[0, 1])


def alternation_ratio(values, threshold):
    """Return fraction of meaningful consecutive values with flipped sign."""
    values = np.asarray(values, dtype=np.float64)
    previous = values[:-1]
    current = values[1:]
    valid = np.logical_and(
        np.abs(previous) >= threshold,
        np.abs(current) >= threshold,
    )
    if not np.any(valid):
        return None
    return float(np.mean(previous[valid] * current[valid] < 0.0))


def correction_metrics(updates, odom, settle_ns):
    """Recover per-update map corrections after odometry propagation."""
    valid_updates = []
    for timestamp, map_to_base in updates:
        odom_to_base = odom.at(timestamp)
        if odom_to_base is None:
            continue
        map_to_odom = compose(map_to_base, inverse(odom_to_base))
        valid_updates.append((
            timestamp,
            map_to_base,
            odom_to_base,
            map_to_odom,
        ))

    corrections = []
    for previous, current in zip(valid_updates[:-1], valid_updates[1:]):
        timestamp, map_to_base, odom_to_base, _ = current
        if timestamp < settle_ns:
            continue
        predicted = compose(previous[3], odom_to_base)
        corrections.append(compose(inverse(predicted), map_to_base))
    corrections = np.array(corrections)
    if corrections.size == 0:
        return {}, valid_updates

    translation = np.hypot(corrections[:, 0], corrections[:, 1])
    yaw = np.abs(corrections[:, 2])
    return {
        'count': int(len(corrections)),
        'translation_m': percentile_summary(translation),
        'yaw_deg': percentile_summary(yaw, angular=True),
        'lag1': {
            'longitudinal': lag_one_correlation(corrections[:, 0]),
            'lateral': lag_one_correlation(corrections[:, 1]),
            'yaw': lag_one_correlation(corrections[:, 2]),
        },
        'alternating_fraction': {
            'longitudinal': alternation_ratio(corrections[:, 0], 0.002),
            'lateral': alternation_ratio(corrections[:, 1], 0.002),
            'yaw': alternation_ratio(corrections[:, 2], math.radians(0.1)),
        },
    }, valid_updates


def output_timing(records, source_odom):
    """Summarize timestamp correctness and rate of propagated PF output."""
    poses = stamped_poses(records, odometry=True)
    timestamps = np.array([item[1] for item in poses], dtype=np.int64)
    source_stamps = {
        stamp_ns(message.header.stamp) for _, message in source_odom
    }
    if timestamps.size < 2:
        return {'count': int(timestamps.size)}
    positive_span = (timestamps[-1] - timestamps[0]) / 1e9
    return {
        'count': int(timestamps.size),
        'rate_hz': float((len(timestamps) - 1) / positive_span),
        'unique_stamps': int(len(np.unique(timestamps))),
        'adjacent_duplicate_stamps': int(np.sum(np.diff(timestamps) == 0)),
        'non_monotonic_stamps': int(np.sum(np.diff(timestamps) < 0)),
        'exact_source_odom_stamps': int(sum(
            int(timestamp) in source_stamps for timestamp in timestamps)),
    }


def update_timing(updates, source_scans):
    """Summarize whether PF updates retain their source scan timestamps."""
    timestamps = np.array([item[0] for item in updates], dtype=np.int64)
    source_stamps = {
        stamp_ns(message.header.stamp) for _, message in source_scans
    }
    if timestamps.size < 2:
        return {'count': int(timestamps.size)}
    span_sec = (timestamps[-1] - timestamps[0]) / 1e9
    return {
        'count': int(timestamps.size),
        'rate_hz': float((len(timestamps) - 1) / span_sec),
        'unique_stamps': int(len(np.unique(timestamps))),
        'adjacent_duplicate_stamps': int(np.sum(np.diff(timestamps) == 0)),
        'non_monotonic_stamps': int(np.sum(np.diff(timestamps) < 0)),
        'exact_source_scan_stamps': int(sum(
            int(timestamp) in source_stamps for timestamp in timestamps)),
    }


def nearest_by_stamp(stamps, timestamp):
    """Find the timestamp nearest to a target."""
    upper = int(np.searchsorted(stamps, timestamp, side='left'))
    choices = [
        index for index in (upper - 1, upper)
        if 0 <= index < len(stamps)
    ]
    return min(choices, key=lambda index: abs(int(stamps[index]) - timestamp))


def map_fit_metrics(
        valid_updates, scan_records, map_message, settle_ns,
        laser_pose, predicted=False, beam_offset=5):
    """Measure held-out scan endpoint distances to occupied map cells."""
    scan_items = []
    for _, scan in scan_records:
        timestamp = stamp_ns(scan.header.stamp)
        ranges = np.asarray(scan.ranges, dtype=np.float32)
        angles = scan.angle_min + np.arange(len(ranges)) * scan.angle_increment
        scan_items.append((
            timestamp,
            ranges,
            angles,
            scan.range_min,
            scan.range_max,
        ))
    scan_items.sort(key=lambda item: item[0])
    scan_stamps = np.array([item[0] for item in scan_items], dtype=np.int64)

    width = map_message.info.width
    height = map_message.info.height
    resolution = map_message.info.resolution
    occupied = np.asarray(map_message.data).reshape(height, width) >= 50
    wall_distance = distance_transform_edt(~occupied) * resolution
    origin = pose_2d(map_message.info.origin)
    inverse_origin = inverse(origin)
    endpoint_distances = []
    scan_time_error_ms = []
    outside_count = 0
    endpoint_count = 0

    for index, update in enumerate(valid_updates):
        timestamp, map_to_base, odom_to_base, _ = update
        if timestamp < settle_ns:
            continue
        if predicted:
            if index == 0:
                continue
            map_to_base = compose(valid_updates[index - 1][3], odom_to_base)
        scan_index = nearest_by_stamp(scan_stamps, timestamp)
        scan_data = scan_items[scan_index]
        scan_timestamp, ranges, angles, range_min, range_max = scan_data
        scan_time_error_ms.append(abs(scan_timestamp - timestamp) / 1e6)
        # PF consumes beams 0, 10, 20, ...; offset 5 evaluates unseen beams.
        sampled_ranges = ranges[beam_offset::10]
        sampled_angles = angles[beam_offset::10]
        maximum_hit_range = min(float(range_max) * 0.995, 9.95)
        valid = np.logical_and.reduce((
            np.isfinite(sampled_ranges),
            sampled_ranges > max(float(range_min), 0.05),
            sampled_ranges < maximum_hit_range,
        ))
        sampled_ranges = sampled_ranges[valid]
        sampled_angles = sampled_angles[valid]
        if sampled_ranges.size == 0:
            continue

        map_to_laser = compose(map_to_base, laser_pose)
        endpoint_x = map_to_laser[0] + sampled_ranges * np.cos(
            map_to_laser[2] + sampled_angles)
        endpoint_y = map_to_laser[1] + sampled_ranges * np.sin(
            map_to_laser[2] + sampled_angles)
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
        endpoint_count += len(rows)
        outside_count += int(np.sum(~inside))
        endpoint_distances.extend(wall_distance[rows[inside], columns[inside]])

    distances = np.asarray(endpoint_distances)
    return {
        'pose_stage': 'pre_update_prediction' if predicted else 'post_update',
        'held_out_beam_offset': beam_offset,
        'endpoints': int(endpoint_count),
        'outside_map_fraction': float(outside_count / max(endpoint_count, 1)),
        'wall_distance_median_m': float(np.median(distances)),
        'wall_distance_p90_m': float(np.percentile(distances, 90)),
        'wall_distance_p95_m': float(np.percentile(distances, 95)),
        'within_0.10m_fraction': float(np.mean(distances <= 0.10)),
        'within_0.20m_fraction': float(np.mean(distances <= 0.20)),
        'scan_time_error_ms': percentile_summary(scan_time_error_ms),
    }


def replay_diagnostics(log_records):
    """Extract worker load and jump-guard diagnostics from replay logs."""
    sync_lines = []
    sync_values = []
    counter_values = []
    rejected = []
    for _, message in log_records:
        if message.name != 'particle_filter':
            continue
        if message.msg.startswith('PF sync:'):
            sync_lines.append(message.msg)
            match = SYNC_PATTERN.match(message.msg)
            if match:
                sync_values.append({
                    name: float(value)
                    for name, value in match.groupdict().items()
                })
            counter_match = COUNTERS_PATTERN.search(message.msg)
            if counter_match:
                counter_values.append({
                    name: int(value)
                    for name, value in counter_match.groupdict().items()
                })
        if 'Rejected PF scan update' in message.msg:
            rejected.append(message.msg)
    timing = {}
    for name in ('rate', 'compute', 'turnaround', 'age', 'cpu'):
        timing[name] = percentile_summary([
            values[name] for values in sync_values
            if name != 'age' or values[name] >= 0.0
        ])
    return {
        'sync_samples': len(sync_lines),
        'timing': timing,
        'counters': counter_values[-1] if counter_values else {},
        'last_sync_line': sync_lines[-1] if sync_lines else None,
        'logged_jump_rejections': len(rejected),
        'jump_rejection_lines': rejected,
    }


def percent(value):
    """Format a ratio as a percentage."""
    return 'N/A' if value is None else f'{100.0 * value:.2f}%'


def number(value, digits=2):
    """Format a scalar while preserving missing metrics."""
    return 'N/A' if value is None else f'{value:.{digits}f}'


def ratio(count, total):
    """Format an exact-match count and percentage."""
    if not total:
        return 'N/A'
    return f'{count}/{total} ({100.0 * count / total:.2f}%)'


def reduction(old, new):
    """Format relative reduction, where a positive result is better."""
    if old is None or new is None or abs(old) < 1e-12:
        return 'N/A'
    return f'{100.0 * (old - new) / abs(old):+.1f}%'


def percentage_point_gain(old, new):
    """Format a ratio change in percentage points."""
    if old is None or new is None:
        return 'N/A'
    return f'{100.0 * (new - old):+.2f} pp'


def text_table(headers, rows):
    """Render a compact aligned plain-text table."""
    string_rows = [[str(value) for value in row] for row in rows]
    all_rows = [list(headers)] + string_rows
    widths = [
        max(len(row[column]) for row in all_rows)
        for column in range(len(headers))
    ]

    def render(row):
        return '  '.join(
            value.ljust(widths[index])
            for index, value in enumerate(row)
        ).rstrip()

    return '\n'.join([
        render(list(headers)),
        render(['-' * width for width in widths]),
        *(render(row) for row in string_rows),
    ])


def render_text_report(results, old_path, new_path):
    """Render the requested one-screen PF A/B report."""
    old = results['old']
    new = results['new']
    old_update = old['update_timing']
    new_update = new['update_timing']
    old_output = old['output']
    new_output = new['output']
    old_pre = old['map_fit_held_out_pre']
    new_pre = new['map_fit_held_out_pre']
    old_post = old['map_fit_held_out_post']
    new_post = new['map_fit_held_out_post']
    old_correction = old['corrections']
    new_correction = new['corrections']
    diagnostics = new['diagnostics']
    timing = diagnostics['timing']
    counters = diagnostics['counters']

    timestamp_rows = [
        (
            'PF update == scan stamp',
            ratio(old_update['exact_source_scan_stamps'], old_update['count']),
            ratio(new_update['exact_source_scan_stamps'], new_update['count']),
            '100%',
        ),
        (
            'Duplicate update stamps',
            old_update['adjacent_duplicate_stamps'],
            new_update['adjacent_duplicate_stamps'],
            '0',
        ),
        (
            'Non-monotonic update stamps',
            old_update['non_monotonic_stamps'],
            new_update['non_monotonic_stamps'],
            '0',
        ),
        (
            'PF output == odom stamp',
            ratio(old_output['exact_source_odom_stamps'], old_output['count']),
            ratio(new_output['exact_source_odom_stamps'], new_output['count']),
            '100%',
        ),
        (
            'PF output rate',
            f"{old_output['rate_hz']:.2f} Hz",
            f"{new_output['rate_hz']:.2f} Hz",
            'odom rate',
        ),
    ]

    map_rows = []
    for label, key, scale, unit, higher_is_better in (
            ('Within 10 cm', 'within_0.10m_fraction', 100.0, '%', True),
            ('Wall distance p90', 'wall_distance_p90_m', 100.0, 'cm', False),
            ('Wall distance p95', 'wall_distance_p95_m', 100.0, 'cm', False),
            ('Outside map', 'outside_map_fraction', 100.0, '%', False)):
        old_value = old_pre[key]
        new_value = new_pre[key]
        change = (
            percentage_point_gain(old_value, new_value)
            if higher_is_better
            else reduction(old_value, new_value)
        )
        map_rows.append((
            label,
            f'{scale * old_value:.2f} {unit}',
            f'{scale * new_value:.2f} {unit}',
            change,
        ))
    map_rows.append((
        'Post-update within 10 cm',
        percent(old_post['within_0.10m_fraction']),
        percent(new_post['within_0.10m_fraction']),
        percentage_point_gain(
            old_post['within_0.10m_fraction'],
            new_post['within_0.10m_fraction'],
        ),
    ))

    correction_rows = []
    for label, group, key, scale, unit in (
            ('Translation p50', 'translation_m', 'median', 100.0, 'cm'),
            ('Translation p95', 'translation_m', 'p95', 100.0, 'cm'),
            ('Translation p99', 'translation_m', 'p99', 100.0, 'cm'),
            ('Yaw p50', 'yaw_deg', 'median', 1.0, 'deg'),
            ('Yaw p95', 'yaw_deg', 'p95', 1.0, 'deg'),
            ('Yaw p99', 'yaw_deg', 'p99', 1.0, 'deg')):
        old_value = old_correction[group][key]
        new_value = new_correction[group][key]
        correction_rows.append((
            label,
            f'{scale * old_value:.2f} {unit}',
            f'{scale * new_value:.2f} {unit}',
            reduction(old_value, new_value),
        ))
    for axis in ('longitudinal', 'lateral', 'yaw'):
        old_value = old_correction['alternating_fraction'][axis]
        new_value = new_correction['alternating_fraction'][axis]
        correction_rows.append((
            f'{axis.capitalize()} sign flip',
            percent(old_value),
            percent(new_value),
            percentage_point_gain(old_value, new_value),
        ))
        old_value = old_correction['lag1'][axis]
        new_value = new_correction['lag1'][axis]
        correction_rows.append((
            f'{axis.capitalize()} lag-1',
            number(old_value, 3),
            number(new_value, 3),
            f'delta {new_value - old_value:+.3f}',
        ))

    runtime_rows = []
    for label, group, unit in (
            ('PF update rate', 'rate', 'Hz'),
            ('Compute', 'compute', 'ms'),
            ('Turnaround', 'turnaround', 'ms'),
            ('Correction age', 'age', 'ms'),
            ('Single-core load', 'cpu', '%')):
        values = timing.get(group, {})
        runtime_rows.append((
            label,
            f"{number(values.get('median'))} {unit}",
            f"{number(values.get('p95'))} {unit}",
            f"{number(values.get('p99'))} {unit}",
            f"{number(values.get('max'))} {unit}",
        ))

    offered = counters.get('offered', 0)
    drop_rows = [
        (
            'Latest overwrite',
            counters.get('overwritten', 'N/A'),
            percent(counters.get('overwritten', 0) / offered)
            if offered else 'N/A',
        ),
        (
            'Sync drop',
            counters.get('sync_drop', 'N/A'),
            percent(counters.get('sync_drop', 0) / offered)
            if offered else 'N/A',
        ),
        ('Stale drop', counters.get('stale_drop', 'N/A'), 'expected 0'),
        ('Order drop', counters.get('order_drop', 'N/A'), 'expected 0'),
        ('Worker error', counters.get('worker_error', 'N/A'), 'expected 0'),
    ]

    timestamp_pass = (
        new_update['exact_source_scan_stamps'] == new_update['count']
        and new_update['adjacent_duplicate_stamps'] == 0
        and new_update['non_monotonic_stamps'] == 0
        and new_output['exact_source_odom_stamps'] == new_output['count']
    )
    map_improved = (
        new_pre['within_0.10m_fraction'] > old_pre['within_0.10m_fraction']
        and new_pre['wall_distance_p95_m'] <= old_pre['wall_distance_p95_m']
    )
    correction_improved = (
        new_correction['translation_m']['p95']
        < old_correction['translation_m']['p95']
        and new_correction['yaw_deg']['p95']
        < old_correction['yaw_deg']['p95']
        and new_correction['alternating_fraction']['yaw']
        < old_correction['alternating_fraction']['yaw']
        and abs(new_correction['lag1']['yaw'])
        < abs(old_correction['lag1']['yaw'])
    )
    runtime_pass = (
        timing.get('turnaround', {}).get('p99', math.inf) < 100.0
        and timing.get('age', {}).get('p99', math.inf) < 500.0
        and counters.get('stale_drop', 1) == 0
        and counters.get('order_drop', 1) == 0
        and counters.get('worker_error', 1) == 0
    )
    overall = all((
        timestamp_pass,
        map_improved,
        correction_improved,
        runtime_pass,
    ))

    sections = [
        'PF LOCALIZATION A/B REPORT',
        f'Baseline : {old_path}',
        f'Candidate: {new_path}',
        f"Settle   : {results['settle_sec']:.1f} s excluded",
        'Held-out : beams 5, 15, 25, ... (PF uses 0, 10, 20, ...)',
        '',
        '[1] TIMESTAMP INTEGRITY',
        text_table(
            ('Metric', 'Baseline', 'Candidate', 'Target'), timestamp_rows),
        '',
        '[2] HELD-OUT SCAN-MAP FIT (PRE-UPDATE PREDICTION)',
        text_table(('Metric', 'Baseline', 'Candidate', 'Change'), map_rows),
        '',
        '[3] SCAN CORRECTION STABILITY',
        text_table(
            ('Metric', 'Baseline', 'Candidate', 'Change'), correction_rows),
        '',
        '[4] CANDIDATE RUNTIME',
        text_table(('Metric', 'p50', 'p95', 'p99', 'Max'), runtime_rows),
        '',
        '[5] QUEUE / ERROR COUNTERS (LAST PERIODIC DIAGNOSTIC)',
        text_table(('Counter', 'Count', 'Ratio / Target'), drop_rows),
        '',
        '[SUMMARY]',
        f"Timestamp integrity : {'PASS' if timestamp_pass else 'FAIL'}",
        f"Held-out map fit    : {'IMPROVED' if map_improved else 'CHECK'}",
        'Correction stability: %s' % (
            'IMPROVED' if correction_improved else 'CHECK'),
        f"Runtime health      : {'PASS' if runtime_pass else 'CHECK'}",
        f"Overall no-GT result: {'IMPROVED' if overall else 'CHECK REQUIRED'}",
        '',
        'Note: counters are cumulative at the last 1 Hz diagnostic line.',
        '      Latest overwrite is intentional, not a localization failure.',
    ]
    return '\n'.join(sections)


def main():
    """Run the comparison and print a text or JSON report."""
    parser = argparse.ArgumentParser()
    parser.add_argument('--old', required=True, help='Original db3 bag')
    parser.add_argument('--new', required=True, help='Replay output bag')
    parser.add_argument('--settle-sec', type=float, default=10.0)
    parser.add_argument(
        '--format', choices=('text', 'json'), default='text',
        help='Output format (default: text)',
    )
    parser.add_argument(
        '--output', help='Also write the rendered report to this file',
    )
    args = parser.parse_args()

    old = read_bag(args.old)
    new = read_bag(args.new)
    old_updates, old_pairing_error = old_update_poses(old)
    new_updates = new_update_poses(new)
    odom = OdomInterpolator(old['/odom'])
    laser_pose = base_to_laser_pose(old)
    settle_ns = int(odom.stamps[0] + args.settle_sec * 1e9)
    old_corrections, old_valid = correction_metrics(
        old_updates, odom, settle_ns)
    new_corrections, new_valid = correction_metrics(
        new_updates, odom, settle_ns)
    map_message = old['/map'][0][1]

    results = {
        'settle_sec': args.settle_sec,
        'base_to_laser_pose': laser_pose.tolist(),
        'old': {
            'updates': len(old_updates),
            'update_timing': update_timing(old_updates, old['/scan']),
            'inferred_fake_record_pairing_ms': percentile_summary(
                old_pairing_error),
            'output': output_timing(old['/pf/pose/odom'], old['/odom']),
            'corrections': old_corrections,
            'map_fit_held_out_post': map_fit_metrics(
                old_valid, old['/scan'], map_message, settle_ns,
                laser_pose),
            'map_fit_held_out_pre': map_fit_metrics(
                old_valid, old['/scan'], map_message, settle_ns,
                laser_pose, predicted=True),
        },
        'new': {
            'updates': len(new_updates),
            'update_timing': update_timing(new_updates, old['/scan']),
            'output': output_timing(new['/pf/pose/odom'], new['/odom']),
            'corrections': new_corrections,
            'map_fit_held_out_post': map_fit_metrics(
                new_valid, old['/scan'], map_message, settle_ns,
                laser_pose),
            'map_fit_held_out_pre': map_fit_metrics(
                new_valid, old['/scan'], map_message, settle_ns,
                laser_pose, predicted=True),
            'diagnostics': replay_diagnostics(new['/rosout']),
        },
    }
    if args.format == 'json':
        report = json.dumps(results, indent=2, ensure_ascii=True)
    else:
        report = render_text_report(results, args.old, args.new)
    print(report)
    if args.output:
        Path(args.output).write_text(report + '\n', encoding='utf-8')


if __name__ == '__main__':
    main()
