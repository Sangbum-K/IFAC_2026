#!/usr/bin/env python3
"""Offline geometry A/B for map ray-casting and scan deskew on a ROS 2 bag.

This intentionally evaluates the pre-clustering point gate.  It uses recorded
``/pf/pose/odom`` as the map-frame base pose, applies the production median/FOV/
range/EDT gates, and compares each measured range with the first occupied map
cell along the same ray.  The deskew variant interpolates the recorded pose at
every beam timestamp.
"""

import argparse
import ctypes
import math
from pathlib import Path
import subprocess
import warnings

import numpy as np
from PIL import Image
from scipy.ndimage import binary_dilation, distance_transform_edt
import yaml

from nav_msgs.msg import Odometry
from rclpy.serialization import deserialize_message
from rosbag2_py import ConverterOptions, SequentialReader, StorageOptions
from sensor_msgs.msg import LaserScan


ROOT = Path(__file__).resolve().parents[1]
DEFAULT_BAG = ROOT / "bags/rosbag0813/run_0813_165312"
DEFAULT_MAP = ROOT / "maps/my_track0813.yaml"
LIBRARY = Path("/tmp/ifac_offline_raycast.so")

KNOWN_WALL_ROIS = {
    "wall_+5.34_+2.39": (5.34, 2.39, 0.40),
    "wall_-9.23_-0.12": (-9.23, -0.12, 0.45),
    "wall_-9.38_+4.42": (-9.38, 4.42, 0.45),
}

# Repeated compact groups away from the three known wall-mismatch regions.
# In run_0813_165312 (the only 0813 run with physical obstacles enabled), these
# are the positive-control object locations used to detect collateral removal.
OBJECT_ROIS = {
    "object_-7.78_+1.33": (-7.78, 1.33, 0.35),
    "object_-5.49_+4.30": (-5.49, 4.30, 0.35),
    "object_-1.49_-0.50": (-1.49, -0.50, 0.40),
    "object_-3.64_-0.39": (-3.64, -0.39, 0.35),
    "object_+0.28_+3.16": (0.28, 3.16, 0.35),
    "object_-1.76_+1.91": (-1.76, 1.91, 0.35),
}
ALL_ROIS = {**KNOWN_WALL_ROIS, **OBJECT_ROIS}


def stamp_ns(stamp):
    return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)


def yaw_from_quaternion(q):
    return math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z),
    )


def read_bag(path):
    reader = SequentialReader()
    reader.open(
        StorageOptions(uri=str(path), storage_id="sqlite3"),
        ConverterOptions("cdr", "cdr"),
    )
    scan_stamps = []
    scan_ranges = []
    scan_meta = None
    pose_stamps = []
    poses = []
    while reader.has_next():
        topic, payload, record_stamp = reader.read_next()
        if topic == "/scan":
            message = deserialize_message(payload, LaserScan)
            current = (
                len(message.ranges), float(message.angle_min),
                float(message.angle_increment), float(message.time_increment),
            )
            if scan_meta is None:
                scan_meta = current
            elif current != scan_meta:
                raise RuntimeError(f"LaserScan geometry changed: {scan_meta} -> {current}")
            scan_stamps.append(stamp_ns(message.header.stamp) or int(record_stamp))
            scan_ranges.append(np.asarray(message.ranges, dtype=np.float32))
        elif topic == "/pf/pose/odom":
            message = deserialize_message(payload, Odometry)
            pose_stamps.append(stamp_ns(message.header.stamp) or int(record_stamp))
            p = message.pose.pose.position
            poses.append((float(p.x), float(p.y), yaw_from_quaternion(message.pose.pose.orientation)))
    if not scan_ranges or len(pose_stamps) < 2:
        raise RuntimeError("bag needs /scan and /pf/pose/odom")
    pose_stamps = np.asarray(pose_stamps, dtype=np.int64)
    poses = np.asarray(poses, dtype=np.float64)
    order = np.argsort(pose_stamps, kind="stable")
    pose_stamps = pose_stamps[order]
    poses = poses[order]
    unique = np.r_[True, pose_stamps[1:] != pose_stamps[:-1]]
    return (
        np.asarray(scan_stamps, dtype=np.int64),
        np.asarray(scan_ranges, dtype=np.float32),
        scan_meta,
        pose_stamps[unique],
        poses[unique],
    )


def load_map(path):
    with open(path, encoding="utf-8") as stream:
        metadata = yaml.safe_load(stream)
    image_path = Path(metadata["image"])
    if not image_path.is_absolute():
        image_path = path.parent / image_path
    # OccupancyGrid row zero is the bottom image row.  The production detector
    # treats both occupied and unknown cells as walls; only 254 is known free.
    pixels = np.flipud(np.asarray(Image.open(image_path)))
    occupied = np.where(pixels >= 250, 0, 100).astype(np.int8)
    resolution = float(metadata["resolution"])
    distance = distance_transform_edt(occupied == 0) * resolution
    return {
        "occupied": np.ascontiguousarray(occupied),
        "distance": distance,
        "resolution": resolution,
        "origin": tuple(float(v) for v in metadata["origin"]),
        "width": occupied.shape[1],
        "height": occupied.shape[0],
    }


def load_raycast_library():
    source = ROOT / "tools/offline_raycast.cpp"
    if not LIBRARY.exists() or LIBRARY.stat().st_mtime_ns < source.stat().st_mtime_ns:
        subprocess.run(
            ["g++", "-O3", "-shared", "-fPIC", "-fopenmp", str(source), "-o", str(LIBRARY)],
            check=True,
        )
    library = ctypes.CDLL(str(LIBRARY))
    ptr_i8 = ctypes.POINTER(ctypes.c_int8)
    ptr_f = ctypes.POINTER(ctypes.c_float)
    library.raycast_pairs.argtypes = [
        ptr_i8, ctypes.c_int, ctypes.c_int, ctypes.c_float,
        ctypes.c_float, ctypes.c_float, ctypes.c_float,
        ptr_f, ptr_f, ctypes.c_int, ctypes.c_float, ptr_f,
    ]
    return library


def interpolate_poses(stamps, pose_stamps, poses):
    requested = np.asarray(stamps, dtype=np.int64)
    values = requested.astype(np.float64)
    source = pose_stamps.astype(np.float64)
    yaw = np.unwrap(poses[:, 2])
    return np.column_stack((
        np.interp(values, source, poses[:, 0]),
        np.interp(values, source, poses[:, 1]),
        np.interp(values, source, yaw),
    ))


def median3(ranges):
    output = ranges.copy()
    windows = np.stack((ranges[:, :-2], ranges[:, 1:-1], ranges[:, 2:]), axis=0)
    windows = np.where(np.isfinite(windows), windows, np.nan)
    with warnings.catch_warnings():
        warnings.simplefilter("ignore", category=RuntimeWarning)
        medians = np.nanmedian(windows, axis=0)
    valid = np.isfinite(medians)
    output[:, 1:-1] = np.where(valid, medians, output[:, 1:-1])
    return output


def cast_pairs(library, grid, sensor_poses, angles, maximum_range):
    sensor_poses = np.ascontiguousarray(sensor_poses, dtype=np.float32)
    angles = np.ascontiguousarray(angles, dtype=np.float32)
    output = np.empty(len(angles), dtype=np.float32)
    occupied = grid["occupied"]
    ox, oy, oyaw = grid["origin"]
    library.raycast_pairs(
        occupied.ctypes.data_as(ctypes.POINTER(ctypes.c_int8)),
        grid["width"], grid["height"], grid["resolution"],
        ox, oy, oyaw,
        sensor_poses.ctypes.data_as(ctypes.POINTER(ctypes.c_float)),
        angles.ctypes.data_as(ctypes.POINTER(ctypes.c_float)),
        len(angles), maximum_range,
        output.ctypes.data_as(ctypes.POINTER(ctypes.c_float)),
    )
    return output


def empty_metrics(margins):
    return {
        "valid": 0,
        "edt_keep": 0,
        "ray_keep": {m: 0 for m in margins},
        "residuals": [],
        "rois": {
            name: {"edt": 0, **{m: 0 for m in margins}}
            for name in ALL_ROIS
        },
    }


def evaluate_variant(
        library, grid, scan_stamps, ranges, beam_indices, beam_angles,
        time_increment, pose_stamps, poses, laser_x, deskew, margins,
        chunk_size=192):
    metrics = empty_metrics(margins)
    maximum_range = 12.0
    map_x, map_y, _ = grid["origin"]
    resolution = grid["resolution"]
    for start in range(0, len(scan_stamps), chunk_size):
        stop = min(len(scan_stamps), start + chunk_size)
        selected = median3(ranges[start:stop])[:, beam_indices]
        count = stop - start
        flat_ranges = selected.reshape(-1)
        flat_angles = np.tile(beam_angles, count)
        valid = np.isfinite(flat_ranges) & (flat_ranges >= 0.10) & (flat_ranges <= maximum_range)
        if not np.any(valid):
            continue
        if deskew:
            beam_times = (
                scan_stamps[start:stop, None]
                + np.rint(beam_indices[None, :] * time_increment * 1e9).astype(np.int64)
            ).reshape(-1)
            base = interpolate_poses(beam_times[valid], pose_stamps, poses)
        else:
            base_at_scan = interpolate_poses(scan_stamps[start:stop], pose_stamps, poses)
            base = np.repeat(base_at_scan, len(beam_indices), axis=0)[valid]
        angles_valid = flat_angles[valid]
        ranges_valid = flat_ranges[valid].astype(np.float64)
        cosine = np.cos(base[:, 2])
        sine = np.sin(base[:, 2])
        sensor = np.column_stack((
            base[:, 0] + laser_x * cosine,
            base[:, 1] + laser_x * sine,
            base[:, 2],
        ))
        world_angles = sensor[:, 2] + angles_valid
        point_x = sensor[:, 0] + ranges_valid * np.cos(world_angles)
        point_y = sensor[:, 1] + ranges_valid * np.sin(world_angles)
        columns = np.floor((point_x - map_x) / resolution).astype(np.int64)
        rows = np.floor((point_y - map_y) / resolution).astype(np.int64)
        inside = (
            (rows >= 0) & (rows < grid["height"])
            & (columns >= 0) & (columns < grid["width"])
        )
        wall_distance = np.zeros(len(rows), dtype=np.float64)
        wall_distance[inside] = grid["distance"][rows[inside], columns[inside]]
        edt_keep = inside & (wall_distance >= 0.25)
        expected = cast_pairs(library, grid, sensor, angles_valid, maximum_range).astype(np.float64)
        gap = expected - ranges_valid
        wall_like = edt_keep & (expected < maximum_range - 1e-3) & (np.abs(gap) <= 0.75)
        if np.any(wall_like):
            metrics["residuals"].append(np.abs(gap[wall_like]))
        metrics["valid"] += int(len(ranges_valid))
        metrics["edt_keep"] += int(np.count_nonzero(edt_keep))
        for margin in margins:
            keep = edt_keep & (gap > margin)
            metrics["ray_keep"][margin] += int(np.count_nonzero(keep))
        for name, (cx, cy, radius) in ALL_ROIS.items():
            roi = edt_keep & (np.hypot(point_x - cx, point_y - cy) <= radius)
            metrics["rois"][name]["edt"] += int(np.count_nonzero(roi))
            for margin in margins:
                metrics["rois"][name][margin] += int(np.count_nonzero(roi & (gap > margin)))
    if metrics["residuals"]:
        metrics["residuals"] = np.concatenate(metrics["residuals"])
    else:
        metrics["residuals"] = np.empty(0)
    return metrics


def print_metrics(label, metrics, margins):
    print(f"\n[{label}]")
    valid = metrics["valid"]
    edt = metrics["edt_keep"]
    print(f"valid points={valid:,}  EDT>=0.25m={edt:,} ({100.0 * edt / max(valid, 1):.2f}%)")
    residuals = metrics["residuals"]
    if len(residuals):
        q = np.percentile(residuals, [50, 90, 95, 99])
        print(
            "wall |r_map-r_scan| p50/p90/p95/p99="
            + "/".join(f"{v:.3f}" for v in q) + " m"
        )
    for margin in margins:
        kept = metrics["ray_keep"][margin]
        print(
            f"ray margin={margin:.2f}m: keep={kept:,}/{edt:,} "
            f"({100.0 * kept / max(edt, 1):.2f}%), remove={100.0 * (edt-kept) / max(edt, 1):.2f}%"
        )
    print("ROI point survival (after current EDT -> after raycast):")
    for name, values in metrics["rois"].items():
        text = [f"{name} {values['edt']:,}"]
        for margin in margins:
            text.append(f"m{margin:.2f}={100.0 * values[margin] / max(values['edt'], 1):.1f}%")
        print("  " + "  ".join(text))


def collect_wall_patch_points(
        scan_stamps, ranges, beam_indices, beam_angles,
        pose_stamps, poses, laser_x):
    selected = median3(ranges)[:, beam_indices]
    base = interpolate_poses(scan_stamps, pose_stamps, poses)
    points = []
    for index in range(len(scan_stamps)):
        scan_ranges = selected[index]
        valid = np.isfinite(scan_ranges) & (scan_ranges >= 0.10) & (scan_ranges <= 12.0)
        if not np.any(valid):
            continue
        yaw = base[index, 2]
        sx = base[index, 0] + laser_x * math.cos(yaw)
        sy = base[index, 1] + laser_x * math.sin(yaw)
        world_angles = yaw + beam_angles[valid]
        x = sx + scan_ranges[valid] * np.cos(world_angles)
        y = sy + scan_ranges[valid] * np.sin(world_angles)
        candidate = np.column_stack((x, y))
        for cx, cy, radius in KNOWN_WALL_ROIS.values():
            mask = np.hypot(candidate[:, 0] - cx, candidate[:, 1] - cy) <= radius
            if np.any(mask):
                points.append(candidate[mask])
    return np.concatenate(points) if points else np.empty((0, 2))


def aligned_map_from_training(grid, points):
    """Fit one straight barrier segment per known mismatch ROI.

    This models a manual barrier-map realignment without copying every training
    endpoint into the map.  Only the named wall ROIs are changed; positive-
    control object regions remain untouched.
    """
    aligned = dict(grid)
    occupied = np.array(grid["occupied"], copy=True)
    ox, oy, _ = grid["origin"]
    fitted = {}
    for name, (cx, cy, radius) in KNOWN_WALL_ROIS.items():
        roi = points[np.hypot(points[:, 0] - cx, points[:, 1] - cy) <= radius]
        if len(roi) < 10:
            fitted[name] = 0
            continue
        center = np.median(roi, axis=0)
        covariance = np.cov((roi - center).T)
        direction = np.linalg.eigh(covariance)[1][:, -1]
        along = (roi - center) @ direction
        low, high = np.percentile(along, [2.0, 98.0])
        samples = np.linspace(low, high, max(2, int((high - low) / (grid["resolution"] * 0.4))))
        line = center + samples[:, None] * direction[None, :]
        columns = np.floor((line[:, 0] - ox) / grid["resolution"]).astype(int)
        rows = np.floor((line[:, 1] - oy) / grid["resolution"]).astype(int)
        valid = (
            (rows >= 0) & (rows < grid["height"])
            & (columns >= 0) & (columns < grid["width"])
        )
        line_mask = np.zeros_like(occupied, dtype=bool)
        line_mask[rows[valid], columns[valid]] = True
        # A hand-aligned occupancy barrier has finite thickness.  One 8-neighbor
        # dilation is a 5 cm rasterization allowance, not another filter margin.
        line_mask = binary_dilation(line_mask, structure=np.ones((3, 3), dtype=bool))
        new_cells = line_mask & (occupied == 0)
        occupied[line_mask] = 100
        fitted[name] = int(np.count_nonzero(new_cells))
    aligned["occupied"] = np.ascontiguousarray(occupied)
    aligned["distance"] = distance_transform_edt(occupied == 0) * grid["resolution"]
    return aligned, fitted


def distance_at_points(grid, point_x, point_y):
    ox, oy, _ = grid["origin"]
    columns = np.floor((point_x - ox) / grid["resolution"]).astype(np.int64)
    rows = np.floor((point_y - oy) / grid["resolution"]).astype(np.int64)
    inside = (
        (rows >= 0) & (rows < grid["height"])
        & (columns >= 0) & (columns < grid["width"])
    )
    distance = np.zeros(len(rows), dtype=np.float64)
    distance[inside] = grid["distance"][rows[inside], columns[inside]]
    return inside, distance


def production_clusters(points, point_ranges, angle_increment, grid):
    """Mirror the detector's breakpoint/open-space/radius gates."""
    if len(points) < 4:
        return []
    lambda_angle = math.radians(10.0)
    denominator = max(math.sin(lambda_angle - angle_increment), 1e-6)
    pair_range = 0.5 * (point_ranges[:-1] + point_ranges[1:])
    dmax = (
        pair_range * math.sin(angle_increment) / denominator + 3.0 * 0.015
    ) * 0.5
    gaps = np.hypot(np.diff(points[:, 0]), np.diff(points[:, 1])) > dmax
    boundaries = np.r_[0, np.flatnonzero(gaps) + 1, len(points)]
    output = []
    for begin, end in zip(boundaries[:-1], boundaries[1:]):
        if end - begin < 4:
            continue
        component = points[begin:end]
        centroid = np.mean(component, axis=0)
        if end - begin < 8:
            _, wall_distance = distance_at_points(
                grid, centroid[None, 0], centroid[None, 1])
            if wall_distance[0] < 0.50:
                continue
        radius = float(np.max(np.hypot(
            component[:, 0] - centroid[0], component[:, 1] - centroid[1])))
        if radius <= 0.35:
            output.append((centroid, radius, end - begin))
    return output


def evaluate_cluster_frames(
        library, grid, scan_stamps, ranges, beam_indices, beam_angles,
        pose_stamps, poses, laser_x, time_increment, ray_rule=None,
        deskew=False):
    selected = median3(ranges)[:, beam_indices]
    base = None if deskew else interpolate_poses(scan_stamps, pose_stamps, poses)
    roi_frames = {name: 0 for name in ALL_ROIS}
    cluster_total = 0
    nonempty_frames = 0
    for index in range(len(scan_stamps)):
        scan_ranges = selected[index]
        valid = np.isfinite(scan_ranges) & (scan_ranges >= 0.10) & (scan_ranges <= 12.0)
        if not np.any(valid):
            continue
        local_angles = beam_angles[valid]
        local_ranges = scan_ranges[valid].astype(np.float64)
        if deskew:
            beam_times = scan_stamps[index] + np.rint(
                beam_indices[valid] * time_increment * 1e9).astype(np.int64)
            beam_base = interpolate_poses(beam_times, pose_stamps, poses)
        else:
            beam_base = np.repeat(base[index:index + 1], len(local_angles), axis=0)
        yaw = beam_base[:, 2]
        sx = beam_base[:, 0] + laser_x * np.cos(yaw)
        sy = beam_base[:, 1] + laser_x * np.sin(yaw)
        world_angles = yaw + local_angles
        point_x = sx + local_ranges * np.cos(world_angles)
        point_y = sy + local_ranges * np.sin(world_angles)
        inside, wall_distance = distance_at_points(grid, point_x, point_y)
        keep = inside & (wall_distance >= 0.25)
        if ray_rule is not None and np.any(keep):
            sensor = np.column_stack((sx, sy, yaw)).astype(np.float32)
            expected = cast_pairs(library, grid, sensor, local_angles, 12.0)
            gap = expected.astype(np.float64) - local_ranges
            mode, threshold = ray_rule
            if mode == "short":
                keep &= gap > threshold
            elif mode == "band":
                keep &= np.abs(gap) > threshold
            else:
                raise ValueError(f"unknown ray rule: {ray_rule}")
        points = np.column_stack((point_x[keep], point_y[keep]))
        clusters = production_clusters(
            points, local_ranges[keep], float(beam_angles[1] - beam_angles[0]), grid)
        cluster_total += len(clusters)
        nonempty_frames += bool(clusters)
        for name, (cx, cy, radius) in ALL_ROIS.items():
            if any(np.hypot(c[0][0] - cx, c[0][1] - cy) <= radius for c in clusters):
                roi_frames[name] += 1
    return {
        "scans": len(scan_stamps),
        "cluster_total": cluster_total,
        "nonempty_frames": nonempty_frames,
        "roi_frames": roi_frames,
    }


def print_cluster_comparison(results):
    print("\n[held-out pre-tracker cluster A/B]")
    print("variant                              clusters  nonempty")
    for label, result in results:
        print(
            f"{label:35s} {result['cluster_total']:8d}  "
            f"{result['nonempty_frames']:4d}/{result['scans']}"
        )
    print("ROI detection frames (at least one production-like cluster):")
    header = "ROI" + "".join(f" {label[:14]:>14s}" for label, _ in results)
    print(header)
    for name in ALL_ROIS:
        print(name + "".join(
            f" {result['roi_frames'][name]:14d}" for _, result in results))


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--bag", type=Path, default=DEFAULT_BAG)
    parser.add_argument("--map", dest="map_path", type=Path, default=DEFAULT_MAP)
    parser.add_argument("--max-scans", type=int, default=0)
    args = parser.parse_args()

    scan_stamps, ranges, meta, pose_stamps, poses = read_bag(args.bag)
    if args.max_scans > 0:
        scan_stamps = scan_stamps[:args.max_scans]
        ranges = ranges[:args.max_scans]
    scan_size, angle_min, angle_increment, time_increment = meta
    all_angles = angle_min + np.arange(scan_size) * angle_increment
    beam_indices = np.flatnonzero(
        (all_angles >= math.radians(-80.0)) & (all_angles <= math.radians(80.0))
    )
    # Discard scans whose full acquisition interval is outside the pose history.
    end_stamps = scan_stamps + np.rint((scan_size - 1) * time_increment * 1e9).astype(np.int64)
    usable = (scan_stamps >= pose_stamps[0]) & (end_stamps <= pose_stamps[-1])
    scan_stamps = scan_stamps[usable]
    ranges = ranges[usable]
    grid = load_map(args.map_path)
    library = load_raycast_library()
    margins = (0.10, 0.15, 0.20, 0.25, 0.30)
    print(
        f"bag={args.bag} scans={len(scan_stamps):,} beams/FOV={len(beam_indices)} "
        f"span={(scan_size - 1) * time_increment * 1000.0:.3f}ms map={grid['width']}x{grid['height']}"
    )
    variants = (
        ("raw x=0.27", 0.27, False),
        ("raw x=0.28", 0.28, False),
        ("per-beam pose (deskew) x=0.28", 0.28, True),
    )
    for label, laser_x, deskew in variants:
        metrics = evaluate_variant(
            library, grid, scan_stamps, ranges, beam_indices, all_angles[beam_indices],
            time_increment, pose_stamps, poses, laser_x, deskew, margins,
        )
        print_metrics(label, metrics, margins)

    split = int(round(len(scan_stamps) * 0.55))
    patch_points = collect_wall_patch_points(
        scan_stamps[:split], ranges[:split], beam_indices, all_angles[beam_indices],
        pose_stamps, poses, 0.28,
    )
    aligned_grid, fitted = aligned_map_from_training(grid, patch_points)
    print(
        f"\n[held-out map realignment] train_scans={split:,} test_scans={len(scan_stamps)-split:,} "
        f"wall_points={len(patch_points):,} new_cells={fitted}"
    )
    baseline_test = evaluate_variant(
        library, grid, scan_stamps[split:], ranges[split:], beam_indices,
        all_angles[beam_indices], time_increment, pose_stamps, poses,
        0.28, False, margins,
    )
    print_metrics("held-out current map, raw x=0.28", baseline_test, margins)
    aligned_test = evaluate_variant(
        library, aligned_grid, scan_stamps[split:], ranges[split:], beam_indices,
        all_angles[beam_indices], time_increment, pose_stamps, poses,
        0.28, False, margins,
    )
    print_metrics("held-out aligned map, raw x=0.28", aligned_test, margins)
    cluster_results = []
    for label, active_grid, laser_x, ray_rule, deskew in (
        ("current x=.27 EDT", grid, 0.27, None, False),
        ("current x=.28 EDT", grid, 0.28, None, False),
        ("current x=.28 deskew EDT", grid, 0.28, None, True),
        ("current x=.28 EDT+short.15", grid, 0.28, ("short", 0.15), False),
        ("aligned x=.28 EDT", aligned_grid, 0.28, None, False),
        ("aligned x=.28 deskew EDT", aligned_grid, 0.28, None, True),
        ("aligned x=.28 EDT+short.15", aligned_grid, 0.28, ("short", 0.15), False),
        ("aligned x=.28 EDT+band.03", aligned_grid, 0.28, ("band", 0.03), False),
        ("aligned x=.28 EDT+band.05", aligned_grid, 0.28, ("band", 0.05), False),
        ("aligned x=.28 EDT+band.08", aligned_grid, 0.28, ("band", 0.08), False),
        ("aligned x=.28 EDT+band.10", aligned_grid, 0.28, ("band", 0.10), False),
    ):
        cluster_results.append((label, evaluate_cluster_frames(
            library, active_grid, scan_stamps[split:], ranges[split:],
            beam_indices, all_angles[beam_indices], pose_stamps, poses,
            laser_x, time_increment, ray_rule, deskew,
        )))
    print_cluster_comparison(cluster_results)
    print("\nNear-wall object contract: an object closer to the wall than the selected margin is intentionally ambiguous;")
    print("objects farther in front than the margin pass the ray gate. Validate clustering recall with injected scans before production.")


if __name__ == "__main__":
    main()
