"""
Preprocessing utilities for LaserScan obstacle detection.
"""
import numpy as np


def preprocess_scan(
    ranges,
    angle_min,
    angle_increment,
    range_min=0.05,
    range_max=10.0,
    fov_min=-np.pi,
    fov_max=np.pi,
    median_kernel=3,
    boundary_polygon=None,
):
    """Convert LaserScan ranges into filtered XY points in the laser frame."""
    ranges = np.asarray(ranges, dtype=np.float32)
    n = len(ranges)
    angles = angle_min + np.arange(n) * angle_increment

    if median_kernel >= 3 and median_kernel % 2 == 1 and n > median_kernel:
        ranges = _median_filter_1d(ranges, median_kernel)

    valid = (
        np.isfinite(ranges)
        & (ranges >= range_min)
        & (ranges <= range_max)
        & (angles >= fov_min)
        & (angles <= fov_max)
    )

    r = ranges[valid]
    a = angles[valid]

    xs = r * np.cos(a)
    ys = r * np.sin(a)
    points = np.stack([xs, ys], axis=1)

    if boundary_polygon is not None and len(points) > 0:
        polygon = np.asarray(boundary_polygon, dtype=np.float32)
        if polygon.ndim == 2 and polygon.shape[0] >= 3 and polygon.shape[1] == 2:
            mask = _points_in_polygon(points, polygon)
            points = points[mask]

    return points


def _median_filter_1d(arr, kernel):
    half = kernel // 2
    out = arr.copy()
    for i in range(half, len(arr) - half):
        window = arr[i - half:i + half + 1]
        finite = window[np.isfinite(window)]
        if len(finite) > 0:
            out[i] = np.median(finite)
    return out


def _points_in_polygon(points, polygon):
    x = points[:, 0]
    y = points[:, 1]
    inside = np.zeros(len(points), dtype=bool)

    x1 = polygon[:, 0]
    y1 = polygon[:, 1]
    x2 = np.roll(x1, -1)
    y2 = np.roll(y1, -1)

    for ax, ay, bx, by in zip(x1, y1, x2, y2):
        intersects = ((ay > y) != (by > y))
        denom = (by - ay) + 1e-12
        x_intersection = (bx - ax) * (y - ay) / denom + ax
        inside ^= intersects & (x < x_intersection)

    return inside
