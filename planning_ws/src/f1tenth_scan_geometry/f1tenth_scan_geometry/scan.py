"""3D scan deskew and conservative ground-intersection filtering."""

from dataclasses import dataclass

import numpy as np

from .geometry import level_relative_matrices
from .geometry import normalize_quaternions
from .geometry import quaternion_to_matrix
from .geometry import rotations_from_yaw_and_level_tilt


@dataclass(frozen=True)
class GroundFilterConfig:
    ground_z_m: float = 0.0
    range_tolerance_m: float = 0.08
    range_tolerance_ratio: float = 0.04
    endpoint_z_tolerance_m: float = 0.05
    min_component_beams: int = 6
    min_component_angle_rad: float = np.deg2rad(4.0)
    min_component_span_m: float = 0.30
    max_planar_elevation_rad: float = np.deg2rad(4.0)
    degraded_offplane_fraction: float = 0.55
    degraded_ground_fraction: float = 0.70
    attitude_fallback_tilt_rad: float = np.deg2rad(4.0)
    attitude_ground_evidence_fraction: float = 0.05
    attitude_ground_evidence_min_beams: int = 6
    attitude_evidence_max_abs_angle_rad: float = np.pi
    degraded_valid_fraction: float = 0.25
    min_retained_beams: int = 12


@dataclass(frozen=True)
class GroundFilterResult:
    keep_mask: np.ndarray
    ground_mask: np.ndarray
    offplane_mask: np.ndarray
    predicted_ground_ranges: np.ndarray
    elevation_rad: np.ndarray
    degraded: bool
    reason: str
    attitude_fallback: bool = False


@dataclass(frozen=True)
class DeskewResult:
    points: np.ndarray
    max_translation_m: float
    max_rotation_rad: float


class LevelReference:
    """Estimate a level base attitude only from stationary IMU samples."""

    def __init__(self, min_samples=20, max_gyro_rad_s=0.12,
                 gravity_mps2=9.80665, accel_tolerance_mps2=2.5):
        if min_samples < 2:
            raise ValueError('min_samples must be at least two')
        self.min_samples = int(min_samples)
        self.max_gyro = float(max_gyro_rad_s)
        self.gravity = float(gravity_mps2)
        self.accel_tolerance = float(accel_tolerance_mps2)
        self._samples = []
        self.quaternion = None

    @property
    def ready(self):
        return self.quaternion is not None

    @property
    def sample_count(self):
        return len(self._samples)

    def reset(self):
        self._samples.clear()
        self.quaternion = None

    def update(self, quaternion, angular_speed, acceleration_norm,
               wheel_speed=0.0, wheel_stationary_threshold=0.10):
        if self.ready:
            return True
        values = (angular_speed, acceleration_norm, wheel_speed)
        if not np.all(np.isfinite(values)):
            return False
        stationary = (
            abs(angular_speed) <= self.max_gyro
            and abs(acceleration_norm - self.gravity) <= self.accel_tolerance
            and abs(wheel_speed) <= wheel_stationary_threshold
        )
        if not stationary:
            self._samples.clear()
            return False
        q = normalize_quaternions(quaternion)
        if self._samples and np.dot(q, self._samples[0]) < 0.0:
            q = -q
        self._samples.append(q)
        if len(self._samples) >= self.min_samples:
            self.quaternion = normalize_quaternions(
                np.mean(self._samples, axis=0))
        return self.ready


def _component_runs(mask, beam_indices):
    positions = np.flatnonzero(mask)
    if not len(positions):
        return []
    indices = np.asarray(beam_indices, dtype=np.int64)
    selected_steps = np.diff(indices)
    nominal_step = int(np.median(selected_steps[selected_steps > 0])) \
        if np.any(selected_steps > 0) else 1
    breaks = np.flatnonzero(
        (np.diff(positions) > 1)
        | (np.diff(indices[positions]) > max(1, nominal_step))) + 1
    return np.split(positions, breaks)


def mask_ground_and_offplane(ranges, angles, beam_indices,
                             base_quaternions, level_quaternion,
                             base_to_laser_translation,
                             base_to_laser_quaternion,
                             config=GroundFilterConfig()):
    """Mask only continuous returns matching the predicted floor intersection."""
    ranges = np.asarray(ranges, dtype=np.float64)
    angles = np.asarray(angles, dtype=np.float64)
    indices = np.asarray(beam_indices, dtype=np.int64)
    base_q = normalize_quaternions(base_quaternions)
    if not (ranges.ndim == angles.ndim == indices.ndim == 1
            and len(ranges) == len(angles) == len(indices) == len(base_q)):
        raise ValueError('ranges, angles, indices, and attitudes must align')
    valid = np.isfinite(ranges) & (ranges > 0.0)
    if not len(ranges):
        empty = np.zeros(0, dtype=bool)
        return GroundFilterResult(empty, empty, empty,
                                  np.empty(0), np.empty(0), True, 'empty')

    relative = level_relative_matrices(base_q, level_quaternion)
    laser_rotation = quaternion_to_matrix(base_to_laser_quaternion)
    ray_laser = np.stack((np.cos(angles), np.sin(angles),
                          np.zeros(len(angles))), axis=1)
    ray_base = ray_laser @ laser_rotation.T
    ray_base_azimuth = np.arctan2(ray_base[:, 1], ray_base[:, 0])
    origin_base = np.asarray(base_to_laser_translation,
                             dtype=np.float64).reshape(3)

    def evaluate(attitude_matrices):
        ray_level = np.einsum('nij,nj->ni', attitude_matrices, ray_base)
        origin_level = np.einsum(
            'nij,j->ni', attitude_matrices, origin_base)
        elevation = np.arctan2(
            ray_level[:, 2], np.hypot(ray_level[:, 0], ray_level[:, 1]))
        predicted = np.full(len(ranges), np.nan, dtype=np.float64)
        downward = ray_level[:, 2] < -1e-5
        predicted[downward] = (
            (config.ground_z_m - origin_level[downward, 2])
            / ray_level[downward, 2])
        endpoint_z = origin_level[:, 2] + ranges * ray_level[:, 2]
        tolerance = np.maximum(
            config.range_tolerance_m,
            config.range_tolerance_ratio * predicted,
        )
        candidate = (
            valid & downward & np.isfinite(predicted) & (predicted > 0.0)
            & (np.abs(ranges - predicted) <= tolerance)
            & (np.abs(endpoint_z - config.ground_z_m)
               <= config.endpoint_z_tolerance_m)
        )
        ground = np.zeros(len(ranges), dtype=bool)
        endpoints = origin_level + ranges[:, None] * ray_level
        for component in _component_runs(candidate, indices):
            if len(component) < config.min_component_beams:
                continue
            angle_span = abs(float(
                angles[component[-1]] - angles[component[0]]))
            span = float(np.linalg.norm(
                endpoints[component[-1], :2]
                - endpoints[component[0], :2]))
            if (angle_span >= config.min_component_angle_rad
                    and span >= config.min_component_span_m):
                ground[component] = True
        offplane = np.abs(elevation) > config.max_planar_elevation_rad
        keep = valid & ~ground & ~offplane
        return keep, ground, offplane, predicted, elevation, endpoints

    keep, ground, offplane, predicted, elevation, endpoints = evaluate(relative)
    attitude_tilt = np.arccos(np.clip(relative[:, 2, 2], -1.0, 1.0))
    excessive_tilt = (
        float(np.median(attitude_tilt))
        > config.attitude_fallback_tilt_rad)
    evidence_sector = (
        np.abs(ray_base_azimuth)
        <= config.attitude_evidence_max_abs_angle_rad)
    evidence_beams = max(1, int(np.count_nonzero(evidence_sector)))
    evidence_valid_count = int(np.count_nonzero(valid & evidence_sector))
    evidence_ground_count = int(np.count_nonzero(ground & evidence_sector))
    evidence_ground_fraction = (
        evidence_ground_count / float(evidence_valid_count)
        if evidence_valid_count else 0.0)
    evidence_ground_continuous = any(
        len(component) >= config.attitude_ground_evidence_min_beams
        and abs(float(angles[component[-1]] - angles[component[0]]))
        >= config.min_component_angle_rad
        and float(np.linalg.norm(
            endpoints[component[-1], :2]
            - endpoints[component[0], :2])) >= config.min_component_span_m
        for component in _component_runs(ground & evidence_sector, indices)
    )
    evidence_valid_fraction = evidence_valid_count / float(evidence_beams)
    evidence_offplane_fraction = (
        np.count_nonzero(offplane & evidence_sector) / float(evidence_beams))
    ground_evidence = (
        evidence_ground_continuous
        and evidence_ground_fraction
        >= config.attitude_ground_evidence_fraction)
    sky_evidence = (
        evidence_valid_fraction <= config.degraded_valid_fraction
        and evidence_offplane_fraction
        >= config.degraded_offplane_fraction)
    attitude_fallback = bool(
        excessive_tilt and not ground_evidence and not sky_evidence)
    if attitude_fallback:
        # VESC AHRS roll/pitch is acceleration-sensitive in race data. If a
        # large reported tilt has no scan evidence, retain the measured static
        # LiDAR extrinsic instead of dropping most of the 2D scan.
        level_attitude = np.tile(np.eye(3), (len(ranges), 1, 1))
        keep, ground, offplane, predicted, elevation, endpoints = evaluate(
            level_attitude)
        evidence_ground_count = int(np.count_nonzero(
            ground & evidence_sector))
        evidence_ground_fraction = (
            evidence_ground_count / float(evidence_valid_count)
            if evidence_valid_count else 0.0)
        evidence_offplane_fraction = (
            np.count_nonzero(offplane & evidence_sector)
            / float(evidence_beams))

    retained_count = int(np.count_nonzero(keep))
    degraded = False
    reason = 'attitude_fallback' if attitude_fallback else 'ok'
    if sky_evidence and not attitude_fallback:
        degraded, reason = True, 'offplane_low_returns'
    elif retained_count < config.min_retained_beams:
        degraded, reason = True, 'too_few_retained'
    elif (evidence_offplane_fraction
          >= config.degraded_offplane_fraction):
        degraded, reason = True, 'offplane'
    elif evidence_ground_fraction >= config.degraded_ground_fraction:
        degraded, reason = True, 'ground_dominated'
    return GroundFilterResult(
        keep, ground, offplane, predicted, elevation, degraded, reason,
        attitude_fallback)


def deskew_points_to_reference(points_laser, beam_poses,
                               reference_pose, base_quaternions,
                               reference_base_quaternion,
                               level_quaternion,
                               base_to_laser_translation,
                               base_to_laser_quaternion,
                               translation_scale=1.0):
    """Express per-beam LiDAR endpoints in the reference-time LiDAR frame."""
    points = np.asarray(points_laser, dtype=np.float64)
    poses = np.asarray(beam_poses, dtype=np.float64)
    ref_pose = np.asarray(reference_pose, dtype=np.float64).reshape(3)
    if points.ndim != 2 or points.shape[1] not in (2, 3):
        raise ValueError('points_laser must be shaped (N, 2) or (N, 3)')
    if poses.shape != (len(points), 3):
        raise ValueError('beam_poses must be shaped (N, 3)')
    if not 0.0 <= translation_scale <= 1.0:
        raise ValueError('translation_scale must be in [0, 1]')
    if points.shape[1] == 2:
        points = np.column_stack((points, np.zeros(len(points))))

    rotations = rotations_from_yaw_and_level_tilt(
        poses[:, 2], base_quaternions, level_quaternion)
    ref_rotation = rotations_from_yaw_and_level_tilt(
        np.array([ref_pose[2]]),
        np.asarray(reference_base_quaternion).reshape(1, 4),
        level_quaternion,
    )[0]
    laser_rotation = quaternion_to_matrix(base_to_laser_quaternion)
    laser_offset = np.asarray(base_to_laser_translation,
                              dtype=np.float64).reshape(3)

    relative_xy = poses[:, :2] - ref_pose[:2]
    beam_base_positions = np.column_stack((
        ref_pose[0] + translation_scale * relative_xy[:, 0],
        ref_pose[1] + translation_scale * relative_xy[:, 1],
        np.zeros(len(points)),
    ))
    endpoint_base = laser_offset + points @ laser_rotation.T
    endpoint_world = beam_base_positions + np.einsum(
        'nij,nj->ni', rotations, endpoint_base)
    reference_laser_world = np.array([ref_pose[0], ref_pose[1], 0.0]) \
        + ref_rotation @ laser_offset
    reference_laser_rotation = ref_rotation @ laser_rotation
    output = (endpoint_world - reference_laser_world) \
        @ reference_laser_rotation

    max_translation = float(np.max(np.linalg.norm(relative_xy, axis=1))) \
        if len(points) else 0.0
    relative_rotation = np.einsum(
        'ji,njk->nik', ref_rotation, rotations)
    traces = np.trace(relative_rotation, axis1=1, axis2=2)
    max_rotation = float(np.max(np.arccos(np.clip(
        (traces - 1.0) * 0.5, -1.0, 1.0)))) if len(points) else 0.0
    return DeskewResult(output, max_translation, max_rotation)
