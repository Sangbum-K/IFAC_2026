# Copyright 2026 IFAC F1TENTH Team
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in
# all copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.

"""
Pure NumPy primitives used by the particle filter critical path.

This module intentionally has no ROS or RangeLib dependency.  The production
node and unit tests therefore exercise the same resampling, beam-selection,
weighting, and localization-quality state machine.
"""

from dataclasses import dataclass
from enum import Enum

import numpy as np


@dataclass(frozen=True)
class ProposalResult:
    """Particle proposal and its importance prior for one MCL update."""

    particles: np.ndarray
    prior_weights: np.ndarray
    resampled: bool
    prior_ess_ratio: float


@dataclass(frozen=True)
class BeamSelection:
    """Result of AMCL-style dynamic-beam rejection."""

    mask: np.ndarray
    agreement_fraction: np.ndarray
    skipped_fraction: float
    fallback_to_all_beams: bool
    best_particle_inlier_fraction: float
    weighted_inlier_fraction: float


@dataclass(frozen=True)
class ScanMotionEstimate:
    """Robust scan-to-scan planar motion estimate."""

    valid: bool
    action: np.ndarray
    rmse_m: float
    inlier_fraction: float


@dataclass(frozen=True)
class DeskewResult:
    """One scan represented at a common LiDAR reference timestamp."""

    ranges: np.ndarray
    angles: np.ndarray
    max_translation_m: float
    max_yaw_rad: float


@dataclass(frozen=True)
class SlipUpdate:
    """One hysteretic wheel/IMU/scan innovation update."""

    active: bool
    entered: bool
    recovered: bool
    evidence_valid: bool
    score: float
    translation_innovation_m: float
    scan_odom_yaw_innovation_rad: float
    odom_imu_yaw_innovation_rad: float
    scan_imu_yaw_innovation_rad: float


class QualityState(str, Enum):
    """Externally meaningful localization health states."""

    STARTUP = 'STARTUP'
    OK = 'OK'
    DEGRADED = 'DEGRADED'
    LOST = 'LOST'


@dataclass(frozen=True)
class QualityUpdate:
    """One quality-state transition result."""

    state: QualityState
    transitioned: bool
    inlier_fraction: float
    consecutive_lost: int
    consecutive_good: int


def normalize_weights(weights):
    """Return finite normalized weights, or ``None`` for invalid input."""
    values = np.asarray(weights, dtype=np.float64)
    if values.ndim != 1 or values.size == 0 or not np.all(np.isfinite(values)):
        return None
    if np.any(values < 0.0):
        return None
    total = float(np.sum(values))
    if not np.isfinite(total) or total <= 1.0e-300:
        return None
    return values / total


def effective_sample_size(weights):
    """Return ``1 / sum(w^2)`` after robust normalization."""
    normalized = normalize_weights(weights)
    if normalized is None:
        return 0.0
    squared_sum = float(np.dot(normalized, normalized))
    if not np.isfinite(squared_sum) or squared_sum <= 0.0:
        return 0.0
    return 1.0 / squared_sum


def systematic_resample(weights, rng=None):
    """Draw low-variance systematic-resampling indices in O(N)."""
    normalized = normalize_weights(weights)
    if normalized is None:
        raise ValueError(
            'systematic_resample requires valid non-negative weights')

    generator = rng if rng is not None else np.random.default_rng()
    count = normalized.size
    start = float(generator.random()) / count
    positions = start + np.arange(count, dtype=np.float64) / count
    cumulative = np.cumsum(normalized)
    cumulative[-1] = 1.0
    return np.searchsorted(cumulative, positions, side='right')


def prepare_mcl_proposal(particles, weights, ess_ratio_threshold, rng=None):
    """Copy particles and resample only when the prior ESS is too low."""
    particle_array = np.asarray(particles)
    normalized = normalize_weights(weights)
    if particle_array.ndim != 2 or particle_array.shape[0] == 0:
        raise ValueError('particles must be a non-empty 2-D array')
    if normalized is None or normalized.size != particle_array.shape[0]:
        raise ValueError('weights must match particles and be normalizable')
    if not 0.0 < ess_ratio_threshold <= 1.0:
        raise ValueError('ess_ratio_threshold must be in (0, 1]')

    count = particle_array.shape[0]
    ess_ratio = effective_sample_size(normalized) / count
    if ess_ratio < ess_ratio_threshold:
        indices = systematic_resample(normalized, rng=rng)
        return ProposalResult(
            particles=np.array(particle_array[indices], copy=True),
            prior_weights=np.full(count, 1.0 / count, dtype=np.float64),
            resampled=True,
            prior_ess_ratio=float(ess_ratio),
        )
    return ProposalResult(
        particles=np.array(particle_array, copy=True),
        prior_weights=normalized,
        resampled=False,
        prior_ess_ratio=float(ess_ratio),
    )


def posterior_from_likelihood(prior_weights, likelihoods):
    """Fuse an importance prior with non-negative sensor likelihoods."""
    prior = normalize_weights(prior_weights)
    likelihood = np.asarray(likelihoods, dtype=np.float64)
    if prior is None or likelihood.shape != prior.shape:
        return None
    if not np.all(np.isfinite(likelihood)) or np.any(likelihood < 0.0):
        return None
    return normalize_weights(prior * likelihood)


def estimate_scan_motion(
        previous_ranges, current_ranges, angles, initial_action,
        maximum_range_m, max_correspondence_distance_m=0.35,
        trim_fraction=0.90, iterations=4, min_points=12,
        current_angles=None):
    """
    Estimate current LiDAR motion in the previous LiDAR frame using ICP.

    The point count is intentionally the already-downsampled PF beam count. A
    brute-force nearest-neighbor pass is therefore small, deterministic, and
    avoids adding a SciPy/KD-tree dependency to the Jetson runtime.
    """
    previous = np.asarray(previous_ranges, dtype=np.float64)
    current = np.asarray(current_ranges, dtype=np.float64)
    previous_angles = np.asarray(angles, dtype=np.float64)
    current_beam_angles = np.asarray(
        angles if current_angles is None else current_angles,
        dtype=np.float64,
    )
    pose = np.asarray(initial_action, dtype=np.float64).copy()
    if previous.ndim != 1 or current.shape != previous.shape:
        raise ValueError('scan ranges must be equal-length 1-D arrays')
    if (previous_angles.shape != previous.shape
            or current_beam_angles.shape != current.shape):
        raise ValueError('angles must match their scan ranges')
    if pose.shape != (3,) or not np.all(np.isfinite(pose)):
        raise ValueError('initial_action must be a finite planar pose')
    if not np.isfinite(maximum_range_m) or maximum_range_m <= 0.0:
        raise ValueError('maximum_range_m must be positive')
    if (not np.isfinite(max_correspondence_distance_m)
            or max_correspondence_distance_m <= 0.0):
        raise ValueError('max_correspondence_distance_m must be positive')
    if not 0.0 < trim_fraction <= 1.0:
        raise ValueError('trim_fraction must be in (0.0, 1.0]')
    if iterations <= 0 or min_points < 3:
        raise ValueError('iterations and min_points must be positive')

    usable_limit = maximum_range_m * 0.995
    previous_valid = np.logical_and.reduce((
        np.isfinite(previous), previous > 0.05, previous < usable_limit,
    ))
    current_valid = np.logical_and.reduce((
        np.isfinite(current), current > 0.05, current < usable_limit,
    ))
    if (np.count_nonzero(previous_valid) < min_points
            or np.count_nonzero(current_valid) < min_points):
        return ScanMotionEstimate(False, pose, np.inf, 0.0)

    target = np.column_stack((
        previous[previous_valid] * np.cos(previous_angles[previous_valid]),
        previous[previous_valid] * np.sin(previous_angles[previous_valid]),
    ))
    source = np.column_stack((
        current[current_valid]
        * np.cos(current_beam_angles[current_valid]),
        current[current_valid]
        * np.sin(current_beam_angles[current_valid]),
    ))

    selected = None
    nearest_distances = None
    for _ in range(int(iterations)):
        transformed = _transform_points(source, pose)
        residuals = transformed[:, np.newaxis, :] - target[np.newaxis, :, :]
        squared_distances = np.einsum(
            'ijk,ijk->ij', residuals, residuals)
        nearest_indices = np.argmin(squared_distances, axis=1)
        nearest_distances = np.sqrt(
            squared_distances[np.arange(source.shape[0]), nearest_indices])
        in_range = nearest_distances <= max_correspondence_distance_m
        if np.count_nonzero(in_range) < min_points:
            return ScanMotionEstimate(False, pose, np.inf, 0.0)

        trim_limit = np.quantile(nearest_distances[in_range], trim_fraction)
        selected = np.logical_and(in_range, nearest_distances <= trim_limit)
        if np.count_nonzero(selected) < min_points:
            return ScanMotionEstimate(False, pose, np.inf, 0.0)

        source_points = transformed[selected]
        target_points = target[nearest_indices[selected]]
        source_center = np.mean(source_points, axis=0)
        target_center = np.mean(target_points, axis=0)
        covariance = (
            (source_points - source_center).T
            @ (target_points - target_center)
        )
        left, _, right_transpose = np.linalg.svd(covariance)
        rotation = right_transpose.T @ left.T
        if np.linalg.det(rotation) < 0.0:
            right_transpose[-1, :] *= -1.0
            rotation = right_transpose.T @ left.T
        translation = target_center - rotation @ source_center
        delta = np.array([
            translation[0],
            translation[1],
            np.arctan2(rotation[1, 0], rotation[0, 0]),
        ])
        pose = _compose_pose(delta, pose)

    transformed = _transform_points(source, pose)
    residuals = transformed[:, np.newaxis, :] - target[np.newaxis, :, :]
    squared_distances = np.einsum('ijk,ijk->ij', residuals, residuals)
    nearest_distances = np.sqrt(np.min(squared_distances, axis=1))
    in_range = nearest_distances <= max_correspondence_distance_m
    if np.count_nonzero(in_range) < min_points:
        return ScanMotionEstimate(False, pose, np.inf, 0.0)
    trim_limit = np.quantile(nearest_distances[in_range], trim_fraction)
    selected = np.logical_and(in_range, nearest_distances <= trim_limit)
    if np.count_nonzero(selected) < min_points:
        return ScanMotionEstimate(False, pose, np.inf, 0.0)
    return ScanMotionEstimate(
        valid=True,
        action=pose,
        rmse_m=float(np.sqrt(np.mean(nearest_distances[selected] ** 2))),
        inlier_fraction=float(np.mean(in_range)),
    )


def deskew_scan_to_reference(
        ranges, angles, relative_base_actions, yaw_offsets,
        base_to_laser_pose, translation_scale, maximum_range_m):
    """Transform selected beam endpoints into one reference LiDAR frame."""
    beam_ranges = np.asarray(ranges, dtype=np.float64)
    beam_angles = np.asarray(angles, dtype=np.float64)
    base_actions = np.asarray(relative_base_actions, dtype=np.float64)
    yaw_deltas = np.asarray(yaw_offsets, dtype=np.float64)
    base_to_laser = np.asarray(base_to_laser_pose, dtype=np.float64)
    count = beam_ranges.size
    if beam_ranges.ndim != 1 or beam_angles.shape != beam_ranges.shape:
        raise ValueError('ranges and angles must be equal-length 1-D arrays')
    if base_actions.shape != (count, 3):
        raise ValueError('relative_base_actions must have shape [beams, 3]')
    if yaw_deltas.shape != beam_ranges.shape:
        raise ValueError('yaw_offsets must match beam ranges')
    if base_to_laser.shape != (3,):
        raise ValueError('base_to_laser_pose must contain x, y and yaw')
    if not np.all(np.isfinite(beam_ranges)) or np.any(beam_ranges < 0.0):
        raise ValueError('ranges must be finite and non-negative')
    if (not np.all(np.isfinite(beam_angles))
            or not np.all(np.isfinite(base_actions))
            or not np.all(np.isfinite(yaw_deltas))
            or not np.all(np.isfinite(base_to_laser))):
        raise ValueError('deskew poses and angles must be finite')
    if not 0.0 <= float(translation_scale) <= 1.0:
        raise ValueError('translation_scale must be in [0, 1]')
    if not np.isfinite(maximum_range_m) or maximum_range_m <= 0.0:
        raise ValueError('maximum_range_m must be positive')

    scale = float(translation_scale)
    base_x = base_actions[:, 0] * scale
    base_y = base_actions[:, 1] * scale
    extrinsic_x, extrinsic_y, extrinsic_yaw = base_to_laser
    extrinsic_cosine = np.cos(extrinsic_yaw)
    extrinsic_sine = np.sin(extrinsic_yaw)

    # Vectorized E^-1 * B(t_ref, t_beam) * E for all selected beams.
    inverse_x = (
        -extrinsic_cosine * extrinsic_x
        - extrinsic_sine * extrinsic_y)
    inverse_y = (
        extrinsic_sine * extrinsic_x
        - extrinsic_cosine * extrinsic_y)
    intermediate_x = (
        inverse_x + extrinsic_cosine * base_x
        + extrinsic_sine * base_y)
    intermediate_y = (
        inverse_y - extrinsic_sine * base_x
        + extrinsic_cosine * base_y)
    intermediate_yaw = _wrap_angle(yaw_deltas - extrinsic_yaw)
    intermediate_cosine = np.cos(intermediate_yaw)
    intermediate_sine = np.sin(intermediate_yaw)
    laser_x = (
        intermediate_x + intermediate_cosine * extrinsic_x
        - intermediate_sine * extrinsic_y)
    laser_y = (
        intermediate_y + intermediate_sine * extrinsic_x
        + intermediate_cosine * extrinsic_y)
    laser_yaw = _wrap_angle(intermediate_yaw + extrinsic_yaw)

    point_x = beam_ranges * np.cos(beam_angles)
    point_y = beam_ranges * np.sin(beam_angles)
    laser_cosine = np.cos(laser_yaw)
    laser_sine = np.sin(laser_yaw)
    corrected_x = laser_x + laser_cosine * point_x - laser_sine * point_y
    corrected_y = laser_y + laser_sine * point_x + laser_cosine * point_y
    corrected_ranges = np.clip(
        np.hypot(corrected_x, corrected_y), 0.0, maximum_range_m)
    corrected_angles = np.arctan2(corrected_y, corrected_x)

    no_hit = beam_ranges >= maximum_range_m * 0.995
    corrected_ranges[no_hit] = maximum_range_m
    corrected_angles[no_hit] = _wrap_angle(
        beam_angles[no_hit] + laser_yaw[no_hit])
    maximum_translation = (
        float(np.max(np.hypot(laser_x, laser_y))) if count else 0.0)
    maximum_yaw = (
        float(np.max(np.abs(laser_yaw))) if count else 0.0)

    return DeskewResult(
        ranges=corrected_ranges,
        angles=corrected_angles,
        max_translation_m=maximum_translation,
        max_yaw_rad=maximum_yaw,
    )


def relative_pose_batch(reference_pose, poses):
    """Return ``inverse(reference_pose) * poses`` for an N-by-3 array."""
    reference = np.asarray(reference_pose, dtype=np.float64)
    values = np.asarray(poses, dtype=np.float64)
    if reference.shape != (3,) or not np.all(np.isfinite(reference)):
        raise ValueError('reference_pose must be a finite planar pose')
    if values.ndim != 2 or values.shape[1:] != (3,):
        raise ValueError('poses must have shape [N, 3]')
    if not np.all(np.isfinite(values)):
        raise ValueError('poses must be finite')

    delta_x = values[:, 0] - reference[0]
    delta_y = values[:, 1] - reference[1]
    cosine = np.cos(reference[2])
    sine = np.sin(reference[2])
    return np.column_stack((
        cosine * delta_x + sine * delta_y,
        -sine * delta_x + cosine * delta_y,
        _wrap_angle(values[:, 2] - reference[2]),
    ))


def should_hold_stationary_filter(
        enabled, speed, low_speed_threshold, action, imu_stationary,
        scan_motion, scan_motion_valid, max_scan_translation_m,
        max_scan_yaw_rad, epsilon=1e-12):
    """Require wheel, IMU, and LiDAR agreement before freezing the PF."""
    wheel_motion = np.asarray(action, dtype=np.float64)
    lidar_motion = np.asarray(scan_motion, dtype=np.float64)
    if wheel_motion.shape != (3,) or lidar_motion.shape != (3,):
        raise ValueError('actions must contain x, y and yaw')
    return bool(
        enabled
        and abs(float(speed)) < float(low_speed_threshold)
        and np.hypot(wheel_motion[0], wheel_motion[1]) <= epsilon
        and abs(wheel_motion[2]) <= epsilon
        and bool(imu_stationary)
        and bool(scan_motion_valid)
        and np.hypot(lidar_motion[0], lidar_motion[1])
        <= float(max_scan_translation_m)
        and abs(lidar_motion[2]) <= float(max_scan_yaw_rad)
    )


def can_bypass_jump_guard_for_slip(
        slip_active, scan_fit, minimum_scan_fit, correction_translation_m,
        correction_yaw_rad, maximum_translation_m, maximum_yaw_rad):
    """Allow only bounded PF corrections backed by a strong map match."""
    return bool(
        slip_active
        and float(scan_fit) >= float(minimum_scan_fit)
        and float(correction_translation_m) <= float(maximum_translation_m)
        and float(correction_yaw_rad) <= float(maximum_yaw_rad)
    )


class SlipDetector:
    """Hysteretic detector for disagreement among odom, IMU, and LiDAR."""

    def __init__(
            self, translation_threshold_m, yaw_threshold_rad,
            enter_score, recover_score, enter_updates, recover_updates,
            immediate_score=np.inf):
        """Configure normalized innovation thresholds and hysteresis."""
        if translation_threshold_m <= 0.0 or yaw_threshold_rad <= 0.0:
            raise ValueError('slip innovation thresholds must be positive')
        if not 0.0 <= recover_score < enter_score:
            raise ValueError('slip scores must satisfy 0 <= recover < enter')
        if immediate_score < enter_score:
            raise ValueError('immediate_score must be >= enter_score')
        if enter_updates <= 0 or recover_updates <= 0:
            raise ValueError('slip hysteresis counts must be positive')
        self.translation_threshold_m = float(translation_threshold_m)
        self.yaw_threshold_rad = float(yaw_threshold_rad)
        self.enter_score = float(enter_score)
        self.recover_score = float(recover_score)
        self.immediate_score = float(immediate_score)
        self.enter_updates = int(enter_updates)
        self.recover_updates = int(recover_updates)
        self.reset()

    def reset(self):
        """Clear active state and consecutive evidence."""
        self.active = False
        self.consecutive_slip = 0
        self.consecutive_recovery = 0

    def update(
            self, odom_action, imu_delta_yaw, imu_valid,
            scan_action, scan_valid):
        """Consume one timestamp-aligned multi-sensor innovation sample."""
        odom = np.asarray(odom_action, dtype=np.float64)
        scan = np.asarray(scan_action, dtype=np.float64)
        if odom.shape != (3,) or scan.shape != (3,):
            raise ValueError('odom_action and scan_action must be planar poses')

        translation = 0.0
        scan_odom_yaw = 0.0
        odom_imu_yaw = 0.0
        scan_imu_yaw = 0.0
        component_scores = []
        if scan_valid:
            translation = float(np.hypot(
                scan[0] - odom[0], scan[1] - odom[1]))
            scan_odom_yaw = abs(float(_wrap_angle(scan[2] - odom[2])))
            component_scores.extend((
                translation / self.translation_threshold_m,
                scan_odom_yaw / self.yaw_threshold_rad,
            ))
        if imu_valid:
            imu_yaw = float(imu_delta_yaw)
            odom_imu_yaw = abs(float(_wrap_angle(odom[2] - imu_yaw)))
            component_scores.append(
                odom_imu_yaw / self.yaw_threshold_rad)
            if scan_valid:
                scan_imu_yaw = abs(float(_wrap_angle(scan[2] - imu_yaw)))
                component_scores.append(
                    scan_imu_yaw / self.yaw_threshold_rad)

        evidence_valid = bool(component_scores)
        score = float(max(component_scores, default=0.0))
        entered = False
        recovered = False
        recovery_evidence_valid = bool(scan_valid and imu_valid)
        if self.active:
            if recovery_evidence_valid and score <= self.recover_score:
                self.consecutive_recovery += 1
                if self.consecutive_recovery >= self.recover_updates:
                    self.active = False
                    recovered = True
                    self.consecutive_recovery = 0
            else:
                self.consecutive_recovery = 0
            self.consecutive_slip = 0
        elif evidence_valid:
            if score >= self.immediate_score:
                self.active = True
                entered = True
                self.consecutive_slip = 0
            elif score >= self.enter_score:
                self.consecutive_slip += 1
                if self.consecutive_slip >= self.enter_updates:
                    self.active = True
                    entered = True
                    self.consecutive_slip = 0
            else:
                self.consecutive_slip = 0
            self.consecutive_recovery = 0

        return SlipUpdate(
            active=self.active,
            entered=entered,
            recovered=recovered,
            evidence_valid=evidence_valid,
            score=score,
            translation_innovation_m=translation,
            scan_odom_yaw_innovation_rad=scan_odom_yaw,
            odom_imu_yaw_innovation_rad=odom_imu_yaw,
            scan_imu_yaw_innovation_rad=scan_imu_yaw,
        )


def _transform_points(points, pose):
    cosine = np.cos(pose[2])
    sine = np.sin(pose[2])
    rotation = np.array([[cosine, -sine], [sine, cosine]])
    return points @ rotation.T + pose[:2]


def _compose_pose(lhs, rhs):
    cosine = np.cos(lhs[2])
    sine = np.sin(lhs[2])
    return np.array([
        lhs[0] + cosine * rhs[0] - sine * rhs[1],
        lhs[1] + sine * rhs[0] + cosine * rhs[1],
        _wrap_angle(lhs[2] + rhs[2]),
    ])


def _inverse_pose(pose):
    cosine = np.cos(pose[2])
    sine = np.sin(pose[2])
    return np.array([
        -cosine * pose[0] - sine * pose[1],
        sine * pose[0] - cosine * pose[1],
        _wrap_angle(-pose[2]),
    ])


def _wrap_angle(theta):
    return np.arctan2(np.sin(theta), np.cos(theta))


def select_beams(
        observed_px, predicted_px, distance_px,
        min_particle_agreement, max_skipped_fraction, enabled=True,
        particle_weights=None, short_only=False, required_mask=None):
    """
    Select static-map beams and retain all beams when localization is lost.

    A beam is skipped when too few particles predict the observation within
    ``distance_px``.  If that would discard too much of the scan, all beams are
    restored.  The fallback is essential: otherwise a globally wrong particle
    cloud could explain its failure by skipping every beam.
    """
    observed = np.asarray(observed_px, dtype=np.int32)
    predicted = np.asarray(predicted_px)
    if observed.ndim != 1 or predicted.ndim != 2:
        raise ValueError(
            'observed_px must be 1-D and predicted_px must be 2-D')
    if predicted.shape[1] != observed.size or predicted.shape[0] == 0:
        raise ValueError(
            'predicted_px shape must be [particles, observed beams]')
    if distance_px < 0:
        raise ValueError('distance_px must be non-negative')
    if not 0.0 <= min_particle_agreement <= 1.0:
        raise ValueError('min_particle_agreement must be in [0, 1]')
    if not 0.0 <= max_skipped_fraction < 1.0:
        raise ValueError('max_skipped_fraction must be in [0, 1)')
    if particle_weights is None:
        normalized_weights = np.full(
            predicted.shape[0], 1.0 / predicted.shape[0])
    else:
        normalized_weights = normalize_weights(particle_weights)
        if (normalized_weights is None
                or normalized_weights.size != predicted.shape[0]):
            raise ValueError(
                'particle_weights must match predicted particles')

    residual = np.abs(
        predicted.astype(np.int32, copy=False) - observed[None, :])
    inlier = residual <= int(distance_px)
    if short_only:
        # Moving objects can only make a valid map return shorter. Retain long
        # mismatches so map/extrinsic errors remain visible to MCL and LOST.
        eligible = predicted.astype(
            np.int32, copy=False) <= observed[None, :] + int(distance_px)
        agreement = np.dot(normalized_weights, eligible)
    else:
        agreement = np.dot(normalized_weights, inlier)
    if required_mask is None:
        required = np.ones(observed.size, dtype=bool)
    else:
        required = np.asarray(required_mask, dtype=bool)
        if required.shape != observed.shape:
            raise ValueError('required_mask must match observed beams')
    mask = np.copy(required)
    fallback = False
    skipped_fraction = 0.0
    if enabled:
        mask = required & (agreement >= min_particle_agreement)
        required_count = int(np.count_nonzero(required))
        selected_count = int(np.count_nonzero(mask))
        skipped_fraction = (
            1.0 - selected_count / float(required_count)
            if required_count else 1.0)
        fallback = selected_count == 0 or skipped_fraction > max_skipped_fraction
        if fallback:
            # Dynamic-beam fallback must never restore a beam rejected by the
            # independent 3D ground/off-plane geometry gate.
            mask = np.copy(required)

    selected_inliers = inlier[:, mask]
    best_inlier = (
        float(np.max(np.mean(selected_inliers, axis=1)))
        if selected_inliers.shape[1] else 0.0
    )
    particle_inlier = np.mean(selected_inliers, axis=1)
    weighted_inlier = float(np.dot(normalized_weights, particle_inlier))
    return BeamSelection(
        mask=mask,
        agreement_fraction=agreement,
        skipped_fraction=skipped_fraction,
        fallback_to_all_beams=fallback,
        best_particle_inlier_fraction=best_inlier,
        weighted_inlier_fraction=weighted_inlier,
    )


def likelihoods_from_log_table(
        log_table, observed_px, predicted_px, beam_mask, squash_factor):
    """Evaluate stable beam likelihoods from a precomputed log table."""
    table = np.asarray(log_table)
    observed = np.asarray(observed_px)
    predicted = np.asarray(predicted_px)
    mask = np.asarray(beam_mask, dtype=bool)
    if table.ndim != 2 or observed.ndim != 1 or predicted.ndim != 2:
        raise ValueError('invalid sensor likelihood array dimensions')
    if predicted.shape[1] != observed.size or mask.shape != observed.shape:
        raise ValueError('beam dimensions do not match')
    if not np.any(mask):
        raise ValueError('at least one beam must be selected')
    if not np.isfinite(squash_factor) or squash_factor <= 0.0:
        raise ValueError('squash_factor must be positive')

    selected_observed = observed[mask]
    selected_predicted = predicted[:, mask]
    log_values = table[selected_observed[None, :], selected_predicted]
    log_weights = np.sum(log_values, axis=1, dtype=np.float64) * squash_factor
    maximum = float(np.max(log_weights))
    if not np.isfinite(maximum):
        return np.zeros(predicted.shape[0], dtype=np.float64)
    return np.exp(log_weights - maximum)


class LocalizationQualityTracker:
    """Hysteretic LOST detector driven by scan-to-map inlier support."""

    def __init__(
            self, ok_inlier_fraction, lost_inlier_fraction,
            bad_updates_to_lost, good_updates_to_recover):
        """Configure quality thresholds and transition hysteresis."""
        if not 0.0 <= lost_inlier_fraction < ok_inlier_fraction <= 1.0:
            raise ValueError(
                'quality thresholds must satisfy 0 <= lost < ok <= 1')
        if bad_updates_to_lost <= 0 or good_updates_to_recover <= 0:
            raise ValueError('quality hysteresis counts must be positive')
        self.ok_inlier_fraction = float(ok_inlier_fraction)
        self.lost_inlier_fraction = float(lost_inlier_fraction)
        self.bad_updates_to_lost = int(bad_updates_to_lost)
        self.good_updates_to_recover = int(good_updates_to_recover)
        self.reset()

    def reset(self):
        """Return to startup without carrying stale quality evidence."""
        self.state = QualityState.STARTUP
        self.consecutive_lost = 0
        self.consecutive_good = 0

    def update(self, inlier_fraction):
        """Consume one scan score and update the quality state."""
        score = float(inlier_fraction)
        if not np.isfinite(score):
            score = 0.0
        score = float(np.clip(score, 0.0, 1.0))
        previous = self.state

        if score >= self.ok_inlier_fraction:
            self.consecutive_lost = 0
            self.consecutive_good += 1
            if self.consecutive_good >= self.good_updates_to_recover:
                self.state = QualityState.OK
        elif score < self.lost_inlier_fraction:
            self.consecutive_good = 0
            self.consecutive_lost += 1
            if self.consecutive_lost >= self.bad_updates_to_lost:
                self.state = QualityState.LOST
            elif self.state != QualityState.LOST:
                self.state = QualityState.DEGRADED
        else:
            self.consecutive_good = 0
            self.consecutive_lost = 0
            if self.state != QualityState.LOST:
                self.state = QualityState.DEGRADED

        return QualityUpdate(
            state=self.state,
            transitioned=self.state != previous,
            inlier_fraction=score,
            consecutive_lost=self.consecutive_lost,
            consecutive_good=self.consecutive_good,
        )


class LocalizationReadinessTracker:
    """
    Distinguish initial-pose receipt from application and latch readiness.

    A single DEGRADED scan must not command an abrupt stop at race speed. Once
    armed, readiness therefore remains latched through DEGRADED and is revoked
    only by LOST or an explicit filter reset. Re-arming still requires the
    quality tracker's full consecutive-good hysteresis before it reaches OK.
    """

    def __init__(self):
        """Start fail-closed until the PF applies an ``/initialpose`` message."""
        self.initial_pose_received = False
        self.initial_pose_accepted = False
        self.ready = False

    def receive_initial_pose(self):
        """Record a new request and disarm until that request is applied."""
        self.initial_pose_received = True
        self.initial_pose_accepted = False
        self.ready = False

    def accept_initial_pose(self):
        """Record a successfully applied initial pose and require fresh quality."""
        self.initial_pose_received = True
        self.initial_pose_accepted = True
        self.ready = False

    def clear_initial_pose(self):
        """Invalidate both the initial-pose evidence and autonomy readiness."""
        self.initial_pose_received = False
        self.initial_pose_accepted = False
        self.ready = False

    def reset_quality(self):
        """Keep the pose evidence but require consecutive-good scans again."""
        self.ready = False

    def update(self, quality_state):
        """Consume a quality state and return whether readiness transitioned."""
        state = QualityState(quality_state)
        previous = self.ready
        if not self.initial_pose_accepted or state in (
                QualityState.STARTUP, QualityState.LOST):
            self.ready = False
        elif state == QualityState.OK:
            self.ready = True
        # DEGRADED preserves the current latch: it cannot arm an unready PF and
        # it cannot cause a one-frame emergency stop after the car has started.
        return self.ready != previous
