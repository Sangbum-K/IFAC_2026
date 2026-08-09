#!/usr/bin/env python3
"""Replay baseline and candidate MCL deterministically on real driving bags."""

import argparse
import ctypes
from dataclasses import dataclass
from dataclasses import replace
import math
from pathlib import Path
import subprocess
import sys
import time

from analyze_pf_bags import map_fit_series
from analyze_pf_bags import moving_fit_summary

from compare_pf_replay import base_to_laser_pose
from compare_pf_replay import compose
from compare_pf_replay import correction_metrics
from compare_pf_replay import inverse
from compare_pf_replay import map_fit_metrics
from compare_pf_replay import nearest_by_stamp
from compare_pf_replay import new_update_poses
from compare_pf_replay import number
from compare_pf_replay import OdomInterpolator
from compare_pf_replay import pose_2d
from compare_pf_replay import read_bag
from compare_pf_replay import stamp_ns
from compare_pf_replay import text_table

import numpy as np


REPO_ROOT = Path(__file__).resolve().parents[1]
PF_SOURCE = REPO_ROOT / 'localization_ws' / 'src' / 'particle_filter'
sys.path.insert(0, str(PF_SOURCE))

from particle_filter.mcl_core import (  # noqa: E402
    deskew_scan_to_reference,
    effective_sample_size,
    estimate_scan_motion,
    likelihoods_from_log_table,
    LocalizationQualityTracker,
    posterior_from_likelihood,
    prepare_mcl_proposal,
    QualityState,
    relative_pose_batch,
    select_beams,
    should_hold_stationary_filter,
    SlipDetector,
)


RAYCAST_SOURCE = REPO_ROOT / 'tools' / 'offline_raycast.cpp'
RAYCAST_LIBRARY = Path('/tmp/ifac_offline_raycast.so')


def build_raycast_library():
    """Build the deterministic validation raycaster outside the repository."""
    if (RAYCAST_LIBRARY.exists()
            and RAYCAST_LIBRARY.stat().st_mtime
            >= RAYCAST_SOURCE.stat().st_mtime):
        return
    subprocess.run([
        'g++', '-O3', '-march=native', '-fPIC', '-shared', '-fopenmp',
        str(RAYCAST_SOURCE), '-o', str(RAYCAST_LIBRARY),
    ], check=True)


class DdaRaycaster:
    """ctypes wrapper around an exact occupancy-grid DDA raycaster."""

    def __init__(self, map_message):
        """Load occupancy data and map geometry."""
        build_raycast_library()
        self.library = ctypes.CDLL(str(RAYCAST_LIBRARY))
        self.function = self.library.raycast_many
        float_pointer = ctypes.POINTER(ctypes.c_float)
        byte_pointer = ctypes.POINTER(ctypes.c_int8)
        self.function.argtypes = [
            byte_pointer,
            ctypes.c_int,
            ctypes.c_int,
            ctypes.c_float,
            ctypes.c_float,
            ctypes.c_float,
            ctypes.c_float,
            float_pointer,
            ctypes.c_int,
            float_pointer,
            ctypes.c_int,
            ctypes.c_float,
            float_pointer,
        ]
        self.function.restype = None
        self.map = np.ascontiguousarray(
            np.asarray(map_message.data, dtype=np.int8))
        self.width = int(map_message.info.width)
        self.height = int(map_message.info.height)
        self.resolution = float(map_message.info.resolution)
        origin = pose_2d(map_message.info.origin)
        self.origin_x = float(origin[0])
        self.origin_y = float(origin[1])
        self.origin_yaw = float(origin[2])

    def cast(self, poses, angles, maximum_range):
        """Cast every relative angle from every world-frame pose."""
        pose_array = np.ascontiguousarray(poses, dtype=np.float32)
        angle_array = np.ascontiguousarray(angles, dtype=np.float32)
        ranges = np.empty(
            (len(pose_array), len(angle_array)), dtype=np.float32)
        self.function(
            self.map.ctypes.data_as(ctypes.POINTER(ctypes.c_int8)),
            self.width,
            self.height,
            self.resolution,
            self.origin_x,
            self.origin_y,
            self.origin_yaw,
            pose_array.ctypes.data_as(ctypes.POINTER(ctypes.c_float)),
            len(pose_array),
            angle_array.ctypes.data_as(ctypes.POINTER(ctypes.c_float)),
            len(angle_array),
            float(maximum_range),
            ranges.ctypes.data_as(ctypes.POINTER(ctypes.c_float)),
        )
        return ranges


def sensor_log_table(maximum_range_px):
    """Build the same discretized beam model used by the ROS PF node."""
    width = int(maximum_range_px) + 1
    observed = np.arange(width, dtype=np.float64)[:, np.newaxis]
    predicted = np.arange(width, dtype=np.float64)[np.newaxis, :]
    delta = observed - predicted
    probabilities = (
        0.75 * np.exp(-(delta * delta) / (2.0 * 4.0 * 4.0))
        / (4.0 * np.sqrt(2.0 * np.pi))
    )
    short_mask = observed < predicted
    safe_predicted = np.maximum(predicted, 1.0)
    probabilities += np.where(
        short_mask,
        2.0 * 0.01 * (predicted - observed) / safe_predicted,
        0.0,
    )
    probabilities[-1, :] += 0.07
    probabilities[:-1, :] += 0.12 / maximum_range_px
    probabilities /= np.sum(probabilities, axis=0, keepdims=True)
    return np.log(np.maximum(probabilities, 1e-12)).astype(np.float32)


def weighted_pose(particles, weights):
    """Return weighted x, y and circular-yaw particle mean."""
    xy = np.dot(particles[:, :2].T, weights)
    yaw = math.atan2(
        float(np.dot(np.sin(particles[:, 2]), weights)),
        float(np.dot(np.cos(particles[:, 2]), weights)),
    )
    return np.array([xy[0], xy[1], yaw], dtype=np.float64)


def wrap_angle(values):
    """Wrap scalar or array angles to [-pi, pi]."""
    return np.arctan2(np.sin(values), np.cos(values))


@dataclass(frozen=True)
class ReplayConfig:
    """One deterministic MCL ablation configuration."""

    name: str
    particles: int
    angle_step: int
    conditional_resampling: bool
    beam_skip: bool
    zero_stationary_noise: bool
    squash_factor: float
    reject_bad_stationary_scan: bool = False
    slip_aware: bool = False
    midpoint_reference: bool = False
    deskew: bool = False
    deskew_translation_scale: float = 1.0
    deskew_yaw_source: str = 'imu'


BASELINE = ReplayConfig(
    name='baseline',
    particles=3000,
    angle_step=10,
    conditional_resampling=False,
    beam_skip=False,
    zero_stationary_noise=False,
    squash_factor=2.2,
)
CANDIDATE = ReplayConfig(
    name='candidate',
    particles=2000,
    angle_step=15,
    conditional_resampling=True,
    beam_skip=True,
    zero_stationary_noise=False,
    squash_factor=2.2,
)
CANDIDATE_STOP_SAFE = ReplayConfig(
    name='candidate-stop-safe',
    particles=2000,
    angle_step=15,
    conditional_resampling=True,
    beam_skip=True,
    zero_stationary_noise=True,
    squash_factor=2.2,
)
CANDIDATE_STEP12 = ReplayConfig(
    name='candidate-step12',
    particles=2000,
    angle_step=12,
    conditional_resampling=True,
    beam_skip=True,
    zero_stationary_noise=True,
    squash_factor=2.2,
)
CANDIDATE_2500 = ReplayConfig(
    name='candidate-2500',
    particles=2500,
    angle_step=15,
    conditional_resampling=True,
    beam_skip=True,
    zero_stationary_noise=True,
    squash_factor=2.2,
)
CANDIDATE_SOFT = ReplayConfig(
    name='candidate-soft',
    particles=2000,
    angle_step=12,
    conditional_resampling=True,
    beam_skip=True,
    zero_stationary_noise=True,
    squash_factor=4.0,
)
WORKLOAD_ONLY = ReplayConfig(
    name='workload-only',
    particles=2000,
    angle_step=15,
    conditional_resampling=False,
    beam_skip=False,
    zero_stationary_noise=True,
    squash_factor=2.2,
)
CANDIDATE_NO_SKIP = ReplayConfig(
    name='candidate-no-skip',
    particles=2000,
    angle_step=15,
    conditional_resampling=True,
    beam_skip=False,
    zero_stationary_noise=True,
    squash_factor=2.2,
)
CANDIDATE_HOLD_STOP = ReplayConfig(
    name='candidate-hold-stop',
    particles=2000,
    angle_step=15,
    conditional_resampling=True,
    beam_skip=True,
    zero_stationary_noise=False,
    squash_factor=2.2,
    reject_bad_stationary_scan=True,
    slip_aware=True,
    midpoint_reference=True,
    deskew=True,
)
CANDIDATE_HOLD_STOP_NO_DESKEW = ReplayConfig(
    name='candidate-hold-stop-no-deskew',
    particles=2000,
    angle_step=15,
    conditional_resampling=True,
    beam_skip=True,
    zero_stationary_noise=False,
    squash_factor=2.2,
    reject_bad_stationary_scan=True,
    slip_aware=True,
    midpoint_reference=True,
    deskew=False,
)
CANDIDATE_HOLD_STOP_DESKEW_ROT = replace(
    CANDIDATE_HOLD_STOP,
    name='candidate-hold-stop-deskew-rot',
    deskew_translation_scale=0.0,
)
CANDIDATE_HOLD_STOP_DESKEW_TRANS = replace(
    CANDIDATE_HOLD_STOP,
    name='candidate-hold-stop-deskew-trans',
    deskew_yaw_source='none',
)
CANDIDATE_HOLD_STOP_DESKEW_ODOM_YAW = replace(
    CANDIDATE_HOLD_STOP,
    name='candidate-hold-stop-deskew-odom-yaw',
    deskew_yaw_source='odom',
)


class OfflineParticleFilter:
    """ROS-independent replay of the production PF update equations."""

    def __init__(self, config, raycaster, initial_particles, seed):
        """Initialize one replay from a recorded converged particle cloud."""
        self.config = config
        self.raycaster = raycaster
        self.rng = np.random.default_rng(seed)
        source = np.asarray(initial_particles, dtype=np.float64)
        indices = self.rng.integers(0, len(source), size=config.particles)
        self.particles = np.array(source[indices], copy=True)
        self.particles[:, 0] += self.rng.normal(0.0, 0.005, config.particles)
        self.particles[:, 1] += self.rng.normal(0.0, 0.005, config.particles)
        self.particles[:, 2] = wrap_angle(
            self.particles[:, 2]
            + self.rng.normal(0.0, 0.002, config.particles))
        self.weights = np.full(config.particles, 1.0 / config.particles)
        self.inferred_pose = weighted_pose(self.particles, self.weights)
        self.log_table = sensor_log_table(
            round(10.0 / raycaster.resolution))
        self.maximum_px = self.log_table.shape[0] - 1
        self.quality = LocalizationQualityTracker(0.70, 0.40, 3, 5)
        self.resample_count = 0
        self.beam_fallback_count = 0
        self.jump_reject_count = 0
        self.slip_event_count = 0
        self.slip_recovery_count = 0
        self.slip_jump_bypass_count = 0
        self.consecutive_jump_rejects = 0
        self.quality_counts = {
            state.value: 0 for state in QualityState
        }
        self.ess_prior = []
        self.ess_post = []
        self.scan_fit = []
        self.compute_ms = []
        self.slip_scores = []
        self.deskew_translation_scales = {}
        self.slip_detector = SlipDetector(
            0.15, 0.15, 1.0, 0.50, 2, 5, 2.0)

    def apply_motion(
            self, particles, action, quality_state, noise_multiplier=1.0):
        """Apply the production odometry motion and noise model."""
        cosine = np.cos(particles[:, 2])
        sine = np.sin(particles[:, 2])
        particles[:, 0] += cosine * action[0] - sine * action[1]
        particles[:, 1] += sine * action[0] + cosine * action[1]
        particles[:, 2] = wrap_angle(particles[:, 2] + action[2])
        normalized_motion = (
            np.hypot(action[0], action[1]) / 0.25
            + abs(action[2]) / 0.15
        )
        stationary = normalized_motion <= 1.0e-12
        if (self.config.zero_stationary_noise and stationary
                and noise_multiplier <= 1.0
                and quality_state in (QualityState.STARTUP, QualityState.OK)):
            noise_scale = 0.0
        else:
            noise_scale = float(np.clip(
                np.sqrt(normalized_motion), 0.08, 2.0))
        noise_scale *= max(1.0, float(noise_multiplier))
        count = len(particles)
        particles[:, 0] += self.rng.normal(0.0, 0.05 * noise_scale, count)
        particles[:, 1] += self.rng.normal(0.0, 0.025 * noise_scale, count)
        particles[:, 2] = wrap_angle(
            particles[:, 2]
            + self.rng.normal(0.0, 0.08 * noise_scale, count))

    def update(
            self, action, observed_ranges, relative_angles, low_speed,
            raw_action=None, imu_delta_yaw=0.0, imu_valid=False,
            imu_stationary=False, scan_motion=None,
            scan_motion_valid=False):
        """Run one MCL correction and return whether the pose was accepted."""
        started = time.perf_counter()
        prior_particles = np.array(self.particles, copy=True)
        prior_weights = np.array(self.weights, copy=True)
        prior_pose = np.array(self.inferred_pose, copy=True)
        if raw_action is None:
            raw_action = action
        if scan_motion is None:
            scan_motion = np.zeros(3)
        slip_update = self.slip_detector.update(
            raw_action,
            imu_delta_yaw,
            imu_valid,
            scan_motion,
            scan_motion_valid,
        )
        if not self.config.slip_aware:
            self.slip_detector.reset()
            slip_update = self.slip_detector.update(
                raw_action, 0.0, False, scan_motion, False)
        if slip_update.entered:
            self.slip_event_count += 1
        if slip_update.recovered:
            self.slip_recovery_count += 1
        self.slip_scores.append(slip_update.score)

        if self.config.conditional_resampling:
            proposal = prepare_mcl_proposal(
                self.particles, self.weights, 0.50, rng=self.rng)
            particles = proposal.particles
            proposal_prior = proposal.prior_weights
            prior_ess = proposal.prior_ess_ratio
            if proposal.resampled:
                self.resample_count += 1
        else:
            normalized = self.weights / np.sum(self.weights)
            indices = self.rng.choice(
                len(self.particles), len(self.particles), p=normalized)
            particles = np.array(self.particles[indices], copy=True)
            proposal_prior = np.full(
                len(particles), 1.0 / len(particles))
            prior_ess = effective_sample_size(normalized) / len(normalized)
            self.resample_count += 1

        self.apply_motion(
            particles,
            action,
            self.quality.state,
            noise_multiplier=2.5 if slip_update.active else 1.0,
        )
        predicted_ranges = self.raycaster.cast(
            particles, relative_angles, 10.0)
        predicted_px = np.rint(np.clip(
            predicted_ranges / self.raycaster.resolution,
            0,
            self.maximum_px,
        )).astype(np.uint16)
        observed_px = np.rint(np.clip(
            observed_ranges / self.raycaster.resolution,
            0,
            self.maximum_px,
        )).astype(np.uint16)
        selection = select_beams(
            observed_px,
            predicted_px,
            distance_px=round(0.20 / self.raycaster.resolution),
            min_particle_agreement=0.30,
            max_skipped_fraction=0.50,
            enabled=self.config.beam_skip,
            particle_weights=proposal_prior,
            short_only=self.config.beam_skip,
        )
        if selection.fallback_to_all_beams:
            self.beam_fallback_count += 1
        likelihoods = likelihoods_from_log_table(
            self.log_table,
            observed_px,
            predicted_px,
            selection.mask,
            squash_factor=1.0 / self.config.squash_factor,
        )
        posterior = posterior_from_likelihood(proposal_prior, likelihoods)
        if posterior is None:
            return False
        candidate_pose = weighted_pose(particles, posterior)
        predicted_pose = compose(prior_pose, action)
        correction = compose(inverse(predicted_pose), candidate_pose)
        correction_xy = float(np.hypot(correction[0], correction[1]))
        correction_yaw = float(abs(correction[2]))
        max_xy = 0.50 if low_speed else 0.30
        max_yaw = 0.60 if low_speed else 0.35
        reject = correction_xy > max_xy or correction_yaw > max_yaw
        stationary_hold = should_hold_stationary_filter(
            self.config.reject_bad_stationary_scan
            and low_speed and not slip_update.active,
            0.0,
            0.20,
            action,
            imu_stationary=imu_stationary,
            scan_motion=scan_motion,
            scan_motion_valid=scan_motion_valid,
            max_scan_translation_m=0.04,
            max_scan_yaw_rad=0.04,
        )
        jump_reject = reject
        reject = reject or stationary_hold
        if (jump_reject and slip_update.active
                and selection.weighted_inlier_fraction >= 0.75
                and correction_xy <= 0.80 and correction_yaw <= 0.80):
            reject = False
            jump_reject = False
            self.slip_jump_bypass_count += 1
        if reject and self.consecutive_jump_rejects + 1 >= 10:
            reject = False

        if reject:
            if jump_reject:
                self.jump_reject_count += 1
                self.consecutive_jump_rejects += 1
            else:
                self.consecutive_jump_rejects = 0
            self.particles = prior_particles
            self.weights = prior_weights / np.sum(prior_weights)
            self.apply_deterministic_motion(self.particles, action)
            self.inferred_pose = weighted_pose(self.particles, self.weights)
            quality_score = (
                selection.weighted_inlier_fraction
                if stationary_hold else 0.0
            )
        else:
            self.consecutive_jump_rejects = 0
            self.particles = particles
            self.weights = posterior
            self.inferred_pose = candidate_pose
            quality_score = selection.weighted_inlier_fraction

        quality_update = self.quality.update(quality_score)
        self.quality_counts[quality_update.state.value] += 1
        self.ess_prior.append(prior_ess)
        self.ess_post.append(
            effective_sample_size(self.weights) / len(self.weights))
        self.scan_fit.append(quality_score)
        self.compute_ms.append((time.perf_counter() - started) * 1000.0)
        return not reject

    @staticmethod
    def apply_deterministic_motion(particles, action):
        """Propagate a rejected cloud without injecting process noise."""
        cosine = np.cos(particles[:, 2])
        sine = np.sin(particles[:, 2])
        particles[:, 0] += cosine * action[0] - sine * action[1]
        particles[:, 1] += sine * action[0] + cosine * action[1]
        particles[:, 2] = wrap_angle(particles[:, 2] + action[2])


def initial_particle_cloud(data, target_stamp):
    """Read the recorded particle cloud nearest replay start."""
    records = data['/pf/viz/particles']
    if not records:
        raise RuntimeError(
            '/pf/viz/particles is required for fair replay seed')
    stamps = np.array([
        stamp_ns(message.header.stamp) for _, message in records
    ], dtype=np.int64)
    index = nearest_by_stamp(stamps, target_stamp)
    poses = records[index][1].poses
    particles = []
    for pose in poses:
        particles.append(pose_2d(pose))
    if len(particles) < 10:
        raise RuntimeError('recorded particle cloud is unexpectedly small')
    return np.asarray(particles)


def speed_table(odom_records):
    """Extract timestamped longitudinal odometry speed."""
    items = sorted((
        stamp_ns(message.header.stamp),
        float(message.twist.twist.linear.x),
    ) for _, message in odom_records)
    return (
        np.array([item[0] for item in items], dtype=np.int64),
        np.array([item[1] for item in items], dtype=np.float64),
    )


class ImuTimeline:
    """Timestamped offline IMU integration matching production units."""

    def __init__(self, records):
        """Build sorted gyro and horizontal-acceleration arrays."""
        items = sorted((
            stamp_ns(message.header.stamp),
            float(message.angular_velocity.z) * math.pi / 180.0,
            float(message.linear_acceleration.x) * 9.80665,
            float(message.linear_acceleration.y) * 9.80665,
        ) for _, message in records)
        self.stamps = np.array(
            [item[0] for item in items], dtype=np.int64)
        self.yaw_rates = np.array(
            [item[1] for item in items], dtype=np.float64)
        self.acceleration_x = np.array(
            [item[2] for item in items], dtype=np.float64)
        self.acceleration_y = np.array(
            [item[3] for item in items], dtype=np.float64)

    def integrate(self, start_stamp, end_stamp, max_gap_sec=0.06):
        """Return delta yaw and interval maxima, or ``None`` on a gap."""
        if (self.stamps.size < 2 or end_stamp < start_stamp
                or start_stamp < self.stamps[0]
                or end_stamp > self.stamps[-1]):
            return None
        internal = np.logical_and(
            self.stamps > start_stamp, self.stamps < end_stamp)
        stamps = np.concatenate((
            np.array([start_stamp], dtype=np.int64),
            self.stamps[internal],
            np.array([end_stamp], dtype=np.int64),
        ))
        if np.any(np.diff(stamps) > max_gap_sec * 1e9):
            return None
        yaw_rates = np.interp(stamps, self.stamps, self.yaw_rates)
        acceleration_x = np.interp(
            stamps, self.stamps, self.acceleration_x)
        acceleration_y = np.interp(
            stamps, self.stamps, self.acceleration_y)
        delta_yaw = float(np.trapz(yaw_rates, stamps / 1e9))
        return (
            delta_yaw,
            float(np.max(np.abs(yaw_rates))),
            float(np.max(np.hypot(acceleration_x, acceleration_y))),
        )

    def yaw_offsets(self, reference_stamp, beam_stamps, max_gap_sec=0.06):
        """Integrate all beam yaw offsets from one midpoint in one pass."""
        requested = np.asarray(beam_stamps, dtype=np.int64)
        if requested.size == 0:
            return np.empty(0, dtype=np.float64)
        interval_start = min(int(reference_stamp), int(np.min(requested)))
        interval_end = max(int(reference_stamp), int(np.max(requested)))
        if (self.stamps.size < 2 or interval_start < self.stamps[0]
                or interval_end > self.stamps[-1]):
            return None
        internal = np.logical_and(
            self.stamps > interval_start, self.stamps < interval_end)
        knots = np.unique(np.concatenate((
            np.array([reference_stamp], dtype=np.int64),
            requested,
            self.stamps[internal],
        )))
        if np.any(np.diff(knots) > max_gap_sec * 1e9):
            return None
        rates = np.interp(knots, self.stamps, self.yaw_rates)
        cumulative = np.zeros(knots.size, dtype=np.float64)
        cumulative[1:] = np.cumsum(
            0.5 * (rates[:-1] + rates[1:]) * np.diff(knots) / 1e9)
        reference_yaw = np.interp(reference_stamp, knots, cumulative)
        return np.interp(requested, knots, cumulative) - reference_yaw


class ScanReferenceAdapter:
    """Represent held-out evaluation beams like the configured PF input."""

    def __init__(
            self, odom, imu, base_to_laser, midpoint_reference,
            deskew, translation_scales, yaw_source):
        """Store replay timelines and cache each corrected source scan."""
        self.odom = odom
        self.imu = imu
        self.base_to_laser = np.asarray(base_to_laser, dtype=np.float64)
        self.midpoint_reference = midpoint_reference
        self.deskew = deskew
        self.translation_scales = translation_scales
        self.yaw_source = yaw_source
        self.cache = {}

    def __call__(self, scan):
        """Return timestamp, ranges, and angles at the configured reference."""
        header_stamp = stamp_ns(scan.header.stamp)
        if header_stamp in self.cache:
            return self.cache[header_stamp]

        ranges = np.asarray(scan.ranges, dtype=np.float64)
        indices = np.arange(ranges.size, dtype=np.int64)
        angles = np.asarray(
            scan.angle_min + indices * scan.angle_increment,
            dtype=np.float64,
        )
        scan_duration = (
            (ranges.size - 1) * float(scan.time_increment)
            if np.isfinite(scan.time_increment)
            and scan.time_increment > 0.0 else 0.0
        )
        timing_valid = 0.0 < scan_duration <= 0.05
        if not self.midpoint_reference or not timing_valid:
            result = (header_stamp, ranges, angles)
            self.cache[header_stamp] = result
            return result

        reference_stamp = header_stamp + int(round(
            0.5 * scan_duration * 1e9))
        if not self.deskew:
            result = (reference_stamp, ranges, angles)
            self.cache[header_stamp] = result
            return result
        beam_stamps = header_stamp + np.rint(
            indices * scan.time_increment * 1e9).astype(np.int64)
        reference_pose = self.odom.at(reference_stamp)
        beam_poses = self.odom.at_many(beam_stamps)
        odom_available = reference_pose is not None and beam_poses is not None
        relative_actions = np.zeros((ranges.size, 3), dtype=np.float64)
        if odom_available:
            relative_actions = relative_pose_batch(
                reference_pose, beam_poses)
        yaw_offsets = None
        if self.yaw_source == 'imu':
            yaw_offsets = self.imu.yaw_offsets(
                reference_stamp, beam_stamps)
        elif self.yaw_source == 'none':
            yaw_offsets = np.zeros(ranges.size, dtype=np.float64)
        if ((yaw_offsets is None or self.yaw_source == 'odom')
                and odom_available):
            yaw_offsets = np.copy(relative_actions[:, 2])
        if yaw_offsets is None:
            result = (reference_stamp, ranges, angles)
            self.cache[header_stamp] = result
            return result

        corrected_ranges = np.array(ranges, copy=True)
        invalid = np.logical_or(
            ~np.isfinite(corrected_ranges), corrected_ranges <= 0.0)
        corrected_ranges[invalid] = 10.0
        np.clip(corrected_ranges, 0.0, 10.0, out=corrected_ranges)
        translation_scale = self.translation_scales.get(
            reference_stamp, 1.0)
        if not odom_available:
            translation_scale = 0.0
        corrected = deskew_scan_to_reference(
            corrected_ranges,
            angles,
            relative_actions,
            yaw_offsets,
            self.base_to_laser,
            translation_scale,
            10.0,
        )
        result = (reference_stamp, corrected.ranges, corrected.angles)
        self.cache[header_stamp] = result
        return result


def filter_action(action, speed):
    """Apply the production low-speed odometry deadband."""
    filtered = np.array(action, copy=True)
    if abs(speed) >= 0.20:
        return filtered
    if np.hypot(filtered[0], filtered[1]) < 0.01:
        filtered[0] = 0.0
        filtered[1] = 0.0
    if abs(filtered[2]) < 0.02:
        filtered[2] = 0.0
    filtered *= 0.25
    return filtered


def selected_scans(
        records, start_stamp, end_stamp, rate_hz,
        midpoint_reference=False):
    """Select source scans at a deterministic maximum PF update rate."""
    period_ns = int(1e9 / rate_hz)
    selected = []
    next_stamp = start_stamp
    for _, scan in records:
        timestamp = stamp_ns(scan.header.stamp)
        scan_duration = (
            (len(scan.ranges) - 1) * float(scan.time_increment)
            if np.isfinite(scan.time_increment) and scan.time_increment > 0.0
            else 0.0
        )
        if midpoint_reference and 0.0 < scan_duration <= 0.05:
            timestamp += int(round(0.5 * scan_duration * 1e9))
        if timestamp < start_stamp or timestamp > end_stamp:
            continue
        if timestamp < next_stamp:
            continue
        selected.append((timestamp, scan))
        next_stamp = timestamp + period_ns
    return selected


def replay_config(
        data, raycaster, config, start_stamp, end_stamp, rate_hz, seed):
    """Replay one configuration and return compatible pose updates."""
    initial_particles = initial_particle_cloud(data, start_stamp)
    particle_filter = OfflineParticleFilter(
        config, raycaster, initial_particles, seed)
    odom = OdomInterpolator(data['/odom'])
    speed_stamps, speeds = speed_table(data['/odom'])
    imu = ImuTimeline(data['/sensors/imu/raw'])
    base_to_laser = base_to_laser_pose(data)
    laser_to_base = inverse(base_to_laser)
    scans = selected_scans(
        data['/scan'], start_stamp, end_stamp, rate_hz,
        midpoint_reference=config.midpoint_reference)
    updates = []
    previous_odom = None
    previous_ranges = None
    previous_angles = None
    previous_scan_stamp = None

    for timestamp, scan in scans:
        odom_pose = odom.at(timestamp)
        if odom_pose is None:
            continue
        if previous_odom is None:
            raw_action = np.zeros(3)
        else:
            raw_action = compose(inverse(previous_odom), odom_pose)
        speed_index = nearest_by_stamp(speed_stamps, timestamp)
        speed = float(speeds[speed_index])
        action = filter_action(raw_action, speed)
        selected_indices = np.arange(
            0, len(scan.ranges), config.angle_step, dtype=np.int64)
        observed = np.asarray(
            scan.ranges, dtype=np.float32)[selected_indices]
        invalid = np.logical_or(~np.isfinite(observed), observed <= 0.0)
        observed[invalid] = 10.0
        np.clip(observed, 0.0, 10.0, out=observed)
        angles = np.asarray(
            scan.angle_min + selected_indices * scan.angle_increment,
            dtype=np.float32,
        )
        deskew_compute_ms = 0.0
        deskew_translation_scale = 1.0
        if config.deskew:
            scan_duration = (
                (len(scan.ranges) - 1) * float(scan.time_increment)
                if np.isfinite(scan.time_increment)
                and scan.time_increment > 0.0 else 0.0
            )
            if 0.0 < scan_duration <= 0.05:
                header_stamp = stamp_ns(scan.header.stamp)
                beam_stamps = header_stamp + np.rint(
                    selected_indices * scan.time_increment * 1e9
                ).astype(np.int64)
                beam_poses = odom.at_many(beam_stamps)
                odom_available = beam_poses is not None
                yaw_offsets = None
                if config.deskew_yaw_source == 'imu':
                    yaw_offsets = imu.yaw_offsets(timestamp, beam_stamps)
                elif config.deskew_yaw_source == 'none':
                    yaw_offsets = np.zeros(
                        len(selected_indices), dtype=np.float64)
                deskew_started = time.perf_counter()
                relative_actions = np.zeros(
                    (len(selected_indices), 3), dtype=np.float64)
                if odom_available:
                    relative_actions = relative_pose_batch(
                        odom_pose, beam_poses)
                if ((yaw_offsets is None
                     or config.deskew_yaw_source == 'odom')
                        and odom_available):
                    yaw_offsets = np.copy(relative_actions[:, 2])
                if yaw_offsets is not None:
                    translation_scale = (
                        0.0 if particle_filter.slip_detector.active
                        else config.deskew_translation_scale)
                    if not odom_available:
                        translation_scale = 0.0
                    deskew_translation_scale = translation_scale
                    deskewed = deskew_scan_to_reference(
                        observed,
                        angles,
                        relative_actions,
                        yaw_offsets,
                        base_to_laser,
                        translation_scale,
                        10.0,
                    )
                    observed = deskewed.ranges.astype(np.float32)
                    angles = deskewed.angles.astype(np.float32)
                deskew_compute_ms = (
                    time.perf_counter() - deskew_started) * 1000.0
        particle_filter.deskew_translation_scales[timestamp] = (
            deskew_translation_scale)
        scan_motion = np.zeros(3)
        scan_motion_valid = False
        if (config.slip_aware and previous_ranges is not None
                and previous_angles is not None):
            scan_estimate = estimate_scan_motion(
                previous_ranges,
                observed,
                previous_angles,
                raw_action,
                10.0,
                0.40,
                0.90,
                4,
                12,
                current_angles=angles,
            )
            scan_motion = scan_estimate.action
            scan_motion_valid = bool(
                scan_estimate.valid
                and scan_estimate.inlier_fraction >= 0.55
                and scan_estimate.rmse_m <= 0.15
            )
        imu_interval = None
        if config.slip_aware and previous_scan_stamp is not None:
            imu_interval = imu.integrate(previous_scan_stamp, timestamp)
        imu_valid = imu_interval is not None
        imu_delta_yaw = imu_interval[0] if imu_valid else 0.0
        imu_stationary = bool(
            imu_valid
            and imu_interval[1] <= 0.06
            and imu_interval[2] <= 1.60
        )
        compute_count = len(particle_filter.compute_ms)
        particle_filter.update(
            action,
            observed,
            angles,
            abs(speed) < 0.20,
            raw_action=raw_action,
            imu_delta_yaw=imu_delta_yaw,
            imu_valid=imu_valid,
            imu_stationary=imu_stationary,
            scan_motion=scan_motion,
            scan_motion_valid=scan_motion_valid,
        )
        if len(particle_filter.compute_ms) > compute_count:
            particle_filter.compute_ms[-1] += deskew_compute_ms
        map_to_base = compose(
            particle_filter.inferred_pose, laser_to_base)
        map_to_odom = compose(map_to_base, inverse(odom_pose))
        updates.append((timestamp, map_to_base, odom_pose, map_to_odom))
        previous_odom = odom_pose
        previous_ranges = np.copy(observed)
        previous_angles = np.copy(angles)
        previous_scan_stamp = timestamp
    return particle_filter, updates


def percentile(values, quantile):
    """Return an optional percentile."""
    return float(np.percentile(values, quantile)) if values else None


def stationary_drift(updates):
    """Measure PF motion while scan-to-scan wheel odometry is stationary."""
    translation = []
    yaw = []
    reference = None
    stationary_count = 0
    for previous, current in zip(updates[:-1], updates[1:]):
        odom_delta = compose(inverse(previous[2]), current[2])
        stationary = (
            np.hypot(odom_delta[0], odom_delta[1]) < 0.002
            and abs(odom_delta[2]) < 0.003
        )
        if not stationary:
            reference = None
            stationary_count = 0
            continue
        if reference is None:
            reference = np.array(previous[1], copy=True)
        stationary_count += 1
        if stationary_count < 5:
            continue
        drift = compose(inverse(reference), current[1])
        translation.append(float(np.hypot(drift[0], drift[1])))
        yaw.append(float(abs(np.degrees(drift[2]))))
    return {
        'translation_p95_m': percentile(translation, 95),
        'translation_max_m': max(translation) if translation else None,
        'yaw_p95_deg': percentile(yaw, 95),
        'yaw_max_deg': max(yaw) if yaw else None,
    }


def evaluate_replay(
        data, config, particle_filter, updates, start_stamp,
        laser_pose, excluded_steps):
    """Evaluate one replay using beams unused by either PF configuration."""
    corrections, valid_updates = correction_metrics(
        [(item[0], item[1]) for item in updates],
        OdomInterpolator(data['/odom']),
        start_stamp,
    )
    map_message = data['/map'][0][1]
    scan_adapter = ScanReferenceAdapter(
        OdomInterpolator(data['/odom']),
        ImuTimeline(data['/sensors/imu/raw']),
        laser_pose,
        config.midpoint_reference,
        config.deskew,
        particle_filter.deskew_translation_scales,
        config.deskew_yaw_source,
    )
    pre_fit = map_fit_metrics(
        valid_updates,
        data['/scan'],
        map_message,
        start_stamp,
        laser_pose,
        predicted=True,
        excluded_angle_steps=excluded_steps,
        held_out_stride=10,
        scan_adapter=scan_adapter,
    )
    post_fit = map_fit_metrics(
        valid_updates,
        data['/scan'],
        map_message,
        start_stamp,
        laser_pose,
        predicted=False,
        excluded_angle_steps=excluded_steps,
        held_out_stride=10,
        scan_adapter=scan_adapter,
    )
    series = map_fit_series(
        valid_updates,
        data['/scan'],
        map_message,
        start_stamp,
        laser_pose,
        excluded_angle_steps=excluded_steps,
        held_out_stride=10,
        scan_adapter=scan_adapter,
    )
    moving, _ = moving_fit_summary(series)
    stop_drift = stationary_drift(updates)
    return {
        'config': config.name,
        'updates': len(updates),
        'rays_per_update': config.particles * (
            (1081 + config.angle_step - 1) // config.angle_step),
        'pre_fit_10cm': pre_fit['within_0.10m_fraction'],
        'post_fit_10cm': post_fit['within_0.10m_fraction'],
        'pre_wall_p95_m': pre_fit['wall_distance_p95_m'],
        'moving_fit_10cm': moving.get('fit_10cm_mean'),
        'moving_bad_fraction': moving.get('bad_update_fraction'),
        'correction_p95_m': corrections['translation_m']['p95'],
        'correction_yaw_p95_deg': corrections['yaw_deg']['p95'],
        'resamples': particle_filter.resample_count,
        'beam_fallbacks': particle_filter.beam_fallback_count,
        'jump_rejects': particle_filter.jump_reject_count,
        'quality_lost': particle_filter.quality_counts['LOST'],
        'ess_post_p05': percentile(particle_filter.ess_post, 5),
        'scan_fit_p05': percentile(particle_filter.scan_fit, 5),
        'compute_p50_ms': percentile(particle_filter.compute_ms, 50),
        'compute_p95_ms': percentile(particle_filter.compute_ms, 95),
        'slip_events': particle_filter.slip_event_count,
        'slip_recoveries': particle_filter.slip_recovery_count,
        'slip_jump_bypasses': particle_filter.slip_jump_bypass_count,
        'slip_score_p95': percentile(particle_filter.slip_scores, 95),
        'stationary_drift': stop_drift,
    }


def calibrate_raycaster(data, raycaster, sample_count=30):
    """Compare DDA ranges against RangeLib fake scans recorded on the car."""
    updates = new_update_poses(data)
    update_stamps = np.array([item[0] for item in updates], dtype=np.int64)
    base_to_laser = base_to_laser_pose(data)
    fake_records = data['/pf/viz/fake_scan']
    stride = max(len(fake_records) // sample_count, 1)
    errors = []
    for _, fake_scan in fake_records[::stride]:
        timestamp = stamp_ns(fake_scan.header.stamp)
        update_index = nearest_by_stamp(update_stamps, timestamp)
        if abs(int(update_stamps[update_index]) - timestamp) > 50_000_000:
            continue
        laser_pose = compose(updates[update_index][1], base_to_laser)
        observed = np.asarray(fake_scan.ranges, dtype=np.float32)
        angles = (
            fake_scan.angle_min
            + np.arange(len(observed), dtype=np.float32)
            * fake_scan.angle_increment
        )
        predicted = raycaster.cast(laser_pose[np.newaxis, :], angles, 10.0)[0]
        valid = np.logical_and(np.isfinite(observed), observed > 0.0)
        errors.extend(np.abs(predicted[valid] - observed[valid]))
    values = np.asarray(errors)
    if values.size == 0:
        return {}
    return {
        'samples': int(values.size),
        'median_m': float(np.median(values)),
        'p95_m': float(np.percentile(values, 95)),
        'within_0.10m_fraction': float(np.mean(values <= 0.10)),
    }


def render(bag_name, calibration, results):
    """Render one compact offline replay comparison."""
    rows = []
    for result in results:
        stop_translation = result['stationary_drift']['translation_p95_m']
        stop_yaw = result['stationary_drift']['yaw_p95_deg']
        rows.append((
            result['config'],
            result['rays_per_update'],
            f"{100.0 * result['pre_fit_10cm']:.1f}%",
            (
                'N/A' if result['moving_fit_10cm'] is None
                else f"{100.0 * result['moving_fit_10cm']:.1f}%"
            ),
            (
                'N/A' if result['moving_bad_fraction'] is None
                else f"{100.0 * result['moving_bad_fraction']:.1f}%"
            ),
            f"{100.0 * result['pre_wall_p95_m']:.1f}cm",
            f"{100.0 * result['correction_p95_m']:.1f}cm",
            f"{result['correction_yaw_p95_deg']:.2f}deg",
            result['resamples'],
            result['beam_fallbacks'],
            result['quality_lost'],
            f"{100.0 * result['ess_post_p05']:.1f}%",
            (
                'N/A' if stop_translation is None
                else f'{100.0 * stop_translation:.1f}cm'
            ),
            (
                'N/A' if stop_yaw is None
                else f'{stop_yaw:.2f}deg'
            ),
            f"{result['compute_p95_ms']:.1f}ms",
        ))
    calibration_line = (
        'DDA vs recorded RangeLib: median %.1fcm, p95 %.1fcm, <=10cm %.1f%%'
        % (
            100.0 * calibration.get('median_m', math.nan),
            100.0 * calibration.get('p95_m', math.nan),
            100.0 * calibration.get('within_0.10m_fraction', math.nan),
        )
    )
    slip_rows = [
        (
            result['config'],
            '%d/%d/%d' % (
                result['slip_events'],
                result['slip_recoveries'],
                result['slip_jump_bypasses'],
            ),
            number(result['slip_score_p95']),
        )
        for result in results
    ]
    return '\n'.join((
        f'OFFLINE REAL-BAG MCL A/B: {bag_name}',
        calibration_line,
        text_table(
            ('Config', 'Rays', 'PreFit', 'MoveFit', 'MoveBad', 'WallP95',
             'CorrP95', 'YawP95', 'Resamp', 'Fallback', 'Lost', 'EssP05',
             'StopDrift', 'StopYaw', 'CPU-P95'),
            rows,
        ),
        '',
        text_table(
            ('Config', 'Slip event/recovery/bypass', 'Slip score p95'),
            slip_rows,
        ),
    ))


def resolve_bag(path):
    """Resolve a bag folder or db3 path."""
    candidate = Path(path)
    if candidate.is_file():
        return candidate
    files = sorted(candidate.glob('*.db3'))
    if len(files) != 1:
        raise ValueError('bag directory must contain exactly one db3 file')
    return files[0]


def main():
    """Run deterministic baseline/candidate replay on one real bag."""
    parser = argparse.ArgumentParser()
    parser.add_argument('bag')
    parser.add_argument('--start-sec', type=float, default=10.0)
    parser.add_argument('--duration-sec', type=float, default=30.0)
    parser.add_argument('--rate-hz', type=float, default=15.0)
    parser.add_argument(
        '--configs', nargs='+',
        choices=(
            'baseline',
            'candidate',
            'candidate-stop-safe',
            'candidate-step12',
            'candidate-2500',
            'candidate-soft',
            'workload-only',
            'candidate-no-skip',
            'candidate-hold-stop-no-deskew',
            'candidate-hold-stop-deskew-rot',
            'candidate-hold-stop-deskew-trans',
            'candidate-hold-stop-deskew-odom-yaw',
            'candidate-hold-stop',
        ),
        default=('baseline', 'candidate-hold-stop'),
    )
    parser.add_argument('--seed', type=int, default=7)
    args = parser.parse_args()
    if args.start_sec < 0.0 or args.duration_sec <= 0.0 or args.rate_hz <= 0.0:
        parser.error('start must be non-negative; duration and rate positive')

    bag_path = resolve_bag(args.bag)
    data = read_bag(bag_path)
    map_message = data['/map'][0][1]
    raycaster = DdaRaycaster(map_message)
    calibration = calibrate_raycaster(data, raycaster)
    scan_stamps = np.array([
        stamp_ns(scan.header.stamp) for _, scan in data['/scan']
    ], dtype=np.int64)
    start_stamp = int(scan_stamps[0] + args.start_sec * 1e9)
    end_stamp = min(
        int(start_stamp + args.duration_sec * 1e9), int(scan_stamps[-1]))
    configs = {
        config.name: config
        for config in (
            BASELINE,
            CANDIDATE,
            CANDIDATE_STOP_SAFE,
            CANDIDATE_STEP12,
            CANDIDATE_2500,
            CANDIDATE_SOFT,
            WORKLOAD_ONLY,
            CANDIDATE_NO_SKIP,
            CANDIDATE_HOLD_STOP_NO_DESKEW,
            CANDIDATE_HOLD_STOP_DESKEW_ROT,
            CANDIDATE_HOLD_STOP_DESKEW_TRANS,
            CANDIDATE_HOLD_STOP_DESKEW_ODOM_YAW,
            CANDIDATE_HOLD_STOP,
        )
    }
    laser_pose = base_to_laser_pose(data)
    excluded_steps = tuple(sorted({
        configs[name].angle_step for name in args.configs
    }))
    results = []
    for name in args.configs:
        config = configs[name]
        particle_filter, updates = replay_config(
            data,
            raycaster,
            config,
            start_stamp,
            end_stamp,
            args.rate_hz,
            args.seed,
        )
        results.append(evaluate_replay(
            data,
            config,
            particle_filter,
            updates,
            start_stamp,
            laser_pose,
            excluded_steps,
        ))
    print(render(bag_path.parent.name, calibration, results))


if __name__ == '__main__':
    main()
