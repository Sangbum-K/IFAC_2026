#!/usr/bin/env python3
"""Benchmark PF Python-side work and report the raycast budget."""

import argparse
from pathlib import Path
import sys
import time

import numpy as np


REPO_ROOT = Path(__file__).resolve().parents[1]
PF_SOURCE = REPO_ROOT / 'localization_ws' / 'src' / 'particle_filter'
GEOMETRY_SOURCE = REPO_ROOT / 'planning_ws' / 'src' / 'f1tenth_scan_geometry'
sys.path.insert(0, str(PF_SOURCE))
sys.path.insert(0, str(GEOMETRY_SOURCE))

from f1tenth_scan_geometry import AttitudeSample  # noqa: E402
from f1tenth_scan_geometry import GroundFilterConfig  # noqa: E402
from f1tenth_scan_geometry import TimestampedAttitudeBuffer  # noqa: E402
from f1tenth_scan_geometry import mask_ground_and_offplane  # noqa: E402
from f1tenth_scan_geometry import quaternion_from_rpy  # noqa: E402
from particle_filter.mcl_core import deskew_scan_to_reference  # noqa: E402
from particle_filter.mcl_core import likelihoods_from_log_table  # noqa: E402
from particle_filter.mcl_core import relative_pose_batch  # noqa: E402
from particle_filter.mcl_core import select_beams  # noqa: E402
from particle_filter.mcl_core import systematic_resample  # noqa: E402


def measure(operation, iterations):
    """Return median milliseconds per call after short warmup batches."""
    samples = []
    for batch in range(8):
        started = time.perf_counter()
        for _ in range(iterations):
            operation()
        elapsed_ms = (time.perf_counter() - started) * 1000.0 / iterations
        if batch >= 2:
            samples.append(elapsed_ms)
    return float(np.median(samples))


def sampled_beams(lidar_beams, angle_step):
    """Return the length of ``ranges[::angle_step]``."""
    return (lidar_beams + angle_step - 1) // angle_step


def build_case(particles, beams, table, log_table, rng):
    """Build deterministic arrays and benchmark closures for one PF shape."""
    maximum_px = table.shape[0] - 1
    poses = rng.normal(size=(particles, 3)).astype(np.float32)
    angles = np.linspace(-2.35, 2.35, beams, dtype=np.float32)
    observed = rng.integers(0, maximum_px + 1, size=beams, dtype=np.uint16)
    predicted = rng.integers(
        0, maximum_px + 1, size=(particles, beams), dtype=np.uint16)
    expanded_queries = np.empty((particles * beams, 3), dtype=np.float32)
    repeat_queries = np.empty((particles, 3), dtype=np.float32)
    tiled_angles = np.tile(angles, particles)
    beam_mask = np.ones(beams, dtype=bool)
    weights = np.linspace(1.0, 0.01, particles, dtype=np.float64)
    weights /= np.sum(weights)
    resample_rng = np.random.default_rng(2026)

    def expanded_query_build():
        expanded_queries[:, 0] = np.repeat(poses[:, 0], beams)
        expanded_queries[:, 1] = np.repeat(poses[:, 1], beams)
        expanded_queries[:, 2] = np.repeat(poses[:, 2], beams)
        expanded_queries[:, 2] += tiled_angles

    def repeat_query_build():
        repeat_queries[:, :] = poses

    def direct_log_likelihood():
        probabilities = table[observed[np.newaxis, :], predicted]
        log_weights = np.sum(
            np.log(np.maximum(probabilities, 1e-12)), axis=1) / 2.2
        np.exp(log_weights - np.max(log_weights))

    def precomputed_log_with_beam_skip():
        selection = select_beams(
            observed,
            predicted,
            distance_px=4,
            min_particle_agreement=0.30,
            max_skipped_fraction=0.50,
        )
        likelihoods_from_log_table(
            log_table,
            observed,
            predicted,
            selection.mask if selection.mask.any() else beam_mask,
            squash_factor=1.0 / 2.2,
        )

    def multinomial_resample():
        rng.choice(particles, particles, p=weights)

    def low_variance_resample():
        systematic_resample(weights, rng=resample_rng)

    return {
        'expanded_query': expanded_query_build,
        'repeat_query': repeat_query_build,
        'direct_log': direct_log_likelihood,
        'precomputed_log_beam_skip': precomputed_log_with_beam_skip,
        'multinomial': multinomial_resample,
        'systematic': low_variance_resample,
    }


def build_deskew_case(beams):
    """Build the production-sized midpoint endpoint transform closure."""
    ranges = np.linspace(0.5, 9.5, beams, dtype=np.float64)
    angles = np.linspace(-2.35, 2.35, beams, dtype=np.float64)
    yaw_offsets = np.linspace(-0.03, 0.03, beams, dtype=np.float64)
    reference = np.array([4.0, -2.0, 0.4], dtype=np.float64)
    poses = np.repeat(reference[np.newaxis, :], beams, axis=0)
    poses[:, 0] += np.linspace(-0.02, 0.02, beams)
    poses[:, 1] += np.linspace(0.01, -0.01, beams)
    poses[:, 2] += yaw_offsets
    base_to_laser = np.array([0.27, 0.0, 0.0], dtype=np.float64)

    def deskew():
        relative_actions = relative_pose_batch(reference, poses)
        deskew_scan_to_reference(
            ranges,
            angles,
            relative_actions,
            yaw_offsets,
            base_to_laser,
            1.0,
            10.0,
        )

    return deskew


def build_geometry_case(beams):
    """Build the worst-common-case full-scan attitude and geometry gate."""
    angles = np.linspace(-2.35, 2.35, beams, dtype=np.float64)
    ranges = 2.0 + 0.2 * np.cos(angles)
    indices = np.arange(beams, dtype=np.int64)
    level = quaternion_from_rpy(0.0, 0.0, 0.0)
    tilted = quaternion_from_rpy(np.deg2rad(8.0), 0.0, 0.0)
    buffer = TimestampedAttitudeBuffer(150)
    buffer.append(AttitudeSample(0, tuple(level)))
    buffer.append(AttitudeSample(20_000_000, tuple(tilted)))
    stamps = np.linspace(0, 20_000_000, beams, dtype=np.int64)
    config = GroundFilterConfig(
        attitude_evidence_max_abs_angle_rad=np.deg2rad(80.0))

    def scan_geometry():
        lookup = buffer.lookup_many(stamps, 0.08)
        mask_ground_and_offplane(
            ranges, angles, indices, lookup.quaternions, level,
            [0.27, 0.0, 0.11], level, config)

    return scan_geometry


def main():
    """Run the synthetic CPU benchmark and print a compact comparison."""
    parser = argparse.ArgumentParser()
    parser.add_argument('--lidar-beams', type=int, default=1081)
    parser.add_argument('--baseline-particles', type=int, default=3000)
    parser.add_argument('--baseline-angle-step', type=int, default=10)
    parser.add_argument('--candidate-particles', type=int, default=2000)
    parser.add_argument('--candidate-angle-step', type=int, default=15)
    parser.add_argument('--iterations', type=int, default=10)
    args = parser.parse_args()
    positive = (
        args.lidar_beams,
        args.baseline_particles,
        args.baseline_angle_step,
        args.candidate_particles,
        args.candidate_angle_step,
        args.iterations,
    )
    if any(value <= 0 for value in positive):
        parser.error('all numeric arguments must be positive')

    baseline_beams = sampled_beams(
        args.lidar_beams, args.baseline_angle_step)
    candidate_beams = sampled_beams(
        args.lidar_beams, args.candidate_angle_step)
    baseline_rays = args.baseline_particles * baseline_beams
    candidate_rays = args.candidate_particles * candidate_beams

    rng = np.random.default_rng(7)
    table = rng.random((201, 201), dtype=np.float32) + 1e-5
    table /= np.sum(table, axis=0, keepdims=True)
    log_table = np.log(np.maximum(table, 1e-12)).astype(np.float32)
    baseline = build_case(
        args.baseline_particles, baseline_beams, table, log_table, rng)
    candidate = build_case(
        args.candidate_particles, candidate_beams, table, log_table, rng)

    old_query_ms = measure(baseline['expanded_query'], args.iterations)
    new_query_ms = measure(candidate['repeat_query'], args.iterations)
    old_sensor_ms = measure(baseline['direct_log'], args.iterations)
    new_sensor_ms = measure(
        candidate['precomputed_log_beam_skip'], args.iterations)
    old_resample_ms = measure(baseline['multinomial'], args.iterations)
    new_resample_ms = measure(candidate['systematic'], args.iterations)
    deskew_ms = measure(
        build_deskew_case(candidate_beams), max(args.iterations, 100))
    geometry_ms = measure(
        build_geometry_case(args.lidar_beams), max(args.iterations, 100))

    reduction = 100.0 * (1.0 - candidate_rays / baseline_rays)
    print('PF MCL SYNTHETIC CPU BENCHMARK')
    print(
        'Raycasts/update: %d -> %d (%+.1f%%)' % (
            baseline_rays, candidate_rays, -reduction))
    print(
        'Raycasts/second @15Hz: %.3fM -> %.3fM' % (
            baseline_rays * 15 / 1e6, candidate_rays * 15 / 1e6))
    print('Python block                         baseline   candidate')
    print('Expanded/repeat query build       %8.3fms  %8.3fms' % (
        old_query_ms, new_query_ms))
    print('Log likelihood (+ beam skip new)  %8.3fms  %8.3fms' % (
        old_sensor_ms, new_sensor_ms))
    print('Multinomial/systematic resample   %8.3fms  %8.3fms' % (
        old_resample_ms, new_resample_ms))
    print('Midpoint pose + endpoint deskew   %8s  %8.3fms' % (
        '-', deskew_ms))
    print('1081-beam attitude + 3D gate      %8s  %8.3fms' % (
        '-', geometry_ms))
    print(
        'Note: RangeLib raycasting is not timed; '
        'its call count is shown above.')


if __name__ == '__main__':
    main()
