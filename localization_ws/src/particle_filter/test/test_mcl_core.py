# Copyright 2026 IFAC F1TENTH Team
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Tests for the ROS-independent MCL production primitives."""

import math

import numpy as np

from particle_filter.mcl_core import (
    can_bypass_jump_guard_for_slip,
    deskew_scan_to_reference,
    effective_sample_size,
    estimate_scan_motion,
    likelihoods_from_log_table,
    LocalizationQualityTracker,
    LocalizationReadinessTracker,
    posterior_from_likelihood,
    prepare_mcl_proposal,
    QualityState,
    relative_pose_batch,
    select_beams,
    should_hold_stationary_filter,
    SlipDetector,
    systematic_resample,
)


def test_effective_sample_size_distinguishes_uniform_and_collapsed_weights():
    """ESS is N for uniform weights and one for a collapsed distribution."""
    assert math.isclose(effective_sample_size(np.ones(8)), 8.0)
    assert math.isclose(
        effective_sample_size(np.array([1.0, 0.0, 0.0, 0.0])), 1.0)
    assert effective_sample_size(np.array([np.nan, 1.0])) == 0.0


def test_systematic_resampling_is_sorted_and_favors_dominant_particle():
    """Systematic resampling preserves order and dominant mass."""
    weights = np.array([0.70, 0.10, 0.10, 0.10])
    indices = systematic_resample(weights, rng=np.random.default_rng(7))

    assert len(indices) == len(weights)
    assert np.all(np.diff(indices) >= 0)
    assert np.count_nonzero(indices == 0) >= 2


def test_mcl_proposal_resamples_only_below_configured_ess():
    """High-ESS particles are copied; low-ESS particles are resampled."""
    particles = np.arange(12, dtype=float).reshape(4, 3)
    high = prepare_mcl_proposal(
        particles, np.ones(4), 0.5, rng=np.random.default_rng(1))
    low = prepare_mcl_proposal(
        particles, np.array([0.97, 0.01, 0.01, 0.01]), 0.5,
        rng=np.random.default_rng(1))

    assert not high.resampled
    assert np.array_equal(high.particles, particles)
    assert high.particles is not particles
    assert low.resampled
    assert np.allclose(low.prior_weights, 0.25)
    assert np.count_nonzero(low.particles[:, 0] == particles[0, 0]) >= 3


def test_mcl_posterior_combines_prior_and_sensor_likelihood():
    """A no-resample update must retain the prior importance weights."""
    posterior = posterior_from_likelihood(
        np.array([0.8, 0.2]), np.array([0.1, 0.9]))

    assert posterior is not None
    assert np.allclose(posterior, np.array([0.08, 0.18]) / 0.26)


def test_stationary_hold_requires_zero_action_and_low_speed():
    """Stationary hold requires wheel, IMU, and scan consensus."""
    kwargs = {
        'enabled': True,
        'speed': 0.0,
        'low_speed_threshold': 0.2,
        'action': np.zeros(3),
        'imu_stationary': True,
        'scan_motion': np.zeros(3),
        'scan_motion_valid': True,
        'max_scan_translation_m': 0.03,
        'max_scan_yaw_rad': 0.04,
    }
    assert should_hold_stationary_filter(
        **kwargs)
    assert not should_hold_stationary_filter(**{**kwargs, 'enabled': False})
    assert not should_hold_stationary_filter(**{**kwargs, 'speed': 0.3})
    assert not should_hold_stationary_filter(**{
        **kwargs, 'action': np.array([0.01, 0.0, 0.0])})
    assert not should_hold_stationary_filter(**{
        **kwargs, 'imu_stationary': False})
    assert not should_hold_stationary_filter(**{
        **kwargs, 'scan_motion_valid': False})
    assert not should_hold_stationary_filter(**{
        **kwargs, 'scan_motion': np.array([0.04, 0.0, 0.0])})


def _square_room_scan(sensor_pose, angles, half_extent=5.0):
    """Raycast a synthetic square room for scan-motion tests."""
    headings = angles + sensor_pose[2]
    direction_x = np.cos(headings)
    direction_y = np.sin(headings)
    candidates = []
    with np.errstate(divide='ignore', invalid='ignore'):
        for wall_x in (-half_extent, half_extent):
            distance = (wall_x - sensor_pose[0]) / direction_x
            hit_y = sensor_pose[1] + distance * direction_y
            candidates.append(np.where(
                np.logical_and(
                    distance > 0.0, np.abs(hit_y) <= half_extent),
                distance,
                np.inf,
            ))
        for wall_y in (-half_extent, half_extent):
            distance = (wall_y - sensor_pose[1]) / direction_y
            hit_x = sensor_pose[0] + distance * direction_x
            candidates.append(np.where(
                np.logical_and(
                    distance > 0.0, np.abs(hit_x) <= half_extent),
                distance,
                np.inf,
            ))
    return np.min(np.vstack(candidates), axis=0)


def test_scan_motion_icp_recovers_small_planar_action():
    """The lightweight matcher recovers motion without using wheel odometry."""
    angles = np.linspace(-2.35, 2.35, 109)
    expected = np.array([0.12, -0.04, 0.05])
    previous = _square_room_scan(np.zeros(3), angles)
    current = _square_room_scan(expected, angles)

    result = estimate_scan_motion(
        previous,
        current,
        angles,
        initial_action=np.zeros(3),
        maximum_range_m=10.0,
        max_correspondence_distance_m=0.50,
        iterations=6,
    )

    assert result.valid
    assert result.inlier_fraction > 0.8
    assert np.allclose(result.action[:2], expected[:2], atol=0.04)
    assert math.isclose(result.action[2], expected[2], abs_tol=0.03)


def test_scan_deskew_transforms_hits_but_preserves_no_return_ranges():
    """Endpoint motion is corrected while max-range semantics stay intact."""
    result = deskew_scan_to_reference(
        ranges=np.array([1.0, 1.0, 10.0]),
        angles=np.zeros(3),
        relative_base_actions=np.array([
            [0.10, 0.0, 0.0],
            [0.0, 0.0, 0.0],
            [-0.10, 0.0, 0.0],
        ]),
        yaw_offsets=np.array([0.0, math.pi / 2.0, 0.0]),
        base_to_laser_pose=np.zeros(3),
        translation_scale=1.0,
        maximum_range_m=10.0,
    )

    assert math.isclose(result.ranges[0], 1.10)
    assert math.isclose(result.angles[0], 0.0)
    assert math.isclose(result.ranges[1], 1.0)
    assert math.isclose(result.angles[1], math.pi / 2.0)
    assert math.isclose(result.ranges[2], 10.0)


def test_scan_deskew_keeps_lidar_lever_arm_when_translation_is_disabled():
    """Slip disables wheel translation, not rotation-induced sensor motion."""
    result = deskew_scan_to_reference(
        ranges=np.array([1.0]),
        angles=np.array([0.0]),
        relative_base_actions=np.array([[0.5, 0.0, math.pi / 2.0]]),
        yaw_offsets=np.array([math.pi / 2.0]),
        base_to_laser_pose=np.array([1.0, 0.0, 0.0]),
        translation_scale=0.0,
        maximum_range_m=10.0,
    )

    assert np.allclose(
        result.ranges * np.column_stack((
            np.cos(result.angles), np.sin(result.angles))),
        np.array([[-1.0, 2.0]]),
    )


def test_scan_deskew_matches_independent_homogeneous_transforms():
    """Vectorized SE(2) deskew matches direct matrix multiplication."""
    ranges = np.array([0.8, 2.0, 4.5])
    angles = np.array([-0.7, 0.2, 1.1])
    actions = np.array([
        [-0.04, 0.01, -0.03],
        [0.0, 0.0, 0.0],
        [0.05, -0.02, 0.04],
    ])
    yaw_offsets = np.array([-0.025, 0.0, 0.035])
    extrinsic = np.array([0.27, -0.03, 0.08])
    scale = 0.4

    result = deskew_scan_to_reference(
        ranges,
        angles,
        actions,
        yaw_offsets,
        extrinsic,
        scale,
        10.0,
    )

    def matrix(pose):
        cosine = math.cos(pose[2])
        sine = math.sin(pose[2])
        return np.array([
            [cosine, -sine, pose[0]],
            [sine, cosine, pose[1]],
            [0.0, 0.0, 1.0],
        ])

    extrinsic_matrix = matrix(extrinsic)
    expected = []
    for beam_range, beam_angle, action, yaw_offset in zip(
            ranges, angles, actions, yaw_offsets):
        base_action = np.array([
            action[0] * scale,
            action[1] * scale,
            yaw_offset,
        ])
        laser_action = (
            np.linalg.inv(extrinsic_matrix)
            @ matrix(base_action)
            @ extrinsic_matrix)
        point = np.array([
            beam_range * math.cos(beam_angle),
            beam_range * math.sin(beam_angle),
            1.0,
        ])
        expected.append((laser_action @ point)[:2])

    endpoints = result.ranges[:, np.newaxis] * np.column_stack((
        np.cos(result.angles), np.sin(result.angles)))
    assert np.allclose(endpoints, expected, atol=1.0e-12)


def test_relative_pose_batch_uses_the_reference_body_frame():
    """Batch interpolation poses become beam actions in the midpoint frame."""
    relative = relative_pose_batch(
        np.array([1.0, 2.0, math.pi / 2.0]),
        np.array([
            [1.0, 2.0, math.pi / 2.0],
            [1.0, 3.0, math.pi],
            [2.0, 2.0, 0.0],
        ]),
    )

    assert np.allclose(relative, np.array([
        [0.0, 0.0, 0.0],
        [1.0, 0.0, math.pi / 2.0],
        [0.0, -1.0, -math.pi / 2.0],
    ]), atol=1.0e-12)


def test_slip_detector_uses_hysteresis_and_holds_on_missing_evidence():
    """Persistent innovation enters slip and clean evidence recovers it."""
    detector = SlipDetector(
        translation_threshold_m=0.10,
        yaw_threshold_rad=0.10,
        enter_score=1.0,
        recover_score=0.5,
        enter_updates=2,
        recover_updates=3,
        immediate_score=3.0,
    )
    odom = np.array([0.25, 0.0, 0.0])
    scan = np.array([0.05, 0.0, 0.0])

    first = detector.update(odom, 0.0, True, scan, True)
    second = detector.update(odom, 0.0, True, scan, True)
    missing = detector.update(odom, 0.0, False, scan, False)

    assert not first.active
    assert second.entered and second.active
    assert math.isclose(second.score, 2.0)
    assert missing.active and not missing.recovered
    clean = np.array([0.05, 0.0, 0.0])
    imu_only = detector.update(clean, 0.0, True, clean, False)
    assert imu_only.active and not imu_only.recovered
    assert detector.update(clean, 0.0, True, clean, True).active
    assert detector.update(clean, 0.0, True, clean, True).active
    recovered = detector.update(clean, 0.0, True, clean, True)
    assert recovered.recovered and not recovered.active


def test_slip_detector_enters_immediately_for_large_innovation():
    """A severe one-frame slide does not wait for normal hysteresis."""
    detector = SlipDetector(
        translation_threshold_m=0.15,
        yaw_threshold_rad=0.15,
        enter_score=1.0,
        recover_score=0.5,
        enter_updates=3,
        recover_updates=3,
        immediate_score=2.0,
    )

    update = detector.update(
        np.array([0.40, 0.0, 0.0]),
        0.0,
        True,
        np.zeros(3),
        True,
    )

    assert update.entered and update.active
    assert update.score > 2.0


def test_slip_jump_guard_bypass_requires_all_safety_conditions():
    """Slip alone cannot bypass the guard without map fit and hard bounds."""
    arguments = {
        'slip_active': True,
        'scan_fit': 0.80,
        'minimum_scan_fit': 0.75,
        'correction_translation_m': 0.50,
        'correction_yaw_rad': 0.40,
        'maximum_translation_m': 0.80,
        'maximum_yaw_rad': 0.80,
    }

    assert can_bypass_jump_guard_for_slip(**arguments)
    assert not can_bypass_jump_guard_for_slip(**{
        **arguments, 'slip_active': False})
    assert not can_bypass_jump_guard_for_slip(**{
        **arguments, 'scan_fit': 0.70})
    assert not can_bypass_jump_guard_for_slip(**{
        **arguments, 'correction_translation_m': 0.90})
    assert not can_bypass_jump_guard_for_slip(**{
        **arguments, 'correction_yaw_rad': 0.90})


def test_beam_skipping_removes_dynamic_short_return():
    """One obstacle beam is skipped while static-map beams remain active."""
    predicted = np.array([
        [10, 20, 30, 40],
        [10, 21, 30, 39],
        [11, 20, 29, 40],
    ], dtype=np.uint16)
    observed = np.array([10, 5, 30, 40], dtype=np.uint16)

    selection = select_beams(
        observed, predicted, distance_px=1,
        min_particle_agreement=0.5, max_skipped_fraction=0.5)

    assert np.array_equal(selection.mask, [True, False, True, True])
    assert math.isclose(selection.skipped_fraction, 0.25)
    assert not selection.fallback_to_all_beams
    assert math.isclose(selection.best_particle_inlier_fraction, 1.0)
    assert math.isclose(selection.weighted_inlier_fraction, 1.0)


def test_beam_skipping_falls_back_when_cloud_explains_too_little():
    """A globally wrong cloud cannot hide failure by skipping every beam."""
    predicted = np.full((5, 6), 100, dtype=np.uint16)
    observed = np.arange(6, dtype=np.uint16)

    selection = select_beams(
        observed, predicted, distance_px=2,
        min_particle_agreement=0.3, max_skipped_fraction=0.5)

    assert np.all(selection.mask)
    assert selection.fallback_to_all_beams
    assert math.isclose(selection.skipped_fraction, 1.0)
    assert math.isclose(selection.best_particle_inlier_fraction, 0.0)
    assert math.isclose(selection.weighted_inlier_fraction, 0.0)


def test_beam_skip_fallback_never_restores_geometry_rejected_beams():
    """Ground/off-plane exclusions survive dynamic beam-skip fallback."""
    predicted = np.full((4, 5), 100, dtype=np.uint16)
    observed = np.arange(5, dtype=np.uint16)
    required = np.array([True, False, True, False, True])

    selection = select_beams(
        observed, predicted, distance_px=1,
        min_particle_agreement=0.5, max_skipped_fraction=0.5,
        required_mask=required)

    assert selection.fallback_to_all_beams
    assert np.array_equal(selection.mask, required)


def test_beam_agreement_uses_particle_importance_weights():
    """Low-weight outliers cannot force a static beam to be skipped."""
    observed = np.array([10], dtype=np.uint16)
    predicted = np.array([[10], [50]], dtype=np.uint16)

    selection = select_beams(
        observed,
        predicted,
        distance_px=1,
        min_particle_agreement=0.80,
        max_skipped_fraction=0.50,
        particle_weights=np.array([0.90, 0.10]),
    )

    assert np.array_equal(selection.mask, [True])
    assert not selection.fallback_to_all_beams
    assert math.isclose(selection.agreement_fraction[0], 0.90)
    assert math.isclose(selection.weighted_inlier_fraction, 0.90)


def test_short_only_beam_skip_retains_long_map_mismatch():
    """Missing map returns are not mistaken for a moving obstacle."""
    observed = np.array([80, 10], dtype=np.uint16)
    predicted = np.array([
        [20, 60],
        [20, 60],
        [20, 60],
    ], dtype=np.uint16)

    selection = select_beams(
        observed,
        predicted,
        distance_px=2,
        min_particle_agreement=0.30,
        max_skipped_fraction=0.75,
        short_only=True,
    )

    assert np.array_equal(selection.mask, [True, False])
    assert not selection.fallback_to_all_beams


def test_precomputed_log_table_matches_direct_probability_product():
    """The optimized log table preserves the old sensor likelihood result."""
    table = np.array([
        [0.70, 0.20, 0.10],
        [0.20, 0.60, 0.20],
        [0.10, 0.20, 0.70],
    ])
    observed = np.array([0, 1, 2])
    predicted = np.array([[0, 1, 2], [1, 1, 1]])
    mask = np.array([True, True, True])
    optimized = likelihoods_from_log_table(
        np.log(table), observed, predicted, mask, squash_factor=0.5)

    direct = np.array([
        table[0, 0] * table[1, 1] * table[2, 2],
        table[0, 1] * table[1, 1] * table[2, 1],
    ]) ** 0.5
    direct /= np.max(direct)
    assert np.allclose(optimized, direct)


def test_lost_detector_has_bad_and_recovery_hysteresis():
    """Dynamic-beam noise does not trip LOST, but persistent mismatch does."""
    tracker = LocalizationQualityTracker(
        ok_inlier_fraction=0.75,
        lost_inlier_fraction=0.50,
        bad_updates_to_lost=3,
        good_updates_to_recover=2,
    )

    assert tracker.update(0.90).state == QualityState.STARTUP
    assert tracker.update(0.90).state == QualityState.OK
    assert tracker.update(0.65).state == QualityState.DEGRADED
    assert tracker.update(0.85).state == QualityState.DEGRADED
    assert tracker.update(0.85).state == QualityState.OK

    assert tracker.update(0.20).state == QualityState.DEGRADED
    assert tracker.update(0.20).state == QualityState.DEGRADED
    assert tracker.update(0.20).state == QualityState.LOST
    assert tracker.update(0.90).state == QualityState.LOST
    assert tracker.update(0.90).state == QualityState.OK


def test_readiness_requires_initial_pose_and_quality_ok():
    """Quality alone cannot arm autonomy before the PF accepts initial pose."""
    readiness = LocalizationReadinessTracker()

    assert not readiness.ready
    assert not readiness.update(QualityState.OK)
    assert not readiness.ready

    readiness.receive_initial_pose()
    assert readiness.initial_pose_received
    assert not readiness.initial_pose_accepted
    assert not readiness.update(QualityState.OK)
    assert not readiness.ready

    readiness.accept_initial_pose()
    assert readiness.initial_pose_accepted
    assert not readiness.ready
    assert not readiness.update(QualityState.DEGRADED)
    assert readiness.update(QualityState.OK)
    assert readiness.ready


def test_readiness_uses_quality_trackers_full_consecutive_ok_count():
    """Autonomy arms on the configured fifth good scan, never before it."""
    quality = LocalizationQualityTracker(
        ok_inlier_fraction=0.70,
        lost_inlier_fraction=0.40,
        bad_updates_to_lost=3,
        good_updates_to_recover=5,
    )
    readiness = LocalizationReadinessTracker()
    readiness.accept_initial_pose()

    for _ in range(4):
        update = quality.update(0.90)
        readiness.update(update.state)
        assert not readiness.ready

    update = quality.update(0.90)
    assert update.state == QualityState.OK
    assert readiness.update(update.state)
    assert readiness.ready


def test_readiness_latches_degraded_but_revokes_lost_and_resets():
    """Transient degradation is tolerated; LOST and resets are fail-closed."""
    readiness = LocalizationReadinessTracker()
    readiness.accept_initial_pose()
    readiness.update(QualityState.OK)

    assert not readiness.update(QualityState.DEGRADED)
    assert readiness.ready
    assert readiness.update(QualityState.LOST)
    assert not readiness.ready
    assert readiness.update(QualityState.OK)

    readiness.reset_quality()
    assert readiness.initial_pose_received
    assert readiness.initial_pose_accepted
    assert not readiness.ready
    readiness.update(QualityState.OK)
    readiness.clear_initial_pose()
    assert not readiness.initial_pose_received
    assert not readiness.initial_pose_accepted
    assert not readiness.ready


def test_new_initial_pose_receipt_revokes_ready_until_applied():
    """A replacement RViz request fails closed while waiting for fixed TF."""
    readiness = LocalizationReadinessTracker()
    readiness.accept_initial_pose()
    readiness.update(QualityState.OK)
    assert readiness.ready

    readiness.receive_initial_pose()
    assert readiness.initial_pose_received
    assert not readiness.initial_pose_accepted
    assert not readiness.ready
