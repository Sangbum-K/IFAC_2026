import numpy as np

from f1tenth_scan_geometry import AttitudeSample
from f1tenth_scan_geometry import GroundFilterConfig
from f1tenth_scan_geometry import LOOKUP_GAP_TOO_LARGE
from f1tenth_scan_geometry import LOOKUP_OK
from f1tenth_scan_geometry import PoseSample
from f1tenth_scan_geometry import TimestampedAttitudeBuffer
from f1tenth_scan_geometry import TimestampedPoseBuffer
from f1tenth_scan_geometry import deskew_points_to_reference
from f1tenth_scan_geometry import mask_ground_and_offplane
from f1tenth_scan_geometry import quaternion_from_rpy


def test_pose_buffer_interpolates_xy_and_wrapped_yaw():
    buffer = TimestampedPoseBuffer(5)
    buffer.append(PoseSample(0, 0.0, 0.0, np.deg2rad(179.0), 1.0))
    buffer.append(PoseSample(100, 2.0, 4.0, np.deg2rad(-179.0), 3.0))
    result = buffer.lookup_many((50,), 1.0)
    assert result.status == LOOKUP_OK
    sample = result.samples[0]
    assert np.allclose((sample.x, sample.y, sample.linear_speed), (1.0, 2.0, 2.0))
    assert np.isclose(abs(sample.yaw), np.pi)


def test_attitude_buffer_slerps_and_rejects_large_gap():
    buffer = TimestampedAttitudeBuffer(5)
    buffer.append(AttitudeSample(0, tuple(quaternion_from_rpy(0.0, 0.0, 0.0))))
    buffer.append(AttitudeSample(
        1_000_000_000,
        tuple(quaternion_from_rpy(0.0, 0.0, np.pi)),
    ))
    assert buffer.lookup_many((500_000_000,), 0.1).status == LOOKUP_GAP_TOO_LARGE
    result = buffer.lookup_many((500_000_000,), 2.0)
    assert result.status == LOOKUP_OK
    assert np.isclose(np.linalg.norm(result.quaternions[0]), 1.0)


def test_ground_filter_requires_predicted_continuous_floor_component():
    angles = np.deg2rad(np.linspace(-8.0, 8.0, 17))
    pitch = np.deg2rad(2.0)
    height = 0.11
    predicted = height / np.sin(pitch)
    ranges = np.full(len(angles), predicted)
    quaternions = np.tile(quaternion_from_rpy(0.0, 0.0, 0.0), (len(angles), 1))
    config = GroundFilterConfig(
        min_component_beams=6,
        min_component_angle_rad=np.deg2rad(4.0),
        min_component_span_m=0.20,
        max_planar_elevation_rad=np.deg2rad(5.0),
        min_retained_beams=1,
        degraded_ground_fraction=1.1,
    )
    result = mask_ground_and_offplane(
        ranges, angles, np.arange(len(angles)), quaternions,
        quaternion_from_rpy(0.0, 0.0, 0.0),
        [0.27, 0.0, height], quaternion_from_rpy(0.0, pitch, 0.0), config)
    assert np.count_nonzero(result.ground_mask) == len(ranges)

    ranges[8] -= 0.5
    result = mask_ground_and_offplane(
        ranges, angles, np.arange(len(angles)), quaternions,
        quaternion_from_rpy(0.0, 0.0, 0.0),
        [0.27, 0.0, height], quaternion_from_rpy(0.0, pitch, 0.0), config)
    assert not result.ground_mask[8]


def test_unsubstantiated_ahrs_tilt_falls_back_to_static_extrinsic():
    count = 20
    result = mask_ground_and_offplane(
        np.full(count, 3.0), np.linspace(-1.0, 1.0, count), np.arange(count),
        np.tile(quaternion_from_rpy(0.0, np.deg2rad(8.0), 0.0), (count, 1)),
        quaternion_from_rpy(0.0, 0.0, 0.0), [0.27, 0.0, 0.11],
        quaternion_from_rpy(0.0, 0.0, 0.0),
        GroundFilterConfig(min_retained_beams=5),
    )
    assert not result.degraded
    assert result.reason == 'attitude_fallback'
    assert result.attitude_fallback
    assert not np.any(result.offplane_mask)


def test_upward_tilt_with_few_returns_is_degraded_as_sky_frame():
    count = 20
    ranges = np.full(count, np.nan)
    ranges[:4] = 3.0
    result = mask_ground_and_offplane(
        ranges, np.linspace(-1.0, 1.0, count), np.arange(count),
        np.tile(quaternion_from_rpy(0.0, np.deg2rad(-8.0), 0.0), (count, 1)),
        quaternion_from_rpy(0.0, 0.0, 0.0), [0.27, 0.0, 0.11],
        quaternion_from_rpy(0.0, 0.0, 0.0),
        GroundFilterConfig(min_retained_beams=1),
    )
    assert result.degraded
    assert result.reason == 'offplane_low_returns'
    assert not result.attitude_fallback


def test_large_tilt_with_continuous_floor_evidence_does_not_fallback():
    count = 31
    pitch = np.deg2rad(8.0)
    angles = np.linspace(-0.5, 0.5, count)
    tilted_height = 0.11 * np.cos(pitch) - 0.27 * np.sin(pitch)
    ranges = tilted_height / (np.sin(pitch) * np.cos(angles))
    result = mask_ground_and_offplane(
        ranges, angles, np.arange(count),
        np.tile(quaternion_from_rpy(0.0, pitch, 0.0), (count, 1)),
        quaternion_from_rpy(0.0, 0.0, 0.0), [0.27, 0.0, 0.11],
        quaternion_from_rpy(0.0, 0.0, 0.0),
        GroundFilterConfig(min_retained_beams=1),
    )
    assert result.degraded
    assert not result.attitude_fallback
    assert np.count_nonzero(result.ground_mask) == count


def test_sparse_downsample_hits_do_not_form_a_floor_component():
    count = 241
    pitch = np.deg2rad(8.0)
    angles = np.deg2rad(np.linspace(-30.0, 30.0, count))
    ranges = np.full(count, 3.0)
    sparse = np.arange(0, count, 15)
    tilted_height = 0.11 * np.cos(pitch) - 0.27 * np.sin(pitch)
    ranges[sparse] = tilted_height / (
        np.sin(pitch) * np.cos(angles[sparse]))

    result = mask_ground_and_offplane(
        ranges, angles, np.arange(count),
        np.tile(quaternion_from_rpy(0.0, pitch, 0.0), (count, 1)),
        quaternion_from_rpy(0.0, 0.0, 0.0), [0.27, 0.0, 0.11],
        quaternion_from_rpy(0.0, 0.0, 0.0),
        GroundFilterConfig(min_retained_beams=1),
    )
    assert result.attitude_fallback
    assert not np.any(result.ground_mask)


def test_side_only_floor_shape_does_not_validate_dynamic_attitude():
    count = 1081
    roll = np.deg2rad(8.0)
    angles = np.deg2rad(np.linspace(-135.0, 135.0, count))
    ranges = np.full(count, 3.0)
    side = (angles >= np.deg2rad(-120.0)) & (angles <= np.deg2rad(-90.0))
    ranges[side] = (
        -0.11 * np.cos(roll) / (np.sin(roll) * np.sin(angles[side])))

    result = mask_ground_and_offplane(
        ranges, angles, np.arange(count),
        np.tile(quaternion_from_rpy(roll, 0.0, 0.0), (count, 1)),
        quaternion_from_rpy(0.0, 0.0, 0.0), [0.27, 0.0, 0.11],
        quaternion_from_rpy(0.0, 0.0, 0.0),
        GroundFilterConfig(
            attitude_evidence_max_abs_angle_rad=np.deg2rad(80.0),
            min_retained_beams=1,
        ),
    )
    assert result.attitude_fallback
    assert not np.any(result.ground_mask)


def test_3d_deskew_compensates_translation_to_midpoint():
    points = np.array([[2.0, 0.0], [2.0, 0.0]])
    poses = np.array([[-0.1, 0.0, 0.0], [0.1, 0.0, 0.0]])
    identity = quaternion_from_rpy(0.0, 0.0, 0.0)
    result = deskew_points_to_reference(
        points, poses, [0.0, 0.0, 0.0], np.tile(identity, (2, 1)), identity,
        identity, [0.0, 0.0, 0.0], identity)
    assert np.allclose(result.points[:, 0], [1.9, 2.1])
    assert np.isclose(result.max_translation_m, 0.1)
