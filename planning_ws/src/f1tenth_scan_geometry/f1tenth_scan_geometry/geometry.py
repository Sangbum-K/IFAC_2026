"""Quaternion helpers using ROS's xyzw convention."""

import numpy as np


def normalize_quaternions(quaternions):
    """Normalize one quaternion or an ``(N, 4)`` quaternion array."""
    q = np.asarray(quaternions, dtype=np.float64)
    if q.shape[-1] != 4:
        raise ValueError('quaternions must have a final dimension of four')
    norms = np.linalg.norm(q, axis=-1, keepdims=True)
    if np.any(~np.isfinite(norms)) or np.any(norms < 1e-12):
        raise ValueError('quaternion contains non-finite or zero-norm data')
    return q / norms


def quaternion_conjugate(quaternions):
    q = np.asarray(quaternions, dtype=np.float64).copy()
    q[..., :3] *= -1.0
    return q


def quaternion_multiply(lhs, rhs):
    """Compose xyzw quaternions, with NumPy broadcasting."""
    a = np.asarray(lhs, dtype=np.float64)
    b = np.asarray(rhs, dtype=np.float64)
    ax, ay, az, aw = np.moveaxis(a, -1, 0)
    bx, by, bz, bw = np.moveaxis(b, -1, 0)
    return np.stack([
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
        aw * bw - ax * bx - ay * by - az * bz,
    ], axis=-1)


def quaternion_to_matrix(quaternions):
    """Convert xyzw quaternions to rotation matrices."""
    q = normalize_quaternions(quaternions)
    x, y, z, w = np.moveaxis(q, -1, 0)
    xx, yy, zz = x * x, y * y, z * z
    xy, xz, yz = x * y, x * z, y * z
    wx, wy, wz = w * x, w * y, w * z
    rows = (
        np.stack((1.0 - 2.0 * (yy + zz),
                  2.0 * (xy - wz), 2.0 * (xz + wy)), axis=-1),
        np.stack((2.0 * (xy + wz), 1.0 - 2.0 * (xx + zz),
                  2.0 * (yz - wx)), axis=-1),
        np.stack((2.0 * (xz - wy), 2.0 * (yz + wx),
                  1.0 - 2.0 * (xx + yy)), axis=-1),
    )
    return np.stack(rows, axis=-2)


def quaternion_from_matrix(matrix):
    """Convert one proper 3x3 rotation matrix to an xyzw quaternion."""
    m = np.asarray(matrix, dtype=np.float64)
    if m.shape != (3, 3) or not np.all(np.isfinite(m)):
        raise ValueError('matrix must be a finite 3x3 array')
    trace = float(np.trace(m))
    if trace > 0.0:
        s = np.sqrt(trace + 1.0) * 2.0
        q = np.array([
            (m[2, 1] - m[1, 2]) / s,
            (m[0, 2] - m[2, 0]) / s,
            (m[1, 0] - m[0, 1]) / s,
            0.25 * s,
        ])
    else:
        i = int(np.argmax(np.diag(m)))
        if i == 0:
            s = np.sqrt(1.0 + m[0, 0] - m[1, 1] - m[2, 2]) * 2.0
            q = np.array([0.25 * s,
                          (m[0, 1] + m[1, 0]) / s,
                          (m[0, 2] + m[2, 0]) / s,
                          (m[2, 1] - m[1, 2]) / s])
        elif i == 1:
            s = np.sqrt(1.0 + m[1, 1] - m[0, 0] - m[2, 2]) * 2.0
            q = np.array([(m[0, 1] + m[1, 0]) / s,
                          0.25 * s,
                          (m[1, 2] + m[2, 1]) / s,
                          (m[0, 2] - m[2, 0]) / s])
        else:
            s = np.sqrt(1.0 + m[2, 2] - m[0, 0] - m[1, 1]) * 2.0
            q = np.array([(m[0, 2] + m[2, 0]) / s,
                          (m[1, 2] + m[2, 1]) / s,
                          0.25 * s,
                          (m[1, 0] - m[0, 1]) / s])
    return normalize_quaternions(q)


def matrix_from_rpy(roll, pitch, yaw):
    """Return ``Rz(yaw) @ Ry(pitch) @ Rx(roll)``."""
    cr, sr = np.cos(roll), np.sin(roll)
    cp, sp = np.cos(pitch), np.sin(pitch)
    cy, sy = np.cos(yaw), np.sin(yaw)
    return np.array([
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp, cp * sr, cp * cr],
    ], dtype=np.float64)


def quaternion_from_rpy(roll, pitch, yaw):
    return quaternion_from_matrix(matrix_from_rpy(roll, pitch, yaw))


def slerp_quaternions(lower, upper, ratios):
    """Vectorized shortest-path spherical interpolation."""
    q0 = normalize_quaternions(lower)
    q1 = normalize_quaternions(upper)
    ratio = np.asarray(ratios, dtype=np.float64)
    dots = np.sum(q0 * q1, axis=-1)
    q1 = np.where((dots < 0.0)[..., None], -q1, q1)
    dots = np.abs(dots)
    linear = dots > 0.9995
    theta = np.arccos(np.clip(dots, -1.0, 1.0))
    sin_theta = np.sin(theta)
    safe = np.where(sin_theta > 1e-12, sin_theta, 1.0)
    a = np.sin((1.0 - ratio) * theta) / safe
    b = np.sin(ratio * theta) / safe
    out = a[..., None] * q0 + b[..., None] * q1
    linear_out = (1.0 - ratio)[..., None] * q0 + ratio[..., None] * q1
    out = np.where(linear[..., None], linear_out, out)
    return normalize_quaternions(out)


def base_orientation_from_imu(imu_quaternions, base_to_imu_quaternion):
    """Convert world<-IMU orientations to world<-base orientations."""
    q_world_imu = normalize_quaternions(imu_quaternions)
    q_base_imu = normalize_quaternions(base_to_imu_quaternion)
    q_imu_base = quaternion_conjugate(q_base_imu)
    return normalize_quaternions(
        quaternion_multiply(q_world_imu, q_imu_base))


def level_relative_matrices(base_quaternions, level_quaternion):
    """Return level-reference<-current-base rotations."""
    current = quaternion_to_matrix(base_quaternions)
    level = quaternion_to_matrix(level_quaternion)
    return np.einsum('ji,njk->nik', level, current)


def roll_pitch_from_matrices(matrices):
    """Extract ZYX roll and pitch; yaw is intentionally discarded."""
    r = np.asarray(matrices, dtype=np.float64)
    pitch = np.arctan2(-r[..., 2, 0],
                       np.hypot(r[..., 0, 0], r[..., 1, 0]))
    roll = np.arctan2(r[..., 2, 1], r[..., 2, 2])
    return roll, pitch


def rotations_from_yaw_and_level_tilt(yaws, base_quaternions,
                                      level_quaternion):
    """Fuse odom yaw with IMU roll/pitch relative to a level reference."""
    yaws = np.asarray(yaws, dtype=np.float64)
    relative = level_relative_matrices(base_quaternions, level_quaternion)
    rolls, pitches = roll_pitch_from_matrices(relative)
    output = np.empty((len(yaws), 3, 3), dtype=np.float64)
    for index, (roll, pitch, yaw) in enumerate(zip(rolls, pitches, yaws)):
        output[index] = matrix_from_rpy(roll, pitch, yaw)
    return output
