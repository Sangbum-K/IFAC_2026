# MIT License

# Copyright (c) 2020 Hongrui Zheng, Corey Walsh

# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the 'Software'), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:

# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.

# THE SOFTWARE IS PROVIDED 'AS IS', WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.

import copy
from dataclasses import dataclass
from threading import current_thread, Event, Lock, Thread
import time

from geometry_msgs.msg import PointStamped
from geometry_msgs.msg import PolygonStamped
from geometry_msgs.msg import PoseArray
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import PoseWithCovarianceStamped
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from nav_msgs.srv import GetMap
import numpy as np
from particle_filter import utils as Utils
from particle_filter.mcl_core import (
    can_bypass_jump_guard_for_slip,
    deskew_scan_to_reference,
    effective_sample_size,
    estimate_scan_motion,
    likelihoods_from_log_table,
    LocalizationQualityTracker,
    LocalizationReadinessTracker,
    make_random_generators,
    posterior_from_likelihood,
    prepare_mcl_proposal,
    QualityState,
    relative_pose_batch,
    select_beams,
    should_hold_stationary_filter,
    SlipDetector,
)
from particle_filter.synchronization import ImuSample
from particle_filter.synchronization import LatestValueSlot
from particle_filter.synchronization import LOOKUP_OK
from particle_filter.synchronization import OdomSample
from particle_filter.synchronization import TimestampedImuBuffer
from particle_filter.synchronization import TimestampedOdomBuffer
from f1tenth_scan_geometry import AttitudeSample
from f1tenth_scan_geometry import GroundFilterConfig
from f1tenth_scan_geometry import LevelReference
from f1tenth_scan_geometry import TimestampedAttitudeBuffer
from f1tenth_scan_geometry import base_orientation_from_imu
from f1tenth_scan_geometry import mask_ground_and_offplane
from f1tenth_scan_geometry import quaternion_from_rpy
import range_libc
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time as RosTime
from sensor_msgs.msg import Imu, LaserScan
from std_msgs.msg import Bool, String
from tf2_ros import Buffer, TransformBroadcaster, TransformListener
import tf_transformations

'''
These flags indicate several variants of the sensor model. Only one of them is used at a time.
'''
VAR_NO_EVAL_SENSOR_MODEL = 0
VAR_CALC_RANGE_MANY_EVAL_SENSOR = 1
VAR_REPEAT_ANGLES_EVAL_SENSOR = 2
VAR_REPEAT_ANGLES_EVAL_SENSOR_ONE_SHOT = 3
VAR_RADIAL_CDDT_OPTIMIZATIONS = 4
VAR_REPEAT_ANGLES_LOG_SENSOR = 5


@dataclass(frozen=True)
class ScanSample:
    """A preprocessed scan waiting for timestamp-synchronized PF work."""

    stamp_ns: int
    received_monotonic: float
    ranges: np.ndarray
    valid_returns: np.ndarray
    geometry_ranges: np.ndarray
    beam_offsets_ns: np.ndarray
    geometry_beam_offsets_ns: np.ndarray
    deskew_timing_valid: bool
    scan_duration_sec: float


class ParticleFiler(Node):
    '''
    This class implements Monte Carlo Localization based on odometry and a laser scanner.
    '''

    def __init__(self):
        super().__init__('particle_filter')

        # declare parameters
        self.declare_parameter('angle_step')
        self.declare_parameter('max_particles')
        self.declare_parameter('max_viz_particles')
        self.declare_parameter('squash_factor')
        self.declare_parameter('max_range')
        self.declare_parameter('theta_discretization')
        self.declare_parameter('range_method')
        self.declare_parameter('rangelib_variant')
        # Identifies the immutable launch-time A/B overlay in diagnostics.
        # It does not change behavior by itself.
        self.declare_parameter('pf_profile_label', 'unprofiled')
        # Negative keeps production nondeterministic.  Offline A/B replay
        # supplies one fixed non-negative seed to every feature profile.
        self.declare_parameter('random_seed', -1)
        self.declare_parameter('fine_timing')
        self.declare_parameter('publish_odom')
        self.declare_parameter('viz')
        self.declare_parameter('z_short')
        self.declare_parameter('z_max')
        self.declare_parameter('z_rand')
        self.declare_parameter('z_hit')
        self.declare_parameter('sigma_hit')
        self.declare_parameter('motion_dispersion_x')
        self.declare_parameter('motion_dispersion_y')
        self.declare_parameter('motion_dispersion_theta')
        self.declare_parameter('scan_topic')
        self.declare_parameter('odometry_topic')
        self.declare_parameter('imu_topic', '/sensors/imu/raw')
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('laser_frame', 'laser')
        self.declare_parameter('pose_jump_guard_enabled', True)
        self.declare_parameter('max_pose_jump_translation_m', 0.30)
        self.declare_parameter('max_pose_jump_yaw_rad', 0.35)
        self.declare_parameter('low_speed_threshold_mps', 0.2)
        self.declare_parameter('low_speed_odom_xy_deadband_m', 0.01)
        self.declare_parameter('low_speed_odom_yaw_deadband_rad', 0.02)
        self.declare_parameter('low_speed_odom_scale', 0.25)
        self.declare_parameter('low_speed_jump_guard_translation_m', 0.50)
        self.declare_parameter('low_speed_jump_guard_yaw_rad', 0.60)
        # 가드가 '영구 거부권'이 되는 것을 막는 탈출구. 0 = 비활성(구 동작).
        #
        # 가드는 단발성 스냅을 걸러내라고 있는데, 거부하면 odom 전파 포즈를
        # 유지하므로 진짜 오차가 그대로 남는다 -> 다음 프레임 보정량은 더
        # 커지고 -> 또 거부. 한 번 문턱을 넘으면 스캔 업데이트가 영원히
        # 차단되고 PF는 사실상 데드레커닝이 된다.
        # run_0803_204743 실측: 3035회 업데이트 중 ~725회(24%) 거부, 연속으로
        # 몰려 발생했고 그 사이 포즈가 10.09m / 10.67m 점프했다.
        # 연속 거부가 이 횟수에 닿으면 "필터가 아니라 예측이 틀렸다"고 보고
        # 보정을 한 번 받아들여 재동기화한다.
        self.declare_parameter('max_consecutive_jump_rejects', 10)
        # Jetson-safe scan/odom synchronization. Odom callbacks only append a
        # small sample; a dedicated worker always processes the newest scan.
        self.declare_parameter('odom_buffer_size', 150)
        self.declare_parameter('odom_sync_wait_sec', 0.04)
        self.declare_parameter('max_odom_interp_gap_sec', 0.06)
        self.declare_parameter('max_pending_scan_age_sec', 0.10)
        self.declare_parameter('max_update_rate_hz', 15.0)
        self.declare_parameter('sync_diagnostics_period_sec', 1.0)
        self.declare_parameter('max_localization_update_age_sec', 0.50)
        # Fixed noise on a zero-motion update creates artificial pose shake.
        # Scale the existing noise values by motion, while retaining a small
        # floor so stopped particles do not become completely deterministic.
        self.declare_parameter('motion_noise_min_scale', 0.08)
        self.declare_parameter('motion_noise_reference_translation_m', 0.25)
        self.declare_parameter('motion_noise_reference_yaw_rad', 0.15)
        # Keep particle diversity until degeneracy is measurable instead of
        # paying for multinomial resampling on every scan.
        self.declare_parameter('resample_ess_ratio', 0.50)
        # AMCL-style dynamic-obstacle rejection. A global mismatch restores
        # all beams so a lost cloud cannot hide its failure by skipping them.
        self.declare_parameter('beam_skip_enabled', True)
        self.declare_parameter('beam_skip_distance_m', 0.20)
        self.declare_parameter('beam_skip_min_particle_fraction', 0.30)
        self.declare_parameter('beam_skip_max_fraction', 0.50)
        self.declare_parameter('beam_skip_short_only', True)
        # This hysteresis always drives autonomy readiness. The optional gate
        # below additionally suppresses PF pose output while LOST; keep that
        # separate so diagnostics and recovery still have a pose to inspect.
        self.declare_parameter('quality_ok_inlier_fraction', 0.70)
        self.declare_parameter('quality_lost_inlier_fraction', 0.40)
        self.declare_parameter('quality_bad_updates_to_lost', 3)
        self.declare_parameter('quality_good_updates_to_recover', 5)
        self.declare_parameter('quality_gate_enabled', False)
        self.declare_parameter(
            'localization_ready_topic', '/pf/localization_ready')
        self.declare_parameter(
            'localization_quality_topic', '/pf/localization_quality')
        self.declare_parameter(
            'initialpose_received_topic', '/pf/initialpose_received')
        self.declare_parameter(
            'initialpose_accepted_topic', '/pf/initialpose_accepted')
        # A zero odometry action is a physical constraint: scan noise must not
        # move the map pose while the car is stationary.
        self.declare_parameter('freeze_stationary_filter', True)
        # Represent the selected beams at the scan midpoint. This uses no
        # additional raycasts: only the existing downsampled endpoints move.
        self.declare_parameter('deskew_enabled', True)
        self.declare_parameter('deskew_max_scan_duration_sec', 0.05)
        self.declare_parameter('deskew_slip_translation_scale', 0.0)
        # VESC raw IMU is 50 Hz and currently reports gyro in deg/s and
        # acceleration in g. Keep both scales explicit for future sensors.
        self.declare_parameter('imu_buffer_size', 150)
        self.declare_parameter('imu_sync_wait_sec', 0.02)
        self.declare_parameter('max_imu_interp_gap_sec', 0.06)
        self.declare_parameter(
            'imu_angular_velocity_scale', np.pi / 180.0)
        self.declare_parameter('imu_linear_acceleration_scale', 9.80665)
        # Full 3D attitude is buffered separately from the existing yaw-rate
        # integration so the same selected beams can be ground-gated before
        # both ICP and MCL without changing the fixed RangeLib ray budget.
        self.declare_parameter('base_to_imu_rpy_deg', [0.0, 0.0, 0.0])
        self.declare_parameter('level_reference_samples', 20)
        self.declare_parameter('level_reference_max_gyro_rad_s', 0.12)
        self.declare_parameter('level_reference_accel_tolerance_mps2', 2.5)
        self.declare_parameter('ground_filter_enabled', True)
        self.declare_parameter('ground_z_m', 0.0)
        self.declare_parameter('ground_range_tolerance_m', 0.08)
        self.declare_parameter('ground_range_tolerance_ratio', 0.04)
        self.declare_parameter('ground_endpoint_z_tolerance_m', 0.05)
        self.declare_parameter('ground_min_component_beams', 6)
        self.declare_parameter('ground_min_component_angle_deg', 4.0)
        self.declare_parameter('ground_min_component_span_m', 0.30)
        self.declare_parameter('max_planar_elevation_deg', 4.0)
        self.declare_parameter('degraded_offplane_fraction', 0.55)
        self.declare_parameter('degraded_ground_fraction', 0.70)
        self.declare_parameter('attitude_fallback_tilt_deg', 4.0)
        self.declare_parameter('attitude_ground_evidence_fraction', 0.05)
        self.declare_parameter('attitude_ground_evidence_min_beams', 6)
        self.declare_parameter('attitude_evidence_max_abs_angle_deg', 80.0)
        self.declare_parameter('degraded_valid_fraction', 0.25)
        self.declare_parameter('ground_min_retained_beams', 20)
        self.declare_parameter('stationary_imu_yaw_rate_rad_s', 0.06)
        self.declare_parameter(
            'stationary_imu_horizontal_accel_mps2', 1.60)
        self.declare_parameter('stationary_scan_translation_m', 0.04)
        self.declare_parameter('stationary_scan_yaw_rad', 0.04)
        # The matcher runs on the existing 73 downsampled beams: four small
        # O(B^2) ICP passes, independent of the 2000-particle raycast budget.
        self.declare_parameter('scan_match_enabled', True)
        self.declare_parameter('scan_match_max_correspondence_m', 0.40)
        self.declare_parameter('scan_match_trim_fraction', 0.90)
        self.declare_parameter('scan_match_iterations', 4)
        self.declare_parameter('scan_match_min_points', 12)
        self.declare_parameter('scan_match_min_inlier_fraction', 0.55)
        self.declare_parameter('scan_match_max_rmse_m', 0.15)
        # Persistent normalized innovation activates slip handling. Recovery is
        # deliberately slower so one good scan cannot re-enable a risky hold.
        self.declare_parameter('slip_translation_innovation_m', 0.15)
        self.declare_parameter('slip_yaw_innovation_rad', 0.15)
        self.declare_parameter('slip_enter_score', 1.0)
        self.declare_parameter('slip_immediate_score', 2.0)
        self.declare_parameter('slip_recover_score', 0.50)
        self.declare_parameter('slip_enter_updates', 2)
        self.declare_parameter('slip_recover_updates', 5)
        self.declare_parameter('slip_motion_noise_multiplier', 2.5)
        # A slip correction may bypass the normal jump guard only with strong
        # map support and inside a second, absolute safety envelope.
        self.declare_parameter('slip_jump_bypass_min_scan_fit', 0.75)
        self.declare_parameter(
            'slip_jump_bypass_max_translation_m', 0.80)
        self.declare_parameter('slip_jump_bypass_max_yaw_rad', 0.80)

        # parameters
        self.ANGLE_STEP           = self.get_parameter('angle_step').value
        self.MAX_PARTICLES        = self.get_parameter('max_particles').value
        self.MAX_VIZ_PARTICLES    = self.get_parameter('max_viz_particles').value
        self.INV_SQUASH_FACTOR    = 1.0 / self.get_parameter('squash_factor').value
        self.MAX_RANGE_METERS     = self.get_parameter('max_range').value
        self.THETA_DISCRETIZATION = self.get_parameter('theta_discretization').value
        self.WHICH_RM             = self.get_parameter('range_method').value
        self.RANGELIB_VAR         = self.get_parameter('rangelib_variant').value
        self.PF_PROFILE_LABEL = str(self.get_parameter(
            'pf_profile_label').value).strip() or 'unprofiled'
        self.RANDOM_SEED = int(self.get_parameter('random_seed').value)
        self.SHOW_FINE_TIMING     = self.get_parameter('fine_timing').value
        self.PUBLISH_ODOM         = self.get_parameter('publish_odom').value
        self.DO_VIZ               = self.get_parameter('viz').value

        # sensor model constants
        self.Z_SHORT   = self.get_parameter('z_short').value
        self.Z_MAX     = self.get_parameter('z_max').value
        self.Z_RAND    = self.get_parameter('z_rand').value
        self.Z_HIT     = self.get_parameter('z_hit').value
        self.SIGMA_HIT = self.get_parameter('sigma_hit').value

        # motion model constants
        self.MOTION_DISPERSION_X     = self.get_parameter('motion_dispersion_x').value
        self.MOTION_DISPERSION_Y     = self.get_parameter('motion_dispersion_y').value
        self.MOTION_DISPERSION_THETA = self.get_parameter('motion_dispersion_theta').value
        self.MAP_FRAME               = self.get_parameter('map_frame').value
        self.ODOM_FRAME              = self.get_parameter('odom_frame').value
        self.BASE_FRAME              = self.get_parameter('base_frame').value
        self.LASER_FRAME             = self.get_parameter('laser_frame').value
        self.POSE_JUMP_GUARD_ENABLED = self.get_parameter('pose_jump_guard_enabled').value
        self.MAX_POSE_JUMP_TRANSLATION = self.get_parameter(
            'max_pose_jump_translation_m').value
        self.MAX_POSE_JUMP_YAW = self.get_parameter('max_pose_jump_yaw_rad').value
        self.LOW_SPEED_THRESHOLD = self.get_parameter(
            'low_speed_threshold_mps').value
        self.LOW_SPEED_ODOM_XY_DEADBAND = self.get_parameter(
            'low_speed_odom_xy_deadband_m').value
        self.LOW_SPEED_ODOM_YAW_DEADBAND = self.get_parameter(
            'low_speed_odom_yaw_deadband_rad').value
        self.LOW_SPEED_ODOM_SCALE = self.get_parameter(
            'low_speed_odom_scale').value
        self.LOW_SPEED_JUMP_GUARD_TRANSLATION = self.get_parameter(
            'low_speed_jump_guard_translation_m').value
        self.LOW_SPEED_JUMP_GUARD_YAW = self.get_parameter(
            'low_speed_jump_guard_yaw_rad').value
        self.MAX_CONSECUTIVE_JUMP_REJECTS = int(self.get_parameter(
            'max_consecutive_jump_rejects').value)
        if self.MAX_CONSECUTIVE_JUMP_REJECTS < 0:
            raise ValueError('max_consecutive_jump_rejects must be >= 0')
        self.ODOM_BUFFER_SIZE = int(self.get_parameter('odom_buffer_size').value)
        self.ODOM_SYNC_WAIT_SEC = float(self.get_parameter('odom_sync_wait_sec').value)
        self.MAX_ODOM_INTERP_GAP_SEC = float(self.get_parameter(
            'max_odom_interp_gap_sec').value)
        self.MAX_PENDING_SCAN_AGE_SEC = float(self.get_parameter(
            'max_pending_scan_age_sec').value)
        self.MAX_UPDATE_RATE_HZ = float(self.get_parameter('max_update_rate_hz').value)
        self.SYNC_DIAGNOSTICS_PERIOD_SEC = float(self.get_parameter(
            'sync_diagnostics_period_sec').value)
        self.MAX_LOCALIZATION_UPDATE_AGE_SEC = float(self.get_parameter(
            'max_localization_update_age_sec').value)
        self.MOTION_NOISE_MIN_SCALE = float(self.get_parameter(
            'motion_noise_min_scale').value)
        self.MOTION_NOISE_REFERENCE_TRANSLATION = float(self.get_parameter(
            'motion_noise_reference_translation_m').value)
        self.MOTION_NOISE_REFERENCE_YAW = float(self.get_parameter(
            'motion_noise_reference_yaw_rad').value)
        self.RESAMPLE_ESS_RATIO = float(self.get_parameter(
            'resample_ess_ratio').value)
        self.BEAM_SKIP_ENABLED = bool(self.get_parameter(
            'beam_skip_enabled').value)
        self.BEAM_SKIP_DISTANCE = float(self.get_parameter(
            'beam_skip_distance_m').value)
        self.BEAM_SKIP_MIN_PARTICLE_FRACTION = float(self.get_parameter(
            'beam_skip_min_particle_fraction').value)
        self.BEAM_SKIP_MAX_FRACTION = float(self.get_parameter(
            'beam_skip_max_fraction').value)
        self.BEAM_SKIP_SHORT_ONLY = bool(self.get_parameter(
            'beam_skip_short_only').value)
        self.QUALITY_OK_INLIER_FRACTION = float(self.get_parameter(
            'quality_ok_inlier_fraction').value)
        self.QUALITY_LOST_INLIER_FRACTION = float(self.get_parameter(
            'quality_lost_inlier_fraction').value)
        self.QUALITY_BAD_UPDATES_TO_LOST = int(self.get_parameter(
            'quality_bad_updates_to_lost').value)
        self.QUALITY_GOOD_UPDATES_TO_RECOVER = int(self.get_parameter(
            'quality_good_updates_to_recover').value)
        self.QUALITY_GATE_ENABLED = bool(self.get_parameter(
            'quality_gate_enabled').value)
        self.LOCALIZATION_READY_TOPIC = self.get_parameter(
            'localization_ready_topic').value
        self.LOCALIZATION_QUALITY_TOPIC = self.get_parameter(
            'localization_quality_topic').value
        self.INITIALPOSE_RECEIVED_TOPIC = self.get_parameter(
            'initialpose_received_topic').value
        self.INITIALPOSE_ACCEPTED_TOPIC = self.get_parameter(
            'initialpose_accepted_topic').value
        self.FREEZE_STATIONARY_FILTER = bool(self.get_parameter(
            'freeze_stationary_filter').value)
        self.DESKEW_ENABLED = bool(self.get_parameter(
            'deskew_enabled').value)
        self.DESKEW_MAX_SCAN_DURATION_SEC = float(self.get_parameter(
            'deskew_max_scan_duration_sec').value)
        self.DESKEW_SLIP_TRANSLATION_SCALE = float(self.get_parameter(
            'deskew_slip_translation_scale').value)
        self.IMU_BUFFER_SIZE = int(self.get_parameter(
            'imu_buffer_size').value)
        self.IMU_SYNC_WAIT_SEC = float(self.get_parameter(
            'imu_sync_wait_sec').value)
        self.MAX_IMU_INTERP_GAP_SEC = float(self.get_parameter(
            'max_imu_interp_gap_sec').value)
        self.IMU_ANGULAR_VELOCITY_SCALE = float(self.get_parameter(
            'imu_angular_velocity_scale').value)
        self.IMU_LINEAR_ACCELERATION_SCALE = float(self.get_parameter(
            'imu_linear_acceleration_scale').value)
        base_to_imu_rpy = np.asarray(self.get_parameter(
            'base_to_imu_rpy_deg').value, dtype=np.float64)
        if (base_to_imu_rpy.shape != (3,)
                or not np.all(np.isfinite(base_to_imu_rpy))):
            raise ValueError(
                'base_to_imu_rpy_deg must contain three finite values')
        self.BASE_TO_IMU_FALLBACK = quaternion_from_rpy(
            *np.deg2rad(base_to_imu_rpy))
        self.LEVEL_REFERENCE_SAMPLES = int(self.get_parameter(
            'level_reference_samples').value)
        self.LEVEL_REFERENCE_MAX_GYRO = float(self.get_parameter(
            'level_reference_max_gyro_rad_s').value)
        self.LEVEL_REFERENCE_ACCEL_TOLERANCE = float(self.get_parameter(
            'level_reference_accel_tolerance_mps2').value)
        self.GROUND_FILTER_ENABLED = bool(self.get_parameter(
            'ground_filter_enabled').value)
        self.GROUND_FILTER_CONFIG = GroundFilterConfig(
            ground_z_m=float(self.get_parameter('ground_z_m').value),
            range_tolerance_m=float(self.get_parameter(
                'ground_range_tolerance_m').value),
            range_tolerance_ratio=float(self.get_parameter(
                'ground_range_tolerance_ratio').value),
            endpoint_z_tolerance_m=float(self.get_parameter(
                'ground_endpoint_z_tolerance_m').value),
            min_component_beams=int(self.get_parameter(
                'ground_min_component_beams').value),
            min_component_angle_rad=np.deg2rad(float(self.get_parameter(
                'ground_min_component_angle_deg').value)),
            min_component_span_m=float(self.get_parameter(
                'ground_min_component_span_m').value),
            max_planar_elevation_rad=np.deg2rad(float(self.get_parameter(
                'max_planar_elevation_deg').value)),
            degraded_offplane_fraction=float(self.get_parameter(
                'degraded_offplane_fraction').value),
            degraded_ground_fraction=float(self.get_parameter(
                'degraded_ground_fraction').value),
            attitude_fallback_tilt_rad=np.deg2rad(float(self.get_parameter(
                'attitude_fallback_tilt_deg').value)),
            attitude_ground_evidence_fraction=float(self.get_parameter(
                'attitude_ground_evidence_fraction').value),
            attitude_ground_evidence_min_beams=int(self.get_parameter(
                'attitude_ground_evidence_min_beams').value),
            attitude_evidence_max_abs_angle_rad=np.deg2rad(float(
                self.get_parameter(
                    'attitude_evidence_max_abs_angle_deg').value)),
            degraded_valid_fraction=float(self.get_parameter(
                'degraded_valid_fraction').value),
            min_retained_beams=int(self.get_parameter(
                'ground_min_retained_beams').value),
        )
        self.STATIONARY_IMU_YAW_RATE = float(self.get_parameter(
            'stationary_imu_yaw_rate_rad_s').value)
        self.STATIONARY_IMU_HORIZONTAL_ACCEL = float(self.get_parameter(
            'stationary_imu_horizontal_accel_mps2').value)
        self.STATIONARY_SCAN_TRANSLATION = float(self.get_parameter(
            'stationary_scan_translation_m').value)
        self.STATIONARY_SCAN_YAW = float(self.get_parameter(
            'stationary_scan_yaw_rad').value)
        self.SCAN_MATCH_ENABLED = bool(self.get_parameter(
            'scan_match_enabled').value)
        self.SCAN_MATCH_MAX_CORRESPONDENCE = float(self.get_parameter(
            'scan_match_max_correspondence_m').value)
        self.SCAN_MATCH_TRIM_FRACTION = float(self.get_parameter(
            'scan_match_trim_fraction').value)
        self.SCAN_MATCH_ITERATIONS = int(self.get_parameter(
            'scan_match_iterations').value)
        self.SCAN_MATCH_MIN_POINTS = int(self.get_parameter(
            'scan_match_min_points').value)
        self.SCAN_MATCH_MIN_INLIER_FRACTION = float(self.get_parameter(
            'scan_match_min_inlier_fraction').value)
        self.SCAN_MATCH_MAX_RMSE = float(self.get_parameter(
            'scan_match_max_rmse_m').value)
        self.SLIP_TRANSLATION_INNOVATION = float(self.get_parameter(
            'slip_translation_innovation_m').value)
        self.SLIP_YAW_INNOVATION = float(self.get_parameter(
            'slip_yaw_innovation_rad').value)
        self.SLIP_ENTER_SCORE = float(self.get_parameter(
            'slip_enter_score').value)
        self.SLIP_IMMEDIATE_SCORE = float(self.get_parameter(
            'slip_immediate_score').value)
        self.SLIP_RECOVER_SCORE = float(self.get_parameter(
            'slip_recover_score').value)
        self.SLIP_ENTER_UPDATES = int(self.get_parameter(
            'slip_enter_updates').value)
        self.SLIP_RECOVER_UPDATES = int(self.get_parameter(
            'slip_recover_updates').value)
        self.SLIP_MOTION_NOISE_MULTIPLIER = float(self.get_parameter(
            'slip_motion_noise_multiplier').value)
        self.SLIP_JUMP_BYPASS_MIN_SCAN_FIT = float(self.get_parameter(
            'slip_jump_bypass_min_scan_fit').value)
        self.SLIP_JUMP_BYPASS_MAX_TRANSLATION = float(self.get_parameter(
            'slip_jump_bypass_max_translation_m').value)
        self.SLIP_JUMP_BYPASS_MAX_YAW = float(self.get_parameter(
            'slip_jump_bypass_max_yaw_rad').value)
        self._validate_sync_parameters()
        
        # various data containers used in the MCL algorithm
        self.MAX_RANGE_PX = None
        self.laser = None
        self.iters = 0
        self.map_info = None
        self.map_initialized = False
        self.lidar_initialized = False
        self.odom_initialized = False
        self.imu_initialized = False
        self.laser_angles = None
        self.all_beam_indices = None
        self.selected_beam_indices = None
        self.expected_scan_size = None
        self.downsampled_angles = None
        self.current_scan_angles = None
        self.downsampled_ranges = None
        self.range_method = None
        self.last_time = None
        self.last_stamp = None
        self.first_sensor_update = True
        self.state_lock = Lock()
        self.transform_lock = Lock()
        self.filter_reference_lock = Lock()
        self.initialpose_lock = Lock()
        self.pending_initialpose = None
        self.last_initialpose_key = None
        self.last_initialpose_wait_log_time = 0.0
        self.jump_reject_count = 0            # 로그 스로틀용 누적 카운터
        self.consecutive_jump_rejects = 0     # 탈출구 판정용 연속 카운터

        # Input callbacks never run MCL. They only touch these bounded,
        # independently locked containers and return immediately.
        self.odom_buffer = TimestampedOdomBuffer(self.ODOM_BUFFER_SIZE)
        self.imu_buffer = TimestampedImuBuffer(self.IMU_BUFFER_SIZE)
        self.imu_attitude_buffer = TimestampedAttitudeBuffer(
            self.IMU_BUFFER_SIZE)
        self.attitude_lock = Lock()
        self.level_reference = LevelReference(
            min_samples=self.LEVEL_REFERENCE_SAMPLES,
            max_gyro_rad_s=self.LEVEL_REFERENCE_MAX_GYRO,
            accel_tolerance_mps2=self.LEVEL_REFERENCE_ACCEL_TOLERANCE,
        )
        self.base_to_imu_quaternion = np.copy(self.BASE_TO_IMU_FALLBACK)
        self.imu_extrinsic_frame = ''
        self.pending_scans = LatestValueSlot()
        self.worker_stop = Event()
        self.filter_worker = None
        self.previous_filter_odom = None
        self.previous_filter_scan_stamp_ns = None
        self.previous_filter_scan_ranges = None
        self.previous_filter_scan_angles = None
        self.filter_generation = 0
        self.map_to_odom = None
        self.map_to_odom_updated_monotonic = 0.0
        self.localized_covariance = np.zeros((6, 6))
        self.localization_output_stale_active = False
        self.scan_processed_count = 0
        self.scan_sync_drop_count = 0
        self.scan_stale_drop_count = 0
        self.scan_out_of_order_count = 0
        self.scan_shape_drop_count = 0
        self.worker_error_count = 0
        self.resample_count = 0
        self.beam_skip_fallback_count = 0
        self.quality_lost_update_count = 0
        self.quality_lost_transition_count = 0
        self.stationary_hold_count = 0
        self.imu_sync_miss_count = 0
        self.imu_reject_count = 0
        self.scan_match_invalid_count = 0
        self.deskew_applied_count = 0
        self.deskew_timing_invalid_count = 0
        self.deskew_odom_fallback_count = 0
        self.deskew_imu_fallback_count = 0
        self.ground_filter_sync_miss_count = 0
        self.ground_filtered_beam_count = 0
        self.offplane_filtered_beam_count = 0
        self.geometry_degraded_scan_count = 0
        self.attitude_fallback_scan_count = 0
        self.last_geometry_reason = 'waiting'
        self.last_deskew_source = 'OFF'
        self.last_deskew_translation_scale = 0.0
        self.last_deskew_max_translation_m = 0.0
        self.last_deskew_max_yaw_rad = 0.0
        self.last_scan_duration_sec = 0.0
        self.slip_event_count = 0
        self.slip_recovery_count = 0
        self.slip_jump_bypass_count = 0
        self.slip_enter_stamp_ns = None
        self.slip_last_duration_sec = 0.0
        self.slip_event_max_correction_m = 0.0
        self.slip_event_max_correction_yaw = 0.0
        self.slip_last_max_correction_m = 0.0
        self.slip_last_max_correction_yaw = 0.0
        self.last_slip_update = None
        self.last_scan_motion_valid = False
        self.last_scan_motion = np.zeros(3)
        self.last_scan_match_rmse_m = -1.0
        self.last_scan_match_inlier_fraction = -1.0
        self.last_wheel_stationary = False
        self.last_imu_stationary = False
        self.last_scan_stationary = False
        self.odom_reject_log_count = 0
        self.last_received_scan_stamp_ns = 0
        self.last_published_odom_stamp_ns = 0
        self.last_sync_diagnostic_time = time.monotonic()
        self.compute_time_ms = Utils.CircularArray(20)
        self.scan_turnaround_ms = Utils.CircularArray(20)

        # cache this to avoid memory allocation in motion model
        self.local_deltas = np.zeros((self.MAX_PARTICLES, 3))

        # cache this for the sensor model computation
        self.queries = None
        self.ranges = None
        self.tiled_angles = None
        self.sensor_model_table = None
        self.sensor_log_table = None
        self.query_grid = None
        self.range_pixels = None
        self.sensor_likelihoods = np.ones(
            self.MAX_PARTICLES, dtype=np.float64)
        self.last_beam_selection = None
        self.last_prior_ess_ratio = 1.0
        self.last_posterior_ess_ratio = 1.0
        self.last_resampled = False
        self.mcl_rng, self.viz_rng = make_random_generators(
            self.RANDOM_SEED)
        self.quality_tracker = LocalizationQualityTracker(
            self.QUALITY_OK_INLIER_FRACTION,
            self.QUALITY_LOST_INLIER_FRACTION,
            self.QUALITY_BAD_UPDATES_TO_LOST,
            self.QUALITY_GOOD_UPDATES_TO_RECOVER,
        )
        self.readiness_tracker = LocalizationReadinessTracker()
        self.slip_detector = SlipDetector(
            self.SLIP_TRANSLATION_INNOVATION,
            self.SLIP_YAW_INNOVATION,
            self.SLIP_ENTER_SCORE,
            self.SLIP_RECOVER_SCORE,
            self.SLIP_ENTER_UPDATES,
            self.SLIP_RECOVER_UPDATES,
            self.SLIP_IMMEDIATE_SCORE,
        )

        # particle poses and weights
        self.inferred_pose = None
        self.particle_indices = np.arange(self.MAX_PARTICLES)
        self.particles = np.zeros((self.MAX_PARTICLES, 3))
        self.weights = np.ones(self.MAX_PARTICLES) / float(self.MAX_PARTICLES)

        # initialize the state
        self.smoothing = Utils.CircularArray(10)
        self.timer = Utils.Timer(10)
        # map service client
        self.map_client = self.create_client(GetMap, '/map_server/map')
        self.get_omap()
        self.precompute_sensor_model()
        self.initialize_global()

        # keep track of speed from input odom
        self.current_speed = 0.0

        # Pub Subs
        # these topics are for visualization
        self.pose_pub = self.create_publisher(PoseStamped, '/pf/viz/inferred_pose', 1)
        self.particle_pub = self.create_publisher(PoseArray, '/pf/viz/particles', 1)
        self.pub_fake_scan = self.create_publisher(LaserScan, '/pf/viz/fake_scan', 1)
        self.rect_pub = self.create_publisher(PolygonStamped, '/pf/viz/poly1', 1)

        status_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.localization_ready_pub = self.create_publisher(
            Bool, self.LOCALIZATION_READY_TOPIC, status_qos)
        self.localization_quality_pub = self.create_publisher(
            String, self.LOCALIZATION_QUALITY_TOPIC, status_qos)
        self.initialpose_received_pub = self.create_publisher(
            Bool, self.INITIALPOSE_RECEIVED_TOPIC, status_qos)
        self.initialpose_accepted_pub = self.create_publisher(
            Bool, self.INITIALPOSE_ACCEPTED_TOPIC, status_qos)
        self.publish_localization_status()

        if self.PUBLISH_ODOM:
            self.odom_pub = self.create_publisher(Odometry, '/pf/pose/odom', 1)

        # these topics are for coordinate space things
        self.pub_tf = TransformBroadcaster(self)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.laser_to_base_pose = None
        self.base_to_laser_translation = None
        self.base_to_laser_quaternion = None
        self.awaiting_laser_tf = True

        # these topics are to receive data from the racecar
        self.laser_sub = self.create_subscription(
            LaserScan,
            self.get_parameter('scan_topic').value,
            self.lidarCB,
            1)
        self.odom_sub = self.create_subscription(
            Odometry,
            self.get_parameter('odometry_topic').value,
            self.odomCB,
            10)
        self.imu_sub = self.create_subscription(
            Imu,
            self.get_parameter('imu_topic').value,
            self.imuCB,
            50)
        self.pose_sub = self.create_subscription(
            PoseWithCovarianceStamped,
            '/initialpose',
            self.clicked_pose,
            1)
        # The monitor RViz publisher is transient-local so a click made while
        # the PF is still loading its map is delivered when this subscription
        # appears. Keep the volatile subscription above for bag replay and
        # command-line publishers that use the ROS default QoS.
        self.pose_latched_sub = self.create_subscription(
            PoseWithCovarianceStamped,
            '/initialpose',
            self.clicked_pose,
            status_qos)
        self.click_sub = self.create_subscription(
            PointStamped,
            '/clicked_point',
            self.clicked_pose,
            1)
        self.initialpose_retry_timer = self.create_timer(
            0.1, self.retry_pending_initialpose)

        self.filter_worker = Thread(
            target=self._filter_worker_loop,
            name='pf_scan_worker',
            daemon=True,
        )
        self.filter_worker.start()

        self.get_logger().info(
            'Finished initializing PF profile %s: RangeLib variant=%d, '
            'ground-filter=%s, deskew=%s, scan-match/slip=%s; seed=%s; '
            'odom/IMU buffer=%d/%d, '
            'latest-scan worker<=%.1fHz; autonomy waits for /initialpose '
            'and %d consecutive quality-OK updates' % (
                self.PF_PROFILE_LABEL,
                self.RANGELIB_VAR,
                'ON' if self.GROUND_FILTER_ENABLED else 'OFF',
                'ON' if self.DESKEW_ENABLED else 'OFF',
                'ON' if self.SCAN_MATCH_ENABLED else 'OFF',
                str(self.RANDOM_SEED)
                if self.RANDOM_SEED >= 0 else 'random',
                self.ODOM_BUFFER_SIZE,
                self.IMU_BUFFER_SIZE,
                self.MAX_UPDATE_RATE_HZ,
                self.QUALITY_GOOD_UPDATES_TO_RECOVER,
            )
        )

    def get_omap(self):
        '''
        Fetch the occupancy grid map from the map_server instance, and initialize the correct
        RangeLibc method. Also stores a matrix which indicates the permissible region of the map
        '''

        while not self.map_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Get map service not available, waiting...')
        req = GetMap.Request()
        future = self.map_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        map_msg = future.result().map
        self.map_info = map_msg.info

        oMap = range_libc.PyOMap(map_msg)
        self.MAX_RANGE_PX = int(self.MAX_RANGE_METERS / self.map_info.resolution)

        # initialize range method
        self.get_logger().info('Initializing range method: ' + self.WHICH_RM)
        if self.WHICH_RM == 'bl':
            self.range_method = range_libc.PyBresenhamsLine(oMap, self.MAX_RANGE_PX)
        elif 'cddt' in self.WHICH_RM:
            self.range_method = range_libc.PyCDDTCast(oMap, self.MAX_RANGE_PX, self.THETA_DISCRETIZATION)
            if self.WHICH_RM == 'pcddt':
                self.get_logger().info('Pruning...')
                self.range_method.prune()
        elif self.WHICH_RM == 'rm':
            self.range_method = range_libc.PyRayMarching(oMap, self.MAX_RANGE_PX)
        elif self.WHICH_RM == 'rmgpu':
            self.range_method = range_libc.PyRayMarchingGPU(oMap, self.MAX_RANGE_PX)
        elif self.WHICH_RM == 'glt':
            self.range_method = range_libc.PyGiantLUTCast(oMap, self.MAX_RANGE_PX, self.THETA_DISCRETIZATION)
        self.get_logger().info('Done loading map')

         # 0: permissible, -1: unmapped, 100: blocked
        array_255 = np.array(map_msg.data).reshape((map_msg.info.height, map_msg.info.width))

        # 0: not permissible, 1: permissible
        self.permissible_region = np.zeros_like(array_255, dtype=bool)
        self.permissible_region[array_255==0] = 1
        self.map_initialized = True

    def set_localization_transform(self, pose, odom_sample, covariance):
        """Store map->odom from a PF pose and scan-time interpolated odom."""
        map_to_base_at_scan = self.compose_pose_2d(pose, self.laser_to_base_pose)
        odom_pose_at_scan = self.odom_sample_to_pose(odom_sample)
        map_to_odom = self.compose_pose_2d(
            map_to_base_at_scan,
            self.inverse_pose_2d(odom_pose_at_scan),
        )
        with self.transform_lock:
            self.map_to_odom = map_to_odom
            self.map_to_odom_updated_monotonic = time.monotonic()
            self.localized_covariance = covariance

    def publish_propagated_pose(self, odom_sample):
        """Publish the latest base pose using the most recent PF correction."""
        with self.transform_lock:
            if self.map_to_odom is None:
                return
            map_to_odom = np.copy(self.map_to_odom)
            correction_updated = self.map_to_odom_updated_monotonic
            covariance = np.copy(self.localized_covariance)

        correction_age = time.monotonic() - correction_updated
        if correction_age > self.MAX_LOCALIZATION_UPDATE_AGE_SEC:
            if not self.localization_output_stale_active:
                self.get_logger().error(
                    'PF correction is %.2fs old; stopping localization output' %
                    correction_age
                )
            self.localization_output_stale_active = True
            return
        self.localization_output_stale_active = False

        odom_pose = self.odom_sample_to_pose(odom_sample)
        map_to_base = self.compose_pose_2d(map_to_odom, odom_pose)
        stamp = self.nanoseconds_to_stamp(odom_sample.stamp_ns)

        transform = TransformStamped()
        transform.header.stamp = stamp
        transform.header.frame_id = self.MAP_FRAME
        transform.child_frame_id = self.ODOM_FRAME
        transform.transform.translation.x = map_to_odom[0]
        transform.transform.translation.y = map_to_odom[1]
        transform.transform.translation.z = 0.0
        quaternion = tf_transformations.quaternion_from_euler(
            0.0, 0.0, map_to_odom[2])
        transform.transform.rotation.x = quaternion[0]
        transform.transform.rotation.y = quaternion[1]
        transform.transform.rotation.z = quaternion[2]
        transform.transform.rotation.w = quaternion[3]
        self.pub_tf.sendTransform(transform)

        if not self.PUBLISH_ODOM:
            return

        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = self.MAP_FRAME
        odom.child_frame_id = self.BASE_FRAME
        odom.pose.pose.position.x = map_to_base[0]
        odom.pose.pose.position.y = map_to_base[1]
        odom.pose.pose.orientation = Utils.angle_to_quaternion(map_to_base[2])
        odom.pose.covariance = covariance.flatten().tolist()
        odom.twist.twist.linear.x = odom_sample.linear_speed
        odom.twist.twist.angular.z = odom_sample.angular_speed
        self.odom_pub.publish(odom)

    def visualize(self):
        '''
        Publish various visualization messages.
        '''
        if not self.DO_VIZ:
            return

        if self.laser_to_base_pose is None and not self.lookup_laser_to_base_pose():
            return

        if self.pose_pub.get_subscription_count() > 0 and isinstance(self.inferred_pose, np.ndarray):
            inferred_base_pose = self.compose_pose_2d(self.inferred_pose, self.laser_to_base_pose)
            # Publish the inferred pose for visualization
            ps = PoseStamped()
            ps.header.stamp = self.last_stamp
            ps.header.frame_id = self.MAP_FRAME
            ps.pose.position.x = inferred_base_pose[0]
            ps.pose.position.y = inferred_base_pose[1]
            ps.pose.orientation = Utils.angle_to_quaternion(inferred_base_pose[2])
            self.pose_pub.publish(ps)

        if self.particle_pub.get_subscription_count() > 0:
            # publish a downsampled version of the particle distribution to avoid a lot of latency
            if self.MAX_PARTICLES > self.MAX_VIZ_PARTICLES:
                # randomly downsample particles
                proposal_indices = self.viz_rng.choice(
                    self.particle_indices,
                    self.MAX_VIZ_PARTICLES,
                    p=self.weights,
                )
                self.publish_particles(self.particles[proposal_indices,:])
            else:
                self.publish_particles(self.particles)

        if self.pub_fake_scan.get_subscription_count() > 0 and isinstance(self.ranges, np.ndarray):
            # generate the scan from the point of view of the inferred position for visualization
            self.viz_queries[:,0] = self.inferred_pose[0]
            self.viz_queries[:,1] = self.inferred_pose[1]
            self.viz_queries[:,2] = (
                self.current_scan_angles + self.inferred_pose[2])
            self.range_method.calc_range_many(self.viz_queries, self.viz_ranges)
            self.publish_scan(self.current_scan_angles, self.viz_ranges)

    def publish_particles(self, particles):
        # publish the given particles as a PoseArray object
        pa = PoseArray()
        pa.header.stamp = self.last_stamp
        pa.header.frame_id = self.MAP_FRAME
        pa.poses = Utils.particles_to_poses(particles)
        self.particle_pub.publish(pa)

    def publish_scan(self, angles, ranges):
        # publish the given angels and ranges as a laser scan message
        ls = LaserScan()
        ls.header.stamp = self.last_stamp
        ls.header.frame_id = self.LASER_FRAME
        ls.angle_min = float(np.min(angles))
        ls.angle_max = float(np.max(angles))
        ls.angle_increment = float(np.abs(angles[0] - angles[1]))
        ls.range_min = 0.0
        ls.range_max = float(np.max(ranges))
        ls.ranges = [float(r) for r in ranges]
        self.pub_fake_scan.publish(ls)

    def lidarCB(self, msg):
        '''
        Preprocess a scan and replace any older scan waiting for PF work.
        '''
        if not isinstance(self.laser_angles, np.ndarray):
            self.get_logger().info('...Received first LiDAR message')
            self.expected_scan_size = len(msg.ranges)
            self.all_beam_indices = np.arange(
                self.expected_scan_size, dtype=np.int64)
            self.selected_beam_indices = np.arange(
                0, self.expected_scan_size, self.ANGLE_STEP,
                dtype=np.int64,
            )
            if np.isfinite(msg.angle_increment) and msg.angle_increment != 0.0:
                self.laser_angles = (
                    msg.angle_min
                    + self.all_beam_indices * msg.angle_increment
                )
            else:
                self.laser_angles = np.linspace(
                    msg.angle_min, msg.angle_max, self.expected_scan_size)
            self.downsampled_angles = np.copy(
                self.laser_angles[self.selected_beam_indices]
            ).astype(np.float32)
            self.current_scan_angles = np.copy(self.downsampled_angles)
            self.viz_queries = np.zeros(
                (self.downsampled_angles.shape[0], 3), dtype=np.float32)
            self.viz_ranges = np.zeros(
                self.downsampled_angles.shape[0], dtype=np.float32)
            num_rays = self.downsampled_angles.shape[0]
            self.get_logger().info(
                'PF raycast budget: %d particles x %d beams = %d/update' % (
                    self.MAX_PARTICLES,
                    num_rays,
                    self.MAX_PARTICLES * num_rays,
                )
            )
        elif len(msg.ranges) != self.expected_scan_size:
            self.scan_shape_drop_count += 1
            if (self.scan_shape_drop_count <= 3
                    or self.scan_shape_drop_count % 20 == 0):
                self.get_logger().warn(
                    'Ignoring LiDAR scan with changed beam count %d != %d '
                    '(%d total)' % (
                        len(msg.ranges),
                        self.expected_scan_size,
                        self.scan_shape_drop_count,
                    )
                )
            return

        # Mark UST no-return (65.533 m) and every out-of-contract sample before
        # clipping. MCL still receives max range, while 3D geometry gets a
        # separate validity mask and cannot mistake a sentinel for a floor hit.
        raw_scan_ranges = np.asarray(msg.ranges, dtype=np.float32)
        sensor_max = (float(msg.range_max)
                      if np.isfinite(msg.range_max) else self.MAX_RANGE_METERS)
        sensor_min = (max(0.0, float(msg.range_min))
                      if np.isfinite(msg.range_min) else 0.0)
        geometry_valid_returns = (
            np.isfinite(raw_scan_ranges)
            & (raw_scan_ranges >= sensor_min)
            & (raw_scan_ranges > 0.0)
            & (raw_scan_ranges <= sensor_max)
            & (raw_scan_ranges < 65.5)
            & (raw_scan_ranges < self.MAX_RANGE_METERS * 0.995)
        )
        geometry_ranges = np.where(
            geometry_valid_returns, raw_scan_ranges, np.nan).astype(np.float32)
        raw_ranges = raw_scan_ranges[self.selected_beam_indices]
        valid_returns = geometry_valid_returns[self.selected_beam_indices]
        downsampled_ranges = np.where(
            valid_returns, raw_ranges, self.MAX_RANGE_METERS).astype(
                np.float32)
        np.clip(
            downsampled_ranges, 0.0, self.MAX_RANGE_METERS,
            out=downsampled_ranges,
        )
        header_stamp_ns = self.stamp_to_nanoseconds(msg.header.stamp)
        if header_stamp_ns <= 0:
            header_stamp_ns = self.get_clock().now().nanoseconds

        time_increment = float(msg.time_increment)
        scan_duration_sec = (
            (self.expected_scan_size - 1) * time_increment
            if np.isfinite(time_increment) and time_increment > 0.0
            else 0.0
        )
        deskew_timing_valid = bool(
            self.DESKEW_ENABLED
            and 0.0 < scan_duration_sec
            <= self.DESKEW_MAX_SCAN_DURATION_SEC
        )
        if deskew_timing_valid:
            reference_offset_sec = 0.5 * scan_duration_sec
            beam_offsets_sec = (
                self.selected_beam_indices * time_increment
                - reference_offset_sec
            )
            beam_offsets_ns = np.rint(
                beam_offsets_sec * 1e9).astype(np.int64)
            geometry_beam_offsets_ns = np.rint((
                self.all_beam_indices * time_increment - reference_offset_sec
            ) * 1e9).astype(np.int64)
            stamp_ns = header_stamp_ns + int(round(
                reference_offset_sec * 1e9))
        else:
            beam_offsets_ns = np.zeros(
                self.selected_beam_indices.size, dtype=np.int64)
            geometry_beam_offsets_ns = np.zeros(
                self.expected_scan_size, dtype=np.int64)
            stamp_ns = header_stamp_ns
            if self.DESKEW_ENABLED:
                self.deskew_timing_invalid_count += 1
        if stamp_ns <= self.last_received_scan_stamp_ns:
            backward_jump = self.last_received_scan_stamp_ns - stamp_ns
            if backward_jump < 1_000_000_000:
                self.scan_out_of_order_count += 1
                return
        self.last_received_scan_stamp_ns = stamp_ns

        self.lidar_initialized = True
        self.pending_scans.offer(ScanSample(
            stamp_ns=stamp_ns,
            received_monotonic=time.monotonic(),
            ranges=downsampled_ranges,
            valid_returns=valid_returns,
            geometry_ranges=geometry_ranges,
            beam_offsets_ns=beam_offsets_ns,
            geometry_beam_offsets_ns=geometry_beam_offsets_ns,
            deskew_timing_valid=deskew_timing_valid,
            scan_duration_sec=scan_duration_sec,
        ))

    def odomCB(self, msg):
        '''
        Store one lightweight odom sample and publish a propagated PF pose.
        '''
        stamp_ns = self.stamp_to_nanoseconds(msg.header.stamp)
        if stamp_ns <= 0:
            stamp_ns = self.get_clock().now().nanoseconds

        sample = OdomSample(
            stamp_ns=stamp_ns,
            x=msg.pose.pose.position.x,
            y=msg.pose.pose.position.y,
            yaw=Utils.quaternion_to_angle(msg.pose.pose.orientation),
            linear_speed=msg.twist.twist.linear.x,
            angular_speed=msg.twist.twist.angular.z,
        )
        reset_count = self.odom_buffer.time_reset_count
        if not self.odom_buffer.append(sample):
            self.odom_reject_log_count += 1
            if self.odom_reject_log_count <= 3 or self.odom_reject_log_count % 50 == 0:
                self.get_logger().warn(
                    'Ignoring out-of-order odom timestamp (%d total)' %
                    self.odom_reject_log_count
                )
            return

        if self.odom_buffer.time_reset_count != reset_count:
            self.get_logger().warn('Odom time moved backward; resetting PF time reference')
            with self.state_lock:
                self.reset_filter_reference(clear_transform=True)
            self.last_published_odom_stamp_ns = 0

        if not self.odom_initialized:
            self.get_logger().info('...Received first Odometry message')
        self.odom_initialized = True
        if sample.stamp_ns > self.last_published_odom_stamp_ns:
            self.publish_propagated_pose(sample)
            self.last_published_odom_stamp_ns = sample.stamp_ns

    def imuCB(self, msg):
        """Store yaw-rate integration data and a full base attitude sample."""
        stamp_ns = self.stamp_to_nanoseconds(msg.header.stamp)
        if stamp_ns <= 0:
            stamp_ns = self.get_clock().now().nanoseconds

        values = np.array([
            msg.angular_velocity.x,
            msg.angular_velocity.y,
            msg.angular_velocity.z,
            msg.linear_acceleration.x,
            msg.linear_acceleration.y,
            msg.linear_acceleration.z,
        ], dtype=np.float64)
        orientation = np.array([
            msg.orientation.x, msg.orientation.y,
            msg.orientation.z, msg.orientation.w,
        ], dtype=np.float64)
        orientation_norm = float(np.linalg.norm(orientation))
        if (not np.all(np.isfinite(values))
                or not np.isfinite(orientation_norm)
                or orientation_norm < 1e-9):
            self.imu_reject_count += 1
            return
        sample = ImuSample(
            stamp_ns=stamp_ns,
            yaw_rate=(
                values[2] * self.IMU_ANGULAR_VELOCITY_SCALE),
            acceleration_x=(
                values[3] * self.IMU_LINEAR_ACCELERATION_SCALE),
            acceleration_y=(
                values[4] * self.IMU_LINEAR_ACCELERATION_SCALE),
        )
        reset_count = self.imu_buffer.time_reset_count
        if not self.imu_buffer.append(sample):
            self.imu_reject_count += 1
            return
        if self.imu_buffer.time_reset_count != reset_count:
            self.get_logger().warn(
                'IMU time moved backward; resetting IMU integration history')
            self.imu_attitude_buffer.clear()
            with self.attitude_lock:
                self.level_reference.reset()

        base_to_imu = self.lookup_base_to_imu_quaternion(
            msg.header.frame_id)
        try:
            base_orientation = base_orientation_from_imu(
                orientation / orientation_norm, base_to_imu)
        except ValueError:
            self.imu_reject_count += 1
            return
        angular_speed = float(
            np.linalg.norm(values[:3])
            * abs(self.IMU_ANGULAR_VELOCITY_SCALE))
        acceleration_norm = float(
            np.linalg.norm(values[3:])
            * self.IMU_LINEAR_ACCELERATION_SCALE)
        attitude_sample = AttitudeSample(
            stamp_ns, tuple(base_orientation), angular_speed,
            acceleration_norm)
        if not self.imu_attitude_buffer.append(attitude_sample):
            self.imu_reject_count += 1
            return
        latest_odom = self.odom_buffer.latest()
        wheel_speed = (
            latest_odom.linear_speed if latest_odom is not None else 0.0)
        with self.attitude_lock:
            was_ready = self.level_reference.ready
            self.level_reference.update(
                base_orientation, angular_speed, acceleration_norm,
                wheel_speed=wheel_speed)
            if self.level_reference.ready and not was_ready:
                self.get_logger().info(
                    'PF IMU level reference ready; ground gate enabled')
        if not self.imu_initialized:
            self.get_logger().info('...Received first IMU message')
        self.imu_initialized = True

    def clicked_pose(self, msg):
        '''
        Receive pose messages from RViz and initialize the particle distribution in response.
        '''
        if isinstance(msg, PointStamped):
            self.initialize_global()
        elif isinstance(msg, PoseWithCovarianceStamped):
            pose = msg.pose.pose
            values = np.array([
                pose.position.x,
                pose.position.y,
                pose.position.z,
                pose.orientation.x,
                pose.orientation.y,
                pose.orientation.z,
                pose.orientation.w,
            ])
            if not np.all(np.isfinite(values)):
                self.get_logger().error(
                    'Ignoring /initialpose containing non-finite values')
                return

            # A transient-local RViz publisher matches both subscriptions.
            # Suppress the duplicate delivery while preserving default-QoS
            # publishers and bag replay compatibility.
            key = (
                int(msg.header.stamp.sec),
                int(msg.header.stamp.nanosec),
                msg.header.frame_id,
                *values.tolist(),
            )
            with self.initialpose_lock:
                if key == self.last_initialpose_key:
                    return
                self.last_initialpose_key = key
                self.pending_initialpose = copy.deepcopy(pose)
            with self.state_lock:
                self.readiness_tracker.receive_initial_pose()
            self.publish_localization_status()
            self.get_logger().info(
                '/initialpose received; applying it when laser/base TF is ready')
            self.retry_pending_initialpose()

    def retry_pending_initialpose(self):
        """Apply the newest cached RViz pose once the fixed LiDAR TF exists."""
        with self.initialpose_lock:
            pose = self.pending_initialpose
        if pose is None or not self.initialize_particles_pose(pose):
            return
        with self.initialpose_lock:
            if self.pending_initialpose is pose:
                self.pending_initialpose = None

    def initialize_particles_pose(self, pose):
        '''
        Initialize particles in the general region of the provided pose.
        '''
        if self.laser_to_base_pose is None and not self.lookup_laser_to_base_pose():
            now = time.monotonic()
            if now - self.last_initialpose_wait_log_time >= 2.0:
                self.get_logger().warn(
                    '/initialpose received but not applied: waiting for '
                    'laser<->base_link TF; cached pose will be retried')
                self.last_initialpose_wait_log_time = now
            return False

        base_pose = np.array([
            pose.position.x,
            pose.position.y,
            Utils.quaternion_to_angle(pose.orientation),
        ])
        laser_pose = self.compose_pose_2d(
            base_pose,
            self.inverse_pose_2d(self.laser_to_base_pose),
        )
        self.get_logger().info('SETTING POSE')
        self.get_logger().info(str([base_pose[0], base_pose[1]]))
        with self.state_lock:
            self.weights = np.ones(self.MAX_PARTICLES) / float(self.MAX_PARTICLES)
            self.particles[:,0] = laser_pose[0] + self.mcl_rng.normal(
                loc=0.0, scale=0.5, size=self.MAX_PARTICLES)
            self.particles[:,1] = laser_pose[1] + self.mcl_rng.normal(
                loc=0.0, scale=0.5, size=self.MAX_PARTICLES)
            self.particles[:,2] = self.wrap_angle(
                laser_pose[2] + self.mcl_rng.normal(
                    loc=0.0, scale=0.4, size=self.MAX_PARTICLES))
            self.inferred_pose = np.copy(laser_pose)
            self.readiness_tracker.accept_initial_pose()
            self.reset_filter_reference(clear_transform=True)
        self.get_logger().info(
            '/initialpose accepted; autonomy remains blocked until quality OK')
        return True

    def initialize_global(self):
        '''
        Spread the particle distribution over the permissible region of the state space.
        '''
        self.get_logger().info('GLOBAL INITIALIZATION')
        with self.initialpose_lock:
            self.pending_initialpose = None
        # randomize over grid coordinate space
        with self.state_lock:
            self.readiness_tracker.clear_initial_pose()
            permissible_x, permissible_y = np.where(self.permissible_region == 1)
            indices = self.mcl_rng.integers(
                0, len(permissible_x), size=self.MAX_PARTICLES)

            permissible_states = np.zeros((self.MAX_PARTICLES,3))
            permissible_states[:,0] = permissible_y[indices]
            permissible_states[:,1] = permissible_x[indices]
            permissible_states[:,2] = self.mcl_rng.random(
                self.MAX_PARTICLES) * np.pi * 2.0

            Utils.map_to_world(permissible_states, self.map_info)
            self.particles = permissible_states
            self.weights[:] = 1.0 / self.MAX_PARTICLES
            self.inferred_pose = None
            self.reset_filter_reference(clear_transform=True)

    def precompute_sensor_model(self):
        '''
        Generate and store a table which represents the sensor model. For each discrete computed
        range value, this provides the probability of measuring any (discrete) range.

        This table is indexed by the sensor model at runtime by discretizing the measurements
        and computed ranges from RangeLibc.
        '''
        self.get_logger().info('Precomputing sensor model')
        # sensor model constants
        z_short = self.Z_SHORT
        z_max   = self.Z_MAX
        z_rand  = self.Z_RAND
        z_hit   = self.Z_HIT
        sigma_hit = self.SIGMA_HIT
        
        table_width = int(self.MAX_RANGE_PX) + 1
        self.sensor_model_table = np.zeros((table_width,table_width))

        # d is the computed range from RangeLibc
        for d in range(table_width):
            norm = 0.0
            # r is the observed range from the lidar unit
            for r in range(table_width):
                prob = 0.0
                z = float(r-d)
                # reflects from the intended object
                prob += z_hit * np.exp(-(z*z)/(2.0*sigma_hit*sigma_hit)) / (sigma_hit * np.sqrt(2.0*np.pi))

                # observed range is less than the predicted range - short reading
                if r < d:
                    prob += 2.0 * z_short * (d - r) / float(d)

                # erroneous max range measurement
                if int(r) == int(self.MAX_RANGE_PX):
                    prob += z_max

                # random measurement
                if r < int(self.MAX_RANGE_PX):
                    prob += z_rand * 1.0/float(self.MAX_RANGE_PX)

                norm += prob
                self.sensor_model_table[int(r),int(d)] = prob

            # normalize
            self.sensor_model_table[:,int(d)] /= norm

        # upload the sensor model to RangeLib for ultra fast resolution
        self.sensor_log_table = np.log(
            np.maximum(self.sensor_model_table, 1e-12)
        ).astype(np.float32)
        if 0 < self.RANGELIB_VAR < VAR_REPEAT_ANGLES_LOG_SENSOR:
            self.range_method.set_sensor_model(self.sensor_model_table)

    def motion_model(self, proposal_dist, action, noise_multiplier=1.0):
        '''
        The motion model applies the odometry to the particle distribution. Since there the odometry
        data is inaccurate, the motion model mixes in gaussian noise to spread out the distribution.

        Vectorized motion model. Computing the motion model over all particles is thousands of times
        faster than doing it for each particle individually due to vectorization and reduction in
        function call overhead
        
        TODO this could be better, but it works for now
            - fixed random noise is not very realistic
            - ackermann model provides bad estimates at high speed
        '''
        # rotate the action into the coordinate space of each particle
        # t1 = time.time()
        cosines = np.cos(proposal_dist[:,2])
        sines = np.sin(proposal_dist[:,2])

        self.local_deltas[:,0] = cosines*action[0] - sines*action[1]
        self.local_deltas[:,1] = sines*action[0] + cosines*action[1]
        self.local_deltas[:,2] = action[2]

        proposal_dist[:,:] += self.local_deltas
        translation = np.hypot(action[0], action[1])
        normalized_motion = (
            translation / self.MOTION_NOISE_REFERENCE_TRANSLATION +
            abs(action[2]) / self.MOTION_NOISE_REFERENCE_YAW
        )
        noise_scale = np.clip(
            np.sqrt(normalized_motion),
            self.MOTION_NOISE_MIN_SCALE,
            2.0,
        )
        noise_scale *= max(1.0, float(noise_multiplier))
        proposal_dist[:,0] += self.mcl_rng.normal(
            loc=0.0,
            scale=self.MOTION_DISPERSION_X * noise_scale,
            size=self.MAX_PARTICLES,
        )
        proposal_dist[:,1] += self.mcl_rng.normal(
            loc=0.0,
            scale=self.MOTION_DISPERSION_Y * noise_scale,
            size=self.MAX_PARTICLES,
        )
        proposal_dist[:,2] += self.mcl_rng.normal(
            loc=0.0,
            scale=self.MOTION_DISPERSION_THETA * noise_scale,
            size=self.MAX_PARTICLES,
        )
        proposal_dist[:,2] = self.wrap_angle(proposal_dist[:,2])

    def _allocate_sensor_buffers(self, num_rays):
        """Allocate fixed-size sensor buffers once after the first scan."""
        expanded = self.RANGELIB_VAR in (
            VAR_NO_EVAL_SENSOR_MODEL,
            VAR_CALC_RANGE_MANY_EVAL_SENSOR,
        )
        query_count = (
            num_rays * self.MAX_PARTICLES if expanded
            else self.MAX_PARTICLES
        )
        self.queries = np.zeros((query_count, 3), dtype=np.float32)
        if expanded:
            self.query_grid = self.queries.reshape(
                self.MAX_PARTICLES, num_rays, 3)
        self.ranges = np.zeros(
            num_rays * self.MAX_PARTICLES, dtype=np.float32)
        self.range_pixels = np.zeros(
            (self.MAX_PARTICLES, num_rays), dtype=np.uint16)
        self.first_sensor_update = False

    def _populate_expanded_queries(self, proposal_dist, scan_angles):
        """Build legacy calc_range_many queries without np.repeat temporaries."""
        self.query_grid[:, :, 0] = proposal_dist[:, 0, np.newaxis]
        self.query_grid[:, :, 1] = proposal_dist[:, 1, np.newaxis]
        np.add(
            proposal_dist[:, 2, np.newaxis],
            scan_angles[np.newaxis, :],
            out=self.query_grid[:, :, 2],
        )

    def _evaluate_log_sensor_model(
            self, obs, weights, particle_agreement_weights,
            required_beam_mask=None):
        """Evaluate stable log likelihoods and dynamic-beam agreement."""
        resolution = float(self.map_info.resolution)
        np.multiply(self.ranges, 1.0 / resolution, out=self.ranges)
        np.rint(self.ranges, out=self.ranges)
        np.clip(self.ranges, 0, self.MAX_RANGE_PX, out=self.ranges)
        self.range_pixels.reshape(-1)[:] = self.ranges

        obs_px = np.rint(
            np.clip(obs / resolution, 0, self.MAX_RANGE_PX)
        ).astype(np.uint16)
        distance_px = int(round(self.BEAM_SKIP_DISTANCE / resolution))
        selection = select_beams(
            obs_px,
            self.range_pixels,
            distance_px,
            self.BEAM_SKIP_MIN_PARTICLE_FRACTION,
            self.BEAM_SKIP_MAX_FRACTION,
            enabled=self.BEAM_SKIP_ENABLED,
            particle_weights=particle_agreement_weights,
            short_only=self.BEAM_SKIP_SHORT_ONLY,
            required_mask=required_beam_mask,
        )
        weights[:] = likelihoods_from_log_table(
            self.sensor_log_table,
            obs_px,
            self.range_pixels,
            selection.mask,
            self.INV_SQUASH_FACTOR,
        )
        self.last_beam_selection = selection

    def sensor_model(
            self, proposal_dist, obs, scan_angles, weights,
            particle_agreement_weights=None, required_beam_mask=None):
        """Evaluate scan likelihoods for every proposed particle."""
        num_rays = scan_angles.shape[0]
        if self.first_sensor_update:
            self._allocate_sensor_buffers(num_rays)
        self.last_beam_selection = None

        if self.RANGELIB_VAR == VAR_RADIAL_CDDT_OPTIMIZATIONS:
            if 'cddt' not in self.WHICH_RM:
                self.get_logger().error(
                    'rangelib_variant 4 requires cddt or pcddt')
                return False
            self.queries[:, :] = proposal_dist[:, :]
            self.range_method.calc_range_many_radial_optimized(
                num_rays,
                scan_angles[0],
                scan_angles[-1],
                self.queries,
                self.ranges,
            )
            self.range_method.eval_sensor_model(
                obs, self.ranges, weights, num_rays, self.MAX_PARTICLES)
            np.power(weights, self.INV_SQUASH_FACTOR, out=weights)
        elif self.RANGELIB_VAR == \
                VAR_REPEAT_ANGLES_EVAL_SENSOR_ONE_SHOT:
            self.queries[:, :] = proposal_dist[:, :]
            self.range_method.calc_range_repeat_angles_eval_sensor_model(
                self.queries, scan_angles, obs, weights)
            np.power(weights, self.INV_SQUASH_FACTOR, out=weights)
        elif self.RANGELIB_VAR == VAR_REPEAT_ANGLES_EVAL_SENSOR:
            if self.SHOW_FINE_TIMING:
                t_start = time.time()
            self.queries[:, :] = proposal_dist[:, :]
            if self.SHOW_FINE_TIMING:
                t_init = time.time()
            self.range_method.calc_range_repeat_angles(
                self.queries, scan_angles, self.ranges)
            if self.SHOW_FINE_TIMING:
                t_range = time.time()
            self.range_method.eval_sensor_model(
                obs, self.ranges, weights, num_rays, self.MAX_PARTICLES)
            if self.SHOW_FINE_TIMING:
                t_eval = time.time()
            np.power(weights, self.INV_SQUASH_FACTOR, out=weights)
            if self.SHOW_FINE_TIMING:
                t_squash = time.time()
                t_total = (t_squash - t_start) / 100.0
                if self.iters % 10 == 0:
                    self.get_logger().info(str([
                        'sensor_model: init: ',
                        np.round((t_init - t_start) / t_total, 2),
                        'range:', np.round((t_range - t_init) / t_total, 2),
                        'eval:', np.round((t_eval - t_range) / t_total, 2),
                        'squash:',
                        np.round((t_squash - t_eval) / t_total, 2),
                    ]))
        elif self.RANGELIB_VAR == VAR_CALC_RANGE_MANY_EVAL_SENSOR:
            self._populate_expanded_queries(proposal_dist, scan_angles)
            self.range_method.calc_range_many(self.queries, self.ranges)
            self.range_method.eval_sensor_model(
                obs, self.ranges, weights, num_rays, self.MAX_PARTICLES)
            np.power(weights, self.INV_SQUASH_FACTOR, out=weights)
        elif self.RANGELIB_VAR == VAR_NO_EVAL_SENSOR_MODEL:
            self._populate_expanded_queries(proposal_dist, scan_angles)
            self.range_method.calc_range_many(self.queries, self.ranges)
            self._evaluate_log_sensor_model(
                obs, weights, particle_agreement_weights,
                required_beam_mask)
        elif self.RANGELIB_VAR == VAR_REPEAT_ANGLES_LOG_SENSOR:
            self.queries[:, :] = proposal_dist[:, :]
            self.range_method.calc_range_repeat_angles(
                self.queries, scan_angles, self.ranges)
            self._evaluate_log_sensor_model(
                obs, weights, particle_agreement_weights,
                required_beam_mask)
        else:
            self.get_logger().error(
                'rangelib_variant must be an integer from 0 through 5')
            return False
        return True

    def MCL(self, a, o, scan_angles, motion_noise_multiplier=1.0,
            required_beam_mask=None):
        '''
        Performs one step of Monte Carlo Localization.
            1. resample particle distribution to form the proposal distribution
            2. apply the motion model
            3. apply the sensor model
            4. normalize particle weights

        This is in the critical path of code execution, so it is optimized for speed.
        '''
        if self.SHOW_FINE_TIMING:
            t = time.time()
        try:
            proposal = prepare_mcl_proposal(
                self.particles,
                self.weights,
                self.RESAMPLE_ESS_RATIO,
                rng=self.mcl_rng,
            )
        except ValueError:
            return False
        proposal_distribution = proposal.particles
        if self.SHOW_FINE_TIMING:
            t_propose = time.time()

        # compute the motion model to update the proposal distribution
        self.motion_model(
            proposal_distribution, a, motion_noise_multiplier)
        if self.SHOW_FINE_TIMING:
            t_motion = time.time()

        # compute the sensor model
        sensor_ok = self.sensor_model(
            proposal_distribution,
            o,
            scan_angles,
            self.sensor_likelihoods,
            proposal.prior_weights,
            required_beam_mask,
        )
        if self.SHOW_FINE_TIMING:
            t_sensor = time.time()
        if not sensor_ok:
            return False

        posterior = posterior_from_likelihood(
            proposal.prior_weights,
            self.sensor_likelihoods,
        )
        if posterior is None:
            return False
        self.particles = proposal_distribution
        self.weights = posterior
        self.last_prior_ess_ratio = proposal.prior_ess_ratio
        self.last_posterior_ess_ratio = (
            effective_sample_size(self.weights) / self.MAX_PARTICLES)
        self.last_resampled = proposal.resampled
        if proposal.resampled:
            self.resample_count += 1

        if self.last_beam_selection is not None:
            if self.last_beam_selection.fallback_to_all_beams:
                self.beam_skip_fallback_count += 1

        if self.SHOW_FINE_TIMING:
            t_norm = time.time()
            t_total = (t_norm - t)/100.0

        if self.SHOW_FINE_TIMING and self.iters % 10 == 0:
            self.get_logger().info(str(['MCL: propose: ', np.round((t_propose-t)/t_total, 2), 'motion:', np.round((t_motion-t_propose)/t_total, 2), \
                  'sensor:', np.round((t_sensor-t_motion)/t_total, 2), 'norm:', np.round((t_norm-t_sensor)/t_total, 2)]))

        return True
    
    def expected_pose(self):
        # x, y use weighted arithmetic means; yaw uses a weighted circular mean.
        xy = np.dot(self.particles[:, :2].transpose(), self.weights)
        sin_yaw = np.dot(np.sin(self.particles[:, 2]), self.weights)
        cos_yaw = np.dot(np.cos(self.particles[:, 2]), self.weights)
        yaw = np.arctan2(sin_yaw, cos_yaw)
        return np.array([xy[0], xy[1], yaw])

    def pose_covariance_3x3(self):
        """Weighted covariance in [x, y, yaw], with yaw residuals wrapped.

        A linear covariance on raw yaw values explodes near the +-pi branch cut:
        headings at +3.13 and -3.13 rad are almost identical, but np.cov treats
        them as far apart. Use the circular mean yaw and wrapped residuals so
        published yaw covariance reflects actual spread.
        """
        mean_pose = self.expected_pose()
        centered = np.empty_like(self.particles)
        centered[:, :2] = self.particles[:, :2] - mean_pose[:2]
        centered[:, 2] = self.wrap_angle(self.particles[:, 2] - mean_pose[2])

        weight_sum = np.sum(self.weights)
        if not np.isfinite(weight_sum) or weight_sum <= 1e-12:
            return np.zeros((3, 3))

        normalized_weights = self.weights / weight_sum
        weighted_centered = centered * normalized_weights[:, np.newaxis]
        return centered.T @ weighted_centered

    def restore_motion_only_pose(self, particles, weights, action):
        self.particles = np.copy(particles)
        self.weights = np.copy(weights)
        weight_sum = np.sum(self.weights)
        if not np.isfinite(weight_sum) or weight_sum <= 1e-12:
            self.weights[:] = 1.0 / float(self.MAX_PARTICLES)
        else:
            self.weights /= weight_sum
        self.last_posterior_ess_ratio = (
            effective_sample_size(self.weights) / self.MAX_PARTICLES)
        self.last_resampled = False
        cosines = np.cos(self.particles[:,2])
        sines = np.sin(self.particles[:,2])
        self.local_deltas[:,0] = cosines*action[0] - sines*action[1]
        self.local_deltas[:,1] = sines*action[0] + cosines*action[1]
        self.local_deltas[:,2] = action[2]
        self.particles[:,:] += self.local_deltas
        self.particles[:,2] = self.wrap_angle(self.particles[:,2])
        self.inferred_pose = self.expected_pose()

    def update_localization_quality(self, inlier_fraction):
        """Update LOST hysteresis after accepting or rejecting a correction."""
        quality = self.quality_tracker.update(inlier_fraction)
        readiness_transitioned = self.readiness_tracker.update(quality.state)
        if quality.state == QualityState.LOST:
            self.quality_lost_update_count += 1
        if quality.transitioned:
            if quality.state == QualityState.LOST:
                self.quality_lost_transition_count += 1
                self.get_logger().error(
                    'PF localization quality entered LOST: scan-fit %.2f' %
                    quality.inlier_fraction)
            elif quality.state == QualityState.OK:
                self.get_logger().info(
                    'PF localization quality recovered: scan-fit %.2f' %
                    quality.inlier_fraction)
        if readiness_transitioned:
            if self.readiness_tracker.ready:
                self.get_logger().info(
                    'PF localization READY: /initialpose accepted and '
                    'quality OK; autonomy may start')
            else:
                self.get_logger().error(
                    'PF localization readiness revoked: quality LOST; '
                    'autonomy must stop')
        return quality

    def publish_localization_status(self):
        """Publish latched state plus a per-scan heartbeat for safety gates."""
        if not hasattr(self, 'localization_ready_pub'):
            return
        ready = Bool()
        ready.data = self.readiness_tracker.ready
        quality = String()
        quality.data = self.quality_tracker.state.value
        initialpose_received = Bool()
        initialpose_received.data = \
            self.readiness_tracker.initial_pose_received
        initialpose_accepted = Bool()
        initialpose_accepted.data = \
            self.readiness_tracker.initial_pose_accepted
        self.localization_ready_pub.publish(ready)
        self.localization_quality_pub.publish(quality)
        self.initialpose_received_pub.publish(initialpose_received)
        self.initialpose_accepted_pub.publish(initialpose_accepted)

    def should_reject_pose_update(self, predicted_pose, candidate_pose):
        correction = self.inverse_pose_2d(predicted_pose)
        correction = self.compose_pose_2d(correction, candidate_pose)
        correction_xy = np.hypot(correction[0], correction[1])
        correction_yaw = abs(self.wrap_angle(correction[2]))
        max_correction_xy = self.MAX_POSE_JUMP_TRANSLATION
        max_correction_yaw = self.MAX_POSE_JUMP_YAW
        if abs(self.current_speed) < self.LOW_SPEED_THRESHOLD:
            max_correction_xy = self.LOW_SPEED_JUMP_GUARD_TRANSLATION
            max_correction_yaw = self.LOW_SPEED_JUMP_GUARD_YAW
        reject = (
            correction_xy > max_correction_xy or
            correction_yaw > max_correction_yaw
        )
        return reject, correction_xy, correction_yaw

    def lookup_laser_to_base_pose(self):
        '''Cache legacy 2D and full 6DoF LiDAR extrinsics together.'''
        try:
            transform = self.tf_buffer.lookup_transform(
                self.LASER_FRAME,
                self.BASE_FRAME,
                rclpy.time.Time(),
                timeout=Duration(seconds=0.0),
            )
            base_from_laser = self.tf_buffer.lookup_transform(
                self.BASE_FRAME,
                self.LASER_FRAME,
                rclpy.time.Time(),
                timeout=Duration(seconds=0.0),
            )
            yaw = Utils.quaternion_to_angle(transform.transform.rotation)
            self.laser_to_base_pose = np.array([
                transform.transform.translation.x,
                transform.transform.translation.y,
                yaw,
            ])
            translation = base_from_laser.transform.translation
            rotation = base_from_laser.transform.rotation
            quaternion = np.array([
                rotation.x, rotation.y, rotation.z, rotation.w,
            ], dtype=np.float64)
            norm = float(np.linalg.norm(quaternion))
            if (not np.all(np.isfinite(quaternion)) or norm < 1e-9
                    or not np.all(np.isfinite([
                        translation.x, translation.y, translation.z]))):
                raise ValueError('invalid base<-laser transform')
            self.base_to_laser_translation = np.array([
                translation.x, translation.y, translation.z,
            ], dtype=np.float64)
            self.base_to_laser_quaternion = quaternion / norm
            if self.awaiting_laser_tf:
                self.get_logger().info(
                    f'Received 6DoF {self.BASE_FRAME} <- '
                    f'{self.LASER_FRAME} transform'
                )
                self.awaiting_laser_tf = False
            return True
        except Exception:
            if self.awaiting_laser_tf:
                self.get_logger().info(
                    f'Waiting for {self.LASER_FRAME} <- {self.BASE_FRAME} transform...'
                )
            return False

    def lookup_base_to_imu_quaternion(self, frame_id):
        """Return measured base<-IMU rotation, or the configured fallback."""
        frame = frame_id.strip().lstrip('/')
        if not frame:
            return np.copy(self.BASE_TO_IMU_FALLBACK)
        with self.attitude_lock:
            if self.imu_extrinsic_frame == frame:
                return np.copy(self.base_to_imu_quaternion)
        try:
            transform = self.tf_buffer.lookup_transform(
                self.BASE_FRAME, frame, rclpy.time.Time(),
                timeout=Duration(seconds=0.0))
            rotation = transform.transform.rotation
            quaternion = np.array([
                rotation.x, rotation.y, rotation.z, rotation.w,
            ], dtype=np.float64)
            norm = float(np.linalg.norm(quaternion))
            if not np.isfinite(norm) or norm < 1e-9:
                raise ValueError('invalid IMU quaternion')
            quaternion /= norm
        except Exception:
            return np.copy(self.BASE_TO_IMU_FALLBACK)
        with self.attitude_lock:
            changed = (self.imu_extrinsic_frame != frame
                       or not np.allclose(
                           quaternion, self.base_to_imu_quaternion,
                           atol=1e-8))
            self.base_to_imu_quaternion = quaternion
            self.imu_extrinsic_frame = frame
            if changed:
                self.imu_attitude_buffer.clear()
                self.level_reference.reset()
                self.get_logger().info(
                    f'Using measured IMU extrinsic '
                    f'{self.BASE_FRAME} <- {frame}; level reset')
        return np.copy(quaternion)

    @staticmethod
    def compose_pose_2d(lhs, rhs):
        '''Compose two planar poses represented as [x, y, yaw].'''
        c = np.cos(lhs[2])
        s = np.sin(lhs[2])
        x = lhs[0] + c * rhs[0] - s * rhs[1]
        y = lhs[1] + s * rhs[0] + c * rhs[1]
        yaw = lhs[2] + rhs[2]
        return np.array([x, y, np.arctan2(np.sin(yaw), np.cos(yaw))])

    @staticmethod
    def inverse_pose_2d(pose):
        '''Invert a planar pose represented as [x, y, yaw].'''
        c = np.cos(pose[2])
        s = np.sin(pose[2])
        x = -c * pose[0] - s * pose[1]
        y = s * pose[0] - c * pose[1]
        yaw = -pose[2]
        return np.array([x, y, np.arctan2(np.sin(yaw), np.cos(yaw))])

    @staticmethod
    def wrap_angle(theta):
        return np.arctan2(np.sin(theta), np.cos(theta))

    def _validate_sync_parameters(self):
        """Reject unsafe synchronization settings before subscriptions start."""
        if self.ANGLE_STEP <= 0:
            raise ValueError('angle_step must be positive')
        if self.MAX_PARTICLES <= 0:
            raise ValueError('max_particles must be positive')
        if self.RANGELIB_VAR not in range(6):
            raise ValueError('rangelib_variant must be from 0 through 5')
        if self.BEAM_SKIP_ENABLED and self.RANGELIB_VAR not in (
                VAR_NO_EVAL_SENSOR_MODEL, VAR_REPEAT_ANGLES_LOG_SENSOR):
            raise ValueError(
                'beam_skip_enabled requires rangelib_variant 0 or 5')
        if self.QUALITY_GATE_ENABLED and self.RANGELIB_VAR not in (
                VAR_NO_EVAL_SENSOR_MODEL, VAR_REPEAT_ANGLES_LOG_SENSOR):
            raise ValueError(
                'quality_gate_enabled requires rangelib_variant 0 or 5')
        if self.GROUND_FILTER_ENABLED and self.RANGELIB_VAR not in (
                VAR_NO_EVAL_SENSOR_MODEL, VAR_REPEAT_ANGLES_LOG_SENSOR):
            raise ValueError(
                'ground_filter_enabled requires rangelib_variant 0 or 5')
        if not 100 <= self.ODOM_BUFFER_SIZE <= 150:
            raise ValueError('odom_buffer_size must be between 100 and 150')
        if not 100 <= self.IMU_BUFFER_SIZE <= 150:
            raise ValueError('imu_buffer_size must be between 100 and 150')
        if not 0.0 <= self.ODOM_SYNC_WAIT_SEC <= 0.2:
            raise ValueError('odom_sync_wait_sec must be between 0.0 and 0.2')
        if self.MAX_ODOM_INTERP_GAP_SEC <= 0.0:
            raise ValueError('max_odom_interp_gap_sec must be positive')
        if not 0.0 <= self.IMU_SYNC_WAIT_SEC <= 0.2:
            raise ValueError('imu_sync_wait_sec must be between 0.0 and 0.2')
        if self.MAX_IMU_INTERP_GAP_SEC <= 0.0:
            raise ValueError('max_imu_interp_gap_sec must be positive')
        if (not np.isfinite(self.IMU_ANGULAR_VELOCITY_SCALE)
                or self.IMU_ANGULAR_VELOCITY_SCALE == 0.0):
            raise ValueError('imu_angular_velocity_scale must be finite and non-zero')
        if (not np.isfinite(self.IMU_LINEAR_ACCELERATION_SCALE)
                or self.IMU_LINEAR_ACCELERATION_SCALE <= 0.0):
            raise ValueError('imu_linear_acceleration_scale must be positive')
        ground = self.GROUND_FILTER_CONFIG
        ground_values = (
            ground.ground_z_m, ground.range_tolerance_m,
            ground.range_tolerance_ratio, ground.endpoint_z_tolerance_m,
            ground.min_component_angle_rad, ground.min_component_span_m,
            ground.max_planar_elevation_rad,
            ground.degraded_offplane_fraction,
            ground.degraded_ground_fraction,
            ground.attitude_fallback_tilt_rad,
            ground.attitude_ground_evidence_fraction,
            ground.attitude_evidence_max_abs_angle_rad,
            ground.degraded_valid_fraction,
            self.LEVEL_REFERENCE_MAX_GYRO,
            self.LEVEL_REFERENCE_ACCEL_TOLERANCE,
        )
        if (not all(np.isfinite(value) for value in ground_values)
                or not 2 <= self.LEVEL_REFERENCE_SAMPLES <= 1000
                or not 0.0 < self.LEVEL_REFERENCE_MAX_GYRO <= 5.0
                or not 0.0 < self.LEVEL_REFERENCE_ACCEL_TOLERANCE <= 20.0
                or not -1.0 <= ground.ground_z_m <= 1.0
                or not 0.0 < ground.range_tolerance_m <= 1.0
                or not 0.0 <= ground.range_tolerance_ratio <= 0.5
                or not 0.0 < ground.endpoint_z_tolerance_m <= 0.5
                or not 2 <= ground.min_component_beams <= 100
                or not 0.0 < ground.min_component_angle_rad <= np.pi
                or not 0.05 <= ground.min_component_span_m <= 5.0
                or not np.deg2rad(0.5) <= ground.max_planar_elevation_rad
                <= np.deg2rad(30.0)
                or not 0.0 < ground.degraded_offplane_fraction <= 1.0
                or not 0.0 < ground.degraded_ground_fraction <= 1.0
                or not np.deg2rad(1.0)
                <= ground.attitude_fallback_tilt_rad <= np.deg2rad(45.0)
                or not 0.0
                <= ground.attitude_ground_evidence_fraction <= 1.0
                or not 1 <= ground.attitude_ground_evidence_min_beams <= 1000
                or not np.deg2rad(10.0)
                <= ground.attitude_evidence_max_abs_angle_rad <= np.pi
                or not 0.0 < ground.degraded_valid_fraction <= 1.0
                or not 3 <= ground.min_retained_beams <= 100):
            raise ValueError('invalid PF ground-filter parameters')
        if self.MAX_PENDING_SCAN_AGE_SEC <= 0.0:
            raise ValueError('max_pending_scan_age_sec must be positive')
        if self.MAX_UPDATE_RATE_HZ <= 0.0:
            raise ValueError('max_update_rate_hz must be positive')
        if self.RANDOM_SEED < -1:
            raise ValueError(
                'random_seed must be -1 or a non-negative integer')
        if self.SYNC_DIAGNOSTICS_PERIOD_SEC <= 0.0:
            raise ValueError('sync_diagnostics_period_sec must be positive')
        if self.MAX_LOCALIZATION_UPDATE_AGE_SEC <= 0.0:
            raise ValueError('max_localization_update_age_sec must be positive')
        if not 0.0 < self.MOTION_NOISE_MIN_SCALE <= 1.0:
            raise ValueError('motion_noise_min_scale must be in (0.0, 1.0]')
        if self.MOTION_NOISE_REFERENCE_TRANSLATION <= 0.0:
            raise ValueError('motion_noise_reference_translation_m must be positive')
        if self.MOTION_NOISE_REFERENCE_YAW <= 0.0:
            raise ValueError('motion_noise_reference_yaw_rad must be positive')
        if not 0.0 < self.RESAMPLE_ESS_RATIO <= 1.0:
            raise ValueError('resample_ess_ratio must be in (0.0, 1.0]')
        if self.BEAM_SKIP_DISTANCE < 0.0:
            raise ValueError('beam_skip_distance_m must be non-negative')
        if not 0.0 <= self.BEAM_SKIP_MIN_PARTICLE_FRACTION <= 1.0:
            raise ValueError(
                'beam_skip_min_particle_fraction must be in [0.0, 1.0]')
        if not 0.0 <= self.BEAM_SKIP_MAX_FRACTION < 1.0:
            raise ValueError('beam_skip_max_fraction must be in [0.0, 1.0)')
        if not 0.0 <= self.QUALITY_LOST_INLIER_FRACTION < \
                self.QUALITY_OK_INLIER_FRACTION <= 1.0:
            raise ValueError(
                'quality fractions must satisfy 0 <= lost < ok <= 1')
        if self.QUALITY_BAD_UPDATES_TO_LOST <= 0:
            raise ValueError('quality_bad_updates_to_lost must be positive')
        if self.QUALITY_GOOD_UPDATES_TO_RECOVER <= 0:
            raise ValueError('quality_good_updates_to_recover must be positive')
        if not 0.0 < self.DESKEW_MAX_SCAN_DURATION_SEC <= 0.2:
            raise ValueError(
                'deskew_max_scan_duration_sec must be in (0.0, 0.2]')
        if not 0.0 <= self.DESKEW_SLIP_TRANSLATION_SCALE <= 1.0:
            raise ValueError(
                'deskew_slip_translation_scale must be in [0.0, 1.0]')
        if (self.DESKEW_ENABLED
                and self.RANGELIB_VAR == VAR_RADIAL_CDDT_OPTIMIZATIONS):
            raise ValueError(
                'rangelib_variant 4 assumes uniform angles and cannot deskew')
        if (self.STATIONARY_IMU_YAW_RATE <= 0.0
                or self.STATIONARY_IMU_HORIZONTAL_ACCEL <= 0.0):
            raise ValueError('stationary IMU thresholds must be positive')
        if (self.STATIONARY_SCAN_TRANSLATION < 0.0
                or self.STATIONARY_SCAN_YAW < 0.0):
            raise ValueError('stationary scan thresholds must be non-negative')
        if self.SCAN_MATCH_MAX_CORRESPONDENCE <= 0.0:
            raise ValueError('scan_match_max_correspondence_m must be positive')
        if not 0.0 < self.SCAN_MATCH_TRIM_FRACTION <= 1.0:
            raise ValueError('scan_match_trim_fraction must be in (0.0, 1.0]')
        if self.SCAN_MATCH_ITERATIONS <= 0 or self.SCAN_MATCH_MIN_POINTS < 3:
            raise ValueError('scan matcher iterations/min points are invalid')
        if not 0.0 <= self.SCAN_MATCH_MIN_INLIER_FRACTION <= 1.0:
            raise ValueError(
                'scan_match_min_inlier_fraction must be in [0.0, 1.0]')
        if self.SCAN_MATCH_MAX_RMSE <= 0.0:
            raise ValueError('scan_match_max_rmse_m must be positive')
        if (self.SLIP_TRANSLATION_INNOVATION <= 0.0
                or self.SLIP_YAW_INNOVATION <= 0.0):
            raise ValueError('slip innovation thresholds must be positive')
        if not 0.0 <= self.SLIP_RECOVER_SCORE < self.SLIP_ENTER_SCORE:
            raise ValueError('slip scores must satisfy 0 <= recover < enter')
        if self.SLIP_IMMEDIATE_SCORE < self.SLIP_ENTER_SCORE:
            raise ValueError('slip_immediate_score must be >= slip_enter_score')
        if self.SLIP_ENTER_UPDATES <= 0 or self.SLIP_RECOVER_UPDATES <= 0:
            raise ValueError('slip hysteresis counts must be positive')
        if self.SLIP_MOTION_NOISE_MULTIPLIER < 1.0:
            raise ValueError('slip_motion_noise_multiplier must be >= 1.0')
        if not 0.0 <= self.SLIP_JUMP_BYPASS_MIN_SCAN_FIT <= 1.0:
            raise ValueError(
                'slip_jump_bypass_min_scan_fit must be in [0.0, 1.0]')
        if (self.SLIP_JUMP_BYPASS_MAX_TRANSLATION <= 0.0
                or self.SLIP_JUMP_BYPASS_MAX_YAW <= 0.0):
            raise ValueError('slip jump-bypass limits must be positive')

    def _filter_worker_loop(self):
        """Process only the newest pending scan outside the ROS executor."""
        minimum_period = 1.0 / self.MAX_UPDATE_RATE_HZ
        next_allowed_start = 0.0

        while not self.worker_stop.is_set():
            rate_wait = next_allowed_start - time.monotonic()
            if rate_wait > 0.0 and self.worker_stop.wait(rate_wait):
                break

            scan = self.pending_scans.take(timeout_sec=0.1)
            if scan is None:
                continue

            if time.monotonic() - scan.received_monotonic > \
                    self.MAX_PENDING_SCAN_AGE_SEC:
                self.scan_stale_drop_count += 1
                self._log_sync_diagnostics()
                continue

            with self.filter_reference_lock:
                generation = self.filter_generation
                previous_scan_stamp_ns = self.previous_filter_scan_stamp_ns
            beam_stamps_ns = tuple(
                scan.stamp_ns + int(offset)
                for offset in scan.beam_offsets_ns
            )
            geometry_beam_stamps_ns = tuple(
                scan.stamp_ns + int(offset)
                for offset in scan.geometry_beam_offsets_ns
            )
            beam_odom_samples = None
            odom_sample = None
            if scan.deskew_timing_valid:
                trajectory = self.odom_buffer.wait_for_interpolated_many(
                    (scan.stamp_ns,) + beam_stamps_ns,
                    timeout_sec=self.ODOM_SYNC_WAIT_SEC,
                    max_gap_sec=self.MAX_ODOM_INTERP_GAP_SEC,
                )
                if trajectory.status == LOOKUP_OK:
                    odom_sample = trajectory.samples[0]
                    beam_odom_samples = trajectory.samples[1:]
                else:
                    self.deskew_odom_fallback_count += 1
            if odom_sample is None:
                lookup = self.odom_buffer.wait_for_interpolated(
                    scan.stamp_ns,
                    timeout_sec=(
                        0.0 if scan.deskew_timing_valid
                        else self.ODOM_SYNC_WAIT_SEC),
                    max_gap_sec=self.MAX_ODOM_INTERP_GAP_SEC,
                )
                if lookup.status == LOOKUP_OK:
                    odom_sample = lookup.sample
            if self.worker_stop.is_set():
                break
            if odom_sample is None:
                self.scan_sync_drop_count += 1
                self._log_sync_diagnostics()
                continue
            if time.monotonic() - scan.received_monotonic > \
                    self.MAX_PENDING_SCAN_AGE_SEC:
                self.scan_stale_drop_count += 1
                self._log_sync_diagnostics()
                continue

            imu_yaw_lookup = None
            if scan.deskew_timing_valid and self.imu_initialized:
                imu_yaw_lookup = \
                    self.imu_buffer.wait_for_integrated_yaw_offsets(
                        scan.stamp_ns,
                        beam_stamps_ns,
                        timeout_sec=self.IMU_SYNC_WAIT_SEC,
                        max_gap_sec=self.MAX_IMU_INTERP_GAP_SEC,
                    )
                if imu_yaw_lookup.status != LOOKUP_OK:
                    self.deskew_imu_fallback_count += 1
            elif scan.deskew_timing_valid:
                self.deskew_imu_fallback_count += 1

            imu_attitude_lookup = None
            if self.GROUND_FILTER_ENABLED and self.imu_initialized:
                imu_attitude_lookup = \
                    self.imu_attitude_buffer.wait_for_lookup_many(
                        (scan.stamp_ns,) + geometry_beam_stamps_ns,
                        timeout_sec=(
                            0.0 if imu_yaw_lookup is not None
                            and imu_yaw_lookup.status == LOOKUP_OK
                            else self.IMU_SYNC_WAIT_SEC),
                        max_gap_sec=self.MAX_IMU_INTERP_GAP_SEC,
                    )
                if imu_attitude_lookup.status != LOOKUP_OK:
                    self.ground_filter_sync_miss_count += 1

            imu_integration = None
            if (previous_scan_stamp_ns is not None and self.imu_initialized
                    and scan.stamp_ns > previous_scan_stamp_ns):
                imu_integration = self.imu_buffer.wait_for_integrated(
                    previous_scan_stamp_ns,
                    scan.stamp_ns,
                    timeout_sec=self.IMU_SYNC_WAIT_SEC,
                    max_gap_sec=self.MAX_IMU_INTERP_GAP_SEC,
                )
                if imu_integration.status != LOOKUP_OK:
                    self.imu_sync_miss_count += 1
            elif (previous_scan_stamp_ns is not None
                    and self.imu_initialized):
                self.imu_sync_miss_count += 1

            if time.monotonic() - scan.received_monotonic > \
                    self.MAX_PENDING_SCAN_AGE_SEC:
                self.scan_stale_drop_count += 1
                self._log_sync_diagnostics()
                continue

            started = time.monotonic()
            try:
                processed = self._process_synchronized_scan(
                    scan,
                    odom_sample,
                    generation,
                    imu_integration,
                    beam_odom_samples,
                    imu_yaw_lookup,
                    imu_attitude_lookup,
                )
            except Exception as error:
                self.worker_error_count += 1
                if self.worker_error_count <= 3 or self.worker_error_count % 20 == 0:
                    self.get_logger().error(
                        'PF worker error (%d total): %s' % (
                            self.worker_error_count,
                            error,
                        )
                    )
                continue

            if not processed:
                continue

            finished = time.monotonic()
            compute_ms = (finished - started) * 1000.0
            self.compute_time_ms.append(compute_ms)
            self.scan_turnaround_ms.append(
                (finished - scan.received_monotonic) * 1000.0)
            self.smoothing.append(1000.0 / max(compute_ms, 1e-6))
            self.scan_processed_count += 1
            next_allowed_start = started + minimum_period
            self._log_sync_diagnostics()

    def ground_beam_mask(self, scan, imu_attitude_lookup):
        """Find continuous geometry in the raw scan, then project to PF rays."""
        required = np.ones(scan.ranges.size, dtype=bool)
        self.last_geometry_reason = 'disabled'
        if not self.GROUND_FILTER_ENABLED:
            return required, False
        if (self.base_to_laser_translation is None
                or self.base_to_laser_quaternion is None):
            self.last_geometry_reason = 'laser_tf'
            return required, False
        with self.attitude_lock:
            level = (np.copy(self.level_reference.quaternion)
                     if self.level_reference.ready else None)
        if (level is None or imu_attitude_lookup is None
                or imu_attitude_lookup.status != LOOKUP_OK
                or len(imu_attitude_lookup.quaternions)
                != scan.geometry_ranges.size + 1):
            self.last_geometry_reason = 'imu_waiting'
            return required, False

        result = mask_ground_and_offplane(
            scan.geometry_ranges,
            self.laser_angles,
            self.all_beam_indices,
            imu_attitude_lookup.quaternions[1:],
            level,
            self.base_to_laser_translation,
            self.base_to_laser_quaternion,
            self.GROUND_FILTER_CONFIG,
        )
        selected_ground = result.ground_mask[self.selected_beam_indices]
        selected_offplane = result.offplane_mask[self.selected_beam_indices]
        selected_keep = result.keep_mask[self.selected_beam_indices]
        # Horizontal no-return rays still carry useful max-range evidence for
        # MCL. They are retained unless their 3D direction itself is off-plane.
        required = selected_keep | (
            ~scan.valid_returns & ~selected_offplane)
        retained = int(np.count_nonzero(required))
        degraded = bool(result.degraded)
        if (result.reason == 'too_few_retained'
                and retained >= self.GROUND_FILTER_CONFIG.min_retained_beams):
            degraded = False
        if retained < self.GROUND_FILTER_CONFIG.min_retained_beams:
            degraded = True
            reason = 'too_few_planar'
        else:
            reason = result.reason
        self.ground_filtered_beam_count += int(
            np.count_nonzero(selected_ground))
        self.offplane_filtered_beam_count += int(
            np.count_nonzero(selected_offplane))
        if result.attitude_fallback:
            self.attitude_fallback_scan_count += 1
        if degraded:
            self.geometry_degraded_scan_count += 1
        self.last_geometry_reason = reason
        return required, degraded

    def _process_synchronized_scan(
            self, scan, odom_sample, generation, imu_integration,
            beam_odom_samples, imu_yaw_lookup, imu_attitude_lookup):
        """Run one PF update using odom interpolated at this exact scan time."""
        if self.laser_to_base_pose is None and not self.lookup_laser_to_base_pose():
            return False
        if not (self.lidar_initialized and self.odom_initialized and self.map_initialized):
            return False

        with self.state_lock:
            with self.filter_reference_lock:
                if generation != self.filter_generation:
                    return False
                previous_odom = self.previous_filter_odom
                previous_ranges = self.previous_filter_scan_ranges
                previous_angles = self.previous_filter_scan_angles

            odom_pose = self.odom_sample_to_pose(odom_sample)
            if previous_odom is None:
                raw_action = np.zeros(3)
            else:
                previous_pose = self.odom_sample_to_pose(previous_odom)
                raw_action = self.compose_pose_2d(
                    self.inverse_pose_2d(previous_pose),
                    odom_pose,
                )
            required_beam_mask, geometry_degraded = \
                self.ground_beam_mask(scan, imu_attitude_lookup)
            scan_ranges, scan_angles = self.deskew_scan(
                scan,
                odom_sample,
                beam_odom_samples,
                imu_yaw_lookup,
            )
            action = self.filter_low_speed_action(
                raw_action, odom_sample.linear_speed)

            icp_ranges = None
            if self.SCAN_MATCH_ENABLED:
                icp_ranges = np.copy(scan_ranges)
                icp_ranges[~required_beam_mask] = self.MAX_RANGE_METERS
            if geometry_degraded:
                scan_motion = np.zeros(3)
                scan_motion_valid = False
                self.last_scan_motion_valid = False
                self.last_scan_motion[:] = 0.0
                self.last_scan_match_rmse_m = -1.0
                self.last_scan_match_inlier_fraction = -1.0
            else:
                scan_motion, scan_motion_valid = self.estimate_scan_action(
                    previous_ranges,
                    previous_angles,
                    scan_ranges if icp_ranges is None else icp_ranges,
                    scan_angles,
                    raw_action,
                )
            imu_valid = (
                imu_integration is not None
                and imu_integration.status == LOOKUP_OK
            )
            imu_delta_yaw = (
                imu_integration.delta_yaw if imu_valid else 0.0)
            imu_stationary = bool(
                imu_valid
                and imu_integration.max_abs_yaw_rate
                <= self.STATIONARY_IMU_YAW_RATE
                and imu_integration.max_horizontal_acceleration
                <= self.STATIONARY_IMU_HORIZONTAL_ACCEL
            )
            self.last_imu_stationary = imu_stationary
            self.last_wheel_stationary = bool(
                abs(odom_sample.linear_speed) < self.LOW_SPEED_THRESHOLD
                and np.hypot(action[0], action[1]) <= 1e-12
                and abs(action[2]) <= 1e-12
            )
            self.last_scan_stationary = bool(
                scan_motion_valid
                and np.hypot(scan_motion[0], scan_motion[1])
                <= self.STATIONARY_SCAN_TRANSLATION
                and abs(scan_motion[2]) <= self.STATIONARY_SCAN_YAW
            )
            if self.SCAN_MATCH_ENABLED:
                slip_update = self.slip_detector.update(
                    raw_action,
                    imu_delta_yaw,
                    imu_valid,
                    scan_motion,
                    scan_motion_valid,
                )
            else:
                # Slip recovery requires valid scan and IMU evidence.  An ICP
                # A/B with no scan evidence must therefore clear, not latch,
                # the detector and its motion-noise multiplier.
                slip_update = self.slip_detector.disabled_update()
            self.last_slip_update = slip_update
            self.handle_slip_transition(slip_update, scan.stamp_ns)

            self.current_speed = odom_sample.linear_speed
            self.downsampled_ranges = scan_ranges
            self.current_scan_angles = scan_angles
            self.last_stamp = self.nanoseconds_to_stamp(scan.stamp_ns)
            self.timer.tick()
            self.iters += 1

            prior_particles = np.copy(self.particles)
            prior_weights = np.copy(self.weights)
            prior_pose = None
            if self.inferred_pose is not None:
                prior_pose = np.copy(self.inferred_pose)

            motion_noise_multiplier = (
                self.SLIP_MOTION_NOISE_MULTIPLIER
                if slip_update.active else 1.0
            )
            if geometry_degraded:
                sensor_update_ok = False
                self.last_beam_selection = None
            else:
                sensor_update_ok = self.MCL(
                    action,
                    scan_ranges,
                    scan_angles,
                    motion_noise_multiplier=motion_noise_multiplier,
                    required_beam_mask=required_beam_mask,
                )
            correction_accepted = False
            stationary_hold = (
                sensor_update_ok
                and not slip_update.active
                and should_hold_stationary_filter(
                    self.FREEZE_STATIONARY_FILTER,
                    odom_sample.linear_speed,
                    self.LOW_SPEED_THRESHOLD,
                    action,
                    imu_stationary,
                    scan_motion,
                    scan_motion_valid,
                    self.STATIONARY_SCAN_TRANSLATION,
                    self.STATIONARY_SCAN_YAW,
                )
            )
            if sensor_update_ok:
                candidate_pose = self.expected_pose()
                predicted_pose = None
                correction_xy = 0.0
                correction_yaw = 0.0
                if prior_pose is not None:
                    predicted_pose = self.compose_pose_2d(prior_pose, action)
                    _, correction_xy, correction_yaw = \
                        self.should_reject_pose_update(
                            predicted_pose, candidate_pose)
                    if slip_update.active:
                        self.record_slip_correction(
                            correction_xy, correction_yaw)
                if stationary_hold:
                    self.stationary_hold_count += 1
                    self.consecutive_jump_rejects = 0
                    self.restore_motion_only_pose(
                        prior_particles, prior_weights, action)
                elif self.POSE_JUMP_GUARD_ENABLED and prior_pose is not None:
                    reject, correction_xy, correction_yaw = self.should_reject_pose_update(
                        predicted_pose, candidate_pose)
                    if reject and self.should_bypass_jump_guard_for_slip(
                            slip_update, correction_xy, correction_yaw):
                        self.slip_jump_bypass_count += 1
                        self.get_logger().warn(
                            'PF slip jump-guard bypass: correction %.2fm, '
                            '%.1fdeg, scan-fit %.2f' % (
                                correction_xy,
                                np.degrees(correction_yaw),
                                self.current_scan_fit(),
                            )
                        )
                        reject = False
                    elif (reject and self.MAX_CONSECUTIVE_JUMP_REJECTS > 0
                            and self.consecutive_jump_rejects + 1
                            >= self.MAX_CONSECUTIVE_JUMP_REJECTS):
                        self.get_logger().warn(
                            'PF jump guard 연속 거부 %d회 - 예측이 어긋난 것으로 '
                            '보고 보정 %.2fm, %.1fdeg 수용(재동기화)' % (
                                self.consecutive_jump_rejects + 1,
                                correction_xy,
                                np.degrees(correction_yaw),
                            )
                        )
                        reject = False
                    if reject:
                        self.jump_reject_count += 1
                        self.consecutive_jump_rejects += 1
                        if (self.jump_reject_count <= 5 or
                                self.jump_reject_count % 20 == 0):
                            self.get_logger().warn(
                                'Rejected PF scan update: correction %.2fm, %.1fdeg '
                                'exceeds guard; keeping odom-propagated pose (연속 %d회)' % (
                                    correction_xy,
                                    np.degrees(correction_yaw),
                                    self.consecutive_jump_rejects,
                                )
                            )
                        self.restore_motion_only_pose(
                            prior_particles, prior_weights, action)
                    else:
                        self.consecutive_jump_rejects = 0
                        self.inferred_pose = candidate_pose
                        correction_accepted = True
                else:
                    self.inferred_pose = candidate_pose
                    correction_accepted = True
            else:
                if geometry_degraded:
                    self.get_logger().warn(
                        'PF rejected non-planar LiDAR frame (%s); '
                        'using odom-only propagation' % self.last_geometry_reason,
                        throttle_duration_sec=1.0)
                else:
                    self.get_logger().warn(
                        'PF sensor update produced invalid weights; '
                        'keeping odom-propagated pose')
                self.restore_motion_only_pose(
                    prior_particles, prior_weights, action)

            if self.last_beam_selection is not None:
                quality_score = (
                    self.last_beam_selection.weighted_inlier_fraction
                    if correction_accepted or stationary_hold else 0.0
                )
                self.update_localization_quality(quality_score)

            covariance = self.pose_covariance_6x6()
            quality_blocks_output = (
                self.QUALITY_GATE_ENABLED
                and self.quality_tracker.state == QualityState.LOST
            )
            if not quality_blocks_output:
                self.set_localization_transform(
                    self.inferred_pose,
                    odom_sample,
                    covariance,
                )
            with self.filter_reference_lock:
                self.previous_filter_odom = odom_sample
                self.previous_filter_scan_stamp_ns = scan.stamp_ns
                self.previous_filter_scan_ranges = (
                    None if geometry_degraded or icp_ranges is None
                    else np.copy(icp_ranges))
                self.previous_filter_scan_angles = (
                    None if geometry_degraded or icp_ranges is None
                    else np.copy(scan_angles))

        # This is also the control-node lease heartbeat. Keep DDS publication
        # outside state_lock so a slow subscriber cannot delay pose resets.
        self.publish_localization_status()
        self.visualize()
        return True

    def deskew_scan(
            self, scan, reference_odom, beam_odom_samples,
            imu_yaw_lookup):
        """Represent selected beam endpoints at the scan midpoint."""
        self.last_scan_duration_sec = scan.scan_duration_sec
        if not scan.deskew_timing_valid:
            self.last_deskew_source = (
                'OFF' if not self.DESKEW_ENABLED else 'TIMING')
            self.last_deskew_translation_scale = 0.0
            self.last_deskew_max_translation_m = 0.0
            self.last_deskew_max_yaw_rad = 0.0
            return (
                np.copy(scan.ranges),
                np.copy(self.downsampled_angles),
            )

        count = scan.ranges.size
        reference_pose = self.odom_sample_to_pose(reference_odom)
        relative_base_actions = np.zeros((count, 3), dtype=np.float64)
        odom_available = (
            beam_odom_samples is not None
            and len(beam_odom_samples) == count
        )
        if odom_available:
            beam_poses = np.asarray([
                (sample.x, sample.y, sample.yaw)
                for sample in beam_odom_samples
            ], dtype=np.float64)
            relative_base_actions = relative_pose_batch(
                reference_pose, beam_poses)

        imu_available = bool(
            imu_yaw_lookup is not None
            and imu_yaw_lookup.status == LOOKUP_OK
            and len(imu_yaw_lookup.delta_yaws) == count
        )
        if imu_available:
            yaw_offsets = np.asarray(
                imu_yaw_lookup.delta_yaws, dtype=np.float64)
            source = 'IMU'
        elif odom_available:
            yaw_offsets = np.copy(relative_base_actions[:, 2])
            source = 'ODOM'
        else:
            self.last_deskew_source = 'NONE'
            self.last_deskew_translation_scale = 0.0
            self.last_deskew_max_translation_m = 0.0
            self.last_deskew_max_yaw_rad = 0.0
            return (
                np.copy(scan.ranges),
                np.copy(self.downsampled_angles),
            )

        slip_active = bool(
            self.last_slip_update is not None
            and self.last_slip_update.active
        )
        translation_scale = (
            self.DESKEW_SLIP_TRANSLATION_SCALE
            if slip_active else 1.0
        )
        if not odom_available:
            translation_scale = 0.0

        result = deskew_scan_to_reference(
            scan.ranges,
            self.downsampled_angles,
            relative_base_actions,
            yaw_offsets,
            self.inverse_pose_2d(self.laser_to_base_pose),
            translation_scale,
            self.MAX_RANGE_METERS,
        )
        self.deskew_applied_count += 1
        self.last_deskew_source = source
        self.last_deskew_translation_scale = translation_scale
        self.last_deskew_max_translation_m = result.max_translation_m
        self.last_deskew_max_yaw_rad = result.max_yaw_rad
        return (
            result.ranges.astype(np.float32),
            result.angles.astype(np.float32),
        )

    def estimate_scan_action(
            self, previous_ranges, previous_angles,
            current_ranges, current_angles, odom_action):
        """Run the bounded scan matcher and apply its confidence gate."""
        if (not self.SCAN_MATCH_ENABLED
                or previous_ranges is None or previous_angles is None):
            self.last_scan_motion_valid = False
            self.last_scan_motion[:] = 0.0
            self.last_scan_match_rmse_m = -1.0
            self.last_scan_match_inlier_fraction = -1.0
            return np.zeros(3), False

        estimate = estimate_scan_motion(
            previous_ranges,
            current_ranges,
            previous_angles,
            odom_action,
            self.MAX_RANGE_METERS,
            self.SCAN_MATCH_MAX_CORRESPONDENCE,
            self.SCAN_MATCH_TRIM_FRACTION,
            self.SCAN_MATCH_ITERATIONS,
            self.SCAN_MATCH_MIN_POINTS,
            current_angles=current_angles,
        )
        valid = bool(
            estimate.valid
            and estimate.inlier_fraction
            >= self.SCAN_MATCH_MIN_INLIER_FRACTION
            and estimate.rmse_m <= self.SCAN_MATCH_MAX_RMSE
        )
        if not valid:
            self.scan_match_invalid_count += 1
        self.last_scan_motion_valid = valid
        self.last_scan_motion = np.copy(estimate.action)
        self.last_scan_match_rmse_m = estimate.rmse_m
        self.last_scan_match_inlier_fraction = estimate.inlier_fraction
        return estimate.action, valid

    def current_scan_fit(self):
        """Return current weighted scan-map support, or -1 when unavailable."""
        if self.last_beam_selection is None:
            return -1.0
        return self.last_beam_selection.weighted_inlier_fraction

    def should_bypass_jump_guard_for_slip(
            self, slip_update, correction_xy, correction_yaw):
        """Allow a bounded, map-supported correction during detected slip."""
        return can_bypass_jump_guard_for_slip(
            slip_update.active,
            self.current_scan_fit(),
            self.SLIP_JUMP_BYPASS_MIN_SCAN_FIT,
            correction_xy,
            correction_yaw,
            self.SLIP_JUMP_BYPASS_MAX_TRANSLATION,
            self.SLIP_JUMP_BYPASS_MAX_YAW,
        )

    def handle_slip_transition(self, slip_update, stamp_ns):
        """Record scan-time slip entry and recovery diagnostics."""
        if slip_update.entered:
            self.slip_event_count += 1
            self.slip_enter_stamp_ns = stamp_ns
            self.slip_event_max_correction_m = 0.0
            self.slip_event_max_correction_yaw = 0.0
            self.get_logger().warn(
                'PF slip detected at scan-time %.3f: score %.2f, '
                'dxy %.3fm, yaw scan/odom %.1fdeg, odom/IMU %.1fdeg, '
                'scan/IMU %.1fdeg' % (
                    stamp_ns / 1e9,
                    slip_update.score,
                    slip_update.translation_innovation_m,
                    np.degrees(
                        slip_update.scan_odom_yaw_innovation_rad),
                    np.degrees(
                        slip_update.odom_imu_yaw_innovation_rad),
                    np.degrees(
                        slip_update.scan_imu_yaw_innovation_rad),
                )
            )
        elif slip_update.recovered:
            self.slip_recovery_count += 1
            if self.slip_enter_stamp_ns is not None:
                self.slip_last_duration_sec = max(
                    0.0, (stamp_ns - self.slip_enter_stamp_ns) / 1e9)
            self.slip_last_max_correction_m = \
                self.slip_event_max_correction_m
            self.slip_last_max_correction_yaw = \
                self.slip_event_max_correction_yaw
            self.get_logger().info(
                'PF slip recovered at scan-time %.3f: duration %.3fs, '
                'max correction %.2fm/%.1fdeg' % (
                    stamp_ns / 1e9,
                    self.slip_last_duration_sec,
                    self.slip_last_max_correction_m,
                    np.degrees(self.slip_last_max_correction_yaw),
                )
            )
            self.slip_enter_stamp_ns = None

    def record_slip_correction(self, correction_xy, correction_yaw):
        """Track the largest PF correction observed in the active event."""
        self.slip_event_max_correction_m = max(
            self.slip_event_max_correction_m, float(correction_xy))
        self.slip_event_max_correction_yaw = max(
            self.slip_event_max_correction_yaw, float(correction_yaw))

    def filter_low_speed_action(self, action, speed):
        """Apply the existing low-speed deadband to a scan-to-scan action."""
        filtered = np.copy(action)
        if abs(speed) >= self.LOW_SPEED_THRESHOLD:
            return filtered
        if np.hypot(filtered[0], filtered[1]) < self.LOW_SPEED_ODOM_XY_DEADBAND:
            filtered[0] = 0.0
            filtered[1] = 0.0
        if abs(filtered[2]) < self.LOW_SPEED_ODOM_YAW_DEADBAND:
            filtered[2] = 0.0
        filtered *= self.LOW_SPEED_ODOM_SCALE
        return filtered

    def pose_covariance_6x6(self):
        """Expand PF [x, y, yaw] covariance into an odometry covariance."""
        covariance = np.zeros((6, 6))
        covariance3 = self.pose_covariance_3x3()
        indices = [0, 1, 5]
        for source_row, target_row in enumerate(indices):
            for source_column, target_column in enumerate(indices):
                covariance[target_row, target_column] = covariance3[
                    source_row, source_column]
        return covariance

    def reset_filter_reference(self, clear_transform=False):
        """Start scan-to-scan odom integration again after a state reset."""
        with self.filter_reference_lock:
            self.previous_filter_odom = None
            self.previous_filter_scan_stamp_ns = None
            self.previous_filter_scan_ranges = None
            self.previous_filter_scan_angles = None
            self.filter_generation += 1
        if hasattr(self, 'quality_tracker'):
            self.quality_tracker.reset()
        if hasattr(self, 'readiness_tracker'):
            self.readiness_tracker.reset_quality()
        if hasattr(self, 'slip_detector'):
            self.slip_detector.reset()
            self.last_slip_update = None
            self.slip_enter_stamp_ns = None
            self.slip_event_max_correction_m = 0.0
            self.slip_event_max_correction_yaw = 0.0
        if clear_transform:
            with self.transform_lock:
                self.map_to_odom = None
                self.map_to_odom_updated_monotonic = 0.0
                self.localized_covariance = np.zeros((6, 6))
            self.localization_output_stale_active = False
        self.publish_localization_status()

    def _log_sync_diagnostics(self):
        now = time.monotonic()
        if now - self.last_sync_diagnostic_time < self.SYNC_DIAGNOSTICS_PERIOD_SEC:
            return
        offered, overwritten, _ = self.pending_scans.counts()
        overwrite_ratio = 100.0 * overwritten / max(offered, 1)
        rate_hz = self.timer.fps() if self.timer.arr.num_els else 0.0
        compute_ms = (
            self.compute_time_ms.mean()
            if self.compute_time_ms.num_els else 0.0
        )
        turnaround_ms = (
            self.scan_turnaround_ms.mean()
            if self.scan_turnaround_ms.num_els else 0.0
        )
        estimated_core_load = rate_hz * compute_ms / 10.0
        with self.transform_lock:
            correction_updated = self.map_to_odom_updated_monotonic
        correction_age_ms = -1.0
        if correction_updated > 0.0:
            correction_age_ms = (now - correction_updated) * 1000.0
        scan_fit = -1.0
        beam_skip_percent = -1.0
        if self.last_beam_selection is not None:
            scan_fit = self.last_beam_selection.weighted_inlier_fraction
            beam_skip_percent = (
                100.0 * self.last_beam_selection.skipped_fraction)
        slip_active = bool(
            self.last_slip_update is not None
            and self.last_slip_update.active)
        slip_score = (
            self.last_slip_update.score
            if self.last_slip_update is not None else 0.0)
        with self.filter_reference_lock:
            processed_scan_stamp_ns = self.previous_filter_scan_stamp_ns
        slip_age_sec = 0.0
        if (slip_active and self.slip_enter_stamp_ns is not None
                and processed_scan_stamp_ns is not None):
            slip_age_sec = max(
                0.0,
                (processed_scan_stamp_ns - self.slip_enter_stamp_ns) / 1e9,
            )
        slip_max_correction_m = (
            self.slip_event_max_correction_m
            if slip_active else self.slip_last_max_correction_m)
        slip_max_correction_yaw = (
            self.slip_event_max_correction_yaw
            if slip_active else self.slip_last_max_correction_yaw)
        self.get_logger().info(
            'PF sync: %.1fHz, compute %.1fms, scan turnaround %.1fms, '
            'correction-age %.0fms, CPU-core %.0f%%, scans %d/%d, '
            'latest-overwrite %d (%.0f%%), '
            'sync/stale/order-drop %d/%d/%d, worker-error %d, '
            'odom-buffer %d/%d, ESS-prior/post %.2f/%.2f, '
            'scan-fit %.2f, beam-skip %.0f%%, beam-fallback %d, '
            'quality %s good/bad %d/%d, '
            'ready/initialpose-received/accepted %d/%d/%d, '
            'resampled/lost-updates/lost-transitions %d/%d/%d, '
            'stationary-hold %d, stationary wheel/IMU/scan %d/%d/%d, '
            'IMU-buffer %d/%d, imu-miss/reject %d/%d, '
            'scan-match-invalid %d, '
            'scan-motion %s %.3fm/%.1fdeg rmse %.3fm inlier %.2f, '
            'slip %s score %.2f, events/recoveries/bypass %d/%d/%d, '
            'slip-age/last-duration %.2f/%.2fs, '
            'slip-max-correction %.2fm/%.1fdeg, '
            'deskew %s applied %d, timing/odom/imu-fallback %d/%d/%d, '
            'scan-span %.1fms, translation-scale %.2f, '
            'deskew-max-motion %.3fm/%.1fdeg, '
            'geometry %s ground/offplane/degraded/sync-miss/fallback '
            '%d/%d/%d/%d/%d, '
            'attitude-buffer %d/%d level %s, shape-drop %d, profile %s' % (
                rate_hz,
                compute_ms,
                turnaround_ms,
                correction_age_ms,
                estimated_core_load,
                self.scan_processed_count,
                offered,
                overwritten,
                overwrite_ratio,
                self.scan_sync_drop_count,
                self.scan_stale_drop_count,
                self.scan_out_of_order_count,
                self.worker_error_count,
                len(self.odom_buffer),
                self.ODOM_BUFFER_SIZE,
                self.last_prior_ess_ratio,
                self.last_posterior_ess_ratio,
                scan_fit,
                beam_skip_percent,
                self.beam_skip_fallback_count,
                self.quality_tracker.state.value,
                self.quality_tracker.consecutive_good,
                self.quality_tracker.consecutive_lost,
                int(self.readiness_tracker.ready),
                int(self.readiness_tracker.initial_pose_received),
                int(self.readiness_tracker.initial_pose_accepted),
                self.resample_count,
                self.quality_lost_update_count,
                self.quality_lost_transition_count,
                self.stationary_hold_count,
                int(self.last_wheel_stationary),
                int(self.last_imu_stationary),
                int(self.last_scan_stationary),
                len(self.imu_buffer),
                self.IMU_BUFFER_SIZE,
                self.imu_sync_miss_count,
                self.imu_reject_count,
                self.scan_match_invalid_count,
                'OK' if self.last_scan_motion_valid else 'INVALID',
                np.hypot(
                    self.last_scan_motion[0], self.last_scan_motion[1]),
                np.degrees(abs(self.last_scan_motion[2])),
                self.last_scan_match_rmse_m,
                self.last_scan_match_inlier_fraction,
                'ACTIVE' if slip_active else 'OK',
                slip_score,
                self.slip_event_count,
                self.slip_recovery_count,
                self.slip_jump_bypass_count,
                slip_age_sec,
                self.slip_last_duration_sec,
                slip_max_correction_m,
                np.degrees(slip_max_correction_yaw),
                self.last_deskew_source,
                self.deskew_applied_count,
                self.deskew_timing_invalid_count,
                self.deskew_odom_fallback_count,
                self.deskew_imu_fallback_count,
                self.last_scan_duration_sec * 1000.0,
                self.last_deskew_translation_scale,
                self.last_deskew_max_translation_m,
                np.degrees(self.last_deskew_max_yaw_rad),
                self.last_geometry_reason,
                self.ground_filtered_beam_count,
                self.offplane_filtered_beam_count,
                self.geometry_degraded_scan_count,
                self.ground_filter_sync_miss_count,
                self.attitude_fallback_scan_count,
                len(self.imu_attitude_buffer),
                self.IMU_BUFFER_SIZE,
                'ready' if self.level_reference.ready else 'waiting',
                self.scan_shape_drop_count,
                self.PF_PROFILE_LABEL,
            )
        )
        self.last_sync_diagnostic_time = now

    @staticmethod
    def stamp_to_nanoseconds(stamp):
        """Convert a ROS stamp message to an integer without floating error."""
        return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)

    @staticmethod
    def nanoseconds_to_stamp(stamp_ns):
        """Convert integer nanoseconds back to a ROS stamp message."""
        return RosTime(nanoseconds=int(stamp_ns)).to_msg()

    @staticmethod
    def odom_sample_to_pose(sample):
        """Convert a lightweight odom sample to the PF planar pose format."""
        return np.array([sample.x, sample.y, sample.yaw])

    def destroy_node(self):
        """Stop the PF worker before destroying ROS publishers and handles."""
        self.worker_stop.set()
        self.pending_scans.close()
        self.odom_buffer.close()
        self.imu_buffer.close()
        self.imu_attitude_buffer.close()
        if (self.filter_worker is not None and self.filter_worker.is_alive()
                and current_thread() is not self.filter_worker):
            self.filter_worker.join(timeout=2.0)
        return super().destroy_node()

# import argparse
# import sys
# parser = argparse.ArgumentParser(description='Particle filter.')
# parser.add_argument('--config', help='Path to yaml file containing config parameters. Helpful for calling node directly with Python for profiling.')

# def load_params_from_yaml(fp):
#     from yaml import load
#     with open(fp, 'r') as infile:
#         yaml_data = load(infile)
#         for param in yaml_data:
#             print 'param:', param, ':', yaml_data[param]
#             rospy.set_param('~'+param, yaml_data[param])

# # this function can be used to generate flame graphs easily
# def make_flamegraph(filterx=None):
#     import flamegraph, os
#     perf_log_path = os.path.join(os.path.dirname(__file__), '../tmp/perf.log')
#     flamegraph.start_profile_thread(fd=open(perf_log_path, 'w'),
#                                     filter=filterx,
#                                     interval=0.001)

def main(args=None):
    rclpy.init(args=args)
    pf = ParticleFiler()
    try:
        rclpy.spin(pf)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            pf.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()
        except KeyboardInterrupt:
            # Launch may forward a second SIGINT while rclpy handles the first.
            pass

if __name__ == '__main__':
    main()

# if __name__=='__main__':
#     rospy.init_node('particle_filter')

#     args,_ = parser.parse_known_args()
#     if args.config:
#         load_params_from_yaml(args.config)

#     # make_flamegraph(r'update')

#     pf = ParticleFiler()
#     rospy.spin()
