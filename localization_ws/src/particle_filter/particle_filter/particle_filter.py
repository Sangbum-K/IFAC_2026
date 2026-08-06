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

from dataclasses import dataclass
from threading import current_thread, Event, Lock, Thread
import time

import numpy as np
import range_libc
import rclpy
from geometry_msgs.msg import PointStamped
from geometry_msgs.msg import PolygonStamped
from geometry_msgs.msg import PoseArray
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import PoseWithCovarianceStamped
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from nav_msgs.srv import GetMap
from particle_filter import utils as Utils
from particle_filter.synchronization import LatestValueSlot
from particle_filter.synchronization import LOOKUP_OK
from particle_filter.synchronization import OdomSample
from particle_filter.synchronization import TimestampedOdomBuffer
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time as RosTime
from sensor_msgs.msg import LaserScan
import tf_transformations
from tf2_ros import Buffer, TransformBroadcaster, TransformListener

'''
These flags indicate several variants of the sensor model. Only one of them is used at a time.
'''
VAR_NO_EVAL_SENSOR_MODEL = 0
VAR_CALC_RANGE_MANY_EVAL_SENSOR = 1
VAR_REPEAT_ANGLES_EVAL_SENSOR = 2
VAR_REPEAT_ANGLES_EVAL_SENSOR_ONE_SHOT = 3
VAR_RADIAL_CDDT_OPTIMIZATIONS = 4


@dataclass(frozen=True)
class ScanSample:
    """A preprocessed scan waiting for timestamp-synchronized PF work."""

    stamp_ns: int
    received_monotonic: float
    ranges: np.ndarray


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
        self.declare_parameter('max_update_rate_hz', 25.0)
        self.declare_parameter('sync_diagnostics_period_sec', 1.0)
        self.declare_parameter('max_localization_update_age_sec', 0.50)
        # Fixed noise on a zero-motion update creates artificial pose shake.
        # Scale the existing noise values by motion, while retaining a small
        # floor so stopped particles do not become completely deterministic.
        self.declare_parameter('motion_noise_min_scale', 0.08)
        self.declare_parameter('motion_noise_reference_translation_m', 0.25)
        self.declare_parameter('motion_noise_reference_yaw_rad', 0.15)

        # parameters
        self.ANGLE_STEP           = self.get_parameter('angle_step').value
        self.MAX_PARTICLES        = self.get_parameter('max_particles').value
        self.MAX_VIZ_PARTICLES    = self.get_parameter('max_viz_particles').value
        self.INV_SQUASH_FACTOR    = 1.0 / self.get_parameter('squash_factor').value
        self.MAX_RANGE_METERS     = self.get_parameter('max_range').value
        self.THETA_DISCRETIZATION = self.get_parameter('theta_discretization').value
        self.WHICH_RM             = self.get_parameter('range_method').value
        self.RANGELIB_VAR         = self.get_parameter('rangelib_variant').value
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
        self._validate_sync_parameters()
        
        # various data containers used in the MCL algorithm
        self.MAX_RANGE_PX = None
        self.laser = None
        self.iters = 0
        self.map_info = None
        self.map_initialized = False
        self.lidar_initialized = False
        self.odom_initialized = False
        self.laser_angles = None
        self.downsampled_angles = None
        self.downsampled_ranges = None
        self.range_method = None
        self.last_time = None
        self.last_stamp = None
        self.first_sensor_update = True
        self.state_lock = Lock()
        self.transform_lock = Lock()
        self.filter_reference_lock = Lock()
        self.jump_reject_count = 0            # 로그 스로틀용 누적 카운터
        self.consecutive_jump_rejects = 0     # 탈출구 판정용 연속 카운터

        # Input callbacks never run MCL. They only touch these bounded,
        # independently locked containers and return immediately.
        self.odom_buffer = TimestampedOdomBuffer(self.ODOM_BUFFER_SIZE)
        self.pending_scans = LatestValueSlot()
        self.worker_stop = Event()
        self.filter_worker = None
        self.previous_filter_odom = None
        self.filter_generation = 0
        self.map_to_odom = None
        self.map_to_odom_updated_monotonic = 0.0
        self.localized_covariance = np.zeros((6, 6))
        self.localization_output_stale_active = False
        self.scan_processed_count = 0
        self.scan_sync_drop_count = 0
        self.scan_stale_drop_count = 0
        self.scan_out_of_order_count = 0
        self.worker_error_count = 0
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

        if self.PUBLISH_ODOM:
            self.odom_pub = self.create_publisher(Odometry, '/pf/pose/odom', 1)

        # these topics are for coordinate space things
        self.pub_tf = TransformBroadcaster(self)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.laser_to_base_pose = None
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
        self.pose_sub = self.create_subscription(
            PoseWithCovarianceStamped,
            '/initialpose',
            self.clicked_pose,
            1)
        self.click_sub = self.create_subscription(
            PointStamped,
            '/clicked_point',
            self.clicked_pose,
            1)

        self.filter_worker = Thread(
            target=self._filter_worker_loop,
            name='pf_scan_worker',
            daemon=True,
        )
        self.filter_worker.start()

        self.get_logger().info(
            'Finished initializing: odom buffer=%d, latest-scan worker<=%.1fHz' % (
                self.ODOM_BUFFER_SIZE,
                self.MAX_UPDATE_RATE_HZ,
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
                proposal_indices = np.random.choice(self.particle_indices, self.MAX_VIZ_PARTICLES, p=self.weights)
                # proposal_indices = np.random.choice(self.particle_indices, self.MAX_VIZ_PARTICLES)
                self.publish_particles(self.particles[proposal_indices,:])
            else:
                self.publish_particles(self.particles)

        if self.pub_fake_scan.get_subscription_count() > 0 and isinstance(self.ranges, np.ndarray):
            # generate the scan from the point of view of the inferred position for visualization
            self.viz_queries[:,0] = self.inferred_pose[0]
            self.viz_queries[:,1] = self.inferred_pose[1]
            self.viz_queries[:,2] = self.downsampled_angles + self.inferred_pose[2]
            self.range_method.calc_range_many(self.viz_queries, self.viz_ranges)
            self.publish_scan(self.downsampled_angles, self.viz_ranges)

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
            self.laser_angles = np.linspace(msg.angle_min, msg.angle_max, len(msg.ranges))
            self.downsampled_angles = np.copy(self.laser_angles[0::self.ANGLE_STEP]).astype(np.float32)
            self.viz_queries = np.zeros((self.downsampled_angles.shape[0],3), dtype=np.float32)
            self.viz_ranges = np.zeros(self.downsampled_angles.shape[0], dtype=np.float32)
            self.get_logger().info(str(self.downsampled_angles.shape[0]))

        # Replace NaN/Inf/no-return rays with max range so the sensor model
        # never sees invalid values on long straights or open track segments.
        downsampled_ranges = np.array(msg.ranges[::self.ANGLE_STEP], dtype=np.float32)
        invalid = np.logical_or(~np.isfinite(downsampled_ranges), downsampled_ranges <= 0.0)
        downsampled_ranges[invalid] = self.MAX_RANGE_METERS
        np.clip(downsampled_ranges, 0.0, self.MAX_RANGE_METERS, out=downsampled_ranges)
        stamp_ns = self.stamp_to_nanoseconds(msg.header.stamp)
        if stamp_ns <= 0:
            stamp_ns = self.get_clock().now().nanoseconds
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

    def clicked_pose(self, msg):
        '''
        Receive pose messages from RViz and initialize the particle distribution in response.
        '''
        if isinstance(msg, PointStamped):
            self.initialize_global()
        elif isinstance(msg, PoseWithCovarianceStamped):
            self.initialize_particles_pose(msg.pose.pose)

    def initialize_particles_pose(self, pose):
        '''
        Initialize particles in the general region of the provided pose.
        '''
        if self.laser_to_base_pose is None and not self.lookup_laser_to_base_pose():
            self.get_logger().info('Cannot set pose yet, waiting for laser transform')
            return

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
            self.particles[:,0] = laser_pose[0] + np.random.normal(
                loc=0.0, scale=0.5, size=self.MAX_PARTICLES)
            self.particles[:,1] = laser_pose[1] + np.random.normal(
                loc=0.0, scale=0.5, size=self.MAX_PARTICLES)
            self.particles[:,2] = self.wrap_angle(
                laser_pose[2] + np.random.normal(
                    loc=0.0, scale=0.4, size=self.MAX_PARTICLES))
            self.inferred_pose = np.copy(laser_pose)
            self.reset_filter_reference(clear_transform=True)

    def initialize_global(self):
        '''
        Spread the particle distribution over the permissible region of the state space.
        '''
        self.get_logger().info('GLOBAL INITIALIZATION')
        # randomize over grid coordinate space
        with self.state_lock:
            permissible_x, permissible_y = np.where(self.permissible_region == 1)
            indices = np.random.randint(0, len(permissible_x), size=self.MAX_PARTICLES)

            permissible_states = np.zeros((self.MAX_PARTICLES,3))
            permissible_states[:,0] = permissible_y[indices]
            permissible_states[:,1] = permissible_x[indices]
            permissible_states[:,2] = np.random.random(
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

        t = time.time()
        # d is the computed range from RangeLibc
        for d in range(table_width):
            norm = 0.0
            sum_unkown = 0.0
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
        if self.RANGELIB_VAR > 0:
            self.range_method.set_sensor_model(self.sensor_model_table)

    def motion_model(self, proposal_dist, action):
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
        proposal_dist[:,0] += np.random.normal(
            loc=0.0,
            scale=self.MOTION_DISPERSION_X * noise_scale,
            size=self.MAX_PARTICLES,
        )
        proposal_dist[:,1] += np.random.normal(
            loc=0.0,
            scale=self.MOTION_DISPERSION_Y * noise_scale,
            size=self.MAX_PARTICLES,
        )
        proposal_dist[:,2] += np.random.normal(
            loc=0.0,
            scale=self.MOTION_DISPERSION_THETA * noise_scale,
            size=self.MAX_PARTICLES,
        )
        proposal_dist[:,2] = self.wrap_angle(proposal_dist[:,2])

    def sensor_model(self, proposal_dist, obs, weights):
        '''
        This function computes a probablistic weight for each particle in the proposal distribution.
        These weights represent how probable each proposed (x,y,theta) pose is given the measured
        ranges from the lidar scanner.

        There are 4 different variants using various features of RangeLibc for demonstration purposes.
        - VAR_REPEAT_ANGLES_EVAL_SENSOR is the most stable, and is very fast.
        - VAR_NO_EVAL_SENSOR_MODEL directly indexes the precomputed sensor model. This is slow
                                   but avoids RangeLib weight underflow by accumulating in log space
        - VAR_RADIAL_CDDT_OPTIMIZATIONS is only compatible with CDDT or PCDDT, it implments the radial
                                        optimizations to CDDT which simultaneously performs ray casting
                                        in two directions, reducing the amount of work by roughly a third
        '''
        
        num_rays = self.downsampled_angles.shape[0]
        # only allocate buffers once to avoid slowness
        if self.first_sensor_update:
            if self.RANGELIB_VAR <= 1:
                self.queries = np.zeros((num_rays*self.MAX_PARTICLES,3), dtype=np.float32)
            else:
                self.queries = np.zeros((self.MAX_PARTICLES,3), dtype=np.float32)

            self.ranges = np.zeros(num_rays*self.MAX_PARTICLES, dtype=np.float32)
            self.tiled_angles = np.tile(self.downsampled_angles, self.MAX_PARTICLES)
            self.first_sensor_update = False

        if self.RANGELIB_VAR == VAR_RADIAL_CDDT_OPTIMIZATIONS:
            if 'cddt' in self.WHICH_RM:
                self.queries[:,:] = proposal_dist[:,:]
                self.range_method.calc_range_many_radial_optimized(num_rays, self.downsampled_angles[0], self.downsampled_angles[-1], self.queries, self.ranges)

                # evaluate the sensor model
                self.range_method.eval_sensor_model(obs, self.ranges, self.weights, num_rays, self.MAX_PARTICLES)
                # apply the squash factor
                self.weights = np.power(self.weights, self.INV_SQUASH_FACTOR)
            else:
                self.get_logger().info('Cannot use radial optimizations with non-CDDT based methods, use rangelib_variant 2')
        elif self.RANGELIB_VAR == VAR_REPEAT_ANGLES_EVAL_SENSOR_ONE_SHOT:
            self.queries[:,:] = proposal_dist[:,:]
            self.range_method.calc_range_repeat_angles_eval_sensor_model(self.queries, self.downsampled_angles, obs, self.weights)
            np.power(self.weights, self.INV_SQUASH_FACTOR, self.weights)
        elif self.RANGELIB_VAR == VAR_REPEAT_ANGLES_EVAL_SENSOR:
            if self.SHOW_FINE_TIMING:
                t_start = time.time()
            # this version demonstrates what this would look like with coordinate space conversion pushed to rangelib
            self.queries[:,:] = proposal_dist[:,:]
            if self.SHOW_FINE_TIMING:
                t_init = time.time()
            self.range_method.calc_range_repeat_angles(self.queries, self.downsampled_angles, self.ranges)
            if self.SHOW_FINE_TIMING:
                t_range = time.time()
            # evaluate the sensor model on the GPU
            self.range_method.eval_sensor_model(obs, self.ranges, self.weights, num_rays, self.MAX_PARTICLES)
            if self.SHOW_FINE_TIMING:
                t_eval = time.time()
            np.power(self.weights, self.INV_SQUASH_FACTOR, self.weights)
            if self.SHOW_FINE_TIMING:
                t_squash = time.time()
                t_total = (t_squash - t_start) / 100.0

            if self.SHOW_FINE_TIMING and self.iters % 10 == 0:
                self.get_logger().info(str(['sensor_model: init: ', np.round((t_init-t_start)/t_total, 2), 'range:', np.round((t_range-t_init)/t_total, 2), \
                      'eval:', np.round((t_eval-t_range)/t_total, 2), 'squash:', np.round((t_squash-t_eval)/t_total, 2)]))
        elif self.RANGELIB_VAR == VAR_CALC_RANGE_MANY_EVAL_SENSOR:
            # this version demonstrates what this would look like with coordinate space conversion pushed to rangelib
            # this part is inefficient since it requires a lot of effort to construct this redundant array
            self.queries[:,0] = np.repeat(proposal_dist[:,0], num_rays)
            self.queries[:,1] = np.repeat(proposal_dist[:,1], num_rays)
            self.queries[:,2] = np.repeat(proposal_dist[:,2], num_rays)
            self.queries[:,2] += self.tiled_angles

            self.range_method.calc_range_many(self.queries, self.ranges)

            # evaluate the sensor model on the GPU
            self.range_method.eval_sensor_model(obs, self.ranges, self.weights, num_rays, self.MAX_PARTICLES)
            np.power(self.weights, self.INV_SQUASH_FACTOR, self.weights)
        elif self.RANGELIB_VAR == VAR_NO_EVAL_SENSOR_MODEL:
            # this version directly uses the sensor model in Python, at a significant computational cost
            self.queries[:,0] = np.repeat(proposal_dist[:,0], num_rays)
            self.queries[:,1] = np.repeat(proposal_dist[:,1], num_rays)
            self.queries[:,2] = np.repeat(proposal_dist[:,2], num_rays)
            self.queries[:,2] += self.tiled_angles

            # compute the ranges for all the particles in a single functon call
            self.range_method.calc_range_many(self.queries, self.ranges)

            # Resolve the sensor model by discretizing and indexing into the
            # precomputed table, but accumulate in log space to avoid
            # underflow when many beams are multiplied together.
            obs_px = np.rint(
                np.clip(obs / float(self.map_info.resolution), 0, self.MAX_RANGE_PX)
            ).astype(np.uint16)
            range_px = np.rint(
                np.clip(self.ranges / float(self.map_info.resolution), 0, self.MAX_RANGE_PX)
            ).astype(np.uint16).reshape(self.MAX_PARTICLES, num_rays)

            sensor_probs = self.sensor_model_table[obs_px[np.newaxis, :], range_px]
            log_weights = np.sum(
                np.log(np.maximum(sensor_probs, 1e-12)),
                axis=1,
            ) * self.INV_SQUASH_FACTOR
            log_weights -= np.max(log_weights)
            np.exp(log_weights, out=weights)
        else:
            self.get_logger().info('PLEASE SET rangelib_variant PARAM to 0-4')

    def MCL(self, a, o):
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
        # draw the proposal distribution from the old particles
        proposal_indices = np.random.choice(self.particle_indices, self.MAX_PARTICLES, p=self.weights)
        proposal_distribution = self.particles[proposal_indices,:]
        if self.SHOW_FINE_TIMING:
            t_propose = time.time()

        # compute the motion model to update the proposal distribution
        self.motion_model(proposal_distribution, a)
        if self.SHOW_FINE_TIMING:
            t_motion = time.time()

        # compute the sensor model
        self.sensor_model(proposal_distribution, o, self.weights)
        if self.SHOW_FINE_TIMING:
            t_sensor = time.time()

        # normalize importance weights
        weight_sum = np.sum(self.weights)
        if not np.isfinite(weight_sum) or weight_sum <= 1e-12:
            return False
        self.weights /= weight_sum
        if self.SHOW_FINE_TIMING:
            t_norm = time.time()
            t_total = (t_norm - t)/100.0

        if self.SHOW_FINE_TIMING and self.iters % 10 == 0:
            self.get_logger().info(str(['MCL: propose: ', np.round((t_propose-t)/t_total, 2), 'motion:', np.round((t_motion-t_propose)/t_total, 2), \
                  'sensor:', np.round((t_sensor-t_motion)/t_total, 2), 'norm:', np.round((t_norm-t_sensor)/t_total, 2)]))

        # save the particles
        self.particles = proposal_distribution
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
        cosines = np.cos(self.particles[:,2])
        sines = np.sin(self.particles[:,2])
        self.local_deltas[:,0] = cosines*action[0] - sines*action[1]
        self.local_deltas[:,1] = sines*action[0] + cosines*action[1]
        self.local_deltas[:,2] = action[2]
        self.particles[:,:] += self.local_deltas
        self.particles[:,2] = self.wrap_angle(self.particles[:,2])
        self.inferred_pose = self.expected_pose()

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
        '''Lookup the fixed laser->base_link transform without blocking node startup.'''
        try:
            transform = self.tf_buffer.lookup_transform(
                self.LASER_FRAME,
                self.BASE_FRAME,
                rclpy.time.Time(),
                timeout=Duration(seconds=0.0),
            )
            yaw = Utils.quaternion_to_angle(transform.transform.rotation)
            self.laser_to_base_pose = np.array([
                transform.transform.translation.x,
                transform.transform.translation.y,
                yaw,
            ])
            if self.awaiting_laser_tf:
                self.get_logger().info(
                    f'Received {self.LASER_FRAME} <- {self.BASE_FRAME} transform'
                )
                self.awaiting_laser_tf = False
            return True
        except Exception:
            if self.awaiting_laser_tf:
                self.get_logger().info(
                    f'Waiting for {self.LASER_FRAME} <- {self.BASE_FRAME} transform...'
                )
            return False

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
        if not 100 <= self.ODOM_BUFFER_SIZE <= 150:
            raise ValueError('odom_buffer_size must be between 100 and 150')
        if not 0.0 <= self.ODOM_SYNC_WAIT_SEC <= 0.2:
            raise ValueError('odom_sync_wait_sec must be between 0.0 and 0.2')
        if self.MAX_ODOM_INTERP_GAP_SEC <= 0.0:
            raise ValueError('max_odom_interp_gap_sec must be positive')
        if self.MAX_PENDING_SCAN_AGE_SEC <= 0.0:
            raise ValueError('max_pending_scan_age_sec must be positive')
        if self.MAX_UPDATE_RATE_HZ <= 0.0:
            raise ValueError('max_update_rate_hz must be positive')
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
            lookup = self.odom_buffer.wait_for_interpolated(
                scan.stamp_ns,
                timeout_sec=self.ODOM_SYNC_WAIT_SEC,
                max_gap_sec=self.MAX_ODOM_INTERP_GAP_SEC,
            )
            if self.worker_stop.is_set():
                break
            if lookup.status != LOOKUP_OK or lookup.sample is None:
                self.scan_sync_drop_count += 1
                self._log_sync_diagnostics()
                continue
            if time.monotonic() - scan.received_monotonic > \
                    self.MAX_PENDING_SCAN_AGE_SEC:
                self.scan_stale_drop_count += 1
                self._log_sync_diagnostics()
                continue

            started = time.monotonic()
            try:
                processed = self._process_synchronized_scan(
                    scan,
                    lookup.sample,
                    generation,
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

    def _process_synchronized_scan(self, scan, odom_sample, generation):
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

            odom_pose = self.odom_sample_to_pose(odom_sample)
            if previous_odom is None:
                action = np.zeros(3)
            else:
                previous_pose = self.odom_sample_to_pose(previous_odom)
                action = self.compose_pose_2d(
                    self.inverse_pose_2d(previous_pose),
                    odom_pose,
                )
            action = self.filter_low_speed_action(action, odom_sample.linear_speed)

            self.current_speed = odom_sample.linear_speed
            self.downsampled_ranges = scan.ranges
            self.last_stamp = self.nanoseconds_to_stamp(scan.stamp_ns)
            self.timer.tick()
            self.iters += 1

            prior_particles = np.copy(self.particles)
            prior_weights = np.copy(self.weights)
            prior_pose = None
            if self.inferred_pose is not None:
                prior_pose = np.copy(self.inferred_pose)

            sensor_update_ok = self.MCL(action, scan.ranges)
            if sensor_update_ok:
                candidate_pose = self.expected_pose()
                if self.POSE_JUMP_GUARD_ENABLED and prior_pose is not None:
                    predicted_pose = self.compose_pose_2d(prior_pose, action)
                    reject, correction_xy, correction_yaw = self.should_reject_pose_update(
                        predicted_pose, candidate_pose)
                    if (reject and self.MAX_CONSECUTIVE_JUMP_REJECTS > 0
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
                else:
                    self.inferred_pose = candidate_pose
            else:
                self.get_logger().warn(
                    'PF sensor update produced invalid weights; '
                    'keeping odom-propagated pose')
                self.restore_motion_only_pose(
                    prior_particles, prior_weights, action)

            covariance = self.pose_covariance_6x6()
            self.set_localization_transform(
                self.inferred_pose,
                odom_sample,
                covariance,
            )
            with self.filter_reference_lock:
                self.previous_filter_odom = odom_sample

        self.visualize()
        return True

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
            self.filter_generation += 1
        if clear_transform:
            with self.transform_lock:
                self.map_to_odom = None
                self.map_to_odom_updated_monotonic = 0.0
                self.localized_covariance = np.zeros((6, 6))
            self.localization_output_stale_active = False

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
        self.get_logger().info(
            'PF sync: %.1fHz, compute %.1fms, scan turnaround %.1fms, '
            'correction-age %.0fms, CPU-core %.0f%%, scans %d/%d, '
            'latest-overwrite %d (%.0f%%), '
            'sync/stale/order-drop %d/%d/%d, worker-error %d, '
            'odom-buffer %d/%d' % (
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
