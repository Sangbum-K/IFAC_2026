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

# ros2 python
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node

# libraries
import numpy as np
import range_libc
import time
from threading import Lock
from particle_filter import utils as Utils

# TF
# import tf.transformations
# import tf
from tf2_ros import Buffer, TransformBroadcaster, TransformListener
import tf_transformations

# messages
from std_msgs.msg import String, Header, Float32MultiArray
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point, Pose, PoseStamped, PoseArray, Quaternion, PolygonStamped, Polygon, Point32, PoseWithCovarianceStamped, PointStamped, TransformStamped
from nav_msgs.msg import Odometry
from nav_msgs.srv import GetMap

'''
These flags indicate several variants of the sensor model. Only one of them is used at a time.
'''
VAR_NO_EVAL_SENSOR_MODEL = 0
VAR_CALC_RANGE_MANY_EVAL_SENSOR = 1
VAR_REPEAT_ANGLES_EVAL_SENSOR = 2
VAR_REPEAT_ANGLES_EVAL_SENSOR_ONE_SHOT = 3
VAR_RADIAL_CDDT_OPTIMIZATIONS = 4


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
        
        # various data containers used in the MCL algorithm
        self.MAX_RANGE_PX = None
        self.odometry_data = np.array([0.0, 0.0, 0.0])
        self.laser = None
        self.iters = 0
        self.map_info = None
        self.map_initialized = False
        self.lidar_initialized = False
        self.odom_initialized = False
        self.last_pose = None
        self.current_odom_pose = None
        self.laser_angles = None
        self.downsampled_angles = None
        self.downsampled_ranges = None
        self.range_method = None
        self.last_time = None
        self.last_stamp = None
        self.first_sensor_update = True
        self.state_lock = Lock()
        self.jump_reject_count = 0

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
            1)
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

        self.get_logger().info('Finished initializing, waiting on messages...')

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

    def publish_tf(self, pose, stamp=None):
        '''Publish a map->odom transform derived from the inferred laser pose.'''
        if stamp == None:
            stamp = self.get_clock().now().to_msg()

        if self.current_odom_pose is None:
            return

        if self.laser_to_base_pose is None and not self.lookup_laser_to_base_pose():
            return

        map_to_base = self.compose_pose_2d(pose, self.laser_to_base_pose)
        map_to_odom = self.compose_pose_2d(
            map_to_base,
            self.inverse_pose_2d(self.current_odom_pose),
        )

        t = TransformStamped()
        # header
        t.header.stamp = stamp
        t.header.frame_id = self.MAP_FRAME
        t.child_frame_id = self.ODOM_FRAME
        # translation
        t.transform.translation.x = map_to_odom[0]
        t.transform.translation.y = map_to_odom[1]
        t.transform.translation.z = 0.0
        q = tf_transformations.quaternion_from_euler(0., 0., map_to_odom[2])
        # rotation
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]
        self.pub_tf.sendTransform(t)
        # also publish odometry to facilitate getting the localization pose
        if self.PUBLISH_ODOM:
            odom = Odometry()
            odom.header.stamp = self.get_clock().now().to_msg()
            odom.header.frame_id = self.MAP_FRAME
            odom.child_frame_id = self.BASE_FRAME
            odom.pose.pose.position.x = map_to_base[0]
            odom.pose.pose.position.y = map_to_base[1]
            odom.pose.pose.orientation = Utils.angle_to_quaternion(map_to_base[2])
            cov3 = np.cov(self.particles, rowvar=False, ddof=0, aweights=self.weights)
            cov6 = np.zeros((6, 6))
            idx = [0, 1, 5]  # (x, y, theta) -> (x, y, yaw) slots of the 6x6 matrix
            for i, r in enumerate(idx):
                for j, c in enumerate(idx):
                    cov6[r, c] = cov3[i, j]
            odom.pose.covariance = cov6.flatten().tolist()
            odom.twist.twist.linear.x = self.current_speed
            self.odom_pub.publish(odom)
        
        return

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
            ps.header.stamp = self.get_clock().now().to_msg()
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
        pa.header.stamp = self.get_clock().now().to_msg()
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
        Initializes reused buffers, stores the latest laser scan, and updates
        the filter once per fresh scan.
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
        self.downsampled_ranges = downsampled_ranges
        self.lidar_initialized = True
        self.update()

    def odomCB(self, msg):
        '''
        Store deltas between consecutive odometry messages in the coordinate
        space of the car and accumulate them until the next scan update.

        Odometry data is accumulated via dead reckoning, so it is very inaccurate on its own.
        '''
        position = np.array([
            msg.pose.pose.position.x,
            msg.pose.pose.position.y])

        orientation = Utils.quaternion_to_angle(msg.pose.pose.orientation)
        pose = np.array([position[0], position[1], orientation])
        self.current_speed = msg.twist.twist.linear.x
        self.current_odom_pose = pose

        if isinstance(self.last_pose, np.ndarray):
            # changes in x,y,theta in local coordinate system of the car
            rot = Utils.rotation_matrix(-self.last_pose[2])
            delta = np.array([position - self.last_pose[0:2]]).transpose()
            local_delta = (rot*delta).transpose()

            delta_pose = np.array([
                local_delta[0,0],
                local_delta[0,1],
                self.wrap_angle(orientation - self.last_pose[2]),
            ])
            self.odometry_data = self.compose_pose_2d(self.odometry_data, delta_pose)
            self.last_pose = pose
            self.last_stamp = msg.header.stamp
            self.odom_initialized = True
        else:
            self.get_logger().info('...Received first Odometry message')
            self.last_pose = pose

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
        self.state_lock.acquire()
        self.weights = np.ones(self.MAX_PARTICLES) / float(self.MAX_PARTICLES)
        self.particles[:,0] = laser_pose[0] + np.random.normal(loc=0.0,scale=0.5,size=self.MAX_PARTICLES)
        self.particles[:,1] = laser_pose[1] + np.random.normal(loc=0.0,scale=0.5,size=self.MAX_PARTICLES)
        self.particles[:,2] = self.wrap_angle(
            laser_pose[2] + np.random.normal(loc=0.0,scale=0.4,size=self.MAX_PARTICLES))
        self.inferred_pose = np.copy(laser_pose)
        self.odometry_data[:] = 0.0
        self.state_lock.release()

    def initialize_global(self):
        '''
        Spread the particle distribution over the permissible region of the state space.
        '''
        self.get_logger().info('GLOBAL INITIALIZATION')
        # randomize over grid coordinate space
        self.state_lock.acquire()
        permissible_x, permissible_y = np.where(self.permissible_region == 1)
        indices = np.random.randint(0, len(permissible_x), size=self.MAX_PARTICLES)

        permissible_states = np.zeros((self.MAX_PARTICLES,3))
        permissible_states[:,0] = permissible_y[indices]
        permissible_states[:,1] = permissible_x[indices]
        permissible_states[:,2] = np.random.random(self.MAX_PARTICLES) * np.pi * 2.0

        Utils.map_to_world(permissible_states, self.map_info)
        self.particles = permissible_states
        self.weights[:] = 1.0 / self.MAX_PARTICLES
        self.inferred_pose = None
        self.odometry_data[:] = 0.0
        self.state_lock.release()

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
        proposal_dist[:,0] += np.random.normal(loc=0.0,scale=self.MOTION_DISPERSION_X,size=self.MAX_PARTICLES)
        proposal_dist[:,1] += np.random.normal(loc=0.0,scale=self.MOTION_DISPERSION_Y,size=self.MAX_PARTICLES)
        proposal_dist[:,2] += np.random.normal(loc=0.0,scale=self.MOTION_DISPERSION_THETA,size=self.MAX_PARTICLES)
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
        reject = (
            correction_xy > self.MAX_POSE_JUMP_TRANSLATION or
            correction_yaw > self.MAX_POSE_JUMP_YAW
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

    def update(self):
        '''
        Apply the MCL function to update particle filter state. 

        Ensures the state is correctly initialized, and acquires the state lock before proceeding.
        '''
        if self.laser_to_base_pose is None and not self.lookup_laser_to_base_pose():
            return

        if self.lidar_initialized and self.odom_initialized and self.map_initialized:
            if self.state_lock.locked():
                self.get_logger().info('Concurrency error avoided')
            else:
                self.state_lock.acquire()
                self.timer.tick()
                self.iters += 1

                t1 = time.time()
                observation = np.copy(self.downsampled_ranges).astype(np.float32)
                action = np.copy(self.odometry_data)
                self.odometry_data = np.zeros(3)
                prior_particles = np.copy(self.particles)
                prior_weights = np.copy(self.weights)
                prior_pose = None if self.inferred_pose is None else np.copy(self.inferred_pose)

                # run the MCL update algorithm
                sensor_update_ok = self.MCL(action, observation)

                if sensor_update_ok:
                    candidate_pose = self.expected_pose()
                    if self.POSE_JUMP_GUARD_ENABLED and prior_pose is not None:
                        predicted_pose = self.compose_pose_2d(prior_pose, action)
                        reject, correction_xy, correction_yaw = self.should_reject_pose_update(
                            predicted_pose, candidate_pose)
                        if reject:
                            self.jump_reject_count += 1
                            if self.jump_reject_count <= 5 or self.jump_reject_count % 20 == 0:
                                self.get_logger().warn(
                                    'Rejected PF scan update: correction %.2fm, %.1fdeg exceeds guard; '
                                    'keeping odom-propagated pose' % (
                                        correction_xy,
                                        np.degrees(correction_yaw),
                                    )
                                )
                            self.restore_motion_only_pose(prior_particles, prior_weights, action)
                        else:
                            self.inferred_pose = candidate_pose
                    else:
                        self.inferred_pose = candidate_pose
                else:
                    self.get_logger().warn(
                        'PF sensor update produced invalid weights; keeping odom-propagated pose')
                    self.restore_motion_only_pose(prior_particles, prior_weights, action)
                self.state_lock.release()
                t2 = time.time()

                # publish transformation frame based on inferred pose
                self.publish_tf(self.inferred_pose, self.last_stamp)

                # this is for tracking particle filter speed
                ips = 1.0 / (t2 - t1)
                self.smoothing.append(ips)
                if self.iters % 10 == 0:
                    self.get_logger().info(str(['iters per sec:', int(self.timer.fps()), ' possible:', int(self.smoothing.mean())]))

                self.visualize()

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
    rclpy.spin(pf)

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
