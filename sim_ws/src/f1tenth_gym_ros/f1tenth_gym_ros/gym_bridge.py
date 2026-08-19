# MIT License

# Copyright (c) 2020 Hongrui Zheng

# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:

# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.

# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import PoseWithCovarianceStamped
from geometry_msgs.msg import Twist
from geometry_msgs.msg import TransformStamped
from geometry_msgs.msg import Transform
from geometry_msgs.msg import Quaternion
from ackermann_msgs.msg import AckermannDriveStamped
from tf2_ros import TransformBroadcaster

import gym
import numpy as np
from transforms3d import euler

class GymBridge(Node):
    def __init__(self):
        super().__init__('gym_bridge')

        self.declare_parameter('ego_namespace')
        self.declare_parameter('ego_odom_topic')
        self.declare_parameter('ego_opp_odom_topic')
        self.declare_parameter('ego_scan_topic')
        self.declare_parameter('ego_drive_topic')
        self.declare_parameter('opp_namespace')
        self.declare_parameter('opp_odom_topic')
        self.declare_parameter('opp_ego_odom_topic')
        self.declare_parameter('opp_scan_topic')
        self.declare_parameter('opp_drive_topic')
        self.declare_parameter('scan_distance_to_base_link')
        self.declare_parameter('scan_fov')
        self.declare_parameter('scan_beams')
        self.declare_parameter('map_path')
        self.declare_parameter('map_img_ext')
        self.declare_parameter('num_agent')
        self.declare_parameter('sx')
        self.declare_parameter('sy')
        self.declare_parameter('stheta')
        self.declare_parameter('sx1')
        self.declare_parameter('sy1')
        self.declare_parameter('stheta1')
        self.declare_parameter('kb_teleop')

        # ⚠ 차량 물리 파라미터 (IFAC_ws 로컬 추가, 2026-08-11).
        # upstream 은 f110_gym 기본값(mu 1.0489 / a_max 9.51 / s_max 0.4189 /
        # v_max 20.0)을 그대로 쓴다. 그 값은 실차보다 훨씬 관대해서 — 마찰이
        # 11% 높고 가속 한계가 2.1배다 — 시뮬에서는 실차가 슬립하는 코너를
        # 그대로 통과해 버린다. 즉 raceline 이 실차에서 성립하는지를 시뮬로
        # 검증할 수 없다.
        # sim_race.launch.py vehicle:=real_car(REAL_CAR_VEHICLE)가 실측값을
        # 넣어 준다 — 파라미터 이름·기본값은 그 dict 와 반드시 짝이 맞아야 한다.
        # 기본값은 upstream 과 같게 둬서 vehicle:=gym_default 는 종전 동작 유지.
        self.declare_parameter('vehicle.mu', 1.0489)
        self.declare_parameter('vehicle.a_max', 9.51)
        self.declare_parameter('vehicle.s_min', -0.4189)
        self.declare_parameter('vehicle.s_max', 0.4189)
        self.declare_parameter('vehicle.v_min', -5.0)
        self.declare_parameter('vehicle.v_max', 20.0)
        self.declare_parameter('vehicle.width', 0.31)
        self.declare_parameter('vehicle.length', 0.58)
        # 센서(스캔) 발행 주기 [Hz]. 기본 250.0 은 upstream 원래 동작(스캔도
        # odom/TF 와 같은 250Hz 단일 타이머) 그대로다. vehicle:=real_car 가
        # 40.0(실차 Hokuyo UST-10LX)으로 덮으면 스캔이 별도 타이머로 갈라지고
        # 아래 scan_timer_callback 의 10ms 격자 재현이 켜진다 (근거는 그쪽 주석).
        self.declare_parameter('sensor_publish_rate_hz', 250.0)

        # check num_agents
        num_agents = self.get_parameter('num_agent').value
        if num_agents < 1 or num_agents > 2:
            raise ValueError('num_agents should be either 1 or 2.')
        elif type(num_agents) != int:
            raise ValueError('num_agents should be an int.')

        # 차량 물리. f110_gym 기본 dict 를 통째로 넘겨야 하므로(부분 갱신 불가)
        # 위 8개만 파라미터로 받고 나머지는 실측이 없어 gym 기본값을 그대로 둔다
        # (lf+lr=0.3302 = 실차 wheelbase 0.33, sv_max 3.2 = hardware max_servo_speed).
        # s_min/s_max 를 독립적으로 받는다 — 실차 서보는 한쪽이 기계 스톱에
        # 먼저 닿지만(TUNING_STATUS.md §1), 그 비대칭은 조향 캘리브레이션
        # (vesc.yaml gain/offset)이 흡수하고 플래닝·컨트롤은 대칭 한계를 쓰므로
        # sim_race.launch.py REAL_CAR_VEHICLE 은 대칭값 한 쌍을 넣는다.
        vehicle_params = {
            'mu': float(self.get_parameter('vehicle.mu').value),
            'C_Sf': 4.718, 'C_Sr': 5.4562,
            'lf': 0.15875, 'lr': 0.17145, 'h': 0.074, 'm': 3.74, 'I': 0.04712,
            's_min': float(self.get_parameter('vehicle.s_min').value),
            's_max': float(self.get_parameter('vehicle.s_max').value),
            'sv_min': -3.2, 'sv_max': 3.2,
            'v_switch': 7.319,
            'a_max': float(self.get_parameter('vehicle.a_max').value),
            'v_min': float(self.get_parameter('vehicle.v_min').value),
            'v_max': float(self.get_parameter('vehicle.v_max').value),
            'width': float(self.get_parameter('vehicle.width').value),
            'length': float(self.get_parameter('vehicle.length').value),
        }
        self.get_logger().info(
            f"차량 물리: mu={vehicle_params['mu']} a_max={vehicle_params['a_max']} "
            f"s_max={vehicle_params['s_max']} v_max={vehicle_params['v_max']} "
            f"width={vehicle_params['width']} length={vehicle_params['length']}")

        # env backend
        self.env = gym.make('f110_gym:f110-v0',
                            map=self.get_parameter('map_path').value,
                            map_ext=self.get_parameter('map_img_ext').value,
                            num_agents=num_agents,
                            params=vehicle_params,
                            lidar_dist=self.get_parameter("scan_distance_to_base_link").value
                            )

        sx = self.get_parameter('sx').value
        sy = self.get_parameter('sy').value
        stheta = self.get_parameter('stheta').value
        self.ego_pose = [sx, sy, stheta]
        self.ego_speed = [0.0, 0.0, 0.0]
        self.ego_requested_speed = 0.0
        self.ego_steer = 0.0
        self.ego_collision = False
        ego_scan_topic = self.get_parameter('ego_scan_topic').value
        ego_drive_topic = self.get_parameter('ego_drive_topic').value
        scan_fov = self.get_parameter('scan_fov').value
        scan_beams = self.get_parameter('scan_beams').value
        self.angle_min = -scan_fov / 2.
        self.angle_max = scan_fov / 2.
        # f110_gym 은 빔을 fov/(num_beams-1) 간격으로 쏜다
        # (laser_models.py ScanSimulator2D.angle_increment). 여기서 fov/num_beams
        # 로 publish 하면 **시뮬 내부의 실제 빔 방향과 어긋난 값**이 나간다:
        # 구독자는 angle_min + i*angle_inc 로 점을 복원하므로 마지막 빔에서
        # 한 칸(≈0.25°) 틀어지고, 5 m 거리에서 2.2 cm 어긋난다.
        # 실차 Hokuyo 도 fov/(beams-1) 이다 (run_0817_194156 /scan 실측:
        # 1081 빔 · angle_increment 0.00436332 = 4.7124/1080).
        self.angle_inc = scan_fov / (scan_beams - 1)
        self.ego_namespace = self.get_parameter('ego_namespace').value
        ego_odom_topic = self.ego_namespace + '/' + self.get_parameter('ego_odom_topic').value
        self.scan_distance_to_base_link = self.get_parameter('scan_distance_to_base_link').value
        
        if num_agents == 2:
            self.has_opp = True
            self.opp_namespace = self.get_parameter('opp_namespace').value
            sx1 = self.get_parameter('sx1').value
            sy1 = self.get_parameter('sy1').value
            stheta1 = self.get_parameter('stheta1').value
            self.opp_pose = [sx1, sy1, stheta1]
            self.opp_speed = [0.0, 0.0, 0.0]
            self.opp_requested_speed = 0.0
            self.opp_steer = 0.0
            self.opp_collision = False
            self.obs, _ , self.done, _ = self.env.reset(np.array([[sx, sy, stheta], [sx1, sy1, stheta1]]))
            self.ego_scan = list(self.obs['scans'][0])
            self.opp_scan = list(self.obs['scans'][1])

            opp_scan_topic = self.get_parameter('opp_scan_topic').value
            opp_odom_topic = self.opp_namespace + '/' + self.get_parameter('opp_odom_topic').value
            opp_drive_topic = self.get_parameter('opp_drive_topic').value

            ego_opp_odom_topic = self.ego_namespace + '/' + self.get_parameter('ego_opp_odom_topic').value
            opp_ego_odom_topic = self.opp_namespace + '/' + self.get_parameter('opp_ego_odom_topic').value
        else:
            self.has_opp = False
            self.obs, _ , self.done, _ = self.env.reset(np.array([[sx, sy, stheta]]))
            self.ego_scan = list(self.obs['scans'][0])

        # sim physical step timer
        self.drive_timer = self.create_timer(0.01, self.drive_timer_callback)

        sensor_rate = float(self.get_parameter('sensor_publish_rate_hz').value)
        if sensor_rate <= 0.0:
            raise ValueError('sensor_publish_rate_hz must be > 0')
        # 기본(250Hz)은 upstream 동작 그대로 — 스캔·odom·TF 를 한 타이머에서 쏜다.
        # 100Hz 미만이 오면(= vehicle:=real_car 의 40Hz) 스캔을 별도 타이머로
        # 갈라 실차 LiDAR 타이밍을 재현한다. 아래 두 갈래가 그 차이다.
        self._split_scan_timer = sensor_rate < 100.0

        if not self._split_scan_timer:
            self.timer = self.create_timer(1.0 / sensor_rate, self.timer_callback)
            self.scan_timer = None
        else:
            # topic publishing timer (odom + TF).
            # ⚠ upstream 은 0.004s(250Hz)다. 실차 odom->base_link 는 vesc_to_odom 이
            # ~47Hz 로 낸다(planning real_car.yaml tf_fixed_frame 주석). 250Hz 는
            # 실차보다 5배 신선한 TF 를 주는 데다, 파이썬으로 매 틱 odom 1개 +
            # TF 4개를 만들어 쏘느라 CPU 를 크게 먹는다 — gym·PF·인지·플래닝이 한
            # 머신에 같이 뜨는 시뮬에서 그 낭비가 PF 를 굶겨 map->odom TF 를
            # 늦추고, 결국 obstacle_detector 의 tf_max_age_sec(0.30) 게이트에
            # 걸려 인지가 멈춘다. 실측: 250Hz 에서 PF 4.8Hz(detector 전 스캔 스킵)
            # -> 50Hz 에서 PF 14.4Hz(실차 ~15Hz, detector 스킵 0).
            self.timer = self.create_timer(0.02, self.timer_callback)

            # ⚠ 스캔은 실차 LiDAR 타임스탬프 분포를 재현한다.
            # 실차 /scan header stamp 간격 실측 (run_0806_152821, 2525스캔):
            #   평균 25.05ms = 39.9Hz, std 5.07ms — 20ms 와 30ms 의 이봉분포이고
            #   거의 교대로 나온다 (연속-동일 확률 0.127). UST-10LX 는 40Hz(25ms)
            #   인데 타임스탬프가 10ms 격자에 걸려 20/30 으로 갈라지는 것으로 보인다.
            #
            # 이 지터가 detector 출력 주기를 결정한다. process_rate_hz 20 =
            # min_period 50ms 인데, 20+30=50.0 이 그 경계에 정확히 걸려서
            # 서브밀리초 노이즈가 통과/탈락을 가른다:
            #   실측 stamp 열에 50ms 스로틀   -> 15.61Hz (dt p50 69.3 / p95 80.1ms)
            #   균일 25ms 입력에 같은 스로틀  -> 매번 통과, ~20Hz
            # 즉 균일 40Hz 로 쏘면 시뮬 detector 가 실차보다 28% 빨라진다.
            # (13b412b 가 차량에서 잰 15.49Hz / p50 69.5 / p95 80.0 과 재현 일치.)
            #
            # 그래서 격자 틱(1/2 주기) 위에서 2틱/3틱을 교대로 낸다. 서브밀리초
            # 노이즈는 파이썬 타이머 지터가 자연히 준다 (σ 0.1~0.5ms 면 재현 확인).
            tick = 1.0 / (sensor_rate * 2.5)   # 40Hz -> 10ms 격자
            self._scan_steps = (2, 3)          # 2틱/3틱 교대 = 20ms/30ms (평균 25ms)
            self._scan_phase = 0
            self._scan_countdown = self._scan_steps[0]
            self.scan_timer = self.create_timer(tick, self.scan_timer_callback)
            self.get_logger().info(
                f'스캔 타이머 분리: {sensor_rate:.1f}Hz 목표, '
                f'{tick*1000:.0f}ms 격자에서 20/30ms 교대 (실차 LiDAR 재현)')

        # transform broadcaster
        self.br = TransformBroadcaster(self)

        # publishers
        self.ego_scan_pub = self.create_publisher(LaserScan, ego_scan_topic, 10)
        self.ego_odom_pub = self.create_publisher(Odometry, ego_odom_topic, 10)
        self.ego_drive_published = False
        if num_agents == 2:
            self.opp_scan_pub = self.create_publisher(LaserScan, opp_scan_topic, 10)
            self.ego_opp_odom_pub = self.create_publisher(Odometry, ego_opp_odom_topic, 10)
            self.opp_odom_pub = self.create_publisher(Odometry, opp_odom_topic, 10)
            self.opp_ego_odom_pub = self.create_publisher(Odometry, opp_ego_odom_topic, 10)
            self.opp_drive_published = False

        # subscribers
        self.ego_drive_sub = self.create_subscription(
            AckermannDriveStamped,
            ego_drive_topic,
            self.drive_callback,
            10)
        self.ego_reset_sub = self.create_subscription(
            PoseWithCovarianceStamped,
            '/initialpose',
            self.ego_reset_callback,
            10)
        if num_agents == 2:
            self.opp_drive_sub = self.create_subscription(
                AckermannDriveStamped,
                opp_drive_topic,
                self.opp_drive_callback,
                10)
            self.opp_reset_sub = self.create_subscription(
                PoseStamped,
                '/goal_pose',
                self.opp_reset_callback,
                10)

        if self.get_parameter('kb_teleop').value:
            self.teleop_sub = self.create_subscription(
                Twist,
                '/cmd_vel',
                self.teleop_callback,
                10)


    def drive_callback(self, drive_msg):
        self.ego_requested_speed = drive_msg.drive.speed
        self.ego_steer = drive_msg.drive.steering_angle
        self.ego_drive_published = True

    def opp_drive_callback(self, drive_msg):
        self.opp_requested_speed = drive_msg.drive.speed
        self.opp_steer = drive_msg.drive.steering_angle
        self.opp_drive_published = True

    def ego_reset_callback(self, pose_msg):
        rx = pose_msg.pose.pose.position.x
        ry = pose_msg.pose.pose.position.y
        rqx = pose_msg.pose.pose.orientation.x
        rqy = pose_msg.pose.pose.orientation.y
        rqz = pose_msg.pose.pose.orientation.z
        rqw = pose_msg.pose.pose.orientation.w
        _, _, rtheta = euler.quat2euler([rqw, rqx, rqy, rqz], axes='sxyz')
        if self.has_opp:
            opp_pose = [self.obs['poses_x'][1], self.obs['poses_y'][1], self.obs['poses_theta'][1]]
            self.obs, _ , self.done, _ = self.env.reset(np.array([[rx, ry, rtheta], opp_pose]))
        else:
            self.obs, _ , self.done, _ = self.env.reset(np.array([[rx, ry, rtheta]]))
        self._update_sim_state()

    def opp_reset_callback(self, pose_msg):
        if self.has_opp:
            rx = pose_msg.pose.position.x
            ry = pose_msg.pose.position.y
            rqx = pose_msg.pose.orientation.x
            rqy = pose_msg.pose.orientation.y
            rqz = pose_msg.pose.orientation.z
            rqw = pose_msg.pose.orientation.w
            _, _, rtheta = euler.quat2euler([rqw, rqx, rqy, rqz], axes='sxyz')
            self.obs, _ , self.done, _ = self.env.reset(np.array([list(self.ego_pose), [rx, ry, rtheta]]))
            self._update_sim_state()

    def teleop_callback(self, twist_msg):
        if not self.ego_drive_published:
            self.ego_drive_published = True

        self.ego_requested_speed = twist_msg.linear.x

        if twist_msg.angular.z > 0.0:
            self.ego_steer = 0.3
        elif twist_msg.angular.z < 0.0:
            self.ego_steer = -0.3
        else:
            self.ego_steer = 0.0

    def drive_timer_callback(self):
        if self.ego_drive_published and not self.has_opp:
            self.obs, _, self.done, _ = self.env.step(np.array([[self.ego_steer, self.ego_requested_speed]]))
        elif self.ego_drive_published and self.has_opp and self.opp_drive_published:
            self.obs, _, self.done, _ = self.env.step(np.array([[self.ego_steer, self.ego_requested_speed], [self.opp_steer, self.opp_requested_speed]]))
        self._update_sim_state()

    def scan_timer_callback(self):
        """실차 LiDAR 와 같은 20/30ms 교대로 스캔만 발행 (위 scan_timer 주석 참고)."""
        # 격자 틱을 세어 2틱(20ms), 3틱(30ms)을 번갈아 낸다 (평균 25ms = 39.9Hz).
        self._scan_countdown -= 1
        if self._scan_countdown > 0:
            return
        self._scan_phase ^= 1
        self._scan_countdown = self._scan_steps[self._scan_phase]
        self._publish_scans(self.get_clock().now().to_msg())

    def _publish_scans(self, ts):
        scan = LaserScan()
        scan.header.stamp = ts
        scan.header.frame_id = self.ego_namespace + '/laser'
        scan.angle_min = self.angle_min
        scan.angle_max = self.angle_max
        scan.angle_increment = self.angle_inc
        scan.range_min = 0.
        scan.range_max = 30.
        scan.ranges = self.ego_scan
        self.ego_scan_pub.publish(scan)

        if self.has_opp:
            opp_scan = LaserScan()
            opp_scan.header.stamp = ts
            opp_scan.header.frame_id = self.opp_namespace + '/laser'
            opp_scan.angle_min = self.angle_min
            opp_scan.angle_max = self.angle_max
            opp_scan.angle_increment = self.angle_inc
            opp_scan.range_min = 0.
            opp_scan.range_max = 30.
            opp_scan.ranges = self.opp_scan
            self.opp_scan_pub.publish(opp_scan)

    def timer_callback(self):
        ts = self.get_clock().now().to_msg()

        # 스캔 타이머를 분리하지 않은 기본(upstream) 경로에서는 여기서 함께 쏜다.
        if not self._split_scan_timer:
            self._publish_scans(ts)

        # pub tf
        self._publish_odom(ts)
        self._publish_transforms(ts)
        self._publish_laser_transforms(ts)
        self._publish_wheel_transforms(ts)

    def _update_sim_state(self):
        self.ego_scan = list(self.obs['scans'][0])
        if self.has_opp:
            self.opp_scan = list(self.obs['scans'][1])
            self.opp_pose[0] = self.obs['poses_x'][1]
            self.opp_pose[1] = self.obs['poses_y'][1]
            self.opp_pose[2] = self.obs['poses_theta'][1]
            self.opp_speed[0] = self.obs['linear_vels_x'][1]
            self.opp_speed[1] = self.obs['linear_vels_y'][1]
            self.opp_speed[2] = self.obs['ang_vels_z'][1]

        self.ego_pose[0] = self.obs['poses_x'][0]
        self.ego_pose[1] = self.obs['poses_y'][0]
        self.ego_pose[2] = self.obs['poses_theta'][0]
        self.ego_speed[0] = self.obs['linear_vels_x'][0]
        self.ego_speed[1] = self.obs['linear_vels_y'][0]
        self.ego_speed[2] = self.obs['ang_vels_z'][0]

        

    def _publish_odom(self, ts):
        ego_odom = Odometry()
        ego_odom.header.stamp = ts
        ego_odom.header.frame_id = 'map'
        ego_odom.child_frame_id = self.ego_namespace + '/base_link'
        ego_odom.pose.pose.position.x = self.ego_pose[0]
        ego_odom.pose.pose.position.y = self.ego_pose[1]
        ego_quat = euler.euler2quat(0., 0., self.ego_pose[2], axes='sxyz')
        ego_odom.pose.pose.orientation.x = ego_quat[1]
        ego_odom.pose.pose.orientation.y = ego_quat[2]
        ego_odom.pose.pose.orientation.z = ego_quat[3]
        ego_odom.pose.pose.orientation.w = ego_quat[0]
        ego_odom.twist.twist.linear.x = self.ego_speed[0]
        ego_odom.twist.twist.linear.y = self.ego_speed[1]
        ego_odom.twist.twist.angular.z = self.ego_speed[2]
        self.ego_odom_pub.publish(ego_odom)

        if self.has_opp:
            opp_odom = Odometry()
            opp_odom.header.stamp = ts
            opp_odom.header.frame_id = 'map'
            opp_odom.child_frame_id = self.opp_namespace + '/base_link'
            opp_odom.pose.pose.position.x = self.opp_pose[0]
            opp_odom.pose.pose.position.y = self.opp_pose[1]
            opp_quat = euler.euler2quat(0., 0., self.opp_pose[2], axes='sxyz')
            opp_odom.pose.pose.orientation.x = opp_quat[1]
            opp_odom.pose.pose.orientation.y = opp_quat[2]
            opp_odom.pose.pose.orientation.z = opp_quat[3]
            opp_odom.pose.pose.orientation.w = opp_quat[0]
            opp_odom.twist.twist.linear.x = self.opp_speed[0]
            opp_odom.twist.twist.linear.y = self.opp_speed[1]
            opp_odom.twist.twist.angular.z = self.opp_speed[2]
            self.opp_odom_pub.publish(opp_odom)
            self.opp_ego_odom_pub.publish(ego_odom)
            self.ego_opp_odom_pub.publish(opp_odom)

    def _publish_transforms(self, ts):
        ego_t = Transform()
        ego_t.translation.x = self.ego_pose[0]
        ego_t.translation.y = self.ego_pose[1]
        ego_t.translation.z = 0.0
        ego_quat = euler.euler2quat(0.0, 0.0, self.ego_pose[2], axes='sxyz')
        ego_t.rotation.x = ego_quat[1]
        ego_t.rotation.y = ego_quat[2]
        ego_t.rotation.z = ego_quat[3]
        ego_t.rotation.w = ego_quat[0]

        ego_ts = TransformStamped()
        ego_ts.transform = ego_t
        ego_ts.header.stamp = ts
        ego_ts.header.frame_id = 'map'
        ego_ts.child_frame_id = self.ego_namespace + '/base_link'
        self.br.sendTransform(ego_ts)

        if self.has_opp:
            opp_t = Transform()
            opp_t.translation.x = self.opp_pose[0]
            opp_t.translation.y = self.opp_pose[1]
            opp_t.translation.z = 0.0
            opp_quat = euler.euler2quat(0.0, 0.0, self.opp_pose[2], axes='sxyz')
            opp_t.rotation.x = opp_quat[1]
            opp_t.rotation.y = opp_quat[2]
            opp_t.rotation.z = opp_quat[3]
            opp_t.rotation.w = opp_quat[0]

            opp_ts = TransformStamped()
            opp_ts.transform = opp_t
            opp_ts.header.stamp = ts
            opp_ts.header.frame_id = 'map'
            opp_ts.child_frame_id = self.opp_namespace + '/base_link'
            self.br.sendTransform(opp_ts)

    def _publish_wheel_transforms(self, ts):
        ego_wheel_ts = TransformStamped()
        ego_wheel_quat = euler.euler2quat(0., 0., self.ego_steer, axes='sxyz')
        ego_wheel_ts.transform.rotation.x = ego_wheel_quat[1]
        ego_wheel_ts.transform.rotation.y = ego_wheel_quat[2]
        ego_wheel_ts.transform.rotation.z = ego_wheel_quat[3]
        ego_wheel_ts.transform.rotation.w = ego_wheel_quat[0]
        ego_wheel_ts.header.stamp = ts
        ego_wheel_ts.header.frame_id = self.ego_namespace + '/front_left_hinge'
        ego_wheel_ts.child_frame_id = self.ego_namespace + '/front_left_wheel'
        self.br.sendTransform(ego_wheel_ts)
        ego_wheel_ts.header.frame_id = self.ego_namespace + '/front_right_hinge'
        ego_wheel_ts.child_frame_id = self.ego_namespace + '/front_right_wheel'
        self.br.sendTransform(ego_wheel_ts)

        if self.has_opp:
            opp_wheel_ts = TransformStamped()
            opp_wheel_quat = euler.euler2quat(0., 0., self.opp_steer, axes='sxyz')
            opp_wheel_ts.transform.rotation.x = opp_wheel_quat[1]
            opp_wheel_ts.transform.rotation.y = opp_wheel_quat[2]
            opp_wheel_ts.transform.rotation.z = opp_wheel_quat[3]
            opp_wheel_ts.transform.rotation.w = opp_wheel_quat[0]
            opp_wheel_ts.header.stamp = ts
            opp_wheel_ts.header.frame_id = self.opp_namespace + '/front_left_hinge'
            opp_wheel_ts.child_frame_id = self.opp_namespace + '/front_left_wheel'
            self.br.sendTransform(opp_wheel_ts)
            opp_wheel_ts.header.frame_id = self.opp_namespace + '/front_right_hinge'
            opp_wheel_ts.child_frame_id = self.opp_namespace + '/front_right_wheel'
            self.br.sendTransform(opp_wheel_ts)

    def _publish_laser_transforms(self, ts):
        ego_scan_ts = TransformStamped()
        ego_scan_ts.transform.translation.x = self.scan_distance_to_base_link
        # ego_scan_ts.transform.translation.z = 0.04+0.1+0.025
        ego_scan_ts.transform.rotation.w = 1.
        ego_scan_ts.header.stamp = ts
        ego_scan_ts.header.frame_id = self.ego_namespace + '/base_link'
        ego_scan_ts.child_frame_id = self.ego_namespace + '/laser'
        self.br.sendTransform(ego_scan_ts)

        if self.has_opp:
            opp_scan_ts = TransformStamped()
            opp_scan_ts.transform.translation.x = self.scan_distance_to_base_link
            opp_scan_ts.transform.rotation.w = 1.
            opp_scan_ts.header.stamp = ts
            opp_scan_ts.header.frame_id = self.opp_namespace + '/base_link'
            opp_scan_ts.child_frame_id = self.opp_namespace + '/laser'
            self.br.sendTransform(opp_scan_ts)

def main(args=None):
    rclpy.init(args=args)
    gym_bridge = GymBridge()
    rclpy.spin(gym_bridge)

if __name__ == '__main__':
    main()