#!/usr/bin/env python3
"""gym 2번째 차량(opp_racecar)을 조종하는 상대 드라이버 (시뮬레이션 전용).

sim_race.launch.py가 num_agent=2로 gym을 띄우면 상대 차량은 맵 구석
'주차장'(park pose, 레이스라인에서 ~49m — ego LiDAR 사거리 밖)에서 시작한다.
RViz '2D Goal Pose' 클릭 = gym_bridge가 상대를 그 위치로 텔레포트 = 스폰.
이 노드는 /opp_racecar/odom만 보고 주차장을 벗어났는지로 활성 여부를 판단한다.

주행 모드 (/sim/opp_mode, std_msgs/String — 패널 콤보박스):
  - path   : 레이스라인 pure pursuit (기본)
  - ftg    : /opp_scan follow-the-gap (상대 LiDAR 사용)
  - manual : /opp_cmd_vel (Twist) 수동 조종
             ros2 run teleop_twist_keyboard teleop_twist_keyboard \
               --ros-args -r cmd_vel:=/opp_cmd_vel

패널 연동 토픽:
  /sim/opp_speed_delta (Float32) : 최고속도 ±delta [m/s] (모든 모드 공통 상한)
  /sim/remove_opponent (Empty)   : 주차장으로 텔레포트 = 제거
  /sim/opp_lidar_enable (Bool)   : false면 FTG가 스캔을 못 쓰므로 정지
  /sim/opp_status (String)       : 패널 표시용 상태 문자열 (2 Hz)

perception 주입:
  활성 + tracking_merge 모드(기본)일 때 ground-truth 관측을
  /perception/opponent (ObstacleArray, is_static=false)로 발행한다.
  /sim/perception_mode가 lidar_overlay면 발행을 멈춘다 — 상대는 ego의
  실제 /scan에 물리적으로 찍히므로 perception 팀 파이프라인 테스트용.
"""
import math

import numpy as np
import rclpy
from rclpy.node import Node

from ackermann_msgs.msg import AckermannDriveStamped
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, Empty, Float32, String

from f1tenth_planning_msgs.msg import Obstacle, ObstacleArray, WpntArray

import pitwall


class OpponentDriverNode(Node):
    def __init__(self):
        super().__init__('opponent_driver_node')
        pitwall.init(self)

        self.declare_parameter('opponent/mode', 'path')          # path | ftg | manual
        self.declare_parameter('opponent/max_speed_mps', 7.0)    # ± 버튼으로 조절되는 상한
        self.declare_parameter('opponent/speed_scale', 0.255)    # path: raceline 속도 배율
        self.declare_parameter('opponent/size_m', 0.3)
        self.declare_parameter('opponent/drive_rate_hz', 40.0)
        self.declare_parameter('opponent/wheelbase_m', 0.33)
        self.declare_parameter('opponent/max_steer_rad', 0.3578)
        # sim_race.launch.py 가 PARK_POSES[map] 로 항상 덮어쓴다. 아래 기본값은
        # 단독 실행용이며, 옛 Spielberg 좌표(-79.81/-31.26)는 현행 기본 맵
        # my_track0807 과 무관해서 제거했다.
        self.declare_parameter('opponent/park_x', -12.0)
        self.declare_parameter('opponent/park_y', 8.5)
        self.declare_parameter('opponent/park_radius_m', 5.0)    # 이 안이면 '주차됨'
        # FTG 파라미터
        self.declare_parameter('opponent/ftg_max_range_m', 8.0)
        self.declare_parameter('opponent/ftg_bubble_m', 0.3)
        self.declare_parameter('opponent/ftg_gap_min_m', 1.5)
        gp = lambda n: self.get_parameter(n).value
        self.mode = gp('opponent/mode')
        self.max_speed = gp('opponent/max_speed_mps')
        self.speed_scale = gp('opponent/speed_scale')
        self.size = gp('opponent/size_m')
        self.rate = gp('opponent/drive_rate_hz')
        self.wheelbase = gp('opponent/wheelbase_m')
        self.max_steer = gp('opponent/max_steer_rad')
        self.park_xy = (gp('opponent/park_x'), gp('opponent/park_y'))
        self.park_radius = gp('opponent/park_radius_m')
        self.ftg_max_range = gp('opponent/ftg_max_range_m')
        self.ftg_bubble = gp('opponent/ftg_bubble_m')
        self.ftg_gap_min = gp('opponent/ftg_gap_min_m')

        self.wpts = None            # [s, x, y, psi, vx]
        self.track_len = 0.0
        self.ds = 0.1
        self.odom = None
        self.scan = None
        self.scan_stamp = None
        self.manual_cmd = None      # (speed, steer)
        self.manual_stamp = None
        self.lidar_enable = True
        self.perception_merge = True   # tracking_merge=True | lidar_overlay=False
        self.was_active = False

        self.create_subscription(WpntArray, '/global_waypoints', self.cb_gb, 10)
        self.create_subscription(Odometry, '/opp_racecar/odom', self.cb_odom, 10)
        self.create_subscription(LaserScan, '/opp_scan', self.cb_scan, 10)
        self.create_subscription(Twist, '/opp_cmd_vel', self.cb_manual, 10)
        self.create_subscription(String, '/sim/opp_mode', self.cb_mode, 10)
        self.create_subscription(Float32, '/sim/opp_speed_delta', self.cb_delta, 10)
        self.create_subscription(Empty, '/sim/remove_opponent', self.cb_remove, 10)
        self.create_subscription(Bool, '/sim/opp_lidar_enable', self.cb_lidar, 10)
        self.create_subscription(String, '/sim/perception_mode', self.cb_pmode, 10)

        self.pub_drive = self.create_publisher(AckermannDriveStamped, '/opp_drive', 10)
        self.pub_obs = self.create_publisher(ObstacleArray, '/perception/opponent', 10)
        self.pub_goal = self.create_publisher(PoseStamped, '/goal_pose', 10)
        self.pub_status = self.create_publisher(String, '/sim/opp_status', 10)

        self.create_timer(1.0 / self.rate, self.step)
        self.create_timer(0.5, self.publish_status)

        self.get_logger().info(
            f'Opponent driver ready: 2D Goal Pose로 스폰, mode={self.mode}, '
            f'vmax={self.max_speed} m/s')

    # ---------------- 콜백 ----------------
    def cb_gb(self, msg: WpntArray):
        self.wpts = np.array(
            [[w.s_m, w.x_m, w.y_m, w.psi_rad, w.vx_mps] for w in msg.wpnts])
        self.ds = self.wpts[1, 0] - self.wpts[0, 0]
        self.track_len = self.wpts[-1, 0] + self.ds

    def cb_odom(self, msg: Odometry):
        self.odom = msg

    def cb_scan(self, msg: LaserScan):
        self.scan = msg
        self.scan_stamp = self.get_clock().now()

    def cb_manual(self, msg: Twist):
        steer = float(np.clip(msg.angular.z * self.max_steer, -self.max_steer,
                              self.max_steer))
        self.manual_cmd = (msg.linear.x, steer)
        self.manual_stamp = self.get_clock().now()

    def cb_mode(self, msg: String):
        mode = msg.data.strip().lower()
        if mode in ('path', 'ftg', 'manual') and mode != self.mode:
            self.mode = mode
            self.get_logger().info(f'상대 주행 모드: {mode}')
            pitwall.event(f'상대 모드 변경: {mode}')

    def cb_delta(self, msg: Float32):
        self.max_speed = float(np.clip(self.max_speed + msg.data, 0.0, 10.0))
        self.get_logger().info(f'상대 최고속도: {self.max_speed:.1f} m/s')
        pitwall.event(f'상대 최고속도 {self.max_speed:.1f} m/s')

    def cb_remove(self, _msg: Empty):
        goal = PoseStamped()
        goal.header.frame_id = 'map'
        goal.header.stamp = self.get_clock().now().to_msg()
        goal.pose.position.x, goal.pose.position.y = self.park_xy
        goal.pose.orientation.w = 1.0
        self.pub_goal.publish(goal)   # gym_bridge가 주차장으로 텔레포트
        self.get_logger().info('상대 제거 (주차장으로 이동)')
        pitwall.event('상대 제거')

    def cb_lidar(self, msg: Bool):
        self.lidar_enable = bool(msg.data)

    def cb_pmode(self, msg: String):
        # 게이팅은 virtual_obstacle_node 가 담당한다. 여기서는 로그용으로만 둔다.
        self.perception_merge = (msg.data.strip().lower() != 'lidar_overlay')

    # ---------------- 상태 ----------------
    def is_active(self):
        if self.odom is None:
            return False
        p = self.odom.pose.pose.position
        return math.hypot(p.x - self.park_xy[0], p.y - self.park_xy[1]) \
            > self.park_radius

    def pose_yaw_v(self):
        p = self.odom.pose.pose.position
        q = self.odom.pose.pose.orientation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                         1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        return p.x, p.y, yaw, self.odom.twist.twist.linear.x

    # ---------------- 모드별 조종 ----------------
    def drive_path(self, cx, cy, yaw, v):
        """레이스라인 pure pursuit."""
        d2 = (self.wpts[:, 1] - cx) ** 2 + (self.wpts[:, 2] - cy) ** 2
        i = int(np.argmin(d2))
        la = float(np.clip(0.5 * abs(v), 0.8, 2.5))
        j = (i + max(1, int(la / self.ds))) % len(self.wpts)
        tx, ty = self.wpts[j, 1], self.wpts[j, 2]
        dx, dy = tx - cx, ty - cy
        y_b = -dx * math.sin(yaw) + dy * math.cos(yaw)
        L = max(math.hypot(dx, dy), 1e-3)
        steer = math.atan(self.wheelbase * 2.0 * y_b / (L * L))
        speed = min(self.wpts[i, 4] * self.speed_scale, self.max_speed)
        return speed, steer

    def drive_ftg(self):
        """follow-the-gap: 전방 ±100° 스캔에서 가장 넓은 갭 중심으로 조향."""
        if self.scan is None or not self.lidar_enable:
            return 0.0, 0.0
        if (self.get_clock().now() - self.scan_stamp).nanoseconds * 1e-9 > 0.5:
            return 0.0, 0.0
        scan = self.scan
        ranges = np.nan_to_num(np.array(scan.ranges), nan=0.0,
                               posinf=self.ftg_max_range)
        angles = scan.angle_min + np.arange(len(ranges)) * scan.angle_increment
        front = np.abs(angles) < math.radians(100)
        r = np.clip(ranges[front], 0.0, self.ftg_max_range)
        a = angles[front]
        if len(r) == 0:
            return 0.0, 0.0
        # 최근접 점 주변 safety bubble 제거
        i_min = int(np.argmin(np.where(r > 0.05, r, np.inf)))
        bubble = int(self.ftg_bubble / (max(r[i_min], 0.3) * scan.angle_increment))
        r_gap = r.copy()
        r_gap[max(0, i_min - bubble):i_min + bubble + 1] = 0.0
        # 통과 가능(free) 구간 중 가장 긴 갭
        free = r_gap > self.ftg_gap_min
        if not free.any():
            return 0.0, 0.0
        edges = np.diff(free.astype(int))
        starts = list(np.where(edges == 1)[0] + 1)
        ends = list(np.where(edges == -1)[0] + 1)
        if free[0]:
            starts.insert(0, 0)
        if free[-1]:
            ends.append(len(free))
        g0, g1 = max(zip(starts, ends), key=lambda se: se[1] - se[0])
        # 갭 안에서 가장 먼 점을 목표로
        i_best = g0 + int(np.argmax(r_gap[g0:g1]))
        steer = float(np.clip(a[i_best] * 0.8, -self.max_steer, self.max_steer))
        # 조향이 클수록·전방이 가까울수록 감속
        d_front = float(np.min(r[np.abs(a) < 0.3])) if (np.abs(a) < 0.3).any() \
            else self.ftg_max_range
        speed = self.max_speed * (1.0 - 0.6 * abs(steer) / self.max_steer)
        speed = min(speed, 0.5 * d_front)
        # 하한을 0.5 로 두면 d_front < 1.0 m 에서 위 감속식이 무엇을 내든
        # 항상 0.5 가 되어, 정작 위험한 근접 구간에서 '가까울수록 감속'이
        # 무력화된다. 아주 가까우면 실제로 멈추게 둔다.
        return max(speed, 0.0), steer

    def drive_manual(self):
        if (self.manual_cmd is None or self.manual_stamp is None or
                (self.get_clock().now() - self.manual_stamp).nanoseconds * 1e-9
                > 0.5):
            return 0.0, 0.0
        return self.manual_cmd

    # ---------------- 메인 루프 ----------------
    def step(self):
        cmd = AckermannDriveStamped()
        cmd.header.stamp = self.get_clock().now().to_msg()
        cmd.header.frame_id = 'opp_racecar/base_link'

        active = self.is_active()
        if active != self.was_active:
            pitwall.event('상대 스폰' if active else '상대 주차(제거)됨')
            self.was_active = active

        if active and self.odom is not None and self.wpts is not None:
            cx, cy, yaw, v = self.pose_yaw_v()
            if self.mode == 'path':
                speed, steer = self.drive_path(cx, cy, yaw, v)
            elif self.mode == 'ftg':
                speed, steer = self.drive_ftg()
            else:
                speed, steer = self.drive_manual()
            speed = float(np.clip(speed, -self.max_speed, self.max_speed))
            cmd.drive.speed = speed
            cmd.drive.steering_angle = float(
                np.clip(steer, -self.max_steer, self.max_steer))
            pitwall.log('opp_speed', v)

        # 주차 중에도 항상 발행 (gym은 opp_drive 최초 수신 전까지 스텝을 멈춘다)
        self.pub_drive.publish(cmd)
        self.publish_perception(active)

    def publish_perception(self, active):
        """ground-truth 관측 -> /perception/opponent (tracking merge 모드만)."""
        arr = ObstacleArray()
        arr.header.stamp = self.get_clock().now().to_msg()
        arr.header.frame_id = 'map'
        # perception 모드 게이팅은 유일한 구독자인 virtual_obstacle_node 가
        # tracking_merge 일 때만 합치는 식으로 이미 하고 있다. 여기서 또 막으면
        # 계층만 늘고, 실제로는 비대칭이었다 — virtual_obstacle 은 launch 가
        # 초기 모드를 파라미터로 넣어 주는데 이쪽은 토픽 수신 전까지 True 였다.
        if active and self.wpts is not None \
                and self.odom is not None:
            cx, cy, yaw, v = self.pose_yaw_v()
            d2 = (self.wpts[:, 1] - cx) ** 2 + (self.wpts[:, 2] - cy) ** 2
            i = int(np.argmin(d2))
            s0, wx, wy, psi = self.wpts[i, 0], self.wpts[i, 1], \
                self.wpts[i, 2], self.wpts[i, 3]
            dx, dy = cx - wx, cy - wy
            s = (s0 + dx * math.cos(psi) + dy * math.sin(psi)) % self.track_len
            d = -dx * math.sin(psi) + dy * math.cos(psi)
            half = self.size / 2.0

            obs = Obstacle()
            obs.id = 99
            obs.position.x, obs.position.y = cx, cy
            obs.s_center = s
            obs.d_center = d
            obs.s_start = (s - half) % self.track_len
            obs.s_end = (s + half) % self.track_len
            obs.d_left = d + half
            obs.d_right = d - half
            obs.vs = v * math.cos(yaw - psi)
            obs.vd = v * math.sin(yaw - psi)
            obs.size = self.size
            obs.is_static = False
            obs.is_visible = True
            arr.obstacles.append(obs)
        self.pub_obs.publish(arr)

    def publish_status(self):
        msg = String()
        if not self.is_active():
            msg.data = f'PARKED | mode={self.mode} | vmax={self.max_speed:.1f}'
        else:
            v = self.odom.twist.twist.linear.x if self.odom else 0.0
            msg.data = (f'ACTIVE | mode={self.mode} | '
                        f'v={v:.1f}/{self.max_speed:.1f} m/s')
        self.pub_status.publish(msg)


def main():
    rclpy.init()
    rclpy.spin(OpponentDriverNode())
    rclpy.shutdown()


if __name__ == '__main__':
    main()
