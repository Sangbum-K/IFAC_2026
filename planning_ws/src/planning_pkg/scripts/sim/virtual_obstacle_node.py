#!/usr/bin/env python3
"""RViz 'Publish Point' 가상 정적 장애물 + Perception Injection (시뮬 전용).

unicorn(HMCL) 튜토리얼의 virtual_perception 역할:
  /clicked_point 클릭 -> Frenet 변환 -> /perception/obstacles (10 Hz 상시 발행)
기존 장애물 근처(remove_radius 이내)를 다시 클릭하면 그 장애물이 제거되고,
/sim/clear_obstacles (Empty, 패널 버튼)로 전체 삭제된다.
장애물이 없어도 빈 배열을 계속 발행한다 (spliner의 stale 타임아웃과 연동).

perception 주입 모드 (/sim/perception_mode, 패널 콤보박스):
  - tracking_merge (기본): ground-truth를 /perception/obstacles로 직접 주입
    (플래너 단독 검증 — 상대 관측 /perception/opponent도 여기서 합쳐짐)
  - lidar_overlay: /perception/obstacles는 빈 배열. 대신 가상 장애물의
    합성 반사점을 ego /scan에 삽입해 /scan_overlay로 재발행한다 —
    perception 팀 파이프라인이 /scan_overlay를 구독하면 탐지->추적->플래너
    전체 체인을 시뮬에서 검증할 수 있다. (상대 차량은 gym 2-agent 실차라
    /scan에 이미 물리적으로 찍힌다 — 삽입 불필요.)

/sim/ego_lidar_enable (Bool, 패널 체크박스): false면 overlay 발행 중단.

perception 팀 노드가 준비되면 이 노드 대신 실제 perception이
/perception/obstacles를 발행하게 된다.
"""
import math

import numpy as np
import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PointStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, Empty, String
from visualization_msgs.msg import Marker, MarkerArray

from f1tenth_planning_msgs.msg import Obstacle, ObstacleArray, WpntArray

try:
    # 설치 트리(copy-install): pitwall.py 가 노드 옆(lib/planning_pkg/)에 깔린다.
    import pitwall
except ModuleNotFoundError:
    # colcon --symlink-install: 설치된 노드가 src 원본으로 가는 심볼릭 링크라
    # 파이썬이 sys.path[0] 을 realpath(=scripts/<하위>/)로 잡는다. 그러면
    # 같이 설치된 pitwall.py 가 경로에 없다 -> 형제 폴더 scripts/lib 를 얹는다.
    import sys
    from pathlib import Path
    sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'lib'))
    import pitwall


class VirtualObstacleNode(Node):
    def __init__(self):
        super().__init__('virtual_obstacle_node')
        pitwall.init(self)

        self.declare_parameter('virtual_obstacle/publish_rate_hz', 10.0)
        self.declare_parameter('virtual_obstacle/obstacle_size_m', 0.3)
        self.declare_parameter('virtual_obstacle/remove_radius_m', 0.5)
        self.declare_parameter('virtual_obstacle/laser_offset_m', 0.275)  # base_link -> laser
        # 시작 시 perception 주입 모드 (tracking_merge | lidar_overlay).
        # 통합 스택(env:=sim, 실제 perception 파이프라인)은 lidar_overlay로 기동한다.
        self.declare_parameter('virtual_obstacle/perception_mode', 'tracking_merge')
        rate = self.get_parameter('virtual_obstacle/publish_rate_hz').value
        self.size = self.get_parameter('virtual_obstacle/obstacle_size_m').value
        self.remove_radius = self.get_parameter('virtual_obstacle/remove_radius_m').value
        self.laser_offset = self.get_parameter('virtual_obstacle/laser_offset_m').value

        self.wpnts = None       # np.array [N, 4]: s, x, y, psi
        self.track_length = 0.0
        self.obstacles = []     # list[Obstacle]
        self.next_id = 1
        self.opponent = None    # 상대 관측 (opponent_driver_node)
        self.opponent_stamp = None
        init_mode = self.get_parameter(
            'virtual_obstacle/perception_mode').value.strip().lower()
        self.tracking_merge = (init_mode != 'lidar_overlay')  # /sim/perception_mode
        self.lidar_enable = True     # /sim/ego_lidar_enable
        self.odom = None

        self.create_subscription(WpntArray, '/global_waypoints', self.cb_gb, 10)
        self.create_subscription(PointStamped, '/clicked_point', self.cb_click, 10)
        self.create_subscription(ObstacleArray, '/perception/opponent', self.cb_opponent, 10)
        self.create_subscription(Empty, '/sim/clear_obstacles', self.cb_clear, 10)
        self.create_subscription(String, '/sim/perception_mode', self.cb_pmode, 10)
        self.create_subscription(Bool, '/sim/ego_lidar_enable', self.cb_lidar, 10)
        self.create_subscription(Odometry, '/ego_racecar/odom', self.cb_odom, 10)
        self.create_subscription(LaserScan, '/scan', self.cb_scan, 10)
        self.pub_obs = self.create_publisher(ObstacleArray, '/perception/obstacles', 10)
        self.pub_scan = self.create_publisher(LaserScan, '/scan_overlay', 10)
        self.pub_mrk = self.create_publisher(MarkerArray, '/perception/obstacle_markers', 10)
        self.create_timer(1.0 / rate, self.publish)

        self.get_logger().info(
            'Virtual obstacle node ready: RViz "Publish Point"로 장애물 추가/제거')

    def cb_gb(self, msg: WpntArray):
        self.wpnts = np.array(
            [[w.s_m, w.x_m, w.y_m, w.psi_rad] for w in msg.wpnts])
        if len(msg.wpnts) >= 2:
            spacing = msg.wpnts[1].s_m - msg.wpnts[0].s_m
            self.track_length = msg.wpnts[-1].s_m + spacing

    def cartesian_to_frenet(self, x, y):
        """frenet_frame_node와 동일한 최근접-waypoint 투영 (d>0 = 왼쪽)."""
        d2 = (self.wpnts[:, 1] - x) ** 2 + (self.wpnts[:, 2] - y) ** 2
        i = int(np.argmin(d2))
        s0, wx, wy, psi = self.wpnts[i]
        dx, dy = x - wx, y - wy
        s = (s0 + dx * math.cos(psi) + dy * math.sin(psi)) % self.track_length
        d = -dx * math.sin(psi) + dy * math.cos(psi)
        return s, d

    def cb_click(self, msg: PointStamped):
        if self.wpnts is None:
            self.get_logger().warn('global waypoints 수신 전 — 클릭 무시')
            return
        x, y = msg.point.x, msg.point.y

        # 기존 장애물 근처 클릭이면 제거
        for obs in self.obstacles:
            if math.hypot(obs.position.x - x, obs.position.y - y) < self.remove_radius:
                self.obstacles.remove(obs)
                self.get_logger().info(f'장애물 #{obs.id} 제거 ({x:.2f}, {y:.2f})')
                pitwall.event(f'장애물 #{obs.id} 제거')
                return

        s, d = self.cartesian_to_frenet(x, y)
        half = self.size / 2.0
        obs = Obstacle()
        obs.id = self.next_id
        self.next_id += 1
        obs.position.x, obs.position.y = x, y
        obs.s_center = s
        obs.d_center = d
        obs.s_start = (s - half) % self.track_length
        obs.s_end = (s + half) % self.track_length
        obs.d_left = d + half
        obs.d_right = d - half
        obs.size = self.size
        obs.is_static = True
        obs.is_visible = True
        self.obstacles.append(obs)
        self.get_logger().info(
            f'장애물 #{obs.id} 추가: s={s:.2f} d={d:.2f} ({x:.2f}, {y:.2f})')
        pitwall.event(f'장애물 #{obs.id} 추가 (s={s:.1f}, d={d:.2f})')

    def cb_opponent(self, msg: ObstacleArray):
        self.opponent = msg.obstacles
        self.opponent_stamp = self.get_clock().now()

    def cb_clear(self, _msg: Empty):
        n = len(self.obstacles)
        self.obstacles.clear()
        self.get_logger().info(f'정적 장애물 {n}개 전체 삭제')
        pitwall.event(f'정적 장애물 {n}개 전체 삭제')

    def cb_pmode(self, msg: String):
        mode = msg.data.strip().lower()
        merge = (mode != 'lidar_overlay')
        if merge != self.tracking_merge:
            self.tracking_merge = merge
            self.get_logger().info(f'perception 주입 모드: {mode}')
            pitwall.event(f'perception 모드: {mode}')

    def cb_lidar(self, msg: Bool):
        self.lidar_enable = bool(msg.data)

    def cb_odom(self, msg: Odometry):
        self.odom = msg

    def cb_scan(self, msg: LaserScan):
        """lidar_overlay 모드: 가상 장애물 합성점을 /scan에 삽입 -> /scan_overlay."""
        if self.tracking_merge or not self.lidar_enable:
            return
        if self.odom is None:
            self.pub_scan.publish(msg)
            return

        p = self.odom.pose.pose.position
        q = self.odom.pose.pose.orientation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                         1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        lx = p.x + self.laser_offset * math.cos(yaw)
        ly = p.y + self.laser_offset * math.sin(yaw)

        ranges = np.array(msg.ranges, dtype=np.float32)
        n = len(ranges)
        r = max(self.size / 2.0, 0.05)
        for obs in self.obstacles:
            rho = math.hypot(obs.position.x - lx, obs.position.y - ly)
            if rho <= r or rho > msg.range_max:
                continue
            bearing = math.atan2(obs.position.y - ly, obs.position.x - lx) - yaw
            bearing = math.atan2(math.sin(bearing), math.cos(bearing))
            half = math.asin(min(r / rho, 1.0))
            i0 = int((bearing - half - msg.angle_min) / msg.angle_increment)
            i1 = int((bearing + half - msg.angle_min) / msg.angle_increment)
            if i1 < 0 or i0 >= n:
                continue
            idx = np.arange(max(i0, 0), min(i1 + 1, n))
            ranges[idx] = np.minimum(ranges[idx], rho - r)

        out = LaserScan()
        out.header = msg.header
        out.angle_min = msg.angle_min
        out.angle_max = msg.angle_max
        out.angle_increment = msg.angle_increment
        out.time_increment = msg.time_increment
        out.scan_time = msg.scan_time
        out.range_min = msg.range_min
        out.range_max = msg.range_max
        out.ranges = ranges.tolist()
        self.pub_scan.publish(out)

    def publish(self):
        arr = ObstacleArray()
        arr.header.stamp = self.get_clock().now().to_msg()
        arr.header.frame_id = 'map'

        # lidar_overlay 모드에서는 플래너 직접 주입 중단 (빈 배열 유지,
        # 장애물은 cb_scan이 /scan_overlay에 합성점으로 삽입)
        if self.tracking_merge:
            arr.obstacles = list(self.obstacles)
            # 상대 관측(동적)을 정적 장애물과 합쳐 발행 (0.5s 타임아웃)
            if (self.opponent and self.opponent_stamp is not None and
                    (self.get_clock().now() - self.opponent_stamp).nanoseconds
                    * 1e-9 < 0.5):
                arr.obstacles += self.opponent
        self.pub_obs.publish(arr)

        mrks = MarkerArray()
        clear = Marker()
        clear.action = Marker.DELETEALL
        mrks.markers.append(clear)
        for obs in self.obstacles:
            m = Marker()
            m.header.frame_id = 'map'
            m.header.stamp = arr.header.stamp
            m.ns = 'virtual_obstacles'
            m.id = obs.id
            m.type = Marker.CYLINDER
            m.scale.x = m.scale.y = self.size
            m.scale.z = 0.3
            m.color.r, m.color.a = 1.0, 0.9
            m.pose.position.x = obs.position.x
            m.pose.position.y = obs.position.y
            m.pose.position.z = 0.15
            m.pose.orientation.w = 1.0
            mrks.markers.append(m)
        self.pub_mrk.publish(mrks)


def main():
    rclpy.init()
    rclpy.spin(VirtualObstacleNode())
    rclpy.shutdown()


if __name__ == '__main__':
    main()
