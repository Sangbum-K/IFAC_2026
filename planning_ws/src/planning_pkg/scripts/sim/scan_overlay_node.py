#!/usr/bin/env python3
"""Perception Injection — LiDAR overlay (시뮬레이션 전용).

unicorn 튜토리얼의 'LiDAR Overlay' 모드: 가상 정적 장애물의 합성 반사점을
ego /scan에 삽입해 /scan_overlay로 재발행한다. perception 팀 파이프라인이
/scan_overlay를 구독하면 탐지→추적→플래너 전체 체인을 시뮬에서 검증할 수 있다.
(상대 차량은 gym 2-agent라 실제 /scan에 이미 물리적으로 찍힌다 — 삽입 불필요.)

모드 (/sim/perception_mode, std_msgs/String — 패널 콤보박스):
  - tracking_merge (기본): 이 노드는 대기. virtual_obstacle_node가
    ground-truth를 /perception/obstacles로 직접 주입한다 (플래너 단독 검증).
  - lidar_overlay: 이 노드가 /scan_overlay 발행, ground-truth 주입은 중단
    (perception 파이프라인 검증 — 현재 perception 노드가 없으면 플래너는
    장애물을 못 본다. 의도된 동작).

/sim/ego_lidar_enable (Bool): false면 overlay 발행 중단 (ego LiDAR 토글).
"""
import math

import numpy as np
import rclpy
from rclpy.node import Node

from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, String

from ae_hyu_msgs.msg import ObstacleArray

import pitwall


class ScanOverlayNode(Node):
    def __init__(self):
        super().__init__('scan_overlay_node')
        pitwall.init(self)

        self.declare_parameter('scan_overlay/laser_offset_m', 0.275)
        self.laser_offset = self.get_parameter('scan_overlay/laser_offset_m').value

        self.mode = 'tracking_merge'
        self.lidar_enable = True
        self.odom = None
        self.obstacles = []      # [(x, y, r)]

        self.create_subscription(String, '/sim/perception_mode', self.cb_mode, 10)
        self.create_subscription(Bool, '/sim/ego_lidar_enable', self.cb_lidar, 10)
        self.create_subscription(Odometry, '/ego_racecar/odom', self.cb_odom, 10)
        self.create_subscription(ObstacleArray, '/sim/virtual_obstacles',
                                 self.cb_obs, 10)
        self.create_subscription(LaserScan, '/scan', self.cb_scan, 10)
        self.pub_scan = self.create_publisher(LaserScan, '/scan_overlay', 10)

        self.get_logger().info('Scan overlay ready (perception_mode=tracking_merge)')

    def cb_mode(self, msg: String):
        mode = msg.data.strip().lower()
        if mode in ('tracking_merge', 'lidar_overlay') and mode != self.mode:
            self.mode = mode
            self.get_logger().info(f'perception 주입 모드: {mode}')
            pitwall.event(f'perception 모드: {mode}')

    def cb_lidar(self, msg: Bool):
        self.lidar_enable = bool(msg.data)

    def cb_odom(self, msg: Odometry):
        self.odom = msg

    def cb_obs(self, msg: ObstacleArray):
        self.obstacles = [(o.position.x, o.position.y, max(o.size / 2.0, 0.05))
                          for o in msg.obstacles]

    def cb_scan(self, msg: LaserScan):
        if self.mode != 'lidar_overlay' or not self.lidar_enable:
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
        for ox, oy, r in self.obstacles:
            rho = math.hypot(ox - lx, oy - ly)
            if rho <= r or rho > msg.range_max:
                continue
            bearing = math.atan2(oy - ly, ox - lx) - yaw
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


def main():
    rclpy.init()
    rclpy.spin(ScanOverlayNode())
    rclpy.shutdown()


if __name__ == '__main__':
    main()
