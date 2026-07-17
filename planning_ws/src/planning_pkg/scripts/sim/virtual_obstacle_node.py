#!/usr/bin/env python3
"""RViz 'Publish Point' 클릭으로 가상 정적 장애물을 만드는 시뮬레이션 전용 노드.

unicorn(HMCL) 튜토리얼의 virtual_perception 역할:
  /clicked_point 클릭 -> Frenet 변환 -> /perception/obstacles (10 Hz 상시 발행)
기존 장애물 근처(remove_radius 이내)를 다시 클릭하면 그 장애물이 제거된다.
장애물이 없어도 빈 배열을 계속 발행한다 (spliner의 stale 타임아웃과 연동).

perception 팀 노드가 준비되면 이 노드 대신 실제 perception이
/perception/obstacles를 발행하게 된다.
"""
import math

import numpy as np
import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PointStamped
from visualization_msgs.msg import Marker, MarkerArray

from ae_hyu_msgs.msg import Obstacle, ObstacleArray, WpntArray


class VirtualObstacleNode(Node):
    def __init__(self):
        super().__init__('virtual_obstacle_node')

        self.declare_parameter('virtual_obstacle/publish_rate_hz', 10.0)
        self.declare_parameter('virtual_obstacle/obstacle_size_m', 0.3)
        self.declare_parameter('virtual_obstacle/remove_radius_m', 0.5)
        rate = self.get_parameter('virtual_obstacle/publish_rate_hz').value
        self.size = self.get_parameter('virtual_obstacle/obstacle_size_m').value
        self.remove_radius = self.get_parameter('virtual_obstacle/remove_radius_m').value

        self.wpnts = None       # np.array [N, 4]: s, x, y, psi
        self.track_length = 0.0
        self.obstacles = []     # list[Obstacle]
        self.next_id = 1
        self.opponent = None    # 가상 상대 (virtual_opponent_node)
        self.opponent_stamp = None

        self.create_subscription(WpntArray, '/global_waypoints', self.cb_gb, 10)
        self.create_subscription(PointStamped, '/clicked_point', self.cb_click, 10)
        self.create_subscription(ObstacleArray, '/perception/opponent', self.cb_opponent, 10)
        self.pub_obs = self.create_publisher(ObstacleArray, '/perception/obstacles', 10)
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

    def cb_opponent(self, msg: ObstacleArray):
        self.opponent = msg.obstacles
        self.opponent_stamp = self.get_clock().now()

    def publish(self):
        arr = ObstacleArray()
        arr.header.stamp = self.get_clock().now().to_msg()
        arr.header.frame_id = 'map'
        arr.obstacles = list(self.obstacles)
        # 가상 상대(동적)를 정적 장애물과 합쳐 발행 (0.5s 타임아웃)
        if (self.opponent and self.opponent_stamp is not None and
                (self.get_clock().now() - self.opponent_stamp).nanoseconds * 1e-9 < 0.5):
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
