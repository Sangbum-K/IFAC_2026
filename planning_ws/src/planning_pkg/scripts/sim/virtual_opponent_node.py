#!/usr/bin/env python3
"""가상 동적 상대 차량 (시뮬레이션 전용) — unicorn 튜토리얼의 상대 스폰 대응.

RViz '2D Goal Pose' 클릭으로 상대를 스폰/제거(토글)한다. 상대는 레이스라인을
`opponent/speed_scale` 배 속도로 달리는 kinematic ghost로, ObstacleArray
(is_static=false, s/d/vs 채움)를 /perception/opponent로 발행한다.
virtual_obstacle_node가 이를 정적 장애물과 합쳐 /perception/obstacles로 낸다.

Predictive Spliner 개발용 테스트베드: perception 팀 노드가 생기기 전까지
상대 관측 데이터를 공급한다.
"""
import math

import numpy as np
import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import Marker

from ae_hyu_msgs.msg import Obstacle, ObstacleArray, WpntArray


class VirtualOpponentNode(Node):
    def __init__(self):
        super().__init__('virtual_opponent_node')

        self.declare_parameter('opponent/speed_scale', 0.5)   # raceline 속도 대비
        self.declare_parameter('opponent/d_offset_m', 0.0)    # 레이스라인 횡 오프셋
        self.declare_parameter('opponent/size_m', 0.4)
        self.declare_parameter('opponent/update_rate_hz', 20.0)
        self.declare_parameter('opponent/autostart', False)   # 켜면 spawn_s에서 자동 스폰
        self.declare_parameter('opponent/spawn_s', 15.0)
        gp = lambda n: self.get_parameter(n).value
        self.speed_scale = gp('opponent/speed_scale')
        self.d_offset = gp('opponent/d_offset_m')
        self.size = gp('opponent/size_m')
        self.rate = gp('opponent/update_rate_hz')
        self.autostart = gp('opponent/autostart')
        self.spawn_s = gp('opponent/spawn_s')

        self.wpts = None          # [s, x, y, psi, vx]
        self.track_len = 0.0
        self.ds = 0.1
        self.active = False
        self.s = 0.0

        self.create_subscription(WpntArray, '/global_waypoints', self.cb_gb, 10)
        self.create_subscription(PoseStamped, '/goal_pose', self.cb_goal, 10)
        self.pub_obs = self.create_publisher(ObstacleArray, '/perception/opponent', 10)
        self.pub_mrk = self.create_publisher(Marker, '/perception/opponent_marker', 10)
        self.create_timer(1.0 / self.rate, self.step)

        self.get_logger().info(
            f'Virtual opponent ready: RViz "2D Goal Pose"로 스폰/제거 '
            f'(speed_scale={self.speed_scale})')

    def cb_gb(self, msg: WpntArray):
        first = self.wpts is None
        self.wpts = np.array(
            [[w.s_m, w.x_m, w.y_m, w.psi_rad, w.vx_mps] for w in msg.wpnts])
        self.ds = self.wpts[1, 0] - self.wpts[0, 0]
        self.track_len = self.wpts[-1, 0] + self.ds
        if first and self.autostart:
            self.s = self.spawn_s
            self.active = True
            self.get_logger().info(f'상대 자동 스폰: s={self.s:.1f}')

    def cb_goal(self, msg: PoseStamped):
        if self.wpts is None:
            return
        if self.active:
            self.active = False
            self.get_logger().info('상대 제거')
            return
        d2 = (self.wpts[:, 1] - msg.pose.position.x) ** 2 + \
             (self.wpts[:, 2] - msg.pose.position.y) ** 2
        self.s = float(self.wpts[int(np.argmin(d2)), 0])
        self.active = True
        self.get_logger().info(f'상대 스폰: s={self.s:.1f} ({self.speed_scale*100:.0f}% 속도)')

    def _interp_row(self, s):
        idx = s / self.ds
        i0 = int(idx) % len(self.wpts)
        i1 = (i0 + 1) % len(self.wpts)
        t = idx - math.floor(idx)
        r0, r1 = self.wpts[i0], self.wpts[i1]
        psi = r0[3] + t * math.atan2(math.sin(r1[3] - r0[3]), math.cos(r1[3] - r0[3]))
        return (r0[1] + t * (r1[1] - r0[1]),
                r0[2] + t * (r1[2] - r0[2]), psi, r0[4] + t * (r1[4] - r0[4]))

    def step(self):
        arr = ObstacleArray()
        arr.header.stamp = self.get_clock().now().to_msg()
        arr.header.frame_id = 'map'

        if self.active and self.wpts is not None:
            x_c, y_c, psi, v_race = self._interp_row(self.s)
            v = v_race * self.speed_scale
            self.s = (self.s + v / self.rate) % self.track_len
            half = self.size / 2.0

            obs = Obstacle()
            obs.id = 99
            obs.position.x = x_c - self.d_offset * math.sin(psi)
            obs.position.y = y_c + self.d_offset * math.cos(psi)
            obs.s_center = self.s
            obs.d_center = self.d_offset
            obs.s_start = (self.s - half) % self.track_len
            obs.s_end = (self.s + half) % self.track_len
            obs.d_left = self.d_offset + half
            obs.d_right = self.d_offset - half
            obs.vs = v
            obs.vd = 0.0
            obs.size = self.size
            obs.is_static = False
            obs.is_visible = True
            arr.obstacles.append(obs)

            mrk = Marker()
            mrk.header.frame_id = 'map'
            mrk.header.stamp = arr.header.stamp
            mrk.ns = 'virtual_opponent'
            mrk.id = 99
            mrk.type = Marker.CUBE
            mrk.scale.x = 0.5
            mrk.scale.y = 0.3
            mrk.scale.z = 0.15
            mrk.color.r, mrk.color.g, mrk.color.a = 1.0, 0.5, 0.95  # 주황
            mrk.pose.position.x = obs.position.x
            mrk.pose.position.y = obs.position.y
            mrk.pose.position.z = 0.08
            mrk.pose.orientation.z = math.sin(psi / 2)
            mrk.pose.orientation.w = math.cos(psi / 2)
            self.pub_mrk.publish(mrk)
        else:
            mrk = Marker()
            mrk.ns = 'virtual_opponent'
            mrk.id = 99
            mrk.action = Marker.DELETE
            self.pub_mrk.publish(mrk)

        self.pub_obs.publish(arr)


def main():
    rclpy.init()
    rclpy.spin(VirtualOpponentNode())
    rclpy.shutdown()


if __name__ == '__main__':
    main()
