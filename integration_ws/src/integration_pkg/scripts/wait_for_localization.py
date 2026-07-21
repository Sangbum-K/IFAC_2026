#!/usr/bin/env python3
"""파티클 필터가 수렴할 때까지 기다렸다가 종료하는 게이트 노드.

real_car.launch.py가 이 노드를 먼저 띄우고, 정상 종료(exit 0)를 신호로
자율 스택(stack.launch.py env:=real_car)을 자동으로 실행한다.

"수렴"의 판정 기준 3가지 — 셋 다 만족해야 통과:
  1. /pf/pose/odom 이 최근에 들어옴          (PF가 살아 있음)
  2. map -> base_link TF 조회 가능           (PF가 TF를 쏘고 있음)
  3. /pf/viz/particles 파티클 구름이 좁음    (초기 위치를 찍어서 실제로 모임)

3번이 핵심이다. PF는 초기 위치를 안 찍어도 (맵 전체에 파티클이 흩어진 채로)
odom과 TF를 그냥 발행하기 때문에, 1·2만 보면 엉뚱한 위치로 스택이 뜬다.
"""

import math
import sys

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

import tf2_ros

from geometry_msgs.msg import PoseArray
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan


class WaitForLocalization(Node):

    def __init__(self):
        super().__init__('wait_for_localization')

        self.declare_parameter('odom_topic', '/pf/pose/odom')
        self.declare_parameter('particles_topic', '/pf/viz/particles')
        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('base_frame', 'base_link')
        # 파티클 x/y 표준편차가 이 값보다 작으면 "모였다"고 본다.
        self.declare_parameter('spread_threshold_m', 0.35)
        # 위 조건을 연속으로 몇 번 만족해야 통과시킬지 (파티클 발행 ~ 10Hz).
        self.declare_parameter('stable_count', 10)
        # 0보다 크면 그 시간(초) 안에 수렴 못 하면 실패로 종료.
        self.declare_parameter('timeout_sec', 0.0)

        self.map_frame = self.get_parameter('map_frame').value
        self.base_frame = self.get_parameter('base_frame').value
        self.spread_threshold = float(self.get_parameter('spread_threshold_m').value)
        self.stable_needed = int(self.get_parameter('stable_count').value)
        self.timeout_sec = float(self.get_parameter('timeout_sec').value)

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.last_scan = None
        self.last_odom = None
        self.spread = None
        self.stable_count = 0
        self.start_time = self.now()

        self.create_subscription(
            LaserScan, self.get_parameter('scan_topic').value,
            self.scan_cb, qos_profile_sensor_data)
        self.create_subscription(
            Odometry, self.get_parameter('odom_topic').value,
            self.odom_cb, 1)
        # PF는 구독자가 있을 때만 파티클을 발행한다 (get_subscription_count 체크).
        # 즉 이 구독 자체가 파티클 발행을 켜는 역할도 한다.
        self.create_subscription(
            PoseArray, self.get_parameter('particles_topic').value,
            self.particles_cb, 1)

        self.create_timer(1.0, self.check)
        self.get_logger().info(
            '파티클 필터 수렴 대기 중 — RViz에서 "2D Pose Estimate"로 '
            '차의 실제 위치를 클릭+드래그 하세요.')

    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def scan_cb(self, _msg):
        self.last_scan = self.now()

    def odom_cb(self, _msg):
        self.last_odom = self.now()

    def particles_cb(self, msg):
        n = len(msg.poses)
        if n < 2:
            return
        xs = [p.position.x for p in msg.poses]
        ys = [p.position.y for p in msg.poses]
        mx = sum(xs) / n
        my = sum(ys) / n
        var = sum((x - mx) ** 2 + (y - my) ** 2 for x, y in zip(xs, ys)) / n
        self.spread = math.sqrt(var)

        if self.spread < self.spread_threshold and self.tf_ready():
            self.stable_count += 1
        else:
            self.stable_count = 0

    def tf_ready(self):
        return self.tf_buffer.can_transform(
            self.map_frame, self.base_frame, rclpy.time.Time())

    def fresh(self, stamp, max_age=1.0):
        return stamp is not None and (self.now() - stamp) < max_age

    def check(self):
        """1초마다 상태를 찍고, 다 됐으면 exit 0으로 빠져나간다."""
        if self.stable_count >= self.stable_needed:
            self.get_logger().info(
                '수렴 완료 (파티클 spread=%.2fm) — 자율 스택을 시작합니다.'
                % (self.spread or 0.0))
            raise SystemExit(0)

        if self.timeout_sec > 0 and (self.now() - self.start_time) > self.timeout_sec:
            self.get_logger().error(
                '%.0f초 안에 수렴하지 못했습니다. 자율 스택을 시작하지 않습니다.'
                % self.timeout_sec)
            raise SystemExit(1)

        # 어느 단계에서 막혔는지 사람이 바로 알 수 있게 찍어준다.
        if not self.fresh(self.last_scan):
            hint = '/scan 없음 — 하드웨어(LiDAR)가 안 떴습니다'
        elif not self.fresh(self.last_odom):
            hint = '/pf/pose/odom 없음 — particle_filter 또는 VESC /odom 확인'
        elif not self.tf_ready():
            hint = 'map -> base_link TF 없음 — PF가 아직 TF를 안 냅니다'
        elif self.spread is None:
            hint = '파티클 미수신 — particle_filter의 viz 설정 확인'
        elif self.spread < self.spread_threshold:
            hint = ('수렴 확인 중 (spread=%.2fm, %d/%d)'
                    % (self.spread, self.stable_count, self.stable_needed))
        else:
            hint = ('파티클이 아직 흩어져 있음 (spread=%.2fm > %.2fm) — '
                    'RViz에서 2D Pose Estimate로 초기 위치를 찍으세요'
                    % (self.spread, self.spread_threshold))
        self.get_logger().info('대기 중: %s' % hint)


def main(args=None):
    rclpy.init(args=args)
    node = WaitForLocalization()
    code = 0
    try:
        rclpy.spin(node)
    except SystemExit as e:
        code = e.code if isinstance(e.code, int) else 0
    except (KeyboardInterrupt, ExternalShutdownException):
        code = 1
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    sys.exit(code)


if __name__ == '__main__':
    main()
