#!/usr/bin/env python3
"""PF가 명시적으로 안전해질 때까지 기다렸다가 종료하는 시작 게이트.

real_car.launch.py가 이 노드를 먼저 띄우고, 정상 종료(exit 0)를 신호로
자율 스택(stack.launch.py env:=real_car)을 자동으로 실행한다.

통과 조건은 PF가 수신한 /initialpose를 실제로 적용했고, scan-map quality가 설정된
횟수만큼 연속 OK가 되어 /pf/localization_ready=true를 내며, 그 heartbeat와
PF odom/TF/scan이 모두 fresh한 것이다. 파티클 spread는 틀린 위치에 좁게
모여도 작아질 수 있으므로 안전 판정으로 사용하지 않는다.
"""

import sys

from nav_msgs.msg import Odometry
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    qos_profile_sensor_data,
    QoSProfile,
    ReliabilityPolicy,
)
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, String
import tf2_ros


class WaitForLocalization(Node):

    def __init__(self):
        super().__init__('wait_for_localization')

        self.declare_parameter('odom_topic', '/pf/pose/odom')
        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('ready_topic', '/pf/localization_ready')
        self.declare_parameter(
            'initialpose_received_topic', '/pf/initialpose_received')
        self.declare_parameter(
            'initialpose_accepted_topic', '/pf/initialpose_accepted')
        self.declare_parameter(
            'quality_topic', '/pf/localization_quality')
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('status_timeout_sec', 0.50)
        # 0보다 크면 그 시간(초) 안에 ready가 안 되면 실패로 종료.
        self.declare_parameter('timeout_sec', 0.0)

        self.map_frame = self.get_parameter('map_frame').value
        self.base_frame = self.get_parameter('base_frame').value
        self.status_timeout = float(
            self.get_parameter('status_timeout_sec').value)
        self.timeout_sec = float(self.get_parameter('timeout_sec').value)

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.last_scan = None
        self.last_odom = None
        self.last_status = None
        self.initialpose_received = False
        self.initialpose_accepted = False
        self.localization_ready = False
        self.quality = 'UNKNOWN'
        self.start_time = self.now()

        self.create_subscription(
            LaserScan, self.get_parameter('scan_topic').value,
            self.scan_cb, qos_profile_sensor_data)
        self.create_subscription(
            Odometry, self.get_parameter('odom_topic').value,
            self.odom_cb, 1)
        status_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.create_subscription(
            Bool, self.get_parameter('ready_topic').value,
            self.ready_cb, status_qos)
        self.create_subscription(
            Bool, self.get_parameter('initialpose_received_topic').value,
            self.initialpose_cb, status_qos)
        self.create_subscription(
            Bool, self.get_parameter('initialpose_accepted_topic').value,
            self.initialpose_accepted_cb, status_qos)
        self.create_subscription(
            String, self.get_parameter('quality_topic').value,
            self.quality_cb, status_qos)

        self.create_timer(1.0, self.check)
        self.get_logger().info(
            'localization readiness 대기 중 - RViz "2D Pose Estimate"로 '
            '실제 위치를 지정한 뒤 연속 quality OK를 확인합니다.')

    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def scan_cb(self, _msg):
        self.last_scan = self.now()

    def odom_cb(self, _msg):
        self.last_odom = self.now()

    def ready_cb(self, msg):
        self.localization_ready = bool(msg.data)
        self.last_status = self.now()

    def initialpose_cb(self, msg):
        self.initialpose_received = bool(msg.data)

    def initialpose_accepted_cb(self, msg):
        self.initialpose_accepted = bool(msg.data)

    def quality_cb(self, msg):
        self.quality = msg.data

    def tf_ready(self):
        return self.tf_buffer.can_transform(
            self.map_frame, self.base_frame, rclpy.time.Time())

    def fresh(self, stamp, max_age=1.0):
        if stamp is None:
            return False
        age = self.now() - stamp
        return 0.0 <= age < max_age

    def check(self):
        """1초마다 상태를 찍고, 다 됐으면 exit 0으로 빠져나간다."""
        status_fresh = self.fresh(
            self.last_status, max_age=self.status_timeout)
        if (
                self.initialpose_received
                and self.initialpose_accepted
                and self.localization_ready
                and status_fresh
                and self.fresh(self.last_scan)
                and self.fresh(self.last_odom)
                and self.tf_ready()):
            self.get_logger().info(
                '초기 위치 적용 + 연속 quality OK 확인 완료 - '
                '자율 스택을 시작합니다.')
            raise SystemExit(0)

        if self.timeout_sec > 0 and (self.now() - self.start_time) > self.timeout_sec:
            self.get_logger().error(
                '%.0f초 안에 ready가 되지 않았습니다. 자율 스택을 시작하지 않습니다.'
                % self.timeout_sec)
            raise SystemExit(1)

        # 어느 단계에서 막혔는지 사람이 바로 알 수 있게 찍어준다.
        if not self.fresh(self.last_scan):
            hint = '/scan 없음 — 하드웨어(LiDAR)가 안 떴습니다'
        elif self.last_status is None:
            hint = '/pf/localization_ready 없음 — 새 particle_filter가 안 떴습니다'
        elif not self.initialpose_received:
            hint = ('/initialpose 미수신 — RViz 2D Pose Estimate로 실제 위치와 '
                    '방향을 지정하세요')
        elif not self.initialpose_accepted:
            hint = ('/initialpose 수신 완료, PF 적용 대기 중 — '
                    'laser<->base_link TF를 확인하세요')
        elif not status_fresh:
            hint = ('PF readiness heartbeat stale (%.2fs 초과) — worker/scan sync 확인'
                    % self.status_timeout)
        elif not self.localization_ready:
            hint = ('연속 scan-map quality OK 대기 중 (현재 %s)'
                    % self.quality)
        elif not self.fresh(self.last_odom):
            hint = '/pf/pose/odom 없음 — particle_filter 또는 VESC /odom 확인'
        elif not self.tf_ready():
            hint = 'map -> base_link TF 없음 — PF가 아직 TF를 안 냅니다'
        else:
            hint = 'readiness 조건 재확인 중'
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
