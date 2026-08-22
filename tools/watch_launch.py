#!/usr/bin/env python3
"""정지 발진 하한 실차 검증용 한 줄 모니터.

    python3 ~/IFAC_ws/tools/watch_launch.py

0.2 초마다 한 줄씩 찍는다:

    t=  12.4  v=0.00  cmd=1.28  steer=+0.31(85%)  GATE=OPEN(0.03s)  esc=.  brake=0.0

  v      실제 속도 (/pf/pose/odom)
  cmd    최종 속도 명령 (/ackermann_cmd) — 이게 0 에 붙어 있으면 못 나가는 것
  steer  조향 명령과 한계(0.36 rad) 대비 비율. 95% 이상이 계속되면 포화다
  GATE   /planner/avoidance/evade_committed. 괄호는 마지막 수신 후 경과.
         0.30 s 넘게 안 오면 STALE (컨트롤러도 그때 하한을 끈다)
  esc    /planner/escape_active
  brake  /commands/motor/brake [A]

커스텀 메시지를 안 쓰므로 ros humble + hardware_ws 만 source 되어 있으면 된다.
"""
import math
import sys

import rclpy
from ackermann_msgs.msg import AckermannDriveStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, Float64

STEER_LIMIT_RAD = 0.36   # TUNING_STATUS.md §1 (플래닝·컨트롤이 쓰는 내림값)


class Watch(Node):
    def __init__(self):
        super().__init__('watch_launch')
        self.v = float('nan')
        self.cmd = float('nan')
        self.steer = float('nan')
        self.gate = None
        self.gate_stamp = None
        self.esc = False
        self.brake = 0.0

        best = QoSProfile(depth=10)
        best.reliability = ReliabilityPolicy.BEST_EFFORT
        rel = QoSProfile(depth=10)
        rel.reliability = ReliabilityPolicy.RELIABLE

        self.create_subscription(Odometry, '/pf/pose/odom', self.cb_odom, best)
        self.create_subscription(
            AckermannDriveStamped, '/ackermann_cmd', self.cb_cmd, best)
        self.create_subscription(
            Bool, '/planner/avoidance/evade_committed', self.cb_gate, rel)
        self.create_subscription(
            Bool, '/planner/escape_active', self.cb_esc, rel)
        self.create_subscription(
            Float64, '/commands/motor/brake', self.cb_brake, best)
        self.create_timer(0.2, self.tick)
        self.t0 = self.get_clock().now()

    def cb_odom(self, m):
        self.v = math.hypot(m.twist.twist.linear.x, m.twist.twist.linear.y)

    def cb_cmd(self, m):
        self.cmd = m.drive.speed
        self.steer = m.drive.steering_angle

    def cb_gate(self, m):
        self.gate = m.data
        self.gate_stamp = self.get_clock().now()

    def cb_esc(self, m):
        self.esc = m.data

    def cb_brake(self, m):
        self.brake = m.data

    def tick(self):
        now = self.get_clock().now()
        t = (now - self.t0).nanoseconds / 1e9
        if self.gate_stamp is None:
            gate = 'none'
        else:
            age = (now - self.gate_stamp).nanoseconds / 1e9
            if age > 0.30:
                gate = 'STALE(%.2fs)' % age
            else:
                gate = '%s(%.2fs)' % ('OPEN' if self.gate else 'shut', age)
        pct = abs(self.steer) / STEER_LIMIT_RAD * 100.0 \
            if math.isfinite(self.steer) else float('nan')
        print('t=%6.1f  v=%4.2f  cmd=%5.2f  steer=%+.3f(%3.0f%%)  '
              'GATE=%-12s esc=%s  brake=%.1f'
              % (t, self.v, self.cmd, self.steer, pct, gate,
                 'E' if self.esc else '.', self.brake),
              flush=True)


def main():
    rclpy.init()
    node = Watch()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
