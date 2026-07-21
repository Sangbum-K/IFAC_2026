#!/usr/bin/env python3
"""조향 스윕 기록기.

스틱을 좌우 끝까지 천천히 밀면서 실행하면, clip 전/후 서보 명령의
최소·최대와 두 값이 갈라지는 지점을 잡아준다.

  python3 ~/servo_sweep.py            # 20초 기록
  python3 ~/servo_sweep.py 30         # 30초 기록
"""
import sys
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64


class Sweep(Node):
    def __init__(self):
        super().__init__('servo_sweep')
        self.cmd = None
        self.out = None
        self.cmd_lo = self.cmd_hi = None
        self.out_lo = self.out_hi = None
        self.max_gap = 0.0
        self.gap_at = None
        self.create_subscription(
            Float64, '/commands/servo/position', self._cmd, 10)
        self.create_subscription(
            Float64, '/sensors/servo_position_command', self._out, 10)
        self.create_timer(0.25, self._print)

    def _cmd(self, m):
        self.cmd = m.data
        self.cmd_lo = m.data if self.cmd_lo is None else min(self.cmd_lo, m.data)
        self.cmd_hi = m.data if self.cmd_hi is None else max(self.cmd_hi, m.data)
        self._gap()

    def _out(self, m):
        self.out = m.data
        self.out_lo = m.data if self.out_lo is None else min(self.out_lo, m.data)
        self.out_hi = m.data if self.out_hi is None else max(self.out_hi, m.data)
        self._gap()

    def _gap(self):
        if self.cmd is None or self.out is None:
            return
        g = abs(self.cmd - self.out)
        if g > self.max_gap:
            self.max_gap = g
            self.gap_at = (self.cmd, self.out)

    def _print(self):
        if self.cmd is None:
            print('  ...토픽 대기중 (launch 떠 있나요?)')
            return
        out = 'n/a' if self.out is None else f'{self.out:.4f}'
        print(f'  clip전 {self.cmd:7.4f}   clip후 {out:>7}')

    def report(self):
        print('\n' + '=' * 52)
        if self.cmd_lo is None:
            print('  데이터 없음 — /commands/servo/position 이 안 나왔습니다.')
            return
        print(f'  clip 전 (/commands/servo/position)     : {self.cmd_lo:.4f} ~ {self.cmd_hi:.4f}')
        if self.out_lo is None:
            print('  clip 후 (/sensors/...) : 수신 없음 -> vesc_driver 확인 필요')
            return
        print(f'  clip 후 (/sensors/servo_position_command): {self.out_lo:.4f} ~ {self.out_hi:.4f}')
        print()
        if self.max_gap < 1e-4:
            print('  => 두 값이 항상 일치. 소프트웨어 clip 없음.')
            print('     즉 명령이 그대로 서보까지 갔으므로, 조향이 덜 꺾였다면')
            print('     원인은 기계적 한계입니다.')
        else:
            print(f'  => 최대 {self.max_gap:.4f} 만큼 갈라짐 '
                  f'(clip전 {self.gap_at[0]:.4f} -> clip후 {self.gap_at[1]:.4f})')
            print('     소프트웨어 clip 이 걸리고 있습니다.')


def main():
    dur = float(sys.argv[1]) if len(sys.argv) > 1 else 20.0
    rclpy.init()
    n = Sweep()
    print(f'{dur:.0f}초간 기록합니다. 조향 스틱을 좌 끝 -> 중앙 -> 우 끝으로')
    print('천천히 밀면서 앞바퀴가 언제 멈추는지 보세요.\n')
    import time
    end = time.time() + dur
    while time.time() < end:
        rclpy.spin_once(n, timeout_sec=0.05)
    n.report()
    n.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
