#!/usr/bin/env python3
"""Stand-test helper: hold or step steering commands with the wheels off the ground.

Publishes to /teleop with speed pinned to 0.0, so the car never drives. Switch the
joystick to AUTONOMOUS (button 5) first -- otherwise joy_toggle_teleop keeps
publishing to the same topic and the two commands interleave.

Calibration is read straight out of vesc.yaml so the printed servo value always
matches what ackermann_to_vesc_node is actually sending.

  sweep  -- step through a range, holding each value, to find where the wheels stop
  hold   -- park at one steering angle, for straightedge / string alignment work

Examples:
  python3 ~/steer_stand.py sweep 0.20 0.36 0.02     # left, toward the suspected stop
  python3 ~/steer_stand.py sweep -0.20 -0.40 -0.02  # right
  python3 ~/steer_stand.py sweep -0.05 0.05 0.01    # backlash band around center
  python3 ~/steer_stand.py hold 0.0                 # park at commanded straight
"""

import argparse
import math
import os
import time

import rclpy
import yaml
from ackermann_msgs.msg import AckermannDriveStamped
from rclpy.node import Node

# realpath 를 쓰므로 ~/steer_stand.py 같은 심볼릭 링크로 실행해도 원본을 찾는다.
DEFAULT_VESC_YAML = os.path.normpath(os.path.join(
    os.path.dirname(os.path.realpath(__file__)),
    '..', 'src', 'f1tenth_stack', 'config', 'vesc.yaml'))

PUBLISH_RATE = 20.0


def load_calibration(path):
    with open(path, 'r') as f:
        params = yaml.safe_load(f)['/**']['ros__parameters']
    return (float(params['steering_angle_to_servo_gain']),
            float(params['steering_angle_to_servo_offset']))


class StandCommander(Node):
    def __init__(self, gain, offset):
        super().__init__('steer_stand')
        self.gain = gain
        self.offset = offset
        self.steering = 0.0
        self.pub = self.create_publisher(AckermannDriveStamped, 'teleop', 10)
        self.create_timer(1.0 / PUBLISH_RATE, self._publish)

    def servo_of(self, steering):
        return self.gain * steering + self.offset

    def _publish(self):
        msg = AckermannDriveStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.drive.speed = 0.0  # wheels are off the ground; never drive on the stand
        msg.drive.steering_angle = self.steering
        self.pub.publish(msg)


def frange(start, stop, step):
    if step == 0.0:
        raise ValueError('step must be non-zero')
    count = int(math.floor(abs((stop - start) / step))) + 1
    return [start + i * step for i in range(count)]


def dwell(node, seconds):
    deadline = time.time() + seconds
    while time.time() < deadline:
        rclpy.spin_once(node, timeout_sec=0.02)


def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--vesc-yaml', default=DEFAULT_VESC_YAML)
    sub = parser.add_subparsers(dest='mode', required=True)

    sweep = sub.add_parser('sweep', help='step through a range of steering angles')
    sweep.add_argument('start', type=float)
    sweep.add_argument('stop', type=float)
    sweep.add_argument('step', type=float)
    sweep.add_argument('--dwell', type=float, default=3.0,
                       help='seconds to hold each value (default 3)')

    hold = sub.add_parser('hold', help='hold a single steering angle until Ctrl+C')
    hold.add_argument('value', type=float)

    args = parser.parse_args()

    gain, offset = load_calibration(args.vesc_yaml)
    print(f'calibration from {args.vesc_yaml}')
    print(f'  gain   = {gain}')
    print(f'  offset = {offset}')
    print('speed is pinned to 0.0 -- switch the joystick to AUTONOMOUS (button 5) first.')
    print('STOP IMMEDIATELY (Ctrl+C) if the servo starts buzzing: that is a stall.\n')

    rclpy.init()
    node = StandCommander(gain, offset)
    try:
        if args.mode == 'hold':
            node.steering = args.value
            servo = node.servo_of(args.value)
            print(f'  delta = {args.value:+.4f} rad ({math.degrees(args.value):+6.2f} deg)'
                  f'   servo = {servo:.4f}   [Ctrl+C to stop]')
            rclpy.spin(node)
        else:
            for value in frange(args.start, args.stop, args.step):
                node.steering = value
                servo = node.servo_of(value)
                flag = '  <-- OUTSIDE servo_min/max' if not 0.05 <= servo <= 0.95 else ''
                print(f'  delta = {value:+.4f} rad ({math.degrees(value):+6.2f} deg)'
                      f'   servo = {servo:.4f}{flag}')
                dwell(node, args.dwell)
            print('\nsweep complete, returning to center')
            node.steering = 0.0
            dwell(node, 1.0)
    except KeyboardInterrupt:
        pass
    finally:
        node.steering = 0.0
        for _ in range(5):
            rclpy.spin_once(node, timeout_sec=0.02)
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
