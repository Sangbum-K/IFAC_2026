"""Joystick teleop with an edge-triggered manual/autonomous toggle.

Replaces the upstream joy_teleop node, which only supports hold-to-drive
deadman buttons. Here the mode is latched: pressing the toggle button once
switches manual <-> autonomous and the stick works without holding anything.

manual      -- joystick axes are published to the teleop topic continuously.
autonomous  -- a short burst of zero commands is sent to bring the car to a
               stop, then this node goes silent so the mux timeout lets the
               autonomy topic (lower priority) through.
"""

import rclpy
from rclpy.node import Node

from ackermann_msgs.msg import AckermannDriveStamped
from sensor_msgs.msg import Joy


class JoyToggleTeleop(Node):
    def __init__(self):
        super().__init__('joy_toggle_teleop')

        self.declare_parameter('teleop_topic', 'teleop')
        self.declare_parameter('toggle_button', 5)
        self.declare_parameter('speed_axis', 1)
        self.declare_parameter('steering_axis', 3)
        self.declare_parameter('max_speed', 1.5)
        self.declare_parameter('max_steering_angle', 0.34)
        self.declare_parameter('speed_deadzone', 0.05)
        self.declare_parameter('steering_deadzone', 0.05)
        self.declare_parameter('publish_rate', 30.0)
        self.declare_parameter('joy_timeout', 0.5)
        # How long to keep sending zeros after switching to autonomous, so the
        # car actually stops instead of coasting on the VESC command timeout.
        self.declare_parameter('stop_burst_sec', 0.5)

        self.teleop_topic = self.get_parameter('teleop_topic').value
        self.toggle_button = int(self.get_parameter('toggle_button').value)
        self.speed_axis = int(self.get_parameter('speed_axis').value)
        self.steering_axis = int(self.get_parameter('steering_axis').value)
        self.max_speed = float(self.get_parameter('max_speed').value)
        self.max_steering_angle = float(self.get_parameter('max_steering_angle').value)
        self.speed_deadzone = float(self.get_parameter('speed_deadzone').value)
        self.steering_deadzone = float(self.get_parameter('steering_deadzone').value)
        self.publish_rate = float(self.get_parameter('publish_rate').value)
        self.joy_timeout = float(self.get_parameter('joy_timeout').value)
        self.stop_burst_sec = float(self.get_parameter('stop_burst_sec').value)

        self.manual = True
        self.last_joy = Joy()
        self.last_joy_stamp = None
        self.last_toggle_state = 0
        self.stop_until = None

        self.publisher = self.create_publisher(
            AckermannDriveStamped, self.teleop_topic, 10)
        self.subscription = self.create_subscription(
            Joy, 'joy', self._joy_callback, 10)
        self.timer = self.create_timer(1.0 / self.publish_rate, self._publish)

        self.get_logger().info(
            f'MANUAL mode. Button {self.toggle_button} toggles manual/autonomous. '
            f'speed_axis={self.speed_axis} steering_axis={self.steering_axis} '
            f'max_speed={self.max_speed} max_steering_angle={self.max_steering_angle}')

    def _joy_callback(self, msg):
        self.last_joy = msg
        self.last_joy_stamp = self.get_clock().now()

        # Edge trigger: only act on the 0 -> 1 transition, otherwise holding the
        # button would flip the mode once per joy message.
        pressed = 0
        if 0 <= self.toggle_button < len(msg.buttons):
            pressed = msg.buttons[self.toggle_button]
        if pressed == 1 and self.last_toggle_state == 0:
            self.manual = not self.manual
            if self.manual:
                self.stop_until = None
                self.get_logger().info('--> MANUAL (joystick drives the car)')
            else:
                self.stop_until = self.get_clock().now() + rclpy.duration.Duration(
                    seconds=self.stop_burst_sec)
                self.get_logger().info('--> AUTONOMOUS (joystick ignored)')
        self.last_toggle_state = pressed

    def _axis(self, index):
        if 0 <= index < len(self.last_joy.axes):
            return float(self.last_joy.axes[index])
        return 0.0

    def _apply_deadzone(self, value, deadzone):
        if abs(value) < deadzone:
            return 0.0
        # Rescale so the command still reaches full range outside the deadzone.
        scaled = (abs(value) - deadzone) / (1.0 - deadzone)
        return scaled if value > 0.0 else -scaled

    def _publish(self):
        if not self.manual:
            # Zero burst right after the switch, then stay quiet so the mux can
            # fall through to the autonomy topic.
            if self.stop_until is not None:
                if self.get_clock().now() < self.stop_until:
                    self._send(0.0, 0.0)
                else:
                    self.stop_until = None
            return

        # No joystick yet, or the joystick went away: command a stop.
        if self.last_joy_stamp is None:
            return
        age = (self.get_clock().now() - self.last_joy_stamp).nanoseconds * 1e-9
        if age > self.joy_timeout:
            self._send(0.0, 0.0)
            return

        speed = self._apply_deadzone(
            self._axis(self.speed_axis), self.speed_deadzone) * self.max_speed
        steering = self._apply_deadzone(
            self._axis(self.steering_axis), self.steering_deadzone) * self.max_steering_angle
        self._send(speed, steering)

    def _send(self, speed, steering_angle):
        msg = AckermannDriveStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.drive.speed = speed
        msg.drive.steering_angle = steering_angle
        self.publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = JoyToggleTeleop()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
