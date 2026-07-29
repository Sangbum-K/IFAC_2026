"""Joystick teleop with an edge-triggered manual/autonomous/estop state machine.

Replaces the upstream joy_teleop node, which only supports hold-to-drive
deadman buttons. Here the mode is latched: pressing the toggle button once
switches manual <-> autonomous and the stick works without holding anything.

manual      -- joystick axes are published to the teleop topic continuously.
autonomous  -- a short burst of zero commands is sent to bring the car to a
               stop, then this node goes silent so the mux timeout lets the
               autonomy topic (lower priority) through.
estop       -- entered from either mode by the estop button, regardless of
               manual/autonomous. Zero commands are published continuously on
               a dedicated topic with the highest mux priority, so the mux
               can't just fall silent and let the last real command coast
               through (a lock-only design would do exactly that). The estop
               button also flips /safety/emergency_stop for control_node /
               recovery_supervisor_node, which is a second, independent stop
               path. Releasing estop always lands in manual, never back into
               autonomous -- the mode button is ignored entirely while
               estopped.
"""

import rclpy
from rclpy.node import Node

from ackermann_msgs.msg import AckermannDriveStamped
from sensor_msgs.msg import Joy
from std_msgs.msg import Bool

MANUAL = 'manual'
AUTONOMOUS = 'autonomous'
ESTOP = 'estop'


class JoyToggleTeleop(Node):
    def __init__(self):
        super().__init__('joy_toggle_teleop')

        self.declare_parameter('teleop_topic', 'teleop')
        self.declare_parameter('toggle_button', 5)
        self.declare_parameter('estop_button', 4)
        self.declare_parameter('estop_topic', '/estop_cmd')
        self.declare_parameter('estop_status_topic', '/safety/emergency_stop')
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
        self.estop_button = int(self.get_parameter('estop_button').value)
        self.estop_topic = self.get_parameter('estop_topic').value
        self.estop_status_topic = self.get_parameter('estop_status_topic').value
        self.speed_axis = int(self.get_parameter('speed_axis').value)
        self.steering_axis = int(self.get_parameter('steering_axis').value)
        self.max_speed = float(self.get_parameter('max_speed').value)
        self.max_steering_angle = float(self.get_parameter('max_steering_angle').value)
        self.speed_deadzone = float(self.get_parameter('speed_deadzone').value)
        self.steering_deadzone = float(self.get_parameter('steering_deadzone').value)
        self.publish_rate = float(self.get_parameter('publish_rate').value)
        self.joy_timeout = float(self.get_parameter('joy_timeout').value)
        self.stop_burst_sec = float(self.get_parameter('stop_burst_sec').value)

        self.state = MANUAL
        self.last_joy = Joy()
        self.last_joy_stamp = None
        self.last_toggle_state = 0
        self.last_estop_state = 0
        self.stop_until = None

        self.publisher = self.create_publisher(
            AckermannDriveStamped, self.teleop_topic, 10)
        self.estop_publisher = self.create_publisher(
            AckermannDriveStamped, self.estop_topic, 10)
        self.estop_status_publisher = self.create_publisher(
            Bool, self.estop_status_topic, 10)
        self.subscription = self.create_subscription(
            Joy, 'joy', self._joy_callback, 10)
        self.timer = self.create_timer(1.0 / self.publish_rate, self._publish)

        self.get_logger().info(
            f'MANUAL mode. Button {self.estop_button} is estop (toggle, always '
            f'wins). Button {self.toggle_button} toggles manual/autonomous '
            f'(ignored while estopped). speed_axis={self.speed_axis} '
            f'steering_axis={self.steering_axis} max_speed={self.max_speed} '
            f'max_steering_angle={self.max_steering_angle}')

    def _edge(self, msg, button_index, last_state_attr):
        pressed = 0
        if 0 <= button_index < len(msg.buttons):
            pressed = msg.buttons[button_index]
        was_pressed = getattr(self, last_state_attr)
        setattr(self, last_state_attr, pressed)
        return pressed == 1 and was_pressed == 0

    def _joy_callback(self, msg):
        self.last_joy = msg
        self.last_joy_stamp = self.get_clock().now()

        # Estop edge is checked first and wins outright: it can fire from any
        # state, and while estopped it's the only button this node reacts to.
        if self._edge(msg, self.estop_button, 'last_estop_state'):
            if self.state == ESTOP:
                self._enter_manual()
                self.get_logger().info('--> ESTOP released, back to MANUAL')
            else:
                self.state = ESTOP
                self.get_logger().info('--> ESTOP (highest priority, ignoring joystick)')
            self.estop_status_publisher.publish(Bool(data=self.state == ESTOP))
            return

        if self.state == ESTOP:
            # Mode button is ignored entirely while estopped.
            return

        if self._edge(msg, self.toggle_button, 'last_toggle_state'):
            if self.state == AUTONOMOUS:
                self._enter_manual()
                self.get_logger().info('--> MANUAL (joystick drives the car)')
            else:
                self.state = AUTONOMOUS
                self.stop_until = self.get_clock().now() + rclpy.duration.Duration(
                    seconds=self.stop_burst_sec)
                self.get_logger().info('--> AUTONOMOUS (joystick ignored)')

    def _enter_manual(self):
        self.state = MANUAL
        self.stop_until = None

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
        # Heartbeat the estop status on every tick (not just on the toggle
        # edge) so a subscriber that starts late, or a single dropped
        # message, can't end up stuck believing the wrong state.
        self.estop_status_publisher.publish(Bool(data=self.state == ESTOP))

        if self.state == ESTOP:
            # Highest-priority topic in the mux: published continuously so
            # the mux always has a fresh zero command to fall back on,
            # instead of relying on the joystick/autonomy topics going quiet.
            self._send(0.0, 0.0, self.estop_publisher)
            return

        if self.state == AUTONOMOUS:
            # Zero burst right after the switch, then stay quiet so the mux can
            # fall through to the autonomy topic.
            if self.stop_until is not None:
                if self.get_clock().now() < self.stop_until:
                    self._send(0.0, 0.0, self.publisher)
                else:
                    self.stop_until = None
            return

        # MANUAL. No joystick yet, or the joystick went away: command a stop.
        if self.last_joy_stamp is None:
            return
        age = (self.get_clock().now() - self.last_joy_stamp).nanoseconds * 1e-9
        if age > self.joy_timeout:
            self._send(0.0, 0.0, self.publisher)
            return

        speed = self._apply_deadzone(
            self._axis(self.speed_axis), self.speed_deadzone) * self.max_speed
        steering = self._apply_deadzone(
            self._axis(self.steering_axis), self.steering_deadzone) * self.max_steering_angle
        self._send(speed, steering, self.publisher)

    def _send(self, speed, steering_angle, publisher):
        msg = AckermannDriveStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.drive.speed = speed
        msg.drive.steering_angle = steering_angle
        publisher.publish(msg)


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
