from math import copysign

from ackermann_msgs.msg import AckermannDriveStamped
from rclpy.duration import Duration
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy


class AckermannJoyTeleop(Node):
    def __init__(self) -> None:
        super().__init__('ackermann_joy_teleop')

        self.joy_topic = self.declare_parameter('joy_topic', '/joy').value
        self.ackermann_topic = self.declare_parameter('ackermann_topic', '/ackermann_cmd').value
        self.speed_axis = int(self.declare_parameter('speed_axis', 1).value)
        self.steering_axis = int(self.declare_parameter('steering_axis', 3).value)
        self.reverse_button = int(self.declare_parameter('reverse_button', 0).value)
        self.max_speed = float(self.declare_parameter('max_speed', 1.5).value)
        self.max_speed_reverse = float(self.declare_parameter('max_speed_reverse', 1.0).value)
        self.max_steering_angle = float(self.declare_parameter('max_steering_angle', 0.4189).value)
        self.joy_timeout = float(self.declare_parameter('joy_timeout', 0.5).value)
        self.publish_rate = float(self.declare_parameter('publish_rate', 30.0).value)
        self.speed_deadzone = float(self.declare_parameter('speed_deadzone', 0.05).value)
        self.steering_deadzone = float(self.declare_parameter('steering_deadzone', 0.05).value)
        # 0이면 램프 미적용 (즉시 목표 속도로 점프). 급가속 시 VESC openloop 구간에서
        # 못 따라가 덜컥거리는 것을 막기 위해 가속 방향으로만 속도를 서서히 올림.
        self.max_accel = float(self.declare_parameter('max_accel', 3.0).value)

        self.last_joy_msg = Joy()
        self.last_joy_stamp = None
        self.last_commanded_speed = 0.0

        self.publisher = self.create_publisher(AckermannDriveStamped, self.ackermann_topic, 10)
        self.subscription = self.create_subscription(Joy, self.joy_topic, self._joy_callback, 10)
        self.timer = self.create_timer(1.0 / self.publish_rate, self._publish_drive)

        self.get_logger().info(
            f'Publishing {self.ackermann_topic} from {self.joy_topic} '
            f'(speed_axis={self.speed_axis}, steering_axis={self.steering_axis})'
        )

    def _joy_callback(self, msg: Joy) -> None:
        self.last_joy_msg = msg
        self.last_joy_stamp = self.get_clock().now()

    def _axis_value(self, index: int) -> float:
        if 0 <= index < len(self.last_joy_msg.axes):
            return float(self.last_joy_msg.axes[index])
        return 0.0

    def _button_pressed(self, index: int) -> bool:
        if 0 <= index < len(self.last_joy_msg.buttons):
            return self.last_joy_msg.buttons[index] == 1
        return False

    def _apply_deadzone(self, value: float, deadzone: float) -> float:
        if abs(value) < deadzone:
            return 0.0
        scaled = (abs(value) - deadzone) / (1.0 - deadzone)
        return copysign(scaled, value)

    def _joy_is_stale(self) -> bool:
        if self.last_joy_stamp is None:
            return True
        return (self.get_clock().now() - self.last_joy_stamp) > Duration(seconds=self.joy_timeout)

    def _publish_drive(self) -> None:
        drive = AckermannDriveStamped()
        drive.header.stamp = self.get_clock().now().to_msg()

        if self._joy_is_stale():
            self.last_commanded_speed = 0.0
            self.publisher.publish(drive)
            return

        speed_input = self._apply_deadzone(self._axis_value(self.speed_axis), self.speed_deadzone)
        steering_input = self._apply_deadzone(self._axis_value(self.steering_axis), self.steering_deadzone)

        if self._button_pressed(self.reverse_button):
            target_speed = speed_input * -abs(self.max_speed_reverse)
        else:
            target_speed = speed_input * self.max_speed

        drive.drive.speed = self._ramp_speed(target_speed)
        drive.drive.steering_angle = steering_input * self.max_steering_angle
        self.publisher.publish(drive)

    def _ramp_speed(self, target_speed: float) -> float:
        if self.max_accel <= 0.0:
            self.last_commanded_speed = target_speed
            return target_speed

        max_delta = self.max_accel / self.publish_rate
        accelerating = abs(target_speed) > abs(self.last_commanded_speed)
        speed_diff = target_speed - self.last_commanded_speed

        if accelerating and abs(speed_diff) > max_delta:
            target_speed = self.last_commanded_speed + copysign(max_delta, speed_diff)

        self.last_commanded_speed = target_speed
        return target_speed


def main() -> None:
    rclpy.init()
    node = AckermannJoyTeleop()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
