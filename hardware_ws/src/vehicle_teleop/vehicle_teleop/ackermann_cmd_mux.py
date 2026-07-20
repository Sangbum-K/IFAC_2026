from ackermann_msgs.msg import AckermannDriveStamped
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from sensor_msgs.msg import Joy
from std_msgs.msg import Bool, String


class AckermannCmdMux(Node):
    def __init__(self) -> None:
        super().__init__('ackermann_cmd_mux')

        self.joy_topic = self.declare_parameter('joy_topic', '/joy').value
        self.manual_topic = self.declare_parameter(
            'manual_ackermann_topic', '/manual_ackermann_cmd'
        ).value
        self.autonomous_topic = self.declare_parameter(
            'autonomous_ackermann_topic', '/autonomy/ackermann_cmd'
        ).value
        self.output_topic = self.declare_parameter(
            'output_ackermann_topic', '/ackermann_cmd'
        ).value
        self.mode_topic = self.declare_parameter(
            'mode_topic', '/ackermann_cmd_mux/mode'
        ).value
        # 자율/수동 토글 버튼 (엣지 감지 - 누르는 순간 한 번만 상태 반전).
        self.toggle_button = int(self.declare_parameter('autonomous_button', 5).value)
        self.estop_topic = self.declare_parameter(
            'estop_topic', '/safety/emergency_stop'
        ).value
        self.command_timeout = float(self.declare_parameter('command_timeout', 0.5).value)
        self.publish_rate = float(self.declare_parameter('publish_rate', 30.0).value)

        self.last_joy_msg = Joy()
        self.last_manual_msg = AckermannDriveStamped()
        self.last_autonomous_msg = AckermannDriveStamped()
        self.last_manual_stamp = None
        self.last_autonomous_stamp = None
        self.last_mode = 'manual'
        self.estop_active = False
        self.toggle_autonomous = False
        self.prev_toggle_pressed = False

        self.publisher = self.create_publisher(AckermannDriveStamped, self.output_topic, 10)
        self.mode_publisher = self.create_publisher(String, self.mode_topic, 10)
        self.create_subscription(Joy, self.joy_topic, self._joy_callback, 10)
        self.create_subscription(
            AckermannDriveStamped, self.manual_topic, self._manual_callback, 10
        )
        self.create_subscription(
            AckermannDriveStamped, self.autonomous_topic, self._autonomous_callback, 10
        )
        self.create_subscription(Bool, self.estop_topic, self._estop_callback, 10)
        self.timer = self.create_timer(1.0 / self.publish_rate, self._publish_selected_command)

        self.get_logger().info(
            f'Muxing manual={self.manual_topic}, auto={self.autonomous_topic} -> {self.output_topic} '
            f'with toggle button {self.toggle_button}, estop {self.estop_topic}'
        )

    def _joy_callback(self, msg: Joy) -> None:
        self.last_joy_msg = msg

    def _estop_callback(self, msg: Bool) -> None:
        self.estop_active = msg.data

    def _manual_callback(self, msg: AckermannDriveStamped) -> None:
        self.last_manual_msg = msg
        self.last_manual_stamp = self.get_clock().now()

    def _autonomous_callback(self, msg: AckermannDriveStamped) -> None:
        self.last_autonomous_msg = msg
        self.last_autonomous_stamp = self.get_clock().now()

    def _button_pressed(self, index: int) -> bool:
        if 0 <= index < len(self.last_joy_msg.buttons):
            return self.last_joy_msg.buttons[index] == 1
        return False

    def _is_fresh(self, stamp) -> bool:
        if stamp is None:
            return False
        return (self.get_clock().now() - stamp) <= Duration(seconds=self.command_timeout)

    def _update_toggle_state(self) -> None:
        current = self._button_pressed(self.toggle_button)
        if current and not self.prev_toggle_pressed:
            self.toggle_autonomous = not self.toggle_autonomous
        self.prev_toggle_pressed = current

    def _publish_selected_command(self) -> None:
        self._update_toggle_state()

        if self.estop_active:
            # e-stop은 manual/autonomous 어느 쪽이든 무조건 우선 - 정지 명령으로 덮어씀.
            msg = AckermannDriveStamped()
            mode = 'estop'
        elif self.toggle_autonomous and self._is_fresh(self.last_autonomous_stamp):
            msg = self.last_autonomous_msg
            mode = 'autonomous'
        elif self._is_fresh(self.last_manual_stamp):
            msg = self.last_manual_msg
            mode = 'manual'
        else:
            msg = AckermannDriveStamped()
            mode = 'idle'

        msg.header.stamp = self.get_clock().now().to_msg()
        self.publisher.publish(msg)
        self.mode_publisher.publish(String(data=mode))

        if mode != self.last_mode:
            self.get_logger().info(f'Command source switched to {mode}')
            self.last_mode = mode


def main() -> None:
    rclpy.init()
    node = AckermannCmdMux()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
