import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from std_msgs.msg import Bool


class EStopNode(Node):
    def __init__(self):
        super().__init__('estop_node')
        self.declare_parameter('joy_topic', '/joy')
        self.declare_parameter('estop_topic', '/safety/emergency_stop')
        self.declare_parameter('button_index', 4)
        self.declare_parameter('latch_estop', True)

        joy_topic = self.get_parameter('joy_topic').value
        estop_topic = self.get_parameter('estop_topic').value
        self.button_index = self.get_parameter('button_index').value
        self.latch_estop = self.get_parameter('latch_estop').value
        self.estop_active = False

        self.publisher = self.create_publisher(Bool, estop_topic, 10)
        self.subscription = self.create_subscription(Joy, joy_topic, self.joy_cb, 10)
        self.get_logger().info('Emergency stop node started')

    def joy_cb(self, msg):
        if self.button_index >= len(msg.buttons):
            self.get_logger().warn('Configured e-stop button index is out of range')
            return

        button_pressed = msg.buttons[self.button_index] == 1

        if button_pressed:
            if self.latch_estop:
                self.estop_active = True
            else:
                self.estop_active = not self.estop_active

            estop_msg = Bool()
            estop_msg.data = self.estop_active
            self.publisher.publish(estop_msg)
            self.get_logger().warn(f'E-stop state changed: {self.estop_active}')


def main(args=None):
    rclpy.init(args=args)
    node = EStopNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
