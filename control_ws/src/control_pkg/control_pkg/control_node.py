import rclpy
from rclpy.node import Node


class ControlNode(Node):
    def __init__(self):
        super().__init__('control')
        self.declare_parameter('control_rate', 20)
        self.control_rate = self.get_parameter('control_rate').value
        self.sub = self.create_subscription(String, '/planning/plan', self.plan_cb, 10)
        self.pub = self.create_publisher(String, '/control/cmd', 10)
        self.srv = self.create_service(Trigger, '/control/set_mode', self.set_mode_cb)
        self.mode = 'idle'
        self.get_logger().info('Control node started')


def main(args=None):
    rclpy.init(args=args)
    node = ControlNode()
    def plan_cb(self, msg):
        self.get_logger().info(f'Received plan: {msg.data}')
        cmd = String()
        cmd.data = f'cmd_from_plan:{msg.data}'
        self.pub.publish(cmd)

    def set_mode_cb(self, request, response):
        # toggle between 'idle' and 'active'
        self.mode = 'active' if self.mode == 'idle' else 'idle'
        response.success = True
        response.message = f'mode_set:{self.mode}'
        self.get_logger().info(f'Control mode changed to {self.mode}')
        return response

    def main(args=None):
        rclpy.init(args=args)
        node = ControlNode()
        try:
            rclpy.spin(node)
        except KeyboardInterrupt:
            pass
        finally:
            node.destroy_node()
            rclpy.shutdown()


if __name__ == '__main__':
    main()


if __name__ == '__main__':
    main()
