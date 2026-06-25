import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from example_interfaces.srv import Trigger


class ControlNode(Node):
    def __init__(self):
        super().__init__('control')
        self.declare_parameter('control_rate', 20)
        self.declare_parameter('input_topic', '/planning/plan')
        self.declare_parameter('output_topic', '/control/cmd')
        self.declare_parameter('service_name', '/control/set_mode')
        self.control_rate = self.get_parameter('control_rate').value
        input_topic = self.get_parameter('input_topic').value
        output_topic = self.get_parameter('output_topic').value
        service_name = self.get_parameter('service_name').value
        self.sub = self.create_subscription(String, input_topic, self.plan_cb, 10)
        self.pub = self.create_publisher(String, output_topic, 10)
        self.srv = self.create_service(Trigger, service_name, self.set_mode_cb)
        self.mode = 'idle'
        self.get_logger().info('Control node started')

    def plan_cb(self, msg):
        self.get_logger().info(f'Received plan: {msg.data}')
        cmd = String()
        cmd.data = f'cmd_from_plan:{msg.data}'
        self.pub.publish(cmd)

    def set_mode_cb(self, request, response):
        del request
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
