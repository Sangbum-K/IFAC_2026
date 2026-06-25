import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from example_interfaces.srv import Trigger


class IntegrationNode(Node):
    def __init__(self):
        super().__init__('integration')
        self.cli_plan = self.create_client(Trigger, '/planning/compute_plan')
        self.cli_mode = self.create_client(Trigger, '/control/set_mode')
        self.sub = self.create_subscription(String, '/perception/objects', self.objects_cb, 10)
        self.timer = self.create_timer(5.0, self.timer_cb)
        self.get_logger().info('Integration node started')

    def objects_cb(self, msg):
        self.get_logger().info(f'Integration received: {msg.data}')

    def timer_cb(self):
        # call planning service
        if self.cli_plan.wait_for_service(timeout_sec=1.0):
            req = Trigger.Request()
            future = self.cli_plan.call_async(req)
            future.add_done_callback(self.plan_response)
        else:
            self.get_logger().warn('Planning service not available')

        # call control service
        if self.cli_mode.wait_for_service(timeout_sec=1.0):
            req2 = Trigger.Request()
            future2 = self.cli_mode.call_async(req2)
            future2.add_done_callback(self.mode_response)
        else:
            self.get_logger().warn('Control service not available')

    def plan_response(self, future):
        try:
            res = future.result()
            self.get_logger().info(f'Planning service replied: success={res.success}, msg={res.message}')
        except Exception as e:
            self.get_logger().error(f'Planning service call failed: {e}')

    def mode_response(self, future):
        try:
            res = future.result()
            self.get_logger().info(f'Control service replied: success={res.success}, msg={res.message}')
        except Exception as e:
            self.get_logger().error(f'Control service call failed: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = IntegrationNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
