import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from example_interfaces.srv import Trigger


class IntegrationNode(Node):
    def __init__(self):
        super().__init__('integration')
        self.declare_parameter('use_sim_time', False)
        self.declare_parameter('planning_service', '/planning/compute_plan')
        self.declare_parameter('control_service', '/control/set_mode')
        self.declare_parameter('perception_topic', '/perception/objects')
        self.declare_parameter('service_check_timeout', 1.0)
        self.declare_parameter('timer_period', 5.0)
        self.declare_parameter('system_mode', 'sim')
        self.declare_parameter('slam_enabled', False)
        self.declare_parameter('map_mode', False)
        self.declare_parameter('localization_mode', 'none')
        self.declare_parameter('localization_map_file', '')
        self.declare_parameter('localization_scan_topic', '/scan')
        self.declare_parameter('localization_odom_topic', '/odom')
        planning_service = self.get_parameter('planning_service').value
        control_service = self.get_parameter('control_service').value
        perception_topic = self.get_parameter('perception_topic').value
        self.service_check_timeout = self.get_parameter('service_check_timeout').value
        timer_period = self.get_parameter('timer_period').value
        self.cli_plan = self.create_client(Trigger, planning_service)
        self.cli_mode = self.create_client(Trigger, control_service)
        self.sub = self.create_subscription(String, perception_topic, self.objects_cb, 10)
        self.timer = self.create_timer(timer_period, self.timer_cb)
        self.get_logger().info('Integration node started')

    def objects_cb(self, msg):
        self.get_logger().info(f'Integration received: {msg.data}')

    def timer_cb(self):
        # call planning service
        if self.cli_plan.wait_for_service(timeout_sec=self.service_check_timeout):
            req = Trigger.Request()
            future = self.cli_plan.call_async(req)
            future.add_done_callback(self.plan_response)
        else:
            self.get_logger().warn('Planning service not available')

        # call control service
        if self.cli_mode.wait_for_service(timeout_sec=self.service_check_timeout):
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
