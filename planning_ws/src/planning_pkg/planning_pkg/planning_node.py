import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from example_interfaces.srv import Trigger


class PlanningNode(Node):
    def __init__(self):
        super().__init__('planning')
        self.declare_parameter('planning_horizon', 5.0)
        self.planning_horizon = self.get_parameter('planning_horizon').value
        self.sub = self.create_subscription(String, '/perception/objects', self.objects_cb, 10)
        self.pub = self.create_publisher(String, '/planning/plan', 10)
        self.srv = self.create_service(Trigger, '/planning/compute_plan', self.compute_plan_cb)
        self.get_logger().info('Planning node started')

    def objects_cb(self, msg):
        self.get_logger().info(f'Received objects: {msg.data}')
        plan = String()
        plan.data = f'plan_based_on:{msg.data}'
        self.pub.publish(plan)

    def compute_plan_cb(self, request, response):
        response.success = True
        response.message = f'Computed plan (horizon={self.planning_horizon})'
        self.get_logger().info('compute_plan service called')
        return response


def main(args=None):
    rclpy.init(args=args)
    node = PlanningNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
