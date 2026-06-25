import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class PerceptionNode(Node):
    def __init__(self):
        super().__init__('perception')
        self.declare_parameter('detection_threshold', 0.5)
        self.declare_parameter('publish_topic', '/perception/objects')
        self.declare_parameter('publish_period', 1.0)
        self.detection_threshold = self.get_parameter('detection_threshold').value
        publish_topic = self.get_parameter('publish_topic').value
        publish_period = self.get_parameter('publish_period').value
        self.pub = self.create_publisher(String, publish_topic, 10)
        self.timer = self.create_timer(publish_period, self.timer_callback)
        self.get_logger().info('Perception node started')

    def timer_callback(self):
        msg = String()
        msg.data = f"object_detected:confidence={self.detection_threshold}"
        self.pub.publish(msg)
        self.get_logger().debug(f'Published: {msg.data}')


def main(args=None):
    rclpy.init(args=args)
    node = PerceptionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
