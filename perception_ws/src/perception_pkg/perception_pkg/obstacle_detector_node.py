"""
Obstacle detector ROS2 node:
LaserScan -> preprocessing -> clustering -> tracking -> classification.
"""
import math

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from geometry_msgs.msg import Point, Vector3
from sensor_msgs.msg import LaserScan

from f1tenth_perception_msgs.msg import Obstacle, ObstacleArray

from .classifier import classify_track
from .clustering import cluster_points
from .preprocessing import preprocess_scan
from .tracker import Tracker


class ObstacleDetectorNode(Node):
    def __init__(self):
        super().__init__('obstacle_detector_node')

        self.declare_parameter('range_min', 0.05)
        self.declare_parameter('range_max', 10.0)
        self.declare_parameter('fov_min_deg', -180.0)
        self.declare_parameter('fov_max_deg', 180.0)
        self.declare_parameter('median_kernel', 3)
        self.declare_parameter('cluster_eps', 0.18)
        self.declare_parameter('min_cluster_size', 3)
        self.declare_parameter('gating_distance', 0.5)
        self.declare_parameter('track_timeout', 10)
        self.declare_parameter('v_thresh', 0.25)
        self.declare_parameter('n_confirm', 5)
        self.declare_parameter('n_release', 8)
        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('obstacle_topic', '/perception/obstacles')
        self.declare_parameter('frame_id_override', '')
        self.declare_parameter('min_publish_hits', 1)
        self.declare_parameter('boundary_polygon_xy', [])

        p = self.get_parameter
        self.range_min = p('range_min').value
        self.range_max = p('range_max').value
        self.fov_min = p('fov_min_deg').value * math.pi / 180.0
        self.fov_max = p('fov_max_deg').value * math.pi / 180.0
        self.median_kernel = p('median_kernel').value
        self.cluster_eps = p('cluster_eps').value
        self.min_cluster_size = p('min_cluster_size').value
        self.v_thresh = p('v_thresh').value
        self.n_confirm = p('n_confirm').value
        self.n_release = p('n_release').value
        self.frame_id_override = p('frame_id_override').value
        self.min_publish_hits = p('min_publish_hits').value
        self.boundary_polygon = self._parse_boundary_polygon(
            p('boundary_polygon_xy').value
        )

        self.tracker = Tracker(
            gating_distance=p('gating_distance').value,
            track_timeout=p('track_timeout').value,
        )

        scan_topic = p('scan_topic').value
        obstacle_topic = p('obstacle_topic').value

        self.sub = self.create_subscription(
            LaserScan, scan_topic, self.on_scan, qos_profile_sensor_data
        )
        self.pub = self.create_publisher(ObstacleArray, obstacle_topic, 10)

        self.get_logger().info(
            f'obstacle_detector_node started: {scan_topic} -> {obstacle_topic}'
        )
        if self.boundary_polygon is not None:
            self.get_logger().info(
                f'boundary polygon enabled with {len(self.boundary_polygon)} vertices'
            )

    def _parse_boundary_polygon(self, flat_xy):
        if flat_xy is None or len(flat_xy) == 0:
            return None
        if len(flat_xy) < 6 or len(flat_xy) % 2 != 0:
            self.get_logger().warn(
                'boundary_polygon_xy must contain an even number of values with at least 3 vertices. '
                'Boundary filter disabled.'
            )
            return None
        polygon = []
        for i in range(0, len(flat_xy), 2):
            polygon.append([float(flat_xy[i]), float(flat_xy[i + 1])])
        return polygon

    def on_scan(self, msg: LaserScan):
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9

        points = preprocess_scan(
            msg.ranges,
            msg.angle_min,
            msg.angle_increment,
            range_min=self.range_min,
            range_max=self.range_max,
            fov_min=self.fov_min,
            fov_max=self.fov_max,
            median_kernel=self.median_kernel,
            boundary_polygon=self.boundary_polygon,
        )

        clusters = cluster_points(
            points, eps=self.cluster_eps, min_cluster_size=self.min_cluster_size
        )
        active_tracks = self.tracker.step(clusters, stamp)

        obstacle_array = ObstacleArray()
        obstacle_array.header = msg.header
        if self.frame_id_override:
            obstacle_array.header.frame_id = self.frame_id_override

        for track in active_tracks:
            classify_track(
                track,
                v_thresh=self.v_thresh,
                n_confirm=self.n_confirm,
                n_release=self.n_release,
            )

            if track.hits < self.min_publish_hits:
                continue

            x, y = track.kf.position
            vx, vy = track.kf.velocity

            obs = Obstacle()
            obs.id = track.id
            obs.position = Point(x=float(x), y=float(y), z=0.0)
            obs.velocity = Vector3(x=float(vx), y=float(vy), z=0.0)
            obs.is_dynamic = bool(track.is_dynamic)
            obs.confidence = float(track.confidence)
            obs.radius = float(track.radius)
            obstacle_array.obstacles.append(obs)

        self.pub.publish(obstacle_array)


def main(args=None):
    rclpy.init(args=args)
    node = ObstacleDetectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
