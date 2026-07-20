"""
1) frame_id_override 제거.
   - 기존에는 좌표는 그대로 두고 header.frame_id 문자열만 바꿔치기해서,
     실제 좌표(laser frame)와 메시지에 적힌 프레임(map 등)이 어긋나는 버그가 있었음.
   - 이제는 target_frame 파라미터로 "실제로 변환해서 publish할 프레임"을 지정하고,
     TF2로 laser -> target_frame 실좌표 변환을 수행한 뒤 publish함.
2) boundary_polygon_xy의 의미 변경.
   - 기존: laser frame 기준 폴리곤 (차량이 움직이면 매번 다시 계산해줘야 하는데 그 로직이 없었음)
   - 변경: target_frame(보통 map) 기준 폴리곤. 트랙 경계처럼 고정된 지형은 map frame에
     한 번만 정의해두면 차량 위치와 무관하게 항상 올바르게 동작함.
3) TF lookup 실패 시 해당 스캔은 스킵하고 경고 로그만 남김 (잘못된 좌표로 publish하지 않음).
"""
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from rclpy.duration import Duration

import tf2_ros
from tf2_ros import LookupException, ConnectivityException, ExtrapolationException

from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Point, Vector3

from f1tenth_perception_msgs.msg import Obstacle, ObstacleArray

from perception_pkg.preprocessing import (
    preprocess_scan,
    transform_points,
    rotation_matrix_2d_from_quaternion,
    filter_points_in_polygon,
)
from perception_pkg.clustering import cluster_points
from perception_pkg.tracker import Tracker
from perception_pkg.classifier import classify_track


class ObstacleDetectorNode(Node):

    def __init__(self):
        super().__init__('obstacle_detector_node')

        # ---- 파라미터 선언 (config/obstacle_params.yaml에서 오버라이드) ----
        self.declare_parameter('range_min', 0.05)
        self.declare_parameter('range_max', 10.0)
        self.declare_parameter('fov_min_deg', -135.0)
        self.declare_parameter('fov_max_deg', 135.0)
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
        self.declare_parameter('target_frame', 'map')   # frame_id_override 대체
        self.declare_parameter('tf_timeout_sec', 0.05)
        self.declare_parameter('boundary_polygon_xy', [])  # target_frame(map) 기준

        p = self.get_parameter
        self.range_min = p('range_min').value
        self.range_max = p('range_max').value
        self.fov_min = p('fov_min_deg').value * np.pi / 180.0
        self.fov_max = p('fov_max_deg').value * np.pi / 180.0
        self.median_kernel = p('median_kernel').value

        self.cluster_eps = p('cluster_eps').value
        self.min_cluster_size = p('min_cluster_size').value

        self.v_thresh = p('v_thresh').value
        self.n_confirm = p('n_confirm').value
        self.n_release = p('n_release').value

        self.target_frame = p('target_frame').value
        self.tf_timeout = Duration(seconds=p('tf_timeout_sec').value)
        self.boundary_polygon = self._parse_boundary_polygon(
            p('boundary_polygon_xy').value
        )

        self.tracker = Tracker(
            gating_distance=p('gating_distance').value,
            track_timeout=p('track_timeout').value,
        )

        scan_topic = p('scan_topic').value
        obstacle_topic = p('obstacle_topic').value

        # ---- TF2 ----
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.sub = self.create_subscription(
            LaserScan, scan_topic, self.on_scan, qos_profile_sensor_data
        )
        self.pub = self.create_publisher(ObstacleArray, obstacle_topic, 10)

        self.get_logger().info(
            f'obstacle_detector_node started: {scan_topic} -> {obstacle_topic} '
            f'(target_frame={self.target_frame})'
        )
        if self.boundary_polygon is not None:
            self.get_logger().info(
                f'boundary polygon enabled with {len(self.boundary_polygon)} vertices '
                f'(in {self.target_frame} frame)'
            )

    def _parse_boundary_polygon(self, flat_xy):
        """[x1, y1, x2, y2, ...] 형식 파라미터를 (N, 2)로 변환."""
        if flat_xy is None or len(flat_xy) == 0:
            return None
        if len(flat_xy) < 6 or len(flat_xy) % 2 != 0:
            self.get_logger().warn(
                'boundary_polygon_xy must contain an even number of values '
                'with at least 3 vertices. Boundary filter disabled.'
            )
            return None
        polygon = []
        for i in range(0, len(flat_xy), 2):
            polygon.append([float(flat_xy[i]), float(flat_xy[i + 1])])
        return polygon

    def on_scan(self, msg: LaserScan):
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        source_frame = msg.header.frame_id

        # ① 전처리 (laser frame 기준, range/FOV/median filter만)
        points = preprocess_scan(
            msg.ranges, msg.angle_min, msg.angle_increment,
            range_min=self.range_min, range_max=self.range_max,
            fov_min=self.fov_min, fov_max=self.fov_max,
            median_kernel=self.median_kernel,
        )
        if len(points) == 0:
            return

        # ①.5 TF 변환: laser frame -> target_frame (보통 map)
        # 이 변환이 있어야 이후 boundary_polygon(map 기준)과 최종 publish 좌표가 실제로 맞음.
        try:
            tf_stamped = self.tf_buffer.lookup_transform(
                self.target_frame, source_frame, Time(), timeout=self.tf_timeout
            )
        except (LookupException, ConnectivityException, ExtrapolationException) as ex:
            self.get_logger().warn(
                f'TF lookup {source_frame} -> {self.target_frame} failed: {ex}. '
                'skipping this scan.',
                throttle_duration_sec=2.0,
            )
            return

        t = tf_stamped.transform.translation
        q = tf_stamped.transform.rotation
        R = rotation_matrix_2d_from_quaternion(q.x, q.y, q.z, q.w)
        points = transform_points(points, [t.x, t.y], R)

        # ②.5 경계 필터 (이제 target_frame 기준이므로 정적 폴리곤을 그대로 써도 됨)
        if self.boundary_polygon is not None:
            points = filter_points_in_polygon(points, self.boundary_polygon)
        if len(points) == 0:
            return

        # ② 클러스터링
        clusters = cluster_points(
            points, eps=self.cluster_eps, min_cluster_size=self.min_cluster_size
        )

        # ③④ 매칭 + 칼만필터 트래킹
        active_tracks = self.tracker.step(clusters, stamp)

        # ⑤ 정적/동적 판별
        obstacle_array = ObstacleArray()
        obstacle_array.header = msg.header
        obstacle_array.header.frame_id = self.target_frame  # 실제로 변환된 프레임과 항상 일치

        for track in active_tracks:
            classify_track(
                track, v_thresh=self.v_thresh,
                n_confirm=self.n_confirm, n_release=self.n_release,
            )

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


if __name__ == '__main__':
    main()
