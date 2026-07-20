#!/usr/bin/env python3
"""
obstacle_detector_ros2.py

LiDAR 기반 장애물 인지(Obstacle Detection) 노드 - ROS2 (rclpy) 버전.

ForzaETH race_stack (perception/abd_tracker/opponent_tracker/src/detect.py, MIT License,
https://github.com/ForzaETH/race_stack) 의 detection 알고리즘을 기반으로 재작성했습니다.
이전에 만든 ROS1 버전(obstacle_detector.py)을 rclpy 기반으로 포팅한 것입니다.

ROS1 -> ROS2 변경 사항:
  - rospy            -> rclpy (Node 클래스 기반)
  - tf / TransformListener -> tf2_ros.Buffer + tf2_ros.TransformListener
  - tf.transformations.quaternion_matrix / quaternion_from_euler
    -> 외부 tf_transformations 패키지에 의존하지 않도록 numpy로 직접 구현
       (환경에 따라 tf_transformations가 안 깔려있는 경우가 많아서 의도적으로 제거함)
  - dynamic_reconfigure -> ROS2 파라미터 콜백(add_on_set_parameters_callback)으로 대체
  - rospy.Rate 기반 while 루프 -> create_timer 기반 콜백 (ROS2 권장 방식)

알고리즘 단계는 이전 버전과 동일합니다:
  1) LiDAR 스캔 -> map 좌표계 포인트클라우드 변환
  2) adaptive breakpoint segmentation 으로 클러스터 분리
  3) 클러스터 크기 / 차량으로부터의 거리로 필터링
  4) L-shape(사각형) fitting 으로 중심좌표 / 크기 / 방향 추출
  5) PoseArray / Float32MultiArray / MarkerArray 로 publish

필요 패키지: rclpy, tf2_ros, tf2_geometry_msgs(불필요 - 직접 변환),
             sensor_msgs, geometry_msgs, std_msgs, visualization_msgs, nav_msgs
"""

import math
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from rclpy.duration import Duration

import tf2_ros
from tf2_ros import LookupException, ConnectivityException, ExtrapolationException

from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose, PoseArray, Point
from std_msgs.msg import Float32MultiArray, Float32
from visualization_msgs.msg import Marker, MarkerArray


# --------------------------------------------------------------------- #
# 작은 쿼터니언 유틸 (tf_transformations 의존성 제거용)
# --------------------------------------------------------------------- #
def yaw_to_quaternion(yaw):
    """yaw(rad) -> (x, y, z, w) 쿼터니언 (roll=pitch=0 가정)"""
    return 0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0)


def quaternion_to_rotation_matrix(x, y, z, w):
    """쿼터니언 -> 3x3 회전행렬 (numpy)"""
    return np.array(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ]
    )


class Obstacle:
    """검출된 장애물 하나를 표현하는 자료구조"""

    def __init__(self, x, y, size, theta):
        self.center_x = x
        self.center_y = y
        self.size = size      # 정사각형으로 가정한 한 변의 길이 [m]
        self.theta = theta    # 장애물의 방향 [rad]
        self.id = None


class ObstacleDetector(Node):
    """
    LiDAR 스캔으로부터 장애물을 검출하는 ROS2 노드.

    Subscribes:
        scan_topic (default '/scan')  : sensor_msgs/LaserScan
        odom_topic (default '/odom')  : nav_msgs/Odometry (차량 위치, 거리 필터링용. 없어도 동작함)

    Publishes:
        obstacles/poses   (geometry_msgs/PoseArray)       : 장애물 중심 좌표 + 방향
        obstacles/sizes   (std_msgs/Float32MultiArray)    : 장애물 크기 (poses와 같은 순서)
        obstacles/markers (visualization_msgs/MarkerArray): rviz 시각화용
        obstacles/latency (std_msgs/Float32, optional)     : 처리 지연시간 [s] (measure:=true 일 때)
    """

    def __init__(self):
        super().__init__("obstacle_detector")

        # --- 파라미터 선언 (ROS2는 명시적 declare가 필요) ---
        self.declare_parameter("map_frame", "map")
        self.declare_parameter("laser_frame", "laser")
        self.declare_parameter("scan_topic", "/scan")
        self.declare_parameter("odom_topic", "/odom")

        self.declare_parameter("rate", 40.0)
        self.declare_parameter("lambda_deg", 10.0)
        self.declare_parameter("sigma", 0.03)
        self.declare_parameter("min_2_points_dist", 0.01)

        self.declare_parameter("min_obs_size", 10)
        self.declare_parameter("max_obs_size", 0.5)
        self.declare_parameter("max_viewing_distance", 9.0)

        self.declare_parameter("measure", False)

        self.map_frame = self.get_parameter("map_frame").value
        self.laser_frame = self.get_parameter("laser_frame").value
        scan_topic = self.get_parameter("scan_topic").value
        odom_topic = self.get_parameter("odom_topic").value

        self.rate_hz = self.get_parameter("rate").value
        self.lambda_angle = math.radians(self.get_parameter("lambda_deg").value)
        self.sigma = self.get_parameter("sigma").value
        self.min_2_points_dist = self.get_parameter("min_2_points_dist").value

        self.min_obs_size = self.get_parameter("min_obs_size").value
        self.max_obs_size = self.get_parameter("max_obs_size").value
        self.max_viewing_distance = self.get_parameter("max_viewing_distance").value

        self.measuring = self.get_parameter("measure").value

        # 런타임에 `ros2 param set /obstacle_detector min_obs_size 15` 등으로 값 변경 시 반영
        self.add_on_set_parameters_callback(self._on_param_change)

        # --- 상태 변수 ---
        self.scans = None
        self.current_stamp = None
        self.car_x, self.car_y = 0.0, 0.0
        self.tracked_obstacles = []

        # --- TF2 ---
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # --- Subscribers ---
        self.create_subscription(LaserScan, scan_topic, self._laser_cb, qos_profile_sensor_data)
        self.create_subscription(Odometry, odom_topic, self._odom_cb, 10)

        # --- Publishers ---
        self.poses_pub = self.create_publisher(PoseArray, "obstacles/poses", 5)
        self.sizes_pub = self.create_publisher(Float32MultiArray, "obstacles/sizes", 5)
        self.markers_pub = self.create_publisher(MarkerArray, "obstacles/markers", 5)
        if self.measuring:
            self.latency_pub = self.create_publisher(Float32, "obstacles/latency", 5)

        # --- 메인 처리 타이머 (ROS1의 while+rate.sleep 대신) ---
        self.create_timer(1.0 / self.rate_hz, self._on_timer)

        self.get_logger().info("[ObstacleDetector] Ready")

    # ------------------------------------------------------------------ #
    # Callbacks
    # ------------------------------------------------------------------ #
    def _laser_cb(self, msg):
        self.scans = msg

    def _odom_cb(self, msg):
        self.car_x = msg.pose.pose.position.x
        self.car_y = msg.pose.pose.position.y

    def _on_param_change(self, params):
        for p in params:
            if p.name == "min_obs_size":
                self.min_obs_size = p.value
            elif p.name == "max_obs_size":
                self.max_obs_size = p.value
            elif p.name == "max_viewing_distance":
                self.max_viewing_distance = p.value
        from rcl_interfaces.msg import SetParametersResult
        return SetParametersResult(successful=True)

    def _on_timer(self):
        if self.scans is None:
            return

        start = time.perf_counter() if self.measuring else None

        clusters = self._scan_to_clusters()
        obstacles = self._clusters_to_obstacles(clusters)
        self._filter_by_size(obstacles)

        if self.measuring:
            lat = Float32()
            lat.data = time.perf_counter() - start
            self.latency_pub.publish(lat)

        self._publish_poses_and_sizes()
        self._publish_markers()

    # ------------------------------------------------------------------ #
    # Core algorithm (ROS1 버전과 동일한 로직)
    # ------------------------------------------------------------------ #
    @staticmethod
    def _clear_markers():
        marker = Marker()
        marker.action = Marker.DELETEALL
        return [marker]

    def _scan_to_clusters(self):
        scan = self.scans
        d_phi = scan.angle_increment
        l_ang = self.lambda_angle
        sigma = self.sigma

        self.current_stamp = self.get_clock().now().to_msg()

        try:
            tf_stamped = self.tf_buffer.lookup_transform(
                self.map_frame, self.laser_frame, Time(), timeout=Duration(seconds=0.05)
            )
        except (LookupException, ConnectivityException, ExtrapolationException) as ex:
            self.get_logger().error(
                f"[ObstacleDetector] TF lookup {self.map_frame} -> {self.laser_frame} failed: {ex}",
                throttle_duration_sec=2.0,
            )
            return []

        t = tf_stamped.transform.translation
        q = tf_stamped.transform.rotation
        trans = np.array([t.x, t.y, t.z])
        R3 = quaternion_to_rotation_matrix(q.x, q.y, q.z, q.w)

        ranges = np.array(scan.ranges)
        angles = np.linspace(scan.angle_min, scan.angle_max, len(ranges))

        valid = np.isfinite(ranges) & (ranges >= scan.range_min) & (ranges <= scan.range_max)
        ranges = ranges[valid]
        angles = angles[valid]
        if len(ranges) < 2:
            return []

        x_l = ranges * np.cos(angles)
        y_l = ranges * np.sin(angles)
        z_l = np.zeros_like(x_l)
        xyz_l = np.vstack((x_l, y_l, z_l))  # (3, N)

        xyz_map = (R3 @ xyz_l) + trans.reshape(3, 1)

        points = list(zip(xyz_map[0, :], xyz_map[1, :]))

        # --- adaptive breakpoint segmentation ---
        clusters = [[list(points[0])]]
        for idx in range(1, len(points)):
            p_prev = points[idx - 1]
            p_curr = points[idx]
            dist_from_sensor = math.hypot(p_curr[0] - trans[0], p_curr[1] - trans[1])
            d_max = (dist_from_sensor * math.sin(d_phi) / math.sin(max(l_ang - d_phi, 1e-6)) + 3 * sigma) / 2
            if math.dist(p_prev, p_curr) > d_max:
                clusters.append([list(p_curr)])
            else:
                clusters[-1].append(list(p_curr))

        # --- 크기 / 차량 거리 필터링 ---
        filtered = []
        for cluster in clusters:
            if len(cluster) < self.min_obs_size:
                continue
            mid = cluster[len(cluster) // 2]
            if math.hypot(mid[0] - self.car_x, mid[1] - self.car_y) > self.max_viewing_distance:
                continue
            filtered.append(cluster)

        return filtered

    def _clusters_to_obstacles(self, clusters):
        obstacles = []
        min_dist = self.min_2_points_dist

        for cluster in clusters:
            pts = np.array(cluster)  # (N, 2)

            theta = np.linspace(0, np.pi / 2 - np.pi / 180, 90)
            cos_t, sin_t = np.cos(theta), np.sin(theta)

            proj1 = np.dot(pts, [cos_t, sin_t])
            proj2 = np.dot(pts, [-sin_t, cos_t])

            D10 = -proj1 + np.amax(proj1, axis=0)
            D11 = proj1 - np.amin(proj1, axis=0)
            D20 = -proj2 + np.amax(proj2, axis=0)
            D21 = proj2 - np.amin(proj2, axis=0)

            sel1 = np.argmin([np.linalg.norm(D10, axis=0), np.linalg.norm(D11, axis=0)], axis=0)
            D10, D11 = D10.T, D11.T
            D10[sel1 == 1] = D11[sel1 == 1]
            D10 = D10.T

            sel2 = np.argmin([np.linalg.norm(D20, axis=0), np.linalg.norm(D21, axis=0)], axis=0)
            D20, D21 = D20.T, D21.T
            D20[sel2 == 1] = D21[sel2 == 1]
            D20 = D20.T

            D = np.minimum(D10, D20)
            D[D < min_dist] = min_dist

            theta_opt = np.argmax(np.sum(np.reciprocal(D), axis=0)) * np.pi / 180
            d1 = np.dot(pts, [np.cos(theta_opt), np.sin(theta_opt)])
            d2 = np.dot(pts, [-np.sin(theta_opt), np.cos(theta_opt)])
            max_d1, min_d1 = np.max(d1), np.min(d1)
            max_d2, min_d2 = np.max(d2), np.min(d2)

            def corner(a, b):
                return np.array(
                    [np.cos(theta_opt) * a - np.sin(theta_opt) * b, np.sin(theta_opt) * a + np.cos(theta_opt) * b]
                )

            if np.var(d2) > np.var(d1):
                if np.linalg.norm(-d1 + max_d1) < np.linalg.norm(d1 - min_d1):
                    corner1, corner2 = corner(max_d1, min_d2), corner(max_d1, max_d2)
                else:
                    corner1, corner2 = corner(min_d1, max_d2), corner(min_d1, min_d2)
            else:
                if np.linalg.norm(-d2 + max_d2) < np.linalg.norm(d2 - min_d2):
                    corner1, corner2 = corner(max_d1, max_d2), corner(min_d1, max_d2)
                else:
                    corner1, corner2 = corner(min_d1, min_d2), corner(max_d1, min_d2)

            col_vec = corner2 - corner1
            orth_vec = np.array([-col_vec[1], col_vec[0]])
            center = corner1 + 0.5 * col_vec + 0.5 * orth_vec

            obstacles.append(Obstacle(center[0], center[1], np.linalg.norm(col_vec), theta_opt))

        return obstacles

    def _filter_by_size(self, obstacles):
        result = [o for o in obstacles if o.size <= self.max_obs_size]
        for idx, o in enumerate(result):
            o.id = idx
        self.tracked_obstacles = result

    # ------------------------------------------------------------------ #
    # Publishing
    # ------------------------------------------------------------------ #
    def _publish_poses_and_sizes(self):
        pose_array = PoseArray()
        pose_array.header.stamp = self.current_stamp
        pose_array.header.frame_id = self.map_frame

        sizes = Float32MultiArray()

        for obs in self.tracked_obstacles:
            pose = Pose()
            pose.position = Point(x=obs.center_x, y=obs.center_y, z=0.0)
            qx, qy, qz, qw = yaw_to_quaternion(obs.theta)
            pose.orientation.x, pose.orientation.y = qx, qy
            pose.orientation.z, pose.orientation.w = qz, qw
            pose_array.poses.append(pose)
            sizes.data.append(float(obs.size))

        self.poses_pub.publish(pose_array)
        self.sizes_pub.publish(sizes)

    def _publish_markers(self):
        markers = []
        for obs in self.tracked_obstacles:
            marker = Marker()
            marker.header.frame_id = self.map_frame
            marker.header.stamp = self.current_stamp
            marker.id = obs.id
            marker.type = Marker.CUBE
            marker.scale.x = marker.scale.y = marker.scale.z = float(obs.size)
            marker.color.a, marker.color.r, marker.color.g, marker.color.b = 0.5, 0.0, 1.0, 1.0
            marker.pose.position.x = obs.center_x
            marker.pose.position.y = obs.center_y
            qx, qy, qz, qw = yaw_to_quaternion(obs.theta)
            marker.pose.orientation.x, marker.pose.orientation.y = qx, qy
            marker.pose.orientation.z, marker.pose.orientation.w = qz, qw
            markers.append(marker)

        self.markers_pub.publish(MarkerArray(markers=self._clear_markers()))
        self.markers_pub.publish(MarkerArray(markers=markers))


def main(args=None):
    rclpy.init(args=args)
    node = ObstacleDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
