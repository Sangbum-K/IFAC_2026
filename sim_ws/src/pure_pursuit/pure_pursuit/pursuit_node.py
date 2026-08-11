import rclpy
from rclpy.node import Node
from ackermann_msgs.msg import AckermannDriveStamped
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped
import numpy as np
import math
import csv

class PurePursuitNode(Node):
    def __init__(self):
        super().__init__('pure_pursuit')
        self.waypoints = self.load_waypoints('/home/sangbum/sim_ws/src/waypoints.csv')
        self.L = 0.8  # lookahead 거리

        self.drive_pub = self.create_publisher(AckermannDriveStamped, '/drive', 10)
        self.path_pub  = self.create_publisher(Path, '/raceline', 10)
        self.create_subscription(Odometry, '/ego_racecar/odom', self.odom_cb, 10)

        # 1초 후 경로 퍼블리시
        self.create_timer(1.0, self.publish_path)
        self.get_logger().info(f'waypoints 로드 완료: {len(self.waypoints)}개')

    def load_waypoints(self, path):
        pts = []
        with open(path) as f:
            for line in f:
                line = line.strip()
                if line.startswith('#') or not line:
                    continue
                cols = [c.strip() for c in line.split(';')]
                # s_m; x_m; y_m; psi_rad; kappa_radpm; vx_mps; ax_mps2
                x   = float(cols[1])
                y   = float(cols[2])
                vx  = float(cols[5])
                pts.append([x, y, vx])
        return np.array(pts)

    def publish_path(self):
        path = Path()
        path.header.frame_id = 'map'
        path.header.stamp = self.get_clock().now().to_msg()
        for pt in self.waypoints:
            ps = PoseStamped()
            ps.header.frame_id = 'map'
            ps.pose.position.x = pt[0]
            ps.pose.position.y = pt[1]
            path.poses.append(ps)
        self.path_pub.publish(path)

    def odom_cb(self, msg):
        cx = msg.pose.pose.position.x
        cy = msg.pose.pose.position.y

        # 가장 가까운 waypoint
        dists = np.hypot(self.waypoints[:,0] - cx,
                          self.waypoints[:,1] - cy)
        idx = int(np.argmin(dists))

        # lookahead 거리 안의 목표점
        target = None
        n = len(self.waypoints)
        for i in range(n):
            wp = self.waypoints[(idx + i) % n]
            if math.hypot(wp[0]-cx, wp[1]-cy) >= self.L:
                target = wp
                break
        if target is None:
            return

        # yaw 추출
        q = msg.pose.pose.orientation
        yaw = math.atan2(2*(q.w*q.z + q.x*q.y),
                          1 - 2*(q.y**2 + q.z**2))

        # 조향각 계산
        alpha  = math.atan2(target[1]-cy, target[0]-cx) - yaw
        steer  = math.atan2(2 * 0.33 * math.sin(alpha), self.L)
        steer  = max(-0.4, min(0.4, steer))

        # 속도 (최대 2m/s로 제한)
        speed = min(float(target[2]), 2.0)

        drive = AckermannDriveStamped()
        drive.drive.steering_angle = steer
        drive.drive.speed = speed
        self.drive_pub.publish(drive)

def main():
    rclpy.init()
    node = PurePursuitNode()
    rclpy.spin(node)

if __name__ == '__main__':
    main()
