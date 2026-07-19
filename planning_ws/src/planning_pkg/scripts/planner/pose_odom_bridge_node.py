#!/usr/bin/env python3
"""TF(map->base_link) + /odom 속도 -> /pf/pose/odom (실차 전용).

control_node·frenet_frame_node(real)·recovery_supervisor는 map 프레임
nav_msgs/Odometry를 /pf/pose/odom에서 기대한다. 그러나 localization_ws는
slam_toolbox/amcl이라 map->odom TF만 낼 뿐 그 토픽을 발행하지 않는다
(/pf/pose/odom은 원래 ForzaETH particle_filter의 출력 이름).

이 노드가 그 간극을 메운다:
  pose  <- TF map->base_link (slam map->odom + vesc odom->base_link 합성)
  twist <- /odom (vesc_to_odom, child=base_link) 그대로 — Odometry 규약상
           twist는 child_frame 기준이므로 재변환 불필요

localization이 죽어 TF가 stale_sec보다 오래되면 발행을 멈춘다 —
control의 odom_timeout(0.5s)이 이를 감지해 정지하게 하는 안전 체인.
"""
import rclpy
from rclpy.node import Node
from rclpy.time import Time

import tf2_ros

from nav_msgs.msg import Odometry


class PoseOdomBridgeNode(Node):
    def __init__(self):
        super().__init__('pose_odom_bridge_node')

        self.declare_parameter('pose_bridge/rate_hz', 40.0)
        self.declare_parameter('pose_bridge/map_frame', 'map')
        self.declare_parameter('pose_bridge/base_frame', 'base_link')
        self.declare_parameter('pose_bridge/vel_odom_topic', '/odom')
        self.declare_parameter('pose_bridge/output_topic', '/pf/pose/odom')
        self.declare_parameter('pose_bridge/stale_sec', 0.5)

        p = self.get_parameter
        self.map_frame = p('pose_bridge/map_frame').value
        self.base_frame = p('pose_bridge/base_frame').value
        self.stale_sec = p('pose_bridge/stale_sec').value

        self.vel_odom = None

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.create_subscription(
            Odometry, p('pose_bridge/vel_odom_topic').value, self.cb_vel, 10)
        self.pub = self.create_publisher(
            Odometry, p('pose_bridge/output_topic').value, 10)
        self.create_timer(1.0 / p('pose_bridge/rate_hz').value, self.publish)

        self.get_logger().info(
            f"pose 브릿지 시작: TF {self.map_frame}->{self.base_frame} + "
            f"{p('pose_bridge/vel_odom_topic').value} -> "
            f"{p('pose_bridge/output_topic').value}")

    def cb_vel(self, msg: Odometry):
        self.vel_odom = msg

    def publish(self):
        try:
            tf = self.tf_buffer.lookup_transform(
                self.map_frame, self.base_frame, Time())
        except tf2_ros.TransformException as e:
            self.get_logger().warn(
                f'TF {self.map_frame}->{self.base_frame} 대기 중: {e}',
                throttle_duration_sec=2.0)
            return

        now = self.get_clock().now()
        age = (now - Time.from_msg(tf.header.stamp)).nanoseconds * 1e-9
        if age > self.stale_sec:
            # localization 두절 — 발행 중단으로 control odom_timeout 정지 유도
            self.get_logger().warn(
                f'TF stale ({age:.2f}s) — /pf/pose/odom 발행 중단',
                throttle_duration_sec=1.0)
            return

        msg = Odometry()
        msg.header.stamp = now.to_msg()
        msg.header.frame_id = self.map_frame
        msg.child_frame_id = self.base_frame
        msg.pose.pose.position.x = tf.transform.translation.x
        msg.pose.pose.position.y = tf.transform.translation.y
        msg.pose.pose.position.z = tf.transform.translation.z
        msg.pose.pose.orientation = tf.transform.rotation
        if self.vel_odom is not None:
            msg.twist = self.vel_odom.twist
        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = PoseOdomBridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
