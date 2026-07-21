#!/usr/bin/env python3
"""
ObstacleArray(/perception/obstacles) -> MarkerArray(/perception/obstacle_markers)
RViz에서 장애물 인지 결과를 눈으로 확인하기 위한 임시 시각화 노드.

실행:
  source <워크스페이스>/install/setup.bash   # f1tenth_perception_msgs 필요
  python3 obstacle_viz.py

RViz에서 MarkerArray 디스플레이 추가 → Topic: /perception/obstacle_markers
Fixed Frame 은 obstacle 노드의 target_frame(임시로 base_link)과 동일하게.
"""
import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point
from f1tenth_perception_msgs.msg import ObstacleArray


class ObstacleViz(Node):
    def __init__(self):
        super().__init__('obstacle_viz')
        self.sub = self.create_subscription(
            ObstacleArray, '/perception/obstacles', self.on_obstacles, 10)
        self.pub = self.create_publisher(
            MarkerArray, '/perception/obstacle_markers', 10)
        self.prev_ids = set()
        self.get_logger().info(
            '/perception/obstacles -> /perception/obstacle_markers 시각화 시작')

    def on_obstacles(self, msg: ObstacleArray):
        markers = MarkerArray()
        cur_ids = set()

        for obs in msg.obstacles:
            cur_ids.add(obs.id)
            # 동적=빨강, 정적=초록
            r, g, b = (1.0, 0.1, 0.1) if obs.is_dynamic else (0.1, 0.9, 0.2)

            # 몸통(원기둥)
            m = Marker()
            m.header = msg.header
            m.ns = 'obstacle'
            m.id = obs.id
            m.type = Marker.CYLINDER
            m.action = Marker.ADD
            m.pose.position = obs.position
            m.pose.orientation.w = 1.0
            d = max(0.05, 2.0 * obs.radius)
            m.scale.x = d
            m.scale.y = d
            m.scale.z = 0.3
            m.color.r, m.color.g, m.color.b, m.color.a = r, g, b, 0.7
            markers.markers.append(m)

            # 속도 화살표
            a = Marker()
            a.header = msg.header
            a.ns = 'velocity'
            a.id = obs.id
            a.type = Marker.ARROW
            a.action = Marker.ADD
            a.scale.x = 0.05   # shaft 지름
            a.scale.y = 0.1    # head 지름
            a.scale.z = 0.0
            a.color.r, a.color.g, a.color.b, a.color.a = 1.0, 1.0, 0.0, 0.9
            start = Point(x=obs.position.x, y=obs.position.y, z=0.0)
            end = Point(x=obs.position.x + obs.velocity.x,
                        y=obs.position.y + obs.velocity.y, z=0.0)
            a.points = [start, end]
            markers.markers.append(a)

            # ID 텍스트
            t = Marker()
            t.header = msg.header
            t.ns = 'id'
            t.id = obs.id
            t.type = Marker.TEXT_VIEW_FACING
            t.action = Marker.ADD
            t.pose.position.x = obs.position.x
            t.pose.position.y = obs.position.y
            t.pose.position.z = 0.4
            t.pose.orientation.w = 1.0
            t.scale.z = 0.25
            t.color.r, t.color.g, t.color.b, t.color.a = 1.0, 1.0, 1.0, 1.0
            t.text = f'{obs.id}{"*" if obs.is_dynamic else ""}'
            markers.markers.append(t)

        # 사라진 트랙 마커 삭제
        for old_id in self.prev_ids - cur_ids:
            for ns in ('obstacle', 'velocity', 'id'):
                dm = Marker()
                dm.header = msg.header
                dm.ns = ns
                dm.id = old_id
                dm.action = Marker.DELETE
                markers.markers.append(dm)
        self.prev_ids = cur_ids

        self.pub.publish(markers)


def main():
    rclpy.init()
    node = ObstacleViz()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
