#!/usr/bin/env python3
"""장애물 인지 결과 시각화 — ObstacleArray -> MarkerArray.

  /perception/obstacles (f1tenth_perception_msgs/ObstacleArray)
    -> /perception/obstacle_markers (visualization_msgs/MarkerArray)

monitor_pkg의 monitor.rviz "Obstacles" 디스플레이가 이 토픽을 본다. 실차에서
obstacle_detector_node는 잘 돌면서 마커 발행자가 없어 "회피 라인은 휘는데
무엇을 피하는지 안 보이는" 상태였는데, 그걸 메꾸는 노드다.
(perception_ws/obstacle_viz.py 임시 스크립트를 패키지 노드로 승격한 것)

표시:
  - 정적(is_dynamic=false): 초록 원기둥. 트랙 ID 텍스트 `#3`.
  - 동적(is_dynamic=true):  빨강 원기둥 + 노랑 속도 화살표. 텍스트 `#3 D 1.2m/s`.
  - confidence가 낮으면(코스팅 중) 반투명해진다.
  - 사라진 트랙은 DELETE 마커로 지운다 (안 지우면 RViz에 유령이 남는다).

원기둥 지름은 Obstacle.radius(클러스터 반경 추정치)의 2배라 화면상 크기가
실제 인지 크기다 — 회피 라인이 왜 그만큼 벌어졌는지 눈으로 대조할 수 있다.

프레임은 입력 메시지의 header를 그대로 쓴다. detector의 target_frame이 map이면
마커도 map에 고정되므로, 정적 장애물은 차가 지나가도 제자리에 남는다.
"""
import math

import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point
from f1tenth_perception_msgs.msg import ObstacleArray

# 마커 네임스페이스. DELETE도 같은 (ns, id)로 보내야 지워진다.
NS_BODY = 'obstacle'
NS_VEL = 'velocity'
NS_TEXT = 'id'
ALL_NS = (NS_BODY, NS_VEL, NS_TEXT)


class ObstacleViz(Node):
    def __init__(self):
        super().__init__('obstacle_viz_node')

        self.declare_parameter('viz/input_topic', '/perception/obstacles')
        self.declare_parameter('viz/output_topic', '/perception/obstacle_markers')
        # 마커 수명. 0이면 무한 — 발행이 끊겨도 화면에 남는다. 인지가 죽은 걸
        # 알아채야 하므로 기본은 유한값으로 두고 매 프레임 갱신한다.
        self.declare_parameter('viz/marker_lifetime_sec', 0.5)
        self.declare_parameter('viz/body_height_m', 0.3)
        self.declare_parameter('viz/text_size_m', 0.25)
        self.declare_parameter('viz/min_diameter_m', 0.05)

        def p(name):
            return self.get_parameter(name).value

        input_topic = p('viz/input_topic')
        output_topic = p('viz/output_topic')
        self.lifetime_sec = float(p('viz/marker_lifetime_sec'))
        self.body_height = float(p('viz/body_height_m'))
        self.text_size = float(p('viz/text_size_m'))
        self.min_diameter = float(p('viz/min_diameter_m'))

        self.sub = self.create_subscription(
            ObstacleArray, input_topic, self.on_obstacles, 10)
        self.pub = self.create_publisher(MarkerArray, output_topic, 10)
        self.prev_ids = set()

        self.get_logger().info(
            f'장애물 시각화 시작: {input_topic} -> {output_topic}')

    def _lifetime(self, marker):
        sec = int(self.lifetime_sec)
        marker.lifetime.sec = sec
        marker.lifetime.nanosec = int((self.lifetime_sec - sec) * 1e9)

    def on_obstacles(self, msg: ObstacleArray):
        markers = MarkerArray()
        cur_ids = set()

        for obs in msg.obstacles:
            cur_ids.add(obs.id)
            # 동적=빨강, 정적=초록. 코스팅(외삽) 중이면 confidence가 떨어지므로
            # 알파에 그대로 반영해 "지금 보고 있는 것"과 구분한다.
            r, g, b = (1.0, 0.1, 0.1) if obs.is_dynamic else (0.1, 0.9, 0.2)
            alpha = 0.25 + 0.55 * max(0.0, min(1.0, float(obs.confidence)))

            body = Marker()
            body.header = msg.header
            body.ns = NS_BODY
            body.id = obs.id
            body.type = Marker.CYLINDER
            body.action = Marker.ADD
            body.pose.position = obs.position
            body.pose.position.z = self.body_height / 2.0
            body.pose.orientation.w = 1.0
            diameter = max(self.min_diameter, 2.0 * obs.radius)
            body.scale.x = diameter
            body.scale.y = diameter
            body.scale.z = self.body_height
            body.color.r, body.color.g, body.color.b = r, g, b
            body.color.a = alpha
            self._lifetime(body)
            markers.markers.append(body)

            speed = math.hypot(obs.velocity.x, obs.velocity.y)

            # 속도 화살표는 동적일 때만. 정적 장애물의 잔여 속도(수 cm/s)까지
            # 그리면 화면이 지저분하고 "정적인데 왜 화살표?"로 오해를 부른다.
            if obs.is_dynamic:
                arrow = Marker()
                arrow.header = msg.header
                arrow.ns = NS_VEL
                arrow.id = obs.id
                arrow.type = Marker.ARROW
                arrow.action = Marker.ADD
                arrow.pose.orientation.w = 1.0
                arrow.scale.x = 0.05   # shaft 지름
                arrow.scale.y = 0.1    # head 지름
                arrow.scale.z = 0.0
                arrow.color.r, arrow.color.g, arrow.color.b = 1.0, 1.0, 0.0
                arrow.color.a = 0.9
                arrow.points = [
                    Point(x=obs.position.x, y=obs.position.y, z=0.05),
                    Point(x=obs.position.x + obs.velocity.x,
                          y=obs.position.y + obs.velocity.y, z=0.05),
                ]
                self._lifetime(arrow)
                markers.markers.append(arrow)

            text = Marker()
            text.header = msg.header
            text.ns = NS_TEXT
            text.id = obs.id
            text.type = Marker.TEXT_VIEW_FACING
            text.action = Marker.ADD
            text.pose.position.x = obs.position.x
            text.pose.position.y = obs.position.y
            text.pose.position.z = self.body_height + 0.15
            text.pose.orientation.w = 1.0
            text.scale.z = self.text_size
            text.color.r, text.color.g, text.color.b, text.color.a = 1.0, 1.0, 1.0, 1.0
            if obs.is_dynamic:
                text.text = f'#{obs.id} D {speed:.1f}m/s'
            else:
                text.text = f'#{obs.id}'
            self._lifetime(text)
            markers.markers.append(text)

        # 사라진 트랙 마커 삭제 (lifetime만 믿으면 ID 재사용 시 유령이 남는다)
        for old_id in self.prev_ids - cur_ids:
            for ns in ALL_NS:
                gone = Marker()
                gone.header = msg.header
                gone.ns = ns
                gone.id = old_id
                gone.action = Marker.DELETE
                markers.markers.append(gone)
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
