#!/usr/bin/env python3
"""perception 팀 장애물 출력 -> 플래너 frenet 장애물 변환 브릿지 (실차 전용).

perception 팀(obstacle_detector_node)은 /perception/obstacles에
f1tenth_perception_msgs/ObstacleArray를 발행한다:
  - position: laser(스캔) 프레임 카르테시안 [m]
  - velocity: laser 프레임 [m/s] (ego 이동 미보상 — 참고용으로만 취급)
  - is_dynamic / confidence / radius

spliner·recovery는 f1tenth_planning_msgs/ObstacleArray(map 프레임 + frenet s/d)를
기대하므로 이 노드가 변환한다:
  laser -> map TF 변환 -> /global_waypoints 최근접 투영으로 (s, d) 계산
  -> 트랙 밖(벽) 클러스터 제거 -> id별 (s, d) 유한차분으로 vs/vd 추정
  -> /perception/obstacles_frenet 발행 (real_car.yaml이 spliner/recovery의
     구독 토픽을 이 토픽으로 override)

is_static은 perception의 is_dynamic을 초기값으로 쓰되, 최종 판정은
spliner의 재분류(cls_* 파라미터, std 투표)가 담당한다.
velocity는 ego 이동이 섞인 laser 프레임 값이라 그대로 쓰지 않고
map 프레임 위치의 시간차분으로 다시 추정한다.

sim에서는 virtual_obstacle_node가 f1tenth_planning_msgs 타입을 직접 발행하므로
이 노드는 실차 launch(planner_stack env:=real_car)에서만 뜬다.
"""
import math

import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.time import Time
from rclpy.duration import Duration

import tf2_ros

from f1tenth_planning_msgs.msg import Obstacle, ObstacleArray, WpntArray
from f1tenth_perception_msgs.msg import ObstacleArray as PerceptionObstacleArray


class PerceptionObstacleBridgeNode(Node):
    def __init__(self):
        super().__init__('perception_obstacle_bridge_node')

        self.declare_parameter('bridge/input_topic', '/perception/obstacles')
        self.declare_parameter('bridge/output_topic', '/perception/obstacles_frenet')
        self.declare_parameter('bridge/target_frame', 'map')
        self.declare_parameter('bridge/min_confidence', 0.0)
        self.declare_parameter('bridge/wall_margin_m', 0.10)
        # 벽/펜스 아크 클러스터 제거: 오목한 벽 아크는 반경이 수 m로 커지고
        # centroid가 트랙 안쪽에 떨어져 d-경계 필터를 통과한다 (gym 실측 5.2m).
        # 실제 장애물(박스/상대차)은 반경 0.5m 미만.
        self.declare_parameter('bridge/max_radius_m', 0.60)
        self.declare_parameter('bridge/vel_lpf_alpha', 0.3)
        self.declare_parameter('bridge/track_forget_sec', 2.0)
        self.declare_parameter('bridge/tf_timeout_sec', 0.1)

        p = self.get_parameter
        self.target_frame = p('bridge/target_frame').value
        self.min_confidence = p('bridge/min_confidence').value
        self.wall_margin = p('bridge/wall_margin_m').value
        self.max_radius = p('bridge/max_radius_m').value
        self.lpf_alpha = p('bridge/vel_lpf_alpha').value
        self.forget_sec = p('bridge/track_forget_sec').value
        self.tf_timeout = p('bridge/tf_timeout_sec').value

        self.wpnts = None        # [N, 6]: s, x, y, psi, d_right, d_left
        self.track_length = 0.0
        self.tracks = {}         # id -> {'t', 's', 'd', 'vs', 'vd'}

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.create_subscription(WpntArray, '/global_waypoints', self.cb_gb, 10)
        self.create_subscription(
            PerceptionObstacleArray, p('bridge/input_topic').value, self.cb_obs, 10)
        self.pub = self.create_publisher(
            ObstacleArray, p('bridge/output_topic').value, 10)

        self.get_logger().info(
            f"perception 브릿지 시작: {p('bridge/input_topic').value} "
            f"(f1tenth_perception_msgs) -> {p('bridge/output_topic').value} (f1tenth_planning_msgs)")

    def cb_gb(self, msg: WpntArray):
        self.wpnts = np.array([
            [w.s_m, w.x_m, w.y_m, w.psi_rad, w.d_right, w.d_left]
            for w in msg.wpnts])
        if len(msg.wpnts) >= 2:
            spacing = msg.wpnts[1].s_m - msg.wpnts[0].s_m
            self.track_length = msg.wpnts[-1].s_m + spacing

    def cartesian_to_frenet(self, x, y):
        """virtual_obstacle_node/frenet_frame_node와 동일한 최근접 투영 (d>0 = 왼쪽)."""
        d2 = (self.wpnts[:, 1] - x) ** 2 + (self.wpnts[:, 2] - y) ** 2
        i = int(np.argmin(d2))
        s0, wx, wy, psi = self.wpnts[i, 0], self.wpnts[i, 1], self.wpnts[i, 2], self.wpnts[i, 3]
        dx, dy = x - wx, y - wy
        s = (s0 + dx * math.cos(psi) + dy * math.sin(psi)) % self.track_length
        d = -dx * math.sin(psi) + dy * math.cos(psi)
        return s, d, i

    def cb_obs(self, msg: PerceptionObstacleArray):
        if self.wpnts is None or self.track_length <= 0.0:
            return

        out = ObstacleArray()
        out.header.stamp = msg.header.stamp
        out.header.frame_id = self.target_frame

        # laser -> map 변환. 정확한 스탬프가 없으면 최신 TF로 폴백.
        tf = None
        if msg.header.frame_id and msg.header.frame_id != self.target_frame:
            try:
                tf = self.tf_buffer.lookup_transform(
                    self.target_frame, msg.header.frame_id,
                    Time.from_msg(msg.header.stamp),
                    timeout=Duration(seconds=self.tf_timeout))
            except tf2_ros.TransformException:
                try:
                    tf = self.tf_buffer.lookup_transform(
                        self.target_frame, msg.header.frame_id, Time())
                except tf2_ros.TransformException as e:
                    self.get_logger().warn(
                        f'TF {msg.header.frame_id}->{self.target_frame} 없음: {e}',
                        throttle_duration_sec=2.0)
                    return
        if tf is not None:
            q = tf.transform.rotation
            yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                             1.0 - 2.0 * (q.y * q.y + q.z * q.z))
            cos_y, sin_y = math.cos(yaw), math.sin(yaw)
            tx, ty = tf.transform.translation.x, tf.transform.translation.y

        now_sec = Time.from_msg(msg.header.stamp).nanoseconds * 1e-9
        seen_ids = set()

        for src in msg.obstacles:
            if src.confidence < self.min_confidence:
                continue
            if float(src.radius) > self.max_radius:  # 벽 아크 클러스터
                continue
            x, y = src.position.x, src.position.y
            if tf is not None:
                x, y = cos_y * x - sin_y * y + tx, sin_y * x + cos_y * y + ty

            s, d, wi = self.cartesian_to_frenet(x, y)

            # 트랙 경계 밖(벽/펜스 반사) 클러스터 제거 — d>0 왼쪽, 왼벽 d_left
            d_right_bound, d_left_bound = self.wpnts[wi, 4], self.wpnts[wi, 5]
            if d > d_left_bound - self.wall_margin or d < -d_right_bound + self.wall_margin:
                continue

            # id별 시간차분으로 map 기준 vs/vd 추정 (laser 프레임 속도는
            # ego 이동이 섞여 있어 신뢰 불가)
            vs, vd = 0.0, 0.0
            prev = self.tracks.get(src.id)
            if prev is not None:
                dt = now_sec - prev['t']
                if 1e-3 < dt < 1.0:
                    ds = (s - prev['s'] + self.track_length / 2.0) % self.track_length \
                        - self.track_length / 2.0
                    a = self.lpf_alpha
                    vs = a * (ds / dt) + (1.0 - a) * prev['vs']
                    vd = a * ((d - prev['d']) / dt) + (1.0 - a) * prev['vd']
            self.tracks[src.id] = {'t': now_sec, 's': s, 'd': d, 'vs': vs, 'vd': vd}
            seen_ids.add(src.id)

            half = max(float(src.radius), 0.05)
            obs = Obstacle()
            obs.id = int(src.id)
            obs.position.x, obs.position.y = float(x), float(y)
            obs.s_center = s
            obs.d_center = d
            obs.s_start = (s - half) % self.track_length
            obs.s_end = (s + half) % self.track_length
            obs.d_left = d + half
            obs.d_right = d - half
            obs.vs = vs
            obs.vd = vd
            obs.size = 2.0 * half
            obs.is_static = not src.is_dynamic  # 초기값 — spliner cls_*가 재분류
            obs.is_visible = True
            out.obstacles.append(obs)

        # 오래 안 보인 트랙 정리
        stale = [i for i, t in self.tracks.items()
                 if i not in seen_ids and now_sec - t['t'] > self.forget_sec]
        for i in stale:
            del self.tracks[i]

        # 빈 배열도 그대로 전달 (spliner obs_timeout과 연동)
        self.pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = PerceptionObstacleBridgeNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        # launch가 SIGINT를 보내면 컨텍스트가 이미 shutdown된 상태로 spin이 깨진다.
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
