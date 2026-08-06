#!/usr/bin/env python3
"""perception 팀 장애물 출력 -> 플래너 frenet 장애물 변환 브릿지 (실차 전용).

perception 팀(obstacle_detector_node)은 /perception/obstacles에
f1tenth_perception_msgs/ObstacleArray를 발행한다:
  - position: msg.header.frame_id의 카르테시안 좌표 [m] (현재 detector는 map)
  - velocity: 같은 프레임 [m/s] (브릿지는 id별 위치 차분으로 재검증)
  - is_dynamic / confidence / radius

spliner·recovery는 f1tenth_planning_msgs/ObstacleArray(map 프레임 + frenet s/d)를
기대하므로 이 노드가 변환한다:
  source frame -> map timestamp TF -> /global_waypoints segment 투영으로 (s, d) 계산
  -> 트랙 밖(벽) 클러스터 제거 -> id별 (s, d) 유한차분으로 vs/vd 추정
  -> /perception/obstacles_frenet 발행 (real_car.yaml이 spliner/recovery의
     구독 토픽을 이 토픽으로 override)

is_static은 perception의 is_dynamic을 초기값으로 쓰되, 최종 판정은
spliner의 재분류(cls_* 파라미터, std 투표)가 담당한다.
velocity는 map 프레임 위치의 시간차분으로 다시 추정한다. 첫 관측이라
차분이 아직 없는 프레임에서는 detector의 KF 속도(현재 target_frame=map이라
ego 이동이 섞여 있지 않다)를 Frenet으로 투영해 시드로 쓴다.

sim에서는 virtual_obstacle_node가 f1tenth_planning_msgs 타입을 직접 발행하므로
이 노드는 실차 launch(planner_stack env:=real_car)에서만 뜬다.
"""
import math

import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from rclpy.time import Time
from rclpy.duration import Duration

import tf2_ros

from nav_msgs.msg import OccupancyGrid

from f1tenth_planning_msgs.msg import Obstacle, ObstacleArray, WpntArray
from f1tenth_perception_msgs.msg import ObstacleArray as PerceptionObstacleArray

try:
    from scipy.ndimage import distance_transform_edt
except ImportError:      # scipy 없는 환경(다른 개발 PC 등)은 맵 필터 없이 폴백
    distance_transform_edt = None


class PerceptionObstacleBridgeNode(Node):
    def __init__(self):
        super().__init__('perception_obstacle_bridge_node')

        self.declare_parameter('bridge/input_topic', '/perception/obstacles')
        self.declare_parameter('bridge/output_topic', '/perception/obstacles_frenet')
        self.declare_parameter('bridge/target_frame', 'map')
        # detector가 이미 min_hits로 트랙을 확정한다. 브릿지에서 저신뢰 트랙을
        # 삭제하면 가려지는 순간 장애물이 사라지므로 안전 기본값은 0이다.
        self.declare_parameter('bridge/min_confidence', 0.0)
        self.declare_parameter('bridge/wall_margin_m', 0.10)
        # 큰 extent를 축소/삭제하지 않고 진단하는 임계값. 유령 제거는
        # footprint-corridor overlap과 opt-in map filter가 담당한다.
        self.declare_parameter('bridge/max_extent_m', 1.50)
        self.declare_parameter('bridge/vel_lpf_alpha', 0.3)
        self.declare_parameter('bridge/max_abs_vs_mps', 10.0)
        self.declare_parameter('bridge/max_abs_vd_mps', 4.0)
        self.declare_parameter('bridge/position_var_floor', 0.01)
        self.declare_parameter('bridge/velocity_var_floor', 0.25)
        # 속도를 아직 유한차분으로 못 잰 첫 프레임의 속도 표준편차 prior [m/s].
        # max_abs_v(클립 한계)를 분산으로 쓰던 것을 대체한다 — 근거는 아래
        # obs.vs_var 계산부 주석 참조.
        self.declare_parameter('bridge/init_vel_std_mps', 0.5)
        self.declare_parameter('bridge/track_forget_sec', 2.0)
        self.declare_parameter('bridge/tf_timeout_sec', 0.1)
        self.declare_parameter('bridge/tf_allow_latest_fallback', False)
        self.declare_parameter('bridge/tf_max_age_sec', 0.05)
        self.declare_parameter('bridge/max_source_age_sec', 0.20)
        self.declare_parameter('bridge/source_future_tolerance_sec', 0.05)
        self.declare_parameter('bridge/require_source_stamp', True)
        self.declare_parameter('bridge/projection_window_wpnts', 80)
        self.declare_parameter('bridge/projection_reacquire_distance_m', 1.5)
        # 맵 벽거리 필터: frenet d는 최근접 raceline 점의 법선 거리라 코너에서
        # 실제 벽까지 거리와 어긋난다 — 벽 위 유령(실측 d=-0.85, 경계 1.32)이
        # d-경계 필터를 통과했다. /map(OccupancyGrid) EDT로 "벽까지 실거리"를
        # 직접 판정한다. 맵 미수신/scipy 부재 시 기존 frenet 필터만으로 폴백.
        # ⚠️ 실차 적용 전 PF 수렴 상태에서 정지 장애물 좌표 산포를 먼저 잴 것 —
        #    wall_dist_min이 PF 오차보다 작으면 유령이 새고, 크면 진짜를 버린다.
        self.declare_parameter('bridge/use_map_filter', False)
        self.declare_parameter('bridge/map_topic', '/map')
        self.declare_parameter('bridge/wall_dist_min_m', 0.20)
        self.declare_parameter('bridge/map_timeout_sec', 5.0)
        self.declare_parameter('bridge/map_filter_required', True)

        p = self.get_parameter
        self.target_frame = str(p('bridge/target_frame').value).strip()
        self.min_confidence = float(p('bridge/min_confidence').value)
        self.wall_margin = float(p('bridge/wall_margin_m').value)
        self.max_extent = float(p('bridge/max_extent_m').value)
        self.lpf_alpha = float(p('bridge/vel_lpf_alpha').value)
        self.max_abs_vs = float(p('bridge/max_abs_vs_mps').value)
        self.max_abs_vd = float(p('bridge/max_abs_vd_mps').value)
        self.position_var_floor = float(
            p('bridge/position_var_floor').value)
        self.velocity_var_floor = float(
            p('bridge/velocity_var_floor').value)
        self.init_vel_std = float(p('bridge/init_vel_std_mps').value)
        self.forget_sec = float(p('bridge/track_forget_sec').value)
        self.tf_timeout = float(p('bridge/tf_timeout_sec').value)
        self.tf_allow_latest = bool(
            p('bridge/tf_allow_latest_fallback').value)
        self.tf_max_age = float(p('bridge/tf_max_age_sec').value)
        self.max_source_age = float(p('bridge/max_source_age_sec').value)
        self.source_future_tolerance = float(
            p('bridge/source_future_tolerance_sec').value)
        self.require_source_stamp = bool(
            p('bridge/require_source_stamp').value)
        self.projection_window = int(p('bridge/projection_window_wpnts').value)
        self.projection_reacquire_distance = float(
            p('bridge/projection_reacquire_distance_m').value)
        self.use_map_filter = bool(p('bridge/use_map_filter').value)
        self.wall_dist_min = float(p('bridge/wall_dist_min_m').value)
        self.map_timeout = float(p('bridge/map_timeout_sec').value)
        self.map_filter_required = bool(
            p('bridge/map_filter_required').value)

        finite_values = (
            self.min_confidence, self.wall_margin, self.max_extent,
            self.lpf_alpha, self.max_abs_vs, self.max_abs_vd,
            self.position_var_floor, self.velocity_var_floor, self.init_vel_std,
            self.forget_sec, self.tf_timeout, self.tf_max_age,
            self.max_source_age, self.source_future_tolerance,
            self.projection_reacquire_distance, self.wall_dist_min,
            self.map_timeout,
        )
        if (not all(math.isfinite(value) for value in finite_values)
                or not self.target_frame
                or not 0.0 <= self.min_confidence <= 1.0
                or not 0.0 <= self.wall_margin <= 2.0
                or not 0.05 <= self.max_extent <= 20.0
                or not 0.0 <= self.lpf_alpha <= 1.0
                or not 0.1 <= self.max_abs_vs <= 50.0
                or not 0.1 <= self.max_abs_vd <= 20.0
                or not 1e-6 <= self.position_var_floor <= 100.0
                or not 1e-6 <= self.velocity_var_floor <= 2500.0
                or not 0.0 <= self.init_vel_std <= 50.0
                or not 0.1 <= self.forget_sec <= 10.0
                or not 0.001 <= self.tf_timeout <= 1.0
                or not 0.0 <= self.tf_max_age <= self.max_source_age
                or not 0.01 <= self.max_source_age <= 2.0
                or not 0.0 <= self.source_future_tolerance <= 0.5
                or not 4 <= self.projection_window <= 10000
                or not 0.1 <= self.projection_reacquire_distance <= 20.0
                or not 0.0 <= self.wall_dist_min <= 2.0
                or not 0.1 <= self.map_timeout <= 60.0):
            raise RuntimeError('invalid bridge safety parameters')

        self.wpnts = None        # [N, 6]: s, x, y, psi, d_right, d_left
        self.track_length = 0.0
        self.tracks = {}         # id -> {'t', 'arrival_t', 's', 'd', 'vs', 'vd', 'wi'}
        self.wall_dist = None    # EDT 거리 테이블 [m] (맵 수신 시 1회 계산)
        self.map_origin = (0.0, 0.0)
        self.map_origin_yaw = 0.0
        self.map_res = 0.0
        self.start_time = self.get_clock().now()
        self.last_source_ns = None

        self.tf_buffer = tf2_ros.Buffer()
        # cb_obs 안의 exact-time lookup이 executor를 기다리는 동안 /tf callback이
        # 굶지 않아야 한다. spin_thread=True는 이걸 못 한다 — 오히려 무력화된다:
        # TransformListener가 전용 executor에 node를 add_node한 뒤 main()의
        # rclpy.spin(node)이 글로벌 executor에 같은 node를 넣으면 rclpy의
        # Node.executor setter가 기존 executor에서 노드를 빼버린다
        # (rclpy/node.py: `current_executor.remove_node(self)`).
        # 그러면 /tf 콜백이 결국 메인 executor에 직렬화되고, depth 100인 /tf
        # 큐가 포화되며 버퍼가 ~1.6초 늙는다 (obstacle_detector에서 실측:
        # run_0803_204743, |tf-scan| 1.3~2.1s로 스캔 전멸).
        # -> spin_thread=False + main()의 MultiThreadedExecutor 조합을 쓴다.
        self.tf_listener = tf2_ros.TransformListener(
            self.tf_buffer, self, spin_thread=False)

        waypoint_qos = QoSProfile(
            depth=1, reliability=ReliabilityPolicy.RELIABLE)
        sensor_qos = QoSProfile(
            depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        output_qos = QoSProfile(
            depth=1, reliability=ReliabilityPolicy.RELIABLE)
        self.create_subscription(WpntArray, '/global_waypoints', self.cb_gb, waypoint_qos)
        input_topic = str(p('bridge/input_topic').value).strip()
        output_topic = str(p('bridge/output_topic').value).strip()
        map_topic = str(p('bridge/map_topic').value).strip()
        if not input_topic or not output_topic or \
                (self.use_map_filter and not map_topic):
            raise RuntimeError('bridge input/output/map topics must not be empty')
        self.create_subscription(
            PerceptionObstacleArray, input_topic, self.cb_obs, sensor_qos)
        if self.use_map_filter:
            if distance_transform_edt is None:
                self.get_logger().error(
                    'bridge/use_map_filter=true지만 scipy가 없음 — '
                    'map_filter_required이면 obstacle lease를 발행하지 않음')
            else:
                # map_server는 latched(TRANSIENT_LOCAL) 발행 — 기본 QoS로는 못 받는다
                map_qos = QoSProfile(
                    depth=1,
                    reliability=ReliabilityPolicy.RELIABLE,
                    durability=DurabilityPolicy.TRANSIENT_LOCAL)
                self.create_subscription(
                    OccupancyGrid, map_topic, self.cb_map, map_qos)
        self.pub = self.create_publisher(
            ObstacleArray, output_topic, output_qos)

        self.get_logger().info(
            f"perception 브릿지 시작: {input_topic} "
            f"(f1tenth_perception_msgs) -> {output_topic} (f1tenth_planning_msgs)")

    def cb_map(self, msg: OccupancyGrid):
        """노드 수명 중 1회: 셀별 최근접 벽거리 테이블(EDT) 계산."""
        if self.wall_dist is not None:
            return
        if msg.info.width == 0 or msg.info.height == 0 or \
                not math.isfinite(msg.info.resolution) or \
                msg.info.resolution <= 0.0:
            self.get_logger().error('invalid occupancy grid; map filter unavailable')
            return
        expected_cells = int(msg.info.width) * int(msg.info.height)
        q = msg.info.origin.orientation
        map_values = (
            msg.info.origin.position.x, msg.info.origin.position.y,
            q.x, q.y, q.z, q.w,
        )
        q_norm = math.sqrt(q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w)
        if (len(msg.data) != expected_cells
                or not all(math.isfinite(v) for v in map_values)
                or q_norm < 1e-9):
            self.get_logger().error(
                'occupancy grid data/origin is malformed; map filter unavailable')
            return
        grid = np.array(msg.data, dtype=np.int8).reshape(
            msg.info.height, msg.info.width)
        # unknown도 free로 증명되지 않았으므로 벽으로 취급한다.
        occupied = (grid < 0) | (grid >= 65)
        self.wall_dist = distance_transform_edt(~occupied) * msg.info.resolution
        self.map_origin = (msg.info.origin.position.x, msg.info.origin.position.y)
        qx, qy, qz, qw = (
            q.x / q_norm, q.y / q_norm, q.z / q_norm, q.w / q_norm)
        self.map_origin_yaw = math.atan2(
            2.0 * (qw * qz + qx * qy),
            1.0 - 2.0 * (qy * qy + qz * qz))
        self.map_res = msg.info.resolution
        self.get_logger().info(
            f'맵 벽거리 필터 활성: {msg.info.width}x{msg.info.height} '
            f'res={msg.info.resolution:.3f} wall_dist_min={self.wall_dist_min:.2f}m')

    def wall_distance(self, x, y):
        """map 좌표의 최근접 벽거리. 맵 밖/미수신은 None."""
        if self.wall_dist is None:
            return None
        dx, dy = x - self.map_origin[0], y - self.map_origin[1]
        c_yaw, s_yaw = math.cos(self.map_origin_yaw), math.sin(self.map_origin_yaw)
        grid_x = c_yaw * dx + s_yaw * dy
        grid_y = -s_yaw * dx + c_yaw * dy
        c = int(math.floor(grid_x / self.map_res))
        r = int(math.floor(grid_y / self.map_res))
        h, w = self.wall_dist.shape
        if 0 <= r < h and 0 <= c < w:
            return float(self.wall_dist[r, c])
        return None

    def cb_gb(self, msg: WpntArray):
        candidate = np.array([
            [w.s_m, w.x_m, w.y_m, w.psi_rad, w.d_right, w.d_left]
            for w in msg.wpnts], dtype=float)
        if len(candidate) < 2 or not np.isfinite(candidate).all() or \
                np.any(np.diff(candidate[:, 0]) <= 1e-5) or \
                np.any(candidate[:, 4:6] < 0.0):
            self.get_logger().error(
                'invalid global waypoints; bridge projection disabled',
                throttle_duration_sec=2.0)
            self.wpnts = None
            self.track_length = 0.0
            return
        segment_lengths = np.linalg.norm(np.diff(candidate[:, 1:3], axis=0), axis=1)
        ds = np.diff(candidate[:, 0])
        if (np.any(segment_lengths <= 1e-5)
                or np.any(np.abs(segment_lengths - ds)
                          > np.maximum(0.05, 0.25 * ds))):
            self.get_logger().error(
                'global waypoint geometry/s spacing mismatch; '
                'bridge projection disabled',
                throttle_duration_sec=2.0)
            self.wpnts = None
            self.track_length = 0.0
            return
        closure = float(np.linalg.norm(candidate[0, 1:3] - candidate[-1, 1:3]))
        track_length = candidate[-1, 0] - candidate[0, 0] + closure
        if not math.isfinite(track_length) or track_length <= 0.0:
            self.wpnts = None
            self.track_length = 0.0
            return
        self.wpnts = candidate
        self.track_length = track_length

    def cartesian_to_frenet(self, x, y, hint=None):
        """연속 segment 투영. 기존 track id가 있으면 인접 branch만 탐색한다."""
        n = len(self.wpnts)
        if hint is None:
            candidates = np.arange(n, dtype=int)
        else:
            w = max(self.projection_window, 4)
            candidates = np.arange(hint - w, hint + w + 1, dtype=int) % n

        best = None
        for i_raw in candidates:
            i = int(i_raw)
            j = (i + 1) % n
            p0 = self.wpnts[i, 1:3]
            p1 = self.wpnts[j, 1:3]
            seg = p1 - p0
            seg2 = float(np.dot(seg, seg))
            if seg2 < 1e-10:
                continue
            rel = np.array([x, y]) - p0
            u = float(np.clip(np.dot(rel, seg) / seg2, 0.0, 1.0))
            proj = p0 + u * seg
            dist2 = float(np.dot(np.array([x, y]) - proj,
                                 np.array([x, y]) - proj))
            if best is None or dist2 < best[0]:
                ds = ((self.wpnts[j, 0] - self.wpnts[i, 0])
                      % self.track_length)
                psi = math.atan2(seg[1], seg[0])
                dx, dy = x - proj[0], y - proj[1]
                d = -dx * math.sin(psi) + dy * math.cos(psi)
                right = ((1.0 - u) * self.wpnts[i, 4]
                         + u * self.wpnts[j, 4])
                left = ((1.0 - u) * self.wpnts[i, 5]
                        + u * self.wpnts[j, 5])
                best = (dist2, (self.wpnts[i, 0] + u * ds)
                        % self.track_length, d, i, right, left)
        if best is None:
            raise RuntimeError('global waypoint segments are degenerate')
        if hint is not None and math.sqrt(best[0]) > \
                self.projection_reacquire_distance:
            return self.cartesian_to_frenet(x, y, hint=None)
        _, s, d, i, right, left = best
        return s, d, i, right, left

    def cb_obs(self, msg: PerceptionObstacleArray):
        if self.wpnts is None or self.track_length <= 0.0:
            return

        # 맵 필터를 켰는데 맵이 안 오면(맵서버 미기동/죽음) frenet 필터만으로
        # 동작 중임을 알린다 — 조용히 새는 유령의 원인 파악용
        if self.use_map_filter and self.wall_dist is None:
            waited = (self.get_clock().now() - self.start_time).nanoseconds * 1e-9
            if waited > self.map_timeout:
                self.get_logger().warn(
                    f'/map {self.map_timeout:.0f}s 내 미수신 — wall filter unavailable',
                    throttle_duration_sec=10.0)
            if self.map_filter_required:
                # 빈 obstacle을 발행하면 downstream이 이를 "센서 정상 + clear"로
                # 오해한다. health lease가 만료되도록 아예 발행하지 않는다.
                return

        out = ObstacleArray()
        out.header.stamp = msg.header.stamp
        out.header.frame_id = self.target_frame

        # source timestamp의 TF만 사용한다. latest fallback은 명시 opt-in이며,
        # 허용하더라도 최대 시각차를 넘으면 프레임 전체를 폐기한다.
        tf = None
        stamp_time = Time.from_msg(msg.header.stamp)
        stamp_is_zero = stamp_time.nanoseconds == 0
        if stamp_is_zero and self.require_source_stamp:
            self.get_logger().warn(
                '장애물 source timestamp가 없어 frame을 거부함',
                throttle_duration_sec=2.0)
            return
        if not stamp_is_zero:
            source_age = (
                self.get_clock().now() - stamp_time).nanoseconds * 1e-9
            if source_age < -self.source_future_tolerance or \
                    source_age > self.max_source_age:
                self.get_logger().warn(
                    f'장애물 source age {source_age:.3f}s가 허용 범위를 벗어남',
                    throttle_duration_sec=2.0)
                return
        source_ns = stamp_time.nanoseconds or self.get_clock().now().nanoseconds
        if self.last_source_ns is not None and source_ns <= self.last_source_ns:
            self.get_logger().warn(
                'duplicate/out-of-order obstacle source timestamp; frame rejected',
                throttle_duration_sec=2.0)
            return
        if not msg.header.frame_id:
            self.get_logger().warn(
                '장애물 frame_id가 없어 frame을 거부함',
                throttle_duration_sec=2.0)
            return
        if msg.header.frame_id != self.target_frame:
            try:
                tf = self.tf_buffer.lookup_transform(
                    self.target_frame, msg.header.frame_id,
                    Time() if stamp_is_zero else stamp_time,
                    timeout=Duration(seconds=self.tf_timeout))
            except tf2_ros.TransformException as exact_error:
                if self.tf_allow_latest and not stamp_is_zero:
                    try:
                        tf = self.tf_buffer.lookup_transform(
                            self.target_frame, msg.header.frame_id, Time())
                        tf_time = Time.from_msg(tf.header.stamp)
                        tf_age = abs((tf_time - stamp_time).nanoseconds) * 1e-9
                        if tf_age > self.tf_max_age:
                            raise tf2_ros.ExtrapolationException(
                                f'latest TF age {tf_age:.3f}s > {self.tf_max_age:.3f}s')
                    except tf2_ros.TransformException as e:
                        self.get_logger().warn(
                            f'TF {msg.header.frame_id}->{self.target_frame} invalid: {e}',
                            throttle_duration_sec=2.0)
                        return
                else:
                    self.get_logger().warn(
                        f'TF {msg.header.frame_id}->{self.target_frame} at source stamp '
                        f'unavailable: {exact_error}', throttle_duration_sec=2.0)
                    return
        if tf is not None:
            q = tf.transform.rotation
            tx, ty = tf.transform.translation.x, tf.transform.translation.y
            tf_values = (tx, ty, q.x, q.y, q.z, q.w)
            q_norm = math.sqrt(q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w)
            if (not all(math.isfinite(value) for value in tf_values)
                    or q_norm < 1e-9):
                self.get_logger().error(
                    'TF contains non-finite/invalid values; obstacle frame rejected',
                    throttle_duration_sec=2.0)
                return
            qx, qy, qz, qw = (
                q.x / q_norm, q.y / q_norm, q.z / q_norm, q.w / q_norm)
            yaw = math.atan2(2.0 * (qw * qz + qx * qy),
                             1.0 - 2.0 * (qy * qy + qz * qz))
            cos_y, sin_y = math.cos(yaw), math.sin(yaw)

        arrival_sec = self.get_clock().now().nanoseconds * 1e-9
        now_sec = stamp_time.nanoseconds * 1e-9
        if stamp_is_zero:
            now_sec = arrival_sec
            out.header.stamp = self.get_clock().now().to_msg()
        seen_ids = set()

        for src in msg.obstacles:
            if int(src.id) < 0 or int(src.id) in seen_ids:
                self.get_logger().error(
                    f'negative/duplicate obstacle id={src.id}; frame rejected',
                    throttle_duration_sec=2.0)
                return
            values = (
                float(src.confidence), float(src.radius),
                float(src.position.x), float(src.position.y),
            )
            if (not all(math.isfinite(value) for value in values)
                    or not 0.0 <= src.confidence <= 1.0
                    or src.radius < 0.0):
                # 일부 obstacle만 조용히 버리면 그 물체가 사라진 것으로 보인다.
                # frame 전체를 중단해 downstream health lease가 fail-closed 되게 한다.
                self.get_logger().error(
                    f'비유한/음수 obstacle 입력(id={src.id}); frame 거부',
                    throttle_duration_sec=2.0)
                return
            if src.confidence < self.min_confidence:
                continue
            x, y = src.position.x, src.position.y
            if tf is not None:
                x, y = cos_y * x - sin_y * y + tx, sin_y * x + cos_y * y + ty

            prev = self.tracks.get(src.id)
            hint = prev.get('wi') if prev is not None else None
            try:
                s, d, wi, d_right_bound, d_left_bound = \
                    self.cartesian_to_frenet(x, y, hint)
            except RuntimeError as error:
                self.get_logger().error(
                    f'Frenet projection failed: {error}; frame rejected',
                    throttle_duration_sec=2.0)
                return
            raw_half = max(float(src.radius), 0.05)
            # 실제 extent를 줄여 보고하면 큰 장애물의 끝을 통과 가능한 것으로
            # 오판한다. 큰 wall cluster도 보수적으로 전부 유지해 정지시키며,
            # 유령 제거는 map filter가 담당한다.
            half = raw_half
            if raw_half > self.max_extent:
                self.get_logger().warn(
                    f'large obstacle extent preserved: id={src.id}, '
                    f'radius={raw_half:.2f}m',
                    throttle_duration_sec=2.0)

            # centroid가 아니라 obstacle footprint와 drivable corridor의 overlap을
            # 본다. 벽 가까운 실제 물체가 차로로 돌출하면 삭제하면 안 된다.
            corridor_lo = -d_right_bound + self.wall_margin
            corridor_hi = d_left_bound - self.wall_margin
            if d - half > corridor_hi or d + half < corridor_lo:
                continue

            # 맵 벽거리 필터: frenet d는 코너에서 실제 벽거리와 어긋나
            # 벽 위 유령이 위 검사를 통과한다 — 맵 EDT로 실거리 판정.
            # 유령은 벽에서 0~0.1m, 진짜 장애물은 최소 자기 반폭만큼 떨어져
            # 있어 분리가 물리적으로 명확하다. 맵 미수신 시 폴백(통과).
            if self.use_map_filter:
                wall_dist = self.wall_distance(x, y)
                if wall_dist is None:
                    if self.map_filter_required:
                        # map 밖/조회 불가 장애물을 개별 삭제하고 healthy empty를
                        # 발행하면 clear로 오인된다. frame 전체 lease를 만료시킨다.
                        self.get_logger().error(
                            f'obstacle id={src.id} lies outside/unavailable map; '
                            'frame rejected',
                            throttle_duration_sec=2.0)
                        return
                elif wall_dist < self.wall_dist_min:
                    # centroid 기준으로 판정한다. 예전 조건
                    # (wall_dist + raw_half < wall_dist_min)은 raw_half가
                    # wall_dist_min보다 크면 중심이 벽 픽셀 위(거리 0)여도
                    # 절대 삭제되지 않아 필터가 사실상 무력했다 —
                    # run_0803_213313에서 벽거리 0.000m 유령(radius 0.31~0.40)이
                    # 그대로 통과해 spliner TRAILING 정지 라치를 유발했다.
                    # 물리적 분리는 명확하다: 유령 중심은 벽 위(실측 p50 0.000m),
                    # 진짜 차량(0.30m 폭)은 벽에 붙어도 중심이 >=0.15m 떨어진다.
                    continue

            # id별 시간차분으로 map 기준 vs/vd 추정.
            vs, vd = 0.0, 0.0
            velocity_valid = False
            if prev is not None:
                dt = now_sec - prev['t']
                if 1e-3 < dt < 1.0:
                    ds = (s - prev['s'] + self.track_length / 2.0) % self.track_length \
                        - self.track_length / 2.0
                    a = self.lpf_alpha
                    vs = a * (ds / dt) + (1.0 - a) * prev['vs']
                    vd = a * ((d - prev['d']) / dt) + (1.0 - a) * prev['vd']
                    velocity_valid = True
            if not velocity_valid:
                # 첫 관측/재획득: detector의 칼만필터 속도를 Frenet으로 투영해
                # 시드로 쓴다. detector는 target_frame(=map)에서 트래킹하므로
                # 이 속도에는 ego 이동이 섞여 있지 않다 — "laser 프레임이라
                # 신뢰 불가"라던 옛 주석은 target_frame이 base_link였던 시절
                # 것이고 지금은 성립하지 않는다. 게다가 detector는 min_hits(3)
                # 관측을 채운 트랙만 발행하므로 이 시점의 KF 속도는 이미
                # 3프레임 이상의 근거를 갖는다.
                vx_m, vy_m = float(src.velocity.x), float(src.velocity.y)
                if tf is not None:
                    vx_m, vy_m = (cos_y * vx_m - sin_y * vy_m,
                                  sin_y * vx_m + cos_y * vy_m)
                psi = float(self.wpnts[wi, 3])
                c_psi, s_psi = math.cos(psi), math.sin(psi)
                seed_vs = vx_m * c_psi + vy_m * s_psi
                seed_vd = -vx_m * s_psi + vy_m * c_psi
                if (math.isfinite(seed_vs) and math.isfinite(seed_vd)
                        and abs(seed_vs) <= self.max_abs_vs
                        and abs(seed_vd) <= self.max_abs_vd):
                    vs, vd = seed_vs, seed_vd
            if abs(vs) > self.max_abs_vs or abs(vd) > self.max_abs_vd:
                velocity_valid = False
                vs = float(np.clip(vs, -self.max_abs_vs, self.max_abs_vs))
                vd = float(np.clip(vd, -self.max_abs_vd, self.max_abs_vd))
            self.tracks[src.id] = {
                't': now_sec, 'arrival_t': arrival_sec, 's': s, 'd': d,
                'vs': vs, 'vd': vd, 'wi': wi}
            seen_ids.add(src.id)

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
            confidence = max(float(src.confidence), 1e-3)
            pos_var = max(self.position_var_floor,
                          (half * (1.0 - confidence)) ** 2)
            # 속도 미확정 프레임의 분산으로 max_abs_v(=클립 한계)의 제곱을
            # 쓰면 안 된다. 그건 유한차분 추정치를 자르는 위생 한계지 물체가
            # 실제로 낼 수 있는 속도가 아니다. vs_var=100 / vd_var=16이 되어
            # 하류 spliner의 3σ footprint가 종 9.0m / 횡 3.6m로 부풀고,
            # 폭 1.1~2.1m 트랙에서는 좌우 어느 쪽도 통과 불가로 판정돼 매번
            # TRAILING(정지)으로 떨어졌다 (rosbag_0806 4런 68회 = 장애물
            # 획득 1회당 1회, 요구 간격이 전부 3.76~3.88m로 동일).
            # 대신 detector KF 속도의 실측 노이즈를 prior로 쓴다 — 정지
            # 물체에 대해 |v| p50 0.06~0.11 / p90 0.24~0.48 / max 1.20 m/s
            # (rosbag_0806 4런). init_vel_std 0.5는 그 p90 위쪽이다.
            init_vel_var = max(self.velocity_var_floor, self.init_vel_std ** 2)
            vel_var = self.velocity_var_floor if velocity_valid else init_vel_var
            obs.s_var = pos_var
            obs.d_var = pos_var
            obs.vs_var = vel_var
            obs.vd_var = vel_var
            obs.size = 2.0 * half
            obs.is_static = not src.is_dynamic  # 초기값 — spliner cls_*가 재분류
            obs.is_visible = True
            out.obstacles.append(obs)

        # 오래 안 보인 트랙 정리
        stale = [i for i, t in self.tracks.items()
                 if i not in seen_ids and
                 arrival_sec - t.get('arrival_t', arrival_sec) > self.forget_sec]
        for i in stale:
            del self.tracks[i]

        # 빈 배열도 그대로 전달 (spliner obs_timeout과 연동)
        self.last_source_ns = source_ns
        self.pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = PerceptionObstacleBridgeNode()
    # rclpy.spin(node) 금지 — TransformListener 전용 executor에서 노드를 빼앗아
    # TF 콜백을 메인 스레드에 직렬화시킨다 (생성자 주석 참조).
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        # launch가 SIGINT를 보내면 컨텍스트가 이미 shutdown된 상태로 spin이 깨진다.
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
