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


# 클러스터 반경이 이보다 크면 경고만 남긴다 [m]. 출력은 원본 반경을 그대로
# 보존한다(잘라내면 큰 장애물의 실제 크기를 감춰 회피 폭이 모자라진다).
# 예전엔 ROS 파라미터(bridge/max_extent_m)였는데 어떤 yaml 에도 없었고
# 출력에 전혀 영향이 없어서 '크기 상한'으로 오해를 샀다 — 상수로 되돌린다.
_LARGE_EXTENT_WARN_M = 1.50


class PerceptionObstacleBridgeNode(Node):
    def __init__(self):
        super().__init__('perception_obstacle_bridge_node')

        self.declare_parameter('bridge/input_topic', '/perception/obstacles')
        self.declare_parameter('bridge/output_topic', '/perception/obstacles_frenet')
        self.declare_parameter('bridge/target_frame', 'map')
        # detector가 이미 min_hits로 트랙을 확정한다. 브릿지에서 저신뢰 트랙을
        # 삭제하면 가려지는 순간 장애물이 사라지므로 안전 기본값은 0이다.
        self.declare_parameter('bridge/min_confidence', 0.0)
        # min_confidence 를 **외삽(coasting) 트랙에만** 적용할지.
        #   True  → is_visible(이번 프레임 실측)이면 confidence 와 무관하게 통과.
        #           confidence 는 존재 확률이 아니라 정적/동적 분류 진행률이므로
        #           존재 필터로 쓰면 실측된 장애물이 지워진다 (아래 사용부 주석의
        #           run_0812_013630 3:14 사고). 분류 미확정은 삭제 대신 '정적 취급'.
        #   False → 옛 동작(실측이든 외삽이든 confidence 로 삭제). A/B 용.
        self.declare_parameter('bridge/visible_bypasses_confidence', True)
        self.declare_parameter('bridge/wall_margin_m', 0.10)
        # 큰 extent를 축소/삭제하지 않고 진단하는 임계값. 유령 제거는
        # footprint-corridor overlap과 opt-in map filter가 담당한다.
        self.declare_parameter('bridge/vel_lpf_alpha', 0.3)
        self.declare_parameter('bridge/max_abs_vs_mps', 10.0)
        self.declare_parameter('bridge/max_abs_vd_mps', 4.0)
        self.declare_parameter('bridge/position_var_floor', 0.01)
        # confidence -> pos_var 팽창의 **상한** [m].
        # 예전 식은 (half * (1 - confidence))^2 라 confidence -> 0 이면 팽창이
        # half 전체로 커졌다. 그런데 confidence 를 떨어뜨리는 주범은 coasting
        # (detector 가 misses/track_timeout 만큼 감쇠시킨다)이고, coasting 은
        # "존재가 불확실하다"는 뜻이지 "더 큰 물체"라는 뜻이 아니다.
        # 결과적으로 **유령일수록 spliner 눈에 더 큰 장애물**이 돼서 회랑을 더
        # 많이 막았다 (rosbag_0811 분석). 존재 불확실성은 min_confidence 와
        # is_visible 이 담당하고, 여기서는 위치 불확실성만 유한하게 둔다.
        self.declare_parameter('bridge/confidence_var_inflation_max_m', 0.10)
        # ── 정적 장애물 고정 원통 모델 ────────────────────────────────────
        # detector 의 radius 는 "중심에서 가장 먼 점까지"라 **매 프레임 다시**
        # 측정된다. LiDAR 는 앞쪽 호만 보므로 거리·각도·빔 개수에 따라 값이
        # 흔들린다 — 실차 run_0812_013630 실측(정적·is_visible 트랙 36개 /
        # 894프레임)에서 **같은 콘의 트랙 내 변동폭이 p50 0.14 m** (p90 0.19,
        # max 0.21) 였다. 그런데 아래에서 obs.d_left/d_right = d ± half 이므로
        # **장애물 가장자리가 프레임마다 ±0.07 m 움직인다.**
        # 회피 레인 개폐를 결정한 마진이 6 cm 였다(우측 d_hold -0.40 vs 필요
        # -0.46) — 판정 노이즈가 판정 마진보다 컸다.
        #
        # 크기를 매번 재지 않고 알려진 원통으로 고정한다. 세 조건을 모두
        # 만족할 때만 적용하고, 하나라도 어긋나면 측정값을 그대로 둔다.
        #   ① is_static      — 동적 상대는 크기가 실제로 다르다
        #   ② band 안        — 밖은 콘이라고 믿을 근거가 없다
        #   ③ 벽에서 떨어짐  — 아래 open_space 주석
        #
        # ⚠️ radius 는 **실제로 트랙에 놓는 물체**에 맞춰야 한다. 0.15 는
        #    대회 기준(최대 지름 30 cm)이다. 그보다 큰 콘으로 테스트하는
        #    동안에는 이 값을 그 콘에 맞게 올릴 것 — 큰 물체를 0.15 로
        #    퉁치면 매 프레임 그 차이만큼 **작게** 본다.
        self.declare_parameter('bridge/static_cylinder_radius_m', 0.15)
        # 적용 대역 [m]. 상한이 곧 **최대 피해량의 상한**이다:
        #   실제로 잃는 extent <= (상한 - radius) = 0.25 - 0.15 = 0.10 m
        # 상한 근거(같은 백): 콘 프레임의 97% 가 0.25 이하. 0.22 이하로
        # 낮추면 크게 측정된 소수 프레임이 대역 밖으로 빠지는데, 가장자리
        # 흔들림을 만드는 게 바로 그 프레임들이라 **이득이 0 이 된다**
        # (트랙 내 변동폭 중앙값: 상한 0.18/0.20/0.22 모두 0.084 m 그대로,
        #  0.25 에서 0.000 m). 0.25~0.35 구간(전체의 3%)은 애매하므로 원시값
        # 유지. 0.35 초과는 detector 가 이미 벽으로 거른다
        # (obstacle_detector_node max_cluster_radius_m).
        self.declare_parameter('bridge/static_cylinder_band_m', [0.05, 0.25])
        # 장애물 **중심**이 벽에서 이만큼 떨어져 있을 때만 고정한다.
        # 벽에 붙으면 클러스터가 벽 점을 같이 물어 extent 가 실제로 커진다 —
        # 같은 백에서 벽거리 0~0.30 m 구간의 radius p50 이 0.20 으로, 그 바깥
        # (0.30~0.50 m)의 0.13 보다 크다. 거기서 0.15 로 줄이면 **회랑이 가장
        # 좁은 곳에서 판정을 여는 방향으로** 최대 0.10 m 밀린다. 차폭짜리
        # 가짜 통로가 생기진 않았지만(874프레임 중 0건) 레인 판정을 뒤집기엔
        # 충분하다(실측 레인 개폐 마진 6 cm).
        # 0.30 근거: 오염이 관측되는 경계가 정확히 여기다. 그리고 중심 0.30 은
        # 표면이 벽에서 0.15 m 떨어졌다는 뜻이라 클러스터가 안 붙는다.
        # 적용률 69%(733/1056) — 이 조건을 아예 빼도 80% 라 손실이 작다.
        self.declare_parameter('bridge/static_cylinder_open_space_m', 0.30)
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
        self.visible_bypasses_confidence = bool(
            p('bridge/visible_bypasses_confidence').value)
        self.wall_margin = float(p('bridge/wall_margin_m').value)
        self.lpf_alpha = float(p('bridge/vel_lpf_alpha').value)
        self.max_abs_vs = float(p('bridge/max_abs_vs_mps').value)
        self.max_abs_vd = float(p('bridge/max_abs_vd_mps').value)
        self.position_var_floor = float(
            p('bridge/position_var_floor').value)
        self.conf_inflation_max = float(
            p('bridge/confidence_var_inflation_max_m').value)
        self.cyl_radius = float(p('bridge/static_cylinder_radius_m').value)
        _band = [float(v) for v in p('bridge/static_cylinder_band_m').value]
        self.cyl_lo, self.cyl_hi = (_band + [0.0, 0.0])[:2]
        self.cyl_open_space = float(
            p('bridge/static_cylinder_open_space_m').value)
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
            self.min_confidence, self.wall_margin,
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
                or not 0.0 <= self.lpf_alpha <= 1.0
                or not 0.1 <= self.max_abs_vs <= 50.0
                or not 0.1 <= self.max_abs_vd <= 20.0
                or not 1e-6 <= self.position_var_floor <= 100.0
                # 대역이 뒤집히거나 radius 가 대역 밖이면 "고정"이 오히려
                # extent 를 키우거나 줄이는 방향을 예측할 수 없게 된다.
                or not 0.0 < self.cyl_lo < self.cyl_hi <= 1.0
                or not self.cyl_lo <= self.cyl_radius <= self.cyl_hi
                or not 0.0 <= self.cyl_open_space <= 2.0
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
            # ── 존재 판정과 분류 안정성을 분리한다 (2026-08-12) ──────────────
            # confidence 는 "존재 확률"이 **아니다**. classifier.cpp:78 의
            #   confidence = static_streak / n_release   (정적 트랙)
            # 즉 "정적/동적 판정까지의 진행률"이다. 그리고 static_streak 은
            #   moving = (speed >= v_thresh 0.40) or (spread >= dynamic_pos_std 0.30)
            # 이 **한 프레임만** 참이면 0으로 리셋된다.
            #
            # 그 진행률을 존재 필터로 쓰면 범주 오류가 된다. 실제 사고:
            #   run_0812_013630 3:14 — 트랙 205 는 id 가 끊긴 적 없고
            #   is_visible=1(매 프레임 LiDAR 실측, misses=0)이었는데, 접근 중
            #   콘의 단면이 바뀌며(radius 0.22->0.21->0.19->0.16) 중심점이 몇 cm
            #   튀어 한 프레임이 moving 으로 읽혔다. confidence 0.45 -> 0.00.
            #   그 뒤 0.05/프레임으로 회복해 0.20 에 닿기까지 **4프레임=0.25s**,
            #   그동안 이 게이트가 콘을 지웠다. 플래너는 "앞이 비었다"로 보고
            #   계획 속도를 5.00 -> 6.50 m/s 로 올렸고, 장애물이 1.99m 앞에서
            #   돌아왔을 때는 어떤 회피도 성립하지 않았다.
            #
            # 교차표 (같은 백, 원시 검출 2087 트랙-프레임):
            #             vis=1(실측)  vis=0(외삽)
            #   conf>=0.2      839         510   <- 유령 510건이 통과 (게이트가 샌다)
            #   conf< 0.2      217         521   <- 실물 217건이 삭제 (위 사고)
            # 초기 획득 거리 손실은 0.00 m 다 — min_hits(3)가 이미 그 역할을 하므로
            # 이 게이트의 손실은 전부 **주행 중 리셋**에서 나오고, 하필 근거리에서 터진다.
            #
            # 그래서 게이트를 없애지 않고 **적용 대상만 좁힌다**:
            #   · is_visible(이번 프레임 실측) 이면 통과 — 존재는 측정이 증명한다
            #   · 외삽(coasting) 이면 종전대로 confidence 게이트 유지
            # 두 축 모두에서 종전보다 나쁘지 않다: 실물 217건을 되살리고,
            # 코스팅 처리는 그대로다(= 가려진 실물을 새로 버리지 않는다).
            # 남은 누수(conf>=0.2 인 코스팅 510건)는 여기서 더 조이지 않았다 —
            # 조이면 짧게 가려진 **실물**을 지우게 되고, 그건 같은 사고의 반대
            # 방향이다. 그쪽은 footprint-corridor overlap 이 담당한다.
            measured = bool(src.is_visible)
            if src.confidence < self.min_confidence:
                if not (self.visible_bypasses_confidence and measured):
                    continue
            # 분류 진행률이 낮은 트랙은 **버리지 않고 보수적으로 취급**한다.
            # 정적으로 보면 기하 회피(회랑 판정)로 처리되고, 동적으로 잘못 보면
            # 없는 운동을 예측해 회랑을 엉뚱하게 비운다 — 후자가 더 위험하다.
            # (spliner 의 cls_* 가 몇 프레임 뒤 어차피 재분류한다.)
            unsettled_class = src.confidence < self.min_confidence
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
            if raw_half > _LARGE_EXTENT_WARN_M:
                self.get_logger().warn(
                    f'large obstacle extent preserved: id={src.id}, '
                    f'radius={raw_half:.2f}m',
                    throttle_duration_sec=2.0)
            # 고정 원통 모델 (선언부 주석의 세 조건).
            # 벽거리는 **중심 기준**이다. 가장자리 기준(wd - raw_half)으로 두면
            # 사실상 wd > 0.30 + raw_half ~= 0.48 을 요구하게 되어, 실측상 가장
            # 깨끗한 구간(벽거리 0.30~0.50 m, 457프레임, radius p50 0.13 —
            # 가정값 0.15 와 거의 일치)이 통째로 빠진다. 오염이 실제로 관측되는
            # 건 중심이 0.30 m 안쪽일 때뿐이다(거기서만 radius p50 이 0.20 으로
            # 뜬다). 물리적으로도 같은 결론이다 — 중심 0.30 이면 표면이 벽에서
            # 0.15 m 떨어져 있어 클러스터가 벽 점까지 물지 못한다.
            # 실측 A/B(run_0812_013630, 36트랙): 가장자리 기준은 적용률 34% /
            # 변동폭 0.140->0.107 m, 중심 기준은 69% / 0.140->0.084 m 이고
            # 적용당 위험 비율(11% vs 13%)과 최대 손실(0.098 vs 0.099 m)은
            # 사실상 같다 — 가장자리 기준은 더 안전한 게 아니라 덜 할 뿐이다.
            #
            # 맵 미수신(wall_distance None)이면 적용하지 않는다: 벽 근처인지
            # 모르는 채 extent 를 줄이는 것이 이 변경의 유일한 위험 방향이다.
            if src.is_static and self.cyl_lo <= raw_half <= self.cyl_hi:
                wd = self.wall_distance(x, y)
                if wd is not None and wd > self.cyl_open_space:
                    half = self.cyl_radius

            # centroid가 아니라 obstacle footprint와 drivable corridor의 overlap을
            # 본다. 벽 가까운 실제 물체가 차로로 돌출하면 삭제하면 안 된다.
            # ⚠️ 여기는 **raw_half** 를 쓴다 (고정 전 값). 삭제 판정이라
            # extent 를 줄이면 "살짝 걸친 진짜 물체"를 지우는 쪽으로 움직인다 —
            # 고정 원통은 보고값에만 적용하고 존재 판정은 원시 extent 로 둔다.
            corridor_lo = -d_right_bound + self.wall_margin
            corridor_hi = d_left_bound - self.wall_margin
            if d - raw_half > corridor_hi or d + raw_half < corridor_lo:
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
            # 팽창을 conf_inflation_max 로 자른다 (선언부 주석 참조) — 저신뢰
            # 검출이 더 **큰** 장애물이 되어 회랑을 막던 역전을 없앤다.
            inflation = min(half * (1.0 - confidence), self.conf_inflation_max)
            pos_var = max(self.position_var_floor, inflation ** 2)
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
            if unsettled_class and not obs.is_static:
                # 위 주석 참조: 분류 미확정 트랙은 정적으로 눌러 통과시킨다.
                # 삭제(옛 동작)와 달리 장애물은 그대로 남고, 취급만 보수적이다.
                obs.is_static = True
                self.get_logger().info(
                    f'분류 미확정(conf={src.confidence:.2f} < '
                    f'{self.min_confidence:.2f}, id={src.id}) — 정적으로 취급',
                    throttle_duration_sec=2.0)
            # detector 의 실제 관측 여부를 그대로 전달한다. 예전에는 True 로
            # 하드코딩돼 있어 coasting 외삽이 측정과 구분되지 않았다.
            obs.is_visible = bool(src.is_visible)
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
