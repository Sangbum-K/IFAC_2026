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
3) LaserScan의 원본 시각으로 TF를 조회한다.
   - 움직이는 차량에서 "최신 TF"로 과거 스캔을 변환하면 장애물 위치가 속도×지연만큼
     어긋난다. exact-time TF가 없으면 기본적으로 해당 스캔을 발행하지 않는다.
4) TF lookup 실패 시 해당 스캔은 스킵하고 경고 로그만 남김 (잘못된 좌표로 publish하지 않음).
"""
import numpy as np
import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    QoSProfile,
    ReliabilityPolicy,
    qos_profile_sensor_data,
)
from rclpy.time import Time
from rclpy.duration import Duration

import tf2_ros
from tf2_ros import LookupException, ConnectivityException, ExtrapolationException

from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Point, Vector3

try:
    from scipy.ndimage import distance_transform_edt
except ImportError:      # scipy 없는 환경(다른 개발 PC 등)은 맵 필터 없이 폴백
    distance_transform_edt = None

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
        # 정적/동적 위치분산 근거 (map 프레임)
        self.declare_parameter('static_pos_std', 0.08)   # 이 미만 분산 = 정적 [m]
        self.declare_parameter('dynamic_pos_std', 0.20)  # 이 초과 분산 = 동적 [m]
        self.declare_parameter('dynamic_release', 40)    # 상대(동적) 정적복귀 프레임 (길게)
        # 재식별(occlusion 후 같은 id 부활) + 미확정 트랙 게이팅
        self.declare_parameter('reid_distance', 0.7)     # 재식별 허용 거리 [m]
        self.declare_parameter('reid_timeout_sec', 1.0)  # 소멸 트랙 보관 시간 [s]
        self.declare_parameter('pos_hist_len', 20)       # 위치분산 판별 윈도우 [frames]
        self.declare_parameter('min_hits', 3)            # 이만큼 관측돼야 장애물로 발행

        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('obstacle_topic', '/perception/obstacles')
        self.declare_parameter('target_frame', 'map')   # frame_id_override 대체
        self.declare_parameter('tf_timeout_sec', 0.05)
        self.declare_parameter('require_source_stamp', True)
        self.declare_parameter('max_source_age_sec', 0.20)
        self.declare_parameter('source_future_tolerance_sec', 0.05)
        # 고속주행에서는 latest TF fallback이 위치를 체계적으로 틀리게 하므로 기본 금지.
        # 개발용으로 명시 opt-in해도 원본 스캔 시각과 충분히 가까운 TF만 허용한다.
        self.declare_parameter('tf_allow_latest_fallback', False)
        self.declare_parameter('tf_max_age_sec', 0.05)
        # target_frame(map) 기준. 기본 [0.0]은 "비활성" sentinel —
        # 빈 리스트 기본값은 rclpy에서 타입 미확정(ParameterUninitializedException)이 된다.
        self.declare_parameter('boundary_polygon_xy', [0.0])
        # ---- 맵 벽 포인트 필터 ----
        # 좁은 실내 트랙(11x9m)에서는 range_max 안에 맞은편 벽까지 전부 들어온다.
        # 벽 리턴은 연속이라 breakpoint 클러스터링이 벽 전체를 한 클러스터로
        # 묶고, centroid가 트랙 한복판에 반경 수 m짜리 가짜 장애물로 찍힌다
        # (run_0803_204743 실측: r=5.17m 장애물 하나가 레이스라인 314점 중
        # 207점을 덮음 -> 회피 불가 -> TRAILING 정지).
        # boundary_polygon으로는 못 막는다: 루프 트랙의 주행영역은 고리(annulus)라
        # 단일 폴리곤으로 표현되지 않아 인필드 벽이 그대로 남는다.
        # 그래서 클러스터링 '전에' /map EDT로 벽에 붙은 포인트를 제거한다.
        # (브릿지의 map filter는 클러스터 centroid만 보므로 이 병합을 못 잡는다)
        self.declare_parameter('use_map_filter', False)
        self.declare_parameter('map_topic', '/map')
        # 이 거리 안쪽(맵 벽까지)의 스캔 포인트는 벽 리턴으로 보고 버린다.
        # PF 보정 오차보다 크면 진짜 장애물의 벽쪽 면까지 깎이고, 작으면
        # 벽 리턴이 샌다. 차폭 0.30m -> 벽에 붙은 상대도 중심은 >=0.15m 떨어져
        # 있으므로 트랙쪽 면 포인트가 남아 계속 탐지된다.
        self.declare_parameter('wall_dist_min_m', 0.15)
        # 맵을 기다리며 발행을 보류할 최대 시간. 초과하면 벽 필터 없이
        # 진행한다(시끄러운 error 로그와 함께) — 맵서버가 죽었을 때
        # 조용히 영구 침묵해 회피 자체를 못 하는 상태로 굳는 것을 막는다.
        # /map은 latched라 정상이면 1초 내에 온다(실측 브릿지 0.18s).
        self.declare_parameter('map_wait_timeout_sec', 10.0)
        # 물리적으로 불가능한 크기의 클러스터 거부. 0.0 = 비활성.
        # F1TENTH 차량은 0.30x0.55m -> 반경 ~0.32m. 맵 필터를 통과한 잔여
        # 벽/구조물이 이보다 크면 장애물이 아니다. 맵 필터가 PF 오차로 흔들려도
        # 거대 클러스터가 새는 것을 막는 2차 방어선.
        self.declare_parameter('max_cluster_radius_m', 0.0)
        # 스캔 처리율 상한. gym /scan은 ~70Hz인데 전체 파이프라인을 매 스캔
        # 돌리면 싱글스레드 executor가 TF listener 콜백을 굶겨 (250Hz*8프레임)
        # lookup이 "latest common time" 과거 고정으로 실패한다 — 실측 원인.
        self.declare_parameter('process_rate_hz', 20.0)

        p = self.get_parameter
        self.range_min = float(p('range_min').value)
        self.range_max = float(p('range_max').value)
        self.fov_min = float(p('fov_min_deg').value) * np.pi / 180.0
        self.fov_max = float(p('fov_max_deg').value) * np.pi / 180.0
        self.median_kernel = int(p('median_kernel').value)

        self.cluster_eps = float(p('cluster_eps').value)
        self.min_cluster_size = int(p('min_cluster_size').value)

        self.v_thresh = float(p('v_thresh').value)
        self.n_confirm = int(p('n_confirm').value)
        self.n_release = int(p('n_release').value)
        self.static_pos_std = float(p('static_pos_std').value)
        self.dynamic_pos_std = float(p('dynamic_pos_std').value)
        self.dynamic_release = int(p('dynamic_release').value)
        self.min_hits = int(p('min_hits').value)
        gating_distance = float(p('gating_distance').value)
        track_timeout = int(p('track_timeout').value)
        reid_distance = float(p('reid_distance').value)
        reid_timeout = float(p('reid_timeout_sec').value)
        pos_hist_len = int(p('pos_hist_len').value)

        rate = float(p('process_rate_hz').value)
        tf_timeout_sec = float(p('tf_timeout_sec').value)
        self.require_source_stamp = bool(p('require_source_stamp').value)
        self.max_source_age = float(p('max_source_age_sec').value)
        self.source_future_tolerance = float(
            p('source_future_tolerance_sec').value)
        self.tf_allow_latest_fallback = bool(
            p('tf_allow_latest_fallback').value)
        self.tf_max_age = float(p('tf_max_age_sec').value)
<<<<<<< Updated upstream
=======
        self.tf_fixed_frame = str(p('tf_fixed_frame').value).strip()
        self.use_map_filter = bool(p('use_map_filter').value)
        self.wall_dist_min = float(p('wall_dist_min_m').value)
        self.max_cluster_radius = float(p('max_cluster_radius_m').value)
        self.map_wait_timeout = float(p('map_wait_timeout_sec').value)
        if (not np.isfinite(self.wall_dist_min)
                or not 0.0 <= self.wall_dist_min <= 2.0
                or not np.isfinite(self.max_cluster_radius)
                or not 0.0 <= self.max_cluster_radius <= 100.0
                or not np.isfinite(self.map_wait_timeout)
                or not 0.0 <= self.map_wait_timeout <= 120.0):
            raise ValueError(
                'invalid wall_dist_min_m/max_cluster_radius_m/map_wait_timeout_sec')
>>>>>>> Stashed changes
        timing_values = (
            rate, tf_timeout_sec, self.max_source_age,
            self.source_future_tolerance, self.tf_max_age,
        )
        if (not all(np.isfinite(v) for v in timing_values)
                or not 1.0 <= rate <= 500.0
                or not 0.001 <= tf_timeout_sec <= 5.0
                or not 0.01 <= self.max_source_age <= 5.0
                or not 0.0 <= self.source_future_tolerance <= 1.0
                or not 0.0 <= self.tf_max_age <= self.max_source_age):
            raise ValueError('invalid detector timing/freshness parameters')
        continuous_values = (
            self.range_min, self.range_max, self.fov_min, self.fov_max,
            self.cluster_eps, self.v_thresh, self.static_pos_std,
            self.dynamic_pos_std, gating_distance, reid_distance, reid_timeout,
        )
        if (not all(np.isfinite(v) for v in continuous_values)
                or not 0.0 <= self.range_min < self.range_max <= 100.0
                or not -2.0 * np.pi <= self.fov_min < self.fov_max <= 2.0 * np.pi
                or self.median_kernel < 1 or self.median_kernel % 2 == 0
                or not 0.001 <= self.cluster_eps <= 10.0
                or not 1 <= self.min_cluster_size <= 100000
                or not 0.001 <= gating_distance <= 100.0
                or not 1 <= track_timeout <= 100000
                or not 0.0 <= self.v_thresh <= 100.0
                or not 1 <= self.n_confirm <= 100000
                or not 1 <= self.n_release <= 100000
                or not 0.0 <= self.static_pos_std <= self.dynamic_pos_std <= 100.0
                or not 1 <= self.dynamic_release <= 100000
                or not 0.0 <= reid_distance <= 100.0
                or not 0.0 <= reid_timeout <= 60.0
                or not 2 <= pos_hist_len <= 100000
                or not 1 <= self.min_hits <= track_timeout):
            raise ValueError('invalid detector geometry/tracking parameters')

        self.min_period = 1.0 / rate
        self.last_processed = -1e18
        self.last_seen_stamp = -1e18
        self.target_frame = str(p('target_frame').value).strip()
        if not self.target_frame:
            raise ValueError('target_frame must not be empty')
        self.tf_timeout = Duration(seconds=tf_timeout_sec)
        self.boundary_polygon = self._parse_boundary_polygon(
            p('boundary_polygon_xy').value
        )

        self.tracker = Tracker(
            gating_distance=gating_distance,
            track_timeout=track_timeout,
            reid_distance=reid_distance,
            reid_timeout=reid_timeout,
            pos_hist_len=pos_hist_len,
        )

        scan_topic = str(p('scan_topic').value).strip()
        obstacle_topic = str(p('obstacle_topic').value).strip()
        if not scan_topic or not obstacle_topic:
            raise ValueError('scan_topic and obstacle_topic must not be empty')

        # ---- TF2 ----
        # /tf가 고빈도면 TF 콜백이 스캔 처리에 밀려 굶고, lookup이
        # "frame does not exist"/과거 고정 실패로 수십 초 눈이 먼다 (실측).
        #
        # spin_thread=True는 이 문제를 못 고친다 — 오히려 조용히 무력화된다:
        # TransformListener는 전용 executor를 만들어 node를 add_node하는데,
        # 그 뒤 main()의 rclpy.spin(node)이 글로벌 executor에 같은 node를 넣으면
        # rclpy의 Node.executor setter가 *기존 executor에서 노드를 제거*한다
        # (rclpy/node.py: `current_executor.remove_node(self)`).
        # 결과: 전용 스레드는 노드 없는 빈 executor를 돌고, /tf 콜백은 결국
        # 무거운 on_scan과 같은 단일 스레드 큐에 실린다. TransformListener의
        # /tf QoS depth가 100이고 /tf는 ~61 msg/s라 큐가 포화되면 버퍼가
        # 100/61 ~= 1.6초 늙는다 — run_0803_204743에서 |tf-scan|이 1.3~2.1s로
        # 관측되어 스캔 7576개 중 13개만 살아남았다.
        #
        # 그래서 spin_thread=False + MultiThreadedExecutor로 간다. TF 콜백은
        # TransformListener가 만드는 ReentrantCallbackGroup에 있으므로 on_scan과
        # 병렬로 돌고, on_scan 자신은 기본 MutuallyExclusive 그룹이라 여전히
        # 한 번에 하나만 실행된다(트래커 상태 보호). main() 참조.
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(
            self.tf_buffer, self, spin_thread=False)

        self.sub = self.create_subscription(
            LaserScan, scan_topic, self.on_scan, qos_profile_sensor_data
        )
        self.pub = self.create_publisher(ObstacleArray, obstacle_topic, 10)

        # ---- 맵 벽거리(EDT) 필터 ----
        self.wall_dist = None
        self.map_origin = (0.0, 0.0)
        self.map_origin_yaw = 0.0
        self.map_res = 0.0
        # 맵 구독을 걸었지만 아직 첫 맵을 못 받은 상태. True인 동안 on_scan은
        # 발행을 보류한다 — fail-open으로 벽 파편을 흘리는 것보다 안전하다.
        self.awaiting_map = False
        # on_scan과 같은 시계(스캔 stamp와 비교 가능)로 대기시간을 잰다.
        self.start_time = self.get_clock().now().nanoseconds * 1e-9
        if self.use_map_filter:
            map_topic = str(p('map_topic').value).strip()
            if not map_topic:
                raise ValueError('map_topic must not be empty when use_map_filter')
            if distance_transform_edt is None:
                self.get_logger().error(
                    'use_map_filter=true지만 scipy가 없음 — 벽 필터 없이 동작한다. '
                    '(벽이 거대 클러스터로 잡혀 정지할 수 있음)')
            else:
                # map_server는 latched(TRANSIENT_LOCAL) 발행 — 기본 QoS로는 못 받는다
                map_qos = QoSProfile(
                    depth=1,
                    reliability=ReliabilityPolicy.RELIABLE,
                    durability=DurabilityPolicy.TRANSIENT_LOCAL)
                # cb_map을 on_scan과 같은 기본 MutuallyExclusive 그룹에 두면
                # 40Hz 스캔 큐가 비지 않는 한 cb_map이 영원히 순번을 못 받는다
                # — run_0803_213313에서 latched /map을 78초간 못 받아 벽 필터가
                # 통째로 죽었다 (같은 QoS의 브릿지는 0.2초 만에 수신).
                # 전용 Reentrant 그룹이면 MultiThreadedExecutor가 병렬 실행한다.
                self.map_cb_group = ReentrantCallbackGroup()
                self.create_subscription(
                    OccupancyGrid, map_topic, self.cb_map, map_qos,
                    callback_group=self.map_cb_group)
                self.awaiting_map = True

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
        if flat_xy is None or len(flat_xy) < 2:  # 미설정/sentinel -> 비활성
            return None
        if len(flat_xy) < 6 or len(flat_xy) % 2 != 0:
            raise ValueError(
                'boundary_polygon_xy must contain an even number of values '
                'with at least 3 vertices'
            )
        polygon = []
        for i in range(0, len(flat_xy), 2):
            polygon.append([float(flat_xy[i]), float(flat_xy[i + 1])])
        if not np.all(np.isfinite(polygon)):
            raise ValueError(
                'boundary_polygon_xy contains non-finite coordinates')
        return polygon

    def cb_map(self, msg: OccupancyGrid):
        """노드 수명 중 1회: 셀별 최근접 벽거리 테이블(EDT) 계산."""
        if self.wall_dist is not None:
            return
        grid_frame = msg.header.frame_id.strip().lstrip('/')
        if grid_frame and grid_frame != self.target_frame.lstrip('/'):
            # 좌표계가 다르면 EDT 인덱싱이 통째로 틀린다 — 켜지 않는 게 낫다.
            self.get_logger().error(
                f'맵 프레임({grid_frame})이 target_frame({self.target_frame})과 '
                '다름 — 벽 필터 비활성')
            self.awaiting_map = False   # 영구 불가 — 발행 보류를 풀고 fail-open
            return
        if (msg.info.width == 0 or msg.info.height == 0
                or not np.isfinite(msg.info.resolution)
                or msg.info.resolution <= 0.0):
            self.get_logger().error('invalid occupancy grid; 벽 필터 비활성')
            self.awaiting_map = False
            return
        q = msg.info.origin.orientation
        q_norm = float(np.linalg.norm([q.x, q.y, q.z, q.w]))
        origin_values = (msg.info.origin.position.x, msg.info.origin.position.y,
                         q.x, q.y, q.z, q.w)
        if (len(msg.data) != int(msg.info.width) * int(msg.info.height)
                or not all(np.isfinite(v) for v in origin_values)
                or q_norm < 1e-9):
            self.get_logger().error(
                'occupancy grid data/origin이 malformed; 벽 필터 비활성')
            self.awaiting_map = False
            return
        grid = np.array(msg.data, dtype=np.int8).reshape(
            msg.info.height, msg.info.width)
        # unknown(-1)도 free로 증명되지 않았으므로 벽으로 취급한다 — 맵 밖
        # 영역의 스캔 리턴(트랙 밖으로 새어나간 빔)이 장애물이 되는 것을 막는다.
        occupied = (grid < 0) | (grid >= 65)
        self.wall_dist = (
            distance_transform_edt(~occupied) * msg.info.resolution
        ).astype(np.float32)
        self.map_origin = (msg.info.origin.position.x,
                           msg.info.origin.position.y)
        qx, qy, qz, qw = (q.x / q_norm, q.y / q_norm, q.z / q_norm, q.w / q_norm)
        self.map_origin_yaw = float(np.arctan2(
            2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz)))
        self.map_res = float(msg.info.resolution)
        self.awaiting_map = False
        self.get_logger().info(
            f'맵 벽거리 필터 활성: {msg.info.width}x{msg.info.height} '
            f'res={self.map_res:.3f} wall_dist_min={self.wall_dist_min:.2f}m')

    def reject_wall_points(self, points):
        """맵 벽에서 wall_dist_min 이내인 포인트를 제거한다 (target_frame 좌표).

        scipy 부재/맵 영구 불가면 그대로 통과(fail-open)시킨다. 맵을 '아직'
        기다리는 동안은 on_scan이 발행 자체를 보류하므로(awaiting_map) 여기로
        오지 않는다 — fail-open 상태의 벽 파편 발행이 run_0803_213313에서
        TRAILING 정지 라치를 유발했기 때문.
        """
        if self.wall_dist is None or len(points) == 0:
            return points
        dx = points[:, 0] - self.map_origin[0]
        dy = points[:, 1] - self.map_origin[1]
        c_yaw, s_yaw = np.cos(self.map_origin_yaw), np.sin(self.map_origin_yaw)
        cols = np.floor((c_yaw * dx + s_yaw * dy) / self.map_res).astype(np.int64)
        rows = np.floor((-s_yaw * dx + c_yaw * dy) / self.map_res).astype(np.int64)
        h, w = self.wall_dist.shape
        inside = (rows >= 0) & (rows < h) & (cols >= 0) & (cols < w)
        keep = np.zeros(len(points), dtype=bool)   # 맵 밖 포인트는 버린다
        if np.any(inside):
            keep[inside] = (
                self.wall_dist[rows[inside], cols[inside]] >= self.wall_dist_min)
        return points[keep]

    def on_scan(self, msg: LaserScan):
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        source_frame = msg.header.frame_id.strip()
        now = self.get_clock().now().nanoseconds * 1e-9

        if not source_frame:
            self.get_logger().warn(
                'LaserScan has an empty frame_id; skipping.',
                throttle_duration_sec=2.0,
            )
            return
        if not np.isfinite(stamp) or (self.require_source_stamp and stamp <= 0.0):
            self.get_logger().warn(
                'LaserScan has a missing/non-finite source stamp; skipping.',
                throttle_duration_sec=2.0,
            )
            return
        age = now - stamp
        if (not np.isfinite(age) or age > self.max_source_age
                or age < -self.source_future_tolerance):
            self.get_logger().warn(
                f'LaserScan source stamp is outside the freshness window '
                f'(age={age:.3f}s); skipping.',
                throttle_duration_sec=2.0,
            )
            return
        if stamp <= self.last_seen_stamp:
            self.get_logger().warn(
                'Duplicate or out-of-order LaserScan stamp; skipping.',
                throttle_duration_sec=2.0,
            )
            return
        self.last_seen_stamp = stamp
        if stamp - self.last_processed < self.min_period:
            return
        self.last_processed = stamp

        # 맵 필터가 켜져 있고 맵을 기다리는 중이면 발행 자체를 보류한다.
        # fail-open으로 돌리면 벽 파편이 conf 1.0 장애물로 나간다
        # (run_0803_213313: 유령 416건, 전부 벽거리 <=0.10m). 하류 spliner는
        # 장애물 프레임을 한 번도 못 받은 동안에는 정상 주행하므로(발행 후
        # 끊김만 fail-safe 정지) 보류가 정지 라치보다 안전하다. /map은
        # latched라 구독 콜백만 돌면 수 초 내 풀린다.
        # 단, 무한정 보류하면 맵서버가 죽었을 때 회피 자체를 못 하는 상태로
        # 조용히 굳는다 — 제한 시간 뒤에는 시끄럽게 fail-open으로 내려간다.
        # (브릿지의 centroid 벽거리 필터가 2차 방어선으로 남아 있다)
        if self.awaiting_map:
            waited = now - self.start_time
            if waited < self.map_wait_timeout:
                self.get_logger().warn(
                    'use_map_filter=true, /map 수신 대기 중 — 장애물 발행 보류',
                    throttle_duration_sec=5.0)
                return
            self.awaiting_map = False
            self.get_logger().error(
                f'/map을 {self.map_wait_timeout:.0f}s 내 못 받음 — 벽 필터 없이 '
                '발행 시작 (벽 파편이 유령 장애물로 나갈 수 있음)')

        ranges = np.asarray(msg.ranges, dtype=np.float64)
        scan_metadata = (
            msg.angle_min, msg.angle_max, msg.angle_increment,
            msg.range_min, msg.range_max,
        )
        if (ranges.ndim != 1 or ranges.size == 0
                or not all(np.isfinite(v) for v in scan_metadata)
                or msg.angle_increment <= 0.0
                or msg.range_min < 0.0 or msg.range_max <= msg.range_min
                or not np.any((np.isfinite(ranges) & (ranges > 0.0))
                              | np.isposinf(ranges))):
            self.get_logger().warn(
                'Malformed LaserScan metadata/ranges; skipping.',
                throttle_duration_sec=2.0,
            )
            return

        # ① 전처리 (laser frame 기준, range/FOV/median filter만)
        points = preprocess_scan(
            msg.ranges, msg.angle_min, msg.angle_increment,
            range_min=self.range_min, range_max=self.range_max,
            fov_min=self.fov_min, fov_max=self.fov_max,
            median_kernel=self.median_kernel,
        )

        # ①.5 TF 변환: laser frame -> target_frame (보통 map)
        # 이 변환이 있어야 이후 boundary_polygon(map 기준)과 최종 publish 좌표가 실제로 맞음.
        if self.target_frame != source_frame:
            scan_time = Time.from_msg(msg.header.stamp)
            try:
                tf_stamped = self.tf_buffer.lookup_transform(
                    self.target_frame, source_frame, scan_time,
                    timeout=self.tf_timeout
                )
            except (LookupException, ConnectivityException,
                    ExtrapolationException) as exact_ex:
                if not self.tf_allow_latest_fallback:
                    self.get_logger().warn(
                        f'Exact-time TF lookup {source_frame} -> '
                        f'{self.target_frame} failed: {exact_ex}. '
                        'Skipping this scan.',
                        throttle_duration_sec=2.0,
                    )
                    return
                try:
                    tf_stamped = self.tf_buffer.lookup_transform(
                        self.target_frame, source_frame, Time(),
                        timeout=self.tf_timeout
                    )
                except (LookupException, ConnectivityException,
                        ExtrapolationException) as latest_ex:
                    self.get_logger().warn(
                        f'TF lookup {source_frame} -> {self.target_frame} '
                        f'failed: {latest_ex}. Skipping this scan.',
                        throttle_duration_sec=2.0,
                    )
                    return
                tf_stamp = (tf_stamped.header.stamp.sec
                            + tf_stamped.header.stamp.nanosec * 1e-9)
                if (not np.isfinite(tf_stamp) or tf_stamp <= 0.0
                        or abs(tf_stamp - stamp) > self.tf_max_age):
                    self.get_logger().warn(
                        'Latest TF is not close enough to the LaserScan '
                        'source stamp; skipping.',
                        throttle_duration_sec=2.0,
                    )
                    return

            t = tf_stamped.transform.translation
            q = tf_stamped.transform.rotation
            transform_values = (t.x, t.y, q.x, q.y, q.z, q.w)
            q_norm = float(np.linalg.norm([q.x, q.y, q.z, q.w]))
            if (not all(np.isfinite(v) for v in transform_values)
                    or q_norm < 1e-9):
                self.get_logger().warn(
                    'TF contains non-finite/invalid values; skipping.',
                    throttle_duration_sec=2.0,
                )
                return
            qx, qy, qz, qw = (
                q.x / q_norm, q.y / q_norm,
                q.z / q_norm, q.w / q_norm,
            )
            R = rotation_matrix_2d_from_quaternion(qx, qy, qz, qw)
            points = transform_points(points, [t.x, t.y], R)

        # ②.5 경계 필터 (이제 target_frame 기준이므로 정적 폴리곤을 그대로 써도 됨)
        if self.boundary_polygon is not None:
            points = filter_points_in_polygon(points, self.boundary_polygon)

        # ②.6 맵 벽 포인트 제거 — 반드시 클러스터링 '전'이어야 한다.
        # 클러스터가 만들어진 뒤에 centroid로 판정하면(브릿지의 map filter)
        # 벽 전체가 병합된 클러스터의 centroid가 트랙 한복판이라 통과한다.
        if self.use_map_filter:
            if self.wall_dist is None:
                # 여기 도달 = 맵이 '영구 불가'(scipy 부재/프레임 불일치/malformed).
                # 아직 기다리는 중이면 위에서 이미 발행을 보류했다.
                self.get_logger().warn(
                    'use_map_filter=true지만 벽 필터를 쓸 수 없음 — 필터 없이 진행 '
                    '(벽 파편이 유령 장애물로 나갈 수 있음)',
                    throttle_duration_sec=5.0,
                )
            else:
                points = self.reject_wall_points(points)

        # ② 클러스터링
        clusters = cluster_points(
            points, eps=self.cluster_eps, min_cluster_size=self.min_cluster_size
        )

        # ②.7 물리적으로 불가능한 크기의 클러스터 거부 (맵 필터 2차 방어선).
        # 트래킹 '전'에 버려야 한다 — 트래커에 들어가면 재식별/코스팅으로
        # 살아남아 계속 발행된다.
        if self.max_cluster_radius > 0.0:
            kept = [c for c in clusters if c.radius <= self.max_cluster_radius]
            if len(kept) != len(clusters):
                self.get_logger().warn(
                    f'{len(clusters) - len(kept)}개 초대형 클러스터 거부 '
                    f'(max_radius={self.max_cluster_radius:.2f}m, '
                    f'최대 {max(c.radius for c in clusters):.2f}m) — 벽/구조물로 판단',
                    throttle_duration_sec=2.0,
                )
            clusters = kept

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
                static_pos_std=self.static_pos_std,
                dynamic_pos_std=self.dynamic_pos_std,
                dynamic_release=self.dynamic_release,
            )

            # 관측이 부족한(확정 전) 트랙은 노이즈 클러스터일 수 있어 발행하지 않는다.
            # 단, 한 번 동적 확정된 상대는 occlusion 코스팅 중에도 계속 발행해
            # 다운스트림(GP/회피)이 상대를 놓치지 않게 한다.
            if track.hits < self.min_hits and not track.ever_dynamic:
                continue

            x, y = track.kf.position
            vx, vy = track.kf.velocity

            # 코스팅(미관측 외삽) 중이면 신뢰도를 감쇠 — 다운스트림이 측정값과
            # 외삽값을 구분할 수 있게 한다 (misses가 timeout에 가까울수록 0).
            coast = 1.0
            if track.misses > 0:
                coast = max(0.0, 1.0 - track.misses / float(self.tracker.track_timeout))

            obs = Obstacle()
            obs.id = track.id
            obs.position = Point(x=float(x), y=float(y), z=0.0)
            obs.velocity = Vector3(x=float(vx), y=float(vy), z=0.0)
            obs.is_dynamic = bool(track.is_dynamic)
            obs.confidence = float(track.confidence * coast)
            obs.radius = float(track.radius)

            obstacle_array.obstacles.append(obs)

        self.pub.publish(obstacle_array)


def main(args=None):
    rclpy.init(args=args)
    node = ObstacleDetectorNode()
    # rclpy.spin(node)을 쓰면 안 된다: TransformListener(spin_thread=True)가
    # 만든 전용 executor에서 노드를 빼앗아 TF 콜백이 on_scan과 같은 단일
    # 스레드에 직렬화되고, /tf 큐(depth 100)가 포화되며 버퍼가 ~1.6초 늙는다.
    # 노드 생성자에서 spin_thread=False로 두고 여기서 명시적으로
    # MultiThreadedExecutor를 돌린다 — TF 콜백(ReentrantCallbackGroup)만
    # 병렬로 뜨고 on_scan은 기본 MutuallyExclusive 그룹이라 직렬 유지된다.
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
