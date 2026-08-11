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
from threading import Lock
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

from sensor_msgs.msg import Imu, LaserScan
from nav_msgs.msg import OccupancyGrid, Odometry
from geometry_msgs.msg import Point, Vector3
from std_msgs.msg import Header

try:
    from scipy.ndimage import distance_transform_edt
except ImportError:      # scipy 없는 환경(다른 개발 PC 등)은 맵 필터 없이 폴백
    distance_transform_edt = None

from f1tenth_perception_msgs.msg import Obstacle, ObstacleArray
from f1tenth_scan_geometry import AttitudeSample
from f1tenth_scan_geometry import GroundFilterConfig
from f1tenth_scan_geometry import LevelReference
from f1tenth_scan_geometry import LOOKUP_OK
from f1tenth_scan_geometry import PoseSample
from f1tenth_scan_geometry import TimestampedAttitudeBuffer
from f1tenth_scan_geometry import TimestampedPoseBuffer
from f1tenth_scan_geometry import base_orientation_from_imu
from f1tenth_scan_geometry import deskew_points_to_reference
from f1tenth_scan_geometry import mask_ground_and_offplane
from f1tenth_scan_geometry import quaternion_from_rpy
from f1tenth_scan_geometry import quaternion_to_matrix
from f1tenth_scan_geometry.geometry import rotations_from_yaw_and_level_tilt

try:
    # 트랙 경계 필터 전용. planning_ws가 소스되지 않은 개발 환경에서도
    # 나머지 파이프라인은 그대로 돌아야 하므로 임포트를 강제하지 않는다.
    from f1tenth_planning_msgs.msg import WpntArray
except ImportError:
    WpntArray = None

from perception_pkg.preprocessing import (
    preprocess_scan,
    polygon_mask,
    sanitize_scan_ranges,
)
from perception_pkg.clustering import adaptive_cluster_points
from perception_pkg.clustering import adaptive_component_provenance
from perception_pkg.tracker import Tracker
from perception_pkg.classifier import classify_track
from perception_pkg.track_filter import TrackBounds


def partition_clusters_by_postfilter_radius(clusters, max_radius):
    """Split clusters using only their geometry after map/track filtering.

    ``source_radius`` deliberately describes the raw connected component.  A
    real obstacle touching a wall in the raw scan can therefore have a large
    source radius even after the wall points have correctly been removed.  It
    remains attached to the cluster for diagnostics and for a future,
    separately tuned wall-connectivity rule, but it must not tighten this
    physical-size gate.
    """
    clusters = list(clusters)
    if max_radius <= 0.0:
        return clusters, []
    kept = [cluster for cluster in clusters
            if cluster.radius <= max_radius]
    rejected = [cluster for cluster in clusters
                if cluster.radius > max_radius]
    return kept, rejected


class ObstacleDetectorNode(Node):

    def __init__(self):
        super().__init__('obstacle_detector_node')

        # ---- 파라미터 선언 (config/obstacle_params.yaml에서 오버라이드) ----
        self.declare_parameter('range_min', 0.05)
        self.declare_parameter('range_max', 10.0)
        self.declare_parameter('fov_min_deg', -135.0)
        self.declare_parameter('fov_max_deg', 135.0)
        self.declare_parameter('median_kernel', 3)

        # adaptive breakpoint 클러스터링: 임계값이 센서 거리에 따라 커지므로
        # 구 cluster_eps(고정 거리)는 필요 없다.
        self.declare_parameter('lambda_angle_deg', 10.0)
        self.declare_parameter('sigma', 0.015)
        self.declare_parameter('min_cluster_size', 3)
        # 벽에서 떨어진(열린 공간) 클러스터에만 적용하는 완화된 최소 크기.
        # min_cluster_size를 8로 올린 이유는 벽 fringe(맵 필터를 통과한 벽
        # 리턴 몇 점) 억제였는데, 그 대가로 벽에서 먼 작은 물체까지 함께
        # 잘린다 — run_0806_155621에서 폭 0.2m 물체(스캔 4점)가 15프레임
        # 미검출되어 하류에 0.48~0.63m짜리 '가짜 틈'이 생겼다. 실제로 막힌
        # 통로를 열렸다고 판단하면 그 틈으로 회피를 시도하므로 위험하다.
        # 벽에서 open_space_dist_m 이상 떨어진 클러스터는 벽 fringe일 수
        # 없으므로 이 값까지 허용한다. 0이면 비활성.
        self.declare_parameter('min_cluster_size_open', 0)
        self.declare_parameter('open_space_dist_m', 0.50)

        self.declare_parameter('gating_distance', 0.5)
        # 크기 일관성 게이트: 트랙이 보던 반경 대비 후보 클러스터 반경의
        # 비가 이 배수를 넘으면 매칭하지 않는다. 0/1 이하면 비활성.
        # 근거는 tracker.py의 해당 게이트 주석 참조.
        self.declare_parameter('radius_ratio_max', 4.0)
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
        self.declare_parameter('odom_topic', '/odom')
        self.declare_parameter('imu_topic', '/sensors/imu/raw')
        self.declare_parameter('obstacle_topic', '/perception/obstacles')
        self.declare_parameter('target_frame', 'map')   # frame_id_override 대체
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('tf_timeout_sec', 0.05)
        self.declare_parameter('require_source_stamp', True)
        self.declare_parameter('max_source_age_sec', 0.20)
        self.declare_parameter('source_future_tolerance_sec', 0.05)
        # 고속주행에서는 latest TF fallback이 위치를 체계적으로 틀리게 하므로 기본 금지.
        # 개발용으로 명시 opt-in해도 원본 스캔 시각과 충분히 가까운 TF만 허용한다.
        self.declare_parameter('tf_allow_latest_fallback', False)
        self.declare_parameter('tf_max_age_sec', 0.05)
        # time-travel lookup의 경유(fixed) 프레임. ''(기본) = 비활성.
        # 'odom'으로 주면 laser->odom은 scan 정시각, odom->map은 latest로
        # 합성해, latest 폴백의 주행 중 오차(속도 x TF지연)를 PF 보정
        # 드리프트 수준으로 줄인다. odom 프레임이 없는 환경(sim)은 비워 둘 것.
        self.declare_parameter('tf_fixed_frame', '')
        # Timestamped 3D scan geometry. Buffers stay small and callbacks only
        # append; interpolation is vectorized once per accepted scan.
        self.declare_parameter('odom_buffer_size', 200)
        self.declare_parameter('imu_buffer_size', 200)
        self.declare_parameter('sensor_sync_wait_sec', 0.005)
        self.declare_parameter('max_odom_interp_gap_sec', 0.08)
        self.declare_parameter('max_imu_interp_gap_sec', 0.08)
        self.declare_parameter('deskew_enabled', True)
        self.declare_parameter('deskew_max_scan_duration_sec', 0.05)
        self.declare_parameter('deskew_translation_scale', 1.0)
        self.declare_parameter('imu_angular_velocity_scale', 0.017453292519943295)
        self.declare_parameter('imu_linear_acceleration_scale', 9.80665)
        # Used only when the IMU frame TF is absent. The real-car bringup now
        # publishes base_link -> vesc_imu, so measured TF normally wins.
        self.declare_parameter('base_to_imu_rpy_deg', [0.0, 0.0, 0.0])
        self.declare_parameter('level_reference_samples', 20)
        self.declare_parameter('level_reference_max_gyro_rad_s', 0.12)
        self.declare_parameter('level_reference_accel_tolerance_mps2', 2.5)
        self.declare_parameter('ground_filter_enabled', False)
        self.declare_parameter('ground_z_m', 0.0)
        self.declare_parameter('ground_range_tolerance_m', 0.08)
        self.declare_parameter('ground_range_tolerance_ratio', 0.04)
        self.declare_parameter('ground_endpoint_z_tolerance_m', 0.05)
        self.declare_parameter('ground_min_component_beams', 6)
        self.declare_parameter('ground_min_component_angle_deg', 4.0)
        self.declare_parameter('ground_min_component_span_m', 0.30)
        self.declare_parameter('max_planar_elevation_deg', 4.0)
        self.declare_parameter('degraded_offplane_fraction', 0.55)
        self.declare_parameter('degraded_ground_fraction', 0.70)
        self.declare_parameter('attitude_fallback_tilt_deg', 4.0)
        self.declare_parameter('attitude_ground_evidence_fraction', 0.05)
        self.declare_parameter('attitude_ground_evidence_min_beams', 6)
        self.declare_parameter('attitude_evidence_max_abs_angle_deg', 80.0)
        self.declare_parameter('degraded_valid_fraction', 0.25)
        self.declare_parameter('ground_min_retained_beams', 20)
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
        # ---- 트랙 경계(Frenet) 필터 ----
        # ForzaETH race_stack의 Detect::laserPointOnTrack() 이식.
        # 맵 벽거리(EDT)와 직교하는 두 번째 기준이다 — 근거는 track_filter.py 참고.
        self.declare_parameter('use_track_filter', False)
        self.declare_parameter('global_waypoints_topic', '/global_waypoints')
        # 트랙 경계를 안쪽으로 줄이는 여유 [m] (ForzaETH boundaries_inflation).
        # 경계에 바짝 붙은 벽 리턴을 확실히 빼되, 벽에 붙은 상대차의 트랙쪽
        # 면 포인트는 남겨야 한다.
        self.declare_parameter('track_bound_inflation_m', 0.10)
        # 자차 기준 전방 s 시야창 [m] (ForzaETH max_viewing_distance).
        # 실효 인지거리(run_0805 실측 ~4.5m)보다 약간 크게 잡는다. 이보다 먼
        # 검출은 어차피 신뢰할 수 없는데 유령은 대부분 거기서 나온다.
        self.declare_parameter('max_view_ahead_m', 6.0)
        # 후방 s 시야창 [m]. 지나친 장애물의 트랙을 곧바로 끊지 않을 만큼만.
        self.declare_parameter('max_view_behind_m', 2.0)
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

        self.lambda_angle = float(p('lambda_angle_deg').value) * np.pi / 180.0
        self.sigma = float(p('sigma').value)
        self.min_cluster_size = int(p('min_cluster_size').value)
        self.min_cluster_size_open = int(p('min_cluster_size_open').value)
        self.open_space_dist = float(p('open_space_dist_m').value)

        self.v_thresh = float(p('v_thresh').value)
        self.n_confirm = int(p('n_confirm').value)
        self.n_release = int(p('n_release').value)
        self.static_pos_std = float(p('static_pos_std').value)
        self.dynamic_pos_std = float(p('dynamic_pos_std').value)
        self.dynamic_release = int(p('dynamic_release').value)
        self.min_hits = int(p('min_hits').value)
        gating_distance = float(p('gating_distance').value)
        radius_ratio_max = float(p('radius_ratio_max').value)
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
        self.tf_fixed_frame = str(p('tf_fixed_frame').value).strip()
        self.base_frame = str(p('base_frame').value).strip()
        self.odom_buffer_size = int(p('odom_buffer_size').value)
        self.imu_buffer_size = int(p('imu_buffer_size').value)
        self.sensor_sync_wait = float(p('sensor_sync_wait_sec').value)
        self.max_odom_interp_gap = float(
            p('max_odom_interp_gap_sec').value)
        self.max_imu_interp_gap = float(
            p('max_imu_interp_gap_sec').value)
        self.deskew_enabled = bool(p('deskew_enabled').value)
        self.deskew_max_scan_duration = float(
            p('deskew_max_scan_duration_sec').value)
        self.deskew_translation_scale = float(
            p('deskew_translation_scale').value)
        self.imu_angular_velocity_scale = float(
            p('imu_angular_velocity_scale').value)
        self.imu_linear_acceleration_scale = float(
            p('imu_linear_acceleration_scale').value)
        self.ground_filter_enabled = bool(
            p('ground_filter_enabled').value)
        base_to_imu_rpy = np.asarray(
            p('base_to_imu_rpy_deg').value, dtype=np.float64)
        if base_to_imu_rpy.shape != (3,) or not np.all(np.isfinite(base_to_imu_rpy)):
            raise ValueError('base_to_imu_rpy_deg must contain three finite values')
        self.base_to_imu_fallback = quaternion_from_rpy(
            *np.deg2rad(base_to_imu_rpy))
        level_samples = int(p('level_reference_samples').value)
        level_max_gyro = float(
            p('level_reference_max_gyro_rad_s').value)
        level_accel_tolerance = float(
            p('level_reference_accel_tolerance_mps2').value)
        self.ground_config = GroundFilterConfig(
            ground_z_m=float(p('ground_z_m').value),
            range_tolerance_m=float(p('ground_range_tolerance_m').value),
            range_tolerance_ratio=float(
                p('ground_range_tolerance_ratio').value),
            endpoint_z_tolerance_m=float(
                p('ground_endpoint_z_tolerance_m').value),
            min_component_beams=int(p('ground_min_component_beams').value),
            min_component_angle_rad=np.deg2rad(float(
                p('ground_min_component_angle_deg').value)),
            min_component_span_m=float(
                p('ground_min_component_span_m').value),
            max_planar_elevation_rad=np.deg2rad(float(
                p('max_planar_elevation_deg').value)),
            degraded_offplane_fraction=float(
                p('degraded_offplane_fraction').value),
            degraded_ground_fraction=float(
                p('degraded_ground_fraction').value),
            attitude_fallback_tilt_rad=np.deg2rad(float(
                p('attitude_fallback_tilt_deg').value)),
            attitude_ground_evidence_fraction=float(
                p('attitude_ground_evidence_fraction').value),
            attitude_ground_evidence_min_beams=int(
                p('attitude_ground_evidence_min_beams').value),
            attitude_evidence_max_abs_angle_rad=np.deg2rad(float(
                p('attitude_evidence_max_abs_angle_deg').value)),
            degraded_valid_fraction=float(
                p('degraded_valid_fraction').value),
            min_retained_beams=int(p('ground_min_retained_beams').value),
        )
        self.use_map_filter = bool(p('use_map_filter').value)
        self.wall_dist_min = float(p('wall_dist_min_m').value)
        self.max_cluster_radius = float(p('max_cluster_radius_m').value)
        self.map_wait_timeout = float(p('map_wait_timeout_sec').value)
        self.use_track_filter = bool(p('use_track_filter').value)
        self.track_bound_inflation = float(p('track_bound_inflation_m').value)
        self.max_view_ahead = float(p('max_view_ahead_m').value)
        self.max_view_behind = float(p('max_view_behind_m').value)
        if (not np.isfinite(self.track_bound_inflation)
                or not 0.0 <= self.track_bound_inflation <= 1.0
                or not np.isfinite(self.max_view_ahead)
                or not 0.5 <= self.max_view_ahead <= 1000.0
                or not np.isfinite(self.max_view_behind)
                or not 0.0 <= self.max_view_behind <= 1000.0):
            raise ValueError(
                'invalid track_bound_inflation_m/max_view_ahead_m/max_view_behind_m')
        if (not np.isfinite(self.wall_dist_min)
                or not 0.0 <= self.wall_dist_min <= 2.0
                or not np.isfinite(self.max_cluster_radius)
                or not 0.0 <= self.max_cluster_radius <= 100.0
                or not np.isfinite(self.map_wait_timeout)
                or not 0.0 <= self.map_wait_timeout <= 120.0):
            raise ValueError(
                'invalid wall_dist_min_m/max_cluster_radius_m/map_wait_timeout_sec')
        timing_values = (
            rate, tf_timeout_sec, self.max_source_age,
            self.source_future_tolerance, self.tf_max_age,
            self.sensor_sync_wait, self.max_odom_interp_gap,
            self.max_imu_interp_gap, self.deskew_max_scan_duration,
        )
        if (not all(np.isfinite(v) for v in timing_values)
                or not 1.0 <= rate <= 500.0
                or not 0.0 <= tf_timeout_sec <= 5.0
                or not 0.01 <= self.max_source_age <= 5.0
                or not 0.0 <= self.source_future_tolerance <= 1.0
                or not 0.0 <= self.tf_max_age <= 1.0
                or not 0.0 <= self.sensor_sync_wait <= 0.1
                or not 0.001 <= self.max_odom_interp_gap <= 1.0
                or not 0.001 <= self.max_imu_interp_gap <= 1.0
                or not 0.001 <= self.deskew_max_scan_duration <= 0.2):
            raise ValueError('invalid detector timing/freshness parameters')
        ground_values = (
            self.ground_config.ground_z_m,
            self.ground_config.range_tolerance_m,
            self.ground_config.range_tolerance_ratio,
            self.ground_config.endpoint_z_tolerance_m,
            self.ground_config.min_component_angle_rad,
            self.ground_config.min_component_span_m,
            self.ground_config.max_planar_elevation_rad,
            self.ground_config.degraded_offplane_fraction,
            self.ground_config.degraded_ground_fraction,
            self.ground_config.attitude_fallback_tilt_rad,
            self.ground_config.attitude_ground_evidence_fraction,
            self.ground_config.attitude_evidence_max_abs_angle_rad,
            self.ground_config.degraded_valid_fraction,
            level_max_gyro, level_accel_tolerance,
            self.imu_angular_velocity_scale,
            self.imu_linear_acceleration_scale,
            self.deskew_translation_scale,
        )
        if (not all(np.isfinite(v) for v in ground_values)
                or not 2 <= self.odom_buffer_size <= 2000
                or not 2 <= self.imu_buffer_size <= 2000
                or not 2 <= level_samples <= 1000
                or not 0.0 < level_max_gyro <= 5.0
                or not 0.0 < level_accel_tolerance <= 20.0
                or not -1.0 <= self.ground_config.ground_z_m <= 1.0
                or not 0.0 < self.ground_config.range_tolerance_m <= 1.0
                or not 0.0 <= self.ground_config.range_tolerance_ratio <= 0.5
                or not 0.0 < self.ground_config.endpoint_z_tolerance_m <= 0.5
                or not 2 <= self.ground_config.min_component_beams <= 1000
                or not 0.0 < self.ground_config.min_component_angle_rad <= np.pi
                or not 0.05 <= self.ground_config.min_component_span_m <= 5.0
                or not np.deg2rad(0.5) <= self.ground_config.max_planar_elevation_rad <= np.deg2rad(30.0)
                or not 0.0 < self.ground_config.degraded_offplane_fraction <= 1.0
                or not 0.0 < self.ground_config.degraded_ground_fraction <= 1.0
                or not np.deg2rad(1.0)
                <= self.ground_config.attitude_fallback_tilt_rad
                <= np.deg2rad(45.0)
                or not 0.0
                <= self.ground_config.attitude_ground_evidence_fraction <= 1.0
                or not 1
                <= self.ground_config.attitude_ground_evidence_min_beams <= 1000
                or not np.deg2rad(10.0)
                <= self.ground_config.attitude_evidence_max_abs_angle_rad
                <= np.pi
                or not 0.0 < self.ground_config.degraded_valid_fraction <= 1.0
                or not 1 <= self.ground_config.min_retained_beams <= 10000
                or not 0.0 <= self.deskew_translation_scale <= 1.0
                or self.imu_angular_velocity_scale <= 0.0
                or self.imu_linear_acceleration_scale <= 0.0):
            raise ValueError('invalid detector deskew/ground-filter parameters')
        continuous_values = (
            self.range_min, self.range_max, self.fov_min, self.fov_max,
            self.lambda_angle, self.sigma, self.v_thresh, self.static_pos_std,
            self.dynamic_pos_std, gating_distance, radius_ratio_max,
            reid_distance, reid_timeout,
        )
        if (not all(np.isfinite(v) for v in continuous_values)
                or not 0.0 <= self.range_min < self.range_max <= 100.0
                or not -2.0 * np.pi <= self.fov_min < self.fov_max <= 2.0 * np.pi
                or self.median_kernel < 1 or self.median_kernel % 2 == 0
                # lambda는 angle_increment보다 확실히 커야 한다. 작거나 같으면
                # d_max가 음수가 되어 모든 포인트가 쪼개지고 장애물이 통째로
                # 사라진다. 하한 1deg는 실사용 라이다 증분(UST-10LX 0.25deg)의
                # 4배 여유. 상한 45deg 이상은 사실상 무한 병합이라 의미가 없다.
                or not np.deg2rad(1.0) <= self.lambda_angle <= np.deg2rad(45.0)
                or not 0.0 <= self.sigma <= 1.0
                or not 1 <= self.min_cluster_size <= 100000
                or not 0 <= self.min_cluster_size_open <= self.min_cluster_size
                or not 0.0 <= self.open_space_dist <= 10.0
                or not 0.001 <= gating_distance <= 100.0
                or not 0.0 <= radius_ratio_max <= 1000.0
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
        if not self.target_frame or not self.base_frame:
            raise ValueError('target_frame and base_frame must not be empty')
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
            radius_ratio_max=radius_ratio_max,
        )

        scan_topic = str(p('scan_topic').value).strip()
        odom_topic = str(p('odom_topic').value).strip()
        imu_topic = str(p('imu_topic').value).strip()
        obstacle_topic = str(p('obstacle_topic').value).strip()
        if not scan_topic or not odom_topic or not imu_topic or not obstacle_topic:
            raise ValueError('scan/odom/imu/obstacle topics must not be empty')

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

        self.odom_buffer = TimestampedPoseBuffer(self.odom_buffer_size)
        self.attitude_buffer = TimestampedAttitudeBuffer(self.imu_buffer_size)
        self.sensor_state_lock = Lock()
        self.current_wheel_speed = 0.0
        self.base_to_imu_quaternion = np.copy(self.base_to_imu_fallback)
        self.imu_extrinsic_frame = ''
        self.level_reference = LevelReference(
            min_samples=level_samples,
            max_gyro_rad_s=level_max_gyro,
            accel_tolerance_mps2=level_accel_tolerance,
        )
        self.laser_extrinsics = {}
        self.ground_filtered_beams = 0
        self.offplane_filtered_beams = 0
        self.geometry_degraded_scans = 0
        self.geometry_sync_misses = 0
        self.attitude_fallback_scans = 0
        self.deskew_applied_scans = 0
        self.last_geometry_log_time = -1e18

        sensor_cb_group = ReentrantCallbackGroup()
        self.sub = self.create_subscription(
            LaserScan, scan_topic, self.on_scan, qos_profile_sensor_data
        )
        self.odom_sub = self.create_subscription(
            Odometry, odom_topic, self.on_odom, qos_profile_sensor_data,
            callback_group=sensor_cb_group)
        self.imu_sub = self.create_subscription(
            Imu, imu_topic, self.on_imu, qos_profile_sensor_data,
            callback_group=sensor_cb_group)
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

        # ---- 트랙 경계(Frenet) 필터 구독 ----
        # 맵 필터와 달리 발행을 보류하지 않는다: 이건 벽거리 필터 위에 얹는
        # 2차 기준이라 없어도 파이프라인이 성립하고, 웨이포인트는 전역 플래너가
        # 뜬 뒤에야 나오므로 보류하면 기동 순서에 묶인다.
        self.track_bounds = None
        if self.use_track_filter:
            wpnt_topic = str(p('global_waypoints_topic').value).strip()
            if not wpnt_topic:
                raise ValueError(
                    'global_waypoints_topic must not be empty when use_track_filter')
            if WpntArray is None:
                self.get_logger().error(
                    'use_track_filter=true지만 f1tenth_planning_msgs를 임포트할 수 '
                    '없음 — 트랙 필터 없이 동작한다.')
            else:
                # 전역 플래너가 타이머로 주기 재발행(~2Hz)하므로 latch가 필요 없다
                # (브릿지도 같은 QoS로 받고 있다).
                wpnt_qos = QoSProfile(
                    depth=1, reliability=ReliabilityPolicy.RELIABLE)
                # 맵과 같은 이유로 전용 Reentrant 그룹 — 기본 그룹에 두면
                # 스캔 콜백이 큐를 채우는 동안 순번을 못 받는다.
                self.wpnt_cb_group = ReentrantCallbackGroup()
                self.create_subscription(
                    WpntArray, wpnt_topic, self.cb_global_waypoints, wpnt_qos,
                    callback_group=self.wpnt_cb_group)

        self.get_logger().info(
            f'obstacle_detector_node started: {scan_topic} -> {obstacle_topic} '
            f'(target_frame={self.target_frame}, odom={odom_topic}, imu={imu_topic})'
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

    def cb_global_waypoints(self, msg):
        """전역 레이스라인 수신 -> Frenet 투영/트랙 폭 테이블 갱신.

        전역 플래너는 주기적으로 재발행하지만 내용은 거의 항상 같으므로
        웨이포인트 개수가 바뀔 때만 다시 만든다 (매번 만들면 2Hz로 297점
        배열을 재할당하게 된다). 섹터 재계산으로 개수가 바뀌면 따라간다.
        """
        wpnts = msg.wpnts
        if self.track_bounds is not None and self.track_bounds.n == len(wpnts):
            return
        frame = msg.header.frame_id.strip().lstrip('/')
        if frame and frame != self.target_frame.lstrip('/'):
            # 좌표계가 다르면 투영이 통째로 틀린다 — 켜지 않는 게 낫다.
            self.get_logger().error(
                f'웨이포인트 프레임({frame})이 target_frame({self.target_frame})과 '
                '다름 — 트랙 필터 비활성',
                throttle_duration_sec=10.0)
            return
        try:
            bounds = TrackBounds(wpnts)
        except ValueError as ex:
            self.get_logger().error(
                f'전역 웨이포인트가 트랙 필터에 부적합함 ({ex}) — 트랙 필터 비활성',
                throttle_duration_sec=10.0)
            return
        first = self.track_bounds is None
        self.track_bounds = bounds
        if first:
            self.get_logger().info(
                f'트랙 경계 필터 활성: {bounds.n}점, 트랙길이 {bounds.length:.1f}m, '
                f'inflation={self.track_bound_inflation:.2f}m, '
                f'view={-self.max_view_behind:.1f}~+{self.max_view_ahead:.1f}m')

    def wall_point_mask(self, points):
        """맵 벽에서 wall_dist_min 이상 떨어진 포인트의 bool 마스크 (target_frame 좌표).

        **마스크를 돌려주는 이유**: 호출부는 points와 point_ranges를 1:1로
        유지해야 한다. 예전처럼 points만 잘라 돌려주면 클러스터링이
        `len(ranges) != len(points)` ValueError로 죽고, MultiThreadedExecutor에서
        콜백 예외는 spin() 밖으로 전파되므로 노드가 그대로 종료된다
        (-> /perception/obstacles 끊김 -> spliner fail-safe 정지).

        scipy 부재/맵 영구 불가면 전부 통과(fail-open)시킨다. 맵을 '아직'
        기다리는 동안은 on_scan이 발행 자체를 보류하므로(awaiting_map) 여기로
        오지 않는다 — fail-open 상태의 벽 파편 발행이 run_0803_213313에서
        TRAILING 정지 라치를 유발했기 때문.
        """
        if self.wall_dist is None or len(points) == 0:
            return np.ones(len(points), dtype=bool)
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
        return keep

    def wall_distance_at(self, xy):
        """맵 벽까지의 거리 [m] (target_frame 좌표). 맵/EDT가 없으면 None.

        wall_point_mask와 같은 인덱스 변환을 쓰되 값 자체를 돌려준다 —
        클러스터가 '열린 공간'에 있는지(= 벽 fringe일 수 없는지) 판정용.
        """
        if self.wall_dist is None or self.map_res <= 0.0:
            return None
        dx = float(xy[0]) - self.map_origin[0]
        dy = float(xy[1]) - self.map_origin[1]
        c_yaw, s_yaw = np.cos(self.map_origin_yaw), np.sin(self.map_origin_yaw)
        col = int(np.floor((c_yaw * dx + s_yaw * dy) / self.map_res))
        row = int(np.floor((-s_yaw * dx + c_yaw * dy) / self.map_res))
        h, w = self.wall_dist.shape
        if not (0 <= row < h and 0 <= col < w):
            return None            # 맵 밖 — 열린 공간으로 취급하지 않는다
        return float(self.wall_dist[row, col])

    @staticmethod
    def _stamp_to_ns(stamp):
        return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)

    @staticmethod
    def _normalized_quaternion(message_quaternion):
        quaternion = np.array([
            message_quaternion.x, message_quaternion.y,
            message_quaternion.z, message_quaternion.w,
        ], dtype=np.float64)
        norm = float(np.linalg.norm(quaternion))
        if not np.isfinite(norm) or norm < 1e-9:
            return None
        return quaternion / norm

    def on_odom(self, msg: Odometry):
        """Append a lightweight odom sample; no scan work runs here."""
        stamp_ns = self._stamp_to_ns(msg.header.stamp)
        if stamp_ns <= 0:
            stamp_ns = self.get_clock().now().nanoseconds
        q = self._normalized_quaternion(msg.pose.pose.orientation)
        if q is None:
            return
        yaw = float(np.arctan2(
            2.0 * (q[3] * q[2] + q[0] * q[1]),
            1.0 - 2.0 * (q[1] * q[1] + q[2] * q[2])))
        speed = float(msg.twist.twist.linear.x)
        values = (msg.pose.pose.position.x, msg.pose.pose.position.y,
                  yaw, speed)
        if not np.all(np.isfinite(values)):
            return
        self.odom_buffer.append(PoseSample(
            stamp_ns, float(values[0]), float(values[1]), yaw, speed))
        with self.sensor_state_lock:
            self.current_wheel_speed = speed

    def _resolve_imu_extrinsic(self, frame_id):
        """Resolve base<-IMU rotation once, retaining a parameter fallback."""
        frame = frame_id.strip().lstrip('/')
        if not frame:
            return np.copy(self.base_to_imu_fallback)
        with self.sensor_state_lock:
            if self.imu_extrinsic_frame == frame:
                return np.copy(self.base_to_imu_quaternion)
        try:
            transform = self.tf_buffer.lookup_transform(
                self.base_frame, frame, Time(), timeout=Duration(seconds=0.0))
        except (LookupException, ConnectivityException, ExtrapolationException):
            self.get_logger().warn(
                f'IMU TF {self.base_frame} <- {frame} unavailable; using '
                'base_to_imu_rpy_deg fallback until TF arrives.',
                throttle_duration_sec=5.0)
            return np.copy(self.base_to_imu_fallback)
        q = self._normalized_quaternion(transform.transform.rotation)
        if q is None:
            return np.copy(self.base_to_imu_fallback)
        with self.sensor_state_lock:
            changed = (self.imu_extrinsic_frame != frame
                       or not np.allclose(q, self.base_to_imu_quaternion,
                                          atol=1e-8))
            self.base_to_imu_quaternion = q
            self.imu_extrinsic_frame = frame
            if changed:
                self.attitude_buffer.clear()
                self.level_reference.reset()
                self.get_logger().info(
                    f'Using measured IMU extrinsic {self.base_frame} <- {frame}; '
                    'level reference reset')
        return np.copy(q)

    def on_imu(self, msg: Imu):
        """Convert one timestamped IMU orientation to the base frame."""
        stamp_ns = self._stamp_to_ns(msg.header.stamp)
        if stamp_ns <= 0:
            stamp_ns = self.get_clock().now().nanoseconds
        imu_q = self._normalized_quaternion(msg.orientation)
        values = np.array([
            msg.angular_velocity.x, msg.angular_velocity.y,
            msg.angular_velocity.z, msg.linear_acceleration.x,
            msg.linear_acceleration.y, msg.linear_acceleration.z,
        ], dtype=np.float64)
        if imu_q is None or not np.all(np.isfinite(values)):
            return
        base_to_imu = self._resolve_imu_extrinsic(msg.header.frame_id)
        try:
            base_q = base_orientation_from_imu(imu_q, base_to_imu)
        except ValueError:
            return
        angular_speed = float(
            np.linalg.norm(values[:3]) * self.imu_angular_velocity_scale)
        acceleration_norm = float(
            np.linalg.norm(values[3:]) * self.imu_linear_acceleration_scale)
        if not self.attitude_buffer.append(AttitudeSample(
                stamp_ns, tuple(base_q), angular_speed, acceleration_norm)):
            return
        with self.sensor_state_lock:
            was_ready = self.level_reference.ready
            self.level_reference.update(
                base_q, angular_speed, acceleration_norm,
                wheel_speed=self.current_wheel_speed)
            if self.level_reference.ready and not was_ready:
                self.get_logger().info(
                    'IMU level reference ready; 3D ground filtering enabled')

    def _laser_extrinsic(self, source_frame):
        """Return measured base<-LiDAR translation and full quaternion."""
        frame = source_frame.strip().lstrip('/')
        if frame == self.base_frame.lstrip('/'):
            return np.zeros(3), quaternion_from_rpy(0.0, 0.0, 0.0)
        if frame in self.laser_extrinsics:
            translation, quaternion = self.laser_extrinsics[frame]
            return np.copy(translation), np.copy(quaternion)
        try:
            transform = self.tf_buffer.lookup_transform(
                self.base_frame, frame, Time(), timeout=Duration(seconds=0.0))
        except (LookupException, ConnectivityException, ExtrapolationException):
            self.get_logger().warn(
                f'LiDAR extrinsic TF {self.base_frame} <- {frame} unavailable; '
                'skipping scan.', throttle_duration_sec=2.0)
            return None
        q = self._normalized_quaternion(transform.transform.rotation)
        t = transform.transform.translation
        translation = np.array([t.x, t.y, t.z], dtype=np.float64)
        if q is None or not np.all(np.isfinite(translation)):
            return None
        self.laser_extrinsics[frame] = (translation, q)
        self.get_logger().info(
            f'Using 6DoF LiDAR extrinsic {self.base_frame} <- {frame}: '
            f'xyz=({t.x:.3f},{t.y:.3f},{t.z:.3f})')
        return np.copy(translation), np.copy(q)

    def _lookup_target_transform(self, source_frame, source_time, stamp_sec):
        """Lookup target<-source at scan time with the existing bounded fallbacks."""
        if self.target_frame.lstrip('/') == source_frame.lstrip('/'):
            return np.zeros(3), quaternion_from_rpy(0.0, 0.0, 0.0)
        try:
            transform = self.tf_buffer.lookup_transform(
                self.target_frame, source_frame, source_time,
                timeout=self.tf_timeout)
        except (LookupException, ConnectivityException,
                ExtrapolationException) as exact_ex:
            transform = None
            if self.tf_fixed_frame:
                try:
                    fixed_leg = self.tf_buffer.lookup_transform(
                        self.tf_fixed_frame, source_frame, Time(),
                        timeout=self.tf_timeout)
                    fixed_stamp = (fixed_leg.header.stamp.sec
                                   + fixed_leg.header.stamp.nanosec * 1e-9)
                    if (np.isfinite(fixed_stamp) and fixed_stamp > 0.0
                            and abs(stamp_sec - fixed_stamp) <= self.tf_max_age):
                        clamped_time = (source_time if fixed_stamp >= stamp_sec
                                        else Time.from_msg(fixed_leg.header.stamp))
                        transform = self.tf_buffer.lookup_transform_full(
                            self.target_frame, Time(), source_frame,
                            clamped_time, self.tf_fixed_frame,
                            timeout=self.tf_timeout)
                except (LookupException, ConnectivityException,
                        ExtrapolationException):
                    transform = None
            if transform is None and self.tf_allow_latest_fallback:
                try:
                    transform = self.tf_buffer.lookup_transform(
                        self.target_frame, source_frame, Time(),
                        timeout=self.tf_timeout)
                except (LookupException, ConnectivityException,
                        ExtrapolationException):
                    transform = None
            if transform is None:
                self.get_logger().warn(
                    f'TF lookup {source_frame} -> {self.target_frame} failed: '
                    f'{exact_ex}. Skipping scan.', throttle_duration_sec=2.0)
                return None
            tf_stamp = (transform.header.stamp.sec
                        + transform.header.stamp.nanosec * 1e-9)
            if (not np.isfinite(tf_stamp) or tf_stamp <= 0.0
                    or abs(tf_stamp - stamp_sec) > self.tf_max_age):
                self.get_logger().warn(
                    f'TF is too far from scan midpoint '
                    f'(|tf-scan|={abs(tf_stamp - stamp_sec):.3f}s); skipping.',
                    throttle_duration_sec=2.0)
                return None
        q = self._normalized_quaternion(transform.transform.rotation)
        t = transform.transform.translation
        translation = np.array([t.x, t.y, t.z], dtype=np.float64)
        if q is None or not np.all(np.isfinite(translation)):
            return None
        return translation, q

    def _points_in_target_frame(self, points_laser, target_from_base,
                                reference_base_quaternion,
                                level_quaternion, laser_extrinsic):
        target_translation, target_quaternion = target_from_base
        laser_translation, laser_quaternion = laser_extrinsic
        target_rotation = quaternion_to_matrix(target_quaternion)
        if reference_base_quaternion is not None and level_quaternion is not None:
            yaw = float(np.arctan2(
                target_rotation[1, 0], target_rotation[0, 0]))
            target_rotation = rotations_from_yaw_and_level_tilt(
                np.array([yaw]),
                np.asarray(reference_base_quaternion).reshape(1, 4),
                level_quaternion)[0]
        laser_rotation = quaternion_to_matrix(laser_quaternion)
        points_base = laser_translation + points_laser @ laser_rotation.T
        points_target = target_translation + points_base @ target_rotation.T
        return points_target[:, :2], target_translation[:2]

    def _publish_tracks(self, active_tracks, _source_header, stamp_ns):
        obstacle_array = ObstacleArray()
        obstacle_array.header = Header(
            stamp=Time(nanoseconds=stamp_ns).to_msg(),
            frame_id=self.target_frame)
        for track in active_tracks:
            classify_track(
                track, v_thresh=self.v_thresh,
                n_confirm=self.n_confirm, n_release=self.n_release,
                static_pos_std=self.static_pos_std,
                dynamic_pos_std=self.dynamic_pos_std,
                dynamic_release=self.dynamic_release)
            if track.hits < self.min_hits and not track.ever_dynamic:
                continue
            x, y = track.kf.position
            vx, vy = track.kf.velocity
            coast = 1.0
            if track.misses > 0:
                coast = max(
                    0.0, 1.0 - track.misses / float(self.tracker.track_timeout))
            obs = Obstacle()
            obs.id = track.id
            obs.position = Point(x=float(x), y=float(y), z=0.0)
            obs.velocity = Vector3(x=float(vx), y=float(vy), z=0.0)
            obs.is_dynamic = bool(track.is_dynamic)
            obs.confidence = float(track.confidence * coast)
            obs.radius = float(track.radius)
            obstacle_array.obstacles.append(obs)
        self.pub.publish(obstacle_array)

    def _log_geometry_diagnostics(self, stamp_sec):
        if stamp_sec - self.last_geometry_log_time < 5.0:
            return
        self.last_geometry_log_time = stamp_sec
        self.get_logger().info(
            'scan geometry: ground=%d offplane=%d degraded=%d sync_miss=%d '
            'attitude_fallback=%d deskew=%d level=%s' % (
                self.ground_filtered_beams, self.offplane_filtered_beams,
                self.geometry_degraded_scans, self.geometry_sync_misses,
                self.attitude_fallback_scans, self.deskew_applied_scans,
                'ready' if self.level_reference.ready else 'waiting'))

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
            msg.range_min, msg.range_max, msg.time_increment,
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

        header_stamp_ns = self._stamp_to_ns(msg.header.stamp)
        scan_duration = ((len(ranges) - 1) * float(msg.time_increment)
                         if msg.time_increment > 0.0 else 0.0)
        timing_valid = bool(
            0.0 < scan_duration <= self.deskew_max_scan_duration)
        reference_offset_ns = (int(round(0.5 * scan_duration * 1e9))
                               if timing_valid else 0)
        reference_stamp_ns = header_stamp_ns + reference_offset_ns
        reference_stamp = reference_stamp_ns * 1e-9
        reference_time = Time(nanoseconds=reference_stamp_ns)

        # Keep a full, sanitized FOV for geometry. In particular, no-return
        # beams must remain represented so an upward-looking sky frame can be
        # distinguished from a normal scan before median filtering.
        geometry_all_ranges, geometry_all_angles = sanitize_scan_ranges(
            msg.ranges, msg.angle_min, msg.angle_increment,
            range_min=self.range_min, range_max=self.range_max,
            sensor_range_max=msg.range_max,
        )
        geometry_fov = (
            (geometry_all_angles >= self.fov_min)
            & (geometry_all_angles <= self.fov_max)
        )
        geometry_indices = np.flatnonzero(geometry_fov).astype(np.int64)
        geometry_ranges = geometry_all_ranges[geometry_fov]
        geometry_angles = geometry_all_angles[geometry_fov]

        # Clustering still uses the existing median-filtered points, with each
        # point carrying its original beam index for geometry and provenance.
        points, point_ranges, beam_indices, _point_angles = preprocess_scan(
            msg.ranges, msg.angle_min, msg.angle_increment,
            range_min=self.range_min, range_max=self.range_max,
            fov_min=self.fov_min, fov_max=self.fov_max,
            median_kernel=self.median_kernel,
            sensor_range_max=msg.range_max,
            return_metadata=True,
        )

        laser_extrinsic = self._laser_extrinsic(source_frame)
        if laser_extrinsic is None:
            return

        attitude_indices = (
            geometry_indices if self.ground_filter_enabled else beam_indices)
        if timing_valid:
            attitude_stamps_ns = (
                header_stamp_ns
                + np.rint(
                    attitude_indices * msg.time_increment * 1e9).astype(np.int64)
            )
        else:
            attitude_stamps_ns = np.full(
                len(attitude_indices), reference_stamp_ns, dtype=np.int64)

        attitude_lookup = None
        reference_base_quaternion = None
        attitude_quaternions = None
        beam_base_quaternions = None
        level_quaternion = None
        if len(self.attitude_buffer) and len(attitude_indices):
            attitude_lookup = self.attitude_buffer.wait_for_lookup_many(
                (reference_stamp_ns,) + tuple(attitude_stamps_ns),
                timeout_sec=self.sensor_sync_wait,
                max_gap_sec=self.max_imu_interp_gap)
            if attitude_lookup.status == LOOKUP_OK:
                reference_base_quaternion = attitude_lookup.quaternions[0]
                attitude_quaternions = attitude_lookup.quaternions[1:]
        with self.sensor_state_lock:
            if self.level_reference.ready:
                level_quaternion = np.copy(self.level_reference.quaternion)

        if self.ground_filter_enabled:
            if (level_quaternion is None or attitude_quaternions is None):
                self.geometry_sync_misses += 1
                self.get_logger().warn(
                    'Ground filter waiting for timestamped IMU level data; '
                    'scan is kept unchanged.', throttle_duration_sec=5.0)
            else:
                ground = mask_ground_and_offplane(
                    geometry_ranges, geometry_angles, geometry_indices,
                    attitude_quaternions, level_quaternion,
                    laser_extrinsic[0], laser_extrinsic[1],
                    self.ground_config)
                self.ground_filtered_beams += int(
                    np.count_nonzero(ground.ground_mask))
                self.offplane_filtered_beams += int(
                    np.count_nonzero(ground.offplane_mask))
                if ground.attitude_fallback:
                    self.attitude_fallback_scans += 1
                    reference_base_quaternion = level_quaternion
                    attitude_quaternions = np.tile(
                        level_quaternion, (len(attitude_indices), 1))
                if ground.degraded:
                    self.geometry_degraded_scans += 1
                    self.get_logger().warn(
                        f'LiDAR frame rejected as non-planar '
                        f'({ground.reason}); coasting existing tracks.',
                        throttle_duration_sec=1.0)
                    active_tracks = self.tracker.step([], reference_stamp)
                    self._publish_tracks(
                        active_tracks, msg.header, reference_stamp_ns)
                    self._log_geometry_diagnostics(reference_stamp)
                    return
                rejected_indices = geometry_indices[
                    ground.ground_mask | ground.offplane_mask]
                keep = ~np.isin(
                    beam_indices, rejected_indices, assume_unique=True)
                points = points[keep]
                point_ranges = point_ranges[keep]
                beam_indices = beam_indices[keep]

        if not len(points):
            active_tracks = self.tracker.step([], reference_stamp)
            self._publish_tracks(active_tracks, msg.header, reference_stamp_ns)
            self._log_geometry_diagnostics(reference_stamp)
            return

        if timing_valid:
            beam_stamps_ns = (
                header_stamp_ns
                + np.rint(
                    beam_indices * msg.time_increment * 1e9).astype(np.int64)
            )
        else:
            beam_stamps_ns = np.full(
                len(beam_indices), reference_stamp_ns, dtype=np.int64)
        if attitude_quaternions is not None:
            positions = np.searchsorted(attitude_indices, beam_indices)
            aligned = (
                np.all(positions < len(attitude_indices))
                and np.array_equal(attitude_indices[positions], beam_indices)
            )
            if aligned:
                beam_base_quaternions = attitude_quaternions[positions]

        points_laser = np.column_stack((points, np.zeros(len(points))))
        if self.deskew_enabled and timing_valid and len(points):
            odom_lookup = self.odom_buffer.wait_for_lookup_many(
                (reference_stamp_ns,) + tuple(beam_stamps_ns),
                timeout_sec=self.sensor_sync_wait,
                max_gap_sec=self.max_odom_interp_gap)
            if odom_lookup.status == LOOKUP_OK:
                reference_pose_sample = odom_lookup.samples[0]
                beam_pose_samples = odom_lookup.samples[1:]
                beam_poses = np.asarray([
                    (sample.x, sample.y, sample.yaw)
                    for sample in beam_pose_samples], dtype=np.float64)
                reference_pose = np.array([
                    reference_pose_sample.x, reference_pose_sample.y,
                    reference_pose_sample.yaw], dtype=np.float64)
                if (level_quaternion is None
                        or reference_base_quaternion is None
                        or beam_base_quaternions is None):
                    identity = quaternion_from_rpy(0.0, 0.0, 0.0)
                    level_for_deskew = identity
                    reference_for_deskew = identity
                    beam_for_deskew = np.tile(identity, (len(points), 1))
                else:
                    level_for_deskew = level_quaternion
                    reference_for_deskew = reference_base_quaternion
                    beam_for_deskew = beam_base_quaternions
                deskewed = deskew_points_to_reference(
                    points_laser, beam_poses, reference_pose,
                    beam_for_deskew, reference_for_deskew,
                    level_for_deskew, laser_extrinsic[0], laser_extrinsic[1],
                    translation_scale=self.deskew_translation_scale)
                points_laser = deskewed.points
                self.deskew_applied_scans += 1
            else:
                self.geometry_sync_misses += 1

        target_from_base = self._lookup_target_transform(
            self.base_frame, reference_time, reference_stamp)
        if target_from_base is None:
            return
        points, sensor_xy = self._points_in_target_frame(
            points_laser, target_from_base, reference_base_quaternion,
            level_quaternion, laser_extrinsic)

        # Capture raw connectivity before map/track filters fragment a wall.
        source_labels, source_radii, source_sizes = \
            adaptive_component_provenance(
                points, point_ranges, msg.angle_increment,
                lambda_angle=self.lambda_angle, sigma=self.sigma,
                beam_indices=beam_indices)

        # ②.5 경계 필터 (이제 target_frame 기준이므로 정적 폴리곤을 그대로 써도 됨)
        # point_ranges도 같은 마스크로 잘라야 포인트와 1:1 대응이 유지된다.
        if self.boundary_polygon is not None:
            keep = polygon_mask(points, self.boundary_polygon)
            points = points[keep]
            point_ranges = point_ranges[keep]
            beam_indices = beam_indices[keep]
            source_labels = source_labels[keep]

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
                keep = self.wall_point_mask(points)
                points = points[keep]
                point_ranges = point_ranges[keep]
                beam_indices = beam_indices[keep]
                source_labels = source_labels[keep]

        # ②.65 트랙 경계(Frenet) 필터 — ForzaETH laserPointOnTrack 이식.
        # 벽거리(②.6)와 직교하는 기준이라 벽 fringe가 남아도 여기서 걸린다:
        #  - 코리도 밖(트랙 폭 초과) 포인트 제거
        #  - 자차보다 max_view_ahead_m 넘게 앞선 포인트 제거. 루프 트랙에서
        #    반대편 벽은 유클리드로 가까워 range_max로는 못 자르지만 s로는
        #    반 바퀴 밖이다 (rosbag_0801: 레이스라인을 막는 유령의 s가
        #    p50 8.0m / p90 11.3m — 실효 인지거리 ~4.5m 밖).
        # 웨이포인트 미수신 중에는 적용하지 않는다(fail-open) — 여기서 스캔을
        # 버리면 스트림이 끊겨 spliner가 fail-safe 정지한다.
        if self.use_track_filter and self.track_bounds is not None:
            if sensor_xy is None:
                self.get_logger().warn(
                    'use_track_filter=true지만 target_frame이 스캔 프레임과 같아 '
                    '자차 위치를 알 수 없음 — 트랙 필터 미적용',
                    throttle_duration_sec=5.0,
                )
            else:
                keep = self.track_bounds.on_track_mask(
                    points, sensor_xy,
                    inflation=self.track_bound_inflation,
                    max_ahead=self.max_view_ahead,
                    max_behind=self.max_view_behind,
                )
                points = points[keep]
                point_ranges = point_ranges[keep]
                beam_indices = beam_indices[keep]
                source_labels = source_labels[keep]

        # ② 클러스터링
        if msg.angle_increment >= self.lambda_angle:
            self.get_logger().warn(
                f'angle_increment ({np.rad2deg(msg.angle_increment):.3f} deg) >= '
                f'lambda_angle_deg ({np.rad2deg(self.lambda_angle):.3f}); '
                'adjacent beams will split aggressively. Raise lambda_angle_deg.',
                throttle_duration_sec=5.0,
            )
        # 열린 공간 완화가 켜져 있으면 낮은 임계로 뽑은 뒤 벽 거리로 선별한다.
        # (adaptive_cluster_points가 내부에서 잘라버리면 되살릴 수 없으므로
        #  낮은 쪽으로 먼저 뽑아야 한다.)
        open_relax = (0 < self.min_cluster_size_open < self.min_cluster_size
                      and self.wall_dist is not None)
        clusters = adaptive_cluster_points(
            points,
            point_ranges,
            angle_increment=msg.angle_increment,
            lambda_angle=self.lambda_angle,
            sigma=self.sigma,
            min_cluster_size=(self.min_cluster_size_open if open_relax
                              else self.min_cluster_size),
            beam_indices=beam_indices,
            source_labels=source_labels,
            source_radii=source_radii,
            source_sizes=source_sizes,
        )
        if open_relax:
            selected = []
            for c in clusters:
                if len(c.points) >= self.min_cluster_size:
                    selected.append(c)
                    continue
                # 작은 클러스터는 벽에서 충분히 떨어져 있을 때만 살린다
                wd = self.wall_distance_at(c.centroid)
                if wd is not None and wd >= self.open_space_dist:
                    selected.append(c)
            if len(selected) != len(clusters):
                self.get_logger().debug(
                    f'열린공간 완화: {len(clusters)}개 중 {len(selected)}개 채택 '
                    f'(min {self.min_cluster_size_open}->{self.min_cluster_size}, '
                    f'벽거리 >= {self.open_space_dist:.2f}m)',
                    throttle_duration_sec=5.0,
                )
            clusters = selected

        # ②.7 물리적으로 불가능한 크기의 클러스터 거부 (맵 필터 2차 방어선).
        # 트래킹 '전'에 버려야 한다 — 트래커에 들어가면 재식별/코스팅으로
        # 살아남아 계속 발행된다.
        if self.max_cluster_radius > 0.0:
            kept, rejected = partition_clusters_by_postfilter_radius(
                clusters, self.max_cluster_radius)
            if rejected:
                rejected_radius = max(c.radius for c in rejected)
                rejected_source_radius = max(
                    c.source_radius for c in rejected)
                self.get_logger().warn(
                    f'{len(rejected)}개 초대형 클러스터 거부 '
                    f'(max_radius={self.max_cluster_radius:.2f}m, '
                    f'필터 후 최대 {rejected_radius:.2f}m, '
                    f'원본 연결성 최대 {rejected_source_radius:.2f}m) '
                    '— 필터 후 크기로 벽/구조물 판단',
                    throttle_duration_sec=2.0,
                )
            clusters = kept

        # ③④ 매칭 + 칼만필터 트래킹
        active_tracks = self.tracker.step(clusters, reference_stamp)
        self._publish_tracks(active_tracks, msg.header, reference_stamp_ns)
        self._log_geometry_diagnostics(reference_stamp)


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
