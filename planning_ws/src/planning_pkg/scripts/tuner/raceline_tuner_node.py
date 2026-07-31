#!/usr/bin/env python3
"""RViz 인터랙티브 레이스라인 튜너 (HMCL unicorn raceline_tuner의 ROS2 구현).

코너별로 라인과 속도를 RViz에서 직접 조절한다:
  - 보라 구체(2.5m 간격)를 드래그 -> 주변 라인이 코사인 falloff로 함께 이동
    (횡방향 성분만 적용, 트랙 경계-safe margin 안으로 자동 클램프)
  - 속도 핸들을 위아래로 드래그 -> 그 구간 속도 조절 (높이 = 속도 * vel_handle_scale).
    핸들·속도곡선·바닥라인 색이 모두 속도에 연동된다 (파랑 느림 -> 빨강 빠름)
  - 아무 핸들이나 우클릭 메뉴:
      속도 재계산(마찰한계) / 실행취소 / 전체 리셋 / CSV 저장
  - 저장 시 원본은 .bak으로 백업하고 같은 9열 CSV 포맷으로 기록.
    저장 후 colcon build 해야 런타임 노드가 새 라인을 읽는다.

실행: ros2 launch planning_pkg raceline_tuner.launch.py
"""
import copy
import importlib.util
import json
import math
import os
import shutil
import sys
import tempfile
import uuid
from pathlib import Path

import numpy as np
import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Point
from interactive_markers.interactive_marker_server import InteractiveMarkerServer
from interactive_markers.menu_handler import MenuHandler
from nav_msgs.msg import OccupancyGrid
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import (InteractiveMarker, InteractiveMarkerControl,
                                    InteractiveMarkerFeedback, Marker, MarkerArray)

G = 9.81


class RacelineTunerNode(Node):
    def __init__(self):
        super().__init__('raceline_tuner_node')

        self.declare_parameter('tuner/raceline_csv', '')      # 절대경로 (launch가 지정)
        self.declare_parameter('tuner/output_csv', '')        # 비면 입력 위에 저장
        self.declare_parameter('tuner/handle_stride', 25)     # 핸들 간격 (wpt 수, 0.1m 간격)
        self.declare_parameter('tuner/falloff_window_m', 8.0) # 드래그 영향 반경
        # d columns는 실벽 거리에서 corridor_margin을 이미 뺀 가용 폭이다.
        # 0.20 + 기본 my_track margin 0.08 = center-to-wall 0.28 m로,
        # 차 반폭 0.15 m와 recovery 0.25 m 계약을 모두 보수적으로 넘는다.
        self.declare_parameter('tuner/min_wall_dist_m', 0.20)
        # build_map CSV의 d_left/d_right는 실제 벽 거리에서 이 값을 뺀
        # "사용 가능한 corridor"다. 저장 때 map에서 실거리를 다시 측정한 뒤
        # 같은 계약으로 이 값을 새로 적용한다.
        self.declare_parameter('tuner/corridor_margin_m', 0.15)
        self.declare_parameter('tuner/v_max_mps', 12.0)
        self.declare_parameter('tuner/mu', 0.85)
        self.declare_parameter('tuner/a_acc_max', 5.5)
        # build_map A_DEC_MAX와 동일 유지. 2026-07-31 실측 제동 한계 ~4.2 m/s²
        # (7.0 가정 프로파일은 헤어핀 진입 과속 → 이탈).
        self.declare_parameter('tuner/a_dec_max', 4.0)
        self.declare_parameter('tuner/wheelbase_m', 0.33)
        self.declare_parameter('tuner/max_steer_rad', 0.36)
        self.declare_parameter('tuner/validation_tolerance', 0.03)
        self.declare_parameter('tuner/ax_tolerance_mps2', 0.5)
        self.declare_parameter('tuner/require_map_validation', True)
        self.declare_parameter('tuner/map_topic', '/map')
        self.declare_parameter('tuner/vehicle_width_m', 0.30)
        self.declare_parameter('tuner/vehicle_length_m', 0.55)
        self.declare_parameter('tuner/vel_handle_scale', 0.3) # 높이 = v * scale [m]

        gp = lambda n: self.get_parameter(n).value
        self.csv_path = Path(gp('tuner/raceline_csv'))
        out = gp('tuner/output_csv')
        self.out_path = Path(out) if out else self.csv_path
        self.stride = int(gp('tuner/handle_stride'))
        self.window_m = gp('tuner/falloff_window_m')
        self.min_wall = gp('tuner/min_wall_dist_m')
        self.corridor_margin = gp('tuner/corridor_margin_m')
        self.v_max = gp('tuner/v_max_mps')
        self.mu = gp('tuner/mu')
        self.a_acc = gp('tuner/a_acc_max')
        self.a_dec = gp('tuner/a_dec_max')
        self.wheelbase = gp('tuner/wheelbase_m')
        self.max_steer = gp('tuner/max_steer_rad')
        self.validation_tolerance = gp('tuner/validation_tolerance')
        self.ax_tolerance = gp('tuner/ax_tolerance_mps2')
        self.require_map_validation = gp('tuner/require_map_validation')
        self.vehicle_width = gp('tuner/vehicle_width_m')
        self.vehicle_length = gp('tuner/vehicle_length_m')
        self.z_scale = gp('tuner/vel_handle_scale')
        safety_values = (
            self.min_wall, self.v_max, self.mu, self.a_acc, self.a_dec,
            self.wheelbase, self.max_steer, self.ax_tolerance, self.z_scale,
            self.vehicle_width, self.vehicle_length,
        )
        if not all(math.isfinite(float(value)) and float(value) > 0.0
                   for value in safety_values) or \
                self.stride <= 0 or \
                not math.isfinite(self.window_m) or self.window_m <= 0.0 or \
                self.mu > 1.5 or \
                self.min_wall < 0.5 * self.vehicle_width or \
                not math.isfinite(self.corridor_margin) or \
                not 0.0 <= self.corridor_margin <= 1.0 or \
                not 0.1 <= self.wheelbase <= 1.0 or \
                self.max_steer > 0.8 or \
                not math.isfinite(self.validation_tolerance) or \
                not 0.0 <= self.validation_tolerance <= 0.10 or \
                self.ax_tolerance > 1.0:
            raise RuntimeError('튜너 안전 파라미터가 비유한/비양수입니다')
        if not self.require_map_validation:
            raise RuntimeError(
                'tuner/require_map_validation=false는 안전 저장 계약상 허용되지 않습니다')

        # CSV 로드: s,x,y,d_right,d_left,psi,kappa,vx,ax
        self.wpts = self._load_csv(self.csv_path)
        self.original = self.wpts.copy()
        self.undo_stack = []           # 최근 5개 스냅샷
        self.backed_up = False
        self.map_free = None
        self.map_resolution = 0.0
        self.map_origin = (0.0, 0.0)
        self.map_origin_yaw = 0.0
        self._tracking_model = None
        self._tracking_model_error = ''
        self._tracking_profile_signature = ''
        self._workspace_root = None
        try:
            self._tracking_model = self._load_tracking_safety_model()
        except Exception as exc:
            # 편집/진단은 계속 허용하되, 모델을 못 불러온 상태에서는 저장을
            # 절대 허용하지 않는다. 사용자가 작업을 마친 뒤에야 알게 되지 않도록
            # 시작 시점에 명확히 알린다.
            self._tracking_model_error = str(exc)
            self.get_logger().error(
                '폐루프 배포 제어기 검증 모델을 불러오지 못했습니다. '
                f'CSV 저장은 잠깁니다: {exc}')

        n = len(self.wpts)
        self.ds = float(np.mean(np.diff(self.wpts[:, 0])))
        self.get_logger().info(
            f'raceline 로드: {n} wpts, {self.wpts[-1,0]:.1f} m ({self.csv_path.name})')

        self.pub_preview = self.create_publisher(MarkerArray, '/raceline_tuner/preview', 10)
        map_qos = QoSProfile(
            depth=1, reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(
            OccupancyGrid, gp('tuner/map_topic'), self._on_map, map_qos)

        self.server = InteractiveMarkerServer(self, 'raceline_tuner')
        self.menu = MenuHandler()
        self.menu.insert('속도 재계산 (마찰한계)', callback=self.on_menu)   # id 1
        self.menu.insert('실행취소 (undo)', callback=self.on_menu)          # id 2
        self.menu.insert('전체 리셋', callback=self.on_menu)                # id 3
        self.menu.insert('CSV 저장', callback=self.on_menu)                 # id 4

        self.anchor_idxs = list(range(0, n, self.stride))
        self._make_all_markers()
        self.server.applyChanges()
        self._publish_preview()

        # RViz 재시작 등으로 preview가 사라지는 것 대비 주기 재발행
        self.create_timer(2.0, self._publish_preview)
        self.get_logger().info(
            f'튜너 준비 완료: 핸들 {len(self.anchor_idxs)}개 (간격 {self.stride*self.ds:.1f} m). '
            'RViz에서 드래그 + 우클릭 메뉴 사용')

    # ------------------------------------------------------------------ io
    def _on_map(self, msg: OccupancyGrid):
        if msg.info.width == 0 or msg.info.height == 0 or \
                not math.isfinite(msg.info.resolution) or \
                msg.info.resolution <= 0.0:
            self.get_logger().error('유효하지 않은 OccupancyGrid 거부')
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
            self.get_logger().error('OccupancyGrid data/origin이 비정상이라 거부')
            return
        grid = np.asarray(msg.data, dtype=np.int16).reshape(
            msg.info.height, msg.info.width)
        # 알려진 free 셀만 안전하다고 본다. unknown과 occupied는 모두 불가.
        self.map_free = (grid >= 0) & (grid < 65)
        self.map_resolution = float(msg.info.resolution)
        self.map_origin = (
            float(msg.info.origin.position.x),
            float(msg.info.origin.position.y))
        qx, qy, qz, qw = (
            q.x / q_norm, q.y / q_norm, q.z / q_norm, q.w / q_norm)
        self.map_origin_yaw = math.atan2(
            2.0 * (qw * qz + qx * qy),
            1.0 - 2.0 * (qy * qy + qz * qz))
        self.get_logger().info(
            f'저장 검증용 map 수신: {msg.info.width}x{msg.info.height}, '
            f'{self.map_resolution:.3f}m/cell')

    def _load_tracking_safety_model(self):
        """ROS system Python용 pure-numpy 배포 제어기 모델을 로드한다."""
        candidates = []
        package_root = None
        try:
            package_root = self.csv_path.resolve().parents[2]
            candidates.append(
                package_root / 'scripts' / 'tuner' /
                'controller_safety_model.py')
        except (IndexError, OSError):
            pass
        source_script = Path(__file__).resolve()
        candidates.append(
            source_script.parent / 'controller_safety_model.py')
        candidates = list(dict.fromkeys(candidates))
        model_path = next((path for path in candidates if path.is_file()), None)
        if model_path is None:
            raise RuntimeError(
                'controller_safety_model.py를 찾을 수 없음 (찾은 위치: '
                + ', '.join(str(path) for path in candidates) + ')')

        module_name = '_planning_pkg_tuner_controller_safety'
        cached = sys.modules.get(module_name)
        if cached is not None:
            cached_path = Path(getattr(cached, '__file__', '')).resolve()
            if cached_path != model_path.resolve():
                raise RuntimeError(
                    f'다른 build_map 모델이 이미 로드됨: {cached_path}')
            module = cached
        else:
            spec = importlib.util.spec_from_file_location(module_name, model_path)
            if spec is None or spec.loader is None:
                raise RuntimeError(
                    f'controller model import spec 생성 실패: {model_path}')
            module = importlib.util.module_from_spec(spec)
            sys.modules[module_name] = module
            try:
                spec.loader.exec_module(module)
                profiles = module.selected_controller_profiles()
                if not profiles:
                    raise RuntimeError('배포 controller profile 선택이 비어 있음')
            except Exception:
                sys.modules.pop(module_name, None)
                raise
        module.selected_controller_profiles()
        if package_root is None:
            raise RuntimeError(
                'source planning_pkg root를 CSV 경로에서 찾을 수 없음')
        generator_path = (
            package_root / 'scripts' / 'global_traj_sim' / 'build_map.py')
        self._tracking_profile_signature = \
            module.assert_generator_profile_contract(generator_path)
        self._workspace_root = package_root.parents[2]
        return module

    def _map_points_free(self, points):
        if self.map_free is None or self.map_resolution <= 0.0:
            return np.zeros(len(np.atleast_2d(points)), dtype=bool)
        points = np.atleast_2d(points)
        dx = points[:, 0] - self.map_origin[0]
        dy = points[:, 1] - self.map_origin[1]
        c_yaw, s_yaw = math.cos(self.map_origin_yaw), math.sin(self.map_origin_yaw)
        gx = c_yaw * dx + s_yaw * dy
        gy = -s_yaw * dx + c_yaw * dy
        cols = np.floor(gx / self.map_resolution).astype(int)
        rows = np.floor(gy / self.map_resolution).astype(int)
        height, width = self.map_free.shape
        in_map = ((rows >= 0) & (rows < height) &
                  (cols >= 0) & (cols < width))
        free = np.zeros(len(points), dtype=bool)
        free[in_map] = self.map_free[rows[in_map], cols[in_map]]
        return free

    def _world_to_map_metric(self, point, direction=None):
        """ROS map origin pose 기준 좌표[m]로 점/방향을 변환한다."""
        dx = float(point[0]) - self.map_origin[0]
        dy = float(point[1]) - self.map_origin[1]
        c_yaw = math.cos(self.map_origin_yaw)
        s_yaw = math.sin(self.map_origin_yaw)
        mapped_point = np.array([
            c_yaw * dx + s_yaw * dy,
            -s_yaw * dx + c_yaw * dy,
        ])
        if direction is None:
            return mapped_point
        mapped_direction = np.array([
            c_yaw * float(direction[0]) + s_yaw * float(direction[1]),
            -s_yaw * float(direction[0]) + c_yaw * float(direction[1]),
        ])
        return mapped_point, mapped_direction

    def _cell_is_free(self, row, col):
        if self.map_free is None:
            return False
        height, width = self.map_free.shape
        return (0 <= row < height and 0 <= col < width
                and bool(self.map_free[row, col]))

    def _raycast_free_distance(self, point, direction):
        """첫 non-free cell 경계까지의 보수적 거리[m]를 grid DDA로 구한다.

        샘플링 간격에 따라 얇은 occupied/unknown cell을 건너뛰지 않으며,
        grid corner를 정확히 지날 때는 맞닿은 양 옆 cell까지 모두 free여야
        통과시킨다.
        """
        if self.map_free is None or self.map_resolution <= 0.0:
            return None
        mapped_point, mapped_direction = self._world_to_map_metric(
            point, direction)
        if not np.isfinite(mapped_point).all() or \
                not np.isfinite(mapped_direction).all():
            return None
        norm = float(np.hypot(*mapped_direction))
        if norm <= 1e-12:
            return None
        mapped_direction /= norm
        gx, gy = mapped_point
        dx, dy = mapped_direction
        resolution = self.map_resolution
        col = int(math.floor(gx / resolution))
        row = int(math.floor(gy / resolution))
        if not self._cell_is_free(row, col):
            return None

        if dx > 1e-12:
            step_col = 1
            t_max_x = ((col + 1) * resolution - gx) / dx
            t_delta_x = resolution / dx
        elif dx < -1e-12:
            step_col = -1
            t_max_x = (col * resolution - gx) / dx
            t_delta_x = -resolution / dx
        else:
            step_col = 0
            t_max_x = math.inf
            t_delta_x = math.inf
        if dy > 1e-12:
            step_row = 1
            t_max_y = ((row + 1) * resolution - gy) / dy
            t_delta_y = resolution / dy
        elif dy < -1e-12:
            step_row = -1
            t_max_y = (row * resolution - gy) / dy
            t_delta_y = -resolution / dy
        else:
            step_row = 0
            t_max_y = math.inf
            t_delta_y = math.inf
        t_max_x = max(float(t_max_x), 0.0)
        t_max_y = max(float(t_max_y), 0.0)
        height, width = self.map_free.shape
        # 한 ray가 직사각 grid 안에서 넘을 수 있는 cell boundary 수의 상한.
        for _ in range(height + width + 4):
            corner_tol = 1e-12 * max(1.0, abs(t_max_x), abs(t_max_y))
            if math.isfinite(t_max_x) and math.isfinite(t_max_y) and \
                    abs(t_max_x - t_max_y) <= corner_tol:
                distance = min(t_max_x, t_max_y)
                candidates = (
                    (row, col + step_col),
                    (row + step_row, col),
                    (row + step_row, col + step_col),
                )
                if any(not self._cell_is_free(r, c) for r, c in candidates):
                    return max(0.0, distance - max(1e-6, resolution * 1e-4))
                row += step_row
                col += step_col
                t_max_x += t_delta_x
                t_max_y += t_delta_y
            elif t_max_x < t_max_y:
                distance = t_max_x
                col += step_col
                if not self._cell_is_free(row, col):
                    return max(0.0, distance - max(1e-6, resolution * 1e-4))
                t_max_x += t_delta_x
            else:
                distance = t_max_y
                row += step_row
                if not self._cell_is_free(row, col):
                    return max(0.0, distance - max(1e-6, resolution * 1e-4))
                t_max_y += t_delta_y
        return None

    def _recompute_wall_distances_from_map(self):
        """현재 line의 좌/우 corridor 열을 OccupancyGrid에서 완전히 재작성."""
        if self.map_free is None:
            return False, 'map을 아직 받지 못함'
        xy = self.wpts[:, 1:3]
        psi = self.wpts[:, 5]
        left_raw = np.empty(len(self.wpts), dtype=float)
        right_raw = np.empty(len(self.wpts), dtype=float)
        for index, (point, heading) in enumerate(zip(xy, psi)):
            normal = np.array([-math.sin(heading), math.cos(heading)])
            left = self._raycast_free_distance(point, normal)
            right = self._raycast_free_distance(point, -normal)
            if left is None or right is None:
                return False, (
                    f'map normal raycast 실패/라인 중심 non-free: idx={index}, '
                    f's={self.wpts[index, 0]:.2f}m')
            left_raw[index] = left
            right_raw[index] = right
        # build_map과 동일하게 실제 벽 거리에서 corridor reserve를 차감한다.
        left = np.maximum(left_raw - self.corridor_margin, 0.0)
        right = np.maximum(right_raw - self.corridor_margin, 0.0)
        if not np.isfinite(left).all() or not np.isfinite(right).all():
            return False, 'map raycast 결과에 NaN/Inf 포함'
        self.wpts[:, 3] = right
        self.wpts[:, 4] = left
        return True, ''

    def _wall_clearance_is_safe(self):
        clearance = np.minimum(self.wpts[:, 3], self.wpts[:, 4])
        bad = np.flatnonzero(clearance + 1e-9 < self.min_wall)
        if not len(bad):
            return True, ''
        first = int(bad[0])
        return False, (
            f'사용 가능 벽 여유(min_wall={self.min_wall:.3f}m) 미달: '
            f'{len(bad)}점, 최소 {clearance[bad].min():.3f}m, '
            f'최초 s={self.wpts[first, 0]:.2f}m '
            f'(실벽 거리에는 corridor margin {self.corridor_margin:.3f}m 포함)')

    def _footprints_free(self, xy, psi):
        """주어진 pose들의 직사각 차량 footprint 전체를 map에 대조."""
        if self.map_free is None:
            return np.zeros(len(np.atleast_2d(xy)), dtype=bool)
        xy = np.atleast_2d(xy)
        psi = np.atleast_1d(psi)
        if len(xy) != len(psi) or not np.isfinite(xy).all() or \
                not np.isfinite(psi).all():
            return np.zeros(len(xy), dtype=bool)
        step = max(0.5 * self.map_resolution, 0.01)
        n_lon = max(1, int(math.ceil(self.vehicle_length / step)))
        n_lat = max(1, int(math.ceil(self.vehicle_width / step)))
        lon = np.linspace(
            -0.5 * self.vehicle_length, 0.5 * self.vehicle_length, n_lon + 1)
        lat = np.linspace(
            -0.5 * self.vehicle_width, 0.5 * self.vehicle_width, n_lat + 1)
        cos_psi, sin_psi = np.cos(psi), np.sin(psi)
        ok = np.ones(len(xy), dtype=bool)
        for dl in lon:
            for dd in lat:
                points = xy + np.column_stack([
                    dl * cos_psi - dd * sin_psi,
                    dl * sin_psi + dd * cos_psi])
                ok &= self._map_points_free(points)
        return ok

    def _footprint_is_free(self):
        if self.map_free is None:
            return False, 'map을 아직 받지 못함'
        xy = self.wpts[:, 1:3]
        psi = self.wpts[:, 5]
        ok = self._footprints_free(xy, psi)
        if ok.all():
            return True, ''
        bad = np.flatnonzero(~ok)
        return False, (
            f'차량 footprint가 map 벽/unknown 침범: {len(bad)}점, '
            f'최초 s={self.wpts[bad[0], 0]:.2f}m')

    def _closed_loop_tracking_is_safe(self):
        """build_map과 동일한 선택 배포 프로파일 모두로 1 lap을 검증한다."""
        if self._tracking_model is None:
            return False, (
                '폐루프 검증 모델이 없어 저장 불가: '
                + (self._tracking_model_error or '알 수 없는 로드 오류'))
        if self.map_free is None:
            return False, '폐루프 검증용 map을 아직 받지 못함'

        node = self

        class OccupancyGridAdapter:
            def __init__(self):
                self.res = node.map_resolution

            def is_free(self, points):
                return node._map_points_free(points)

        model = self._tracking_model
        trajectory = {
            's': self.wpts[:, 0].copy(),
            'xy': self.wpts[:, 1:3].copy(),
            'd_right': self.wpts[:, 3].copy(),
            'd_left': self.wpts[:, 4].copy(),
            'psi': self.wpts[:, 5].copy(),
            'kappa': self.wpts[:, 6].copy(),
            'vx': self.wpts[:, 7].copy(),
            'ax': self.wpts[:, 8].copy(),
        }
        grid = OccupancyGridAdapter()
        failures = []
        passed = []
        try:
            profiles = model.selected_controller_profiles()
            for profile in profiles:
                profile_name = profile['name']
                log = model.PurePursuitSim(
                    trajectory, profile_name,
                    workspace_root=self._workspace_root,
                    mu=self.mu).run(requested_laps=1)
                ok, messages = model.verify_simulation(
                    self.out_path.name, log, grid, 1, profile_name, self.mu)
                # build_map 기본 차량보다 tuner 설정이 큰 경우까지 포함해,
                # 실제 추종 pose마다 현재 설정 직사각 footprint를 다시 확인한다.
                if len(log.get('x', ())) and len(log.get('theta', ())):
                    pose_xy = np.column_stack([log['x'], log['y']])
                    footprint_ok = self._footprints_free(
                        pose_xy, log['theta'])
                    bad = np.flatnonzero(~footprint_ok)
                    if len(bad):
                        first = int(bad[0])
                        messages.append(
                            f'{profile_name}: [FAIL] 설정 차량 footprint가 '
                            f'폐루프 중 벽/unknown 침범 ({len(bad)} step, '
                            f'최초 t={log["t"][first]:.2f}s)')
                        ok = False
                else:
                    messages.append(
                        f'{profile_name}: [FAIL] 폐루프 pose log가 비어 있음')
                    ok = False
                if ok:
                    passed.append(profile_name)
                else:
                    failures.extend(
                        message for message in messages
                        if '[FAIL]' in message)
        except Exception as exc:
            return False, f'폐루프 배포 제어기 검증 실행 오류: {exc}'
        if failures:
            return False, '\n  - '.join(failures)
        if not passed:
            return False, '검증할 배포 controller profile이 비어 있음'
        return True, ', '.join(passed)

    def _load_csv(self, path):
        rows = []
        with open(path) as f:
            for line in f:
                if line.startswith('#') or not line.strip():
                    continue
                row = [float(v) for v in line.split(';')]
                if len(row) != 9:
                    raise RuntimeError(f'CSV 행이 9열 아님: {len(row)}열')
                rows.append(row)
        a = np.asarray(rows, dtype=float)
        if a.ndim != 2 or a.shape[1] != 9 or len(a) < 8:
            raise RuntimeError(f'CSV 9열 아님: {a.shape}')
        if not np.isfinite(a).all():
            raise RuntimeError('CSV에 NaN/Inf 포함')
        return a

    def _save_csv(self):
        # CSV의 s/psi/kappa 및 corridor 열을 기존 값으로 신뢰하지 않는다.
        # 현재 xy에서 기하를 재생성하고 최신 OccupancyGrid로 좌우 벽을 raycast한다.
        self._recompute_geometry()
        wall_ok, reason = self._recompute_wall_distances_from_map()
        if not wall_ok:
            self.get_logger().error(f'저장 거부: {reason}')
            return
        wall_ok, reason = self._wall_clearance_is_safe()
        if not wall_ok:
            self.get_logger().error(f'저장 거부: {reason}')
            return
        try:
            self._enforce_velocity_feasibility()
        except (RuntimeError, ValueError) as exc:
            self.get_logger().error(f'저장 거부: 속도 안전 투영 실패: {exc}')
            return
        errors = self._validation_errors()
        if errors:
            self.get_logger().error(
                '저장 거부: 고속주행 안전 검증 실패\n  - ' + '\n  - '.join(errors))
            return
        footprint_ok, reason = self._footprint_is_free()
        if not footprint_ok:
            self.get_logger().error(f'저장 거부: {reason}')
            return
        tracking_ok, reason = self._closed_loop_tracking_is_safe()
        if not tracking_ok:
            self.get_logger().error(
                '저장 거부: 선택된 배포 제어기 폐루프 검증 실패\n  - ' + reason)
            return
        self.get_logger().info(f'폐루프 배포 제어기 검증 통과: {reason}')
        if not self.backed_up and self.out_path == self.csv_path:
            backup_path = Path(str(self.csv_path) + '.bak')
            suffix = 1
            while backup_path.exists():
                backup_path = Path(str(self.csv_path) + f'.bak.{suffix}')
                suffix += 1
            fd, backup_tmp = tempfile.mkstemp(
                prefix=f'.{backup_path.name}.', suffix='.tmp',
                dir=backup_path.parent)
            try:
                with self.csv_path.open('rb') as src, os.fdopen(fd, 'wb') as dst:
                    shutil.copyfileobj(src, dst)
                    dst.flush()
                    os.fsync(dst.fileno())
                os.replace(backup_tmp, backup_path)
                self._fsync_directory(backup_path.parent)
            finally:
                if os.path.exists(backup_tmp):
                    os.unlink(backup_tmp)
            self.backed_up = True
            self.get_logger().info(f'원본 백업: {backup_path}')
        generation_id = uuid.uuid4().hex
        role, counterpart_path, generator_manifest_path = \
            self._pairing_paths(self.out_path)
        header = (
                  f'# generation_id={generation_id}\n'
                  '# tuned by raceline_tuner_node; map raycast + deployed-controller closed-loop verified\n'
                  f'# controller_safety_model={self._tracking_model.MODEL_VERSION}; '
                  f'profile_signature={self._tracking_profile_signature}\n'
                  f'# pairing_status=disabled_until_counterpart_regeneration; '
                  f'edited_role={role}\n'
                  f'# corridor_margin_m={self.corridor_margin:.7f}; '
                  f'min_wall_dist_m={self.min_wall:.7f}\n'
                  '# s_m; x_m; y_m; d_right; d_left; psi_rad; kappa_radpm; vx_mps; ax_mps2\n')
        self.out_path.parent.mkdir(parents=True, exist_ok=True)
        fd, tmp_name = tempfile.mkstemp(
            prefix=f'.{self.out_path.name}.', suffix='.tmp',
            dir=self.out_path.parent)
        try:
            with os.fdopen(fd, 'w') as f:
                f.write(header)
                for r in self.wpts:
                    f.write('; '.join(f'{v:.7f}' for v in r) + '\n')
                f.flush()
                os.fsync(f.fileno())
            os.replace(tmp_name, self.out_path)
            self._fsync_directory(self.out_path.parent)
        finally:
            if os.path.exists(tmp_name):
                os.unlink(tmp_name)
        self._write_tuner_manifest(
            generation_id, role, counterpart_path, generator_manifest_path)
        self.get_logger().info(
            f'저장 완료: {self.out_path}\n'
            f'  generation_id={generation_id}; IQP/SP pairing 비활성 '
            f'(counterpart 재생성 필요: {counterpart_path or "해당 없음"})\n'
            f'  기존 generator manifest는 stale: '
            f'{generator_manifest_path or "해당 없음"}\n'
            '  -> 런타임 반영: cd ~/IFAC_ws/planning_ws && colcon build --packages-select planning_pkg')

    @staticmethod
    def _pairing_paths(path):
        """편집한 IQP/SP의 counterpart와 기존 generator manifest를 식별."""
        path = Path(path)
        name = path.name
        if '_iqp_' in name:
            map_name = name.split('_iqp_', 1)[0]
            return (
                'main',
                path.with_name(name.replace('_iqp_', '_sp_', 1)),
                path.with_name(f'{map_name}_generation_manifest.json'))
        if '_sp_' in name:
            map_name = name.split('_sp_', 1)[0]
            return (
                'sp',
                path.with_name(name.replace('_sp_', '_iqp_', 1)),
                path.with_name(f'{map_name}_generation_manifest.json'))
        return 'standalone', None, None

    def _write_tuner_manifest(
            self, generation_id, role, counterpart_path,
            generator_manifest_path):
        """원본 generator manifest를 보존하며 stale/pair-disabled 상태를 기록."""
        manifest_path = Path(str(self.out_path) + '.tuner_manifest.json')
        payload = {
            'schema_version': 1,
            'generation_id': generation_id,
            'edited_csv': str(self.out_path),
            'edited_role': role,
            'paired_counterpart': (
                None if counterpart_path is None else str(counterpart_path)),
            'paired_counterpart_enabled': False,
            'pairing_disabled_reason': (
                'interactive tuning created a new unpaired generation; '
                'regenerate IQP and SP together before enabling overtaking'),
            'generator_manifest': (
                None if generator_manifest_path is None
                else str(generator_manifest_path)),
            'generator_manifest_status': 'stale_do_not_use',
            'controller_safety_model': self._tracking_model.MODEL_VERSION,
            'profile_signature': self._tracking_profile_signature,
        }
        fd, temporary = tempfile.mkstemp(
            prefix=f'.{manifest_path.name}.', suffix='.tmp',
            dir=manifest_path.parent)
        try:
            with os.fdopen(fd, 'w') as handle:
                json.dump(payload, handle, indent=2, sort_keys=True)
                handle.write('\n')
                handle.flush()
                os.fsync(handle.fileno())
            os.replace(temporary, manifest_path)
            self._fsync_directory(manifest_path.parent)
        finally:
            if os.path.exists(temporary):
                os.unlink(temporary)

    @staticmethod
    def _fsync_directory(directory):
        """rename 결과까지 power-loss에 견디도록 부모 directory entry를 동기화."""
        flags = os.O_RDONLY | getattr(os, 'O_DIRECTORY', 0)
        directory_fd = os.open(directory, flags)
        try:
            os.fsync(directory_fd)
        finally:
            os.close(directory_fd)

    def _validation_errors(self):
        """런타임 global_trajectory_node와 같은 핵심 의미 검증을 저장 전에 수행."""
        errors = []
        a = self.wpts
        if a.ndim != 2 or a.shape[1] != 9 or len(a) < 8:
            return [f'9열 waypoint가 8개 이상이어야 함 (현재 shape={a.shape})']
        if not np.isfinite(a).all():
            errors.append('NaN/Inf 포함')
            return errors
        if np.any(a[:, 0] < 0.0) or np.any(a[:, 3:5] < 0.0) or \
                np.any(a[:, 7] < 0.0):
            errors.append('음수 s/벽거리/속도 포함')
        ds = np.diff(a[:, 0])
        if np.any(ds <= 1e-5):
            errors.append('s가 단조 증가하지 않음')
            return errors
        if ds.max() > ds.min() * 1.25:
            errors.append(f'waypoint 간격 불균일 ({ds.min():.3f}~{ds.max():.3f} m)')
        geometric_ds = np.hypot(
            np.diff(a[:, 1]), np.diff(a[:, 2]))
        spacing_error = np.abs(geometric_ds - ds)
        if np.any(geometric_ds <= 1e-5) or \
                np.any(spacing_error > np.maximum(0.02, 0.20 * ds)):
            errors.append('s 간격과 Cartesian waypoint 간격 불일치')
        closure = float(np.hypot(*(a[0, 1:3] - a[-1, 1:3])))
        if closure > ds.max() * 2.5:
            errors.append(f'폐루프가 닫히지 않음 (끝-시작 {closure:.3f} m)')
        elif closure < ds.min() * 0.25:
            errors.append(
                f'끝 waypoint가 시작점과 중복/과밀함 (끝-시작 {closure:.3f} m)')
        closed_seg = np.hypot(
            np.roll(a[:, 1], -1) - a[:, 1],
            np.roll(a[:, 2], -1) - a[:, 2])
        if np.any(closed_seg <= 1e-5):
            errors.append('폐루프 segment에 중복 waypoint 포함')
            return errors
        kappa_limit = math.tan(self.max_steer) / max(self.wheelbase, 1e-6)
        max_kappa = float(np.abs(a[:, 6]).max())
        if max_kappa > kappa_limit * (1.0 + self.validation_tolerance):
            errors.append(f'조향 한계 초과: |kappa|max {max_kappa:.3f} > '
                          f'{kappa_limit:.3f} 1/m')
        lateral = a[:, 7] ** 2 * np.abs(a[:, 6])
        worst_segment_lateral = np.maximum(
            lateral, np.roll(lateral, -1))
        combined = np.hypot(worst_segment_lateral, a[:, 8])
        if combined.max() > self.mu * G * (1.0 + self.validation_tolerance):
            errors.append(f'마찰 한계 초과: 합성가속도 max {combined.max():.2f} > '
                          f'mu*g {self.mu * G:.2f} m/s²')
        if np.max(a[:, 8]) > self.a_acc * (1.0 + self.validation_tolerance):
            errors.append(
                f'가속 한계 초과: ax max {np.max(a[:, 8]):.2f} > '
                f'{self.a_acc:.2f} m/s²')
        if -np.min(a[:, 8]) > self.a_dec * (1.0 + self.validation_tolerance):
            errors.append(
                f'감속 한계 초과: -ax max {-np.min(a[:, 8]):.2f} > '
                f'{self.a_dec:.2f} m/s²')
        numeric_ax = (
            np.roll(a[:, 7], -1) ** 2 - a[:, 7] ** 2) / (2.0 * closed_seg)
        ax_error = float(np.abs(numeric_ax - a[:, 8]).max())
        if ax_error > self.ax_tolerance:
            wrap_error = abs(float(numeric_ax[-1] - a[-1, 8]))
            errors.append(
                f'ax 열 불일치(끝→시작 포함): max error {ax_error:.2f}, '
                f'wrap error {wrap_error:.2f} m/s²')
        return errors

    # ---------------------------------------------------------- geometry
    def _recompute_geometry(self):
        """xy 변경 후 s, psi, kappa 재계산 (주기적 경로)."""
        xy = self.wpts[:, 1:3]
        d = np.diff(xy, axis=0, append=xy[:1])
        seg = np.hypot(d[:, 0], d[:, 1])
        s = np.concatenate([[0.0], np.cumsum(seg)])[:-1]
        self.wpts[:, 0] = s
        psi = np.arctan2(d[:, 1], d[:, 0])
        # 각도 평활 (전후 세그먼트 평균)
        psi_prev = np.roll(psi, 1)
        dpsi = np.arctan2(np.sin(psi - psi_prev), np.cos(psi - psi_prev))
        self.wpts[:, 5] = psi_prev + dpsi * 0.5
        ds = np.maximum(seg, 1e-6)
        self.wpts[:, 6] = dpsi / ds  # kappa

    def _recompute_velocity(self):
        """양 endpoint 최악 횡가속도를 쓰는 폐루프 fixed-point 재계산."""
        self._project_velocity_profile(speed_ceiling=None)

    def _enforce_velocity_feasibility(self):
        """현재 수동 속도를 상한으로 폐루프 segment 제약에 고정점 투영."""
        self._project_velocity_profile(
            speed_ceiling=np.maximum(self.wpts[:, 7], 0.0))

    def _project_velocity_profile(self, speed_ceiling):
        if self._tracking_model is None:
            raise RuntimeError(
                'velocity 안전 투영 모델 없음: ' +
                (self._tracking_model_error or '알 수 없는 로드 오류'))
        kappa = np.abs(self.wpts[:, 6])
        # 짧은 곡률 spike를 평균으로 숨기지 않고 ±5 waypoint 최댓값으로
        # 확장한다. 이 값으로 segment 양 endpoint 중 최악을 다시 사용한다.
        filter_radius = 5
        filtered_kappa = np.max([
            np.roll(kappa, shift)
            for shift in range(-filter_radius, filter_radius + 1)
        ], axis=0)
        xy = self.wpts[:, 1:3]
        segment_ds = np.hypot(
            np.roll(xy[:, 0], -1) - xy[:, 0],
            np.roll(xy[:, 1], -1) - xy[:, 1])
        if np.any(segment_ds <= 1e-6) or not np.isfinite(segment_ds).all():
            raise RuntimeError('속도 투영 거부: 폐루프 segment 길이가 비정상')
        v = self._tracking_model.velocity_profile(
            self.wpts[:, 0], filtered_kappa, self.v_max, self.mu,
            self.a_acc, self.a_dec, segment_ds,
            speed_ceiling=speed_ceiling)
        self.wpts[:, 7] = v
        self.wpts[:, 8] = self._tracking_model.closed_loop_ax(
            v, segment_ds)

    def _recompute_velocity_window(self, center_idx):
        """라인 편집으로 곡률이 바뀐 근방만 속도를 재계산.

        _recompute_velocity()처럼 트랙 전체를 다시 풀면, 다른 코너에서 이미
        수동으로 튜닝해둔 속도(vel 핸들)가 라인을 아주 조금만 옮겨도 매번
        자동 계산값으로 리셋돼 버린다. 곡률이 실제로 바뀐 구간
        (_smooth_xy_window 와 같은 폭)에, 감속이 코너보다 앞에서 시작될 수
        있는 만큼(최대 제동거리)의 여유를 더한 구간만 다시 풀고, 그 양 끝은
        기존 속도(수동 튜닝 포함)에 고정한 채 안쪽만 최적화한다.
        """
        n = len(self.wpts)
        edit_half = int(self.window_m / self.ds) + 6      # _smooth_xy_window 와 동일 폭
        a_lim = max(min(self.a_dec, self.a_acc), 0.5)
        brake_half = int(self.v_max ** 2 / (2 * a_lim) / self.ds)
        kf = 5                                             # kappa 최댓값 필터 반경
        half = edit_half + brake_half + kf

        if 2 * half + 1 >= n:              # 트랙이 작으면 국소화 의미 없음 -> 전체 재계산
            self._recompute_velocity()
            return int(self.window_m / self.ds)

        L = 2 * half + 1
        idxs = np.array([(center_idx + o) % n for o in range(-half, half + 1)])
        kappa_raw = np.abs(self.wpts[idxs, 6])
        kappa = np.array([kappa_raw[max(0, i - kf):i + kf + 1].max() for i in range(L)])

        a_lat_max = self.mu * G
        v = np.minimum(np.sqrt(a_lat_max / np.maximum(kappa, 1e-9)), self.v_max)

        left_pin = float(self.wpts[idxs[0], 7])
        right_pin = float(self.wpts[idxs[-1], 7])
        v[0], v[-1] = left_pin, right_pin

        seg = np.maximum(np.hypot(np.diff(self.wpts[idxs, 1]), np.diff(self.wpts[idxs, 2])), 1e-6)

        for _ in range(3):
            for i in range(L - 1):                          # forward (가속 한계)
                a_lat = v[i] ** 2 * kappa[i]
                a_avail = self.a_acc * math.sqrt(max(0.0, 1.0 - (a_lat / a_lat_max) ** 2))
                v[i + 1] = min(v[i + 1], math.sqrt(v[i] ** 2 + 2 * a_avail * seg[i]))
            v[-1] = right_pin
            for i in range(L - 2, -1, -1):                  # backward (감속 한계)
                a_lat = v[i + 1] ** 2 * kappa[i + 1]
                a_avail = self.a_dec * math.sqrt(max(0.0, 1.0 - (a_lat / a_lat_max) ** 2))
                v[i] = min(v[i], math.sqrt(v[i + 1] ** 2 + 2 * a_avail * seg[i]))
            v[0] = left_pin

        # 점별 요철 저역통과 (전체 재계산과 동일 방식, 이 구간 내부만)
        kernel = np.array([1.0, 2.0, 3.0, 2.0, 1.0])
        kernel /= kernel.sum()
        pad = len(kernel) // 2
        if L > 2 * pad:
            v_pad = np.concatenate([np.full(pad, v[0]), v, np.full(pad, v[-1])])
            v_smooth = np.convolve(v_pad, kernel, mode='valid')
            v = np.minimum(v, v_smooth)
            v[0], v[-1] = left_pin, right_pin

        self.wpts[idxs, 7] = v
        # ax: idxs[-1] 이후는 v가 안 바뀌었으므로 idxs[0..-2] 구간만 갱신하면 충분
        vn = v[1:]
        self.wpts[idxs[:-1], 8] = (vn ** 2 - v[:-1] ** 2) / (2 * seg)

        return half

    def _falloff_apply(self, center_idx, delta, col_apply):
        """center 주변 window에 코사인 falloff로 delta 적용. col_apply(i, w*delta)."""
        n = len(self.wpts)
        win = int(self.window_m / self.ds)
        for off in range(-win, win + 1):
            i = (center_idx + off) % n
            w = 0.5 * (1.0 + math.cos(math.pi * off / win)) if win > 0 else 1.0
            col_apply(i, delta * w)

    def _smooth_xy_window(self, center_idx, k=6):
        """드래그 후 편집 구간의 고주파 요철 제거.

        윈도우 경계는 고정(연속성 유지)하고 중심부만 이동평균(±k*ds)으로
        블렌딩한다. 의도한 범프(저주파)는 살리고 점별 독립 이동으로 생긴
        요철(고주파)만 없애 '찌그러짐'을 막는다.
        """
        n = len(self.wpts)
        span = int(self.window_m / self.ds)
        if span < 1 or n < 2 * (span + k) + 1:
            return
        rng = range(-span - k, span + k + 1)
        idxs = np.array([(center_idx + o) % n for o in rng])
        kernel = np.ones(2 * k + 1) / (2 * k + 1)
        xs = np.convolve(self.wpts[idxs, 1], kernel, mode='valid')  # len 2*span+1
        ys = np.convolve(self.wpts[idxs, 2], kernel, mode='valid')
        for m in range(2 * span + 1):
            off = m - span
            w = 0.5 * (1.0 + math.cos(math.pi * off / span))  # 중심 1 -> 경계 0
            i = (center_idx + off) % n
            self.wpts[i, 1] = (1.0 - w) * self.wpts[i, 1] + w * xs[m]
            self.wpts[i, 2] = (1.0 - w) * self.wpts[i, 2] + w * ys[m]

    def _snapshot(self):
        self.undo_stack.append(self.wpts.copy())
        if len(self.undo_stack) > 5:
            self.undo_stack.pop(0)

    # -------------------------------------------------------- colormap
    def _speed_color(self, v, alpha=1.0):
        """속도 -> 파랑(느림) → 청록 → 노랑 → 빨강(빠름).

        핸들 / 속도곡선 / 바닥 라인이 모두 이 함수를 쓰므로 세 표현의 색이
        항상 일치한다. 범위는 현재 프로파일의 min~max (대비를 최대로).
        """
        lo, hi = self._v_range
        t = float(np.clip((float(v) - lo) / (hi - lo), 0.0, 1.0))
        # 파랑(0,0.35,1) -> 청록(0,0.9,0.8) -> 노랑(1,0.9,0) -> 빨강(1,0.15,0)
        stops = [(0.00, (0.00, 0.35, 1.00)),
                 (0.40, (0.00, 0.90, 0.80)),
                 (0.72, (1.00, 0.90, 0.00)),
                 (1.00, (1.00, 0.15, 0.00))]
        for (t0, c0), (t1, c1) in zip(stops, stops[1:]):
            if t <= t1:
                f = (t - t0) / (t1 - t0)
                rgb = [a + (b - a) * f for a, b in zip(c0, c1)]
                break
        else:
            rgb = stops[-1][1]
        return ColorRGBA(r=float(rgb[0]), g=float(rgb[1]), b=float(rgb[2]), a=float(alpha))

    @property
    def _v_range(self):
        v = self.wpts[:, 7]
        lo = float(v.min())
        return lo, max(float(v.max()), lo + 0.1)

    # ----------------------------------------------------- marker setup
    def _make_all_markers(self):
        for idx in self.anchor_idxs:
            self._make_pos_marker(idx)
            self._make_vel_marker(idx)

    def _plane_orientation(self, im_control):
        # XY 평면 이동/Z축 이동용 orientation (RViz 관례: +Y 축 쿼터니언 -> Z normal)
        im_control.orientation.w = 1.0
        im_control.orientation.y = 1.0

    def _make_pos_marker(self, idx):
        im = InteractiveMarker()
        im.header.frame_id = 'map'
        im.name = f'pos_{idx}'
        im.description = ''
        im.pose.position.x = self.wpts[idx, 1]
        im.pose.position.y = self.wpts[idx, 2]
        im.pose.position.z = 0.03
        im.scale = 0.8

        sphere = Marker()
        sphere.type = Marker.SPHERE
        sphere.scale.x = sphere.scale.y = sphere.scale.z = 0.22
        sphere.color.r, sphere.color.b, sphere.color.a = 0.7, 0.9, 0.95  # 보라

        ctrl = InteractiveMarkerControl()
        ctrl.interaction_mode = InteractiveMarkerControl.MOVE_PLANE
        self._plane_orientation(ctrl)
        ctrl.always_visible = True
        ctrl.markers.append(sphere)
        im.controls.append(ctrl)

        self.server.insert(im, feedback_callback=self.on_pos_feedback)
        self.menu.apply(self.server, im.name)

    def _make_vel_marker(self, idx):
        v = self.wpts[idx, 7]
        im = InteractiveMarker()
        im.header.frame_id = 'map'
        im.name = f'vel_{idx}'
        im.description = ''
        im.pose.position.x = self.wpts[idx, 1]
        im.pose.position.y = self.wpts[idx, 2]
        im.pose.position.z = v * self.z_scale
        im.scale = 0.6

        box = Marker()
        box.type = Marker.CUBE
        box.scale.x = box.scale.y = 0.15
        box.scale.z = 0.1
        box.color = self._speed_color(v, alpha=0.95)   # 속도별 색 (곡선/바닥선과 동일)

        ctrl = InteractiveMarkerControl()
        ctrl.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
        self._plane_orientation(ctrl)
        ctrl.always_visible = True
        ctrl.markers.append(box)
        im.controls.append(ctrl)

        self.server.insert(im, feedback_callback=self.on_vel_feedback)
        self.menu.apply(self.server, im.name)

    def _refresh_markers_near(self, center_idx, win=None):
        """창 안의 핸들 위치를 갱신 (드래그 결과 반영). win 생략 시 falloff 창 폭."""
        n = len(self.wpts)
        if win is None:
            win = int(self.window_m / self.ds)
        for idx in self.anchor_idxs:
            off = (idx - center_idx) % n
            if min(off, n - off) > win and center_idx >= 0:
                continue
            self._reset_marker_pose(idx)
        self.server.applyChanges()

    def _reset_marker_pose(self, idx):
        im_pose_z = 0.03
        pose = self._make_pose(self.wpts[idx, 1], self.wpts[idx, 2], im_pose_z)
        self.server.setPose(f'pos_{idx}', pose)
        # 속도 핸들은 위치(높이)뿐 아니라 '색'도 속도에 따라 바뀌므로 setPose 로는
        # 부족하다 — 다시 만들어 덮어쓴다 (insert 는 같은 이름이면 교체).
        self._make_vel_marker(idx)

    @staticmethod
    def _make_pose(x, y, z):
        from geometry_msgs.msg import Pose
        p = Pose()
        p.position.x, p.position.y, p.position.z = float(x), float(y), float(z)
        p.orientation.w = 1.0
        return p

    def _refresh_all_markers(self):
        for idx in self.anchor_idxs:
            self._reset_marker_pose(idx)
        self.server.applyChanges()

    # ------------------------------------------------------- callbacks
    def on_pos_feedback(self, fb: InteractiveMarkerFeedback):
        if fb.event_type != InteractiveMarkerFeedback.MOUSE_UP:
            return
        idx = int(fb.marker_name.split('_')[1])
        dx = fb.pose.position.x - self.wpts[idx, 1]
        dy = fb.pose.position.y - self.wpts[idx, 2]
        if not math.isfinite(dx) or not math.isfinite(dy):
            self.get_logger().error('비유한 position feedback 거부')
            self._reset_marker_pose(idx)
            self.server.applyChanges()
            return
        if math.hypot(dx, dy) < 0.01:
            return
        corridor_ok, reason = self._recompute_wall_distances_from_map()
        if not corridor_ok:
            self.get_logger().error(
                f'라인 이동 거부: 최신 map corridor를 계산할 수 없음: {reason}')
            self._reset_marker_pose(idx)
            self.server.applyChanges()
            return
        # 횡방향(법선) 성분만 사용 — 간격 유지
        psi = self.wpts[idx, 5]
        nx, ny = -math.sin(psi), math.cos(psi)
        delta_d = dx * nx + dy * ny

        self._snapshot()
        old_xy = self.wpts[:, 1:3].copy()
        old_psi = self.wpts[:, 5].copy()
        old_right = self.wpts[:, 3].copy()
        old_left = self.wpts[:, 4].copy()

        def apply(i, dd):
            # 트랙 경계 클램프: d_left/d_right는 현재 라인 기준 남은 폭
            hi = old_left[i] - self.min_wall   # 왼쪽 여유
            lo = -(old_right[i] - self.min_wall)
            dd = float(np.clip(dd, min(lo, 0.0), max(hi, 0.0)))
            psi_i = old_psi[i]
            self.wpts[i, 1] += dd * -math.sin(psi_i)
            self.wpts[i, 2] += dd * math.cos(psi_i)

        self._falloff_apply(idx, delta_d, apply)
        self._smooth_xy_window(idx)      # 점별 이동으로 생긴 요철 제거 (찌그러짐 방지)
        # 스무딩이 만든 추가 변위까지 원래 법선으로 다시 투영하고 corridor 안으로
        # 클램프한다. 그렇지 않으면 xy와 d_left/d_right가 서로 다른 경로를 설명한다.
        normal = np.column_stack([-np.sin(old_psi), np.cos(old_psi)])
        actual_d = np.sum((self.wpts[:, 1:3] - old_xy) * normal, axis=1)
        lo = np.minimum(-(old_right - self.min_wall), 0.0)
        hi = np.maximum(old_left - self.min_wall, 0.0)
        actual_d = np.clip(actual_d, lo, hi)
        self.wpts[:, 1:3] = old_xy + actual_d[:, None] * normal
        self._recompute_geometry()
        corridor_ok, reason = self._recompute_wall_distances_from_map()
        if not corridor_ok:
            # occupied/unknown으로 중심선이 넘어간 편집은 즉시 원복한다.
            self.wpts = self.undo_stack.pop()
            self._refresh_all_markers()
            self._publish_preview()
            self.get_logger().error(f'라인 이동 거부 및 원복: {reason}')
            return
        # 라인이 바뀌면 곡률도 바뀌므로 속도를 반드시 같이 재계산한다.
        # (이전에는 여기서 vx_mps 가 그대로 남아, 라인을 안쪽으로 좁혀도 예전
        #  넓은 라인 기준 속도로 진입해 바깥쪽 벽과 충돌하는 문제가 있었다.)
        # 단, 트랙 전체를 다시 풀면 다른 코너의 수동 속도 튜닝이 매번 사라지므로
        # 곡률이 바뀐 근방 + 제동거리 여유만 국소적으로 재계산한다.
        try:
            half = self._recompute_velocity_window(idx)
            self._enforce_velocity_feasibility()
        except (RuntimeError, ValueError) as exc:
            self.wpts = self.undo_stack.pop()
            self._refresh_all_markers()
            self._publish_preview()
            self.get_logger().error(
                f'라인 이동 거부 및 원복: 속도 안전 투영 실패: {exc}')
            return
        self._refresh_markers_near(idx, win=half)
        self._publish_preview()
        self.get_logger().info(
            f'라인 이동: idx {idx}, Δd={delta_d:+.2f} m (윈도우 {self.window_m} m) '
            f'-> 속도 자동 재계산 (min {self.wpts[:,7].min():.1f} / max {self.wpts[:,7].max():.1f} m/s)')

    def on_vel_feedback(self, fb: InteractiveMarkerFeedback):
        if fb.event_type != InteractiveMarkerFeedback.MOUSE_UP:
            return
        idx = int(fb.marker_name.split('_')[1])
        requested_v = fb.pose.position.z / self.z_scale
        if not math.isfinite(requested_v):
            self.get_logger().error('비유한 velocity feedback 거부')
            self._reset_marker_pose(idx)
            self.server.applyChanges()
            return
        new_v = max(requested_v, 0.5)
        delta_v = new_v - self.wpts[idx, 7]
        if abs(delta_v) < 0.05:
            self._reset_marker_pose(idx)
            self.server.applyChanges()
            return
        self._snapshot()

        def apply(i, dv):
            self.wpts[i, 7] = float(np.clip(self.wpts[i, 7] + dv, 0.5, self.v_max))

        self._falloff_apply(idx, delta_v, apply)
        try:
            self._enforce_velocity_feasibility()
        except (RuntimeError, ValueError) as exc:
            self.wpts = self.undo_stack.pop()
            self._refresh_all_markers()
            self._publish_preview()
            self.get_logger().error(
                f'속도 변경 거부 및 원복: 안전 투영 실패: {exc}')
            return
        self._refresh_markers_near(idx)
        self._publish_preview()
        self.get_logger().info(f'속도 조절: idx {idx}, {new_v:.1f} m/s (Δ{delta_v:+.1f})')

    def on_menu(self, fb: InteractiveMarkerFeedback):
        eid = fb.menu_entry_id
        if eid == 1:      # 속도 재계산
            self._snapshot()
            try:
                self._recompute_velocity()
            except (RuntimeError, ValueError) as exc:
                self.wpts = self.undo_stack.pop()
                self.get_logger().error(
                    f'속도 재계산 거부 및 원복: 안전 투영 실패: {exc}')
                self._refresh_all_markers()
                self._publish_preview()
                return
            self.get_logger().info(
                f'속도 재계산: min {self.wpts[:,7].min():.1f} / max {self.wpts[:,7].max():.1f} m/s')
        elif eid == 2:    # undo
            if self.undo_stack:
                self.wpts = self.undo_stack.pop()
                self.get_logger().info(f'실행취소 (남은 undo: {len(self.undo_stack)})')
            else:
                self.get_logger().warn('undo 스택 비어 있음')
        elif eid == 3:    # 리셋
            self._snapshot()
            self.wpts = self.original.copy()
            self.get_logger().info('전체 리셋')
        elif eid == 4:    # 저장
            self._save_csv()
            return
        self._refresh_all_markers()
        self._publish_preview()

    # --------------------------------------------------------- preview
    def _publish_preview(self):
        arr = MarkerArray()
        clear = Marker()
        clear.action = Marker.DELETEALL
        arr.markers.append(clear)

        stamp = self.get_clock().now().to_msg()

        # 1) 바닥 라인 — 주행 라인 자체
        line = Marker()
        line.header.frame_id = 'map'
        line.header.stamp = stamp
        line.ns = 'tuned_line'
        line.id = 0
        line.type = Marker.LINE_STRIP
        line.scale.x = 0.07
        line.pose.orientation.w = 1.0
        for r in self.wpts[::2]:
            line.points.append(Point(x=r[1], y=r[2], z=0.02))
            line.colors.append(self._speed_color(r[7]))
        # 폐곡선
        line.points.append(Point(x=self.wpts[0, 1], y=self.wpts[0, 2], z=0.02))
        line.colors.append(line.colors[0])
        arr.markers.append(line)

        # 2) 속도 곡선 — 핸들(초록 큐브)을 잇는 선. 앵커끼리 직선으로 잇지 않고
        #    전 wpt(0.1 m)를 지나가므로 falloff 로 만든 속도 변화가 그대로 부드러운
        #    곡선으로 보인다. 높이 = 속도 * z_scale 로 핸들과 정확히 같은 높이.
        curve = Marker()
        curve.header.frame_id = 'map'
        curve.header.stamp = stamp
        curve.ns = 'vel_curve'
        curve.id = 0
        curve.type = Marker.LINE_STRIP
        curve.scale.x = 0.05
        curve.pose.orientation.w = 1.0
        for r in self.wpts:
            curve.points.append(Point(x=r[1], y=r[2], z=r[7] * self.z_scale))
            curve.colors.append(self._speed_color(r[7]))
        curve.points.append(Point(x=self.wpts[0, 1], y=self.wpts[0, 2],
                                  z=self.wpts[0, 7] * self.z_scale))
        curve.colors.append(curve.colors[0])
        arr.markers.append(curve)

        # 3) 수직 눈금 — 곡선 높이가 어느 지점의 속도인지 읽히도록 바닥과 연결
        stems = Marker()
        stems.header.frame_id = 'map'
        stems.header.stamp = stamp
        stems.ns = 'vel_stems'
        stems.id = 0
        stems.type = Marker.LINE_LIST
        stems.scale.x = 0.015
        stems.pose.orientation.w = 1.0
        for r in self.wpts[::5]:
            c = self._speed_color(r[7], alpha=0.35)
            stems.points.append(Point(x=r[1], y=r[2], z=0.02))
            stems.points.append(Point(x=r[1], y=r[2], z=r[7] * self.z_scale))
            stems.colors.extend([c, c])
        arr.markers.append(stems)

        # 앵커마다 속도 텍스트
        for k, idx in enumerate(self.anchor_idxs[::2]):
            t = Marker()
            t.header.frame_id = 'map'
            t.ns = 'vel_text'
            t.id = k + 1
            t.type = Marker.TEXT_VIEW_FACING
            t.scale.z = 0.3
            t.color.r = t.color.g = t.color.b = t.color.a = 1.0
            t.pose.position.x = self.wpts[idx, 1]
            t.pose.position.y = self.wpts[idx, 2]
            t.pose.position.z = self.wpts[idx, 7] * self.z_scale + 0.3
            t.text = f'{self.wpts[idx, 7]:.1f}'
            arr.markers.append(t)
        self.pub_preview.publish(arr)


def main():
    rclpy.init()
    rclpy.spin(RacelineTunerNode())
    rclpy.shutdown()


if __name__ == '__main__':
    main()
