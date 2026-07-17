#!/usr/bin/env python3
"""RViz 인터랙티브 레이스라인 튜너 (HMCL unicorn raceline_tuner의 ROS2 구현).

코너별로 라인과 속도를 RViz에서 직접 조절한다:
  - 보라 구체(2.5m 간격)를 드래그 -> 주변 라인이 코사인 falloff로 함께 이동
    (횡방향 성분만 적용, 트랙 경계-safe margin 안으로 자동 클램프)
  - 초록 수직 핸들을 위아래로 드래그 -> 그 구간 속도 조절 (높이 = 속도)
  - 아무 핸들이나 우클릭 메뉴:
      속도 재계산(마찰한계) / 실행취소 / 전체 리셋 / CSV 저장
  - 저장 시 원본은 .bak으로 백업하고 같은 9열 CSV 포맷으로 기록.
    저장 후 colcon build 해야 런타임 노드가 새 라인을 읽는다.

실행: ros2 launch planning_pkg raceline_tuner.launch.py
"""
import copy
import math
import shutil
from pathlib import Path

import numpy as np
import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Point
from interactive_markers.interactive_marker_server import InteractiveMarkerServer
from interactive_markers.menu_handler import MenuHandler
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
        self.declare_parameter('tuner/min_wall_dist_m', 0.35) # 라인-벽 최소 여유
        self.declare_parameter('tuner/v_max_mps', 12.0)
        self.declare_parameter('tuner/mu', 0.85)
        self.declare_parameter('tuner/a_acc_max', 5.5)
        self.declare_parameter('tuner/a_dec_max', 7.0)
        self.declare_parameter('tuner/vel_handle_scale', 0.3) # 높이 = v * scale [m]

        gp = lambda n: self.get_parameter(n).value
        self.csv_path = Path(gp('tuner/raceline_csv'))
        out = gp('tuner/output_csv')
        self.out_path = Path(out) if out else self.csv_path
        self.stride = int(gp('tuner/handle_stride'))
        self.window_m = gp('tuner/falloff_window_m')
        self.min_wall = gp('tuner/min_wall_dist_m')
        self.v_max = gp('tuner/v_max_mps')
        self.mu = gp('tuner/mu')
        self.a_acc = gp('tuner/a_acc_max')
        self.a_dec = gp('tuner/a_dec_max')
        self.z_scale = gp('tuner/vel_handle_scale')

        # CSV 로드: s,x,y,d_right,d_left,psi,kappa,vx,ax
        self.wpts = self._load_csv(self.csv_path)
        self.original = self.wpts.copy()
        self.undo_stack = []           # 최근 5개 스냅샷
        self.backed_up = False

        n = len(self.wpts)
        self.ds = float(np.mean(np.diff(self.wpts[:, 0])))
        self.get_logger().info(
            f'raceline 로드: {n} wpts, {self.wpts[-1,0]:.1f} m ({self.csv_path.name})')

        self.pub_preview = self.create_publisher(MarkerArray, '/raceline_tuner/preview', 10)

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
    def _load_csv(self, path):
        rows = []
        with open(path) as f:
            for line in f:
                if line.startswith('#') or not line.strip():
                    continue
                rows.append([float(v) for v in line.split(';')])
        a = np.array(rows)
        if a.shape[1] != 9:
            raise RuntimeError(f'CSV 9열 아님: {a.shape}')
        return a

    def _save_csv(self):
        if not self.backed_up and self.out_path == self.csv_path:
            shutil.copy(self.csv_path, str(self.csv_path) + '.bak')
            self.backed_up = True
            self.get_logger().info(f'원본 백업: {self.csv_path}.bak')
        header = ('# tuned by raceline_tuner_node\n'
                  '# s_m; x_m; y_m; d_right; d_left; psi_rad; kappa_radpm; vx_mps; ax_mps2\n')
        with open(self.out_path, 'w') as f:
            f.write(header)
            for r in self.wpts:
                f.write('; '.join(f'{v:.7f}' for v in r) + '\n')
        self.get_logger().info(
            f'저장 완료: {self.out_path}\n'
            '  -> 런타임 반영: cd ~/IFAC_ws/planning_ws && colcon build --packages-select planning_pkg')

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
        """마찰한계 + 가감속 한계 forward-backward 속도 프로파일."""
        kappa = np.abs(self.wpts[:, 6])
        # 곡률 노이즈 완화 (이동평균 1m)
        k = 5
        kernel = np.ones(2 * k + 1) / (2 * k + 1)
        kappa = np.convolve(np.concatenate([kappa[-k:], kappa, kappa[:k]]),
                            kernel, mode='valid')
        v = np.minimum(np.sqrt(self.mu * G / np.maximum(kappa, 1e-9)), self.v_max)
        seg = np.maximum(np.diff(self.wpts[:, 0], append=self.wpts[-1, 0] + self.ds), 1e-6)
        n = len(v)
        for _ in range(3):
            for i in range(n):          # forward (가속 한계)
                j = (i + 1) % n
                v[j] = min(v[j], math.sqrt(v[i] ** 2 + 2 * self.a_acc * seg[i]))
            for i in range(n - 1, -1, -1):  # backward (감속 한계)
                j = (i + 1) % n
                v[i] = min(v[i], math.sqrt(v[j] ** 2 + 2 * self.a_dec * seg[i]))
        self.wpts[:, 7] = v
        # ax 프로파일도 갱신
        vn = np.roll(v, -1)
        self.wpts[:, 8] = (vn ** 2 - v ** 2) / (2 * seg)

    def _falloff_apply(self, center_idx, delta, col_apply):
        """center 주변 window에 코사인 falloff로 delta 적용. col_apply(i, w*delta)."""
        n = len(self.wpts)
        win = int(self.window_m / self.ds)
        for off in range(-win, win + 1):
            i = (center_idx + off) % n
            w = 0.5 * (1.0 + math.cos(math.pi * off / win)) if win > 0 else 1.0
            col_apply(i, delta * w)

    def _snapshot(self):
        self.undo_stack.append(self.wpts.copy())
        if len(self.undo_stack) > 5:
            self.undo_stack.pop(0)

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
        box.color.g, box.color.a = 0.9, 0.95  # 초록

        ctrl = InteractiveMarkerControl()
        ctrl.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
        self._plane_orientation(ctrl)
        ctrl.always_visible = True
        ctrl.markers.append(box)
        im.controls.append(ctrl)

        self.server.insert(im, feedback_callback=self.on_vel_feedback)
        self.menu.apply(self.server, im.name)

    def _refresh_markers_near(self, center_idx):
        """falloff 창 안의 핸들 위치를 갱신 (드래그 결과 반영)."""
        n = len(self.wpts)
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
        pose_v = self._make_pose(self.wpts[idx, 1], self.wpts[idx, 2],
                                 self.wpts[idx, 7] * self.z_scale)
        self.server.setPose(f'vel_{idx}', pose_v)

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
        if math.hypot(dx, dy) < 0.01:
            return
        # 횡방향(법선) 성분만 사용 — 간격 유지
        psi = self.wpts[idx, 5]
        nx, ny = -math.sin(psi), math.cos(psi)
        delta_d = dx * nx + dy * ny

        self._snapshot()

        def apply(i, dd):
            # 트랙 경계 클램프: d_left/d_right는 현재 라인 기준 남은 폭
            hi = self.wpts[i, 4] - self.min_wall   # 왼쪽 여유
            lo = -(self.wpts[i, 3] - self.min_wall)
            dd = float(np.clip(dd, min(lo, 0.0), max(hi, 0.0)))
            psi_i = self.wpts[i, 5]
            self.wpts[i, 1] += dd * -math.sin(psi_i)
            self.wpts[i, 2] += dd * math.cos(psi_i)
            self.wpts[i, 4] -= dd   # d_left
            self.wpts[i, 3] += dd   # d_right

        self._falloff_apply(idx, delta_d, apply)
        self._recompute_geometry()
        self._refresh_markers_near(idx)
        self._publish_preview()
        self.get_logger().info(f'라인 이동: idx {idx}, Δd={delta_d:+.2f} m (윈도우 {self.window_m} m)')

    def on_vel_feedback(self, fb: InteractiveMarkerFeedback):
        if fb.event_type != InteractiveMarkerFeedback.MOUSE_UP:
            return
        idx = int(fb.marker_name.split('_')[1])
        new_v = max(fb.pose.position.z / self.z_scale, 0.5)
        delta_v = new_v - self.wpts[idx, 7]
        if abs(delta_v) < 0.05:
            self._reset_marker_pose(idx)
            self.server.applyChanges()
            return
        self._snapshot()

        def apply(i, dv):
            self.wpts[i, 7] = float(np.clip(self.wpts[i, 7] + dv, 0.5, self.v_max))

        self._falloff_apply(idx, delta_v, apply)
        self._refresh_markers_near(idx)
        self._publish_preview()
        self.get_logger().info(f'속도 조절: idx {idx}, {new_v:.1f} m/s (Δ{delta_v:+.1f})')

    def on_menu(self, fb: InteractiveMarkerFeedback):
        eid = fb.menu_entry_id
        if eid == 1:      # 속도 재계산
            self._snapshot()
            self._recompute_velocity()
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

        line = Marker()
        line.header.frame_id = 'map'
        line.header.stamp = self.get_clock().now().to_msg()
        line.ns = 'tuned_line'
        line.id = 0
        line.type = Marker.LINE_STRIP
        line.scale.x = 0.07
        line.pose.orientation.w = 1.0
        v = self.wpts[:, 7]
        v_lo, v_hi = v.min(), max(v.max(), v.min() + 0.1)
        from std_msgs.msg import ColorRGBA
        for r in self.wpts[::2]:
            p = Point(x=r[1], y=r[2], z=0.02)
            line.points.append(p)
            t = (r[7] - v_lo) / (v_hi - v_lo)   # 파랑(느림) -> 빨강(빠름)
            line.colors.append(ColorRGBA(r=float(t), g=0.2, b=float(1 - t), a=1.0))
        # 폐곡선
        line.points.append(Point(x=self.wpts[0, 1], y=self.wpts[0, 2], z=0.02))
        line.colors.append(line.colors[0])
        arr.markers.append(line)

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
