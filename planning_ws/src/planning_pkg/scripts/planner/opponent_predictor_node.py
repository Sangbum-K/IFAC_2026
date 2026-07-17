#!/usr/bin/env python3
"""Predictive Spliner의 상대 궤적 예측 노드 (arXiv:2410.04868 구현).

동적 상대를 뒤따르는 동안 (s, d)와 (s, vs) 관측을 수집하고, GP 회귀로
상대의 주행 라인 d(s)와 속도 프로파일 vs(s)를 학습한다. 자차와 상대의
속도 프로파일을 앞으로 적분해 "따라잡는 시점"의 충돌 구간 [s_start, s_end]를
계산하고, 그 위치에 가상 장애물을 만들어 /opponent_prediction/obstacles로
발행한다. spliner_node는 (spliner/use_prediction=true일 때) 같은 id의 원시
관측 대신 이 예측 장애물 주위로 회피 스플라인을 생성한다 → 등속 전파보다
빠른 상대도 안전하게 추월 가능.

의존: numpy만 사용 (GP는 RBF 커널 + Cholesky 직접 구현, 학습점 <=150이라 ms급).
시뮬(virtual_opponent)과 실차(perception) 모두 동일 인터페이스로 동작한다.
"""
import math

import numpy as np
import rclpy
from rclpy.node import Node

from nav_msgs.msg import Odometry
from std_msgs.msg import ColorRGBA
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker, MarkerArray

from ae_hyu_msgs.msg import Obstacle, ObstacleArray, WpntArray

import pitwall


class GP1D:
    """s -> y 1차원 GP 회귀 (RBF 커널). 트랙 s 경계는 데이터 wrap 증강으로 처리."""

    def __init__(self, length_scale, sigma_f, sigma_n, track_len):
        self.ls = length_scale
        self.sf2 = sigma_f ** 2
        self.sn2 = sigma_n ** 2
        self.L = track_len
        self.x = None

    def _wrap_augment(self, x, y):
        # 경계 근처(2*ls) 데이터를 반대편으로 복제해 주기성 근사
        pad = 2.0 * self.ls
        lo = x < pad
        hi = x > self.L - pad
        return (np.concatenate([x, x[lo] + self.L, x[hi] - self.L]),
                np.concatenate([y, y[lo], y[hi]]))

    def fit(self, x, y):
        x, y = self._wrap_augment(np.asarray(x), np.asarray(y))
        if len(x) > 150:                      # 균일 서브샘플로 계산량 제한
            idx = np.linspace(0, len(x) - 1, 150).astype(int)
            x, y = x[idx], y[idx]
        self.x = x
        self.y_mean = float(np.mean(y))
        d = x[:, None] - x[None, :]
        K = self.sf2 * np.exp(-0.5 * (d / self.ls) ** 2) + self.sn2 * np.eye(len(x))
        self.Lchol = np.linalg.cholesky(K)
        self.alpha = np.linalg.solve(
            self.Lchol.T, np.linalg.solve(self.Lchol, y - self.y_mean))

    def predict(self, xq):
        # 학습 데이터가 wrap 증강돼 있어 [0, L) 질의는 직접 거리로 충분
        xq = np.atleast_1d(np.asarray(xq, dtype=float))
        d = xq[:, None] - self.x[None, :]
        k = self.sf2 * np.exp(-0.5 * (d / self.ls) ** 2)
        return self.y_mean + k @ self.alpha


class OpponentPredictorNode(Node):
    def __init__(self):
        super().__init__('opponent_predictor_node')
        pitwall.init(self)

        self.declare_parameter('predictor/min_samples', 40)      # GP 학습 최소 관측 수
        self.declare_parameter('predictor/sample_min_ds', 0.15)  # 관측 간 최소 s 간격 [m]
        self.declare_parameter('predictor/max_samples', 600)
        self.declare_parameter('predictor/gp_length_scale', 3.0) # [m]
        self.declare_parameter('predictor/gp_sigma_n', 0.08)
        self.declare_parameter('predictor/refit_every', 20)      # N 관측마다 재학습
        self.declare_parameter('predictor/horizon_sec', 8.0)     # 따라잡기 탐색 시간
        self.declare_parameter('predictor/catch_gap_m', 0.8)     # 이 간격이면 충돌 임박
        self.declare_parameter('predictor/pass_length_m', 1.2)   # 추월에 필요한 상대 s 여유
        self.declare_parameter('predictor/publish_rate_hz', 10.0)
        gp = lambda n: self.get_parameter(n).value
        self.min_samples = int(gp('predictor/min_samples'))
        self.sample_min_ds = gp('predictor/sample_min_ds')
        self.max_samples = int(gp('predictor/max_samples'))
        self.gp_ls = gp('predictor/gp_length_scale')
        self.gp_sn = gp('predictor/gp_sigma_n')
        self.refit_every = int(gp('predictor/refit_every'))
        self.horizon = gp('predictor/horizon_sec')
        self.catch_gap = gp('predictor/catch_gap_m')
        self.pass_len = gp('predictor/pass_length_m')

        self.track_len = 0.0
        self.gb = None                 # [s, vx]
        self.ds = 0.1
        self.ego_s = 0.0
        self.ego_vs = 0.0
        # 상대별 관측 버퍼: id -> dict(s[], d[], vs[], last_s, size, n_since_fit)
        self.buf = {}
        self.gp_d = {}
        self.gp_v = {}

        self.create_subscription(WpntArray, '/global_waypoints', self.cb_gb, 10)
        self.create_subscription(Odometry, '/car_state/frenet/odom', self.cb_odom, 10)
        self.create_subscription(ObstacleArray, '/perception/obstacles', self.cb_obs, 10)
        self.pub_pred = self.create_publisher(
            ObstacleArray, '/opponent_prediction/obstacles', 10)
        self.pub_mrk = self.create_publisher(
            MarkerArray, '/opponent_prediction/markers', 10)
        self.create_timer(1.0 / gp('predictor/publish_rate_hz'), self.predict_and_publish)

        self.get_logger().info('Opponent predictor (Predictive Spliner) ready')

    def cb_gb(self, msg: WpntArray):
        # [s, vx, x, y, psi]
        self.gb = np.array(
            [[w.s_m, w.vx_mps, w.x_m, w.y_m, w.psi_rad] for w in msg.wpnts])
        self.ds = self.gb[1, 0] - self.gb[0, 0]
        self.track_len = self.gb[-1, 0] + self.ds

    def cb_odom(self, msg: Odometry):
        self.ego_s = msg.pose.pose.position.x
        self.ego_vs = msg.twist.twist.linear.x

    def cb_obs(self, msg: ObstacleArray):
        if self.track_len <= 0.0:
            return
        for o in msg.obstacles:
            if o.is_static or not o.is_visible:
                continue
            b = self.buf.setdefault(o.id, {
                'S': [], 'D': [], 'V': [], 'last_s': None,
                'size': o.size, 'n_new': 0})
            if b['last_s'] is not None:
                ds = (o.s_center - b['last_s']) % self.track_len
                if ds < self.sample_min_ds:
                    continue
            b['S'].append(o.s_center)
            b['D'].append(o.d_center)
            b['V'].append(max(o.vs, 0.1))
            b['last_s'] = o.s_center
            b['n_new'] += 1
            if len(b['S']) > self.max_samples:
                b['S'] = b['S'][-self.max_samples:]
                b['D'] = b['D'][-self.max_samples:]
                b['V'] = b['V'][-self.max_samples:]
            # 주기적 GP 재학습
            if len(b['S']) >= self.min_samples and b['n_new'] >= self.refit_every:
                self._fit(o.id, b)
                b['n_new'] = 0

    def _fit(self, oid, b):
        gpd = GP1D(self.gp_ls, sigma_f=0.5, sigma_n=self.gp_sn, track_len=self.track_len)
        gpd.fit(b['S'], b['D'])
        gpv = GP1D(self.gp_ls, sigma_f=2.0, sigma_n=0.2, track_len=self.track_len)
        gpv.fit(b['S'], b['V'])
        first_fit = oid not in self.gp_d
        self.gp_d[oid] = gpd
        self.gp_v[oid] = gpv
        if first_fit:
            pitwall.event(f'GP 학습 완료 (opp {oid}, {len(b["S"])} 관측)')
        self.get_logger().info(
            f'GP 재학습: opp {oid}, {len(b["S"])} 관측 '
            f'(트랙 커버 {len(b["S"])*self.sample_min_ds/self.track_len*100:.0f}%+)',
            throttle_duration_sec=5.0)

    # ------------------------------------------------------------ 예측
    def _raceline_v(self, s):
        return float(self.gb[int(s / self.ds) % len(self.gb), 1])

    def predict_and_publish(self):
        arr = ObstacleArray()
        arr.header.stamp = self.get_clock().now().to_msg()
        arr.header.frame_id = 'map'
        mrks = MarkerArray()
        clear = Marker()
        clear.action = Marker.DELETEALL
        mrks.markers.append(clear)

        if self.gb is not None:
            for oid, b in self.buf.items():
                if b['last_s'] is not None:
                    pitwall.log('opp_gap',
                                (b['last_s'] - self.ego_s) % self.track_len)
                if oid not in self.gp_v or b['last_s'] is None:
                    continue
                pred = self._predict_catch(oid, b)
                if pred is not None:
                    arr.obstacles.append(pred)
                    self._make_markers(mrks, oid, pred)
                    pitwall.log('pred_catch_s', pred.s_center)

        pred_active = len(arr.obstacles) > 0
        if pred_active and not getattr(self, '_pred_was_active', False):
            pitwall.event(f'예측 발동: 충돌구간 s={arr.obstacles[0].s_center:.1f}')
        self._pred_was_active = pred_active

        self.pub_pred.publish(arr)
        self.pub_mrk.publish(mrks)

    def _predict_catch(self, oid, b):
        """자차/상대 속도 프로파일 전방 적분 -> 따라잡는 s 구간 계산."""
        L = self.track_len
        gpv, gpd = self.gp_v[oid], self.gp_d[oid]
        s_e, s_o = self.ego_s, float(b['last_s'])
        # 자차 속도 = raceline 프로파일 * 현재 스케일 (컨트롤러 speed_scale 반영)
        ref_v = max(self._raceline_v(s_e), 0.5)
        ego_scale = float(np.clip(max(self.ego_vs, 0.3) / ref_v, 0.2, 1.2))

        dt = 0.1
        t_catch = None
        for _ in range(int(self.horizon / dt)):
            gap = (s_o - s_e) % L
            if gap < self.catch_gap:
                t_catch = True
                break
            s_e = (s_e + max(self._raceline_v(s_e) * ego_scale, 0.3) * dt) % L
            s_o = (s_o + max(float(gpv.predict(s_o)[0]), 0.1) * dt) % L
        if t_catch is None:
            return None      # 호라이즌 내 추월 상황 아님

        # 충돌 구간: 따라잡는 지점부터 상대가 pass_length만큼 더 가는 구간
        s_c = s_o
        v_o = max(float(gpv.predict(s_c)[0]), 0.1)
        v_e = max(self._raceline_v(s_c) * ego_scale, 0.3)
        rel = max(v_e - v_o, 0.3)
        # 자차가 상대를 완전히 지나치는 데 걸리는 시간 동안 상대의 이동량
        t_pass = (self.pass_len + b['size']) / rel
        s_end = (s_c + v_o * t_pass + b['size'] / 2) % L
        s_start = (s_c - b['size'] / 2 - 0.3) % L
        d_c = float(gpd.predict((s_c + (s_end - s_c) % L / 2) % L)[0])
        half = b['size'] / 2

        o = Obstacle()
        o.id = oid
        o.s_start = s_start
        o.s_end = s_end
        o.s_center = (s_start + ((s_end - s_start) % L) / 2) % L
        o.d_center = d_c
        o.d_left = d_c + half
        o.d_right = d_c - half
        o.vs = v_o
        o.size = b['size']
        o.is_static = True        # 이미 미래 위치로 전파됨 -> spliner가 재전파 안 함
        o.is_visible = True
        return o

    def _frenet_to_xy(self, s, d):
        i = int(s / self.ds) % len(self.gb)
        _, _, x, y, psi = self.gb[i]
        return x - d * math.sin(psi), y + d * math.cos(psi)

    def _make_markers(self, mrks, oid, pred):
        """예측 충돌 구간(빨간 밴드) + 학습된 상대 라인(주황 선)."""
        L = self.track_len
        gpd = self.gp_d[oid]

        # 충돌 구간 밴드 (예측된 상대 라인 위)
        band = Marker()
        band.header.frame_id = 'map'
        band.ns = f'pred_zone_{oid}'
        band.id = 0
        band.type = Marker.LINE_STRIP
        band.scale.x = 0.3
        band.pose.orientation.w = 1.0
        band.color = ColorRGBA(r=1.0, g=0.1, b=0.1, a=0.8)
        span = (pred.s_end - pred.s_start) % L
        for k in np.arange(0.0, span + 0.1, 0.2):
            s = (pred.s_start + k) % L
            d = float(gpd.predict(s)[0])
            x, y = self._frenet_to_xy(s, d)
            band.points.append(Point(x=x, y=y, z=0.06))
        if band.points:
            mrks.markers.append(band)

        # 학습된 상대 주행 라인 (GP 평균, 전방 20m)
        line = Marker()
        line.header.frame_id = 'map'
        line.ns = f'pred_line_{oid}'
        line.id = 1
        line.type = Marker.LINE_STRIP
        line.scale.x = 0.05
        line.pose.orientation.w = 1.0
        line.color = ColorRGBA(r=1.0, g=0.6, b=0.0, a=0.8)
        for k in np.arange(0.0, 20.0, 0.5):
            s = (self.ego_s + k) % L
            d = float(gpd.predict(s)[0])
            x, y = self._frenet_to_xy(s, d)
            line.points.append(Point(x=x, y=y, z=0.04))
        mrks.markers.append(line)


def main():
    rclpy.init()
    rclpy.spin(OpponentPredictorNode())
    rclpy.shutdown()


if __name__ == '__main__':
    main()
