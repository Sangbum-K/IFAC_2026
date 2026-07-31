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
import time

import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from rclpy.time import Time

from nav_msgs.msg import Odometry
from std_msgs.msg import ColorRGBA
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker, MarkerArray

from f1tenth_planning_msgs.msg import Obstacle, ObstacleArray, WpntArray

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

    def predict_stats(self, xq):
        """Posterior mean and point-wise standard deviation."""
        xq = np.atleast_1d(np.asarray(xq, dtype=float))
        d = xq[:, None] - self.x[None, :]
        k = self.sf2 * np.exp(-0.5 * (d / self.ls) ** 2)
        mean = self.y_mean + k @ self.alpha
        solved = np.linalg.solve(self.Lchol, k.T)
        var = np.maximum(self.sf2 - np.sum(solved * solved, axis=0), self.sn2)
        return mean, np.sqrt(var)


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
        self.declare_parameter('predictor/obstacle_topic', '/perception/obstacles')
        self.declare_parameter('predictor/odom_timeout_sec', 0.20)
        self.declare_parameter('predictor/obstacle_timeout_sec', 0.30)
        self.declare_parameter('predictor/require_source_stamp', True)
        self.declare_parameter('predictor/source_future_tolerance_sec', 0.05)
        self.declare_parameter('predictor/uncertainty_sigma', 3.0)
        self.declare_parameter('predictor/min_pass_relative_speed_mps', 0.5)
        self.declare_parameter('predictor/max_prediction_span_ratio', 0.45)
        self.declare_parameter('predictor/max_fit_ms', 30.0)
        # 정지 상대 가드 (unicorn multi_tracking vs_reset 대응): 최근 실측
        # |vs| 평균이 미만이면 예측 중단 — 파킹한 상대를 GP의 과거 주행
        # 속도로 적분하면 수십 m짜리 유령 충돌구간이 나온다.
        self.declare_parameter('predictor/vs_reset', 0.1)
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
        self.vs_reset = gp('predictor/vs_reset')
        self.odom_timeout = gp('predictor/odom_timeout_sec')
        self.obstacle_timeout = gp('predictor/obstacle_timeout_sec')
        self.require_source_stamp = gp('predictor/require_source_stamp')
        self.source_future_tolerance = gp(
            'predictor/source_future_tolerance_sec')
        self.uncertainty_sigma = gp('predictor/uncertainty_sigma')
        self.min_pass_relative_speed = gp(
            'predictor/min_pass_relative_speed_mps')
        self.max_prediction_span_ratio = gp(
            'predictor/max_prediction_span_ratio')
        self.max_fit_ms = gp('predictor/max_fit_ms')
        self.publish_rate = gp('predictor/publish_rate_hz')

        positive = {
            'min_samples': self.min_samples,
            'max_samples': self.max_samples,
            'sample_min_ds': self.sample_min_ds,
            'gp_length_scale': self.gp_ls,
            'gp_sigma_n': self.gp_sn,
            'refit_every': self.refit_every,
            'horizon': self.horizon,
            'catch_gap': self.catch_gap,
            'pass_length': self.pass_len,
            'vs_reset': self.vs_reset,
            'publish_rate': self.publish_rate,
            'odom_timeout': self.odom_timeout,
            'obstacle_timeout': self.obstacle_timeout,
            'uncertainty_sigma': self.uncertainty_sigma,
            'min_pass_relative_speed': self.min_pass_relative_speed,
            'max_prediction_span_ratio': self.max_prediction_span_ratio,
            'max_fit_ms': self.max_fit_ms,
        }
        invalid = [name for name, value in positive.items()
                   if not math.isfinite(float(value)) or float(value) <= 0.0]
        if invalid or not 0.0 < self.max_prediction_span_ratio < 0.5 or \
                not math.isfinite(self.source_future_tolerance) or \
                self.source_future_tolerance < 0.0:
            raise RuntimeError(f'invalid predictor safety parameters: {invalid}')

        self.track_len = 0.0
        self.gb = None                 # [s, vx]
        self.ds = 0.1
        self.ego_s = 0.0
        self.ego_vs = 0.0
        self.ego_stamp = None
        self.gb_stamp = None
        # 상대별 관측 버퍼: id -> dict(s[], d[], vs[], last_s, size, n_since_fit)
        self.buf = {}
        self.gp_d = {}
        self.gp_v = {}

        input_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE)
        self.create_subscription(
            WpntArray, '/global_waypoints', self.cb_gb, input_qos)
        self.create_subscription(
            Odometry, '/car_state/frenet/odom', self.cb_odom, input_qos)
        self.create_subscription(
            ObstacleArray, gp('predictor/obstacle_topic'), self.cb_obs, input_qos)
        self.pub_pred = self.create_publisher(
            ObstacleArray, '/opponent_prediction/obstacles', 10)
        self.pub_mrk = self.create_publisher(
            MarkerArray, '/opponent_prediction/markers', 10)
        self.create_timer(1.0 / self.publish_rate, self.predict_and_publish)

        self.get_logger().info('Opponent predictor (Predictive Spliner) ready')

    def cb_gb(self, msg: WpntArray):
        # [s, vx, x, y, psi]
        if len(msg.wpnts) < 2:
            return
        self.gb = np.array(
            [[w.s_m, w.vx_mps, w.x_m, w.y_m, w.psi_rad] for w in msg.wpnts])
        if not np.isfinite(self.gb).all() or \
                np.any(np.diff(self.gb[:, 0]) <= 1e-5):
            self.gb = None
            self.track_len = 0.0
            self.get_logger().error(
                'invalid global waypoints; predictor disabled',
                throttle_duration_sec=2.0)
            return
        self.ds = self.gb[1, 0] - self.gb[0, 0]
        closure = float(np.linalg.norm(self.gb[0, 2:4] - self.gb[-1, 2:4]))
        self.track_len = self.gb[-1, 0] - self.gb[0, 0] + closure
        self.gb_stamp = self.get_clock().now()

    def cb_odom(self, msg: Odometry):
        self.ego_s = msg.pose.pose.position.x
        self.ego_vs = msg.twist.twist.linear.x
        source = Time.from_msg(msg.header.stamp)
        now = self.get_clock().now()
        source_age = (now - source).nanoseconds * 1e-9 \
            if source.nanoseconds else math.inf
        if not math.isfinite(self.ego_s) or not math.isfinite(self.ego_vs) or \
                (self.require_source_stamp and source.nanoseconds == 0) or \
                (source.nanoseconds and (
                    source_age > self.odom_timeout or
                    source_age < -self.source_future_tolerance)):
            self.ego_stamp = None
            self.get_logger().error(
                'invalid, unstamped, or stale Frenet odometry; '
                'prediction disabled',
                throttle_duration_sec=2.0)
            return
        self.ego_stamp = source if source.nanoseconds else now

    def cb_obs(self, msg: ObstacleArray):
        if self.track_len <= 0.0:
            return
        source_stamp = Time.from_msg(msg.header.stamp)
        now = self.get_clock().now()
        if self.require_source_stamp and source_stamp.nanoseconds == 0:
            self.get_logger().warn(
                'prediction input has no source timestamp',
                throttle_duration_sec=2.0)
            return
        if source_stamp.nanoseconds:
            age = (now - source_stamp).nanoseconds * 1e-9
            if age < -self.source_future_tolerance or age > self.obstacle_timeout:
                self.get_logger().warn(
                    f'prediction input stale: source age={age:.3f}s',
                    throttle_duration_sec=2.0)
                return
        for o in msg.obstacles:
            values = (
                o.s_center, o.d_center, o.vs, o.size,
                o.s_var, o.d_var, o.vs_var,
            )
            if not all(math.isfinite(float(value)) for value in values) or \
                    o.size < 0.0 or o.s_var < 0.0 or \
                    o.d_var < 0.0 or o.vs_var < 0.0:
                self.get_logger().error(
                    f'malformed prediction input obstacle {o.id}; frame ignored',
                    throttle_duration_sec=2.0)
                return
        for o in msg.obstacles:
            if o.is_static or not o.is_visible:
                continue
            b = self.buf.setdefault(o.id, {
                'S': [], 'D': [], 'V': [], 'last_s': None,
                'size': o.size, 'n_new': 0, 'vs_recent': [], 't_last': None,
                'source_ns': None,
                'latest_s': o.s_center, 'latest_d': o.d_center,
                'latest_vs': o.vs, 's_var': o.s_var, 'd_var': o.d_var,
                'vs_var': o.vs_var})
            frame_source_ns = source_stamp.nanoseconds or now.nanoseconds
            if (b['source_ns'] is not None
                    and frame_source_ns <= b['source_ns']):
                continue
            # 실측 vs / 최종 관측 시각은 데시메이션과 무관하게 항상 갱신 —
            # 파킹(정지) 감지와 소멸 감지는 "안 움직이는" 동안에도 돌아야 한다
            b['vs_recent'] = (b['vs_recent'] + [o.vs])[-10:]
            b['t_last'] = now
            b['source_ns'] = frame_source_ns
            b['latest_s'], b['latest_d'], b['latest_vs'] = \
                o.s_center, o.d_center, o.vs
            b['size'] = o.size
            b['s_var'], b['d_var'], b['vs_var'] = o.s_var, o.d_var, o.vs_var
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
            if len(b['S']) >= self.min_samples and (
                    o.id not in self.gp_d or b['n_new'] >= self.refit_every):
                self._fit(o.id, b)
                b['n_new'] = 0

    def _fit(self, oid, b):
        started = time.perf_counter()
        try:
            gpd = GP1D(
                self.gp_ls, sigma_f=0.5, sigma_n=self.gp_sn,
                track_len=self.track_len)
            gpd.fit(b['S'], b['D'])
            gpv = GP1D(
                self.gp_ls, sigma_f=2.0, sigma_n=0.2,
                track_len=self.track_len)
            gpv.fit(b['S'], b['V'])
        except (np.linalg.LinAlgError, ValueError) as exc:
            self.get_logger().error(f'GP fit failed for opponent {oid}: {exc}')
            return
        elapsed_ms = (time.perf_counter() - started) * 1000.0
        if elapsed_ms > self.max_fit_ms:
            self.get_logger().warn(
                f'GP fit over budget: {elapsed_ms:.1f}ms > {self.max_fit_ms:.1f}ms',
                throttle_duration_sec=2.0)
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

        now = self.get_clock().now()
        odom_age = (now - self.ego_stamp).nanoseconds * 1e-9 \
            if self.ego_stamp is not None else math.inf
        odom_fresh = (
            -self.source_future_tolerance <= odom_age <= self.odom_timeout)
        if self.gb is not None and odom_fresh:
            for oid, b in self.buf.items():
                if b['last_s'] is not None:
                    pitwall.log('opp_gap',
                                (b['last_s'] - self.ego_s) % self.track_len)
                if b['last_s'] is None or b['t_last'] is None:
                    continue
                source_age = (
                    (now.nanoseconds - b['source_ns']) * 1e-9
                    if b.get('source_ns') is not None else math.inf)
                if (source_age > self.obstacle_timeout
                        or source_age < -self.source_future_tolerance):
                    continue
                pred = self._predict_catch(oid, b) if oid in self.gp_v \
                    else self._predict_catch_cv(oid, b)
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

    def _make_prediction_obstacle(self, oid, b, s_start, span_m,
                                  d_mean, d_sigma, v_mean, v_sigma,
                                  prediction_time):
        """Build a conservative Frenet tube, not a midpoint-only obstacle."""
        L = self.track_len
        max_span = max(1.0, self.max_prediction_span_ratio * L)
        nominal_span = min(max(float(span_m), 0.0), max_span)
        nominal_start = float(s_start) % L
        s_sigma = (
            math.sqrt(max(float(b.get('s_var', 0.0)), 0.0)) +
            max(float(v_sigma), 0.0) * max(float(prediction_time), 0.0))
        longitudinal_inflation = self.uncertainty_sigma * s_sigma
        requested_span = nominal_span + 2.0 * longitudinal_inflation
        uncertainty_capped = False
        if requested_span <= max_span:
            s_start = (nominal_start - longitudinal_inflation) % L
            span = requested_span
        else:
            uncertainty_capped = True
            # 단순히 start만 I만큼 뒤로 옮긴 뒤 span을 자르면 I>cap일 때
            # nominal catch zone 자체가 잘려 나간다. 제한 폭을 nominal tube
            # 중심에 놓아 적어도 계산된 충돌 구간은 항상 포함한다.
            nominal_center = nominal_start + 0.5 * nominal_span
            s_start = (nominal_center - 0.5 * max_span) % L
            span = max_span
            self.get_logger().warn(
                f'prediction uncertainty tube capped at {max_span:.2f}m '
                'around nominal catch zone',
                throttle_duration_sec=2.0)
        s_end = (s_start + span) % L
        half = max(b['size'] / 2.0, 0.10)
        sigma = max(float(d_sigma), math.sqrt(max(b.get('d_var', 0.0), 0.0)))

        o = Obstacle()
        o.id = oid
        o.s_start = s_start
        o.s_end = s_end
        o.s_center = (s_start + span / 2.0) % L
        o.d_center = float(d_mean)
        if uncertainty_capped:
            # 모델 불확실성이 허용 longitudinal 표현 폭보다 크면 안전한
            # 추월 corridor를 증명할 수 없다. 전 차로를 막는 no-pass tube와
            # 정지 target을 보내 Spliner가 회피 대신 trailing/stop 하게 한다.
            o.d_left = 1.0e3
            o.d_right = -1.0e3
            o.vs = 0.0
        else:
            o.d_left = float(d_mean + half + self.uncertainty_sigma * sigma)
            o.d_right = float(d_mean - half - self.uncertainty_sigma * sigma)
            # trailing 목표는 상대 속도의 보수적 하한을 쓴다.
            o.vs = max(
                float(v_mean) - self.uncertainty_sigma * max(float(v_sigma), 0.0),
                0.0)
        o.s_var = max(float(b.get('s_var', 0.0)), 0.05 ** 2)
        o.d_var = sigma ** 2
        o.vs_var = max(float(v_sigma) ** 2,
                       float(b.get('vs_var', 0.0)), 0.1 ** 2)
        o.vd_var = 0.1 ** 2
        o.size = b['size']
        o.is_static = True
        o.is_visible = True
        return o

    def _predict_catch_cv(self, oid, b):
        """Cold-start constant-velocity reachable tube before the first GP fit."""
        gap = (b['latest_s'] - self.ego_s) % self.track_len
        closing = self.ego_vs - max(b['latest_vs'], 0.0)
        if closing <= self.min_pass_relative_speed:
            return None
        t_catch = gap / max(closing, 1e-3)
        if t_catch > self.horizon:
            return None
        s_c = (b['latest_s'] + max(b['latest_vs'], 0.0) * t_catch) \
            % self.track_len
        position_sigma = math.sqrt(max(b.get('d_var', 0.0), 0.05 ** 2))
        speed_sigma = math.sqrt(max(b.get('vs_var', 0.0), 0.25))
        travel = max(b['size'] + self.pass_len,
                     max(b['latest_vs'], 0.0) *
                     (self.pass_len + b['size']) / max(closing, 0.3))
        prediction_time = min(
            self.horizon,
            t_catch + (self.pass_len + b['size']) / max(closing, 0.3))
        return self._make_prediction_obstacle(
            oid, b, (s_c - b['size'] / 2.0 - 0.3) % self.track_len,
            travel + b['size'] / 2.0 + 0.3,
            b['latest_d'], position_sigma,
            b['latest_vs'], speed_sigma, prediction_time)

    def _predict_catch(self, oid, b):
        """자차/상대 속도 프로파일 전방 적분 -> 따라잡는 s 구간 계산."""
        # 관측 두절(상대 제거) 또는 정지(파킹) 상대는 예측하지 않는다 —
        # GP는 과거 주행 속도만 알고 있어 정지 상대에게 유령 충돌구간을 만든다.
        # 정지 상대의 원시 관측은 spliner가 정적 장애물로 재분류해 처리한다.
        if b['t_last'] is None or \
                (self.get_clock().now() - b['t_last']).nanoseconds * 1e-9 > 1.0:
            return None
        vr = b['vs_recent']
        if len(vr) >= 5 and sum(abs(v) for v in vr) / len(vr) < self.vs_reset:
            return None
        L = self.track_len
        gpv, gpd = self.gp_v[oid], self.gp_d[oid]
        s_e, s_o = self.ego_s, float(b['latest_s'])
        # 자차 속도 = raceline 프로파일 * 현재 스케일 (컨트롤러 speed_scale 반영)
        ref_v = max(self._raceline_v(s_e), 0.5)
        ego_scale = float(np.clip(max(self.ego_vs, 0.3) / ref_v, 0.2, 1.2))

        dt = 0.1
        t_catch = None
        elapsed = 0.0
        for _ in range(int(self.horizon / dt)):
            gap = (s_o - s_e) % L
            if gap < self.catch_gap:
                t_catch = elapsed
                break
            s_e = (s_e + max(self._raceline_v(s_e) * ego_scale, 0.3) * dt) % L
            s_o = (s_o + max(float(gpv.predict(s_o)[0]), 0.1) * dt) % L
            elapsed += dt
        if t_catch is None:
            return None      # 호라이즌 내 추월 상황 아님

        # 충돌 구간: 따라잡는 지점부터 상대가 pass_length만큼 더 가는 구간
        s_c = s_o
        v_o = max(float(gpv.predict(s_c)[0]), 0.1)
        v_e = max(self._raceline_v(s_c) * ego_scale, 0.3)
        rel = v_e - v_o
        # 상대속도가 비슷하면 modulo로 여러 lap의 추월구간을 짧은 구간으로
        # 접지 않는다. 대신 horizon 동안의 no-pass tube를 만든다.
        if rel < self.min_pass_relative_speed:
            prediction_time = self.horizon
            travel = min(
                self.max_prediction_span_ratio * L,
                max(self.pass_len + b['size'], v_o * self.horizon))
        else:
            t_pass = (self.pass_len + b['size']) / rel
            prediction_time = min(self.horizon, t_catch + t_pass)
            travel = v_o * t_pass + b['size'] / 2
        s_start = (s_c - b['size'] / 2 - 0.3) % L
        span = min(
            travel + b['size'] / 2.0 + 0.3,
            self.max_prediction_span_ratio * L)
        samples = (s_start + np.linspace(0.0, span, max(3, int(span / 0.5)))) % L
        d_means, d_stds = gpd.predict_stats(samples)
        v_means, v_stds = gpv.predict_stats(samples)
        d_mid = float(d_means[len(d_means) // 2])
        # 전체 구간 lateral 변화와 posterior uncertainty를 하나의 보수 tube로.
        d_sigma = float(max(
            np.max(d_stds),
            (np.max(d_means) - np.min(d_means)) / 2.0))
        return self._make_prediction_obstacle(
            oid, b, s_start, span, d_mid, d_sigma,
            float(np.mean(v_means)), float(np.max(v_stds)),
            prediction_time)

    def _frenet_to_xy(self, s, d):
        i = int(s / self.ds) % len(self.gb)
        _, _, x, y, psi = self.gb[i]
        return x - d * math.sin(psi), y + d * math.cos(psi)

    def _make_markers(self, mrks, oid, pred):
        """예측 충돌 구간(빨간 밴드) + 학습된 상대 라인(주황 선)."""
        L = self.track_len
        gpd = self.gp_d.get(oid)

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
            d = float(gpd.predict(s)[0]) if gpd is not None else pred.d_center
            x, y = self._frenet_to_xy(s, d)
            band.points.append(Point(x=x, y=y, z=0.06))
        if band.points:
            mrks.markers.append(band)

        if gpd is None:
            return

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
    node = OpponentPredictorNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
