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

try:
    # 설치 트리(copy-install): pitwall.py 가 노드 옆(lib/planning_pkg/)에 깔린다.
    import pitwall
except ModuleNotFoundError:
    # colcon --symlink-install: 설치된 노드가 src 원본으로 가는 심볼릭 링크라
    # 파이썬이 sys.path[0] 을 realpath(=scripts/<하위>/)로 잡는다. 그러면
    # 같이 설치된 pitwall.py 가 경로에 없다 -> 형제 폴더 scripts/lib 를 얹는다.
    import sys
    from pathlib import Path
    sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'lib'))
    import pitwall


class GP1D:
    """s -> y 1차원 GP 회귀 (RBF 커널). 트랙 s 경계는 데이터 wrap 증강으로 처리."""

    def __init__(self, length_scale, sigma_f, sigma_n, track_len,
                 mean_fn=None):
        self.ls = length_scale
        self.sf2 = sigma_f ** 2
        self.sn2 = sigma_n ** 2
        self.L = track_len
        self.x = None
        # 사전평균 m(s). None 이면 학습 데이터의 상수평균(종전 동작).
        # 속도 GP 는 여기에 "레이스라인 프로파일 x 상대 페이스 배율"을 넣는다:
        # 상수평균은 데이터 없는 s 에서 코너 감속/직선 가속을 전혀 모르고
        # (my_track0821 raceline vx 4.66~7.80, std 0.84 -> 최악 1.68 m/s 오차)
        # 그 오차가 predict_stats 의 sigma -> 튜브 종방향 팽창으로 직결된다.
        self.mean_fn = mean_fn

    def _wrap_augment(self, x, y):
        # 경계 근처(2*ls) 데이터를 반대편으로 복제해 주기성 근사
        pad = 2.0 * self.ls
        lo = x < pad
        hi = x > self.L - pad
        return (np.concatenate([x, x[lo] + self.L, x[hi] - self.L]),
                np.concatenate([y, y[lo], y[hi]]))

    def _prior(self, x):
        """사전평균 m(x). mean_fn 이 없으면 학습 데이터 상수평균(종전 동작)."""
        x = np.atleast_1d(np.asarray(x, dtype=float))
        if self.mean_fn is None:
            return np.full(x.shape, self.y_mean, dtype=float)
        # _wrap_augment 가 [-2ls, L+2ls] 범위를 만들므로 mean_fn 에는 항상
        # [0, L) 로 접어서 넘긴다.
        m = np.asarray(self.mean_fn(x % self.L), dtype=float)
        return np.broadcast_to(m, x.shape).astype(float)

    def fit(self, x, y):
        x, y = self._wrap_augment(np.asarray(x), np.asarray(y))
        if len(x) > 150:                      # 균일 서브샘플로 계산량 제한
            idx = np.linspace(0, len(x) - 1, 150).astype(int)
            x, y = x[idx], y[idx]
        self.x = x
        # mean_fn 이 있어도 y_mean 은 계산해 둔다 — mean_fn 이 예외를 던지는
        # 등으로 못 쓰게 되면 상수평균으로 되돌릴 수 있어야 한다.
        self.y_mean = float(np.mean(y))
        d = x[:, None] - x[None, :]
        K = self.sf2 * np.exp(-0.5 * (d / self.ls) ** 2) + self.sn2 * np.eye(len(x))
        self.Lchol = np.linalg.cholesky(K)
        # GP 는 사전평균으로부터의 **잔차**만 학습한다.
        self.alpha = np.linalg.solve(
            self.Lchol.T, np.linalg.solve(self.Lchol, y - self._prior(x)))

    def predict(self, xq):
        # 학습 데이터가 wrap 증강돼 있어 [0, L) 질의는 직접 거리로 충분
        xq = np.atleast_1d(np.asarray(xq, dtype=float))
        d = xq[:, None] - self.x[None, :]
        k = self.sf2 * np.exp(-0.5 * (d / self.ls) ** 2)
        return self._prior(xq) + k @ self.alpha

    def predict_stats(self, xq):
        """Posterior mean and point-wise standard deviation."""
        xq = np.atleast_1d(np.asarray(xq, dtype=float))
        d = xq[:, None] - self.x[None, :]
        k = self.sf2 * np.exp(-0.5 * (d / self.ls) ** 2)
        mean = self._prior(xq) + k @ self.alpha
        solved = np.linalg.solve(self.Lchol, k.T)
        var = np.maximum(self.sf2 - np.sum(solved * solved, axis=0), self.sn2)
        return mean, np.sqrt(var)


class PaceProfile:
    """GP 학습 전에 쓰는 속도 사전모델: raceline vx(s) * 상대 페이스 배율.

    GP1D 와 같은 (predict / predict_stats) 인터페이스를 갖는다 —
    _predict_catch 가 둘을 구분하지 않고 쓰도록 하기 위한 것이다.
    종전에는 GP 가 학습될 때까지(min_samples 40 x sample_min_ds 0.15 = 6.0m
    주행) 등속 CV 튜브밖에 없었고, 그 사이 코너 감속을 전혀 반영하지 못했다.
    """

    def __init__(self, raceline_v_arr, scale, sigma):
        self._v = raceline_v_arr
        self.scale = float(scale)
        self.sigma = float(sigma)

    def predict(self, xq):
        # GP1D 와 같이 스칼라 질의에도 1-D 배열을 돌려준다.
        return np.atleast_1d(self._v(xq)) * self.scale

    def predict_stats(self, xq):
        mean = self.predict(xq)
        return mean, np.full(np.shape(mean), self.sigma, dtype=float)


class ConstProfile:
    """상수 사전모델 (횡방향 d 용). GP1D 와 같은 인터페이스."""

    def __init__(self, value, sigma):
        self.value = float(value)
        self.sigma = float(sigma)

    def predict(self, xq):
        return np.full(np.atleast_1d(np.asarray(xq)).shape, self.value,
                       dtype=float)

    def predict_stats(self, xq):
        mean = self.predict(xq)
        return mean, np.full(np.shape(mean), self.sigma, dtype=float)


class OpponentPredictorNode(Node):
    def __init__(self):
        super().__init__('opponent_predictor_node')
        pitwall.init(self)

        self.declare_parameter('predictor/min_samples', 40)      # GP 학습 최소 관측 수
        self.declare_parameter('predictor/sample_min_ds', 0.15)  # 관측 간 최소 s 간격 [m]
        self.declare_parameter('predictor/max_samples', 600)
        self.declare_parameter('predictor/gp_length_scale', 3.0) # [m]
        self.declare_parameter('predictor/gp_sigma_n', 0.08)      # 주행라인(d) GP
        # 속도(vs) GP 의 관측 노이즈. predict_stats 의 분산 하한(sn2)이라
        # v_sigma -> s_sigma -> requested_span -> uncertainty_capped 로
        # spliner 가 보는 예측 튜브 크기를 사실상 지배한다. 예전엔 0.2 로
        # 하드코딩돼 '예측이 너무 보수적'일 때 손댈 노브가 없었다.
        self.declare_parameter('predictor/gp_sigma_n_v', 0.2)
        self.declare_parameter('predictor/refit_every', 20)      # N 관측마다 재학습
        self.declare_parameter('predictor/horizon_sec', 8.0)     # 따라잡기 탐색 시간
        self.declare_parameter('predictor/catch_gap_m', 0.8)     # 이 간격이면 충돌 임박
        self.declare_parameter('predictor/pass_length_m', 1.2)   # 추월에 필요한 상대 s 여유
        self.declare_parameter('predictor/publish_rate_hz', 10.0)
        self.declare_parameter('predictor/obstacle_topic', '/perception/obstacles')
        self.declare_parameter('predictor/odom_timeout_sec', 0.20)
        self.declare_parameter('predictor/obstacle_timeout_sec', 0.30)
        # 사라진 상대의 관측 버퍼/GP를 폐기하기까지의 시간 [s].
        # 브릿지의 bridge/track_forget_sec 와 같은 역할.
        self.declare_parameter('predictor/track_forget_sec', 2.0)
        # /global_waypoints 는 2 Hz 발행이므로 넉넉히 잡는다.
        self.declare_parameter('predictor/global_waypoints_timeout_sec', 2.0)
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
        self.gp_sn_v = gp('predictor/gp_sigma_n_v')
        self.refit_every = int(gp('predictor/refit_every'))
        self.horizon = gp('predictor/horizon_sec')
        self.catch_gap = gp('predictor/catch_gap_m')
        self.pass_len = gp('predictor/pass_length_m')
        self.vs_reset = gp('predictor/vs_reset')
        self.odom_timeout = gp('predictor/odom_timeout_sec')
        self.obstacle_timeout = gp('predictor/obstacle_timeout_sec')
        self.forget_sec = gp('predictor/track_forget_sec')
        self.gb_timeout = gp('predictor/global_waypoints_timeout_sec')
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
        # 상한 0.9: 튜브가 트랙 한 바퀴에 가까워지면 s wrap 으로 자기 자신을
        # 감싸 "어디에도 통로가 없다"가 되므로 1.0 미만으로 묶는다.
        # (0.5 였을 때 소형 트랙에서 느린 추월이 상시 CAP 되던 것을 완화 —
        #  params.yaml 의 max_prediction_span_ratio 주석 참조.)
        if invalid or not 0.0 < self.max_prediction_span_ratio < 0.9 or \
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
            # ── 2026-08-19: 무엇에 걸렸는지 찍는다 ────────────────────────
            # run_0818_232744 에서 이 로그가 62초 내내 나왔는데
            # /car_state/frenet/odom 의 실측 지연은 p50 18ms / p90 30ms 로
            # odom_timeout(0.20)보다 한참 안쪽이었다. 즉 지연이 아니라 다른
            # 조건(스탬프 0, 미래 스탬프, 비유한값, 또는 노드 clock 소스와
            # 헤더 시각의 불일치)이 원인일 가능성이 크다. 값을 봐야 좁혀진다.
            self.get_logger().error(
                'invalid, unstamped, or stale Frenet odometry; '
                'prediction disabled '
                f'[ego_s={self.ego_s} ego_vs={self.ego_vs} '
                f'stamp_ns={source.nanoseconds} '
                f'age={source_age:.3f}s timeout={self.odom_timeout:.3f}s '
                f'future_tol={self.source_future_tolerance:.3f}s '
                f'require_stamp={self.require_source_stamp} '
                f'use_sim_time={self.get_parameter("use_sim_time").value}]',
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
            # 속도 GP 는 raceline 프로파일 x 페이스 배율을 사전평균으로 쓰고
            # **잔차만** 학습한다. 잔차의 진폭은 원신호보다 훨씬 작으므로
            # sigma_f 도 2.0 -> 0.5 로 내린다 (데이터 없는 s 에서 사후 sigma 가
            # sigma_f 로 발산하는데, 그 sigma 가 그대로 튜브 종방향 팽창이 된다).
            pace_scale, _ = self._pace_scale(b)
            b['pace_scale'] = pace_scale
            if pace_scale is None:
                gpv = GP1D(
                    self.gp_ls, sigma_f=2.0, sigma_n=self.gp_sn_v,
                    track_len=self.track_len)
            else:
                gpv = GP1D(
                    self.gp_ls, sigma_f=0.5, sigma_n=self.gp_sn_v,
                    track_len=self.track_len,
                    mean_fn=lambda sq, k=pace_scale:
                        self._raceline_v_arr(sq) * k)
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

    def _raceline_v_arr(self, s):
        """_raceline_v 의 벡터판 — GP 사전평균/PaceProfile 이 부른다.

        _predict_catch 는 GP 를 스칼라(gpv.predict(s_o)[0])로도 부르므로
        GP1D 와 같이 항상 1-D 배열을 돌려줘야 한다.
        """
        idx = (np.atleast_1d(np.asarray(s, dtype=float)) /
               self.ds).astype(int) % len(self.gb)
        return self.gb[idx, 1].astype(float)

    def _pace_scale(self, b):
        """상대 페이스 배율 = median(관측 vs / 그 지점 raceline vx).

        자차가 이미 ego_scale 로 쓰는 것과 **같은 모델**을 상대에 적용한다.
        종전에는 자차만 raceline 프로파일로 적분하고 상대는 상수평균 GP 라
        비대칭이었다. 배율 하나(1 파라미터)라 관측 몇 점만으로 추정된다.
        """
        if self.gb is None or self.ds <= 0.0:
            return None, 0
        ratios = []
        for s_i, v_i in zip(b['S'], b['V']):
            ref = self._raceline_v(s_i)
            if ref > 0.5 and math.isfinite(v_i) and v_i > 0.0:
                ratios.append(v_i / ref)
        if not ratios:
            # 데시메이션 버퍼가 아직 비어 있으면 최신 관측 한 점으로라도.
            ref = self._raceline_v(b['latest_s'])
            v_i = b['latest_vs']
            if ref > 0.5 and math.isfinite(v_i) and v_i > 0.0:
                ratios.append(v_i / ref)
        if not ratios:
            return None, 0
        # clip 범위는 자차 ego_scale 과 같다 — 그 밖의 값은 인지 노이즈로 본다.
        return float(np.clip(np.median(ratios), 0.2, 1.2)), len(ratios)

    def _pace_model(self, b):
        """GP 학습 전 속도 사전모델. 만들 수 없으면 None."""
        scale, n = self._pace_scale(b)
        if scale is None:
            return None
        # 표본이 적을수록 배율 추정이 거칠다 — sigma 를 표본 수로 눌러 준다.
        # n=1 -> 0.60, n=9 -> 0.20 으로 gp_sigma_n_v 하한에 수렴한다.
        sigma = max(self.gp_sn_v, 0.6 / math.sqrt(max(n, 1)))
        return PaceProfile(self._raceline_v_arr, scale, sigma)

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
        # 사라진 상대의 버퍼/GP를 정리한다. 예전엔 지우는 코드가 없어
        # (a) opp_gap 텔레메트리가 이미 없어진 상대의 마지막 위치로 계속
        # 발행됐고 (여러 id가 같은 토픽에 섞여 읽을 수 없었다)
        # (b) buf(최대 600점x3) + GP(150x150 Cholesky)가 무한 누적됐다.
        # 브릿지의 track_forget_sec 와 같은 성격의 정리다.
        stale_ids = [
            oid for oid, b in self.buf.items()
            if b.get('source_ns') is None
            or (now.nanoseconds - b['source_ns']) * 1e-9 > self.forget_sec]
        for oid in stale_ids:
            self.buf.pop(oid, None)
            self.gp_d.pop(oid, None)
            self.gp_v.pop(oid, None)

        # /global_waypoints 신선도. 예전엔 gb_stamp 를 갱신만 하고 읽지 않아,
        # global_trajectory_node 가 죽어도 predictor 는 마지막 raceline 으로
        # 계속 예측했다 (ego/장애물은 이미 stale 검사를 한다).
        gb_age = (now - self.gb_stamp).nanoseconds * 1e-9 \
            if self.gb_stamp is not None else math.inf
        gb_fresh = gb_age <= self.gb_timeout
        if self.gb is not None and odom_fresh and gb_fresh:
            for oid, b in self.buf.items():
                if b['last_s'] is None or b['t_last'] is None:
                    continue
                source_age = (
                    (now.nanoseconds - b['source_ns']) * 1e-9
                    if b.get('source_ns') is not None else math.inf)
                if (source_age > self.obstacle_timeout
                        or source_age < -self.source_future_tolerance):
                    continue
                # 신선도 게이트를 통과한 상대만 텔레메트리에 싣는다.
                pitwall.log('opp_gap',
                            (b['last_s'] - self.ego_s) % self.track_len)
                # GP 학습 전에도 사전모델(raceline x 페이스 배율)로 같은
                # 경로를 탄다 — 종전에는 min_samples(40) x sample_min_ds(0.15)
                # = 6.0m 를 따라다녀야 첫 fit 이 나왔고, 그 전 구간은 코너
                # 감속을 모르는 등속 CV 튜브뿐이었다 (랩 1의 첫 조우가 통째로
                # 그 구간에 들어간다).
                if oid in self.gp_v:
                    pred = self._predict_catch(oid, b)
                else:
                    pace = self._pace_model(b)
                    if pace is not None:
                        d_sigma = math.sqrt(
                            max(b.get('d_var', 0.0), 0.05 ** 2))
                        pred = self._predict_catch(
                            oid, b, gpv=pace,
                            gpd=ConstProfile(b['latest_d'], d_sigma))
                    else:
                        pred = self._predict_catch_cv(oid, b)
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
            # 추월 corridor를 증명할 수 없다. 전 차로를 막는 no-pass tube로
            # Spliner 가 회피 대신 trailing 하게 한다.
            o.d_left = 1.0e3
            o.d_right = -1.0e3
            # ── 종전 o.vs = 0.0 (정지 target) 을 상대 속도 하한으로 바꾼다.
            #    (2026-08-21, max_prediction_span_ratio 0.45 -> 0.8 과 한 세트)
            # 캡이 걸린 튜브는 span 이 곧 max_span 이라 0.8 트랙에서는 랩의
            # 80% 를 덮는다. 그러면 FilterObstacles 의 ego_inside 가 참이 되고
            # BuildTrailing 이 "이미 gap 안" 분기로 들어가 **그 자리에서**
            # v_target = obs.vs 를 명령한다. vs=0 이면 그게 트랙 한가운데
            # 급정지다.
            #   오프라인 스윕(my_track0821, 432 시나리오): 봉쇄 튜브 중
            #   자차가 그 안에 있는 비율이 ratio 0.45 에서 29%(71/242),
            #   0.8 에서 100%(91/91) — 캡이 커질수록 반드시 이 분기로 간다.
            # "추월 통로를 증명 못 한다"의 옳은 답은 정지가 아니라 **상대
            # 속도로 따라가기**다. 통로를 막는 것은 d_left/d_right 가 이미
            # 하고 있고, 속도까지 0 으로 만들 이유는 없다.
            # 값은 통과 분기와 **같은** 보수적 하한을 쓴다.
            o.vs = max(
                float(v_mean) - self.uncertainty_sigma * max(float(v_sigma), 0.0),
                0.0)
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

    def _predict_catch(self, oid, b, gpv=None, gpd=None):
        """자차/상대 속도 프로파일 전방 적분 -> 따라잡는 s 구간 계산.

        gpv/gpd 를 넘기면 학습된 GP 대신 그 모델을 쓴다 (GP 학습 전
        PaceProfile/ConstProfile 사전모델용). 인터페이스가 같으므로 아래
        본문은 두 경우를 구분하지 않는다.
        """
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
        if gpv is None:
            gpv = self.gp_v[oid]
        if gpd is None:
            gpd = self.gp_d[oid]
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
