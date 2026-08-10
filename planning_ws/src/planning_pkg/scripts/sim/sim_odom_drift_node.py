#!/usr/bin/env python3
"""gym ground truth -> 드리프트가 있는 휠 오도메트리 (시뮬 전용).

gym_bridge는 `map -> <ego_ns>/base_link` TF를 직접 발행한다 (= 완벽한 위치추정).
그 상태로는 particle_filter가 보정을 끼워 넣을 `odom` 프레임이 없고, 설령
띄워도 오차가 0이라 map 필터/장애물 인지 검증이 무의미해진다.

이 노드가 그 자리를 메운다:

  gym_bridge (/tf -> /tf_gym 으로 리맵)
      |
      +-- map -> <ego_ns>/base_link      : **버림** (GT 위치추정)
      +-- 그 외 전부 (base_link->laser,
          휠 힌지, opp_racecar ...)       : 그대로 /tf 로 중계
      |
  /ego_racecar/odom (GT, map frame)
      |
      +--> 바디프레임 증분에 스케일오차 + 랜덤워크 주입 -> 누적
             -> /odom (nav_msgs/Odometry, frame odom, child <ego_ns>/base_link)
             -> TF odom -> <ego_ns>/base_link
      |
  particle_filter -> map -> odom TF + /pf/pose/odom

결과 TF 체인: map -(PF)-> odom -(이 노드)-> base_link -(gym)-> laser

--- 기본값 근거 (run_0804_215144 실측) ---
map->odom(= PF가 누적 보정한 odom 드리프트)을 bag에서 뽑으면 29.4 m 주행 동안
순 변화가 x -0.152 m / y +0.178 m (합 0.234 m), yaw -0.0786 rad 였다.
아래 기본값은 이 크기를 재현하도록 잡았다:
  거리 스케일오차 1% -> 29.4 m 에서 0.29 m
  랜덤워크 0.02 m/√m -> 29.4 m 에서 0.11 m
  yaw 스케일오차 1% -> 한 바퀴(2π) 에서 0.063 rad
  yaw 랜덤워크 0.010 rad/√m -> 29.4 m 에서 0.054 rad
스케일오차는 seed로 고정되는 상수라 매 런 재현 가능하고, 랜덤워크만 매번 다르다.

주의: PF 파티클 산포(실측 std_x 0.044 / std_y 0.021 / std_yaw 0.018 rad)는
localize_sim.yaml의 motion_dispersion이 결정한다. 여기 드리프트는 PF가
"보정해야 할 대상"을 만드는 쪽이고, 둘은 별개 손잡이다.
"""
import math

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped, TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from tf2_msgs.msg import TFMessage


def _yaw(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def _wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


class SimOdomDriftNode(Node):

    def __init__(self):
        super().__init__('sim_odom_drift_node')

        self.declare_parameter('drift/ego_namespace', 'ego_racecar')
        self.declare_parameter('drift/gt_odom_topic', '/ego_racecar/odom')
        self.declare_parameter('drift/odom_topic', '/odom')
        self.declare_parameter('drift/odom_frame', 'odom')
        self.declare_parameter('drift/map_frame', 'map')
        # 드리프트 크기 (근거는 모듈 docstring)
        self.declare_parameter('drift/distance_scale_sigma', 0.010)
        self.declare_parameter('drift/yaw_scale_sigma', 0.010)
        self.declare_parameter('drift/sigma_xy_per_sqrt_m', 0.020)
        self.declare_parameter('drift/sigma_theta_per_sqrt_m', 0.010)
        # 0이면 매 실행 다른 시드. 회귀 비교용으로는 고정값을 줄 것.
        self.declare_parameter('drift/seed', 0)
        # 텔레포트(/initialpose, 스폰) 판정. 이 이상 한 스텝에 튀면 적분하지 않고
        # 기준만 갱신한다 — 안 그러면 순간이동이 통째로 드리프트로 들어간다.
        self.declare_parameter('drift/teleport_jump_m', 0.5)
        # PF 초기화용 /initialpose 자동 발행 (PF는 기동 시 initialize_global()이라
        # 좁은 트랙에서 수렴이 느리다).
        #
        # ⚠️ /initialpose 는 **두 노드가 듣는다**: PF는 파티클을 그 포즈 주위로
        # 다시 뿌리고(SETTING POSE), gym_bridge는 **차를 그 포즈로 순간이동시킨다**
        # (ego_reset_callback). 그래서 이건 "PF 힌트"가 아니라 시뮬 리셋이다.
        #
        # 예전 동작(3 s 주기 × 3회 무조건 발행)이 만든 문제:
        #   PF가 맵 로딩·센서모델 사전계산으로 뜨는 데 ~7 s 걸린다. 그 사이 플래너/
        #   컨트롤은 2 s 만에 준비돼 차가 이미 달리고 있다. 그래서 3 s 발행은 PF가
        #   구독하기 전이라 그냥 버려지고 gym만 차를 출발선으로 되돌리며, 6·9 s
        #   발행은 달리는 차를 두 번 더 출발선으로 끌어당긴다. PF 추정과 실제가
        #   어긋난 채로 남으면 보정량이 jump guard(0.30 m)를 넘어 연속 거부되고
        #   (`Rejected PF scan update ... exceeds guard`) 필터는 데드레커닝으로
        #   발산한다. 2026-08-07 실측: 거부 27~48회, 경로오차 85~240 m.
        #
        # 지금 동작: **PF가 실제로 살아난 뒤(=/pf/pose/odom 첫 수신) 딱 한 번** 쏜다.
        # 그 한 번이 차를 출발선으로 되돌리면서 PF를 같은 포즈로 seed 하므로
        # 둘이 일치한 상태에서 주행이 시작된다. 그 뒤로는 절대 다시 쏘지 않는다.
        self.declare_parameter('drift/publish_initialpose', True)
        self.declare_parameter('drift/init_pose_x', 0.0)
        self.declare_parameter('drift/init_pose_y', 0.0)
        self.declare_parameter('drift/init_pose_yaw', 0.0)
        # PF 기동 여부를 확인하는 폴링 주기[s]. (예전엔 '발행 지연'이었다)
        self.declare_parameter('drift/init_delay_sec', 0.5)
        # 최대 시도 횟수. 첫 발행이 PF에 닿지 않았을 때(laser TF 미준비로 PF가
        # 버리는 경우)만 재시도하며, 아래 init_accept_m 로 판정한다.
        self.declare_parameter('drift/init_repeat', 3)
        # 재시도 판정 거리[m]. 발행 후에도 PF 포즈가 시작 포즈에서 이만큼 넘게
        # 떨어져 있으면 "안 먹었다"로 보고 다시 쏜다. 정상적으로 seed 됐다면
        # 폴링 한 주기(0.5 s) 동안 차가 가는 거리(<2 m)만 벌어진다.
        self.declare_parameter('drift/init_accept_m', 3.0)

        def p(name):
            return self.get_parameter(name).value

        self.ego_ns = str(p('drift/ego_namespace')).strip().strip('/')
        self.base_frame = f'{self.ego_ns}/base_link' if self.ego_ns else 'base_link'
        self.odom_frame = str(p('drift/odom_frame')).strip()
        self.map_frame = str(p('drift/map_frame')).strip()
        self.dist_scale_sigma = float(p('drift/distance_scale_sigma'))
        self.yaw_scale_sigma = float(p('drift/yaw_scale_sigma'))
        self.sigma_xy = float(p('drift/sigma_xy_per_sqrt_m'))
        self.sigma_th = float(p('drift/sigma_theta_per_sqrt_m'))
        self.teleport_jump = float(p('drift/teleport_jump_m'))

        bad = [v for v in (self.dist_scale_sigma, self.yaw_scale_sigma,
                           self.sigma_xy, self.sigma_th) if not (0.0 <= v <= 1.0)]
        if bad or self.teleport_jump <= 0.0 or not self.odom_frame or not self.map_frame:
            raise ValueError('invalid drift parameters')

        seed = int(p('drift/seed'))
        self.rng = __import__('random').Random(seed if seed else None)
        # 런 내내 고정되는 계통오차 (휠 반경/erpm gain 오차에 해당)
        self.dist_scale = self.rng.gauss(0.0, self.dist_scale_sigma)
        self.yaw_scale = self.rng.gauss(0.0, self.yaw_scale_sigma)

        # 드리프트 누적 상태 (odom 프레임 기준 포즈)
        self.ox = self.oy = self.oth = 0.0
        self.prev_gt = None          # (x, y, yaw)
        self.travelled = 0.0

        tf_qos = QoSProfile(
            depth=100,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.VOLATILE,
            history=QoSHistoryPolicy.KEEP_LAST,
        )
        # gym_bridge의 /tf 는 launch에서 /tf_gym 으로 리맵돼 들어온다.
        self.pub_tf = self.create_publisher(TFMessage, '/tf', tf_qos)
        self.create_subscription(TFMessage, '/tf_gym', self.cb_tf, tf_qos)

        self.pub_odom = self.create_publisher(Odometry, str(p('drift/odom_topic')), 10)
        self.create_subscription(
            Odometry, str(p('drift/gt_odom_topic')), self.cb_gt, 10)

        self.pub_init = None
        if bool(p('drift/publish_initialpose')):
            self.pub_init = self.create_publisher(
                PoseWithCovarianceStamped, '/initialpose', 10)
            self.init_left = max(1, int(p('drift/init_repeat')))
            self.init_accept_m = max(0.0, float(p('drift/init_accept_m')))
            self.init_xyz = (float(p('drift/init_pose_x')),
                             float(p('drift/init_pose_y')),
                             float(p('drift/init_pose_yaw')))
            self.init_sent = 0
            self.pf_pose = None      # PF가 살아났는지 + 어디로 추정하는지
            self.create_subscription(
                Odometry, '/pf/pose/odom', self.cb_pf_pose, 1)
            self.init_timer = self.create_timer(
                max(0.1, float(p('drift/init_delay_sec'))), self.cb_init_pose)

        self.get_logger().info(
            f'sim odom drift: base={self.base_frame} '
            f'scale_err={self.dist_scale * 100:+.2f}% '
            f'yaw_scale_err={self.yaw_scale * 100:+.2f}% '
            f'walk={self.sigma_xy}m/sqrt(m), {self.sigma_th}rad/sqrt(m) '
            f'(seed={seed or "random"})')

    # ---- TF 중계: GT map->base_link 만 버리고 나머지는 그대로 ----
    def cb_tf(self, msg: TFMessage):
        kept = [t for t in msg.transforms
                if not (t.header.frame_id == self.map_frame
                        and t.child_frame_id == self.base_frame)]
        if kept:
            out = TFMessage()
            out.transforms = kept
            self.pub_tf.publish(out)

    # ---- GT 증분 -> 드리프트 누적 -> /odom + odom->base_link ----
    def cb_gt(self, msg: Odometry):
        gx = msg.pose.pose.position.x
        gy = msg.pose.pose.position.y
        gth = _yaw(msg.pose.pose.orientation)

        if self.prev_gt is None:
            self.prev_gt = (gx, gy, gth)
            self.publish(msg)
            return

        px, py, pth = self.prev_gt
        dxw, dyw = gx - px, gy - py
        step = math.hypot(dxw, dyw)
        if step > self.teleport_jump:
            # 텔레포트: 적분하지 않고 기준만 옮긴다. odom 포즈는 유지하므로
            # PF가 map->odom을 다시 잡아야 하고, 그게 실차 재초기화와 같다.
            self.prev_gt = (gx, gy, gth)
            self.get_logger().warn(
                f'teleport detected ({step:.2f} m) — 드리프트 적분 건너뜀',
                throttle_duration_sec=1.0)
            self.publish(msg)
            return

        # 이전 바디프레임 기준 증분
        fwd = math.cos(pth) * dxw + math.sin(pth) * dyw
        lat = -math.sin(pth) * dxw + math.cos(pth) * dyw
        dth = _wrap(gth - pth)

        rt = math.sqrt(step)
        fwd += fwd * self.dist_scale + self.rng.gauss(0.0, self.sigma_xy * rt)
        lat += self.rng.gauss(0.0, self.sigma_xy * rt)
        dth += dth * self.yaw_scale + self.rng.gauss(0.0, self.sigma_th * rt)

        self.ox += math.cos(self.oth) * fwd - math.sin(self.oth) * lat
        self.oy += math.sin(self.oth) * fwd + math.cos(self.oth) * lat
        self.oth = _wrap(self.oth + dth)
        self.prev_gt = (gx, gy, gth)
        self.travelled += step
        self.publish(msg)

    def publish(self, gt: Odometry):
        stamp = gt.header.stamp
        qz, qw = math.sin(self.oth * 0.5), math.cos(self.oth * 0.5)

        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id = self.base_frame
        odom.pose.pose.position.x = self.ox
        odom.pose.pose.position.y = self.oy
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw
        # twist는 바디프레임이라 드리프트와 무관 — GT 그대로 (실차 VESC도
        # 속도는 그때그때 측정값이지 누적오차가 없다)
        odom.twist = gt.twist
        self.pub_odom.publish(odom)

        t = TransformStamped()
        t.header.stamp = stamp
        t.header.frame_id = self.odom_frame
        t.child_frame_id = self.base_frame
        t.transform.translation.x = self.ox
        t.transform.translation.y = self.oy
        t.transform.rotation.z = qz
        t.transform.rotation.w = qw
        out = TFMessage()
        out.transforms = [t]
        self.pub_tf.publish(out)

    def cb_pf_pose(self, msg: Odometry):
        self.pf_pose = (msg.pose.pose.position.x, msg.pose.pose.position.y)

    def _finish_init(self, reason):
        self.init_left = 0
        self.init_timer.cancel()
        self.get_logger().info(f'PF 초기 포즈 주입 종료 — {reason}')

    def cb_init_pose(self):
        if self.pub_init is None or self.init_left <= 0:
            return
        x, y, yaw = self.init_xyz

        if self.pf_pose is None:
            # PF가 아직 안 떴다 (맵 로딩 + 센서모델 사전계산 ~7 s). 지금 쏘면
            # PF는 구독 전이라 못 받고 gym만 차를 출발선으로 되돌린다 — 아무
            # 소용 없이 주행만 리셋된다. 살아날 때까지 그냥 기다린다.
            return

        if self.init_sent:
            # 이미 쐈다. PF 추정이 시작 포즈 근처면 seed 성공 -> 끝.
            # (실패한 경우에만 재시도한다. 파라미터 주석의 init_accept_m 참고)
            gap = math.hypot(self.pf_pose[0] - x, self.pf_pose[1] - y)
            if gap <= self.init_accept_m:
                self._finish_init(f'PF seed 확인 (시작 포즈에서 {gap:.2f} m)')
                return
            if self.init_left <= 0:
                self._finish_init(f'재시도 소진 — PF가 여전히 {gap:.2f} m 이탈')
                return

        m = PoseWithCovarianceStamped()
        m.header.stamp = self.get_clock().now().to_msg()
        m.header.frame_id = self.map_frame
        m.pose.pose.position.x = x
        m.pose.pose.position.y = y
        m.pose.pose.orientation.z = math.sin(yaw * 0.5)
        m.pose.pose.orientation.w = math.cos(yaw * 0.5)
        self.pub_init.publish(m)
        self.init_sent += 1
        self.init_left -= 1
        self.get_logger().info(
            f'PF 초기 포즈 발행 #{self.init_sent} ({x:.2f}, {y:.2f}, {yaw:.3f} rad) '
            '— gym도 이 포즈로 차를 되돌린다')


def main(args=None):
    rclpy.init(args=args)
    node = SimOdomDriftNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
