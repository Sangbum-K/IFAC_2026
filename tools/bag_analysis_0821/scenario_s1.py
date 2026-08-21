#!/usr/bin/env python3
"""S1 — 봉쇄 정지 시나리오 (PLAN.md §5).

  달리던 상대가 정적 장애물 바로 옆에 나란히 멈춰 모든 라인이 막혔을 때
  자차가 설 수 있는가.

절차 (전부 자동):
  1) 지정 s 위치 옆에 콘 1개를 /clicked_point 로 놓는다.
  2) 상대(/perception/opponent 의 s_center)가 그 s 에 도달하면
     /sim/opp_speed_delta 로 최고속도를 0 으로 만들어 **그 자리에** 세운다.
  3) HOLD 초 유지 -> 다시 최고속도를 올려 재출발시킨다.
  4) 그동안 자차의 최소 차체 여유와 발행 속도를 기록한다.

사용:
  python3 scenario_s1.py [--s 31.03] [--cone-d 0.47] [--hold 30]
"""
import argparse, math, sys, time

import rclpy
from rclpy.node import Node
from ackermann_msgs.msg import AckermannDriveStamped
from geometry_msgs.msg import PointStamped, PoseStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32
from f1tenth_planning_msgs.msg import ObstacleArray, WpntArray


class ScenarioS1(Node):
    def __init__(self, a):
        super().__init__('scenario_s1')
        self.a = a
        self.gb = None
        self.state = 'WAIT_WPNTS'
        self.t_state = time.time()
        self.min_slack = 9.9
        self.min_cmd = 9.9
        self.ego = None
        self.opp = (0.0, 0.0, 0.0)
        self.opp_seen = False
        self.spawn_tries = 0
        self.NEED, self.R = 0.55, 0.17
        self.create_subscription(WpntArray, '/global_waypoints', self.cb_gb, 1)
        self.create_subscription(ObstacleArray, '/perception/opponent', self.cb_opp, 10)
        self.create_subscription(Odometry, '/car_state/frenet/odom', self.cb_ego, 10)
        # 정식 스택(stack.launch.py)의 명령 토픽은 /drive 다.
        # /pitwall/cmd_speed 는 파이썬 트래커(tracker:=true) 전용이라 여기선 안 온다.
        self.create_subscription(AckermannDriveStamped, '/drive', self.cb_cmd, 10)
        self.p_click = self.create_publisher(PointStamped, '/clicked_point', 10)
        # RViz '2D Goal Pose' = /goal_pose = gym_bridge 가 상대를 그 자리로 텔레포트.
        # --no-rviz 로 돌리므로 직접 쏜다 (주차장 반경 5 m 밖이면 '스폰'으로 친다).
        self.p_goal = self.create_publisher(PoseStamped, '/goal_pose', 10)
        self.spawned = False
        self.p_delta = self.create_publisher(Float32, '/sim/opp_speed_delta', 10)
        self.create_timer(0.2, self.step)
        self.get_logger().info('S1 준비 — /global_waypoints 대기')

    def cb_gb(self, m):
        if self.gb is None and m.wpnts:
            self.gb = [(w.s_m, w.x_m, w.y_m, w.d_right, w.d_left, w.psi_rad)
                       for w in m.wpnts]
            self.get_logger().info(f'레이스라인 {len(self.gb)}점 수신')

    def cb_ego(self, m):
        self.ego = (m.pose.pose.position.x, m.pose.pose.position.y)   # s, d

    def cb_cmd(self, m):
        if self.state == 'STOPPED':
            self.min_cmd = min(self.min_cmd, m.drive.speed)

    def cb_opp(self, m):
        if m.obstacles:
            self.opp_seen = True      # 주차장 밖 = 스폰됨
        if not m.obstacles or self.gb is None:
            return
        o = m.obstacles[0]
        self.opp = (o.s_center, o.d_center, o.vs)
        self.opp = (o.s_center, o.d_center, o.vs)
        if self.state == 'ARMED':
            ds = (o.s_center - self.a.s) % self.track_len
            if ds > self.track_len / 2:
                ds -= self.track_len
            if abs(ds) < 0.35:
                self.p_delta.publish(Float32(data=-10.0))
                self.state, self.t_state = 'SETTLING', time.time()
                self.get_logger().warn(
                    f'상대 정지 명령 — s={o.s_center:.2f} d={o.d_center:+.2f} '
                    f'(타행 끝날 때까지 {self.a.settle}s 대기 후 콘 배치)')
        if self.state in ('SETTLING', 'STOPPED') and self.ego is not None:
            ds = (o.s_center - self.ego[0]) % self.track_len
            if ds > self.track_len / 2:
                ds -= self.track_len
            if abs(ds) < 0.45:
                slack = abs(o.d_center - self.ego[1]) - o.size / 2.0 - 0.15
                self.min_slack = min(self.min_slack, slack)

    @property
    def track_len(self):
        return self.gb[-1][0] + (self.gb[1][0] - self.gb[0][0])

    def step(self):
        if self.gb is None:
            return
        now = time.time()

        if self.state == 'WAIT_WPNTS':
            # 상대를 봉쇄지점 한참 앞에 스폰해 레이스라인을 따라 달려오게 한다.
            j = min(range(len(self.gb)),
                    key=lambda k: abs(self.gb[k][0] -
                                      (self.a.s - self.a.spawn_lead) % self.track_len))
            _s2, gx, gy, _dr, _dl, gpsi = self.gb[j]
            g = PoseStamped()
            g.header.frame_id = 'map'
            g.header.stamp = self.get_clock().now().to_msg()
            g.pose.position.x, g.pose.position.y = float(gx), float(gy)
            g.pose.orientation.z = float(math.sin(gpsi / 2.0))
            g.pose.orientation.w = float(math.cos(gpsi / 2.0))
            # ⚠️ 한 번만 쏘면 유실된다. gym_bridge 구독자가 아직 안 붙었으면
            #    기본 QoS 가 조용히 버린다 — S1-B 가 그래서 상대 없이 4분을 돌았다.
            #    /perception/opponent 가 비어 있지 않을 때까지 계속 쏜다.
            self.p_goal.publish(g)
            self.spawn_tries += 1
            if not self.opp_seen:
                if self.spawn_tries % 20 == 1:
                    self.get_logger().warn(
                        f'상대 스폰 시도 {self.spawn_tries} — s={_s2:.2f}')
                if self.spawn_tries > 150:
                    self.get_logger().error('상대 스폰 실패 — 중단')
                    raise SystemExit(2)
                return
            self.get_logger().warn(f'상대 스폰 확인 (시도 {self.spawn_tries}회)')
            self.state, self.t_state = 'ARMED', now
            self.get_logger().warn(
                f'상대 스폰 s={_s2:.2f} map({gx:.2f},{gy:.2f}) — '
                f's={self.a.s:.2f} 에 오면 세운다')

        elif self.state == 'SETTLING':
            # 정지 명령 후 타행이 끝나기를 기다린다. 실측 2.2 m 미끄러졌다.
            # 콘을 미리 놓으면 상대가 콘을 지나쳐 서서 봉쇄가 성립하지 않는다.
            if now - self.t_state < self.a.settle:
                return
            s_stop, d_stop = self.opp[0], self.opp[1]
            i = min(range(len(self.gb)), key=lambda k: abs(self.gb[k][0] - s_stop))
            _s3, x, y, dr, dl, psi = self.gb[i]
            # 상대가 막지 못한 쪽(여유가 큰 쪽)을 콘으로 막는다.
            right_free = dr + d_stop - 0.15
            left_free = dl - d_stop - 0.15
            if left_free >= right_free:
                d = dl - self.NEED - self.R + 0.10          # 좌측을 막는다
                other = right_free
            else:
                d = -(dr - self.NEED - self.R + 0.10)       # 우측을 막는다
                other = left_free
            cx = x - d * math.sin(psi)
            cy = y + d * math.cos(psi)
            p = PointStamped()
            p.header.frame_id = 'map'
            p.header.stamp = self.get_clock().now().to_msg()
            p.point.x, p.point.y = float(cx), float(cy)
            self.p_click.publish(p)
            self.state, self.t_state = 'STOPPED', now
            self.get_logger().warn(
                f'상대 정지 확정 s={s_stop:.2f} d={d_stop:+.2f} | '
                f'콘 d={d:+.2f} map({cx:.3f},{cy:.3f}) | '
                f'벽 d_r={dr:.2f} d_l={dl:.2f} | 반대쪽 여유 {other:.2f} '
                f'(필요 {self.NEED}) -> {"봉쇄 성립" if other < self.NEED else "★봉쇄 실패★"}')

        elif self.state == 'STOPPED' and now - self.t_state > self.a.hold:
            self.p_delta.publish(Float32(data=+10.0))
            self.state, self.t_state = 'RELEASED', now
            self.get_logger().warn(
                f'상대 재출발 | 정지 구간 최소 차체여유 {self.min_slack:+.3f} m, '
                f'자차 최소 명령속도 {self.min_cmd:.2f} m/s')

        elif self.state == 'RELEASED' and now - self.t_state > 15:
            self.get_logger().warn('S1 종료')
            raise SystemExit(0)


def main():
    p = argparse.ArgumentParser()
    p.add_argument('--s', type=float, default=31.03, help='봉쇄 지점 s [m]')
    p.add_argument('--cone-d', type=float, default=None,
                   help='콘의 d [m]. 기본은 좌측 여유가 0.45 가 되도록 자동')
    p.add_argument('--hold', type=float, default=30.0, help='상대 정지 유지 [s]')
    p.add_argument('--settle', type=float, default=4.0,
                   help='정지 명령 후 타행이 끝나기를 기다리는 시간 [s]')
    p.add_argument('--spawn-lead', type=float, default=12.0,
                   dest='spawn_lead', help='봉쇄지점 몇 m 앞에 상대를 스폰할지')
    a = p.parse_args()
    rclpy.init()
    n = ScenarioS1(a)
    try:
        rclpy.spin(n)
    except SystemExit:
        pass
    finally:
        n.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
