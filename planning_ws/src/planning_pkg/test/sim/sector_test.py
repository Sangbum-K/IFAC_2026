#!/usr/bin/env python3
"""추월 섹터 + 속도 프로파일 검증 테스트.

전제: sim_race.launch.py rviz:=false 가 이미 떠 있음 (map은 인자로 지정).
검사:
  A. /planner/ot_sectors/markers 발행 (섹터 개수 > 0)
  B. 솔로 주행 속도 변화 (max/min 비율 > 1.6 — v7 캡 시절엔 ~1.0)
  C. 정적 장애물(비섹터 좁은 구간) -> TRAILING 정지, 관통 없음
  D. 장애물 제거 후 재가속
  E. 상대 스폰 -> 섹터에서 추월 (ego가 앞으로, 관통 없음)
"""
import math
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from geometry_msgs.msg import PointStamped, PoseStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Empty, String
from visualization_msgs.msg import MarkerArray
import numpy as np

MAP = sys.argv[1] if len(sys.argv) > 1 else 'Spielberg_map'
CSV = ('/home/sangbum/IFAC_ws/planning_ws/src/planning_pkg/config/global_trajectory/'
       f'{MAP}_iqp_max15.00ms_margin0.15m_mu0.85.csv')

D = np.genfromtxt(CSV, delimiter=';', comments='#')
S, X, Y, PSI = D[:, 0], D[:, 1], D[:, 2], D[:, 5]
MAX_S = S[-1]
CLEAR = np.maximum(np.abs(D[:, 3]), np.abs(D[:, 4]))

results = []


def check(name, ok, detail=''):
    results.append((name, ok))
    print(('PASS' if ok else 'FAIL') + f': {name} {detail}', flush=True)


def frenet_s(x, y):
    i = int(np.argmin(np.hypot(X - x, Y - y)))
    return S[i]


class T(Node):
    def __init__(self):
        super().__init__('sector_test')
        self.ego = None
        self.opp = None
        self.sectors = None
        self.events = []
        tl = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(Odometry, '/ego_racecar/odom', self.cb_ego, 10)
        self.create_subscription(Odometry, '/opp_racecar/odom', self.cb_opp, 10)
        self.create_subscription(MarkerArray, '/planner/ot_sectors/markers',
                                 self.cb_sect, tl)
        self.create_subscription(String, '/pitwall/events', self.cb_ev, 50)
        self.p_click = self.create_publisher(PointStamped, '/clicked_point', 10)
        self.p_goal = self.create_publisher(PoseStamped, '/goal_pose', 10)
        self.p_clear = self.create_publisher(Empty, '/sim/clear_obstacles', 10)

    def cb_ego(self, m):
        self.ego = m

    def cb_opp(self, m):
        self.opp = m

    def cb_sect(self, m):
        self.sectors = m

    def cb_ev(self, m):
        self.events.append(m.data)

    def spin_for(self, sec):
        end = time.time() + sec
        while time.time() < end:
            rclpy.spin_once(self, timeout_sec=0.05)

    def wait_pub_ready(self, pub, sec=10):
        end = time.time() + sec
        while time.time() < end:
            if pub.get_subscription_count() >= 1:
                return True
            rclpy.spin_once(self, timeout_sec=0.1)
        return False

    def click(self, x, y):
        msg = PointStamped()
        msg.header.frame_id = 'map'
        msg.point.x, msg.point.y = float(x), float(y)
        for _ in range(3):
            msg.header.stamp = self.get_clock().now().to_msg()
            self.p_click.publish(msg)
            self.spin_for(0.4)

    def spawn_opp(self, x, y, psi):
        msg = PoseStamped()
        msg.header.frame_id = 'map'
        msg.pose.position.x, msg.pose.position.y = float(x), float(y)
        msg.pose.orientation.z = math.sin(psi / 2)
        msg.pose.orientation.w = math.cos(psi / 2)
        msg.header.stamp = self.get_clock().now().to_msg()
        self.p_goal.publish(msg)
        self.spin_for(0.5)


def main():
    rclpy.init()
    t = T()
    t.wait_pub_ready(t.p_click)
    t.wait_pub_ready(t.p_goal)

    # 스택 기동 대기
    end = time.time() + 30
    while time.time() < end and t.ego is None:
        rclpy.spin_once(t, timeout_sec=0.2)
    check('ego odom 수신', t.ego is not None)

    # A. 섹터 마커
    end = time.time() + 15
    while time.time() < end and t.sectors is None:
        rclpy.spin_once(t, timeout_sec=0.2)
    n_sect = 0
    if t.sectors:
        n_sect = sum(1 for m in t.sectors.markers if m.ns == 'ot_sectors')
    check('추월 섹터 마커 발행', n_sect > 0, f'({n_sect}개)')

    # B. 솔로 속도 변화 (35s 샘플)
    speeds = []
    end = time.time() + 35
    while time.time() < end:
        rclpy.spin_once(t, timeout_sec=0.05)
        if t.ego:
            speeds.append(t.ego.twist.twist.linear.x)
    vmax, vmin_run = max(speeds), min(v for v in speeds if v > 0.3)
    ratio = vmax / max(vmin_run, 0.1)
    check('속도 프로파일 변화', ratio > 1.6,
          f'(v {vmin_run:.1f}~{vmax:.1f} m/s, 비율 {ratio:.2f})')

    # C. 비섹터 좁은 구간 정적 장애물 -> TRAILING 정지 (Spielberg 전용)
    if MAP == 'Spielberg_map':
        nx, ny = -70.30, 45.04  # s=238.6, clearance 0.84 (비섹터)
        t.events.clear()
        t.click(nx, ny)
        min_gap = 1e9
        stopped = False
        end = time.time() + 45
        while time.time() < end:
            rclpy.spin_once(t, timeout_sec=0.05)
            if t.ego:
                p = t.ego.pose.pose.position
                gap = math.hypot(p.x - nx, p.y - ny)
                v = t.ego.twist.twist.linear.x
                if gap < min_gap:
                    min_gap = gap
                if gap < 6.0 and v < 0.3:
                    stopped = True
                    break
        trailing_ev = any('TRAILING' in e for e in t.events)
        check('비섹터 장애물 TRAILING 정지', stopped and min_gap > 0.5,
              f'(최소거리 {min_gap:.2f}m, TRAILING 이벤트={trailing_ev})')
        # D. 제거 -> 재가속
        t.p_clear.publish(Empty())
        resumed = False
        end = time.time() + 15
        while time.time() < end:
            rclpy.spin_once(t, timeout_sec=0.05)
            if t.ego and t.ego.twist.twist.linear.x > 2.0:
                resumed = True
                break
        check('장애물 제거 후 재가속', resumed)

    # E. 상대 스폰 -> 추월
    if t.ego:
        p = t.ego.pose.pose.position
        ego_s = frenet_s(p.x, p.y)
    else:
        ego_s = 0.0
    sp_s = (ego_s + 20.0) % MAX_S
    k = int(np.argmin(np.abs(S - sp_s)))
    t.events.clear()
    t.spawn_opp(X[k], Y[k], PSI[k])

    overtaken = False
    min_sep = 1e9
    was_behind = False
    end = time.time() + 90
    while time.time() < end:
        rclpy.spin_once(t, timeout_sec=0.05)
        if not (t.ego and t.opp):
            continue
        pe, po = t.ego.pose.pose.position, t.opp.pose.pose.position
        sep = math.hypot(pe.x - po.x, pe.y - po.y)
        if sep < min_sep:
            min_sep = sep
        es, os_ = frenet_s(pe.x, pe.y), frenet_s(po.x, po.y)
        ahead = (es - os_) % MAX_S
        if 2.0 < (os_ - es) % MAX_S < 40.0:
            was_behind = True
        if was_behind and 2.0 < ahead < 30.0 and sep > 1.5:
            overtaken = True
            break
    ot_ev = any('섹터' in e for e in t.events)
    check('상대 추월 성공', overtaken,
          f'(최소 이격 {min_sep:.2f}m, 섹터 이벤트={ot_ev})')
    check('추월 중 관통 없음', min_sep > 0.35, f'({min_sep:.2f}m)')

    n_fail = sum(1 for _, ok in results if not ok)
    print(f'\n== {MAP}: {len(results) - n_fail}/{len(results)} PASS ==', flush=True)
    t.destroy_node()
    rclpy.shutdown()
    sys.exit(1 if n_fail else 0)


main()
