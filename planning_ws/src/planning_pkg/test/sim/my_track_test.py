#!/usr/bin/env python3
"""실차 소형 트랙 회피 검증 (기본 my_track0807).

사용: python3 my_track_test.py [<맵>]
전제: sim_race.launch.py rviz:=false map:=<맵> 이 이미 떠 있음 (맵 생략 시 기본값).
검사:
  A. /planner/ot_sectors/markers 발행 (섹터 개수 > 0 — 소형 트랙 게이팅 통과)
  B. 솔로 주행 (ego odom 이동)
  C. 레이스라인 위 정적 장애물 -> spliner 반응(otwpnts) + 통과 또는 안전정지,
     관통 금지 (center-to-center < 0.44 m = 접촉)
  D. 장애물 제거 후 재가속
"""
import glob
import math
import os
import sys
import time
from pathlib import Path

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from geometry_msgs.msg import PointStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Empty
from visualization_msgs.msg import MarkerArray
from f1tenth_planning_msgs.msg import WpntArray

# sim_race.launch.py 의 기본 맵과 같게 유지할 것 (인자로 덮어쓸 수 있다).
MAP = sys.argv[1] if len(sys.argv) > 1 else 'my_track0807'
GT = str(Path(__file__).resolve().parents[2] / 'config' / 'global_trajectory')
csvs = sorted(glob.glob(os.path.join(GT, f'{MAP}_iqp_*.csv')))
if not csvs:
    print(f'FAIL: raceline CSV 없음 ({MAP}_iqp_*.csv)', flush=True)
    sys.exit(2)
CSV = csvs[0]
print(f'[cfg] CSV={os.path.basename(CSV)}', flush=True)
D = np.genfromtxt(CSV, delimiter=';', comments='#')
S, X, Y, PSI = D[:, 0], D[:, 1], D[:, 2], D[:, 5]
MAX_S = S[-1]

results = []


def check(name, ok, detail=''):
    results.append((name, ok))
    print(('PASS' if ok else 'FAIL') + f': {name} {detail}', flush=True)


def frenet_s(x, y):
    return S[int(np.argmin(np.hypot(X - x, Y - y)))]


class STracker:
    """윈도우 최근접 s 추적 — 전역 argmin은 소형 트랙 접힘(평행 직선)에서
    반대편으로 점프해 s 매핑을 망친다."""

    def __init__(self):
        self.idx = None

    def s(self, x, y):
        n = len(S)
        if self.idx is None:
            self.idx = int(np.argmin(np.hypot(X - x, Y - y)))
        else:
            cand = np.arange(self.idx - 120, self.idx + 120) % n
            self.idx = int(cand[int(np.argmin(np.hypot(X[cand] - x, Y[cand] - y)))])
        return S[self.idx]


class T(Node):
    def __init__(self):
        super().__init__('my_track_test')
        self.ego = None
        self.sectors = None
        self.otwpnts = 0
        tl = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(Odometry, '/ego_racecar/odom', self.cb_ego, 10)
        self.create_subscription(MarkerArray, '/planner/ot_sectors/markers',
                                 self.cb_sect, tl)
        self.create_subscription(WpntArray, '/planner/avoidance/otwpnts',
                                 self.cb_ot, 10)
        self.p_click = self.create_publisher(PointStamped, '/clicked_point', 10)
        self.p_clear = self.create_publisher(Empty, '/sim/clear_obstacles', 10)

    def cb_ego(self, m):
        self.ego = m

    def cb_sect(self, m):
        self.sectors = m

    def cb_ot(self, m):
        if m.wpnts:
            self.otwpnts += 1

    def spin_for(self, sec):
        end = time.time() + sec
        while time.time() < end:
            rclpy.spin_once(self, timeout_sec=0.05)

    def click(self, x, y):
        msg = PointStamped()
        msg.header.frame_id = 'map'
        msg.point.x, msg.point.y = float(x), float(y)
        for _ in range(3):
            msg.header.stamp = self.get_clock().now().to_msg()
            self.p_click.publish(msg)
            self.spin_for(0.4)


def main():
    rclpy.init()
    t = T()

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

    # B. 솔로 주행 (ego s 진행)
    s0 = frenet_s(t.ego.pose.pose.position.x, t.ego.pose.pose.position.y)
    t.spin_for(8.0)
    s1 = frenet_s(t.ego.pose.pose.position.x, t.ego.pose.pose.position.y)
    moved = (s1 - s0) % MAX_S
    check('솔로 주행 진행', 0.5 < moved, f'(8s간 s +{moved:.1f}m)')

    # C. 추월 섹터 중심(가장 곧은 구간)에 정적 장애물 배치 — 회피가 설계된
    #    구간에서 실제로 클리어하는지 검증. 소형 루프라 ego가 여러 바퀴 돌며
    #    반복 접근. 조기중단 없이 60s 관측: (1) otwpnts 반응, (2) 근접 통과 시
    #    min_gap(관통 금지 >0.44), (3) ego s 진행 지속(영구 스턱 아님).
    tr = STracker()
    tr.s(t.ego.pose.pose.position.x, t.ego.pose.pose.position.y)
    # 섹터 s0=25.27~31.46 중심(kappa~0, clr 0.70) — 자동 산출 섹터와 일치
    obs_s = 28.36
    k = int(np.argmin(np.abs(S - obs_s)))
    ox, oy = X[k], Y[k]
    obs_s = S[k]
    t.otwpnts = 0
    t.click(ox, oy)
    min_gap = 1e9          # 실제 근접(s 기준 장애물 전후 3m 이내)일 때만 집계
    laps_seen = set()
    s_prev = None
    progressed = 0.0
    approaches = 0
    end = time.time() + 60
    while time.time() < end:
        rclpy.spin_once(t, timeout_sec=0.05)
        if not t.ego:
            continue
        p = t.ego.pose.pose.position
        s_now = tr.s(p.x, p.y)
        if s_prev is not None:
            progressed += (s_now - s_prev + MAX_S / 2) % MAX_S - MAX_S / 2
        s_prev = s_now
        ds = (s_now - obs_s) % MAX_S          # ego가 장애물 뒤 얼마나
        near = ds < 3.0 or ds > MAX_S - 3.0    # 장애물 s ±3m
        if near:
            approaches += 1
            gap = math.hypot(p.x - ox, p.y - oy)
            min_gap = min(min_gap, gap)
    reacted = t.otwpnts > 0
    check('정적 장애물 회피 반응 (otwpnts 발행)', reacted, f'({t.otwpnts} msgs)')
    # 접촉 = 장애물 반경 + 차 반폭 = 0.29 + 0.15 = 0.44 m.
    # 2026-08-24: 0.30 -> 0.44. virtual_obstacle/obstacle_size_m 이 0.3(반경
    # 0.15) 이던 시절의 값이었다. 물체를 실차와 같은 0.58 로 키웠으므로
    # 접촉선도 같이 움직인다 — 안 움직이면 실제로 박은 통과를 통과시킨다.
    check('근접 통과 시 관통 없음', approaches == 0 or min_gap > 0.44,
          f'(근접샘플 {approaches}, 최소거리 '
          f'{min_gap if min_gap < 1e8 else float("nan"):.2f}m)')
    check('장애물 존재 중 주행 지속(스턱 아님)', progressed > 5.0,
          f'(50s간 s 진행 {progressed:.1f}m)')

    # D. 제거 후 재가속
    t.p_clear.publish(Empty())
    resumed = False
    end = time.time() + 15
    while time.time() < end:
        rclpy.spin_once(t, timeout_sec=0.05)
        if t.ego and t.ego.twist.twist.linear.x > 1.5:
            resumed = True
            break
    check('장애물 제거 후 재가속', resumed)

    n_fail = sum(1 for _, ok in results if not ok)
    print(f'\n== {MAP}: {len(results) - n_fail}/{len(results)} PASS ==', flush=True)
    t.destroy_node()
    rclpy.shutdown()
    sys.exit(1 if n_fail else 0)


main()
