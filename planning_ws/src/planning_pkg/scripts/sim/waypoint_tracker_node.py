#!/usr/bin/env python3
"""/local_waypoints를 추종하는 pure pursuit 컨트롤러 (시뮬레이션 전용).

f1tenth_gym_ros 시뮬레이터의 /ego_racecar/odom을 읽어 /drive
(AckermannDriveStamped)를 발행한다. spliner의 회피 궤적은
local_trajectory_node가 이미 /local_waypoints에 splice하므로
이 노드는 로컬 경로만 따라가면 회피가 이루어진다.

벽 충돌 복구: f1tenth gym은 충돌(iTTC) 시 차량 속도를 0으로 만들어
정지시키므로, 전진 명령 중인데 실제 속도가 0이면 스턱으로 판정하고
경로 방향으로 기수가 돌아가도록 후진했다가 재개한다 (TRACK <-> REVERSE).

ego 제어 모드 (unicorn Pitwall 패널의 Joy/Keyboard 대응):
  - autonomous (기본): 이 노드가 /drive 발행
  - manual: /drive 발행 중단 (1회 정지 발행 후 침묵) -> gym kb_teleop
    (/cmd_vel, teleop_twist_keyboard)이 차를 조종한다
  전환: /sim/ego_mode (String "autonomous"|"manual", 패널 버튼)
        또는 /joy — LB(버튼4)=manual, RB(버튼5)=autonomous
  현재 모드는 /sim/ego_status (String)로 발행 (패널 표시용).

실차 통합 시 control 팀 노드(control_pkg)로 대체될 자리이며,
그 전까지 RViz 시뮬 데모 용도. (실차의 모드 전환은 control_node의
/control/set_mode 서비스가 담당 — 개념 동일.)
"""
import math

import numpy as np
import rclpy
from rclpy.node import Node

from ackermann_msgs.msg import AckermannDriveStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Joy
from std_msgs.msg import String

from f1tenth_planning_msgs.msg import WpntArray

import pitwall


class WaypointTrackerNode(Node):
    def __init__(self):
        super().__init__('waypoint_tracker_node')
        pitwall.init(self)

        self.declare_parameter('tracker/t_lookahead', 0.5)     # [s] 속도 비례 lookahead
        self.declare_parameter('tracker/min_lookahead_m', 1.2)
        self.declare_parameter('tracker/max_lookahead_m', 3.0)
        self.declare_parameter('tracker/wheelbase_m', 0.33)
        self.declare_parameter('tracker/max_steer_rad', 0.4)
        self.declare_parameter('tracker/speed_scale', 0.5)
        self.declare_parameter('tracker/waypoint_timeout_sec', 0.5)
        # 벽 충돌 복구
        self.declare_parameter('tracker/stuck_detect_sec', 0.6)     # 이 시간 동안 못 움직이면 스턱
        self.declare_parameter('tracker/reverse_speed_mps', 1.2)    # 후진 속도 (양수로 지정)
        self.declare_parameter('tracker/reverse_max_sec', 3.0)      # 최대 후진 시간
        self.declare_parameter('tracker/reverse_min_dist_m', 0.3)   # 최소 후진 거리
        self.declare_parameter('tracker/resume_speed_cap_mps', 2.0) # 복구 직후 속도 제한
        self.declare_parameter('tracker/resume_cap_sec', 1.5)
        gp = lambda n: self.get_parameter(n).value
        self.t_la = gp('tracker/t_lookahead')
        self.min_la = gp('tracker/min_lookahead_m')
        self.max_la = gp('tracker/max_lookahead_m')
        self.wheelbase = gp('tracker/wheelbase_m')
        self.max_steer = gp('tracker/max_steer_rad')
        self.speed_scale = gp('tracker/speed_scale')
        self.wpnt_timeout = gp('tracker/waypoint_timeout_sec')
        self.stuck_detect_sec = gp('tracker/stuck_detect_sec')
        self.reverse_speed = gp('tracker/reverse_speed_mps')
        self.reverse_max_sec = gp('tracker/reverse_max_sec')
        self.reverse_min_dist = gp('tracker/reverse_min_dist_m')
        self.resume_speed_cap = gp('tracker/resume_speed_cap_mps')
        self.resume_cap_sec = gp('tracker/resume_cap_sec')

        self.wpnts = None        # np.array [N, 3]: x, y, v
        self.wpnts_stamp = None

        # 복구 상태머신
        self.mode = 'TRACK'      # TRACK | REVERSE
        self.stuck_since = None      # 정지 감지 시작 시각
        self.reverse_start_t = None
        self.reverse_start_xy = None
        self.resume_until = None     # 복구 직후 속도 제한 해제 시각

        # ego 제어 모드 (autonomous | manual)
        self.ego_mode = 'autonomous'
        self.manual_stop_sent = False

        self.create_subscription(WpntArray, '/local_waypoints', self.cb_wpnts, 10)
        self.create_subscription(Odometry, '/ego_racecar/odom', self.cb_odom, 10)
        self.create_subscription(String, '/sim/ego_mode', self.cb_ego_mode, 10)
        self.create_subscription(Joy, '/joy', self.cb_joy, 10)
        self.pub_drive = self.create_publisher(AckermannDriveStamped, '/drive', 10)
        self.pub_status = self.create_publisher(String, '/sim/ego_status', 10)
        self.create_timer(0.5, self.publish_status)

        self.get_logger().info('Waypoint tracker (pure pursuit + stuck recovery) ready')

    def set_ego_mode(self, mode):
        if mode == self.ego_mode:
            return
        self.ego_mode = mode
        self.manual_stop_sent = False
        self.get_logger().info(f'ego 제어 모드: {mode}')
        pitwall.event(f'ego 모드 전환: {mode}')

    def cb_ego_mode(self, msg: String):
        mode = msg.data.strip().lower()
        if mode in ('autonomous', 'manual'):
            self.set_ego_mode(mode)

    def cb_joy(self, msg: Joy):
        # unicorn 패널과 동일: LB(4)=manual, RB(5)=autonomous
        if len(msg.buttons) > 5:
            if msg.buttons[4]:
                self.set_ego_mode('manual')
            elif msg.buttons[5]:
                self.set_ego_mode('autonomous')

    def publish_status(self):
        msg = String()
        state = self.mode if self.ego_mode == 'autonomous' else 'MANUAL'
        msg.data = f'{self.ego_mode} | {state}'
        self.pub_status.publish(msg)

    def cb_wpnts(self, msg: WpntArray):
        if msg.wpnts:
            self.wpnts = np.array([[w.x_m, w.y_m, w.vx_mps] for w in msg.wpnts])
            self.wpnts_stamp = self.get_clock().now()

    def now_sec(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def path_heading_error(self, cx, cy, yaw):
        """차량 헤딩과 로컬 경로 진행 방향의 각도차 [-pi, pi]."""
        dists = np.hypot(self.wpnts[:, 0] - cx, self.wpnts[:, 1] - cy)
        i = int(np.argmin(dists))
        j = min(i + 5, len(self.wpnts) - 1)   # ~0.5m 앞 점으로 방향 계산
        if j == i:
            return 0.0
        path_yaw = math.atan2(self.wpnts[j, 1] - self.wpnts[i, 1],
                              self.wpnts[j, 0] - self.wpnts[i, 0])
        err = path_yaw - yaw
        while err > math.pi:
            err -= 2.0 * math.pi
        while err < -math.pi:
            err += 2.0 * math.pi
        return err

    def cb_odom(self, msg: Odometry):
        cmd = AckermannDriveStamped()
        cmd.header.stamp = msg.header.stamp
        cmd.header.frame_id = 'base_link'

        # manual 모드: 정지 1회 발행 후 침묵 -> kb teleop(/cmd_vel)이 조종
        if self.ego_mode == 'manual':
            if not self.manual_stop_sent:
                self.pub_drive.publish(cmd)
                self.manual_stop_sent = True
            self.stuck_since = None
            self.mode = 'TRACK'
            return

        # 로컬 경로가 없거나 오래됐으면 정지 (안전)
        if (self.wpnts is None or self.wpnts_stamp is None or
                (self.get_clock().now() - self.wpnts_stamp).nanoseconds * 1e-9
                > self.wpnt_timeout):
            self.pub_drive.publish(cmd)  # speed/steer 0
            return

        cx = msg.pose.pose.position.x
        cy = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                         1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        v = msg.twist.twist.linear.x
        t = self.now_sec()

        if self.mode == 'REVERSE':
            heading_err = self.path_heading_error(cx, cy, yaw)
            reversed_dist = math.hypot(cx - self.reverse_start_xy[0],
                                       cy - self.reverse_start_xy[1])
            elapsed = t - self.reverse_start_t
            done = (abs(heading_err) < 0.8 and reversed_dist > self.reverse_min_dist) \
                or elapsed > self.reverse_max_sec
            if done:
                self.mode = 'TRACK'
                self.stuck_since = None
                self.resume_until = t + self.resume_cap_sec
                self.get_logger().info(
                    f'복구 완료: {reversed_dist:.2f}m 후진, heading_err={heading_err:.2f}rad — 주행 재개')
                pitwall.event(f'복구 완료 ({reversed_dist:.2f}m 후진)')
            else:
                # 후진 중: 기수가 경로 방향으로 돌도록 조향
                # (후진 시 yaw_rate = v/L*tan(δ), v<0 이므로 δ는 목표 회전의 반대 부호)
                cmd.drive.steering_angle = float(
                    -np.sign(heading_err) * min(self.max_steer, 0.35))
                cmd.drive.speed = -abs(self.reverse_speed)
                self.pub_drive.publish(cmd)
                return

        # --- TRACK: pure pursuit ---
        la = float(np.clip(self.t_la * abs(v), self.min_la, self.max_la))

        # 로컬 경로는 차량 앞으로 정렬돼 있음 -> lookahead 이상 떨어진 첫 점
        dists = np.hypot(self.wpnts[:, 0] - cx, self.wpnts[:, 1] - cy)
        idx_candidates = np.where(dists >= la)[0]
        target_idx = int(idx_candidates[0]) if len(idx_candidates) else len(dists) - 1
        tx, ty, tv = self.wpnts[target_idx]

        # base_link 기준 목표점 -> pure pursuit 곡률
        dx, dy = tx - cx, ty - cy
        y_b = -dx * math.sin(yaw) + dy * math.cos(yaw)
        L = max(math.hypot(dx, dy), 1e-3)
        curvature = 2.0 * y_b / (L * L)
        steer = math.atan(self.wheelbase * curvature)

        speed = tv * self.speed_scale
        # 복구 직후에는 천천히 재가속
        if self.resume_until is not None and t < self.resume_until:
            speed = min(speed, self.resume_speed_cap)

        cmd.drive.steering_angle = float(np.clip(steer, -self.max_steer, self.max_steer))
        cmd.drive.speed = float(speed)
        self.pub_drive.publish(cmd)
        pitwall.log('cmd_speed', cmd.drive.speed)
        pitwall.log('cmd_steer', cmd.drive.steering_angle)
        pitwall.log('lookahead', la)

        # --- 스턱 감지: 전진 명령 중인데 실제로 못 움직임 (gym 충돌 시 v=0 고정) ---
        if speed > 0.3 and abs(v) < 0.1:
            if self.stuck_since is None:
                self.stuck_since = t
            elif t - self.stuck_since > self.stuck_detect_sec:
                self.mode = 'REVERSE'
                self.reverse_start_t = t
                self.reverse_start_xy = (cx, cy)
                self.stuck_since = None
                self.get_logger().warn(
                    f'벽 충돌/스턱 감지 (cmd={speed:.1f} m/s, 실제={v:.2f} m/s) — 후진 복구 시작')
                pitwall.event('벽 충돌/스턱 — 후진 복구 시작')
        else:
            self.stuck_since = None


def main():
    rclpy.init()
    rclpy.spin(WaypointTrackerNode())
    rclpy.shutdown()


if __name__ == '__main__':
    main()
