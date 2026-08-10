#!/usr/bin/env python3
"""/local_waypoints를 추종하는 pure pursuit 컨트롤러 (시뮬레이션 전용).

f1tenth_gym_ros 시뮬레이터의 /ego_racecar/odom을 읽어 /drive
(AckermannDriveStamped)를 발행한다. spliner의 회피 궤적은
local_trajectory_node가 이미 /local_waypoints에 splice하므로
이 노드는 로컬 경로만 따라가면 회피가 이루어진다.

복구 명령은 /planner/recovery/state와 target_speed를 짧은 paired lease로
검증해 소비한다. 후방 coverage와 전용 후진 경로가 없는 시뮬 tracker의
기본 동작은 STOP-only다. 과거의 blind/internal stuck 후진은 별도 opt-in
파라미터를 켠 경우에만 제한적으로 사용할 수 있다.

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
from std_msgs.msg import Bool, Float64, String

from f1tenth_planning_msgs.msg import WpntArray

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


class WaypointTrackerNode(Node):
    def __init__(self):
        super().__init__('waypoint_tracker_node')
        pitwall.init(self)

        self.declare_parameter('tracker/t_lookahead', 0.5)     # [s] 속도 비례 lookahead
        self.declare_parameter('tracker/min_lookahead_m', 0.6)
        self.declare_parameter('tracker/max_lookahead_m', 1.2)
        self.declare_parameter('tracker/wheelbase_m', 0.33)
        self.declare_parameter('tracker/max_steer_rad', 0.36)
        self.declare_parameter('tracker/speed_scale', 0.30)
        self.declare_parameter('tracker/max_steering_rate_radps', 2.0)
        self.declare_parameter('tracker/max_accel_mps2', 5.5)
        self.declare_parameter('tracker/max_decel_mps2', 7.0)
        self.declare_parameter('tracker/min_command_dt_sec', 0.001)
        self.declare_parameter('tracker/max_command_dt_sec', 0.10)
        self.declare_parameter('tracker/waypoint_timeout_sec', 0.5)
        self.declare_parameter('tracker/odom_timeout_sec', 0.5)
        self.declare_parameter('tracker/require_source_stamps', True)
        self.declare_parameter('tracker/source_future_tolerance_sec', 0.1)
        self.declare_parameter('tracker/safety_watchdog_hz', 20.0)
        # Recovery supervisor contract. 기본 스택은 recovery 노드가 선택적이므로
        # required=false지만, state/target 중 하나라도 본 뒤에는 fail-closed다.
        self.declare_parameter(
            'tracker/recovery_state_topic', '/planner/recovery/state')
        self.declare_parameter(
            'tracker/recovery_target_speed_topic',
            '/planner/recovery/target_speed')
        self.declare_parameter(
            'tracker/estop_topic', '/safety/emergency_stop')
        self.declare_parameter('tracker/recovery_required', False)
        self.declare_parameter('tracker/recovery_command_timeout_sec', 0.25)
        self.declare_parameter('tracker/recovery_pair_max_skew_sec', 0.10)
        self.declare_parameter('tracker/recovery_allow_reverse', False)
        self.declare_parameter('tracker/recovery_max_reverse_speed_mps', 0.30)
        self.declare_parameter(
            'tracker/recovery_max_forward_override_mps', 1.0)
        # 벽 충돌 감지 (감지 후 동작은 STOP 래치 — 후진은 supervisor 몫)
        self.declare_parameter('tracker/stuck_detect_sec', 0.6)     # 이 시간 동안 못 움직이면 스턱
        gp = lambda n: self.get_parameter(n).value
        self.t_la = gp('tracker/t_lookahead')
        self.min_la = gp('tracker/min_lookahead_m')
        self.max_la = gp('tracker/max_lookahead_m')
        self.wheelbase = gp('tracker/wheelbase_m')
        self.max_steer = gp('tracker/max_steer_rad')
        self.speed_scale = gp('tracker/speed_scale')
        self.max_steering_rate = float(
            gp('tracker/max_steering_rate_radps'))
        self.max_accel = float(gp('tracker/max_accel_mps2'))
        self.max_decel = float(gp('tracker/max_decel_mps2'))
        self.min_command_dt = float(gp('tracker/min_command_dt_sec'))
        self.max_command_dt = float(gp('tracker/max_command_dt_sec'))
        self.motion_limits_valid = (
            math.isfinite(self.max_steering_rate) and
            math.isfinite(self.max_accel) and
            math.isfinite(self.max_decel) and
            math.isfinite(self.min_command_dt) and
            math.isfinite(self.max_command_dt) and
            self.max_steering_rate >= 0.0 and
            self.max_accel >= 0.0 and
            self.max_decel >= 0.0 and
            self.min_command_dt > 0.0 and
            self.max_command_dt >= self.min_command_dt)
        self.wpnt_timeout = max(float(gp('tracker/waypoint_timeout_sec')), 0.01)
        self.odom_timeout = max(float(gp('tracker/odom_timeout_sec')), 0.01)
        self.require_source_stamps = bool(
            gp('tracker/require_source_stamps'))
        self.source_future_tolerance = max(
            float(gp('tracker/source_future_tolerance_sec')), 0.0)
        self.safety_watchdog_hz = max(
            float(gp('tracker/safety_watchdog_hz')), 1.0)
        if not math.isfinite(self.safety_watchdog_hz):
            self.safety_watchdog_hz = 20.0
        self.recovery_state_topic = gp('tracker/recovery_state_topic')
        self.recovery_target_speed_topic = gp(
            'tracker/recovery_target_speed_topic')
        self.estop_topic = gp('tracker/estop_topic')
        self.recovery_required = bool(gp('tracker/recovery_required'))
        self.recovery_command_timeout = max(
            float(gp('tracker/recovery_command_timeout_sec')), 0.01)
        self.recovery_pair_max_skew = min(
            max(float(gp('tracker/recovery_pair_max_skew_sec')), 0.0),
            self.recovery_command_timeout)
        self.recovery_allow_reverse = bool(
            gp('tracker/recovery_allow_reverse'))
        self.recovery_max_reverse_speed = max(
            float(gp('tracker/recovery_max_reverse_speed_mps')), 0.0)
        if not math.isfinite(self.recovery_max_reverse_speed):
            self.recovery_max_reverse_speed = 0.0
        self.recovery_max_forward_override = max(
            float(gp('tracker/recovery_max_forward_override_mps')), 0.0)
        if not math.isfinite(self.recovery_max_forward_override):
            self.recovery_max_forward_override = 0.0
        self.stuck_detect_sec = max(
            float(gp('tracker/stuck_detect_sec')), 0.0)

        self.wpnts = None        # np.array [N, 3]: x, y, v
        self.wpnts_stamp = None
        self.wpnts_source_stamp = None
        self.odom_stamp = None
        self.odom_source_stamp = None

        # Recovery paired lease. std_msgs에는 header가 없으므로 각 callback의
        # 수신 시각과 두 토픽 간 skew를 함께 검사한다.
        self.recovery_state = None
        self.recovery_state_stamp = None
        self.recovery_target_speed = math.nan
        self.recovery_target_stamp = None
        self.estop = False
        self.last_safety_log = None
        self.last_safety_log_t = -math.inf
        self.last_command_stamp = None
        self.last_command_speed = 0.0
        self.last_command_steer = 0.0

        # 복구 상태머신
        self.mode = 'TRACK'      # TRACK | STOP
        self.stuck_since = None      # 정지 감지 시작 시각

        # ego 제어 모드 (autonomous | manual)
        self.ego_mode = 'autonomous'
        self.manual_stop_sent = False

        self.create_subscription(WpntArray, '/local_waypoints', self.cb_wpnts, 10)
        self.create_subscription(Odometry, '/ego_racecar/odom', self.cb_odom, 10)
        self.create_subscription(
            String, self.recovery_state_topic, self.cb_recovery_state, 10)
        self.create_subscription(
            Float64, self.recovery_target_speed_topic,
            self.cb_recovery_target_speed, 10)
        self.create_subscription(Bool, self.estop_topic, self.cb_estop, 10)
        self.create_subscription(String, '/sim/ego_mode', self.cb_ego_mode, 10)
        self.create_subscription(Joy, '/joy', self.cb_joy, 10)
        self.pub_drive = self.create_publisher(AckermannDriveStamped, '/drive', 10)
        self.pub_status = self.create_publisher(String, '/sim/ego_status', 10)
        self.create_timer(0.5, self.publish_status)
        self.create_timer(
            1.0 / self.safety_watchdog_hz, self.safety_watchdog)

        self.get_logger().info(
            'Waypoint tracker ready: recovery lease '
            f'required={self.recovery_required}, '
            f'reverse={"OPT-IN" if self.recovery_allow_reverse else "OFF"}, '
            f'steer_rate={self.max_steering_rate:.2f}rad/s, '
            f'accel/decel={self.max_accel:.2f}/{self.max_decel:.2f}m/s^2')
        if not self.motion_limits_valid:
            self.get_logger().error(
                'Invalid tracker motion/dt limits — normal motion will remain stopped')

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

    @staticmethod
    def source_stamp_sec(stamp):
        return float(stamp.sec) + float(stamp.nanosec) * 1e-9

    def safety_log(self, reason):
        now = self.now_sec()
        if reason != self.last_safety_log or now - self.last_safety_log_t >= 2.0:
            self.get_logger().error(f'{reason} -> stop')
            self.last_safety_log = reason
            self.last_safety_log_t = now

    def publish_stop(self):
        now = self.now_sec()
        cmd = AckermannDriveStamped()
        cmd.header.stamp = self.get_clock().now().to_msg()
        cmd.header.frame_id = 'base_link'
        self.pub_drive.publish(cmd)
        self.last_command_stamp = now
        self.last_command_speed = 0.0
        self.last_command_steer = 0.0

    def publish_immediate_command(self, cmd, receipt_time):
        """Publish an already safety-qualified command without slew limiting."""
        speed = float(cmd.drive.speed)
        steer = float(cmd.drive.steering_angle)
        if (not math.isfinite(receipt_time) or
                not math.isfinite(speed) or not math.isfinite(steer)):
            self.safety_log('Immediate command/time is non-finite')
            self.publish_stop()
            return False
        self.pub_drive.publish(cmd)
        self.last_command_stamp = receipt_time
        self.last_command_speed = speed
        self.last_command_steer = steer
        return True

    def apply_motion_limits(self, target_speed, target_steer, receipt_time):
        """Limit normal TRACK/ALIGN commands using callback receipt-time dt."""
        values = (target_speed, target_steer, receipt_time,
                  self.last_command_speed, self.last_command_steer)
        if (not self.motion_limits_valid or
                not all(math.isfinite(value) for value in values)):
            self.safety_log('Motion limit state/target is non-finite or invalid')
            return None
        if self.last_command_stamp is None:
            self.safety_log('Command dt unavailable on first command')
            return None

        dt = receipt_time - self.last_command_stamp
        if (not math.isfinite(dt) or
                dt < self.min_command_dt or dt > self.max_command_dt):
            self.safety_log(
                f'Abnormal command dt={dt!r}s '
                f'(expected {self.min_command_dt:.4f}..'
                f'{self.max_command_dt:.3f})')
            return None

        steer_delta_max = self.max_steering_rate * dt
        limited_steer = self.last_command_steer + float(np.clip(
            target_steer - self.last_command_steer,
            -steer_delta_max, steer_delta_max))

        speed_delta = target_speed - self.last_command_speed
        if speed_delta >= 0.0:
            limited_delta = min(speed_delta, self.max_accel * dt)
        else:
            limited_delta = max(speed_delta, -self.max_decel * dt)
        limited_speed = self.last_command_speed + limited_delta

        if not math.isfinite(limited_speed) or not math.isfinite(limited_steer):
            self.safety_log('Slew limiter produced a non-finite command')
            return None
        return limited_speed, limited_steer

    def stamp_fresh(self, receipt_stamp, source_stamp, timeout):
        now = self.now_sec()

        def valid(stamp):
            if stamp is None:
                return False
            age = now - stamp
            return (math.isfinite(age) and
                    age >= -self.source_future_tolerance and
                    age <= timeout)

        if not valid(receipt_stamp):
            return False
        if source_stamp is None or source_stamp <= 0.0:
            return not self.require_source_stamps
        return valid(source_stamp)

    def recovery_contract_active(self):
        return (self.recovery_required or
                self.recovery_state_stamp is not None or
                self.recovery_target_stamp is not None)

    def recovery_directive(self):
        """Return (NONE|DRIVING|STOP|REVERSE|ALIGN, target_speed)."""
        if not self.recovery_contract_active():
            return 'NONE', None

        now = self.now_sec()

        def lease_fresh(stamp):
            if stamp is None:
                return False
            age = now - stamp
            return (math.isfinite(age) and
                    age >= -self.source_future_tolerance and
                    age <= self.recovery_command_timeout)

        pair_fresh = (
            lease_fresh(self.recovery_state_stamp) and
            lease_fresh(self.recovery_target_stamp) and
            abs(self.recovery_state_stamp - self.recovery_target_stamp)
            <= self.recovery_pair_max_skew)
        if not pair_fresh:
            self.safety_log('Recovery command pair missing/stale/skewed')
            return 'STOP', 0.0

        state = self.recovery_state
        target = self.recovery_target_speed
        stop_states = {
            'WAITING_FOR_INPUTS', 'STUCK_DETECTED', 'COLLISION_STOP',
            'RECOVER_STOP', 'STOP'}
        if state in stop_states:
            if not math.isfinite(target):
                self.safety_log(
                    'Recovery stop state has non-finite target '
                    '(stop remains authoritative)')
            return 'STOP', 0.0
        if state == 'DRIVING':
            # NaN은 이 상태에서만 의도된 no-override sentinel이다.
            if not math.isnan(target):
                self.safety_log(
                    'Recovery DRIVING state must carry NaN target')
                return 'STOP', 0.0
            return 'DRIVING', None
        if state == 'RECOVER_REVERSE':
            if (not self.recovery_allow_reverse or
                    not math.isfinite(target) or target >= 0.0 or
                    self.recovery_max_reverse_speed <= 0.0):
                self.safety_log(
                    'Recovery reverse rejected '
                    '(not opted-in or invalid target)')
                return 'STOP', 0.0
            return 'REVERSE', -min(
                abs(target), self.recovery_max_reverse_speed)
        if state == 'RECOVER_ALIGN':
            if not math.isfinite(target) or target < 0.0:
                self.safety_log('Recovery align target invalid')
                return 'STOP', 0.0
            return 'ALIGN', min(
                target, self.recovery_max_forward_override)

        self.safety_log(f'Unknown recovery state {state!r}')
        return 'STOP', 0.0

    def cb_recovery_state(self, msg: String):
        self.recovery_state = msg.data
        self.recovery_state_stamp = self.now_sec()
        # STOP 계열은 target callback 순서를 기다리지 않고 즉시 정지한다.
        if self.recovery_state in {
                'WAITING_FOR_INPUTS', 'STUCK_DETECTED', 'COLLISION_STOP',
                'RECOVER_STOP', 'STOP'}:
            self.publish_stop()

    def cb_recovery_target_speed(self, msg: Float64):
        self.recovery_target_speed = float(msg.data)
        self.recovery_target_stamp = self.now_sec()

    def cb_estop(self, msg: Bool):
        self.estop = bool(msg.data)
        if self.estop:
            self.mode = 'TRACK'
            self.stuck_since = None
            self.publish_stop()

    def publish_status(self):
        msg = String()
        if self.estop:
            state = 'E_STOP'
        elif self.ego_mode != 'autonomous':
            state = 'MANUAL'
        elif self.recovery_contract_active() and self.recovery_state is not None:
            state = self.recovery_state
        else:
            state = self.mode
        msg.data = f'{self.ego_mode} | {state}'
        self.pub_status.publish(msg)

    def cb_wpnts(self, msg: WpntArray):
        self.wpnts_stamp = self.now_sec()
        self.wpnts_source_stamp = self.source_stamp_sec(msg.header.stamp)
        if not msg.wpnts:
            self.wpnts = None
            return
        wpnts = np.array(
            [[w.x_m, w.y_m, w.vx_mps] for w in msg.wpnts],
            dtype=float)
        if not np.all(np.isfinite(wpnts)):
            self.wpnts = None
            self.safety_log('Local waypoints contain non-finite values')
            return
        self.wpnts = wpnts

    def now_sec(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def safety_watchdog(self):
        if self.estop:
            self.publish_stop()
            return

        directive, _ = self.recovery_directive()
        if directive == 'STOP':
            self.publish_stop()
            return
        if self.ego_mode != 'autonomous':
            return
        if not self.stamp_fresh(
                self.odom_stamp, self.odom_source_stamp, self.odom_timeout):
            self.safety_log('Odometry missing or receipt/source stale')
            self.publish_stop()
            return
        if self.wpnts is None or not self.stamp_fresh(
                self.wpnts_stamp, self.wpnts_source_stamp,
                self.wpnt_timeout):
            self.safety_log('Local waypoints missing or receipt/source stale')
            self.publish_stop()
            return
        if self.mode == 'STOP':
            self.publish_stop()

    def cb_odom(self, msg: Odometry):
        receipt_time = self.now_sec()
        self.odom_stamp = receipt_time
        self.odom_source_stamp = self.source_stamp_sec(msg.header.stamp)

        cmd = AckermannDriveStamped()
        cmd.header.stamp = self.get_clock().now().to_msg()
        cmd.header.frame_id = 'base_link'

        # E-stop은 manual/recovery/path 상태보다 항상 최우선이다.
        if self.estop:
            self.publish_stop()
            self.mode = 'TRACK'
            self.stuck_since = None
            return

        recovery_directive, recovery_target = self.recovery_directive()
        if recovery_directive == 'STOP':
            self.publish_stop()
            self.mode = 'TRACK'
            self.stuck_since = None
            return

        # manual 모드: 정지 1회 발행 후 침묵 -> kb teleop(/cmd_vel)이 조종
        if self.ego_mode == 'manual':
            if not self.manual_stop_sent:
                self.publish_stop()
                self.manual_stop_sent = True
            self.stuck_since = None
            self.mode = 'TRACK'
            return

        if not self.stamp_fresh(
                self.odom_stamp, self.odom_source_stamp, self.odom_timeout):
            self.safety_log('Odometry receipt/source stale')
            self.publish_stop()
            return

        # 로컬 경로가 없거나 receipt/source 중 하나가 오래됐으면 정지.
        if self.wpnts is None or not self.stamp_fresh(
                self.wpnts_stamp, self.wpnts_source_stamp,
                self.wpnt_timeout):
            self.safety_log('Local waypoints missing or receipt/source stale')
            self.publish_stop()
            return

        cx = msg.pose.pose.position.x
        cy = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                         1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        v = msg.twist.twist.linear.x
        t = receipt_time
        odom_values = (
            cx, cy, q.x, q.y, q.z, q.w, yaw, v,
            msg.twist.twist.linear.y)
        quaternion_norm = q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w
        if (not all(math.isfinite(value) for value in odom_values) or
                not math.isfinite(quaternion_norm) or
                quaternion_norm < 1e-9):
            self.safety_log('Odometry contains non-finite/invalid values')
            self.publish_stop()
            return

        # Supervisor가 연결된 동안에는 legacy 내부 상태머신을 절대 병행하지 않는다.
        if self.recovery_contract_active():
            self.mode = 'TRACK'
            self.stuck_since = None

        if recovery_directive == 'REVERSE':
            # forward pure-pursuit 조향을 역주행에 재사용하지 않는다.
            # 명시 opt-in된 supervisor 명령도 0rad 직진 저속 후진으로 제한한다.
            cmd.drive.steering_angle = 0.0
            cmd.drive.speed = float(recovery_target)
            if not math.isfinite(cmd.drive.speed) or cmd.drive.speed >= 0.0:
                self.safety_log('Computed recovery reverse command invalid')
                self.publish_stop()
                return
            if not self.publish_immediate_command(cmd, receipt_time):
                return
            pitwall.log('cmd_speed', cmd.drive.speed)
            pitwall.log('cmd_steer', 0.0)
            return

        if self.mode == 'STOP':
            self.publish_stop()
            return

        # (tracker 자체 스턱-후진 상태머신 제거: internal_stuck_recovery_enabled 와
        #  recovery_allow_reverse 두 게이트가 모두 false 로 고정돼 mode='REVERSE'
        #  에 도달할 수 없었다. 스턱 처리는 아래의 STOP 래치와 supervisor 의
        #  RECOVER_REVERSE 경로만 남는다 — self.mode 는 TRACK|STOP 2상태다.)

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

        speed = max(0.0, float(tv) * self.speed_scale)
        if recovery_directive == 'ALIGN':
            speed = float(recovery_target)

        target_steer = float(np.clip(
            steer, -self.max_steer, self.max_steer))
        if not math.isfinite(target_steer) or not math.isfinite(speed):
            self.safety_log('Tracker produced a non-finite command')
            self.publish_stop()
            return

        limited = self.apply_motion_limits(speed, target_steer, receipt_time)
        if limited is None:
            self.publish_stop()
            return
        limited_speed, limited_steer = limited
        cmd.drive.speed = float(limited_speed)
        cmd.drive.steering_angle = float(limited_steer)
        if not self.publish_immediate_command(cmd, receipt_time):
            return
        speed = limited_speed
        pitwall.log('cmd_speed', cmd.drive.speed)
        pitwall.log('cmd_steer', cmd.drive.steering_angle)
        pitwall.log('lookahead', la)

        # --- 스턱 감지: 전진 명령 중인데 실제로 못 움직임 (gym 충돌 시 v=0 고정) ---
        if (not self.recovery_contract_active() and
                speed > 0.3 and abs(v) < 0.1):
            if self.stuck_since is None:
                self.stuck_since = t
            elif t - self.stuck_since > self.stuck_detect_sec:
                self.stuck_since = None
                self.mode = 'STOP'
                self.publish_stop()
                self.get_logger().error(
                    '스턱 감지 — blind reverse 비활성, STOP 래치')
                pitwall.event('스턱 — STOP-only 래치')
        else:
            self.stuck_since = None


def main():
    rclpy.init()
    rclpy.spin(WaypointTrackerNode())
    rclpy.shutdown()


if __name__ == '__main__':
    main()
