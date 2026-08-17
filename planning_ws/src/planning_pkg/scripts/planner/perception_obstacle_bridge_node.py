#!/usr/bin/env python3
"""perception 팀 장애물 출력 -> 플래너 frenet 장애물 변환 브릿지 (실차 전용).

perception 팀(obstacle_detector_node)은 /perception/obstacles에
f1tenth_perception_msgs/ObstacleArray를 발행한다:
  - position: msg.header.frame_id의 카르테시안 좌표 [m] (현재 detector는 map)
  - velocity: 같은 프레임 [m/s] (브릿지는 id별 위치 차분으로 재검증)
  - is_dynamic / confidence / radius

spliner·recovery는 f1tenth_planning_msgs/ObstacleArray(map 프레임 + frenet s/d)를
기대하므로 이 노드가 변환한다:
  source frame -> map timestamp TF -> /global_waypoints segment 투영으로 (s, d) 계산
  -> 트랙 밖(벽) 클러스터 제거 -> id별 (s, d) 유한차분으로 vs/vd 추정
  -> /perception/obstacles_frenet 발행 (real_car.yaml이 spliner/recovery의
     구독 토픽을 이 토픽으로 override)

is_static은 perception의 is_dynamic을 초기값으로 쓰되, 최종 판정은
spliner의 재분류(cls_* 파라미터, std 투표)가 담당한다.
velocity는 map 프레임 위치의 시간차분으로 다시 추정한다. 첫 관측이라
차분이 아직 없는 프레임에서는 detector의 KF 속도(현재 target_frame=map이라
ego 이동이 섞여 있지 않다)를 Frenet으로 투영해 시드로 쓴다.

sim에서는 virtual_obstacle_node가 f1tenth_planning_msgs 타입을 직접 발행하므로
이 노드는 실차 launch(planner_stack env:=real_car)에서만 뜬다.
"""
import math

import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from rclpy.time import Time
from rclpy.duration import Duration

import tf2_ros

from nav_msgs.msg import OccupancyGrid

from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan

from f1tenth_planning_msgs.msg import Obstacle, ObstacleArray, WpntArray
from f1tenth_perception_msgs.msg import ObstacleArray as PerceptionObstacleArray

try:
    from scipy.ndimage import distance_transform_edt
except ImportError:      # scipy 없는 환경(다른 개발 PC 등)은 맵 필터 없이 폴백
    distance_transform_edt = None


# 클러스터 반경이 이보다 크면 경고만 남긴다 [m]. 출력은 원본 반경을 그대로
# 보존한다(잘라내면 큰 장애물의 실제 크기를 감춰 회피 폭이 모자라진다).
# 예전엔 ROS 파라미터(bridge/max_extent_m)였는데 어떤 yaml 에도 없었고
# 출력에 전혀 영향이 없어서 '크기 상한'으로 오해를 샀다 — 상수로 되돌린다.
_LARGE_EXTENT_WARN_M = 1.50

# 랩 기억 장애물에 부여하는 id 대역. detector id 와 절대 겹치면 안 된다
# (겹치면 spliner 의 cls_tracks_ / committed 상태가 서로 오염된다).
# detector 의 next_id_ 는 0 부터 단조 증가하고 실측 22랩에서 240 이었다.
_MEMORY_ID_BASE = 900000


class PerceptionObstacleBridgeNode(Node):
    def __init__(self):
        super().__init__('perception_obstacle_bridge_node')

        self.declare_parameter('bridge/input_topic', '/perception/obstacles')
        self.declare_parameter('bridge/output_topic', '/perception/obstacles_frenet')
        self.declare_parameter('bridge/target_frame', 'map')
        # detector가 이미 min_hits로 트랙을 확정한다. 브릿지에서 저신뢰 트랙을
        # 삭제하면 가려지는 순간 장애물이 사라지므로 안전 기본값은 0이다.
        self.declare_parameter('bridge/min_confidence', 0.0)
        # min_confidence 를 **외삽(coasting) 트랙에만** 적용할지.
        #   True  → is_visible(이번 프레임 실측)이면 confidence 와 무관하게 통과.
        #           confidence 는 존재 확률이 아니라 정적/동적 분류 진행률이므로
        #           존재 필터로 쓰면 실측된 장애물이 지워진다 (아래 사용부 주석의
        #           run_0812_013630 3:14 사고). 분류 미확정은 삭제 대신 '정적 취급'.
        #   False → 옛 동작(실측이든 외삽이든 confidence 로 삭제). A/B 용.
        self.declare_parameter('bridge/visible_bypasses_confidence', True)
        self.declare_parameter('bridge/wall_margin_m', 0.10)
        # 큰 extent를 축소/삭제하지 않고 진단하는 임계값. 유령 제거는
        # footprint-corridor overlap과 opt-in map filter가 담당한다.
        self.declare_parameter('bridge/vel_lpf_alpha', 0.3)
        self.declare_parameter('bridge/max_abs_vs_mps', 10.0)
        self.declare_parameter('bridge/max_abs_vd_mps', 4.0)
        self.declare_parameter('bridge/position_var_floor', 0.01)
        # confidence -> pos_var 팽창의 **상한** [m].
        # 예전 식은 (half * (1 - confidence))^2 라 confidence -> 0 이면 팽창이
        # half 전체로 커졌다. 그런데 confidence 를 떨어뜨리는 주범은 coasting
        # (detector 가 misses/track_timeout 만큼 감쇠시킨다)이고, coasting 은
        # "존재가 불확실하다"는 뜻이지 "더 큰 물체"라는 뜻이 아니다.
        # 결과적으로 **유령일수록 spliner 눈에 더 큰 장애물**이 돼서 회랑을 더
        # 많이 막았다 (rosbag_0811 분석). 존재 불확실성은 min_confidence 와
        # is_visible 이 담당하고, 여기서는 위치 불확실성만 유한하게 둔다.
        self.declare_parameter('bridge/confidence_var_inflation_max_m', 0.10)
        # ── 확정 정적 트랙의 중심 누적 추정 (2026-08-16) ──────────────────
        # 위 position_var_floor/velocity_var_floor 는 **트랙 성숙도와 무관한
        # 상수**다. 정지 콘을 랩당 90~188 번 봐도 보고하는 위치 불확실성이
        # 계속 단일 프레임 지터(std 0.10 m) 수준이라, 하류 spliner 의
        #   u = min(collision_sigma 3.0 * sqrt(pos_var + t^2*vel_var),
        #           max_uncertainty_m 0.12)
        # 가 **항상 캡 0.12 에 붙는다**. (t = static_time_uncertainty_cap 0.3)
        #
        # run_0815_191056 실측: 그 0.12 가 매 랩 우측 회피를 막았다.
        #   장애물 우측 끝 -0.42 + 차반폭 0.15 + u <= 벽 클램프 0.65
        #   -> u <= 0.08 이어야 통과하는데 0.12 라 3~6 cm 부족
        #   -> lane_wall_no_room 260 회, 로그 14 건 중 13 건이 right,
        #      전부 ego_s≈24 (32.58/44.72/57.01/69.58/81.87/104.22 s)
        #
        # 고정 원통(static_cylinder_radius_m)이 폭을 이미 잡아 두었으므로 남은
        # 불확실성은 **중심 위치**뿐이다. 실측으로도 그렇다 — d_left/d_right 의
        # 프레임 간 변화가 좌우 동일(p90 8.3 cm)하고 폭 변화는 med 0.0 cm 다.
        #
        # 그래서 확정 정적 + 충분히 본 트랙은 중심을 **누적 평균**하고, 그
        # 평균의 분산(sigma^2/n)을 보고한다. 문턱을 무르게 하는 게 아니라
        # 추정을 정확하게 하는 것이라 안전여유를 깎지 않는다 — 오히려 매
        # 프레임 흔들리던 중심이 고정되어 커밋 파기(path_collision 43 회)도
        # 같이 줄어든다.
        #
        # ⚠️ 조건을 느슨하게 하지 말 것. is_static 확정 + 관측수 임계를 둘 다
        #    요구하는 이유는, 실제로는 천천히 움직이는 물체를 정적으로 오분류한
        #    경우 누적 평균이 뒤처져 **과거 위치**를 발행하게 되기 때문이다.
        self.declare_parameter('bridge/static_mean_min_obs', 30)
        # 하한은 "평균의 분산"이 물리적으로 가질 수 있는 값보다 낮게 가지
        # 않도록 잡는다. 목표는 u ~= 0.06 (<= 0.08 요구치, 여유 포함):
        #   3.0 * sqrt(3e-4 + 0.09*1e-3) = 3.0 * 0.0197 = 0.059
        self.declare_parameter('bridge/static_mean_position_var_floor', 3.0e-4)
        self.declare_parameter('bridge/static_mean_velocity_var_floor', 1.0e-3)
        # ── 같은 물체의 재등록/중복 흡수 ──────────────────────────────────
        # detector 가 같은 콘에 **새 id** 를 붙이는 일이 잦다. run_0814_191443
        # 실측: 새 id 561개 중 164개(29%)가 차 앞 2 m 안에서 태어나고, 그중
        #   중복(duplicate) 96건 — 기존 트랙이 살아 있는데 0.24 m(p50) 옆에
        #     또 하나가 생긴다. 새 쪽 수명 p50 3프레임 / 기존 p50 67프레임,
        #     공존 p50 0.1 초. 0.30 m 콘 두 개가 0.24 m 간격일 수는 없으므로
        #     이건 한 콘이 잠깐 둘로 쪼개진 것이다.
        #   재등록(hand-off) 68건 — 기존 트랙이 사라지면서 같은 자리에 새 id.
        # 둘 다 하류에서 **새 장애물**로 보인다. 그 대가가 크다:
        #   - Welford 누적(static_mean_min_obs)이 0 부터 다시 쌓여, 바로 위
        #     주석의 "매 랩 4cm 부족" 수정이 정작 코앞에서 안 듣는다.
        #   - spliner 의 commit_break 가 관측수 게이트를 채운 새 id 를 보고
        #     회피 커밋을 파기한다 (run_0814 43회).
        # 그래서 정적 트랙에 한해 같은 자리의 새 id 를 기존 트랙으로 흡수한다.
        # ⚠️ 동적 트랙에는 적용하지 않는다 — 두 대가 붙어 달리는 경우를 하나로
        #    합치면 사이 공간이 사라진다. 게이트도 콘 지름(0.30)보다 크게 잡지
        #    말 것: 그보다 넓히면 실제로 나란히 선 콘 두 개를 합치게 된다.
        self.declare_parameter('bridge/reassoc_enabled', True)
        # 실측 중복 분리거리 p50 0.24 m. 게이트는 콘 지름 0.30 m 까지만 —
        # 그 이상 떨어진 둘은 물리적으로 겹치지 않으므로 별개로 둔다
        # (p90 0.45 m 짜리는 일부러 안 잡는다).
        self.declare_parameter('bridge/reassoc_match_s', 0.30)
        self.declare_parameter('bridge/reassoc_match_d', 0.25)
        # 기존 트랙이 이만큼은 관측돼 있어야 새 id 를 흡수할 자격이 있다.
        # (갓 태어난 트랙 둘이 서로를 흡수하는 왕복을 막는다.)
        self.declare_parameter('bridge/reassoc_min_obs', 5)
        # 같은 트랙에 겹쳐 들어오는 검출을 몇 프레임까지 흡수할 것인가.
        # 실측: 쪼개진 조각 수명 p50 3프레임 / 진짜 이웃 콘은 30~210프레임.
        # 이 한도를 넘으면 별개 장애물로 되돌린다 (hold_duplicate).
        self.declare_parameter('bridge/reassoc_max_dup_frames', 8)
        # ── 정적 장애물 고정 원통 모델 ────────────────────────────────────
        # detector 의 radius 는 "중심에서 가장 먼 점까지"라 **매 프레임 다시**
        # 측정된다. LiDAR 는 앞쪽 호만 보므로 거리·각도·빔 개수에 따라 값이
        # 흔들린다 — 실차 run_0812_013630 실측(정적·is_visible 트랙 36개 /
        # 894프레임)에서 **같은 콘의 트랙 내 변동폭이 p50 0.14 m** (p90 0.19,
        # max 0.21) 였다. 그런데 아래에서 obs.d_left/d_right = d ± half 이므로
        # **장애물 가장자리가 프레임마다 ±0.07 m 움직인다.**
        # 회피 레인 개폐를 결정한 마진이 6 cm 였다(우측 d_hold -0.40 vs 필요
        # -0.46) — 판정 노이즈가 판정 마진보다 컸다.
        #
        # 크기를 매번 재지 않고 알려진 원통으로 고정한다. 세 조건을 모두
        # 만족할 때만 적용하고, 하나라도 어긋나면 측정값을 그대로 둔다.
        #   ① is_static      — 동적 상대는 크기가 실제로 다르다
        #   ② band 안        — 밖은 콘이라고 믿을 근거가 없다
        #   ③ 벽에서 떨어짐  — 아래 open_space 주석
        #
        # ⚠️ radius 는 **실제로 트랙에 놓는 물체**에 맞춰야 한다. 0.15 는
        #    대회 기준(최대 지름 30 cm)이다. 그보다 큰 콘으로 테스트하는
        #    동안에는 이 값을 그 콘에 맞게 올릴 것 — 큰 물체를 0.15 로
        #    퉁치면 매 프레임 그 차이만큼 **작게** 본다.
        self.declare_parameter('bridge/static_cylinder_radius_m', 0.15)
        # 적용 대역 [m]. 상한이 곧 **최대 피해량의 상한**이다:
        #   실제로 잃는 extent <= (상한 - radius) = 0.25 - 0.15 = 0.10 m
        # 상한 근거(같은 백): 콘 프레임의 97% 가 0.25 이하. 0.22 이하로
        # 낮추면 크게 측정된 소수 프레임이 대역 밖으로 빠지는데, 가장자리
        # 흔들림을 만드는 게 바로 그 프레임들이라 **이득이 0 이 된다**
        # (트랙 내 변동폭 중앙값: 상한 0.18/0.20/0.22 모두 0.084 m 그대로,
        #  0.25 에서 0.000 m). 0.25~0.35 구간(전체의 3%)은 애매하므로 원시값
        # 유지. 0.35 초과는 detector 가 이미 벽으로 거른다
        # (obstacle_detector_node max_cluster_radius_m).
        self.declare_parameter('bridge/static_cylinder_band_m', [0.05, 0.25])
        # 장애물 **중심**이 벽에서 이만큼 떨어져 있을 때만 고정한다.
        # 벽에 붙으면 클러스터가 벽 점을 같이 물어 extent 가 실제로 커진다 —
        # 같은 백에서 벽거리 0~0.30 m 구간의 radius p50 이 0.20 으로, 그 바깥
        # (0.30~0.50 m)의 0.13 보다 크다. 거기서 0.15 로 줄이면 **회랑이 가장
        # 좁은 곳에서 판정을 여는 방향으로** 최대 0.10 m 밀린다. 차폭짜리
        # 가짜 통로가 생기진 않았지만(874프레임 중 0건) 레인 판정을 뒤집기엔
        # 충분하다(실측 레인 개폐 마진 6 cm).
        # 0.30 근거: 오염이 관측되는 경계가 정확히 여기다. 그리고 중심 0.30 은
        # 표면이 벽에서 0.15 m 떨어졌다는 뜻이라 클러스터가 안 붙는다.
        # 적용률 69%(733/1056) — 이 조건을 아예 빼도 80% 라 손실이 작다.
        self.declare_parameter('bridge/static_cylinder_open_space_m', 0.30)
        self.declare_parameter('bridge/velocity_var_floor', 0.25)
        # 속도를 아직 유한차분으로 못 잰 첫 프레임의 속도 표준편차 prior [m/s].
        # max_abs_v(클립 한계)를 분산으로 쓰던 것을 대체한다 — 근거는 아래
        # obs.vs_var 계산부 주석 참조.
        self.declare_parameter('bridge/init_vel_std_mps', 0.5)
        self.declare_parameter('bridge/track_forget_sec', 2.0)
        self.declare_parameter('bridge/tf_timeout_sec', 0.1)
        self.declare_parameter('bridge/tf_allow_latest_fallback', False)
        self.declare_parameter('bridge/tf_max_age_sec', 0.05)
        self.declare_parameter('bridge/max_source_age_sec', 0.20)
        self.declare_parameter('bridge/source_future_tolerance_sec', 0.05)
        self.declare_parameter('bridge/require_source_stamp', True)
        self.declare_parameter('bridge/projection_window_wpnts', 80)
        self.declare_parameter('bridge/projection_reacquire_distance_m', 1.5)
        # 맵 벽거리 필터: frenet d는 최근접 raceline 점의 법선 거리라 코너에서
        # 실제 벽까지 거리와 어긋난다 — 벽 위 유령(실측 d=-0.85, 경계 1.32)이
        # d-경계 필터를 통과했다. /map(OccupancyGrid) EDT로 "벽까지 실거리"를
        # 직접 판정한다. 맵 미수신/scipy 부재 시 기존 frenet 필터만으로 폴백.
        # ⚠️ 실차 적용 전 PF 수렴 상태에서 정지 장애물 좌표 산포를 먼저 잴 것 —
        #    wall_dist_min이 PF 오차보다 작으면 유령이 새고, 크면 진짜를 버린다.
        # ── 랩 기억 (정적 장애물 Frenet 앵커 레지스트리) ────────────────────
        # 인지 트랙 id 는 랩을 못 넘는다 — run_0813_165312 실측으로 22랩 동안
        # 발급된 id 240개 중 랩 경계를 넘은 것은 **0개**이고, 수명 p50 0.43s /
        # max 4.72s 다 (상한은 track_timeout_sec 0.80 + reid_timeout_sec 1.0).
        # 그래서 같은 콘을 매 랩 처음부터 다시 알아보고, 판단은 늘 5~6 m 에서
        # 내려진다(반응 예산 p50 1.20s / p10 0.09s).
        #
        # 기억이 주는 것은 두 가지이고 **정확도 쪽이 더 크다**:
        #  ① 시간   : 첫 인지 5.77 m -> lookahead 전체. S/F 기준 예산 0.94 -> 2.5s
        #  ② 정확도 : 판단이 내려지는 5~8 m 구간의 d 추정 표준편차 0.091 m 인데
        #             근접(0~0.5 m) 관측은 0.029 m 다. 랩 간 재현성은 s std
        #             0.022~0.131 / d std 0.007~0.062 m 로 그보다도 좁다.
        #             실제로 #178 은 4 m 시점에 왼쪽 끝을 +0.09 로 봤는데
        #             수렴값은 -0.07 이었다 (좌측 회랑 0.16 m 과소평가).
        #  -> 기억은 "지난 랩의 근접 관측"을 이번 랩의 판단 시점에 갖다 준다.
        #
        # 기본값은 **비활성**이다. 켜는 것은 실차 프로파일의 명시적 선택이어야
        # 한다 — 아래 4개 대책이 전부 이 트랙 실측에 맞춰진 값이기 때문이다.
        self.declare_parameter('bridge/memory_enabled', False)
        # 확정 조건 ①: 서로 다른 랩에서 이만큼 관측돼야 주입을 시작한다.
        # 유령은 랩마다 위치가 흔들려 같은 앵커에 두 번 모이지 못한다
        # (실측: 회랑 안 |d|<0.4 에서 3 m 이상 보였다가 사라진 트랙 0건,
        #  사라진 7건은 전부 d -0.53~-0.82 = 우벽 밖 파편이었다).
        self.declare_parameter('bridge/memory_confirm_laps', 2)
        # 같은 물리 장애물로 볼 (s, d) 반경. 랩 간 재현성 실측(s std <=0.131,
        # d std <=0.062)의 3배 이상이되 두 콘 간격(실측 ~1.7 m)보다는 작아야
        # 한다 — 크면 서로 다른 콘이 한 앵커로 합쳐진다.
        self.declare_parameter('bridge/memory_match_s_m', 0.60)
        self.declare_parameter('bridge/memory_match_d_m', 0.35)
        # 주입 거리. spliner/lookahead_m(실차 12.0) 안쪽만 의미가 있다.
        self.declare_parameter('bridge/memory_inject_ahead_m', 12.0)
        # 대책 ③: 살아 있는 **동적** 트랙이 기억 위치에서 이 거리 안에 있으면
        # 주입하지 않는다. 상대차가 기억 옆을 지날 때 기억이 사이드 선택을
        # 왜곡해 상대와 벽 사이에 갇히는 것을 막는다. 동적이 항상 우선.
        self.declare_parameter('bridge/memory_dynamic_suppress_m', 1.5)
        # 대책 ① 의 단서: "동적으로 보였으면 기억하지 않는다"를 거리 무관하게
        # 걸면 **살려야 할 기억이 첫 접촉에 파괴된다.** run_0813_165312 의
        # 정적->동적 전환 7건은 전부 최근접 통과 0.05~0.47 m 에서 났고,
        # 통과 직후 is_visible=0 외삽 프레임에서 넘어갔다 — 콘이 스치며
        # 밀렸거나 근거리에서 중심점이 걸어간 것이지 상대차가 아니다.
        # 그래서 **이 거리보다 먼 곳에서** 동적으로 보인 경우만 독으로 친다.
        # 진짜 상대차는 멀리서부터 움직이므로 이 게이트를 반드시 통과한다.
        self.declare_parameter('bridge/memory_dynamic_poison_min_dist_m', 1.0)
        # 관련성 밴드: raceline 주행 시 요구 여유를 침범하는 것만 기억한다.
        # |d| - half - 차반폭 > 이 값이면 앵커를 만들지 않는다.
        self.declare_parameter('bridge/memory_relevant_margin_m', 0.25)
        # 근거리 주입 게이트. 이 거리보다 가까워지면, **이번 통과에서 실측이
        # 한 번이라도 붙은** 기억만 계속 주입한다.
        #
        # 기억의 존재 이유는 검출이 불안정한 원거리(5~12 m)를 덮는 것이고,
        # 근거리는 라이다가 가장 잘 보는 구간이다. 재생 검증 실측:
        # confirm=2/forget=2 에서 5 m 안쪽 주입 프레임의 56% 가 그 랩에 실물이
        # 없던 것이었다(= 유령을 코앞까지 끌고 감). 반대로 이 게이트를 그냥
        # 거리로만 자르면 run_0813 1:44 처럼 실물이 1.84 m 에서 인지에서
        # 사라지는 경우(드롭아웃 57건)를 못 덮는다.
        # 그래서 자르는 기준을 거리가 아니라 **이번 랩의 실측 유무**로 둔다:
        #   · 실물  -> 5.7 m 에서 이미 matched -> 근거리까지 계속 주입 (드롭아웃 커버)
        #   · 유령  -> 한 번도 matched 안 됨   -> 이 거리에서 주입 중단
        self.declare_parameter('bridge/memory_close_requires_match_m', 3.0)
        # 대책 ④: free-space carving. 기억 위치를 지나갔는데 그 통과 동안
        # 대응하는 실측이 한 번도 없으면 miss 를 올리고, 이만큼 쌓이면 지운다.
        # "N초 뒤 잊는다"가 아니라 **부정 증거**로 지우는 것이 핵심이다 —
        # 시간 만료는 있는 것도 없는 것도 못 맞춘다.
        self.declare_parameter('bridge/memory_forget_misses', 2)
        # 기억이 보고하는 위치 불확실성 하한 [m]. 랩 간 d std 실측 최대
        # 0.062 의 두 배 — 기억을 실측보다 더 확신하지 않게 한다.
        self.declare_parameter('bridge/memory_pos_std_m', 0.12)
        # ── 스캔 빈자리 카빙 (2026-08-17, run_0817_150059 / _145548) ─────
        # **정지 탈출 장치다.** 랩 경계 carving(위 ④)은 차가 서면 영원히 안
        # 온다 — 150059 에서 유령 앵커 2개가 트랙을 닫아 정지했는데, 정지라서
        # 랩 경계가 안 오고, 랩 경계가 안 와서 miss 를 못 쌓는 자기잠금이
        # 14초 이어졌다(수동 개입). 그래서 프레임 단위 부정 증거를 둔다:
        # 앵커 자리를 /scan 빔이 관통(측정거리 > 앵커거리 + slack)하는
        # 프레임이 연속 clear_frames 개면 그 앵커의 주입을 즉시 멈추고,
        # 랩 경계에서 miss 로 정산한다. 검출기 매칭(중심 매칭)만이 되살린다.
        #   · 벽에 붙은 콘: 빔이 콘/벽에 맞아 되돌아옴 -> hit -> 카빙 안 됨
        #     (slack 안의 벽 반사는 hit 로 세므로 fail-safe 방향은 "유지")
        #   · 가림(다른 콘 뒤): 빔이 앞에서 끊김 -> 판정 보류, 스트릭 유지
        # ⚠️ 판정은 **전방(ahead 0.5 ~ L/2) 앵커에만** 돈다. 옆을 지나치는
        #    순간에는 앵커 위치오차(0.1~0.2 m)가 빔 창을 실물 밖으로 밀어내
        #    실물을 "관통"으로 오판한다 — 150059 재생에서 벽 콘(s=16.64)을
        #    4 m/s 통과(r 0.58, 방위 47→137도 스윕) 중에 지웠고, 그러면
        #    다음 랩의 6.9 m 조기 커밋(이 기능 최대 수혜)이 사라진다.
        #    전방을 막 벗어난 유령(예: 145548 충돌 유령 900016, 관통 증거가
        #    전부 ahead<0.5)은 카빙이 아니라 중심 매칭 게이트가 끊는다 —
        #    중심 매칭 0건 -> close_requires_match(3 m)가 주입 차단
        #    (tools/bag_analysis_0817/carve_replay.py 재생 검증).
        self.declare_parameter('bridge/memory_scan_carve_enabled', True)
        self.declare_parameter('bridge/memory_scan_topic', '/scan')
        # 40 Hz 스캔 기준 15프레임 = 0.375 s. 관통 증거가 이만큼 연속되어야
        # 끈다 — 단일 프레임 노이즈/부분 가림으로 실물 기억을 지우지 않는다.
        self.declare_parameter('bridge/memory_scan_clear_frames', 15)
        # 판정 거리 창 [m]. 상한: 원거리는 빔 간격이 콘보다 넓어지고 반사율도
        # 떨어져 "관통"의 증거 가치가 없다. 하한: 150059 정지에서 유령이
        # 레이저에서 0.72 m 에 있었다 — 여기까지는 내려가야 정지 탈출이 된다.
        # 하한을 1.0 으로 올리는 실험은 유령 카빙(관통이 근거리에서만 열림)을
        # 전멸시켰다 (carve_replay.py).
        self.declare_parameter('bridge/memory_scan_range_min_m', 0.3)
        self.declare_parameter('bridge/memory_scan_range_max_m', 7.0)
        # 앵커 거리 ± slack 안의 반사는 hit(실물)로 센다 [m]. 콘 반경 0.15 +
        # PF 위치 오차를 덮는 값. 이보다 멀리 뚫려야 관통이다.
        self.declare_parameter('bridge/memory_scan_slack_m', 0.25)
        # 한 프레임을 "관통"으로 인정하는 최소 빔 수. 콘 폭은 5 m 에서
        # 각도 ±1.7도 ≈ 빔 13개라 3개는 보수적 하한이다.
        self.declare_parameter('bridge/memory_scan_min_pass_beams', 3)
        # ── 중심 매칭 게이트 (2026-08-17, run_0817_150059) ────────────────
        # matched_this_pass 라치가 **밴드 가장자리** 한 프레임으로 서는 것을
        # 막는다. 150059: 실물 검출이 이웃 앵커 밴드 끝(cost 1.03)에 한 프레임
        # 걸리며 유령의 3 m 근거리 게이트를 껐다 — 그 유령이 정지의 원인.
        # 실물 매칭은 중심에 붙는다(벽 콘 1프레임 실측 cost 0.18). 타원
        # cost = (ds/match_s)² + (dd/match_d)² 가 이 값 이하일 때만 라치.
        self.declare_parameter('bridge/memory_match_latch_cost', 0.5)
        # ego Frenet 위치 토픽 (랩 카운트 + 주입 거리 판정).
        self.declare_parameter('bridge/ego_frenet_topic',
                               '/car_state/frenet/odom')
        self.declare_parameter('bridge/ego_frenet_timeout_sec', 0.5)

        self.declare_parameter('bridge/use_map_filter', False)
        self.declare_parameter('bridge/map_topic', '/map')
        self.declare_parameter('bridge/wall_dist_min_m', 0.20)
        self.declare_parameter('bridge/map_timeout_sec', 5.0)
        self.declare_parameter('bridge/map_filter_required', True)

        p = self.get_parameter
        self.target_frame = str(p('bridge/target_frame').value).strip()
        self.min_confidence = float(p('bridge/min_confidence').value)
        self.visible_bypasses_confidence = bool(
            p('bridge/visible_bypasses_confidence').value)
        self.wall_margin = float(p('bridge/wall_margin_m').value)
        self.lpf_alpha = float(p('bridge/vel_lpf_alpha').value)
        self.max_abs_vs = float(p('bridge/max_abs_vs_mps').value)
        self.max_abs_vd = float(p('bridge/max_abs_vd_mps').value)
        self.position_var_floor = float(
            p('bridge/position_var_floor').value)
        self.conf_inflation_max = float(
            p('bridge/confidence_var_inflation_max_m').value)
        self.static_mean_min_obs = int(p('bridge/static_mean_min_obs').value)
        self.static_mean_pos_var_floor = float(
            p('bridge/static_mean_position_var_floor').value)
        self.static_mean_vel_var_floor = float(
            p('bridge/static_mean_velocity_var_floor').value)
        self.reassoc_enabled = bool(p('bridge/reassoc_enabled').value)
        self.reassoc_match_s = float(p('bridge/reassoc_match_s').value)
        self.reassoc_match_d = float(p('bridge/reassoc_match_d').value)
        self.reassoc_min_obs = int(p('bridge/reassoc_min_obs').value)
        self.reassoc_max_dup_frames = int(
            p('bridge/reassoc_max_dup_frames').value)
        self.cyl_radius = float(p('bridge/static_cylinder_radius_m').value)
        _band = [float(v) for v in p('bridge/static_cylinder_band_m').value]
        self.cyl_lo, self.cyl_hi = (_band + [0.0, 0.0])[:2]
        self.cyl_open_space = float(
            p('bridge/static_cylinder_open_space_m').value)
        self.velocity_var_floor = float(
            p('bridge/velocity_var_floor').value)
        self.init_vel_std = float(p('bridge/init_vel_std_mps').value)
        self.forget_sec = float(p('bridge/track_forget_sec').value)
        self.tf_timeout = float(p('bridge/tf_timeout_sec').value)
        self.tf_allow_latest = bool(
            p('bridge/tf_allow_latest_fallback').value)
        self.tf_max_age = float(p('bridge/tf_max_age_sec').value)
        self.max_source_age = float(p('bridge/max_source_age_sec').value)
        self.source_future_tolerance = float(
            p('bridge/source_future_tolerance_sec').value)
        self.require_source_stamp = bool(
            p('bridge/require_source_stamp').value)
        self.projection_window = int(p('bridge/projection_window_wpnts').value)
        self.projection_reacquire_distance = float(
            p('bridge/projection_reacquire_distance_m').value)
        self.use_map_filter = bool(p('bridge/use_map_filter').value)
        self.wall_dist_min = float(p('bridge/wall_dist_min_m').value)
        self.map_timeout = float(p('bridge/map_timeout_sec').value)
        self.map_filter_required = bool(
            p('bridge/map_filter_required').value)
        self.memory_enabled = bool(p('bridge/memory_enabled').value)
        self.memory_confirm_laps = int(p('bridge/memory_confirm_laps').value)
        self.memory_match_s = float(p('bridge/memory_match_s_m').value)
        self.memory_match_d = float(p('bridge/memory_match_d_m').value)
        self.memory_inject_ahead = float(
            p('bridge/memory_inject_ahead_m').value)
        self.memory_dynamic_suppress = float(
            p('bridge/memory_dynamic_suppress_m').value)
        self.memory_poison_min_dist = float(
            p('bridge/memory_dynamic_poison_min_dist_m').value)
        self.memory_relevant_margin = float(
            p('bridge/memory_relevant_margin_m').value)
        self.memory_close_requires_match = float(
            p('bridge/memory_close_requires_match_m').value)
        self.memory_forget_misses = int(p('bridge/memory_forget_misses').value)
        self.memory_pos_std = float(p('bridge/memory_pos_std_m').value)
        self.memory_scan_carve = bool(
            p('bridge/memory_scan_carve_enabled').value)
        self.memory_scan_clear_frames = int(
            p('bridge/memory_scan_clear_frames').value)
        self.memory_scan_range_min = float(
            p('bridge/memory_scan_range_min_m').value)
        self.memory_scan_range_max = float(
            p('bridge/memory_scan_range_max_m').value)
        self.memory_scan_slack = float(p('bridge/memory_scan_slack_m').value)
        self.memory_scan_min_pass = int(
            p('bridge/memory_scan_min_pass_beams').value)
        self.memory_match_latch_cost = float(
            p('bridge/memory_match_latch_cost').value)
        self.ego_frenet_timeout = float(
            p('bridge/ego_frenet_timeout_sec').value)

        finite_values = (
            self.min_confidence, self.wall_margin,
            self.lpf_alpha, self.max_abs_vs, self.max_abs_vd,
            self.position_var_floor, self.velocity_var_floor, self.init_vel_std,
            self.forget_sec, self.tf_timeout, self.tf_max_age,
            self.max_source_age, self.source_future_tolerance,
            self.projection_reacquire_distance, self.wall_dist_min,
            self.map_timeout,
        )
        if (not all(math.isfinite(value) for value in finite_values)
                or not self.target_frame
                or not 0.0 <= self.min_confidence <= 1.0
                or not 0.0 <= self.wall_margin <= 2.0
                or not 0.0 <= self.lpf_alpha <= 1.0
                or not 0.1 <= self.max_abs_vs <= 50.0
                or not 0.1 <= self.max_abs_vd <= 20.0
                or not 1e-6 <= self.position_var_floor <= 100.0
                # 대역이 뒤집히거나 radius 가 대역 밖이면 "고정"이 오히려
                # extent 를 키우거나 줄이는 방향을 예측할 수 없게 된다.
                or not 0.0 < self.cyl_lo < self.cyl_hi <= 1.0
                or not self.cyl_lo <= self.cyl_radius <= self.cyl_hi
                or not 0.0 <= self.cyl_open_space <= 2.0
                or not 1e-6 <= self.velocity_var_floor <= 2500.0
                or not 0.0 <= self.init_vel_std <= 50.0
                or not 0.1 <= self.forget_sec <= 10.0
                or not 0.001 <= self.tf_timeout <= 1.0
                or not 0.0 <= self.tf_max_age <= self.max_source_age
                or not 0.01 <= self.max_source_age <= 2.0
                or not 0.0 <= self.source_future_tolerance <= 0.5
                or not 4 <= self.projection_window <= 10000
                or not 0.1 <= self.projection_reacquire_distance <= 20.0
                or not 0.0 <= self.wall_dist_min <= 2.0
                or not 0.1 <= self.map_timeout <= 60.0
                # 기억 파라미터. 비활성이어도 검증한다 — 켜는 시점에 값이
                # 이상하면 그때는 이미 주행 중이다.
                or not 1 <= self.memory_confirm_laps <= 20
                or not 0.05 <= self.memory_match_s <= 2.0
                or not 0.05 <= self.memory_match_d <= 1.0
                or not 0.0 <= self.memory_inject_ahead <= 30.0
                or not 0.0 <= self.memory_dynamic_suppress <= 10.0
                or not 0.0 <= self.memory_poison_min_dist <= 5.0
                or not 0.0 <= self.memory_relevant_margin <= 2.0
                or not 0.0 <= self.memory_close_requires_match
                <= self.memory_inject_ahead
                or not 1 <= self.memory_forget_misses <= 20
                or not 0.01 <= self.memory_pos_std <= 1.0
                # 스캔 카빙: 프레임 수/거리/slack 이 뒤집히면 관통 판정이
                # 즉발(=단일 노이즈로 기억 삭제) 또는 불능이 된다.
                or not 3 <= self.memory_scan_clear_frames <= 200
                or not 0.2 <= self.memory_scan_range_min <= 2.0
                or not self.memory_scan_range_min < self.memory_scan_range_max
                or not 1.0 <= self.memory_scan_range_max <= 30.0
                or not 0.05 <= self.memory_scan_slack <= 1.0
                or not 1 <= self.memory_scan_min_pass <= 50
                # 라치 게이트가 1.0 을 넘으면 밴드 가장자리(cost 1.0)까지
                # 라치를 허용해 게이트가 없는 것과 같다.
                or not 0.05 <= self.memory_match_latch_cost <= 1.0
                # 재연결 게이트: 콘 지름(2 * static_cylinder_radius_m)을 넘기면
                # 나란히 선 콘 두 개를 하나로 합치게 된다.
                or not 0.0 <= self.reassoc_match_s <= 2.0 * self.cyl_radius
                or not 0.0 <= self.reassoc_match_d <= 2.0 * self.cyl_radius
                or not 1 <= self.reassoc_min_obs <= 100
                or not 1 <= self.reassoc_max_dup_frames <= 100
                or not 0.05 <= self.ego_frenet_timeout <= 5.0):
            raise RuntimeError('invalid bridge safety parameters')

        self.wpnts = None        # [N, 6]: s, x, y, psi, d_right, d_left
        self.track_length = 0.0
        self.tracks = {}         # id -> {'t', 'arrival_t', 's', 'd', 'vs', 'vd', 'wi'}
        # detector id -> {'to': 기존 트랙 id, 'dup': 연속 중복 프레임 수}.
        # 매 프레임 게이트를 다시 확인해서, 그 물체가 실제로 갈라져 나가면
        # (게이트 밖) 매핑을 끊는다.
        self.reassoc = {}
        self.reassoc_absorbed = 0    # 진단용 누적 카운터
        self.wall_dist = None    # EDT 거리 테이블 [m] (맵 수신 시 1회 계산)
        self.map_origin = (0.0, 0.0)
        self.map_origin_yaw = 0.0
        self.map_res = 0.0
        self.start_time = self.get_clock().now()
        self.last_source_ns = None

        # ── 랩 기억 상태 ────────────────────────────────────────────────
        # anchor: dict(s, d, half, laps:set, confirmed, misses, poisoned,
        #              best_dist, best_s, best_d, best_half, mem_id,
        #              matched_this_pass, pass_open)
        self.memory = []
        self.memory_next_id = _MEMORY_ID_BASE
        self.ego_s = None
        self.ego_d = None
        self.ego_stamp_sec = None
        self.lap_index = 0
        self._last_ego_s = None

        self.tf_buffer = tf2_ros.Buffer()
        # cb_obs 안의 exact-time lookup이 executor를 기다리는 동안 /tf callback이
        # 굶지 않아야 한다. spin_thread=True는 이걸 못 한다 — 오히려 무력화된다:
        # TransformListener가 전용 executor에 node를 add_node한 뒤 main()의
        # rclpy.spin(node)이 글로벌 executor에 같은 node를 넣으면 rclpy의
        # Node.executor setter가 기존 executor에서 노드를 빼버린다
        # (rclpy/node.py: `current_executor.remove_node(self)`).
        # 그러면 /tf 콜백이 결국 메인 executor에 직렬화되고, depth 100인 /tf
        # 큐가 포화되며 버퍼가 ~1.6초 늙는다 (obstacle_detector에서 실측:
        # run_0803_204743, |tf-scan| 1.3~2.1s로 스캔 전멸).
        # -> spin_thread=False + main()의 MultiThreadedExecutor 조합을 쓴다.
        self.tf_listener = tf2_ros.TransformListener(
            self.tf_buffer, self, spin_thread=False)

        waypoint_qos = QoSProfile(
            depth=1, reliability=ReliabilityPolicy.RELIABLE)
        sensor_qos = QoSProfile(
            depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        output_qos = QoSProfile(
            depth=1, reliability=ReliabilityPolicy.RELIABLE)
        self.create_subscription(WpntArray, '/global_waypoints', self.cb_gb, waypoint_qos)
        if self.memory_enabled:
            ego_topic = str(p('bridge/ego_frenet_topic').value).strip()
            if not ego_topic:
                raise RuntimeError('memory_enabled requires ego_frenet_topic')
            self.create_subscription(
                Odometry, ego_topic, self.cb_ego_frenet, sensor_qos)
            if self.memory_scan_carve:
                scan_topic = str(p('bridge/memory_scan_topic').value).strip()
                if not scan_topic:
                    raise RuntimeError(
                        'memory_scan_carve_enabled requires scan topic')
                self.create_subscription(
                    LaserScan, scan_topic, self.cb_scan, sensor_qos)
        input_topic = str(p('bridge/input_topic').value).strip()
        output_topic = str(p('bridge/output_topic').value).strip()
        map_topic = str(p('bridge/map_topic').value).strip()
        if not input_topic or not output_topic or \
                (self.use_map_filter and not map_topic):
            raise RuntimeError('bridge input/output/map topics must not be empty')
        self.create_subscription(
            PerceptionObstacleArray, input_topic, self.cb_obs, sensor_qos)
        if self.use_map_filter:
            if distance_transform_edt is None:
                self.get_logger().error(
                    'bridge/use_map_filter=true지만 scipy가 없음 — '
                    'map_filter_required이면 obstacle lease를 발행하지 않음')
            else:
                # map_server는 latched(TRANSIENT_LOCAL) 발행 — 기본 QoS로는 못 받는다
                map_qos = QoSProfile(
                    depth=1,
                    reliability=ReliabilityPolicy.RELIABLE,
                    durability=DurabilityPolicy.TRANSIENT_LOCAL)
                self.create_subscription(
                    OccupancyGrid, map_topic, self.cb_map, map_qos)
        self.pub = self.create_publisher(
            ObstacleArray, output_topic, output_qos)

        self.get_logger().info(
            f"perception 브릿지 시작: {input_topic} "
            f"(f1tenth_perception_msgs) -> {output_topic} (f1tenth_planning_msgs)")

    # ── 랩 기억 ────────────────────────────────────────────────────────
    def cb_ego_frenet(self, msg: Odometry):
        """ego Frenet 위치 + 랩 카운트. s 가 크게 뒤로 점프하면 랩 경계."""
        s = float(msg.pose.pose.position.x)
        d = float(msg.pose.pose.position.y)
        if not (math.isfinite(s) and math.isfinite(d)):
            return
        stamp = Time.from_msg(msg.header.stamp)
        self.ego_stamp_sec = (stamp.nanoseconds * 1e-9 if stamp.nanoseconds
                              else self.get_clock().now().nanoseconds * 1e-9)
        self.ego_s, self.ego_d = s, d
        if self.track_length > 0.0 and self._last_ego_s is not None:
            # 전진 중 s 가 반 바퀴 이상 줄면 결승선 통과. 후진/PF 점프는
            # 이 조건에 안 걸린다 (점프는 보통 반 바퀴보다 작다).
            if self._last_ego_s - s > self.track_length * 0.5:
                self.lap_index += 1
                self._on_lap_boundary()
        self._last_ego_s = s

    def _on_lap_boundary(self):
        """랩 경계에서 이번 랩 통과를 정산한다.

        ① 실측이 있었으면 그 랩의 **최근접** 관측으로 앵커를 갱신하고
           miss 를 0 으로 되돌린다 (긍정 증거).
        ② 기억 위치를 시야에 넣고 지나갔는데 실측이 한 번도 없었으면
           miss 를 올린다 (부정 증거 = free-space carving). 시간 만료가
           아니라 이쪽이 삭제 기준이다.
        """
        for a in self.memory:
            if a.get('absent'):
                # 스캔이 빈자리를 증명한 앵커는 실측 에지 프레임(best_dist)
                # 보다 관통 증거를 우선한다 — run_0817_150059 의 유령은 매 랩
                # 밴드 가장자리 1프레임으로 misses 를 리셋하며 살아남았다.
                # (중심 매칭이 있었다면 memory_observe 가 absent 를 이미
                #  풀었으므로 여기 안 들어온다.)
                a['misses'] += 1
                a['pass_open'] = False
                a['matched_this_pass'] = False
                a['best_dist'] = float('inf')
                continue
            if math.isfinite(a['best_dist']):
                # 확정 전에는 최근접 관측을 그대로 받고, 확정 뒤에는 절반씩
                # 섞는다 (랩 간 재현성 실측 d std 0.007~0.062 m 수준이라
                # 급격히 따라갈 이유가 없다).
                w = 0.5 if a['confirmed'] else 1.0
                a['s'] = (a['s'] + w * self._ds_signed(a['best_s'], a['s'])) \
                    % self.track_length
                a['d'] += w * (a['best_d'] - a['d'])
                a['half'] += w * (a['best_half'] - a['half'])
                a['misses'] = 0
            elif a['pass_open']:
                a['misses'] += 1
            a['pass_open'] = False
            a['matched_this_pass'] = False
            a['best_dist'] = float('inf')
        self._merge_memory()
        self._prune_memory()

    def _merge_memory(self):
        """수렴한 뒤 서로 밴드 안에 들어온 앵커를 합친다.

        같은 콘이 랩마다 다른 앵커로 갈라지는 것을 막는다. 원거리 s 추정이
        근접 수렴값보다 +0.36 m 앞서 있어(실측) 새 트랙의 첫 관측이 기존
        앵커 밴드 밖에 떨어지는 일이 생기고, 그렇게 생긴 조각들은 각자
        2랩씩 헛주입을 하다 carving 으로 사라진다. 갱신이 끝난 **뒤**
        합치면 그 왕복이 없어진다.
        """
        merged = True
        while merged and len(self.memory) > 1:
            merged = False
            for i in range(len(self.memory)):
                for j in range(i + 1, len(self.memory)):
                    a, b = self.memory[i], self.memory[j]
                    if abs(self._ds_signed(a['s'], b['s'])) > self.memory_match_s \
                            or abs(a['d'] - b['d']) > self.memory_match_d:
                        continue
                    # 관측 랩이 많은 쪽을 살리고, 확정/독 상태는 보수적으로
                    # 합친다 (한쪽이 독이면 합친 것도 독).
                    keep, drop = (a, b) if len(a['laps']) >= len(b['laps']) else (b, a)
                    keep['laps'] |= drop['laps']
                    keep['poisoned'] = keep['poisoned'] or drop['poisoned']
                    keep['misses'] = min(keep['misses'], drop['misses'])
                    keep['half'] = min(max(keep['half'], drop['half']),
                                       self.cyl_radius)
                    # 카빙 상태는 둘 다 비어 있을 때만 유지 — 한쪽이라도
                    # 점유 증거가 있으면 살아 있는 쪽을 믿는다 (fail-safe).
                    keep['absent'] = bool(keep.get('absent')) and \
                        bool(drop.get('absent'))
                    if not keep['absent']:
                        keep['absent_streak'] = 0
                    if not keep['confirmed'] and \
                            len(keep['laps']) >= self.memory_confirm_laps:
                        keep['confirmed'] = True
                    self.memory.remove(drop)
                    merged = True
                    break
                if merged:
                    break

    def _prune_memory(self):
        keep = []
        for a in self.memory:
            if a['poisoned'] or a['misses'] >= self.memory_forget_misses:
                self.get_logger().info(
                    f"랩 기억 삭제: s={a['s']:.2f} d={a['d']:+.2f} "
                    f"(misses={a['misses']}, poisoned={a['poisoned']}, "
                    f"carved={bool(a.get('absent'))})")
                continue
            keep.append(a)
        self.memory = keep

    def _ds_signed(self, a, b):
        """a - b 를 [-L/2, L/2) 로 감아서 돌려준다."""
        return ((a - b + self.track_length / 2.0) % self.track_length
                - self.track_length / 2.0)

    def _find_anchor(self, s, d):
        """밴드 안에서 **가장 가까운** 앵커와 그 cost. 첫 매칭이 아니다.

        첫 매칭으로 두면 같은 콘이 랩마다 새 앵커로 갈라진다 — 원거리
        s 추정이 근접 수렴값보다 +0.36 m 앞서 있기 때문이다(실측). 재생
        검증에서 S/F 콘 하나가 앵커 8개로 쪼개졌고, 그 조각들이 서로
        free-space carving 으로 지워지며 확정이 매 랩 리셋됐다.

        cost 는 밴드 정규화 타원 거리다. 라치/부활 판정(memory_observe)이
        "중심 매칭"과 "가장자리 매칭"을 갈라야 해서 함께 돌려준다 —
        run_0817_150059 에서 가장자리 한 프레임(cost 1.03)이 유령 앵커의
        근거리 게이트를 껐다.
        """
        best, best_cost = None, float('inf')
        for a in self.memory:
            ds = abs(self._ds_signed(s, a['s']))
            dd = abs(d - a['d'])
            if ds > self.memory_match_s or dd > self.memory_match_d:
                continue
            cost = (ds / max(self.memory_match_s, 1e-6)) ** 2 + \
                   (dd / max(self.memory_match_d, 1e-6)) ** 2
            if cost < best_cost:
                best, best_cost = a, cost
        return best, best_cost

    def cb_scan(self, msg: LaserScan):
        """스캔 빈자리 카빙 (선언부 주석). 앵커 자리의 빔 관통을 판정한다.

        프레임당 앵커 ≤ 12개 x 창 빔 ≤ 40개 순회라 40 Hz 에서도 무시할
        비용이다. 판정은 세 갈래다:
          hit    |r - r_a| <= slack     -> 실물(또는 slack 안 벽): 스트릭 리셋
          pass   r > r_a + slack        -> 관통: 빈자리 증거
          block  r < r_a - slack        -> 가림: 증거 없음 (스트릭 유지)
        hit 이 하나라도 있으면 그 프레임은 점유다. hit 0 + pass 충분이면
        빈자리, 그 외(가림/시야 밖/원거리)는 판정 보류다.
        """
        if not self.memory or self.wpnts is None or self.track_length <= 0.0 \
                or self.ego_s is None:
            return
        try:
            tf = self.tf_buffer.lookup_transform(
                self.target_frame, msg.header.frame_id,
                Time.from_msg(msg.header.stamp),
                timeout=Duration(seconds=self.tf_timeout))
        except tf2_ros.TransformException:
            return          # TF 가 없으면 판정하지 않는다 (증거 아님)
        t = tf.transform.translation
        q = tf.transform.rotation
        laser_yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                               1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        n_beams = len(msg.ranges)
        if n_beams < 8 or not msg.angle_increment > 0.0:
            return
        r_valid_min = max(float(msg.range_min), 0.02)
        r_open = float(msg.range_max) * 0.99
        for a in self.memory:
            if a['poisoned']:
                continue
            # **전방 앵커만** 판정한다. 재생 검증에서 나온 유일한 오탐 모드가
            # "옆을 지나치는 순간"(r 0.3~0.7 m) 카빙이었다 — 근접에서는 앵커
            # 위치오차(0.1~0.2 m)가 빔 창을 실물 밖으로 밀어낸다 (150059
            # 실물 벽콘 t=9.6 / 145548 s=3.5 콘 t=122.7). 주입은 어차피
            # 전방(inject_ahead)만 막으므로 옆/뒤 판정은 가치도 없다.
            ahead = (a['s'] - self.ego_s) % self.track_length
            if ahead < 0.5 or ahead > self.track_length / 2.0:
                continue
            try:
                ax, ay = self.frenet_to_cartesian(a['s'], a['d'])
            except RuntimeError:
                continue
            dx, dy = ax - t.x, ay - t.y
            r_a = math.hypot(dx, dy)
            if not self.memory_scan_range_min <= r_a \
                    <= self.memory_scan_range_max:
                continue
            bearing = math.atan2(dy, dx) - laser_yaw
            bearing = math.atan2(math.sin(bearing), math.cos(bearing))
            # 앵커 원통이 차지하는 각도 창. 폭이 아주 작아도 최소 0.10 m 로
            # 보고 판정한다 (한두 빔짜리 창은 노이즈에 휘둘린다).
            half_ang = math.atan2(max(a['half'], 0.10), r_a)
            i0 = int(math.ceil((bearing - half_ang - msg.angle_min)
                               / msg.angle_increment))
            i1 = int(math.floor((bearing + half_ang - msg.angle_min)
                                / msg.angle_increment))
            if i0 < 0 or i1 >= n_beams or i1 < i0:
                continue    # 창이 FOV 를 벗어남 — 판정 보류
            hits = 0
            passes = 0
            for i in range(i0, i1 + 1):
                r = msg.ranges[i]
                if not math.isfinite(r):
                    # Hokuyo 무반사는 inf/NaN — 그 방향은 끝까지 뚫린 것
                    passes += 1
                    continue
                if r < r_valid_min:
                    continue    # 0/근접 노이즈 — 무효 빔
                if r >= r_open or r > r_a + self.memory_scan_slack:
                    passes += 1
                elif r >= r_a - self.memory_scan_slack:
                    hits += 1
                # else: 앞에서 끊김(가림) — 증거 없음
            if hits > 0:
                a['absent_streak'] = 0
                continue
            if passes < self.memory_scan_min_pass:
                continue        # 가림/무효 위주 — 판정 보류, 스트릭 유지
            a['absent_streak'] = int(a['absent_streak']) + 1
            if not a['absent'] and \
                    a['absent_streak'] >= self.memory_scan_clear_frames:
                a['absent'] = True
                self.get_logger().info(
                    f"랩 기억 카빙: s={a['s']:.2f} d={a['d']:+.2f} — "
                    f"스캔 관통 {a['absent_streak']}프레임 "
                    f"(r={r_a:.2f}m), 주입 중단")

    def resolve_track_id(self, src_id, s, d, is_dynamic):
        """이 검출을 어느 트랙 id 로 취급할지 정한다 (선언부 주석 참조).

        같은 자리에 이미 충분히 관측된 **정적** 트랙이 있으면 그 id 를 쓴다.
        돌려주는 것은 이 검출을 기록/발행할 트랙 id 다. src_id 와 다르면
        같은 물체의 재등록/중복이라는 뜻이다.
        """
        if not self.reassoc_enabled or is_dynamic:
            # 동적으로 바뀐 트랙은 매핑을 끊는다 — 콘인 줄 알았던 것이
            # 움직이기 시작하면 그때부터는 별개로 추적해야 한다.
            self.reassoc.pop(src_id, None)
            return src_id

        def within(track):
            # 기준점은 트랙의 **누적 평균**이다. 마지막 위치로 재확인하면,
            # 흡수한 검출이 트랙 위치를 끌고 가고 그 위치로 다시 게이트를
            # 보게 되어 트랙이 이웃까지 끌려간다(drift chaining). 실측에서
            # 실제로 한 트랙이 1.0 m 떨어진 물체를 따라갔다.
            n = int(track.get('n', 0))
            a_s = float(track.get('mean_s', track['s'])) if n > 1 else track['s']
            a_d = float(track.get('mean_d', track['d'])) if n > 1 else track['d']
            return (abs(self._ds_signed(s, a_s)) <= self.reassoc_match_s
                    and abs(d - a_d) <= self.reassoc_match_d)

        # 이미 매핑돼 있으면 게이트만 재확인한다 (갈라져 나가면 끊는다).
        entry = self.reassoc.get(src_id)
        if entry is not None:
            target = self.tracks.get(entry['to'])
            if target is not None and within(target):
                return entry['to']
            self.reassoc.pop(src_id, None)

        # 이미 자기 트랙이 있으면 그대로 쓴다.
        if src_id in self.tracks:
            return src_id

        best, best_cost = None, float('inf')
        for tid, track in self.tracks.items():
            if tid == src_id or not track.get('static', False):
                continue
            if int(track.get('n', 0)) < self.reassoc_min_obs or not within(track):
                continue
            n_t = int(track.get('n', 0))
            a_s = float(track.get('mean_s', track['s'])) if n_t > 1 else track['s']
            a_d = float(track.get('mean_d', track['d'])) if n_t > 1 else track['d']
            cost = (abs(self._ds_signed(s, a_s)) / max(self.reassoc_match_s, 1e-6)) ** 2 \
                + (abs(d - a_d) / max(self.reassoc_match_d, 1e-6)) ** 2
            if cost < best_cost:
                best, best_cost = tid, cost
        if best is None:
            return src_id
        self.reassoc[src_id] = {'to': best, 'dup': 0}
        self.reassoc_absorbed += 1
        self.get_logger().info(
            f'트랙 재연결: 새 id={src_id} -> {best} '
            f'(s={s:.2f} d={d:+.2f}, 기존 관측 {self.tracks[best].get("n", 0)}회)',
            throttle_duration_sec=2.0)
        return best

    def hold_duplicate(self, src_id):
        """이 중복 검출을 계속 흡수해도 되는가.

        쪼개진 조각은 오래 못 산다 (run_0814 실측: 수명 p50 3프레임). 반대로
        **실제로 옆에 있는 다른 콘**은 수십~수백 프레임을 산다. 그래서 같은
        트랙에 계속 겹쳐 들어오는 검출은 일정 프레임까지만 흡수하고, 그보다
        오래 버티면 별개 물체로 인정해 풀어 준다 — 진짜 장애물을 지워 버리는
        쪽으로는 절대 기울지 않게.
        """
        entry = self.reassoc.get(src_id)
        if entry is None:
            return True                      # 매핑 없이 겹친 경우(같은 프레임 순서)
        entry['dup'] = int(entry.get('dup', 0)) + 1
        if entry['dup'] <= self.reassoc_max_dup_frames:
            return True
        self.reassoc.pop(src_id, None)
        return False

    def memory_observe(self, s, d, half, fwd, is_dynamic, measured):
        """실측 프레임 하나를 기억에 반영한다.

        `fwd` 는 ego 기준 **부호 있는** 전방거리 [-L/2, L/2) 다. 감기지 않은
        (s - ego) % L 을 쓰면 방금 지나친 장애물이 "31.9 m 앞"이 되어 아래
        거리 게이트가 통째로 뒤집힌다 — 재생 검증에서 실제로 S/F 콘이
        매 랩 POISON 됐다.

        확정 조건(대책 ①②): **실측 프레임**(is_visible)만 적립하고, 멀리서
        동적으로 보인 앵커는 영구히 독을 먹인다. 위치는 그 랩의 **전방
        최근접 관측**으로만 갱신한다 — 그게 기억이 주는 정확도의 전부다
        (근접 std 0.029 vs 판단거리 std 0.091).
        """
        if not self.memory_enabled or self.track_length <= 0.0:
            return
        # 앵커 폭 상한 = 원통 반경. 실측 발행에는 static_cylinder 고정이
        # 있는데 기억만 관측 클러스터 폭을 그대로 흡수하는 비대칭이었다 —
        # run_0817_145548 에서 half 0.26~0.32 앵커(콘 규격의 2배)가 확정돼
        # 주입 밴드 0.64 m 가 회랑을 실제보다 좁게 만들었다. 벽 접선/이웃
        # 콘이 물린 큰 클러스터도 기억은 콘 하나로 취급한다. 큰 물체로
        # 연습하는 날은 static_cylinder_radius_m 을 같이 올릴 것 (선언부).
        half = min(half, self.cyl_radius)
        # 근거리 동적 전환은 콘이 스치며 밀린 것이거나 근거리 중심점 걸음이다
        # (선언부 주석의 7건 실측). 그 거리에서 나온 동적 표는 무시한다.
        far_dynamic = is_dynamic and fwd >= self.memory_poison_min_dist
        a, match_cost = self._find_anchor(s, d)
        # 중심 매칭(cost <= latch 게이트)만이 라치를 세우고 카빙된 앵커를
        # 되살린다. 가장자리 매칭은 위치/랩 적립에는 쓰되 정책은 못 바꾼다
        # — run_0817_150059 의 유령 불사(가장자리 1프레임이 매 랩 misses 를
        # 리셋)와 근거리 게이트 무력화가 전부 가장자리 프레임이었다.
        central = a is not None and match_cost <= self.memory_match_latch_cost
        if a is None:
            if far_dynamic or not measured:
                return          # 동적/외삽으로는 새 앵커를 만들지 않는다
            if is_dynamic:
                return          # 근거리 동적은 새 앵커 근거로도 쓰지 않는다
            # ── 관련성 밴드 ────────────────────────────────────────────
            # 레이스라인을 실제로 밀어내지 못하는 것은 기억하지 않는다.
            # 기준은 "raceline(d=0) 을 그대로 달릴 때 이 물체와의 여유가
            # 요구 간격보다 좁은가" 다. 실측 근거: 접촉 5건과 s12 크리프의
            # 앵커는 전부 |d| <= 0.31 이고, 한 번도 문제가 없던 벽 근처
            # 파편은 |d| >= 0.53 이다. 이 밴드가 없으면 재생 검증 기준
            # 주입이 프레임당 1.28개까지 늘어난다 (레지스트리 12개 중
            # 절반이 |d| 0.74~1.8 의 벽 구조물).
            if abs(d) - half - self.cyl_radius > self.memory_relevant_margin:
                return
            a = {
                's': s, 'd': d, 'half': half,
                'laps': set(), 'confirmed': False, 'misses': 0,
                'poisoned': False, 'best_dist': float('inf'),
                'best_s': s, 'best_d': d, 'best_half': half,
                'mem_id': self.memory_next_id,
                'matched_this_pass': False, 'pass_open': False,
                'absent': False, 'absent_streak': 0,
            }
            self.memory_next_id += 1
            self.memory.append(a)
            central = True      # 방금 이 관측으로 만든 앵커다
        if far_dynamic:
            # 대책 ①: 먼 거리에서 상대차로 보인 자리는 영구히 기억하지 않는다.
            a['poisoned'] = True
            return
        if is_dynamic or not measured:
            # 근거리 동적 표는 독으로 치지 않되 위치 근거로도 쓰지 않는다
            # (그 프레임의 중심점이 실제로 흔들린 상태이므로).
            # 다만 "지나가면서 실제로 봤다"는 사실은 남겨 free-space carving 이
            # 멀쩡한 기억을 지우지 않게 한다.
            if measured and central:
                a['matched_this_pass'] = True
            return
        if central:
            a['matched_this_pass'] = True
            if a['absent']:
                # 스캔 카빙으로 꺼진 앵커를 실측 중심 매칭이 되살린다
                # (콘을 도로 갖다 놓은 경우). 관통 판정과 검출이 다투면
                # 검출이 이긴다 — 실물을 놓치는 쪽이 더 비싸다.
                a['absent'] = False
                a['absent_streak'] = 0
                self.get_logger().info(
                    f"랩 기억 부활: s={a['s']:.2f} d={a['d']:+.2f} "
                    f"(실측 재매칭, cost={match_cost:.2f})")
        # 위치 갱신은 **전방** 관측만 쓴다. 지나친 뒤의 프레임은 근거리
        # 중심점 걸음이 가장 심한 구간이라(정적->동적 전환 7건이 전부 거기)
        # 정확도를 올리려는 목적과 정반대로 움직인다.
        if 0.0 <= fwd < a['best_dist']:
            a['best_dist'] = fwd
            a['best_s'], a['best_d'], a['best_half'] = s, d, half
        a['laps'].add(self.lap_index)
        if not a['confirmed'] and len(a['laps']) >= self.memory_confirm_laps:
            a['confirmed'] = True
            self.get_logger().info(
                f"랩 기억 확정: s={a['s']:.2f} d={a['d']:+.2f} "
                f"half={a['half']:.2f} (랩 {sorted(a['laps'])})")

    def memory_inject(self, out, live_entries, now_sec):
        """확정된 기억 중 이번 프레임에 실측이 없는 것을 장애물로 주입한다.

        주입된 장애물은 **is_visible=False** 다. 이것이 계약이다 — 기억은
        측정이 아니므로, is_visible 을 요구하는 하류 게이트(BuildEscapeArc 의
        escape_not_visible_static 등)는 기억만으로는 발동하지 않는다.
        반대로 회랑 기하와 속도 상한에는 그대로 쓰인다(비대칭 사용).
        """
        if not self.memory_enabled or self.ego_s is None:
            return
        if self.ego_stamp_sec is not None and \
                now_sec - self.ego_stamp_sec > self.ego_frenet_timeout:
            return          # ego Frenet 이 낡으면 주입하지 않는다 (fail-closed)
        for a in self.memory:
            if a['poisoned']:
                continue
            ahead = (a['s'] - self.ego_s) % self.track_length
            if ahead > self.memory_inject_ahead:
                continue
            # 통과 판정 창을 연다: 기억 위치가 시야에 들어온 순간부터
            # 지나갈 때까지가 "이번 통과"다. **확정 여부와 무관하게** 연다 —
            # 확정된 것만 열면 미확정 앵커는 miss 를 못 쌓아 free-space
            # carving 이 안 걸리고 레지스트리가 영원히 자란다.
            a['pass_open'] = True
            if not a['confirmed']:
                continue
            # 스캔 카빙 (선언부 주석): 자리가 비어 있음을 빔 관통으로 증명한
            # 앵커는 랩 경계를 기다리지 않고 즉시 주입을 멈춘다. pass_open
            # 뒤에 두어 랩 정산(miss)은 그대로 돌아가게 한다.
            if a.get('absent'):
                continue
            # 근거리 게이트 (선언부 주석): 이번 통과에서 실측이 한 번도 안
            # 붙은 기억은 여기서 손을 뗀다. 유령을 코앞까지 끌고 가지 않고,
            # 실물은 이미 matched 라 드롭아웃 구간까지 그대로 덮는다.
            if ahead < self.memory_close_requires_match and \
                    not a['matched_this_pass']:
                continue
            # 라이브가 있으면 라이브가 이긴다 — 기억은 절대 실측을 덮지 않는다.
            if any(abs(self._ds_signed(a['s'], e[0])) <= self.memory_match_s and
                   abs(a['d'] - e[1]) <= self.memory_match_d
                   for e in live_entries):
                continue
            # 대책 ③: 살아 있는 동적 트랙 근처에서는 주입하지 않는다.
            if any(e[2] and abs(self._ds_signed(a['s'], e[0]))
                   <= self.memory_dynamic_suppress for e in live_entries):
                continue
            half = max(a['half'], 0.05)
            obs = Obstacle()
            obs.id = int(a['mem_id'])
            try:
                x, y = self.frenet_to_cartesian(a['s'], a['d'])
            except RuntimeError:
                continue
            obs.position.x, obs.position.y = float(x), float(y)
            obs.s_center = a['s']
            obs.d_center = a['d']
            obs.s_start = (a['s'] - half) % self.track_length
            obs.s_end = (a['s'] + half) % self.track_length
            obs.d_left = a['d'] + half
            obs.d_right = a['d'] - half
            obs.vs = 0.0
            obs.vd = 0.0
            var = max(self.memory_pos_std ** 2, self.position_var_floor)
            obs.s_var = var
            obs.d_var = var
            obs.vs_var = self.velocity_var_floor
            obs.vd_var = self.velocity_var_floor
            obs.size = 2.0 * half
            obs.is_static = True
            obs.is_visible = False
            out.obstacles.append(obs)

    def frenet_to_cartesian(self, s, d):
        """기억 앵커를 map 좌표로 되돌린다.

        cartesian_to_frenet 의 정확한 역이어야 한다 — 거기서 psi 는 저장된
        psi_rad 가 아니라 **세그먼트 방향**이므로 여기서도 세그먼트로 푼다
        (둘이 어긋나면 왕복이 닫히지 않아 기억 위치가 조금씩 흐른다).
        """
        if self.wpnts is None or self.track_length <= 0.0:
            raise RuntimeError('waypoints unavailable')
        n = len(self.wpnts)
        i = int(np.searchsorted(self.wpnts[:, 0], s % self.track_length)) - 1
        i = min(max(i, 0), n - 1)
        j = (i + 1) % n
        p0 = self.wpnts[i, 1:3]
        p1 = self.wpnts[j, 1:3]
        seg = p1 - p0
        seg_len = float(np.linalg.norm(seg))
        if seg_len < 1e-9:
            raise RuntimeError('degenerate waypoint segment')
        ds = ((self.wpnts[j, 0] - self.wpnts[i, 0]) % self.track_length)
        u = 0.0 if ds <= 1e-9 else float(
            np.clip((s % self.track_length - self.wpnts[i, 0]) / ds, 0.0, 1.0))
        proj = p0 + u * seg
        psi = math.atan2(float(seg[1]), float(seg[0]))
        return (float(proj[0]) - d * math.sin(psi),
                float(proj[1]) + d * math.cos(psi))

    def cb_map(self, msg: OccupancyGrid):
        """노드 수명 중 1회: 셀별 최근접 벽거리 테이블(EDT) 계산."""
        if self.wall_dist is not None:
            return
        if msg.info.width == 0 or msg.info.height == 0 or \
                not math.isfinite(msg.info.resolution) or \
                msg.info.resolution <= 0.0:
            self.get_logger().error('invalid occupancy grid; map filter unavailable')
            return
        expected_cells = int(msg.info.width) * int(msg.info.height)
        q = msg.info.origin.orientation
        map_values = (
            msg.info.origin.position.x, msg.info.origin.position.y,
            q.x, q.y, q.z, q.w,
        )
        q_norm = math.sqrt(q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w)
        if (len(msg.data) != expected_cells
                or not all(math.isfinite(v) for v in map_values)
                or q_norm < 1e-9):
            self.get_logger().error(
                'occupancy grid data/origin is malformed; map filter unavailable')
            return
        grid = np.array(msg.data, dtype=np.int8).reshape(
            msg.info.height, msg.info.width)
        # unknown도 free로 증명되지 않았으므로 벽으로 취급한다.
        occupied = (grid < 0) | (grid >= 65)
        self.wall_dist = distance_transform_edt(~occupied) * msg.info.resolution
        self.map_origin = (msg.info.origin.position.x, msg.info.origin.position.y)
        qx, qy, qz, qw = (
            q.x / q_norm, q.y / q_norm, q.z / q_norm, q.w / q_norm)
        self.map_origin_yaw = math.atan2(
            2.0 * (qw * qz + qx * qy),
            1.0 - 2.0 * (qy * qy + qz * qz))
        self.map_res = msg.info.resolution
        self.get_logger().info(
            f'맵 벽거리 필터 활성: {msg.info.width}x{msg.info.height} '
            f'res={msg.info.resolution:.3f} wall_dist_min={self.wall_dist_min:.2f}m')

    def wall_distance(self, x, y):
        """map 좌표의 최근접 벽거리. 맵 밖/미수신은 None."""
        if self.wall_dist is None:
            return None
        dx, dy = x - self.map_origin[0], y - self.map_origin[1]
        c_yaw, s_yaw = math.cos(self.map_origin_yaw), math.sin(self.map_origin_yaw)
        grid_x = c_yaw * dx + s_yaw * dy
        grid_y = -s_yaw * dx + c_yaw * dy
        c = int(math.floor(grid_x / self.map_res))
        r = int(math.floor(grid_y / self.map_res))
        h, w = self.wall_dist.shape
        if 0 <= r < h and 0 <= c < w:
            return float(self.wall_dist[r, c])
        return None

    def cb_gb(self, msg: WpntArray):
        candidate = np.array([
            [w.s_m, w.x_m, w.y_m, w.psi_rad, w.d_right, w.d_left]
            for w in msg.wpnts], dtype=float)
        if len(candidate) < 2 or not np.isfinite(candidate).all() or \
                np.any(np.diff(candidate[:, 0]) <= 1e-5) or \
                np.any(candidate[:, 4:6] < 0.0):
            self.get_logger().error(
                'invalid global waypoints; bridge projection disabled',
                throttle_duration_sec=2.0)
            self.wpnts = None
            self.track_length = 0.0
            return
        segment_lengths = np.linalg.norm(np.diff(candidate[:, 1:3], axis=0), axis=1)
        ds = np.diff(candidate[:, 0])
        if (np.any(segment_lengths <= 1e-5)
                or np.any(np.abs(segment_lengths - ds)
                          > np.maximum(0.05, 0.25 * ds))):
            self.get_logger().error(
                'global waypoint geometry/s spacing mismatch; '
                'bridge projection disabled',
                throttle_duration_sec=2.0)
            self.wpnts = None
            self.track_length = 0.0
            return
        closure = float(np.linalg.norm(candidate[0, 1:3] - candidate[-1, 1:3]))
        track_length = candidate[-1, 0] - candidate[0, 0] + closure
        if not math.isfinite(track_length) or track_length <= 0.0:
            self.wpnts = None
            self.track_length = 0.0
            return
        self.wpnts = candidate
        self.track_length = track_length

    def cartesian_to_frenet(self, x, y, hint=None):
        """연속 segment 투영. 기존 track id가 있으면 인접 branch만 탐색한다.

        후보 세그먼트를 **한 번에** 계산한다 (2026-08-14). 예전에는 후보를
        파이썬 루프로 돌면서 원소 2개짜리 벡터에 np.dot / np.array 를 세그먼트당
        8~9회 불렀는데, numpy 는 호출당 고정비(이 Jetson 에서 ~4.5 us)가 붙는
        도구라 **계산보다 호출 포장비가 100배 컸다.**
          Jetson 실측(웨이포인트 320, projection_window 80 = 후보 161):
            같은 161회 루프를 순수 파이썬 산술로만  :    63.6 us
            실제 이 함수                            :  6348.7 us   <- 99% 가 호출비
            아래 벡터화                             :   104.0 us   (61배)
        브릿지는 이 함수를 **장애물 1건마다** 부르고 인지는 15.4 Hz(주기 65 ms)
        이므로, 트랙 5개면 32 ms 로 프레임 예산의 49% 를 여기서 썼다.
        (참고로 같은 로직의 C++ 은 8.2 us 다. 즉 남은 96 us 는 언어 문제지만
         프레임 예산의 0.15% 라, 포팅으로 얻을 것은 여기 없다.)

        결과는 예전 구현과 동일해야 한다. 유지되는 규약 3가지:
          · 후보 순서 그대로 + np.argmin 은 **첫** 최소를 돌려주므로, 동률에서
            먼저 나온 세그먼트가 이기던 `dist2 < best[0]`(strict) 와 같다.
          · degenerate(seg2 < 1e-10) 세그먼트는 '후보에서 제외'다. dist2 를
            inf 로 채워 argmin 이 절대 고르지 못하게 하고, 전부 degenerate 면
            예전처럼 RuntimeError 를 낸다.
          · dist2 는 np.dot(2원소) 과 같은 성분곱-합 순서로 계산한다.
        """
        n = len(self.wpnts)
        if hint is None:
            idx = np.arange(n, dtype=int)
        else:
            w = max(self.projection_window, 4)
            idx = np.arange(hint - w, hint + w + 1, dtype=int) % n
        jdx = (idx + 1) % n

        x0 = self.wpnts[idx, 1]
        y0 = self.wpnts[idx, 2]
        seg_x = self.wpnts[jdx, 1] - x0
        seg_y = self.wpnts[jdx, 2] - y0
        seg2 = seg_x * seg_x + seg_y * seg_y
        usable = seg2 >= 1e-10
        # degenerate 세그먼트는 아래에서 inf 로 배제된다. 0 나눗셈 경고만 피한다.
        u = np.clip(((x - x0) * seg_x + (y - y0) * seg_y)
                    / np.where(usable, seg2, 1.0), 0.0, 1.0)
        off_x = x - (x0 + u * seg_x)
        off_y = y - (y0 + u * seg_y)
        dist2 = np.where(usable, off_x * off_x + off_y * off_y, np.inf)

        k = int(np.argmin(dist2))
        if not bool(usable[k]):
            raise RuntimeError('global waypoint segments are degenerate')
        if hint is not None and math.sqrt(float(dist2[k])) > \
                self.projection_reacquire_distance:
            return self.cartesian_to_frenet(x, y, hint=None)

        i, j = int(idx[k]), int(jdx[k])
        u_k = float(u[k])
        psi = math.atan2(float(seg_y[k]), float(seg_x[k]))
        d = -float(off_x[k]) * math.sin(psi) + float(off_y[k]) * math.cos(psi)
        right = (1.0 - u_k) * self.wpnts[i, 4] + u_k * self.wpnts[j, 4]
        left = (1.0 - u_k) * self.wpnts[i, 5] + u_k * self.wpnts[j, 5]
        ds = (self.wpnts[j, 0] - self.wpnts[i, 0]) % self.track_length
        s = (self.wpnts[i, 0] + u_k * ds) % self.track_length
        return s, d, i, right, left

    def cb_obs(self, msg: PerceptionObstacleArray):
        if self.wpnts is None or self.track_length <= 0.0:
            return

        # 맵 필터를 켰는데 맵이 안 오면(맵서버 미기동/죽음) frenet 필터만으로
        # 동작 중임을 알린다 — 조용히 새는 유령의 원인 파악용
        if self.use_map_filter and self.wall_dist is None:
            waited = (self.get_clock().now() - self.start_time).nanoseconds * 1e-9
            if waited > self.map_timeout:
                self.get_logger().warn(
                    f'/map {self.map_timeout:.0f}s 내 미수신 — wall filter unavailable',
                    throttle_duration_sec=10.0)
            if self.map_filter_required:
                # 빈 obstacle을 발행하면 downstream이 이를 "센서 정상 + clear"로
                # 오해한다. health lease가 만료되도록 아예 발행하지 않는다.
                return

        out = ObstacleArray()
        out.header.stamp = msg.header.stamp
        out.header.frame_id = self.target_frame

        # source timestamp의 TF만 사용한다. latest fallback은 명시 opt-in이며,
        # 허용하더라도 최대 시각차를 넘으면 프레임 전체를 폐기한다.
        tf = None
        stamp_time = Time.from_msg(msg.header.stamp)
        stamp_is_zero = stamp_time.nanoseconds == 0
        if stamp_is_zero and self.require_source_stamp:
            self.get_logger().warn(
                '장애물 source timestamp가 없어 frame을 거부함',
                throttle_duration_sec=2.0)
            return
        if not stamp_is_zero:
            source_age = (
                self.get_clock().now() - stamp_time).nanoseconds * 1e-9
            if source_age < -self.source_future_tolerance or \
                    source_age > self.max_source_age:
                self.get_logger().warn(
                    f'장애물 source age {source_age:.3f}s가 허용 범위를 벗어남',
                    throttle_duration_sec=2.0)
                return
        source_ns = stamp_time.nanoseconds or self.get_clock().now().nanoseconds
        if self.last_source_ns is not None and source_ns <= self.last_source_ns:
            self.get_logger().warn(
                'duplicate/out-of-order obstacle source timestamp; frame rejected',
                throttle_duration_sec=2.0)
            return
        if not msg.header.frame_id:
            self.get_logger().warn(
                '장애물 frame_id가 없어 frame을 거부함',
                throttle_duration_sec=2.0)
            return
        if msg.header.frame_id != self.target_frame:
            try:
                tf = self.tf_buffer.lookup_transform(
                    self.target_frame, msg.header.frame_id,
                    Time() if stamp_is_zero else stamp_time,
                    timeout=Duration(seconds=self.tf_timeout))
            except tf2_ros.TransformException as exact_error:
                if self.tf_allow_latest and not stamp_is_zero:
                    try:
                        tf = self.tf_buffer.lookup_transform(
                            self.target_frame, msg.header.frame_id, Time())
                        tf_time = Time.from_msg(tf.header.stamp)
                        tf_age = abs((tf_time - stamp_time).nanoseconds) * 1e-9
                        if tf_age > self.tf_max_age:
                            raise tf2_ros.ExtrapolationException(
                                f'latest TF age {tf_age:.3f}s > {self.tf_max_age:.3f}s')
                    except tf2_ros.TransformException as e:
                        self.get_logger().warn(
                            f'TF {msg.header.frame_id}->{self.target_frame} invalid: {e}',
                            throttle_duration_sec=2.0)
                        return
                else:
                    self.get_logger().warn(
                        f'TF {msg.header.frame_id}->{self.target_frame} at source stamp '
                        f'unavailable: {exact_error}', throttle_duration_sec=2.0)
                    return
        if tf is not None:
            q = tf.transform.rotation
            tx, ty = tf.transform.translation.x, tf.transform.translation.y
            tf_values = (tx, ty, q.x, q.y, q.z, q.w)
            q_norm = math.sqrt(q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w)
            if (not all(math.isfinite(value) for value in tf_values)
                    or q_norm < 1e-9):
                self.get_logger().error(
                    'TF contains non-finite/invalid values; obstacle frame rejected',
                    throttle_duration_sec=2.0)
                return
            qx, qy, qz, qw = (
                q.x / q_norm, q.y / q_norm, q.z / q_norm, q.w / q_norm)
            yaw = math.atan2(2.0 * (qw * qz + qx * qy),
                             1.0 - 2.0 * (qy * qy + qz * qz))
            cos_y, sin_y = math.cos(yaw), math.sin(yaw)

        arrival_sec = self.get_clock().now().nanoseconds * 1e-9
        now_sec = stamp_time.nanoseconds * 1e-9
        if stamp_is_zero:
            now_sec = arrival_sec
            out.header.stamp = self.get_clock().now().to_msg()
        seen_src = set()          # 이번 프레임의 **detector** id (중복 검사용)
        seen_ids = set()          # 이번 프레임에 갱신된 **트랙** id
        # 랩 기억용: 이번 프레임에 살아남은 장애물의 (s, d, is_dynamic).
        live_entries = []

        for src in msg.obstacles:
            if int(src.id) >= _MEMORY_ID_BASE:
                # detector id 가 기억 대역에 침범하면 두 상태가 섞인다.
                # 조용히 버리지 않고 프레임을 거부해 원인을 드러낸다.
                self.get_logger().error(
                    f'detector id={src.id}가 기억 id 대역({_MEMORY_ID_BASE}+)과 '
                    '충돌; frame 거부', throttle_duration_sec=2.0)
                return
            if int(src.id) < 0 or int(src.id) in seen_src:
                self.get_logger().error(
                    f'negative/duplicate obstacle id={src.id}; frame rejected',
                    throttle_duration_sec=2.0)
                return
            values = (
                float(src.confidence), float(src.radius),
                float(src.position.x), float(src.position.y),
            )
            if (not all(math.isfinite(value) for value in values)
                    or not 0.0 <= src.confidence <= 1.0
                    or src.radius < 0.0):
                # 일부 obstacle만 조용히 버리면 그 물체가 사라진 것으로 보인다.
                # frame 전체를 중단해 downstream health lease가 fail-closed 되게 한다.
                self.get_logger().error(
                    f'비유한/음수 obstacle 입력(id={src.id}); frame 거부',
                    throttle_duration_sec=2.0)
                return
            # ── 존재 판정과 분류 안정성을 분리한다 (2026-08-12) ──────────────
            # confidence 는 "존재 확률"이 **아니다**. classifier.cpp:78 의
            #   confidence = static_streak / n_release   (정적 트랙)
            # 즉 "정적/동적 판정까지의 진행률"이다. 그리고 static_streak 은
            #   moving = (speed >= v_thresh 0.40) or (spread >= dynamic_pos_std 0.30)
            # 이 **한 프레임만** 참이면 0으로 리셋된다.
            #
            # 그 진행률을 존재 필터로 쓰면 범주 오류가 된다. 실제 사고:
            #   run_0812_013630 3:14 — 트랙 205 는 id 가 끊긴 적 없고
            #   is_visible=1(매 프레임 LiDAR 실측, misses=0)이었는데, 접근 중
            #   콘의 단면이 바뀌며(radius 0.22->0.21->0.19->0.16) 중심점이 몇 cm
            #   튀어 한 프레임이 moving 으로 읽혔다. confidence 0.45 -> 0.00.
            #   그 뒤 0.05/프레임으로 회복해 0.20 에 닿기까지 **4프레임=0.25s**,
            #   그동안 이 게이트가 콘을 지웠다. 플래너는 "앞이 비었다"로 보고
            #   계획 속도를 5.00 -> 6.50 m/s 로 올렸고, 장애물이 1.99m 앞에서
            #   돌아왔을 때는 어떤 회피도 성립하지 않았다.
            #
            # 교차표 (같은 백, 원시 검출 2087 트랙-프레임):
            #             vis=1(실측)  vis=0(외삽)
            #   conf>=0.2      839         510   <- 유령 510건이 통과 (게이트가 샌다)
            #   conf< 0.2      217         521   <- 실물 217건이 삭제 (위 사고)
            # 초기 획득 거리 손실은 0.00 m 다 — min_hits(3)가 이미 그 역할을 하므로
            # 이 게이트의 손실은 전부 **주행 중 리셋**에서 나오고, 하필 근거리에서 터진다.
            #
            # 그래서 게이트를 없애지 않고 **적용 대상만 좁힌다**:
            #   · is_visible(이번 프레임 실측) 이면 통과 — 존재는 측정이 증명한다
            #   · 외삽(coasting) 이면 종전대로 confidence 게이트 유지
            # 두 축 모두에서 종전보다 나쁘지 않다: 실물 217건을 되살리고,
            # 코스팅 처리는 그대로다(= 가려진 실물을 새로 버리지 않는다).
            # 남은 누수(conf>=0.2 인 코스팅 510건)는 여기서 더 조이지 않았다 —
            # 조이면 짧게 가려진 **실물**을 지우게 되고, 그건 같은 사고의 반대
            # 방향이다. 그쪽은 footprint-corridor overlap 이 담당한다.
            measured = bool(src.is_visible)
            if src.confidence < self.min_confidence:
                if not (self.visible_bypasses_confidence and measured):
                    continue
            # 분류 진행률이 낮은 트랙은 **버리지 않고 보수적으로 취급**한다.
            # 정적으로 보면 기하 회피(회랑 판정)로 처리되고, 동적으로 잘못 보면
            # 없는 운동을 예측해 회랑을 엉뚱하게 비운다 — 후자가 더 위험하다.
            # (spliner 의 cls_* 가 몇 프레임 뒤 어차피 재분류한다.)
            unsettled_class = src.confidence < self.min_confidence
            x, y = src.position.x, src.position.y
            if tf is not None:
                x, y = cos_y * x - sin_y * y + tx, sin_y * x + cos_y * y + ty

            # 맵 벽거리 필터: frenet d는 코너에서 실제 벽거리와 어긋나
            # 벽 위 유령이 아래 회랑 검사를 통과한다 — 맵 EDT로 실거리 판정.
            # 유령은 벽에서 0~0.1m, 진짜 장애물은 최소 자기 반폭만큼 떨어져
            # 있어 분리가 물리적으로 명확하다. 맵 미수신 시 폴백(통과).
            #
            # ⚠️ 이 블록은 **투영 앞**에 있어야 한다 (2026-08-14 이동).
            # 판정에 (x, y)만 쓰므로 frenet 이 필요 없는데, 예전에는 투영 뒤에
            # 있어서 여기서 버려질 벽 유령까지 전부 투영하고 있었다. 투영은
            # 브릿지에서 가장 비싼 연산이고(cartesian_to_frenet 주석 참조),
            # 유령은 실물만큼 자주 뜬다 — min_confidence 교차표 기준 코스팅
            # 유령만 510건/2087. 회랑 검사(d 필요)는 그대로 투영 뒤에 남는다.
            # ⚠️ wall_distance() 의 None 은 **두 가지**가 합쳐진 값이다
            # (2026-08-14 분리). 필터를 투영 앞으로 옮기기 전에는 회랑 검사가
            # 먼저라 둘 다 조용히 continue 됐지만, 옮긴 뒤로는 "격자 밖" 하나가
            # 프레임 전체를 죽인다:
            #   run_0813_221645 실측 — 격자 밖 11건이 전부 t=192.48~192.70
            #   한 뭉치이고 d -1.12~-1.32 로 **회랑 밖**이다. 프레임 거부면
            #   0.22 s 연속 장애물 블랙아웃이 되고, 이는 local_trajectory 의
            #   "회피 경로 상실" 홀드를 트리거하는 20~30 ms 공백의 7배다.
            # 그래서 두 경우를 갈라 쓴다:
            #   · 맵 미수신 (self.wall_dist is None) -> 종전대로 통과(폴백).
            #     map_filter_required=True 면 이 콜백 앞부분이 이미 프레임을
            #     거부하므로, 여기로 오는 것은 required=False 인 경우뿐이다.
            #   · 격자 밖 좌표      -> 이 장애물만 삭제. 맵 밖 점은 트랙 위에
            #     있을 수 없으므로 프레임을 죽일 이유가 없다.
            if self.use_map_filter and self.wall_dist is not None:
                wall_dist = self.wall_distance(x, y)
                if wall_dist is None:
                    continue        # 격자 밖 — 트랙 위일 수 없다
                if wall_dist < self.wall_dist_min:
                    # centroid 기준으로 판정한다. 예전 조건
                    # (wall_dist + raw_half < wall_dist_min)은 raw_half가
                    # wall_dist_min보다 크면 중심이 벽 픽셀 위(거리 0)여도
                    # 절대 삭제되지 않아 필터가 사실상 무력했다 —
                    # run_0803_213313에서 벽거리 0.000m 유령(radius 0.31~0.40)이
                    # 그대로 통과해 spliner TRAILING 정지 라치를 유발했다.
                    # 물리적 분리는 명확하다: 유령 중심은 벽 위(실측 p50 0.000m),
                    # 진짜 차량(0.30m 폭)은 벽에 붙어도 중심이 >=0.15m 떨어진다.
                    continue

            prev = self.tracks.get(src.id)
            if prev is None:
                # 재연결 후보의 wi 를 투영 힌트로 쓴다 (같은 자리이므로 맞다).
                entry = self.reassoc.get(int(src.id))
                prev_hint = (self.tracks.get(entry['to'])
                             if entry is not None else None)
                hint = prev_hint.get('wi') if prev_hint is not None else None
            else:
                hint = prev.get('wi')
            try:
                s, d, wi, d_right_bound, d_left_bound = \
                    self.cartesian_to_frenet(x, y, hint)
            except RuntimeError as error:
                self.get_logger().error(
                    f'Frenet projection failed: {error}; frame rejected',
                    throttle_duration_sec=2.0)
                return
            # ── 같은 물체의 재등록/중복이면 기존 트랙 id 로 취급한다 ────────
            # 여기서 정하는 track_id 가 (1) Welford 누적의 키, (2) 발행 id 다.
            # 발행 id 를 유지해야 spliner 의 회피 커밋이 "새 장애물"로 깨지지
            # 않는다 (commit_released_new_obs).
            track_id = self.resolve_track_id(
                int(src.id), s, d, bool(src.is_dynamic))
            if track_id in seen_ids and self.hold_duplicate(int(src.id)):
                # 한 트랙에 이번 프레임의 검출이 둘 붙었다 = 한 콘이 둘로
                # 쪼개진 프레임. 먼저 처리된 쪽만 남긴다.
                # (재연결 여부로 나누지 않는다 — 쪼개진 조각이 원본보다
                #  msg 앞에 올 수 있고, 그때 조건을 걸면 같은 id 를 가진
                #  장애물 두 개가 발행된다. 하류는 id 로 트랙을 관리한다.)
                # 폭을 합치지도 않는다: 0.30 m 콘 두 개가 겹쳐 보이는 것이라
                # 합치면 실물보다 넓어져 회랑(요구 0.52 m)을 오히려 막는다.
                self.get_logger().info(
                    f'중복 검출 흡수: id={src.id} (트랙 {track_id}, '
                    f's={s:.2f} d={d:+.2f})', throttle_duration_sec=2.0)
                seen_src.add(int(src.id))
                continue
            if track_id != int(src.id) and track_id in seen_ids:
                # hold_duplicate 가 False = 중복이 너무 오래간다. 쪼개진
                # 조각은 실측상 p50 3프레임이면 사라지므로, 이만큼 버티는
                # 것은 **옆에 실제로 있는 다른 물체**로 봐야 한다. 별개
                # 트랙으로 풀어 준다 (흡수 이전 동작 = 안전한 쪽).
                self.get_logger().warn(
                    f'중복 흡수 해제: id={src.id} 가 트랙 {track_id} 옆에서 '
                    f'{self.reassoc_max_dup_frames}프레임 넘게 유지됨 — 별개 '
                    f'장애물로 되돌린다 (s={s:.2f} d={d:+.2f})',
                    throttle_duration_sec=2.0)
                track_id = int(src.id)
            prev = self.tracks.get(track_id)
            raw_half = max(float(src.radius), 0.05)
            # 실제 extent를 줄여 보고하면 큰 장애물의 끝을 통과 가능한 것으로
            # 오판한다. 큰 wall cluster도 보수적으로 전부 유지해 정지시키며,
            # 유령 제거는 map filter가 담당한다.
            half = raw_half
            if raw_half > _LARGE_EXTENT_WARN_M:
                self.get_logger().warn(
                    f'large obstacle extent preserved: id={src.id}, '
                    f'radius={raw_half:.2f}m',
                    throttle_duration_sec=2.0)
            # 고정 원통 모델 (선언부 주석의 세 조건).
            # 벽거리는 **중심 기준**이다. 가장자리 기준(wd - raw_half)으로 두면
            # 사실상 wd > 0.30 + raw_half ~= 0.48 을 요구하게 되어, 실측상 가장
            # 깨끗한 구간(벽거리 0.30~0.50 m, 457프레임, radius p50 0.13 —
            # 가정값 0.15 와 거의 일치)이 통째로 빠진다. 오염이 실제로 관측되는
            # 건 중심이 0.30 m 안쪽일 때뿐이다(거기서만 radius p50 이 0.20 으로
            # 뜬다). 물리적으로도 같은 결론이다 — 중심 0.30 이면 표면이 벽에서
            # 0.15 m 떨어져 있어 클러스터가 벽 점까지 물지 못한다.
            # 실측 A/B(run_0812_013630, 36트랙): 가장자리 기준은 적용률 34% /
            # 변동폭 0.140->0.107 m, 중심 기준은 69% / 0.140->0.084 m 이고
            # 적용당 위험 비율(11% vs 13%)과 최대 손실(0.098 vs 0.099 m)은
            # 사실상 같다 — 가장자리 기준은 더 안전한 게 아니라 덜 할 뿐이다.
            #
            # 맵 미수신(wall_distance None)이면 적용하지 않는다: 벽 근처인지
            # 모르는 채 extent 를 줄이는 것이 이 변경의 유일한 위험 방향이다.
            # ⚠️ 입력(f1tenth_perception_msgs/Obstacle)에는 is_static 이 없다 —
            # is_dynamic 이다. is_static 은 이 노드가 아래에서 만들어 내보내는
            # **출력**(planning) 필드다. 여기서 src.is_static 을 읽어 노드가
            # 죽었다(첫 장애물 프레임에서 AttributeError -> 스트림 중단 ->
            # spliner fail-safe stop). 여기서는 원시 분류를 그대로 쓴다:
            # unsettled_class 로 정적 취급된 트랙까지 포함시키면 "분류가 아직
            # 안 선" 물체의 extent 를 줄이게 되고, 그건 이 변경의 유일한 위험
            # 방향이다.
            if (not src.is_dynamic) and self.cyl_lo <= raw_half <= self.cyl_hi:
                wd = self.wall_distance(x, y)
                if wd is not None and wd > self.cyl_open_space:
                    half = self.cyl_radius

            # centroid가 아니라 obstacle footprint와 drivable corridor의 overlap을
            # 본다. 벽 가까운 실제 물체가 차로로 돌출하면 삭제하면 안 된다.
            # ⚠️ 여기는 **raw_half** 를 쓴다 (고정 전 값). 삭제 판정이라
            # extent 를 줄이면 "살짝 걸친 진짜 물체"를 지우는 쪽으로 움직인다 —
            # 고정 원통은 보고값에만 적용하고 존재 판정은 원시 extent 로 둔다.
            corridor_lo = -d_right_bound + self.wall_margin
            corridor_hi = d_left_bound - self.wall_margin
            if d - raw_half > corridor_hi or d + raw_half < corridor_lo:
                continue

            # (맵 벽거리 필터는 투영 앞으로 옮겼다 — 위 주석 참조.)

            # id별 시간차분으로 map 기준 vs/vd 추정.
            #
            # 상한 1.35 s 는 인지의 재획득 창을 덮으려고 고른 값이다
            # (2026-08-14). perception 의 dynamic_frenet_reid_timeout_sec 은
            # 1.25 s 라, 코너/FOV 이탈로 1.25 s 까지 가려졌던 상대도 **같은
            # id 로** 돌아온다. 그런데 Tracker::reacquire() 는 코너 진입 전의
            # 접선 속도를 버리려고 KF 를 reset_to_measurement() 로 되돌린다
            # — 즉 부활 직후 프레임의 src.velocity 는 **0** 이다.
            # 상한이 1.0 s 이던 동안에는 1.0~1.25 s 공백이 정확히 아래 시드
            # 경로로 떨어져서, 그 0 을 그대로 받아 상대차를 2~3 프레임
            # (~0.13~0.2 s) 동안 정지물로 발행했다. 두 창이 서로 모르고
            # 정해진 탓이라 재현성 있게 반복된다.
            # 1.25 s 공백을 가로지른 위치 차분은 그 구간의 **평균** 속도지만,
            # 리셋된 0 보다는 낫고 아래 max_abs_vs/vd 클립과 lpf_alpha 가
            # 그대로 걸린다. 인지 창을 줄이는 대신 이쪽을 넓힌 것은, 오래
            # 가려진 상대도 같은 id 로 추적하는 것이 그 커밋의 목적이라서다.
            vs, vd = 0.0, 0.0
            velocity_valid = False
            if prev is not None:
                dt = now_sec - prev['t']
                if 1e-3 < dt < 1.35:
                    ds = (s - prev['s'] + self.track_length / 2.0) % self.track_length \
                        - self.track_length / 2.0
                    a = self.lpf_alpha
                    vs = a * (ds / dt) + (1.0 - a) * prev['vs']
                    vd = a * ((d - prev['d']) / dt) + (1.0 - a) * prev['vd']
                    velocity_valid = True
            if not velocity_valid:
                # 첫 관측/재획득: detector의 칼만필터 속도를 Frenet으로 투영해
                # 시드로 쓴다. detector는 target_frame(=map)에서 트래킹하므로
                # 이 속도에는 ego 이동이 섞여 있지 않다 — "laser 프레임이라
                # 신뢰 불가"라던 옛 주석은 target_frame이 base_link였던 시절
                # 것이고 지금은 성립하지 않는다. 게다가 detector는 min_hits(3)
                # 관측을 채운 트랙만 발행하므로 이 시점의 KF 속도는 이미
                # 3프레임 이상의 근거를 갖는다.
                vx_m, vy_m = float(src.velocity.x), float(src.velocity.y)
                if tf is not None:
                    vx_m, vy_m = (cos_y * vx_m - sin_y * vy_m,
                                  sin_y * vx_m + cos_y * vy_m)
                psi = float(self.wpnts[wi, 3])
                c_psi, s_psi = math.cos(psi), math.sin(psi)
                seed_vs = vx_m * c_psi + vy_m * s_psi
                seed_vd = -vx_m * s_psi + vy_m * c_psi
                if (math.isfinite(seed_vs) and math.isfinite(seed_vd)
                        and abs(seed_vs) <= self.max_abs_vs
                        and abs(seed_vd) <= self.max_abs_vd):
                    vs, vd = seed_vs, seed_vd
            if abs(vs) > self.max_abs_vs or abs(vd) > self.max_abs_vd:
                velocity_valid = False
                vs = float(np.clip(vs, -self.max_abs_vs, self.max_abs_vs))
                vd = float(np.clip(vd, -self.max_abs_vd, self.max_abs_vd))
            # ── 확정 정적 트랙의 중심 누적 (Welford) ──────────────────────
            # 정적일 때만 쌓는다. 동적으로 분류되면 통계를 버려서, 오분류가
            # 나중에 풀렸을 때 과거 위치가 평균에 남지 않게 한다.
            raw_static = not src.is_dynamic
            n_prev = int(prev.get('n', 0)) if prev is not None else 0
            if raw_static and prev is not None and n_prev > 0:
                n_obs = n_prev + 1
                ms_prev = float(prev.get('mean_s', s))
                md_prev = float(prev.get('mean_d', d))
                # s 는 wrap 이 있으므로 이전 평균 기준 최단 편차로 누적한다.
                ds_w = (s - ms_prev + self.track_length / 2.0) \
                    % self.track_length - self.track_length / 2.0
                mean_s = (ms_prev + ds_w / n_obs) % self.track_length
                dd = d - md_prev
                mean_d = md_prev + dd / n_obs
                m2_d = float(prev.get('m2_d', 0.0)) + dd * (d - mean_d)
            else:
                n_obs, mean_s, mean_d, m2_d = 1, s, d, 0.0
            self.tracks[track_id] = {
                't': now_sec, 'arrival_t': arrival_sec, 's': s, 'd': d,
                'vs': vs, 'vd': vd, 'wi': wi, 'static': raw_static,
                'n': n_obs, 'mean_s': mean_s, 'mean_d': mean_d, 'm2_d': m2_d}
            seen_ids.add(track_id)
            seen_src.add(int(src.id))
            entry = self.reassoc.get(int(src.id))
            if entry is not None:
                # 이번 프레임엔 겹치지 않았다 = 원본이 사라진 재등록(hand-off).
                # 연속 중복 카운터를 되돌려, 잠깐씩 겹치는 것만으로 해제되지
                # 않게 한다.
                entry['dup'] = 0

            # 확정 정적 + 충분히 본 트랙은 **누적 평균 중심**을 발행한다
            # (선언부 주석 참조). 매 프레임 흔들리는 순간 중심 대신 평균을
            # 쓰므로 하류 충돌검사 판정이 프레임마다 뒤집히지 않는다.
            use_mean = raw_static and n_obs >= self.static_mean_min_obs
            s_pub, d_pub = (mean_s, mean_d) if use_mean else (s, d)

            obs = Obstacle()
            obs.id = int(track_id)
            obs.position.x, obs.position.y = float(x), float(y)
            obs.s_center = s_pub
            obs.d_center = d_pub
            obs.s_start = (s_pub - half) % self.track_length
            obs.s_end = (s_pub + half) % self.track_length
            obs.d_left = d_pub + half
            obs.d_right = d_pub - half
            obs.vs = vs
            obs.vd = vd
            confidence = max(float(src.confidence), 1e-3)
            # 팽창을 conf_inflation_max 로 자른다 (선언부 주석 참조) — 저신뢰
            # 검출이 더 **큰** 장애물이 되어 회랑을 막던 역전을 없앤다.
            inflation = min(half * (1.0 - confidence), self.conf_inflation_max)
            pos_var = max(self.position_var_floor, inflation ** 2)
            # 속도 미확정 프레임의 분산으로 max_abs_v(=클립 한계)의 제곱을
            # 쓰면 안 된다. 그건 유한차분 추정치를 자르는 위생 한계지 물체가
            # 실제로 낼 수 있는 속도가 아니다. vs_var=100 / vd_var=16이 되어
            # 하류 spliner의 3σ footprint가 종 9.0m / 횡 3.6m로 부풀고,
            # 폭 1.1~2.1m 트랙에서는 좌우 어느 쪽도 통과 불가로 판정돼 매번
            # TRAILING(정지)으로 떨어졌다 (rosbag_0806 4런 68회 = 장애물
            # 획득 1회당 1회, 요구 간격이 전부 3.76~3.88m로 동일).
            # 대신 detector KF 속도의 실측 노이즈를 prior로 쓴다 — 정지
            # 물체에 대해 |v| p50 0.06~0.11 / p90 0.24~0.48 / max 1.20 m/s
            # (rosbag_0806 4런). init_vel_std 0.5는 그 p90 위쪽이다.
            init_vel_var = max(self.velocity_var_floor, self.init_vel_std ** 2)
            vel_var = self.velocity_var_floor if velocity_valid else init_vel_var
            if use_mean:
                # 평균을 발행하므로 보고하는 분산도 **평균의 분산**이어야
                # 한다: s^2/n. 위 상수 하한(0.01 / 0.25)은 단일 프레임 지터
                # 기준이라 n 이 아무리 커져도 안 줄어들고, 그 탓에 하류 u 가
                # 늘 캡 0.12 에 붙어 우측 회피가 매 랩 막혔다.
                # 표본분산이 0 에 수렴해도 정합/자세 오차가 남으므로 하한을 둔다.
                var_of_mean = (m2_d / (n_obs - 1)) / n_obs if n_obs > 1 else 0.0
                pos_var = max(self.static_mean_pos_var_floor, var_of_mean)
                # 정지가 확정된 물체의 속도 불확실성은 0.5 m/s 가 아니다.
                vel_var = min(vel_var, self.static_mean_vel_var_floor)
            obs.s_var = pos_var
            obs.d_var = pos_var
            obs.vs_var = vel_var
            obs.vd_var = vel_var
            obs.size = 2.0 * half
            obs.is_static = not src.is_dynamic  # 초기값 — spliner cls_*가 재분류
            if unsettled_class and not obs.is_static:
                # 위 주석 참조: 분류 미확정 트랙은 정적으로 눌러 통과시킨다.
                # 삭제(옛 동작)와 달리 장애물은 그대로 남고, 취급만 보수적이다.
                obs.is_static = True
                self.get_logger().info(
                    f'분류 미확정(conf={src.confidence:.2f} < '
                    f'{self.min_confidence:.2f}, id={src.id}) — 정적으로 취급',
                    throttle_duration_sec=2.0)
            # detector 의 실제 관측 여부를 그대로 전달한다. 예전에는 True 로
            # 하드코딩돼 있어 coasting 외삽이 측정과 구분되지 않았다.
            obs.is_visible = bool(src.is_visible)
            out.obstacles.append(obs)

            live_entries.append((s, d, bool(src.is_dynamic)))
            if self.memory_enabled and self.ego_s is not None:
                self.memory_observe(
                    s, d, half, self._ds_signed(s, self.ego_s),
                    bool(src.is_dynamic), measured)

        # 랩 기억 주입 — 실측 뒤에 붙인다 (라이브가 항상 우선).
        self.memory_inject(out, live_entries, arrival_sec)

        # 오래 안 보인 트랙 정리
        stale = [i for i, t in self.tracks.items()
                 if i not in seen_ids and
                 arrival_sec - t.get('arrival_t', arrival_sec) > self.forget_sec]
        for i in stale:
            del self.tracks[i]
        if stale and self.reassoc:
            # 사라진 트랙을 가리키던 매핑은 같이 버린다. 남겨 두면 나중에
            # 같은 detector id 가 다른 물체로 재사용될 때 엉뚱한 트랙에 붙는다.
            gone = set(stale)
            for k in [k for k, v in self.reassoc.items() if v['to'] in gone]:
                del self.reassoc[k]

        # 빈 배열도 그대로 전달 (spliner obs_timeout과 연동)
        self.last_source_ns = source_ns
        self.pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = PerceptionObstacleBridgeNode()
    # rclpy.spin(node) 금지 — TransformListener 전용 executor에서 노드를 빼앗아
    # TF 콜백을 메인 스레드에 직렬화시킨다 (생성자 주석 참조).
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        # launch가 SIGINT를 보내면 컨텍스트가 이미 shutdown된 상태로 spin이 깨진다.
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
