#!/usr/bin/env python3
"""build_map.py — 맵 하나로 레이스라인 CSV를 만드는 올인원 스크립트.

step1(트랙 추출) + step2(IQP 라인·속도) + step3(시뮬) + step4(저장)를
'한 파일'로 합쳤다. 실행할 때 **맵 이름만** 넘기면 된다:

    # venv python으로
    ~/IFAC_ws/.venv/bin/python build_map.py my_map

주행 방향은 **기본이 반시계(CCW)** 다 — 이 스택의 트랙은 CCW로 돈다는 전제.
시계로 돌 맵만 `--dir cw` 를 준다. 생성된 CSV 첫 줄 헤더에 dir=CW/CCW 가 남으므로
나중에 파일만 보고도 방향을 확인할 수 있다.

전제:  <맵이름>.yaml + 거기서 가리키는 image(pgm/png) 한 쌍이 있어야 함.
       맵은 두 곳을 순서대로 찾는다:
         1) ~/IFAC_ws/maps      — mapping.launch.py 저장 위치 (복사 없이 바로 사용)
         2) planning_pkg/maps   — 패키지에 함께 둔 맵
       확장자는 yaml 의 image: 필드를 따르므로 pgm/png 아무거나 된다.

모든 튜닝 파라미터는 바로 아래 '설정' 블록에 모아뒀다. 트랙 크기(스케일)는
SLAM이 만든 yaml의 resolution/origin을 그대로 쓴다. EXPECTED_SIZE_M 은
'보통 이 정도(~20x8m) 나온다'는 참고값이라, 추출 결과가 이 범위를 크게
벗어나면 경고만 띄운다. yaml resolution이 틀린 게 확실하면
RESOLUTION_OVERRIDE 로만 덮어쓴다.

출력:
  outputs/<맵>_trajectory.png / _tracking.png          (눈으로 확인)
  config/global_trajectory/<맵>_iqp_*.csv, _sp_*.csv   (런타임/튜너가 읽음)
  config/centerline/<맵>_centerline.csv, _walls.csv
"""

import argparse
import hashlib
import io
import json
import os
import tempfile
import uuid
from pathlib import Path

import numpy as np
import yaml
from PIL import Image
from scipy import ndimage
from scipy.interpolate import splprep, splev
from scipy.optimize import lsq_linear
from scipy.spatial import cKDTree
from skimage.morphology import skeletonize

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

# ============================================================ 설정 (여기만 고치면 됨)
# --- 스케일 ------------------------------------------------------------------
# 크기(resolution/origin)는 SLAM이 만든 yaml 값을 그대로 쓴다. 아래 값은
# '보통 이 정도로 나온다'는 참고용 기대 범위일 뿐 — 추출된 트랙이 이 범위를
# 크게 벗어나면 경고만 띄운다(yaml resolution이 틀렸을 수 있다는 신호).
EXPECTED_SIZE_M = (20.0, 8.0)   # 대략 20 x 8 m 정도 트랙이라는 참고값
# yaml resolution이 틀린 게 확실할 때만 숫자로 덮어써라(예: 0.05). None=yaml 사용.
RESOLUTION_OVERRIDE = None

# --- 맵 자동 정리 ------------------------------------------------------------
# infield(트랙 안쪽 섬)의 오목한 주머니를 이만큼 메운다[m]. SLAM 트랙이 헤어핀/
# 갈고리 모양 infield를 가지면 중앙선이 그 오목한 곳으로 파고들어 자기교차(꼬임)
# 하는데, 이걸 미리 메워 깨끗한 단일 루프로 만든다. closing(팽창→침식)이라
# 코리도 폭은 안 줄고 오목한 곳만 채운다. 0이면 정리 안 함.
INFIELD_FILL_M = 0.8
# infield 안의 이보다 작은 구멍(SLAM 스페클: 몇 px짜리 free 웅덩이)은 벽으로 메운다[m^2].
# medial-axis 골격이 스페클에서 가지를 치는 걸 막는다.
MIN_ISLAND_M2 = 0.3

# --- 레이스라인 생성 파라미터 ------------------------------------------------
V_MAX      = 6.0     # 최대 속도 [m/s]  (소형 트랙 → 보수적. 잘 돌면 올려라)
MU         = 0.75    # 물리 노면 마찰계수. 폐루프 검증의 타이어 한계(mu*g)로 쓴다.
                     # 2026-07-31 run_0731_223152 IMU 실측 |a_lat| p95 7.0~7.9 m/s²
                     # → 0.75(7.36)는 실측과 일치. 실측이 바뀔 때만 수정할 것.
# 속도 프로파일 계획용 mu. MU(물리 한계)의 ~87%만 쓰도록 낮춰서, 컨트롤러의
# 추종 보정(조향 수정 + 제동 겹침)이 남은 마진 안에서 소화되게 한다.
# 2026-07-31 폐루프 스윕: 프로파일 mu 0.75(마진 0)는 scale 0.71부터 friction
# 초과로 FAIL, 0.65는 scale 0.83/cap 5.0에서 합성 7.22/7.36로 PASS.
MU_PROFILE = 0.65
# 2026-07-24: 0.25 -> 0.40 (4 m/s 주행 대비).
#   0.25로 만든 라인은 실측 최소 벽여유 0.20 m → 차 반폭 0.15를 빼면 차체-벽
#   0.05 m뿐. 저속(1 m/s)에선 통과했지만 폐루프 시뮬 |lat err|가 4 m/s에서
#   0.17 m라 그대로면 접촉한다.
# 2026-07-27: 올려도 소용없다는 걸 실측으로 확인 → 0.40 유지.
#   "마진을 넉넉히" 주려고 0.55까지 올려봤는데, 이 값을 키우면 좁은 구간에서
#   라인이 코리도 중앙으로 밀려 헤어핀 반경이 같이 줄어든다. my_track0727 실측:
#     SAFE_DIST  벽여유min  R_min   판정
#       0.40      0.350     1.01    PASS   <- 최적
#       0.45      0.350     0.91    PASS (R 여유 3 cm뿐)
#       0.50      0.425     0.76    FAIL (조향 포화)
#       0.55      0.425     0.65    FAIL
#   0.40->0.45 는 벽여유가 그대로면서 R 여유만 깎이고, 0.50부터는 못 돈다.
#   즉 이 트랙의 구속 조건은 벽 여유가 아니라 **실차 조향 한계**(R>=0.88 m)다.
#   벽 여유가 더 필요하면 SAFE_DIST 가 아니라 V_MAX(=추적오차)를 낮출 것.
#   참고: 예전 주석의 "0.5 넘으면 IQP 불능"은 이제 해당 없다. 바운드를
#   occupancy 마스크에서 받으면서, 코리도가 2*SAFE_DIST 보다 좁은 구간은
#   불능이 아니라 '정중앙 고정'으로 처리된다(_solve_qp 의 narrow 분기).
#   불능 대신 곡률로 대가를 치르는 형태로 바뀐 것이라 위 표가 나온다.
SAFE_DIST  = 0.40    # IQP 라인-벽 최소 이격 [m] = 차 반폭(≈0.15) + 추적오차 여유
# 소형 트랙(my_track: 실폭 median 1.2 m, 최대 2.15 m)은 벽 여유를 크게 빼면
# 회피용 가용 corridor가 0으로 붕괴한다. MARGIN은 주행선을 안 옮기고 CSV의
# d_left/d_right 보고값만 줄이므로(주행 위험 0) 0.15→0.08로 낮춰 회피 여유 확보.
MARGIN     = 0.08    # CSV의 d_left/d_right에서 추가로 빼는 벽 여유 [m]
RACELINE   = "iqp"   # "iqp"(최소곡률) | "sp"(최단거리) | "center"(중앙선)
MAKE_SP    = True    # iqp일 때 SP 추월 라인도 함께 생성
# SP(최단거리) 목적함수에 섞는 곡률 페널티 가중치.
# 순수 최단거리는 코너를 다각형처럼 깎아 R이 0.4~0.8 m까지 떨어진다. 실차
# 최소 회전반경은 wheelbase/tan(max_steer)=0.88 m라 그런 라인은 핸들을 끝까지
# 꺾어도 못 따라간다 — 지금까지 만든 SP CSV가 전부(Spielberg 0.53, my_track
# 0.79, my_track4 0.59) 이 한계를 넘고 있었는데 아무도 안 봤다.
# 0이면 예전(순수 최단거리) 동작. 올릴수록 부드럽고 길어진다.
# my_track fresh generation 실측: w=4 -> R_min 0.834 m(FAIL),
# w=12 -> R_min 1.020 m(PASS, 실차 한계 0.878 m).
SP_CURV_W  = 12.0
# ⚠️ RACELINE="center"(원시 중앙선)는 이 트랙 규모에선 쓰지 말 것.
# medial-axis 중앙선은 헤어핀에서 반경이 레이스라인보다 작아져 R이 0.3~0.4 m
# 까지 떨어진다 (my_track 0.39 / my_track0727 0.31, 실차 한계 0.88 m).
# 검증에서 걸러지긴 하지만 애초에 주행 가능한 라인이 안 나온다.
# 벽에서 넉넉히 떨어져 달리고 싶으면 center 가 아니라 SAFE_DIST 를 올릴 것.

# --- 최종 검증 (통과 못 하면 CSV를 안 쓰고 exit 1) ---------------------------
# 2026-07-27: my_track0727에서 벽을 관통하는 IQP 라인이 그대로 CSV로 나가
# 고속주행 테스트에 쓰인 사고가 있었다. 원인은 골격 정렬 실패 -> 중앙선이 벽을
# 가로지름 -> 그 점에선 d_left=d_right=0 이라 QP가 "이미 정중앙"으로 보고 방치.
# 어느 단계도 결과를 실제 occupancy 마스크에 대고 확인하지 않았다.
# 이제 마지막에 반드시 검사하고, 실패하면 CSV를 만들지 않는다.
CAR_HALF_WIDTH = 0.15    # 실차 전폭 0.30 m의 절반. 라인-벽 여유가 이 미만이면 접촉
CAR_HALF_LENGTH = 0.275  # 실차 전장 약 0.55 m의 절반

# --- 차량/컨트롤러 파라미터 (실차 실측값, 트랙 크기와 무관) ------------------
# ⚠️ 이 두 값은 출력 CSV(라인 형상·속도 프로파일)에 영향이 없다. 레이스라인은
#    optimize_raceline_iqp(기하 QP) + build_trajectory(곡률·mu)만으로 정해진다.
#    아래 두 값은 PurePursuitSim 폐루프 검증에만 쓰여 "실차 조향 한계로 이 라인을
#    실제로 따라갈 수 있는가"(lat err)를 판정한다 → 반드시 실차값이어야 의미 있음.
WHEELBASE  = 0.33    # [m] 실차 실측 (hardware vesc.yaml wheelbase와 동일)
MAX_STEER  = 0.36    # [rad] (20.7°) 실차 검증 조향 한계. TUNING_STATUS 기준이며
                     # control_pkg vehicle/max_steering_angle_rad와 동일하게 유지할 것
                     # (순정 링키지 명목 24°=0.4189가 아니라 실측 포화 한계값)
# 종방향 한계 — run_0731_231343 실측 (모터 전류/속도 응답):
#   가속: 모터 전류 ~30 A 포화 → 최대 2.2 m/s² (p90 실측 2.23). 예전 5.5는
#     달성 불가라 프로파일 속도에 차가 못 붙고, 직선 끝까지 가속만 하다
#     제동 구간을 놓치는 원인이 됐다.
#   감속: VESC가 브레이크 전류를 전혀 안 씀 (전류 최소 +5.8 A, 음수 없음)
#     → 감속은 코스팅 항력뿐, 실측 평균 -1.5 m/s². 여유 두고 1.3으로 계획.
# ⚠️ VESC 제동(펌웨어 brake 한계 또는 commands/motor/brake 경로)이 생겨
#   실측 감속이 좋아지면 그때 A_DEC_MAX를 실측값으로 올리고 재생성할 것.
A_ACC_MAX  = 2.2     # 최대 가속 [m/s²] (30 A 전류 제한 실측)
A_DEC_MAX  = 1.3     # 최대 제동 [m/s²] (코스팅 전용 실측, 제동 전류 없음)
MAX_STEER_RATE = 2.0 # [rad/s] control 기본 steering-rate limit
N_LOCAL_WP = 130     # 로컬 윈도우 웨이포인트 수 (local_trajectory_node와 동일)

# 폐루프 검증은 이름 없는 "비슷한 pure pursuit" 설정으로 돌리면 안 된다.
# 아래 목록에 선택된 배포 프로파일을 **모두** 통과해야 CSV를 쓴다. 값은 각각
# planning/config와 control/config의 현재 활성 설정을 명시적으로 복제한다.
# 프로파일을 바꿀 때 런타임 YAML과 이 블록을 한 변경으로 맞출 것.
SIM_CONTROLLER_PROFILES = (
    "cpp_real_car",
    "python_sim_tracker",
)
CONTROLLER_PROFILE_LIBRARY = {
    # planning environments/real_car.yaml velocity_scale=0.97 +
    # control environments/real_car.yaml speed_scale=1.0, max_speed=6.0.
    # (2026-08-01: 5 m/s 단계(0.83/5.0) 실차 성공 후 6 m/s 단계로 승격.
    #  프로파일 자체가 A_ACC 2.2/A_DEC 1.3 실측 한계로 v-max ~5.4라
    #  cap 6.0은 클램프 없이 프로파일 속도를 그대로 통과시킨다.
    #  scale 스윕(my_track0801): 1.00 FAIL(footprint 벽 침범), 0.97~0.99 PASS.
    #  0.97은 |lat err|max 0.202 m로 5 m/s 성공 배포(0.207)와 같은 마진 —
    #  0.99(0.212 m)는 절벽(1.0) 한 칸 옆이라 채택하지 않음.)
    # C++ MAP/L1의 실제 NUC6 steering LUT도 그대로 사용한다.
    "cpp_real_car": {
        "kind": "cpp_map_l1",
        "planner_speed_scale": 0.97,
        "controller_speed_scale": 1.0,
        "max_speed_mps": 6.0,
        "lookahead_gain": 0.47,
        "lookahead_offset": -0.2,
        "lookahead_min_m": 0.5,
        "lookahead_max_m": 8.0,
        # control params.yaml map/speed_lookahead와 동일 유지.
        # 2026-07-31 스윕: 0.35/0.45/0.6은 직선 투영이 코너 건너편 웨이포인트를
        # 잡아 전부 악화(|derr| 0.30/0.38/0.51). 이 트랙 크기에선 0.25가 최적.
        # 제동 여유는 speed_lookahead가 아니라 MU_PROFILE(프로파일 마진)로 확보.
        "speed_lookahead_s": 0.25,
        "wheelbase_m": WHEELBASE,
        "max_steer_rad": MAX_STEER,
        "max_steer_rate_radps": MAX_STEER_RATE,
        "max_accel_mps2": A_ACC_MAX,
        "max_decel_mps2": A_DEC_MAX,
        "control_rate_hz": 40.0,
        "num_local_waypoints": N_LOCAL_WP,
        "steering_lut": (
            "control_ws/src/control_pkg/config/steering_lookup/"
            "NUC6_glc_pacejka_lookup_table.csv"),
    },
    # planner params velocity_scale=0.6667 + waypoint_tracker speed_scale=0.30.
    # Python tracker의 lookahead=clip(0.5*v, 0.6, 1.2)를 그대로 사용한다.
    "python_sim_tracker": {
        "kind": "python_pure_pursuit",
        "planner_speed_scale": 0.6667,
        "controller_speed_scale": 0.30,
        "max_speed_mps": V_MAX,
        "lookahead_gain": 0.5,
        "lookahead_offset": 0.0,
        "lookahead_min_m": 0.6,
        "lookahead_max_m": 1.2,
        "wheelbase_m": WHEELBASE,
        "max_steer_rad": MAX_STEER,
        "max_steer_rate_radps": MAX_STEER_RATE,
        "max_accel_mps2": A_ACC_MAX,
        "max_decel_mps2": A_DEC_MAX,
        "control_rate_hz": 40.0,
        "num_local_waypoints": N_LOCAL_WP,
        "steering_lut": None,
    },
}

# --- 이산화 간격 -------------------------------------------------------------
DS_OUT     = 0.1     # 출력 웨이포인트 간격 [m]
LAPS       = 1       # 시뮬 랩 수
# ============================================================================

G = 9.81

HERE = Path(__file__).resolve().parent
PKG_ROOT = next((p for p in HERE.parents if p.name == "planning_pkg"),
                HERE.parent.parent)
# 맵을 찾을 폴더 (앞에서부터 순서대로 — 먼저 있는 걸 쓴다):
#   1) <workspace>/maps      — mapping.launch.py 가 저장하는 워크스페이스 공용 폴더.
#      여기서 먼저 찾으므로 매핑 직후 복사 없이 바로 build_map 을 돌릴 수 있다.
#   2) planning_pkg/maps     — 패키지에 함께 두는 맵 (기존 동작).
WORKSPACE_ROOT = PKG_ROOT.parents[2]
MAPS_SEARCH_DIRS = [WORKSPACE_ROOT / "maps", PKG_ROOT / "maps"]
CSV_DIR = PKG_ROOT / "config" / "global_trajectory"
CENTERLINE_DIR = PKG_ROOT / "config" / "centerline"
OUT_DIR = HERE / "outputs"


# ============================================================ 1. 트랙 추출
def _contour_paths(cs):
    """ContourSet에서 Path 목록을 꺼낸다.

    matplotlib 3.8+ 는 ContourSet.get_paths() 를 주지만, 그 이전 버전은
    레벨별 LineCollection(`cs.collections`)에 Path가 들어 있다.
    """
    if hasattr(cs, "get_paths"):
        return cs.get_paths()
    return [p for coll in cs.collections for p in coll.get_paths()]


def _polyline_length(p):
    return np.sum(np.hypot(*np.diff(p, axis=0).T))


def _signed_area(p):
    x, y = p[:, 0], p[:, 1]
    return 0.5 * np.sum(x * np.roll(y, -1) - np.roll(x, -1) * y)


def _resample_closed(pts, n):
    d = np.hypot(*np.diff(pts, axis=0).T)
    s = np.concatenate([[0.0], np.cumsum(d)])
    si = np.linspace(0.0, s[-1], n, endpoint=False)
    x = np.interp(si, s, pts[:, 0])
    y = np.interp(si, s, pts[:, 1])
    return np.column_stack([x, y])


def _px_to_world(rc, resolution, origin, H):
    """rc: (N,2) (row, col) 픽셀 → 월드 (x, y). load_track 안의 것과 동일 규약.

    map_server(nav2 map_io) 셀 중심 규약과 맞춘다: yaml origin은 이미지 좌하단
    코너의 pose이고, 이미지 row는 위에서부터라 grid row = H-1-row.
        [x, y] = origin_xy + R(origin_yaw) *
                 [(col + 0.5) * res, (H - row - 0.5) * res]
    0.5 셀을 빼먹으면 트랙 모델 전체가 PF가 정합하는 occupancy grid 대비
    (-res/2, +res/2) 만큼 대각선으로 밀린다(0.05 m/px에서 3.5 cm).
    """
    rc = np.atleast_2d(rc)
    local_x = (rc[:, 1] + 0.5) * resolution
    local_y = (H - rc[:, 0] - 0.5) * resolution
    yaw = float(origin[2])
    c, s = np.cos(yaw), np.sin(yaw)
    x = origin[0] + c * local_x - s * local_y
    y = origin[1] + s * local_x + c * local_y
    return np.column_stack([x, y])


class Grid:
    """트랙 occupancy 마스크 + 월드↔픽셀 변환. **폭·침범 판정의 단일 기준**.

    예전에는 폭을 경계 폴리라인(outer/inner)에 대한 최근접 거리로만 쟀다. 그
    값 자체는 대체로 맞지만, 라인이 벽 '안'으로 들어가 버린 경우 d_left 와
    d_right 가 둘 다 0 이 되어 "폭 0인 코리도 정중앙"과 구분이 안 됐다.
    그래서 QP가 위반을 못 알아보고 그대로 뒀다(my_track0727 사고).

    마스크에 법선을 쏘면 '그 점을 담고 있는 free 구간'을 부호 있는 alpha
    범위로 얻을 수 있다. 점이 벽 안이면 두 끝이 같은 부호로 나와서 QP가
    코리도 쪽으로 되끌어낸다 — 즉 최적화가 스스로 복구한다.
    """

    def __init__(self, ring, resolution, origin, H, skeleton_ring=None):
        # ring은 원본 occupancy에서 추출한 안전 기준이며 절대 정리하지 않는다.
        # skeleton_ring만 골격 안정화를 위해 작은 섬/오목부를 정리할 수 있다.
        self.ring = np.asarray(ring, dtype=bool)
        self.skeleton_ring = (
            self.ring if skeleton_ring is None
            else np.asarray(skeleton_ring, dtype=bool))
        if self.skeleton_ring.shape != self.ring.shape:
            raise ValueError("raw/skeleton occupancy masks have different shapes")
        self.res = resolution
        self.origin = np.asarray(origin, dtype=float)
        if self.origin.shape != (3,) or not np.isfinite(self.origin).all():
            raise ValueError("grid origin must be finite [x, y, yaw]")
        if not np.isfinite(self.res) or self.res <= 0.0:
            raise ValueError("grid resolution must be positive and finite")
        self.H = H

    def is_free(self, xy):
        """월드 좌표 (N,2) → 그 셀이 주행 가능한가 (bool N). _px_to_world의 역."""
        xy = np.atleast_2d(xy)
        dx = xy[:, 0] - self.origin[0]
        dy = xy[:, 1] - self.origin[1]
        yaw = float(self.origin[2])
        c, s = np.cos(yaw), np.sin(yaw)
        local_x = c * dx + s * dy
        local_y = -s * dx + c * dy
        col = local_x / self.res - 0.5
        row = self.H - 0.5 - local_y / self.res
        ri = np.round(row).astype(int)
        ci = np.round(col).astype(int)
        ok = ((ri >= 0) & (ri < self.ring.shape[0])
              & (ci >= 0) & (ci < self.ring.shape[1]))
        out = np.zeros(len(xy), bool)
        out[ok] = self.ring[ri[ok], ci[ok]]
        return out

    def corridor_interval(self, xy, psi, max_d=4.0, step=None):
        """각 점에서 법선을 따라 스캔 → free 구간을 (a_lo, a_hi) [m] 로.

        a_lo < 0 < a_hi 면 점이 코리도 안에 있고, 각각 우/좌 벽까지의 거리다.
        점이 벽 안이면 가장 가까운 free 구간을 돌려주므로 a_lo, a_hi 가 같은
        부호가 된다 → 그 방향으로 이동해야 한다는 뜻.

        샘플 간격만큼(기본 res/2 = 2.5 cm) 항상 보수적으로(좁게) 나온다.
        """
        step = step if step is not None else 0.5 * self.res
        m = int(round(max_d / step))
        alphas = np.arange(-m, m + 1) * step          # alpha=0 은 인덱스 m
        nrm = np.column_stack([-np.sin(psi), np.cos(psi)])
        free = np.empty((len(xy), len(alphas)), bool)
        for j, a in enumerate(alphas):
            free[:, j] = self.is_free(xy + a * nrm)

        a_lo = np.zeros(len(xy))
        a_hi = np.zeros(len(xy))
        for i in range(len(xy)):
            idx = np.flatnonzero(free[i])
            if len(idx) == 0:            # 법선 전체가 막힘 (있으면 맵이 이상함)
                continue                 # a_lo = a_hi = 0 → 그 점은 이동 금지
            brk = np.flatnonzero(np.diff(idx) > 1)
            starts = np.r_[idx[0], idx[brk + 1]]
            ends = np.r_[idx[brk], idx[-1]]
            hit = np.flatnonzero((starts <= m) & (m <= ends))
            k = (hit[0] if len(hit)
                 else int(np.argmin(np.minimum(np.abs(starts - m),
                                               np.abs(ends - m)))))
            a_lo[i] = alphas[starts[k]]
            a_hi[i] = alphas[ends[k]]
        return a_lo, a_hi

    def clearance(self, xy, psi, **kw):
        """법선 좌우 벽까지 거리 중 작은 쪽 [m]. 벽 안이면 음수."""
        a_lo, a_hi = self.corridor_interval(xy, psi, **kw)
        inside = (a_lo <= 0.0) & (0.0 <= a_hi)
        near = np.minimum(np.abs(a_lo), np.abs(a_hi))
        return np.where(inside, near, -near)


def _medial_axis_loop(ring):
    """트랙(free 링) 마스크의 medial-axis를 단일 폐루프 마스크로.

    안·바깥 경계 중점(midpoint) 방식은 인필드가 아웃필드 벽까지 뻗는 '게이트'에서
    중점이 한쪽 벽으로 붕괴해 곡률이 튄다(R~0.3m 스파이크). medial-axis는 항상
    코리도 한가운데를 지나 게이트에서도 안 튄다.

    스퍼(핑거/오목한 곳에서 생기는 막다른 가지)는 끝점(8-이웃 <=1개)을 순수
    루프만 남을 때까지 반복 제거해 없앤다. 순수 사이클은 끝점이 없어 안 깎인다.
    """
    sk = skeletonize(ring)
    k3 = np.ones((3, 3), np.uint8)
    while True:
        nb = ndimage.convolve(sk.astype(np.uint8), k3, mode="constant") - sk
        ends = sk & (nb <= 1)
        if not ends.any():
            break
        sk = sk & ~ends
    if not sk.any():
        raise RuntimeError("medial-axis 폐루프를 못 찾음 "
                           "(트랙이 안 닫혔거나 코리도가 너무 얇음)")
    lbl, nl = ndimage.label(sk, structure=k3)
    if nl > 1:                              # 남은 조각 중 가장 큰 루프만
        sizes = ndimage.sum(sk, lbl, range(1, nl + 1))
        sk = lbl == (1 + int(np.argmax(sizes)))
    return sk


def _order_loop(sk):
    """폐루프 골격 마스크의 픽셀을 이웃끼리 이어진 순서 (row,col) 로 정렬.

    ⚠️ 예전 구현은 '가장 가까운 미사용 픽셀'만 보고 한 방향으로 걸었다.
    skeletonize 결과에는 계단(staircase) 때문에 8-이웃이 3개인 픽셀이 몇 개
    생기는데(my_track0727: 656 px 중 7개), 거기서 1 px짜리 곁가지로 새면 곧
    막히고 `break` 로 걸음이 통째로 끝난다. 실제로 71 px(10.8%)를 버렸고,
    그 자리에서 중앙선이 벽을 가로질러 결국 레이스라인이 벽을 관통했다.
    경고만 찍고 통과시킨 게 사고를 키웠다.

    그래서 세 가지를 바꿨다:
      1. 진행 방향과 가장 잘 이어지는 이웃을 고른다 (계단 곁가지 회피).
      2. 한쪽이 막히면 시작점에서 반대 방향으로도 걸어 앞에 붙인다.
      3. 그래도 픽셀이 남거나 루프가 안 닫히면 **죽는다**. 중앙선이 깨진 채로
         진행하면 뒤 단계가 조용히 잘못된 라인을 만든다.
    """
    ys, xs = np.nonzero(sk)
    pts = np.column_stack([ys, xs]).astype(float)
    n = len(pts)
    if n < 8:
        raise RuntimeError(f"medial-axis 골격이 너무 짧다 ({n} px)")
    tree = cKDTree(pts)
    used = np.zeros(n, bool)
    used[0] = True

    def walk(start):
        """start 에서 한쪽 방향으로 끝까지. start 포함 인덱스 리스트."""
        order = [start]
        cur = start
        direction = None
        while True:
            best, best_score = -1, -np.inf
            for j in tree.query_ball_point(pts[cur], r=1.9):
                if used[j]:
                    continue
                step = pts[j] - pts[cur]
                dist = float(np.hypot(*step))
                if dist < 1e-9:
                    continue
                # 직진성 우선, 동점이면 가까운 쪽. 계단에서 곁가지로 새는 걸 막는다.
                score = (float(step @ direction) / dist if direction is not None
                         else 0.0) - 0.1 * dist
                if score > best_score:
                    best, best_score = j, score
            if best < 0:
                return order
            direction = (pts[best] - pts[cur]) / np.hypot(*(pts[best] - pts[cur]))
            used[best] = True
            order.append(best)
            cur = best

    fwd = walk(0)
    bwd = walk(0)                       # 0 의 반대쪽 이웃부터 (앞쪽은 이미 used)
    order = bwd[:0:-1] + fwd            # 뒤쪽을 뒤집어 앞에 붙이고 0 은 한 번만

    # 남는 픽셀이 몇 개 생기는 건 정상이다 — skeletonize 의 계단에서 대각으로
    # 겹치는 '두께' 픽셀이라 건너뛰어도 경로가 끊기지 않는다(루프에 인접함).
    # 반면 통째로 놓친 구간은 루프에서 멀리 떨어져 나온다. 그 둘을 개수가 아니라
    # **루프까지의 거리**로 가른다. 예전처럼 개수 비율(20%)로 봐주면 실제로
    # 놓친 구간(71 px)을 경고만 하고 통과시켜 벽을 가로지르는 중앙선이 나온다.
    left = np.flatnonzero(~used)
    if len(left):
        far = left[cKDTree(pts[order]).query(pts[left])[0] > 1.9]
        if len(far):
            raise RuntimeError(
                f"medial-axis 루프 정렬 실패: {len(far)}/{n} px 가 루프에서 "
                "떨어져 남았다 (골격이 갈라졌거나 끊김). 그대로 두면 중앙선이 "
                "벽을 가로지른다.\n"
                "  → 맵을 다시 따거나 INFIELD_FILL_M / MIN_ISLAND_M2 를 올려 볼 것.")
    # 닫힘 허용치는 한 걸음(1.9 px)이 아니라 3 px(0.15 m)로 둔다. 계단 구간에서
    # 시작/끝 사이에 두께 픽셀 한 개가 낀 채 끝나는 게 정상이고(my_map: 2.0 px),
    # 그 정도 틈은 뒤의 주기 스플라인이 매끄럽게 닫는다. 진짜로 끊긴 트랙은
    # 수십 px 벌어지므로 이 값으로도 충분히 걸러진다.
    gap = float(np.hypot(*(pts[order[-1]] - pts[order[0]])))
    if gap > 3.0:
        raise RuntimeError(
            f"medial-axis 루프가 안 닫힘 (끝-시작 간격 {gap:.1f} px). "
            "트랙이 닫힌 고리가 아니거나 골격이 끊겼다.")
    return pts[order]


def _find_map_yaml(map_name):
    """<맵>.yaml 을 MAPS_SEARCH_DIRS 순서로 찾아 경로를 돌려준다.

    image(pgm/png) 는 yaml 의 image: 필드를 기준으로 같은 폴더에서 연다
    (load_track 참조) — 확장자는 상관없다. 어디에도 없으면 찾아본 곳을
    같이 알려주고 죽는다: 맵 이름 오타/미매핑을 launch 전에 잡기 위함.
    """
    for directory in MAPS_SEARCH_DIRS:
        candidate = directory / f"{map_name}.yaml"
        if candidate.is_file():
            return candidate
    searched = "\n".join(f"    {d}" for d in MAPS_SEARCH_DIRS)
    raise FileNotFoundError(
        f"맵 '{map_name}.yaml' 을 찾을 수 없습니다. 찾아본 곳:\n{searched}\n"
        f"  (매핑을 먼저 하세요: ros2 launch integration_pkg mapping.launch.py "
        f"map_name:={map_name})")


def load_track(map_name):
    """맵 yaml/png 로드 → free 마스크 + 링(트랙) 검출 → 월드 좌표 경계.

    크기(resolution/origin)는 SLAM이 만든 yaml 값을 그대로 사용한다.
    추출된 트랙 크기가 EXPECTED_SIZE_M 참고범위를 크게 벗어나면 경고만 낸다.
    반환: 경계, resolution, [x,y,yaw], raw/skeleton 마스크, 이미지 높이.
    """
    yaml_path = _find_map_yaml(map_name)
    with open(yaml_path) as f:
        meta = yaml.safe_load(f)
    img = np.array(Image.open(yaml_path.parent / meta["image"]).convert("L"),
                   dtype=float)
    if meta.get("negate", 0):
        img = 255.0 - img
    occ = (255.0 - img) / 255.0            # ROS map_server 관례
    free_mask = occ < float(meta.get("free_thresh", 0.196))
    H = img.shape[0]
    resolution = (float(RESOLUTION_OVERRIDE) if RESOLUTION_OVERRIDE is not None
                  else float(meta["resolution"]))
    origin_values = meta.get("origin")
    if not isinstance(origin_values, (list, tuple)) or len(origin_values) < 3:
        raise ValueError("map YAML origin must contain [x, y, yaw]")
    origin = np.array(origin_values[:3], dtype=float)
    if not np.isfinite(origin).all():
        raise ValueError("map YAML origin contains NaN/Inf")

    # --- 링(가운데 구멍 뚫린 free 영역 = 트랙) 검출 ---------------------------
    labels, n = ndimage.label(free_mask)
    sizes = ndimage.sum(free_mask, labels, range(1, n + 1))
    ring = None
    for lab in (1 + np.argsort(sizes)[::-1]):
        comp = labels == lab
        touches_border = (comp[0, :].any() or comp[-1, :].any()
                          or comp[:, 0].any() or comp[:, -1].any())
        if touches_border:                 # 맵 바깥 배경 → 버림
            continue
        filled = ndimage.binary_fill_holes(comp)
        if filled.sum() - comp.sum() > comp.sum() * 0.05:
            ring = comp
            break
    if ring is None:
        raise RuntimeError("닫힌 고리(트랙) 모양의 free 영역을 못 찾음. "
                           "실내 corridor 맵이거나 트랙이 안 닫힘.")
    # 원본 occupancy component는 corridor/footprint의 불변 안전 기준이다.
    # 아래 정리는 medial-axis 추출용 사본에만 적용한다. 실제 작은 기둥/장애물을
    # 지운 마스크로 최종 검증하는 오류를 막는다.
    raw_ring = ring.copy()
    skeleton_ring = ring.copy()

    # --- 자동 정리: infield 오목한 주머니 메우기 (중앙선 자기교차 방지) -------
    if INFIELD_FILL_M > 0:
        close_it = max(1, int(round(INFIELD_FILL_M / resolution)))
        filled = ndimage.binary_fill_holes(skeleton_ring)
        infield = filled & ~skeleton_ring
        infield_c = ndimage.binary_closing(
            infield, structure=np.ones((3, 3)), iterations=close_it)
        infield_c = ndimage.binary_fill_holes(infield_c) & filled
        skeleton_ring = filled & ~infield_c
        filled_new_holes = int((infield_c & ~infield).sum())
        if filled_new_holes > 0:
            print(f"      infield 정리: 오목한 곳 {filled_new_holes} px "
                  f"메움 (close {close_it} px = {INFIELD_FILL_M:.1f} m)")

    # --- SLAM 스페클 제거: infield 안의 아주 작은 구멍은 벽으로 메운다 ----------
    # (몇 px짜리 free 웅덩이가 medial-axis 골격에 가짜 가지를 만든다)
    if MIN_ISLAND_M2 > 0:
        min_px = int(round(MIN_ISLAND_M2 / (resolution * resolution)))
        holes = (
            ndimage.binary_fill_holes(skeleton_ring) & ~skeleton_ring)
        hl, hn = ndimage.label(holes)
        n_speckle = 0
        for lab in range(1, hn + 1):
            m = hl == lab
            if m.sum() < min_px:
                skeleton_ring[m] = True
                n_speckle += 1
        if n_speckle > 0:
            print(f"      스페클 제거: 작은 구멍 {n_speckle}개 메움 "
                  f"(< {MIN_ISLAND_M2:.2f} m^2)")

    # --- 참고용 크기 확인 (스케일은 안 바꿈, 정상 범위인지 경고만) ----------
    # 트랙이 기울어져 있을 수 있으므로 PCA(주축)로 긴 변/짧은 변 실측.
    rows, cols = np.nonzero(raw_ring)
    pts_px = np.column_stack([cols, rows]).astype(float)
    _, _, Vt = np.linalg.svd(pts_px - pts_px.mean(axis=0), full_matrices=False)
    proj = (pts_px - pts_px.mean(axis=0)) @ Vt.T
    major_m = (proj[:, 0].max() - proj[:, 0].min()) * resolution
    minor_m = (proj[:, 1].max() - proj[:, 1].min()) * resolution
    exp_long, exp_short = max(EXPECTED_SIZE_M), min(EXPECTED_SIZE_M)
    print(f"      트랙 크기(yaml resolution {resolution:.4f} m/px): "
          f"{major_m:.1f} x {minor_m:.1f} m  (참고 기대값 ~{exp_long}x{exp_short} m)")
    if not (0.5 * exp_long < major_m < 2.0 * exp_long):
        print(f"      [warn] 트랙 긴 변 {major_m:.1f} m 가 기대범위를 크게 벗어남 "
              f"→ yaml resolution 확인 (RESOLUTION_OVERRIDE로 덮어쓰기 가능)")

    def px_to_world(rc):
        return _px_to_world(rc, resolution, origin, H)

    # --- 링 경계선을 contour로 뽑아 월드 좌표 폴리라인 2개(outer/inner) ------
    fig = plt.figure()
    cs = plt.contour(raw_ring.astype(float), levels=[0.5])
    segs = []
    for path in _contour_paths(cs):
        for poly in path.to_polygons(closed_only=False):
            segs.append(np.asarray(poly))
    plt.close(fig)
    segs = sorted(segs, key=_polyline_length, reverse=True)[:2]
    if len(segs) < 2:
        raise RuntimeError(f"트랙 경계 2개를 기대했는데 {len(segs)}개만 검출됨")
    polys = [px_to_world(np.column_stack([s[:, 1], s[:, 0]])) for s in segs]
    polys.sort(key=lambda p: abs(_signed_area(p)), reverse=True)
    # raw_ring은 모든 안전 판정, skeleton_ring은 중앙선 추출에만 사용한다.
    return (
        polys[0], polys[1], resolution, origin,
        raw_ring, skeleton_ring, H)


def extract_centerline(grid, smooth=3.0, ds=DS_OUT):
    """트랙 마스크의 medial-axis(거리변환 중심선) → 주기 스플라인 → 균일 리샘플.

    예전엔 안·바깥 경계 중점으로 중앙선을 잡았는데, 인필드가 아웃필드 벽까지
    뻗는 '게이트'에서 중점이 한쪽 벽으로 붕괴해 곡률이 튀었다(R~0.3m 스파이크).
    medial-axis는 항상 코리도 한가운데를 지나 게이트에서도 안 튄다.
    폭(d_left/d_right)만 물리 경계(outer/inner)에서 부호 있는 거리로 계산한다.
    """
    loop_rc = _order_loop(
        _medial_axis_loop(grid.skeleton_ring))      # (row, col) 순서
    loop_xy = _px_to_world(loop_rc, grid.res, grid.origin, grid.H)
    # 픽셀 계단 노이즈를 지운 뒤 주기 스플라인
    loop_r = _resample_closed(loop_xy, max(len(loop_xy), 800))
    tck, _ = splprep([loop_r[:, 0], loop_r[:, 1]], s=smooth, per=1)
    u = np.linspace(0, 1, 20000, endpoint=False)
    x, y = splev(u, tck)
    dl = np.hypot(np.diff(x, append=x[0]), np.diff(y, append=y[0]))
    length = dl.sum()
    n_pts = max(int(round(length / ds)), 10)
    s_dense = np.concatenate([[0.0], np.cumsum(dl)])[:-1]
    u_uniform = np.interp(np.linspace(0, length, n_pts, endpoint=False), s_dense, u)

    x, y = splev(u_uniform, tck)
    dx, dy = splev(u_uniform, tck, der=1)
    ddx, ddy = splev(u_uniform, tck, der=2)
    xy = np.column_stack([x, y])
    psi = np.arctan2(dy, dx)
    kappa = (dx * ddy - dy * ddx) / np.power(dx * dx + dy * dy, 1.5)
    s = np.concatenate([[0.0], np.cumsum(np.hypot(np.diff(x), np.diff(y)))])

    # 폭은 레이스라인과 같은 기준(occupancy 마스크에 법선 레이캐스트)으로 잰다.
    a_lo, a_hi = grid.corridor_interval(xy, psi)
    n_out = int(np.sum((a_lo > 0.0) | (a_hi < 0.0)))
    if n_out:
        # 골격 정렬이 성공했다면 여기 걸릴 일이 없다. 걸린다면 스무딩이 좁은
        # 헤어핀의 모서리를 잘라먹은 것 — IQP가 되끌어내지만 알려는 둔다.
        print(f"      [warn] 중앙선 {n_out}/{len(xy)} 점이 코리도 밖 "
              f"(smooth={smooth} 이 헤어핀을 깎았을 수 있음)")
    return {"xy": xy, "psi": psi, "kappa": kappa, "s": s,
            "d_left": np.maximum(a_hi, 0.0), "d_right": np.maximum(-a_lo, 0.0)}


# ============================================================ 2. IQP raceline
def _respline_closed(xy, ds, smooth=0.5):
    tck, _ = splprep([xy[:, 0], xy[:, 1]], s=smooth, per=1)
    u = np.linspace(0, 1, 8 * len(xy), endpoint=False)
    x, y = splev(u, tck)
    dl = np.hypot(np.diff(x, append=x[0]), np.diff(y, append=y[0]))
    length = dl.sum()
    n_pts = max(int(round(length / ds)), 16)
    s_dense = np.concatenate([[0.0], np.cumsum(dl)])[:-1]
    u_uni = np.interp(np.linspace(0, length, n_pts, endpoint=False), s_dense, u)
    x, y = splev(u_uni, tck)
    dx, dy = splev(u_uni, tck, der=1)
    ddx, ddy = splev(u_uni, tck, der=2)
    psi = np.arctan2(dy, dx)
    kappa = (dx * ddy - dy * ddx) / np.power(dx * dx + dy * dy, 1.5)
    s = np.concatenate([[0.0], np.cumsum(np.hypot(np.diff(x), np.diff(y)))])
    return {"s": s, "xy": np.column_stack([x, y]), "psi": psi, "kappa": kappa}


def _solve_qp(ref, safe_dist_m, objective):
    """법선 오프셋 alpha 에 대한 부등식 제약 최소제곱.

    바운드는 마스크에서 받은 free 구간 [a_lo, a_hi] 에 safe_dist 를 물린 것이다.
    a_lo/a_hi 는 부호가 있으므로, 기준점이 벽 안이면 두 끝이 같은 부호로 들어와
    alpha 가 코리도 쪽으로 강제된다 — 위반이 스스로 복구된다.
    코리도가 2*safe_dist 보다 좁으면 정중앙((a_lo+a_hi)/2)에 고정한다.
    """
    n = len(ref["xy"])
    normal = np.column_stack([-np.sin(ref["psi"]), np.cos(ref["psi"])])
    D1 = np.roll(np.eye(n), -1, axis=1) - np.eye(n)          # 구간 길이
    D2 = (np.roll(np.eye(n), -1, axis=1) - 2 * np.eye(n)
          + np.roll(np.eye(n), 1, axis=1))                   # 곡률 근사
    # SP는 순수 최단거리로 풀면 코너가 다각형이 되어 실차 조향 한계를 넘는다.
    # 곡률 항을 sqrt(w) 로 세로로 쌓아 ||D1 x||^2 + w||D2 x||^2 를 최소화한다.
    blocks = ([D2] if objective == "curvature"
              else ([D1] if SP_CURV_W <= 0 else [D1, np.sqrt(SP_CURV_W) * D2]))
    A = np.vstack([np.vstack([D @ np.diag(normal[:, 0]),
                              D @ np.diag(normal[:, 1])]) for D in blocks])
    b = -np.concatenate([np.concatenate([D @ ref["xy"][:, 0],
                                         D @ ref["xy"][:, 1]]) for D in blocks])
    lo = ref["a_lo"] + safe_dist_m
    hi = ref["a_hi"] - safe_dist_m
    # lsq_linear 은 lo < hi 를 '엄격하게' 요구한다. 코리도 폭이 정확히
    # 2*safe_dist 면 lo == hi 가 되어 ValueError 로 죽는다 — 레이캐스트가
    # 0.025 m 격자라 실제로 일어난다(my_track4). 등호도 narrow 로 본다.
    narrow = lo >= hi - 1e-9
    mid = 0.5 * (ref["a_lo"] + ref["a_hi"])
    lo = np.where(narrow, mid - 1e-6, lo)
    hi = np.where(narrow, mid + 1e-6, hi)
    return lsq_linear(A, b, bounds=(lo, hi)).x, normal


def optimize_raceline_iqp(center, grid, safe_dist_m=SAFE_DIST,
                          n_iters=3, ds_opt=0.5, ds_out=DS_OUT,
                          objective="curvature"):
    """중앙선을 법선 방향으로 밀어 최소곡률(IQP) 또는 최단거리(SP) 라인을 만든다.

    폭/바운드는 매 반복 occupancy 마스크에서 다시 잰다(Grid.corridor_interval).
    경계 폴리라인 최근접 거리로는 '벽 안에 들어간 점'을 못 알아봐서 위반이
    영구히 남았다 — 그게 my_track0727 벽 관통의 직접 원인이었다.
    """
    tag = "IQP" if objective == "curvature" else "SP"
    stride = max(int(round(ds_opt / (center["s"][1] - center["s"][0]))), 1)
    ref = {k: center[k][::stride].copy() for k in ("xy", "psi")}
    ref["a_lo"], ref["a_hi"] = grid.corridor_interval(ref["xy"], ref["psi"])
    for it in range(n_iters):
        alpha, normal = _solve_qp(ref, safe_dist_m, objective)
        geo = _respline_closed(ref["xy"] + alpha[:, None] * normal, ds_opt)
        a_lo, a_hi = grid.corridor_interval(geo["xy"], geo["psi"])
        ref = {"xy": geo["xy"], "psi": geo["psi"], "a_lo": a_lo, "a_hi": a_hi}
        n_out = int(np.sum((a_lo > 0.0) | (a_hi < 0.0)))
        print(f"      {tag} iter {it + 1}/{n_iters}: "
              f"max|alpha|={np.max(np.abs(alpha)):.3f} m, "
              f"max|kappa|={np.max(np.abs(geo['kappa'])):.3f} 1/m, "
              f"코리도 밖 {n_out}/{len(a_lo)}")
    alpha, normal = _solve_qp(ref, safe_dist_m, objective)
    xy_final = ref["xy"] + alpha[:, None] * normal
    geo = _respline_closed(xy_final, ds_out, smooth=0.0)
    # 최종 해상도(0.1 m)에서 다시 재는 이유: 위 최적화는 0.5 m 격자에서만
    # 바운드를 걸므로 절점 사이에서 스플라인이 처질 수 있다. CSV에 나가는
    # d_left/d_right 는 실제로 주행할 점에서 잰 값이어야 한다.
    a_lo, a_hi = grid.corridor_interval(geo["xy"], geo["psi"])
    d_left = np.maximum(a_hi, 0.0)
    d_right = np.maximum(-a_lo, 0.0)
    return {"s": geo["s"], "xy": geo["xy"], "psi": geo["psi"],
            "kappa": geo["kappa"], "d_left": d_left, "d_right": d_right}


def _max_segment_endpoint_speed(base_speed, base_kappa, candidate_kappa,
                                ds, longitudinal_limit, tire_limit,
                                current_high):
    """한 segment의 낮은 속도 endpoint를 고정하고 안전한 높은 속도를 찾는다.

    구간 종가속도와 양 endpoint 중 더 큰 횡가속도를 동시에 friction circle에
    넣는다. candidate가 base와 같으면 ax=0이고 두 점 모두 pointwise lateral
    cap 아래이므로 항상 feasible하다.
    """
    base_sq = base_speed ** 2

    def feasible(candidate):
        candidate_sq = candidate ** 2
        longitudinal = (candidate_sq - base_sq) / (2.0 * ds)
        lateral = max(
            base_sq * base_kappa,
            candidate_sq * candidate_kappa)
        return (
            longitudinal <= longitudinal_limit + 1e-12 and
            np.hypot(longitudinal, lateral) <= tire_limit + 1e-12)

    if feasible(current_high):
        return current_high
    lo, hi = base_speed, current_high
    for _ in range(60):
        mid = 0.5 * (lo + hi)
        if feasible(mid):
            lo = mid
        else:
            hi = mid
    return lo


def velocity_profile(s, kappa, v_max, mu, a_acc_max, a_dec_max,
                     segment_ds=None, max_iterations=500):
    """폐루프 속도를 pointwise + segmentwise 합성 타이어 한계에 투영한다.

    각 segment는 양끝 중 더 큰 횡가속도를 사용한다. 한 endpoint만 보던 기존
    forward/backward pass는 곡률이 급증하는 지점 직전의 ax를 과대 허용해,
    최종 CSV에서 hypot(v²*kappa, ax)가 mu*g를 넘길 수 있었다.
    """
    values = (v_max, mu, a_acc_max, a_dec_max)
    if not all(np.isfinite(value) and value > 0.0 for value in values):
        raise ValueError("velocity profile limits must be positive and finite")
    kappa = np.abs(np.asarray(kappa, dtype=float))
    s = np.asarray(s, dtype=float)
    n = len(s)
    if n < 3 or len(kappa) != n or not np.isfinite(s).all() or \
            not np.isfinite(kappa).all():
        raise ValueError("velocity profile input is malformed")
    if segment_ds is None:
        nominal_last = s[-1] - s[-2]
        segment_ds = np.diff(s, append=s[-1] + nominal_last)
    segment_ds = np.asarray(segment_ds, dtype=float)
    if len(segment_ds) != n or not np.isfinite(segment_ds).all() or \
            np.any(segment_ds <= 1e-6):
        raise ValueError("closed-loop segment lengths must be positive and finite")

    tire_limit = mu * G
    # CSV 7-decimal 양자화가 저장 후 한계 초과로 뒤집히지 않게 projection만
    # 10 ppm 안쪽에서 수행한다. 검증 한계 자체는 낮추거나 완화하지 않는다.
    projection_limit = tire_limit * (1.0 - 1e-5)
    with np.errstate(divide="ignore"):
        lateral_cap = np.sqrt(
            projection_limit / np.maximum(kappa, 1e-12))
    v = np.minimum(lateral_cap, v_max)

    # 모든 update는 속도를 낮추기만 한다. 인접 구간을 다시 악화시킬 수 있으므로
    # 폐루프 전체를 반복하고, 마지막에는 독립적으로 재계산해 fail-closed 한다.
    for _ in range(max_iterations):
        max_reduction = 0.0
        for i in range(n):
            j = (i + 1) % n
            if v[j] >= v[i]:
                safe = _max_segment_endpoint_speed(
                    v[i], kappa[i], kappa[j], segment_ds[i],
                    a_acc_max, projection_limit, v[j])
                max_reduction = max(max_reduction, v[j] - safe)
                v[j] = safe
            else:
                safe = _max_segment_endpoint_speed(
                    v[j], kappa[j], kappa[i], segment_ds[i],
                    a_dec_max, projection_limit, v[i])
                max_reduction = max(max_reduction, v[i] - safe)
                v[i] = safe
        if max_reduction < 1e-10:
            break
    else:
        raise RuntimeError("velocity profile projection did not converge")

    next_v = np.roll(v, -1)
    ax = (next_v ** 2 - v ** 2) / (2.0 * segment_ds)
    worst_lateral = np.maximum(
        v ** 2 * kappa,
        next_v ** 2 * np.roll(kappa, -1))
    combined = np.hypot(worst_lateral, ax)
    accel_bad = (ax > a_acc_max + 1e-8)
    decel_bad = (ax < -a_dec_max - 1e-8)
    tire_bad = combined > tire_limit + 1e-8
    if np.any(accel_bad | decel_bad | tire_bad):
        bad = np.flatnonzero(accel_bad | decel_bad | tire_bad)
        raise RuntimeError(
            "velocity profile final projection failed at segments "
            f"{bad[:8].tolist()}: combined max={combined[bad].max():.6f}, "
            f"limit={tire_limit:.6f}")
    return v


def build_trajectory(geo, v_max=V_MAX, mu=MU, margin=MARGIN):
    s, kappa = geo["s"], geo["kappa"]
    xy = np.asarray(geo["xy"], dtype=float)
    ds = np.hypot(
        np.roll(xy[:, 0], -1) - xy[:, 0],
        np.roll(xy[:, 1], -1) - xy[:, 1])
    v = velocity_profile(
        s, kappa, v_max, mu, A_ACC_MAX, A_DEC_MAX,
        segment_ds=ds)
    ax = (np.roll(v, -1) ** 2 - v ** 2) / (2.0 * ds)
    # 실제 CSV 정밀도까지 양자화한 값도 다시 검사한다. 메모리 float가 PASS해도
    # 저장/재로드 후 FAIL하는 generation을 commit하지 않는다.
    csv_v = np.round(v, 7)
    csv_kappa = np.round(kappa, 7)
    csv_ax = np.round(ax, 7)
    csv_combined = np.hypot(csv_v ** 2 * np.abs(csv_kappa), csv_ax)
    if np.any(csv_combined > mu * G + 1e-9):
        bad = np.flatnonzero(csv_combined > mu * G + 1e-9)
        raise RuntimeError(
            "serialized velocity profile exceeds tire budget at "
            f"{bad[:8].tolist()}: max={csv_combined[bad].max():.9f}, "
            f"limit={mu * G:.9f}")
    return {"s": s, "xy": geo["xy"],
            "d_right": np.maximum(geo["d_right"] - margin, 0.0),
            "d_left": np.maximum(geo["d_left"] - margin, 0.0),
            "psi": geo["psi"], "kappa": kappa, "vx": v, "ax": ax}


def loop_orientation(xy):
    """폐루프의 진행 방향을 'cw' / 'ccw' 로. 부호면적 규약(양수=CCW)."""
    return "ccw" if _signed_area(xy) > 0.0 else "cw"


def resolve_orientation(traj, start_xy=(0.0, 0.0), want=None):
    """이 궤적을 어느 방향으로 낼지 결정해서 'cw'/'ccw' 로 돌려준다.

    want 가 'cw'/'ccw' 면 그대로 쓴다(권장 — CLI 기본값은 ccw). None 이면 예전
    forward_x 휴리스틱(--dir auto) — start_xy 최근접점에서 map +x 를 향하도록 맞춘다.

    ⚠️ 휴리스틱은 map (0,0) 이 '매핑을 시작할 때 차량 pose' 일 때만 의미가 있다.
       SLAM 은 map frame 을 차량 초기 pose 에 정렬하므로 원래는 성립하지만,
       yaml 의 origin 을 손으로 바꾸면 (0,0) 이 임의의 점이 되어 기준이 무너진다.
       그때는 라인 전체가 실제 주행 방향과 반대로 나올 수 있고, 이 스크립트는
       그걸 스스로 알아챌 방법이 없다. 반드시 --dir 로 명시할 것.
    """
    xy = traj["xy"]
    current = loop_orientation(xy)
    if want is not None:
        print(f"      [dir] 지정된 주행 방향: {want.upper()}")
        return want

    i0 = int(np.argmin(np.hypot(xy[:, 0] - start_xy[0], xy[:, 1] - start_xy[1])))
    c = float(np.cos(traj["psi"][i0]))
    resolved = current if c >= 0.0 else ("cw" if current == "ccw" else "ccw")
    print(f"      [dir] 자동 판정: map(0,0) 최근접점에서 cos(psi)={c:+.3f} "
          f"→ {resolved.upper()}")
    if abs(c) < 0.3:
        print(f"      [warn] 판정 여유가 작다 (|cos|={abs(c):.3f} < 0.3). 시작점 "
              "부근 트랙이 map +x 에 거의 수직이라 방향이 뒤집힐 수 있음 "
              "→ --dir cw|ccw 로 명시할 것")
    return resolved


def roll_to_start(traj, start_xy=(0.0, 0.0), orientation=None):
    """start_xy 최근접점을 인덱스 0 으로 굴리고, 필요하면 진행 방향을 뒤집는다.

    orientation: 'cw' | 'ccw' — 결과 루프의 진행 방향. None 이면 방향 유지.
    센터라인과 레이스라인이 각자 판정하면 서로 역방향이 될 수 있으므로,
    호출부에서 한 번 결정한 값을 모든 라인에 같이 넘길 것.
    """
    xy = traj["xy"]
    i0 = int(np.argmin(np.hypot(xy[:, 0] - start_xy[0], xy[:, 1] - start_xy[1])))
    flip = orientation is not None and loop_orientation(xy) != orientation

    def reorder(a):
        a = np.roll(a, -i0, axis=0)
        if flip:
            a = np.concatenate([a[:1], a[1:][::-1]])
        return a.copy()

    out = {k: reorder(v) for k, v in traj.items() if k != "s"}
    if flip:
        out["psi"] = np.arctan2(np.sin(out["psi"] + np.pi), np.cos(out["psi"] + np.pi))
        out["kappa"] = -out["kappa"]
        out["d_left"], out["d_right"] = out["d_right"], out["d_left"]
    xy = out["xy"]
    out["s"] = np.concatenate([[0.0], np.cumsum(np.hypot(*np.diff(xy, axis=0).T))])
    return out


def footprint_free(grid, xy, psi, half_length=CAR_HALF_LENGTH,
                   half_width=CAR_HALF_WIDTH):
    """각 pose의 차량 직사각형 전체가 free-space 안인지 검사한다.

    중심점/법선 거리만 보면 코너에서 앞·뒤 모서리가 벽을 자르는 문제를 놓친다.
    occupancy 해상도의 절반 간격으로 footprint 내부를 채워 검사한다.
    """
    xy = np.atleast_2d(xy)
    psi = np.atleast_1d(psi)
    step = max(0.5 * grid.res, 0.01)
    lon = np.arange(-half_length, half_length + 0.5 * step, step)
    lat = np.arange(-half_width, half_width + 0.5 * step, step)
    ok = np.ones(len(xy), dtype=bool)
    c, s = np.cos(psi), np.sin(psi)
    for dl in lon:
        for dd in lat:
            pts = xy + np.column_stack([dl * c - dd * s,
                                        dl * s + dd * c])
            ok &= grid.is_free(pts)
    return ok


def verify_line(name, traj, grid, half_width=CAR_HALF_WIDTH):
    """최종 라인이 실제로 주행 가능한지 occupancy 마스크에 대고 검사.

    ⚠️ 이 검사는 생략하면 안 된다. 2026-07-27 이전에는 어느 단계도 결과를
    맵에 대고 확인하지 않아서, 벽을 관통하는 라인이 CSV로 그대로 나가
    고속주행 테스트에 쓰였다 (my_track0727, 좌측 헤어핀 벽여유 0.016 m).

    두 가지를 본다:
      1. 벽 여유 >= 차 반폭 — 못 지키면 그 지점에서 차체가 벽에 닿는다.
      2. 요구 회전반경 >= 실차 최소 회전반경 — 못 지키면 조향이 포화해
         라인을 아예 못 따라간다 (핸들을 끝까지 꺾어도 모자람).

    반환: (ok, 진단 문자열 리스트)
    """
    required = ("s", "xy", "d_right", "d_left", "psi", "kappa", "vx", "ax")
    if any(k not in traj for k in required):
        return False, [f"      {name}: [FAIL] 필수 trajectory 필드 누락"]
    n = len(traj["s"])
    if n < 8 or any(len(traj[k]) != n for k in required if k != "xy") or \
            len(traj["xy"]) != n:
        return False, [f"      {name}: [FAIL] waypoint 길이 불일치 또는 8점 미만"]
    flat = np.column_stack([
        traj["s"], traj["xy"], traj["d_right"], traj["d_left"], traj["psi"],
        traj["kappa"], traj["vx"], traj["ax"]])
    if not np.isfinite(flat).all():
        return False, [f"      {name}: [FAIL] NaN/Inf 포함"]
    ds = np.diff(traj["s"])
    if np.any(ds <= 1e-5):
        return False, [f"      {name}: [FAIL] s가 단조 증가하지 않음"]
    if ds.max() > ds.min() * 1.25:
        return False, [f"      {name}: [FAIL] waypoint 간격 불균일 "
                       f"({ds.min():.3f}~{ds.max():.3f}m)"]
    geometric_ds = np.hypot(*np.diff(traj["xy"], axis=0).T)
    if np.any(geometric_ds <= 1e-5) or \
            np.any(np.abs(geometric_ds - ds) > np.maximum(0.02, 0.20 * ds)):
        return False, [f"      {name}: [FAIL] s/Cartesian 간격 불일치"]
    closure = float(np.linalg.norm(traj["xy"][0] - traj["xy"][-1]))
    if closure > ds.max() * 2.5:
        return False, [f"      {name}: [FAIL] 폐루프 불연속 {closure:.3f}m"]
    if np.any(traj["d_right"] < 0.0) or np.any(traj["d_left"] < 0.0) or \
            np.any(traj["vx"] < 0.0):
        return False, [f"      {name}: [FAIL] 음수 폭/속도 포함"]
    numeric_ax = (
        traj["vx"][1:] ** 2 - traj["vx"][:-1] ** 2) / (2.0 * ds)
    if np.max(np.abs(numeric_ax - traj["ax"][:-1])) > 0.5:
        return False, [f"      {name}: [FAIL] ax 열과 속도 프로파일 불일치"]

    clear = grid.clearance(traj["xy"], traj["psi"])
    r_min_car = WHEELBASE / np.tan(MAX_STEER)
    kappa = np.abs(traj["kappa"])
    r_line = 1.0 / np.maximum(kappa, 1e-9)
    lat_acc = traj["vx"] ** 2 * kappa

    tight = np.flatnonzero(clear < half_width)
    sharp = np.flatnonzero(r_line < r_min_car)
    combined_acc = np.hypot(lat_acc, traj["ax"])
    friction = np.flatnonzero(combined_acc > MU * G * (1.0 + 1e-6))
    footprint_bad = np.flatnonzero(
        ~footprint_free(grid, traj["xy"], traj["psi"],
                        half_width=half_width))
    msgs = [f"      {name}: 벽여유 min {clear.min():.3f} m "
            f"(기준 {half_width:.2f}) · R_min {r_line.min():.2f} m "
            f"(실차 한계 {r_min_car:.2f})"]

    def spans(idx):
        if len(idx) == 0:
            return []
        return np.split(idx, np.flatnonzero(np.diff(idx) > 3) + 1)

    for g in spans(tight)[:5]:
        msgs.append(f"        [FAIL] 벽여유 {clear[g].min():.3f} m @ "
                    f"s={traj['s'][g[0]]:.1f}~{traj['s'][g[-1]]:.1f} m "
                    f"xy=({traj['xy'][g[0], 0]:.2f},{traj['xy'][g[0], 1]:.2f})"
                    f" — {len(g)}점")
    for g in spans(footprint_bad)[:5]:
        msgs.append(f"        [FAIL] 차량 footprint 벽 침범 @ "
                    f"s={traj['s'][g[0]]:.1f}~{traj['s'][g[-1]]:.1f} m "
                    f"— {len(g)}점")
    for g in spans(sharp)[:5]:
        msgs.append(f"        [FAIL] R={r_line[g].min():.2f} m @ "
                    f"s={traj['s'][g[0]]:.1f}~{traj['s'][g[-1]]:.1f} m "
                    f"xy=({traj['xy'][g[0], 0]:.2f},{traj['xy'][g[0], 1]:.2f})"
                    f" — 조향 포화, {len(g)}점")
    for g in spans(friction)[:5]:
        msgs.append(f"        [FAIL] 합성 타이어 가속도 "
                    f"{combined_acc[g].max():.2f} m/s² > mu*g={MU * G:.2f} @ "
                    f"s={traj['s'][g[0]]:.1f}~{traj['s'][g[-1]]:.1f} m")
    return (len(tight) == 0 and len(footprint_bad) == 0 and
            len(sharp) == 0 and len(friction) == 0), msgs


def _trajectory_csv_bytes(traj, header_note="", generation_id=""):
    cols = np.column_stack([
        traj["s"], traj["xy"][:, 0], traj["xy"][:, 1],
        traj["d_right"], traj["d_left"], traj["psi"],
        traj["kappa"], traj["vx"], traj["ax"]])
    stream = io.StringIO()
    if generation_id:
        stream.write(f"# generation_id={generation_id}\n")
    if header_note:
        stream.write(f"# {header_note}\n")
    stream.write(
        "# s_m; x_m; y_m; d_right; d_left; psi_rad; kappa_radpm; "
        "vx_mps; ax_mps2\n")
    for row in cols:
        stream.write("; ".join(f"{value:.7f}" for value in row) + "\n")
    return stream.getvalue().encode("utf-8")


def write_csv(traj, path, header_note="", generation_id=""):
    """호환용 단일 파일 writer. 실제 생성 승격은 commit_generation_set 사용."""
    Path(path).parent.mkdir(parents=True, exist_ok=True)
    with open(path, "wb") as file:
        file.write(_trajectory_csv_bytes(
            traj, header_note, generation_id=generation_id))


def _savetxt_bytes(data, **kwargs):
    stream = io.StringIO()
    np.savetxt(stream, data, **kwargs)
    return stream.getvalue().encode("utf-8")


def _stage_bytes(path, payload, generation_id):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    fd, tmp_name = tempfile.mkstemp(
        prefix=f".{path.name}.{generation_id}.", suffix=".stage",
        dir=path.parent)
    with os.fdopen(fd, "wb") as file:
        file.write(payload)
        file.flush()
        os.fsync(file.fileno())
    return Path(tmp_name)


def _fsync_directory(path):
    flags = os.O_RDONLY | getattr(os, "O_DIRECTORY", 0)
    fd = os.open(Path(path), flags)
    try:
        os.fsync(fd)
    finally:
        os.close(fd)


def _manifest_path(path):
    path = Path(path)
    try:
        return str(path.relative_to(WORKSPACE_ROOT))
    except ValueError:
        return str(path)


def commit_generation_set(map_name, main_name, main_traj, center, walls,
                          orientation, sp_name=None, sp_traj=None,
                          sp_disabled_reason=""):
    """검증 완료된 생성 세트를 stage/fsync 후 순서 있게 승격한다.

    SP -> center/walls -> main 순으로 승격하고 각 단계 디렉터리를 fsync한다.
    runtime main CSV가 마지막 data commit point다. manifest/hash marker는 그 뒤에
    승격한다. SP가 없으면 같은 suffix의 기존 SP를 먼저 숨긴 후 commit 성공 시
    제거하므로 MAKE_SP=false/검증 실패가 stale SP를 남기지 않는다.

    여러 파일 rename은 하나의 원자 연산이 아니다. 따라서 자동 SP 선택은 꺼진
    상태를 전제로 하며, 명시적 main+SP 소비자는 양 CSV의 generation_id
    일치와 manifest hash를 확인해 전원 상실 중간 pair를 거부해야 한다.
    """
    # 매 실행마다 고유하며 모든 stage write 전에 확정한다. main/SP 헤더와
    # manifest가 이 ID를 공유해 런타임이 mixed generation을 거부할 수 있다.
    generation_id = uuid.uuid4().hex
    main_note = (
        f"build_map ({map_name}, dir={orientation.upper()}, "
        f"safe_dist={SAFE_DIST:.2f}m)")
    main_bytes = _trajectory_csv_bytes(
        main_traj, main_note, generation_id=generation_id)
    sp_bytes = None
    if sp_traj is not None:
        if not sp_name:
            raise ValueError("SP trajectory에 대응하는 파일명이 없음")
        sp_bytes = _trajectory_csv_bytes(
            sp_traj,
            f"build_map ({map_name}, SP overtake, "
            f"dir={orientation.upper()}, safe_dist={SAFE_DIST:.2f}m)",
            generation_id=generation_id)
    elif sp_name and not sp_disabled_reason:
        sp_disabled_reason = "SP not generated"

    walls_bytes = _savetxt_bytes(
        walls, delimiter="; ", fmt="%.7f",
        header="x_m; y_m; wall_id(0=outer,1=inner)")
    center_bytes = _savetxt_bytes(
        center["xy"], delimiter="; ", fmt="%.7f",
        header="x_m; y_m")
    digest = hashlib.sha256()
    for payload in (main_bytes, sp_bytes or b"SP_DISABLED",
                    walls_bytes, center_bytes):
        digest.update(len(payload).to_bytes(8, "big"))
        digest.update(payload)
    content_sha256 = digest.hexdigest()

    main_paths = [
        OUT_DIR / main_name,
        CSV_DIR / main_name,
    ]
    expected_sp_paths = (
        [OUT_DIR / sp_name, CSV_DIR / sp_name] if sp_name else [])
    sp_payloads = (
        [(path, sp_bytes, f"sp_{index}")
         for index, path in enumerate(expected_sp_paths)]
        if sp_bytes is not None else [])
    auxiliary_payloads = [
        (OUT_DIR / f"{map_name}_walls.csv", walls_bytes, "walls_output"),
        (CENTERLINE_DIR / f"{map_name}_walls.csv",
         walls_bytes, "walls_runtime"),
        (OUT_DIR / f"{map_name}_centerline.csv",
         center_bytes, "centerline_output"),
        (CENTERLINE_DIR / f"{map_name}_centerline.csv",
         center_bytes, "centerline_runtime"),
    ]
    main_payloads = [
        (main_paths[0], main_bytes, "main_output"),
        (main_paths[1], main_bytes, "main_runtime"),
    ]
    data_payloads = sp_payloads + auxiliary_payloads + main_payloads
    file_records = [{
        "role": role,
        "path": _manifest_path(path),
        "sha256": hashlib.sha256(payload).hexdigest(),
        "bytes": len(payload),
    } for path, payload, role in data_payloads]
    manifest = {
        "schema_version": 1,
        "generation_id": generation_id,
        "generation_content_sha256": content_sha256,
        "map": map_name,
        "main": main_name,
        "sp": sp_name if sp_bytes is not None else None,
        "sp_expected": sp_name,
        "sp_enabled": sp_bytes is not None,
        "sp_disabled_reason": (
            "" if sp_bytes is not None else sp_disabled_reason),
        "files": file_records,
    }
    manifest_bytes = (
        json.dumps(manifest, indent=2, sort_keys=True) + "\n").encode("utf-8")
    manifest_name = f"{map_name}_generation_manifest.json"
    manifest_payloads = [
        (OUT_DIR / manifest_name, manifest_bytes, "manifest_output"),
        (CSV_DIR / manifest_name, manifest_bytes, "manifest_runtime"),
    ]

    staged = {}
    backups = {}
    promoted = []
    data_committed = False
    touched_dirs = set()

    def stage_group(payloads):
        for path, payload, _ in payloads:
            staged[path] = _stage_bytes(path, payload, generation_id)
            touched_dirs.add(path.parent)

    def backup_existing(path):
        if path.exists():
            backup = path.with_name(
                f".{path.name}.{generation_id}.previous")
            os.replace(path, backup)
            backups[path] = backup

    def promote_group(payloads):
        for path, _, _ in payloads:
            backup_existing(path)
            tmp = staged[path]
            os.replace(tmp, path)
            del staged[path]
            promoted.append(path)
        for directory in {path.parent for path, _, _ in payloads}:
            _fsync_directory(directory)

    try:
        # 모든 payload를 먼저 완전히 기록+fsync한다. 이 전에는 기존 세트 불변.
        stage_group(data_payloads)
        stage_group(manifest_payloads)
        for directory in touched_dirs:
            _fsync_directory(directory)

        # SP disabled도 transaction의 일부다. 같은 suffix stale SP를 숨긴 뒤
        # directory fsync하여 main commit보다 먼저 영속화한다.
        if sp_bytes is None:
            for path in expected_sp_paths:
                path.parent.mkdir(parents=True, exist_ok=True)
                touched_dirs.add(path.parent)
                backup_existing(path)
            for directory in {path.parent for path in expected_sp_paths}:
                _fsync_directory(directory)
        elif sp_payloads:
            promote_group(sp_payloads)

        promote_group(auxiliary_payloads)
        # output 복사본 먼저, 런타임이 읽는 main CSV를 data commit point로 마지막.
        promote_group(main_payloads[:1])
        promote_group(main_payloads[1:])
        data_committed = True

        # Hash manifest는 data가 모두 영속화된 뒤 commit marker로 승격한다.
        promote_group(manifest_payloads)

        # 성공한 세트의 이전/비활성 SP는 glob에 잡히지 않는 hidden backup에서 제거.
        for backup in backups.values():
            if backup.exists():
                backup.unlink()
        for directory in touched_dirs:
            _fsync_directory(directory)
    except Exception:
        if not data_committed:
            # 정상 예외에서는 기존 세트를 복구한다. 전원 상실 중간 상태는
            # main-last + manifest로 탐지하며 자동 SP 선택이 꺼져 있어야 안전하다.
            for path in reversed(promoted):
                if path.exists():
                    path.unlink()
            for path, backup in backups.items():
                if backup.exists():
                    os.replace(backup, path)
            for directory in touched_dirs:
                _fsync_directory(directory)
        raise
    finally:
        for tmp in staged.values():
            if tmp.exists():
                tmp.unlink()

    for path, _, role in data_payloads:
        print(f"      committed [{role}] {path}")
    if sp_bytes is None and expected_sp_paths:
        print(f"      SP disabled before main commit: {sp_disabled_reason}")
    print(f"      generation manifest {generation_id}: "
          f"{CSV_DIR / manifest_name}")
    return manifest


# ============================================================ 3. 시뮬레이션
_PROFILE_NUMERIC_FIELDS = (
    "planner_speed_scale", "controller_speed_scale", "max_speed_mps",
    "lookahead_gain", "lookahead_offset", "lookahead_min_m",
    "lookahead_max_m", "wheelbase_m", "max_steer_rad",
    "max_steer_rate_radps", "max_accel_mps2", "max_decel_mps2",
    "control_rate_hz", "num_local_waypoints",
)


def controller_profile(name):
    """이름으로 선택한 배포 프로파일을 검증해 사본으로 반환한다."""
    if not isinstance(name, str) or name not in CONTROLLER_PROFILE_LIBRARY:
        raise ValueError(f"알 수 없는 controller profile: {name!r}")
    profile = dict(CONTROLLER_PROFILE_LIBRARY[name])
    profile["name"] = name
    if profile.get("kind") not in ("cpp_map_l1", "python_pure_pursuit"):
        raise ValueError(f"{name}: 지원하지 않는 controller kind")
    for field in _PROFILE_NUMERIC_FIELDS:
        value = profile.get(field)
        if isinstance(value, bool) or not isinstance(value, (int, float)) or \
                not np.isfinite(value):
            raise ValueError(f"{name}: {field}가 유한 숫자가 아님")
    if not (0.0 < profile["planner_speed_scale"] <= 1.0) or \
            not (0.0 < profile["controller_speed_scale"] <= 1.0):
        raise ValueError(f"{name}: speed scale은 (0, 1]이어야 함")
    positive = (
        "max_speed_mps", "lookahead_min_m", "lookahead_max_m",
        "wheelbase_m", "max_steer_rad", "max_steer_rate_radps",
        "max_accel_mps2", "max_decel_mps2", "control_rate_hz",
    )
    if any(profile[field] <= 0.0 for field in positive):
        raise ValueError(f"{name}: 차량/제어 한계는 양수여야 함")
    if profile["lookahead_max_m"] < profile["lookahead_min_m"]:
        raise ValueError(f"{name}: lookahead min/max가 뒤집힘")
    if profile["max_steer_rad"] >= 0.5 * np.pi:
        raise ValueError(f"{name}: max steer가 자전거 모델 범위를 벗어남")
    n_local = profile["num_local_waypoints"]
    if int(n_local) != n_local or n_local < 2:
        raise ValueError(f"{name}: num_local_waypoints는 2 이상의 정수여야 함")
    profile["num_local_waypoints"] = int(n_local)
    lut = profile.get("steering_lut")
    if profile["kind"] == "cpp_map_l1" and \
            (not isinstance(lut, str) or not lut):
        raise ValueError(f"{name}: C++ MAP/L1 steering LUT가 지정되지 않음")
    if profile["kind"] == "python_pure_pursuit" and lut is not None:
        raise ValueError(f"{name}: Python pure pursuit에 LUT를 지정할 수 없음")
    return profile


def selected_controller_profiles():
    """빈/중복/오타 선택을 허용하지 않고, 선택된 모든 프로파일을 반환한다."""
    names = tuple(SIM_CONTROLLER_PROFILES)
    if not names:
        raise ValueError("SIM_CONTROLLER_PROFILES가 비어 있음 — 검증 없이 저장 금지")
    if len(names) != len(set(names)):
        raise ValueError("SIM_CONTROLLER_PROFILES에 중복 이름이 있음")
    return tuple(controller_profile(name) for name in names)


def _load_steering_lut(relative_path):
    """control_node.cpp와 같은 CSV 계약으로 LUT를 읽고, 이상하면 즉시 실패한다."""
    path = (WORKSPACE_ROOT / relative_path).resolve()
    try:
        path.relative_to(WORKSPACE_ROOT.resolve())
    except ValueError as exc:
        raise ValueError(f"steering LUT가 workspace 밖을 가리킴: {path}") from exc
    if not path.is_file():
        raise FileNotFoundError(f"배포 steering LUT 없음: {path}")
    try:
        table = np.loadtxt(path, delimiter=",")
    except (OSError, ValueError) as exc:
        raise ValueError(f"steering LUT 파싱 실패: {path}: {exc}") from exc
    if table.ndim != 2 or table.shape[0] < 2 or table.shape[1] < 2:
        raise ValueError(f"steering LUT 크기 오류: {path}")
    if not np.isfinite(table[0, 1:]).all() or \
            not np.isfinite(table[1:, 0]).all():
        raise ValueError(f"steering LUT 축에 NaN/Inf 포함: {path}")
    if np.any(np.sum(np.isfinite(table[1:, 1:]), axis=0) == 0):
        raise ValueError(f"steering LUT에 유효 가속도 행이 없는 속도 열 포함: {path}")
    return table


def _lookup_steer_angle(table, acceleration, velocity):
    """control_node.cpp LookupSteerAngle::lookup_steer_angle의 동일 보간."""
    sign = 1.0 if acceleration > 0.0 else -1.0
    acceleration = abs(float(acceleration))
    velocity_column = int(
        np.argmin(np.abs(table[0, 1:] - velocity))) + 1
    finite_rows = np.flatnonzero(
        np.isfinite(table[1:, velocity_column]) &
        np.isfinite(table[1:, 0])) + 1
    if len(finite_rows) == 0:
        raise ValueError("steering LUT 선택 열에 유효 행이 없음")
    closest_position = int(np.argmin(
        np.abs(table[finite_rows, velocity_column] - acceleration)))
    if closest_position == 0:
        return 0.0
    if closest_position + 1 >= len(finite_rows):
        second_position = closest_position
    else:
        before = closest_position - 1
        after = closest_position + 1
        before_error = abs(
            table[finite_rows[before], velocity_column] - acceleration)
        after_error = abs(
            table[finite_rows[after], velocity_column] - acceleration)
        second_position = before if before_error <= after_error else after
    first_row = finite_rows[closest_position]
    second_row = finite_rows[second_position]
    steering_angle = float(table[first_row, 0])
    if first_row != second_row:
        first_accel = table[first_row, velocity_column]
        second_accel = table[second_row, velocity_column]
        if abs(second_accel - first_accel) > 1e-9:
            ratio = np.clip(
                (acceleration - first_accel) / (second_accel - first_accel),
                0.0, 1.0)
            steering_angle += ratio * (
                table[second_row, 0] - table[first_row, 0])
    return float(steering_angle * sign)


def _lookup_lateral_accel(table, steering_angle, velocity):
    """같은 실측 LUT를 정방향(steer, velocity -> lateral accel)으로 보간한다.

    C++ 제어기의 inverse LUT로 명령을 만든 뒤 plant를 이상적인
    v²*tan(delta)/L로 적분하면 NUC6 실측 응답보다 낙관적이다. 속도 열 선택은
    control_node와 같이 nearest-column이고, 조향축 사이만 선형 보간한다.
    """
    sign = 1.0 if steering_angle > 0.0 else -1.0
    steering = abs(float(steering_angle))
    velocity_abs = abs(float(velocity))
    velocity_column = int(
        np.argmin(np.abs(table[0, 1:] - velocity_abs))) + 1
    finite_rows = np.flatnonzero(
        np.isfinite(table[1:, velocity_column]) &
        np.isfinite(table[1:, 0])) + 1
    if len(finite_rows) == 0:
        raise ValueError("steering LUT selected column has no finite rows")
    steer_axis = table[finite_rows, 0]
    accel_axis = table[finite_rows, velocity_column]
    order = np.argsort(steer_axis)
    steer_axis = steer_axis[order]
    accel_axis = accel_axis[order]
    lateral = float(np.interp(
        steering, steer_axis, accel_axis,
        left=accel_axis[0], right=accel_axis[-1]))
    # LUT의 최저 속도(0.5 m/s) 아래를 그대로 clamp하면 정지 상태에서도
    # 횡가속이 생긴다. bicycle 저속 극한에 맞춰 v²로 보수적으로 축소한다.
    minimum_table_speed = float(np.min(table[0, 1:]))
    if velocity_abs < minimum_table_speed and minimum_table_speed > 0.0:
        lateral *= (velocity_abs / minimum_table_speed) ** 2
    return lateral * sign


class PurePursuitSim:
    """선택한 실제 컨트롤러 계약 + friction-circle 차량 모델.

    profile_name을 필수로 받아 예전의 이름 없는 generic 설정으로 PASS가
    나오는 것을 막는다. 제어기가 타이어 예산을 넘는 명령을 요구하면 모델은
    물리 한계로 포화해 적분하되, verify_simulation은 그 요구 자체를 FAIL한다.
    """

    def __init__(self, traj, profile_name, mu=MU):
        self.profile = controller_profile(profile_name)
        self.traj = traj
        self.xy = np.asarray(traj["xy"], dtype=float)
        raw_vx = np.asarray(traj["vx"], dtype=float)
        self.vx_guidance_ref = (
            raw_vx * self.profile["planner_speed_scale"])
        self.vx_ref = np.minimum(
            self.vx_guidance_ref * self.profile["controller_speed_scale"],
            self.profile["max_speed_mps"])
        self.s_ref = np.asarray(traj["s"], dtype=float)
        self.n = len(self.xy)
        self.track_len = (
            self.s_ref[-1] + np.hypot(*(self.xy[0] - self.xy[-1])))
        self.tree = cKDTree(self.xy)
        self.L = self.profile["wheelbase_m"]
        self.max_steer = self.profile["max_steer_rad"]
        self.max_steer_rate = self.profile["max_steer_rate_radps"]
        self.lookahead_gain = self.profile["lookahead_gain"]
        self.lookahead_offset = self.profile["lookahead_offset"]
        self.lookahead_clip = (
            self.profile["lookahead_min_m"],
            self.profile["lookahead_max_m"])
        self.a_acc_max = self.profile["max_accel_mps2"]
        self.a_dec_max = self.profile["max_decel_mps2"]
        self.n_local = min(self.profile["num_local_waypoints"], self.n)
        self.dt = 1.0 / self.profile["control_rate_hz"]
        if not np.isfinite(mu) or mu <= 0.0:
            raise ValueError("시뮬레이션 mu는 양의 유한값이어야 함")
        self.tire_limit = float(mu * G)
        self.steering_lut = None
        if self.profile["kind"] == "cpp_map_l1":
            self.steering_lut = _load_steering_lut(
                self.profile["steering_lut"])

    def local_window(self, idx):
        return (idx + np.arange(self.n_local)) % self.n

    def lookahead_point(self, pos, idx, lookahead):
        win = self.local_window(idx)
        if self.profile["kind"] == "cpp_map_l1":
            # control_node waypoint_at_distance_before_car(): local vector의
            # 실제 segment 길이를 누적하며, 마지막 점 뒤로 wrap하지 않는다.
            segments = np.hypot(*np.diff(self.xy[win], axis=0).T)
            reached = np.nonzero(np.cumsum(segments) >= lookahead)[0]
            j = win[reached[0] + 1] if len(reached) else win[-1]
            return self.xy[j], j
        # waypoint_tracker_node.py: 차량에서 Euclidean lookahead 이상인 첫 점.
        d = np.hypot(*(self.xy[win] - pos).T)
        ahead = np.nonzero(d >= lookahead)[0]
        j = win[ahead[0]] if len(ahead) else win[-1]
        return self.xy[j], j

    def _steering_target(self, alpha, target_distance, lookahead,
                         guidance_speed):
        if self.profile["kind"] == "cpp_map_l1":
            lateral_accel = (
                2.0 * guidance_speed ** 2 / max(lookahead, 1e-3) *
                np.sin(alpha))
            return _lookup_steer_angle(
                self.steering_lut, lateral_accel, guidance_speed)
        return np.arctan2(
            2.0 * self.L * np.sin(alpha), target_distance)

    def _speed_target(self, pos, theta, speed, target_idx):
        if self.profile["kind"] == "cpp_map_l1":
            # control params map/speed_lookahead (프로파일과 런타임 YAML 동기화).
            query = pos + speed * self.profile["speed_lookahead_s"] * np.array(
                [np.cos(theta), np.sin(theta)])
            _, speed_idx = self.tree.query(query)
        else:
            # waypoint_tracker_node.py는 lookahead target waypoint의 vx 사용.
            speed_idx = target_idx
        return float(self.vx_ref[int(speed_idx)])

    def run(self, n_laps=1, max_time=300.0):
        pos = self.xy[0].copy()
        theta = float(self.traj["psi"][0])
        v = 0.0
        prev_s = 0.0
        fields = (
            "t", "x", "y", "theta", "v", "v_ref", "s", "d_err", "steer",
            "a_long_cmd", "a_lat_cmd", "a_long", "a_lat",
            "combined_demand", "combined_accel", "tire_saturated")
        log = {key: [] for key in fields}
        t = 0.0
        laps_done = 0
        steer = 0.0
        while t < max_time:
            _, idx = self.tree.query(pos)
            idx = int(idx)
            s_now = self.s_ref[idx]
            if prev_s > 0.8 * self.track_len and s_now < 0.2 * self.track_len:
                laps_done += 1
                if laps_done >= n_laps:
                    break
            prev_s = s_now
            t_hat = np.array([np.cos(self.traj["psi"][idx]),
                              np.sin(self.traj["psi"][idx])])
            e_vec = pos - self.xy[idx]
            d_err = t_hat[0] * e_vec[1] - t_hat[1] * e_vec[0]
            lookahead_min = self.lookahead_clip[0]
            if self.profile["kind"] == "cpp_map_l1":
                # calc_l1_point()의 lateral-error dependent lower bound.
                lookahead_min = min(max(
                    lookahead_min, np.sqrt(2.0) * abs(d_err)),
                    self.lookahead_clip[1])
            lookahead = np.clip(
                self.lookahead_gain * abs(v) + self.lookahead_offset,
                lookahead_min, self.lookahead_clip[1])
            target, target_idx = self.lookahead_point(pos, idx, lookahead)
            dx, dy = target - pos
            alpha = np.arctan2(dy, dx) - theta
            alpha = np.arctan2(np.sin(alpha), np.cos(alpha))
            target_distance = max(np.hypot(dx, dy), 1e-3)
            steer_target = np.clip(
                self._steering_target(
                    alpha, target_distance, lookahead,
                    self.vx_guidance_ref[idx]),
                -self.max_steer, self.max_steer)
            steer += np.clip(
                steer_target - steer,
                -self.max_steer_rate * self.dt,
                self.max_steer_rate * self.dt)

            v_cmd = self._speed_target(
                pos, theta, v, int(target_idx))
            a_long_cmd = float(np.clip(
                (v_cmd - v) / self.dt, -self.a_dec_max, self.a_acc_max))
            if self.profile["kind"] == "cpp_map_l1":
                # NUC6의 measured forward response로 plant를 적분한다.
                # inverse LUT command + ideal bicycle plant 조합은 실차
                # understeer를 숨겨 footprint PASS를 낙관적으로 만든다.
                a_lat_cmd = _lookup_lateral_accel(
                    self.steering_lut, steer, v)
            else:
                a_lat_cmd = float(v ** 2 * np.tan(steer) / self.L)
            combined_demand = float(np.hypot(a_long_cmd, a_lat_cmd))

            # Lateral demand gets priority because steering has already been
            # applied. Remaining friction-circle capacity alone may be used
            # for drive/brake. This guarantees the integrated vehicle never
            # receives impossible simultaneous ideal lateral+longitudinal accel.
            a_lat = float(np.clip(
                a_lat_cmd, -self.tire_limit, self.tire_limit))
            a_long_available = float(np.sqrt(max(
                self.tire_limit ** 2 - a_lat ** 2, 0.0)))
            a_long = float(np.clip(
                a_long_cmd, -a_long_available, a_long_available))
            combined_accel = float(np.hypot(a_long, a_lat))
            tire_saturated = combined_demand > self.tire_limit * (1.0 + 1e-9)

            v_next = max(v + a_long * self.dt, 0.0)
            yaw_rate = a_lat / v if v > 1e-6 else 0.0
            theta += yaw_rate * self.dt
            v_mean = 0.5 * (v + v_next)
            pos = pos + v_mean * self.dt * np.array(
                [np.cos(theta), np.sin(theta)])
            v = v_next
            t += self.dt

            log["t"].append(t)
            log["x"].append(pos[0]); log["y"].append(pos[1])
            log["theta"].append(theta)
            log["v"].append(v); log["v_ref"].append(v_cmd)
            log["s"].append(s_now + laps_done * self.track_len)
            log["d_err"].append(d_err); log["steer"].append(steer)
            log["a_long_cmd"].append(a_long_cmd)
            log["a_lat_cmd"].append(a_lat_cmd)
            log["a_long"].append(a_long)
            log["a_lat"].append(a_lat)
            log["combined_demand"].append(combined_demand)
            log["combined_accel"].append(combined_accel)
            log["tire_saturated"].append(float(tire_saturated))
        return {key: np.asarray(value) for key, value in log.items()} | {
            "laps": laps_done,
            "track_len": self.track_len,
            "profile_name": self.profile["name"],
            "tire_limit": self.tire_limit,
        }


def verify_simulation(name, log, grid, requested_laps, profile_name):
    """배포 프로파일 폐루프/footprint/합성 타이어 예산을 fail-closed 검사."""
    profile = controller_profile(profile_name)
    msgs = []
    fields = (
        "t", "x", "y", "theta", "v", "v_ref", "s", "d_err", "steer",
        "a_long_cmd", "a_lat_cmd", "a_long", "a_lat",
        "combined_demand", "combined_accel", "tire_saturated")
    lengths = [len(log.get(key, ())) for key in fields]
    if not lengths or min(lengths) == 0 or len(set(lengths)) != 1:
        return False, [f"      {name}/{profile_name}: [FAIL] "
                       "시뮬레이션 로그가 비었거나 길이가 다름"]
    data = np.column_stack([log[key] for key in fields])
    if not np.isfinite(data).all():
        return False, [f"      {name}/{profile_name}: [FAIL] "
                       "시뮬레이션 로그에 NaN/Inf 포함"]
    prefix = f"      {name}/{profile_name}"
    if log.get("profile_name") != profile_name:
        msgs.append(f"{prefix}: [FAIL] 로그/profile 이름 불일치")
    tire_limit = float(log.get("tire_limit", np.nan))
    if not np.isfinite(tire_limit) or tire_limit <= 0.0 or \
            abs(tire_limit - MU * G) > 1e-9:
        msgs.append(f"{prefix}: [FAIL] tire limit metadata 불일치")
        tire_limit = MU * G
    if int(log.get("laps", 0)) < requested_laps:
        msgs.append(f"{prefix}: [FAIL] {requested_laps}랩 중 "
                    f"{int(log.get('laps', 0))}랩만 완료 "
                    "(timeout/진행방향 오류)")

    dt = np.diff(log["t"])
    if len(dt) and (np.any(dt <= 0.0) or not np.isfinite(dt).all()):
        msgs.append(f"{prefix}: [FAIL] 로그 시간이 단조 증가하지 않음")
    pose_xy = np.column_stack([log["x"], log["y"]])
    footprint_bad = np.flatnonzero(
        ~footprint_free(grid, pose_xy, log["theta"]))
    if len(footprint_bad):
        first = int(footprint_bad[0])
        msgs.append(f"{prefix}: [FAIL] 폐루프 주행 footprint가 벽 침범 "
                    f"({len(footprint_bad)} step, 최초 "
                    f"t={log['t'][first]:.2f}s, "
                    f"xy=({log['x'][first]:.2f},{log['y'][first]:.2f}))")

    steer_bad = np.flatnonzero(
        np.abs(log["steer"]) > profile["max_steer_rad"] + 1e-6)
    if len(steer_bad):
        msgs.append(f"{prefix}: [FAIL] 조향각 한계 초과 "
                    f"{np.abs(log['steer'][steer_bad]).max():.3f} rad")
    if len(log["steer"]) > 1 and len(dt):
        rates = np.abs(np.diff(log["steer"]) / dt)
        if np.any(rates > profile["max_steer_rate_radps"] * 1.001):
            msgs.append(f"{prefix}: [FAIL] 조향 속도 한계 초과 "
                        f"{rates.max():.2f} rad/s")
    if np.any(log["v"] < -1e-6) or \
            np.any(log["v"] > profile["max_speed_mps"] + 1e-6):
        msgs.append(f"{prefix}: [FAIL] 전진 속도/프로파일 cap 위반")

    reconstructed_demand = np.hypot(
        log["a_long_cmd"], log["a_lat_cmd"])
    reconstructed_actual = np.hypot(log["a_long"], log["a_lat"])
    if not np.allclose(
            reconstructed_demand, log["combined_demand"],
            rtol=1e-9, atol=1e-9) or not np.allclose(
                reconstructed_actual, log["combined_accel"],
                rtol=1e-9, atol=1e-9):
        msgs.append(f"{prefix}: [FAIL] 합성 가속도 로그 산술 불일치")
    demand_bad = np.flatnonzero(
        log["combined_demand"] > tire_limit * (1.0 + 1e-9))
    saturation_mask = log["tire_saturated"] > 0.5
    if not np.array_equal(
            saturation_mask,
            log["combined_demand"] > tire_limit * (1.0 + 1e-9)):
        msgs.append(f"{prefix}: [FAIL] tire saturation 로그/요구량 불일치")
    if len(demand_bad):
        first = int(demand_bad[0])
        msgs.append(
            f"{prefix}: [FAIL] 제어 요구 합성가속도 "
            f"{log['combined_demand'][demand_bad].max():.2f} m/s² > "
            f"mu*g={tire_limit:.2f} ({len(demand_bad)} step, 최초 "
            f"t={log['t'][first]:.2f}s, s={log['s'][first]:.2f}m) — "
            "물리 모델은 포화했지만 이 프로파일은 승인하지 않음")
    actual_bad = np.flatnonzero(
        log["combined_accel"] > tire_limit * (1.0 + 1e-9))
    if len(actual_bad):
        msgs.append(
            f"{prefix}: [FAIL] friction-circle 적용 후 합성가속도도 "
            f"{log['combined_accel'][actual_bad].max():.6f} m/s²로 한계 초과")
    cmd_accel_bad = (
        (log["a_long_cmd"] > profile["max_accel_mps2"] + 1e-9) |
        (log["a_long_cmd"] < -profile["max_decel_mps2"] - 1e-9))
    if np.any(cmd_accel_bad):
        msgs.append(f"{prefix}: [FAIL] 종가속/제동 명령 cap 위반")

    if not msgs:
        effective_scale = (
            profile["planner_speed_scale"] *
            profile["controller_speed_scale"])
        msgs.append(
            f"{prefix}: PASS · {requested_laps}랩 · profile speed scale "
            f"{effective_scale:.4f} · footprint 충돌 0 · "
            f"|lat err|max {np.abs(log['d_err']).max():.3f} m · "
            f"합성 요구/실제 max {log['combined_demand'].max():.2f}/"
            f"{log['combined_accel'].max():.2f} <= {tire_limit:.2f} m/s²")
    return not any("[FAIL]" in msg for msg in msgs), msgs


def simulate_controller_profiles(name, traj, grid, requested_laps, profiles):
    """선택된 모든 배포 프로파일을 실행하고 하나라도 실패하면 전체 실패."""
    all_ok = True
    messages = []
    primary_log = None
    first_failed_log = None
    for profile in profiles:
        log = PurePursuitSim(traj, profile["name"]).run(
            n_laps=requested_laps)
        ok, profile_messages = verify_simulation(
            name, log, grid, requested_laps, profile["name"])
        if primary_log is None:
            primary_log = log
        if not ok and first_failed_log is None:
            first_failed_log = log
        all_ok = all_ok and ok
        messages.extend(profile_messages)
    return all_ok, messages, primary_log, first_failed_log


# ============================================================ 4. 오케스트레이션
def _save_plots(m, traj, center, outer, inner, orientation, start_heading,
                grid=None, log=None, suffix=""):
    """궤적/추종 그림 저장. 검증 실패 시에도 (log 없이) 진단용으로 부른다."""
    has_log = log is not None and len(log.get("t", ())) > 0
    lap_time = (log["t"][-1] / max(log["laps"], 1)
                if has_log else float("nan"))
    fig, ax = plt.subplots(figsize=(11, 8))
    ax.plot(*outer.T, "k-", lw=1); ax.plot(*inner.T, "k-", lw=1)
    if RACELINE == "iqp":
        ax.plot(center["xy"][:, 0], center["xy"][:, 1], "--", color="gray",
                lw=0.8, alpha=0.7, label="centerline")
    pts = ax.scatter(traj["xy"][:, 0], traj["xy"][:, 1], c=traj["vx"],
                     cmap="viridis", s=4, label=f"{RACELINE} raceline (vx)")
    if has_log:
        ax.plot(log["x"], log["y"], "r-", lw=0.8, alpha=0.8, label="driven path")
    # 검증 위반 지점을 빨간 X 로 — 실패 그림에서 어디가 문제인지 바로 보이게
    if grid is not None:
        bad = grid.clearance(traj["xy"], traj["psi"]) < CAR_HALF_WIDTH
        if bad.any():
            ax.scatter(traj["xy"][bad, 0], traj["xy"][bad, 1], marker="x",
                       c="red", s=60, lw=2, zorder=6,
                       label=f"clearance < {CAR_HALF_WIDTH:.2f} m")
    ax.plot(traj["xy"][0, 0], traj["xy"][0, 1], "b*", ms=14, label="start")
    # 주행 방향 화살표. 폐루프 시뮬은 초기 헤딩을 궤적 자신에서 가져오므로
    # 방향이 뒤집혀 있어도 깨끗하게 돈다 → 방향 검증은 이 그림으로만 가능하다.
    n_arrow = 12
    step = max(len(traj["xy"]) // n_arrow, 1)
    for i in range(0, len(traj["xy"]), step):
        ax.annotate("", xytext=traj["xy"][i],
                    xy=traj["xy"][i] + 0.45 * np.array(
                        [np.cos(traj["psi"][i]), np.sin(traj["psi"][i])]),
                    arrowprops=dict(arrowstyle="-|>", color="crimson",
                                    lw=1.4, mutation_scale=14))
    ax.annotate("", xytext=traj["xy"][0],
                xy=traj["xy"][0] + 0.9 * np.array(
                    [np.cos(start_heading), np.sin(start_heading)]),
                arrowprops=dict(arrowstyle="-|>", color="blue", lw=2.5,
                                mutation_scale=20))
    fig.colorbar(pts, ax=ax, label="vx [m/s]")
    ax.set_aspect("equal"); ax.legend(loc="upper right")
    # 제목은 ASCII로 — 이 기계 matplotlib 기본 폰트에 한글 글리프가 없어서
    # 한글을 쓰면 전부 두부(tofu)로 깨진다.
    head = (f"global trajectory & lap ({lap_time:.2f} s/lap)" if has_log
            else "VERIFICATION FAILED -- no CSV written")
    ax.set_title(f"{m}: {head}\n"
                 f"travel dir {orientation.upper()}, start heading "
                 f"{np.degrees(start_heading):+.1f} deg "
                 f"-- check against real car placement")
    ax.set_xlabel("x [m]"); ax.set_ylabel("y [m]")
    fig.tight_layout()
    fig.savefig(OUT_DIR / f"{m}_trajectory{suffix}.png", dpi=150)
    plt.close(fig)
    if not has_log:
        return

    fig, axes = plt.subplots(3, 1, figsize=(11, 9), sharex=True)
    axes[0].plot(log["s"], log["v"], label="v actual")
    axes[0].plot(log["s"], log["v_ref"], "--", label="v reference")
    axes[0].set_ylabel("v [m/s]"); axes[0].legend()
    axes[1].plot(log["s"], log["d_err"]); axes[1].axhline(0, color="k", lw=0.5)
    axes[1].set_ylabel("lateral error [m]")
    axes[2].plot(log["s"], np.degrees(log["steer"]))
    axes[2].set_ylabel("steer [deg]"); axes[2].set_xlabel("s [m]")
    axes[0].set_title(f"{m}: closed-loop tracking")
    fig.tight_layout()
    fig.savefig(OUT_DIR / f"{m}_tracking{suffix}.png", dpi=150)
    plt.close(fig)


def main():
    ap = argparse.ArgumentParser(description="맵 이름 하나로 레이스라인 생성")
    ap.add_argument("map", nargs="?", default="my_map",
                    help="<이름>.yaml 의 확장자 뺀 이름 "
                         "(~/IFAC_ws/maps -> planning_pkg/maps 순으로 찾음)")
    ap.add_argument("--dir", choices=("cw", "ccw", "auto"), default="ccw",
                    help="주행 방향 (기본 ccw). 이 스택의 트랙은 반시계로 돈다 — "
                         "시계로 돌 맵만 cw 를 준다. auto 는 예전 휴리스틱으로, "
                         "map(0,0) 최근접점이 +x 를 향하도록 맞춘다. 이는 map origin "
                         "이 SLAM 시작 시 차량 pose 이고 그때 차가 주행 방향을 보고 "
                         "있었을 때만 맞으므로 권장하지 않는다.")
    args = ap.parse_args()
    m = args.map
    profiles = selected_controller_profiles()
    OUT_DIR.mkdir(parents=True, exist_ok=True)
    print("폐루프 필수 배포 프로파일: " +
          ", ".join(profile["name"] for profile in profiles))

    # 1) 트랙 추출 (+ 실측 크기 스케일 보정) --------------------------------
    print(f"[1/4] '{m}' 로드 + 트랙 추출 (크기는 yaml resolution 기준) ...")
    outer, inner, resolution, origin, raw_ring, skeleton_ring, H = \
        load_track(m)
    grid = Grid(
        raw_ring, resolution, origin, H,
        skeleton_ring=skeleton_ring)
    center = extract_centerline(grid)
    # 방향은 여기서 '한 번만' 정하고 레이스라인에도 그대로 넘긴다. 각자 판정하면
    # 시작점 부근이 map +x 에 수직일 때 센터라인과 레이스라인이 역방향이 된다.
    orientation = resolve_orientation(
        center, want=None if args.dir == "auto" else args.dir)
    center = roll_to_start(center, orientation=orientation)
    print(f"      centerline: {len(center['s'])} pts, 길이 {center['s'][-1]:.1f} m")

    walls = np.vstack([
        np.column_stack([outer, np.zeros(len(outer))]),
        np.column_stack([inner, np.ones(len(inner))])])

    # 2) 레이스라인 + 속도 프로파일 ----------------------------------------
    if RACELINE == "iqp":
        print("[2/4] IQP 최소곡률 라인 + 속도 프로파일 ...")
        geo = roll_to_start(optimize_raceline_iqp(center, grid),
                            orientation=orientation)
    elif RACELINE == "sp":
        print("[2/4] SP 최단거리 라인 + 속도 프로파일 ...")
        geo = roll_to_start(optimize_raceline_iqp(center, grid,
                                                  objective="length"),
                            orientation=orientation)
    else:
        # ⚠️ 이 모드는 검증을 거의 못 통과한다 (설정 블록 주석 참조).
        print("[2/4] 중앙선 라인 + 속도 프로파일 ...")
        geo = center
    traj = build_trajectory(geo, mu=MU_PROFILE)
    start_heading = float(traj["psi"][0])
    print(f"      [dir] 주행 방향 {orientation.upper()} · 출발점 "
          f"({traj['xy'][0, 0]:+.2f}, {traj['xy'][0, 1]:+.2f}) m, "
          f"헤딩 {np.degrees(start_heading):+.1f}° (map +x 기준 CCW)")
    print("      ↑ 실차를 이 지점에 이 방향으로 놓는 게 맞는지 눈으로 확인할 것. "
          "폐루프 시뮬은 헤딩을 궤적 자신에서 가져오므로 방향이 뒤집혀도 "
          "정상으로 보인다.")
    # center 도 이름에 태그를 붙인다. planner_stack.launch.py 의 map:= 는
    # <맵>_iqp_*.csv 만 글롭하므로, 태그가 붙어 있으면 center CSV가 실수로
    # 메인 라인으로 잡히는 일이 없고 trajectory_csv_file:= 로만 쓰게 된다.
    tag = {"iqp": "iqp_", "sp": "sp_", "center": "center_"}.get(RACELINE, "")
    csv_name = (f"{m}_{tag}max{V_MAX:.2f}ms_margin{MARGIN:.2f}m"
                f"_mu{MU_PROFILE:.2f}.csv")

    traj_sp = None
    sp_name = (
        f"{m}_sp_max{V_MAX:.2f}ms_margin{MARGIN:.2f}m_mu{MU_PROFILE:.2f}.csv"
        if RACELINE == "iqp" else None)
    sp_disabled_reason = ""
    if RACELINE == "iqp":
        if MAKE_SP:
            print("      SP 추월 라인 생성 ...")
            try:
                geo_sp = roll_to_start(
                    optimize_raceline_iqp(
                        center, grid, objective="length"),
                    orientation=orientation)
                traj_sp = build_trajectory(geo_sp, mu=MU_PROFILE)
                print(f"      경로 길이: IQP {traj['s'][-1]:.1f} m vs "
                      f"SP {traj_sp['s'][-1]:.1f} m")
            except Exception as exc:
                sp_disabled_reason = (
                    f"SP generation failed: {type(exc).__name__}: {exc}")
                print("      [SP DISABLED] SP 생성/속도 투영 실패 — "
                      "안전한 메인 생성은 계속합니다.")
                traj_sp = None
        else:
            sp_disabled_reason = "MAKE_SP=false"

    # 2b) 검증 — 통과 못 하면 CSV를 만들지 않는다 ---------------------------
    # 라인을 실제 occupancy 마스크에 대고 확인하는 유일한 지점. 여기서 막지
    # 않으면 벽을 관통하는 CSV가 그대로 실차로 나간다 (2026-07-27 사고).
    print("      검증: 벽 여유 / 조향 한계 ...")
    ok, msgs = verify_line(RACELINE.upper(), traj, grid)
    for line in msgs:
        print(line)
    if not ok:
        print("\n" + "=" * 72)
        print("  메인 라인 검증 실패 — 어떤 생성 파일도 승격하지 않습니다.")
        print("  벽여유/실차 조향 한계/합성 타이어 예산을 먼저 해결해야 합니다.")
        print("  위반 지점은 outputs/%s_trajectory_FAILED.png 에 표시합니다." % m)
        print("=" * 72 + "\n")
        _save_plots(m, traj, center, outer, inner, orientation,
                    start_heading, grid=grid, suffix="_FAILED")
        raise SystemExit(1)

    if traj_sp is not None:
        try:
            ok_sp, msgs_sp = verify_line("SP  ", traj_sp, grid)
        except Exception as exc:
            ok_sp = False
            msgs_sp = [
                f"      SP: [FAIL] static validation exception "
                f"{type(exc).__name__}: {exc}"]
        for line in msgs_sp:
            print(line)
        if not ok_sp:
            failed = [line.strip() for line in msgs_sp if "[FAIL]" in line]
            sp_disabled_reason = (
                "SP static validation failed: " +
                (" | ".join(failed[:3]) if failed else "unknown violation"))
            print("      [SP DISABLED] 안전 SP를 만들지 못해 메인만 commit합니다. "
                  "동일 suffix의 기존 SP는 main보다 먼저 비활성화합니다.")
            traj_sp = None

    # 3) 폐루프 시뮬 --------------------------------------------------------
    print(f"[3/4] {LAPS}랩 시뮬 (선택된 배포 제어기 + friction circle) ...")
    sim_ok, sim_msgs, log, failed_log = simulate_controller_profiles(
        RACELINE.upper(), traj, grid, LAPS, profiles)
    for line in sim_msgs:
        print(line)
    if not sim_ok:
        print("\n  메인 폐루프 검증 실패 — 기존 생성 세트를 보존합니다.")
        _save_plots(m, traj, center, outer, inner, orientation,
                    start_heading, grid=grid, log=failed_log or log,
                    suffix="_FAILED")
        raise SystemExit(1)

    if traj_sp is not None:
        try:
            sp_sim_ok, sp_sim_msgs, _, _ = simulate_controller_profiles(
                "SP  ", traj_sp, grid, LAPS, profiles)
        except Exception as exc:
            sp_sim_ok = False
            sp_sim_msgs = [
                f"      SP: [FAIL] closed-loop exception "
                f"{type(exc).__name__}: {exc}"]
        for line in sp_sim_msgs:
            print(line)
        if not sp_sim_ok:
            failed = [
                line.strip() for line in sp_sim_msgs if "[FAIL]" in line]
            sp_disabled_reason = (
                "SP closed-loop validation failed: " +
                (" | ".join(failed[:3]) if failed else "unknown violation"))
            print("      [SP DISABLED] 폐루프 검증 실패 — 메인만 commit하고 "
                  "동일 suffix의 기존 SP를 main보다 먼저 비활성화합니다.")
            traj_sp = None

    lap_time = log["t"][-1] / max(log["laps"], 1)
    print(f"      [{log['profile_name']}] lap time {lap_time:.2f} s, "
          f"v_mean {log['v'].mean():.2f}, "
          f"v_max {log['v'].max():.2f} m/s, |lat err| max "
          f"{np.abs(log['d_err']).max():.3f} m")

    # 모든 검증 뒤에만 세트 전체를 stage+fsync하고 SP -> main 순으로 commit.
    commit_generation_set(
        m, csv_name, traj, center, walls, orientation,
        sp_name=sp_name, sp_traj=traj_sp,
        sp_disabled_reason=sp_disabled_reason)

    # 4) 플롯 ---------------------------------------------------------------
    print("[4/4] 플롯 저장 ...")
    _save_plots(m, traj, center, outer, inner, orientation, start_heading,
                grid=grid, log=log)

    print(f"\n완료. outputs/{m}_trajectory.png 확인. "
          f"CSV → config/global_trajectory/{csv_name}")


if __name__ == "__main__":
    main()
