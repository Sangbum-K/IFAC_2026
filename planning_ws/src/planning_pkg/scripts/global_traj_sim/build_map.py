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
MU         = 0.75    # 노면 마찰계수    (실내/접지 낮으면 더 낮춰라 0.6~0.7)
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
# 2026-07-28: my_track0728 은 0.40 에서 R=0.86 (한계 0.88) 로 FAIL → 0.32 로 내림.
#   이 맵의 좌상 헤어핀은 0727 보다 훨씬 조인다 (코리도 폭 0.90 m vs 2.00 m,
#   중앙선 R 0.30 m vs 0.52 m) — 그래서 통과하는 SAFE_DIST 창이 좁다:
#     SAFE_DIST  벽여유min  R_min   판정      시뮬 차체침투(우측 인필드 코너)
#       0.40      0.300     0.86    FAIL      —
#       0.36      0.275     0.87    FAIL      0.125
#       0.32      0.275     0.88    PASS      0.150   <- 통과하는 최대값
#       0.30      0.225     0.92    PASS      0.175
#       0.28      0.225     1.03    PASS      0.200
#   R 을 살리려고 내리면 벽여유가 같이 깎이므로 0.32 가 상한이다.
#   ⚠️ '시뮬 차체침투'는 verify_line 이 보지 않는 값이다(레퍼런스 라인만 검사).
#   폐루프 시뮬 경로를 마스크에 대고 재면 xy≈(3.2,2.6) 인필드 코너에서 차체가
#   0.15 m 파고든다. 참고로 my_track0727(0.40)=0.05, my_track(0.40)=0.025 m.
#   V_MAX 를 6.0→3.0 으로 낮춰도 0.175→0.075 로만 줄어 해소되지 않는다.
#   → 0728 로 고속주행하기 전에 좌상 헤어핀을 완만/넓게 다시 깔 것.
SAFE_DIST  = 0.40    # IQP 라인-벽 최소 이격 [m] = 차 반폭(≈0.15) + 추적오차 여유
# 맵마다 최협부/헤어핀이 달라 최적값이 다르다. 여기 없는 맵은 위 SAFE_DIST 를 쓴다.
# ⚠️ 새 맵을 추가하면 반드시 스윕(0.28~0.45)을 돌려 R 이 통과하는 최대값을 넣을 것.
SAFE_DIST_BY_MAP = {
    "my_track0728": 0.32,   # 0.40 이면 R=0.86 FAIL. 위 2026-07-28 표 참조
    "my_track0729": 0.32,   # 아래 2026-07-29 표 참조 (SP_CURV_W 15 와 세트)
}
# 2026-07-29: my_track0729 스윕. IQP 는 R, SP 는 벽여유가 걸리는데 두 제약이
#   SAFE_DIST 에 반대로 반응해서 기본 SP_CURV_W(2.0)로는 통과하는 값이 없다:
#     SAFE_DIST  IQP벽여유 IQP_R   SP벽여유  판정
#       0.45      0.250    0.68    0.075     FAIL (IQP R)
#       0.40      0.225    0.85    0.025     FAIL (양쪽)
#       0.36      0.200    0.82    0.000     FAIL (양쪽)
#       0.34      0.150    0.91   -0.050     FAIL (SP 가 벽 안쪽)
#       0.32      0.200    0.93   -0.025     FAIL (SP 가 벽 안쪽)
#       0.28      0.200    0.95   -0.050     FAIL (SP 가 벽 안쪽)
#   IQP 는 0.32 에서 벽 0.200 / R 0.93 으로 가장 좋은데, SP 가 s≈20.6 m
#   xy=(-7.5,0.9) 헤어핀에서 코너를 깎아 벽을 0.025~0.075 m 파고든다.
#   → SP_CURV_W 를 이 맵만 15 로 올려 해결 (0.32 고정, SP 벽여유):
#     SP_CURV_W  2.0=-0.025  4.0=0.100  6.0=0.150  15=0.175  25=0.300
#   최종 0.32 + SP_CURV_W 15 → IQP 벽 0.200/R 0.93, SP 벽 0.175/R 0.99 PASS.
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
SP_CURV_W  = 2.0
# 좁은 헤어핀에서 SP 가 코너를 깎아 벽을 파고드는 맵만 여기서 올린다.
# (SAFE_DIST 를 올리면 SP 벽여유는 살지만 IQP 의 R 이 같이 죽어서 못 쓴다 —
#  두 제약이 SAFE_DIST 에 반대로 반응하므로 SP 는 곡률 가중치로 따로 잡는다.)
SP_CURV_W_BY_MAP = {
    "my_track0729": 15.0,   # 2.0 이면 s≈20.6 m 헤어핀에서 벽여유 -0.025 m
}
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
VERIFY_STRICT  = True    # False로 두면 경고만 하고 CSV를 쓴다 (권장하지 않음)

# --- 차량/컨트롤러 파라미터 (실차 실측값, 트랙 크기와 무관) ------------------
# ⚠️ 이 두 값은 출력 CSV(라인 형상·속도 프로파일)에 영향이 없다. 레이스라인은
#    optimize_raceline_iqp(기하 QP) + build_trajectory(곡률·mu)만으로 정해진다.
#    아래 두 값은 PurePursuitSim 폐루프 검증에만 쓰여 "실차 조향 한계로 이 라인을
#    실제로 따라갈 수 있는가"(lat err)를 판정한다 → 반드시 실차값이어야 의미 있음.
WHEELBASE  = 0.33    # [m] 실차 실측 (hardware vesc.yaml wheelbase와 동일)
MAX_STEER  = 0.36    # [rad] (20.7°) 실차 검증 조향 한계. TUNING_STATUS 기준이며
                     # control_pkg vehicle/max_steering_angle_rad와 동일하게 유지할 것
                     # (순정 링키지 명목 24°=0.4189가 아니라 실측 포화 한계값)
A_ACC_MAX  = 5.5     # 최대 가속 [m/s²]
A_DEC_MAX  = 7.0     # 최대 제동 [m/s²]
N_LOCAL_WP = 80      # 로컬 윈도우 웨이포인트 수 (local_trajectory_node와 동일)

# --- 이산화 간격 -------------------------------------------------------------
DS_OUT     = 0.1     # 출력 웨이포인트 간격 [m]
LAPS       = 1       # 시뮬 랩 수
# ============================================================================

G = 9.81

HERE = Path(__file__).resolve().parent
PKG_ROOT = next((p for p in HERE.parents if p.name == "planning_pkg"),
                HERE.parent.parent)
# 맵을 찾을 폴더 (앞에서부터 순서대로 — 먼저 있는 걸 쓴다):
#   1) ~/IFAC_ws/maps        — mapping.launch.py 가 저장하는 워크스페이스 공용 폴더.
#      여기서 먼저 찾으므로 매핑 직후 복사 없이 바로 build_map 을 돌릴 수 있다.
#   2) planning_pkg/maps     — 패키지에 함께 두는 맵 (기존 동작).
MAPS_SEARCH_DIRS = [Path.home() / "IFAC_ws" / "maps", PKG_ROOT / "maps"]
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
        x = origin_x + (col + 0.5) * res
        y = origin_y + (H - row - 0.5) * res
    0.5 셀을 빼먹으면 트랙 모델 전체가 PF가 정합하는 occupancy grid 대비
    (-res/2, +res/2) 만큼 대각선으로 밀린다(0.05 m/px에서 3.5 cm).
    """
    rc = np.atleast_2d(rc)
    x = origin[0] + (rc[:, 1] + 0.5) * resolution
    y = origin[1] + (H - rc[:, 0] - 0.5) * resolution
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

    def __init__(self, ring, resolution, origin, H):
        self.ring = ring
        self.res = resolution
        self.origin = np.asarray(origin, dtype=float)
        self.H = H

    def is_free(self, xy):
        """월드 좌표 (N,2) → 그 셀이 주행 가능한가 (bool N). _px_to_world의 역."""
        xy = np.atleast_2d(xy)
        col = (xy[:, 0] - self.origin[0]) / self.res - 0.5
        row = self.H - 0.5 - (xy[:, 1] - self.origin[1]) / self.res
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
    반환: (outer, inner) 월드 좌표 경계 폴리라인, resolution, origin.
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
    origin = np.array(meta["origin"][:2], dtype=float)

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

    # --- 자동 정리: infield 오목한 주머니 메우기 (중앙선 자기교차 방지) -------
    if INFIELD_FILL_M > 0:
        close_it = max(1, int(round(INFIELD_FILL_M / resolution)))
        filled = ndimage.binary_fill_holes(ring)
        infield = filled & ~ring
        infield_c = ndimage.binary_closing(
            infield, structure=np.ones((3, 3)), iterations=close_it)
        infield_c = ndimage.binary_fill_holes(infield_c) & filled
        ring = filled & ~infield_c
        filled_new_holes = int((infield_c & ~infield).sum())
        if filled_new_holes > 0:
            print(f"      infield 정리: 오목한 곳 {filled_new_holes} px "
                  f"메움 (close {close_it} px = {INFIELD_FILL_M:.1f} m)")

    # --- SLAM 스페클 제거: infield 안의 아주 작은 구멍은 벽으로 메운다 ----------
    # (몇 px짜리 free 웅덩이가 medial-axis 골격에 가짜 가지를 만든다)
    if MIN_ISLAND_M2 > 0:
        min_px = int(round(MIN_ISLAND_M2 / (resolution * resolution)))
        holes = ndimage.binary_fill_holes(ring) & ~ring
        hl, hn = ndimage.label(holes)
        n_speckle = 0
        for lab in range(1, hn + 1):
            m = hl == lab
            if m.sum() < min_px:
                ring[m] = True
                n_speckle += 1
        if n_speckle > 0:
            print(f"      스페클 제거: 작은 구멍 {n_speckle}개 메움 "
                  f"(< {MIN_ISLAND_M2:.2f} m^2)")

    # --- 참고용 크기 확인 (스케일은 안 바꿈, 정상 범위인지 경고만) ----------
    # 트랙이 기울어져 있을 수 있으므로 PCA(주축)로 긴 변/짧은 변 실측.
    rows, cols = np.nonzero(ring)
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
    cs = plt.contour(ring.astype(float), levels=[0.5])
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
    # outer, inner, resolution, origin, 트랙 마스크(ring), 이미지 높이(H)
    return polys[0], polys[1], resolution, origin, ring, H


def extract_centerline(grid, smooth=3.0, ds=DS_OUT):
    """트랙 마스크의 medial-axis(거리변환 중심선) → 주기 스플라인 → 균일 리샘플.

    예전엔 안·바깥 경계 중점으로 중앙선을 잡았는데, 인필드가 아웃필드 벽까지
    뻗는 '게이트'에서 중점이 한쪽 벽으로 붕괴해 곡률이 튀었다(R~0.3m 스파이크).
    medial-axis는 항상 코리도 한가운데를 지나 게이트에서도 안 튄다.
    폭(d_left/d_right)만 물리 경계(outer/inner)에서 부호 있는 거리로 계산한다.
    """
    loop_rc = _order_loop(_medial_axis_loop(grid.ring))      # (row, col) 순서
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


def velocity_profile(s, kappa, v_max, mu, a_acc_max, a_dec_max, n_iters=3):
    kappa = np.abs(kappa)
    n = len(s)
    ds = np.diff(s, append=s[-1] + (s[-1] - s[-2]))
    a_lat_max = mu * G
    with np.errstate(divide="ignore"):
        v = np.minimum(np.sqrt(a_lat_max / np.maximum(kappa, 1e-9)), v_max)
    for _ in range(n_iters):
        for i in range(n):
            j = (i + 1) % n
            a_lat = v[i] ** 2 * kappa[i]
            a_avail = a_acc_max * np.sqrt(max(0.0, 1.0 - (a_lat / a_lat_max) ** 2))
            v[j] = min(v[j], np.sqrt(v[i] ** 2 + 2.0 * a_avail * ds[i]))
        for i in range(n - 1, -1, -1):
            j = (i + 1) % n
            a_lat = v[j] ** 2 * kappa[j]
            a_avail = a_dec_max * np.sqrt(max(0.0, 1.0 - (a_lat / a_lat_max) ** 2))
            v[i] = min(v[i], np.sqrt(v[j] ** 2 + 2.0 * a_avail * ds[i]))
    return v


def build_trajectory(geo, v_max=V_MAX, mu=MU, margin=MARGIN):
    s, kappa = geo["s"], geo["kappa"]
    v = velocity_profile(s, kappa, v_max, mu, A_ACC_MAX, A_DEC_MAX)
    ds = np.diff(s, append=s[-1] + (s[-1] - s[-2]))
    ax = (np.roll(v, -1) ** 2 - v ** 2) / (2.0 * ds)
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


def verify_line(name, traj, grid, half_width=CAR_HALF_WIDTH,
                curvature_strict=True):
    """최종 라인이 실제로 주행 가능한지 occupancy 마스크에 대고 검사.

    ⚠️ 이 검사는 생략하면 안 된다. 2026-07-27 이전에는 어느 단계도 결과를
    맵에 대고 확인하지 않아서, 벽을 관통하는 라인이 CSV로 그대로 나가
    고속주행 테스트에 쓰였다 (my_track0727, 좌측 헤어핀 벽여유 0.016 m).

    두 가지를 본다:
      1. 벽 여유 >= 차 반폭 — 못 지키면 그 지점에서 차체가 벽에 닿는다.
      2. 요구 회전반경 >= 실차 최소 회전반경 — 못 지키면 조향이 포화해
         라인을 아예 못 따라간다 (핸들을 끝까지 꺾어도 모자람).

    curvature_strict=False 는 SP 추월 라인 전용이다. SP는 최단거리라 코너를
    깎아 R이 떨어지는데(SP_CURV_W로 완화해도 남는다), spliner는 SP 차선을
    ot_max_kappa(0.4 = R>=2.5 m) 로 걸러낸 추월 섹터 안에서만 탄다. 실제로
    my_track0727의 SP R 위반 8점은 전부 |kappa| 0.51~0.87 구간, 즉 섹터에서
    배제되는 코너였다. 여기서 SP를 하드 실패시키면 곡률을 맞추려다 SP가
    IQP와 겹쳐버려(간격 0.04 m) 추월 차선 자체가 사라진다.
    반환: (ok, 진단 문자열 리스트)
    """
    clear = grid.clearance(traj["xy"], traj["psi"])
    r_min_car = WHEELBASE / np.tan(MAX_STEER)
    kappa = np.abs(traj["kappa"])
    r_line = 1.0 / np.maximum(kappa, 1e-9)

    tight = np.flatnonzero(clear < half_width)
    sharp = np.flatnonzero(r_line < r_min_car)
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
    lvl = "FAIL" if curvature_strict else "warn"
    for g in spans(sharp)[:5]:
        msgs.append(f"        [{lvl}] R={r_line[g].min():.2f} m @ "
                    f"s={traj['s'][g[0]]:.1f}~{traj['s'][g[-1]]:.1f} m "
                    f"xy=({traj['xy'][g[0], 0]:.2f},{traj['xy'][g[0], 1]:.2f})"
                    f" — 조향 포화, {len(g)}점")
    if len(sharp) and not curvature_strict:
        msgs.append("        ↑ SP는 추월 섹터(ot_max_kappa) 안에서만 타므로 "
                    "코너의 R 위반은 실주행에 안 쓰인다. 다만 위 구간이 "
                    "직선부라면 SP_CURV_W 를 올릴 것.")
    return len(tight) == 0 and (len(sharp) == 0 or not curvature_strict), msgs


def write_csv(traj, path, header_note=""):
    cols = np.column_stack([
        traj["s"], traj["xy"][:, 0], traj["xy"][:, 1],
        traj["d_right"], traj["d_left"], traj["psi"],
        traj["kappa"], traj["vx"], traj["ax"]])
    Path(path).parent.mkdir(parents=True, exist_ok=True)
    with open(path, "w") as f:
        if header_note:
            f.write(f"# {header_note}\n")
        f.write("# s_m; x_m; y_m; d_right; d_left; psi_rad; kappa_radpm; "
                "vx_mps; ax_mps2\n")
        for row in cols:
            f.write("; ".join(f"{v:.7f}" for v in row) + "\n")


# ============================================================ 3. 시뮬레이션
class PurePursuitSim:
    def __init__(self, traj, wheelbase=WHEELBASE, max_steer=MAX_STEER,
                 m_l1=0.3, q_l1=0.4, t_clip_min=0.8, t_clip_max=5.0,
                 a_acc_max=A_ACC_MAX, a_dec_max=A_DEC_MAX,
                 num_local_waypoints=N_LOCAL_WP, dt=0.01):
        self.traj = traj
        self.xy = traj["xy"]
        self.vx_ref = traj["vx"]
        self.s_ref = traj["s"]
        self.n = len(self.xy)
        self.track_len = traj["s"][-1] + np.hypot(*(self.xy[0] - self.xy[-1]))
        self.tree = cKDTree(self.xy)
        self.L = wheelbase
        self.max_steer = max_steer
        self.m_l1, self.q_l1 = m_l1, q_l1
        self.t_clip = (t_clip_min, t_clip_max)
        self.a_acc_max, self.a_dec_max = a_acc_max, a_dec_max
        self.n_local = num_local_waypoints
        self.dt = dt

    def local_window(self, idx):
        return (idx + np.arange(self.n_local)) % self.n

    def lookahead_point(self, pos, idx, l1):
        win = self.local_window(idx)
        d = np.hypot(*(self.xy[win] - pos).T)
        ahead = np.nonzero(d >= l1)[0]
        j = win[ahead[0]] if len(ahead) else win[-1]
        return self.xy[j], j

    def run(self, n_laps=1, max_time=300.0):
        pos = self.xy[0].copy()
        theta = float(self.traj["psi"][0])
        v = 0.5
        s_travel = 0.0
        prev_s = 0.0
        log = {k: [] for k in ("t", "x", "y", "v", "v_ref", "s", "d_err", "steer")}
        t = 0.0
        laps_done = 0
        while t < max_time:
            _, idx = self.tree.query(pos)
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
            l1 = np.clip(self.m_l1 * v + self.q_l1, *self.t_clip)
            target, j = self.lookahead_point(pos, idx, l1)
            dx, dy = target - pos
            alpha = np.arctan2(dy, dx) - theta
            alpha = np.arctan2(np.sin(alpha), np.cos(alpha))
            ld = max(np.hypot(dx, dy), 1e-3)
            steer = np.clip(np.arctan2(2.0 * self.L * np.sin(alpha), ld),
                            -self.max_steer, self.max_steer)
            v_cmd = self.vx_ref[j]
            dv = np.clip(v_cmd - v, -self.a_dec_max * self.dt,
                         self.a_acc_max * self.dt)
            v = max(v + dv, 0.0)
            theta += v / self.L * np.tan(steer) * self.dt
            pos = pos + v * self.dt * np.array([np.cos(theta), np.sin(theta)])
            t += self.dt
            s_travel += v * self.dt
            log["t"].append(t)
            log["x"].append(pos[0]); log["y"].append(pos[1])
            log["v"].append(v); log["v_ref"].append(self.vx_ref[idx])
            log["s"].append(s_now + laps_done * self.track_len)
            log["d_err"].append(d_err); log["steer"].append(steer)
        return {k: np.asarray(v) for k, v in log.items()} | {
            "laps": laps_done, "track_len": self.track_len}


# ============================================================ 4. 오케스트레이션
def _save_plots(m, traj, center, outer, inner, orientation, start_heading,
                grid=None, log=None, suffix=""):
    """궤적/추종 그림 저장. 검증 실패 시에도 (log 없이) 진단용으로 부른다."""
    lap_time = (log["t"][-1] / max(log["laps"], 1)) if log else float("nan")
    fig, ax = plt.subplots(figsize=(11, 8))
    ax.plot(*outer.T, "k-", lw=1); ax.plot(*inner.T, "k-", lw=1)
    if RACELINE == "iqp":
        ax.plot(center["xy"][:, 0], center["xy"][:, 1], "--", color="gray",
                lw=0.8, alpha=0.7, label="centerline")
    pts = ax.scatter(traj["xy"][:, 0], traj["xy"][:, 1], c=traj["vx"],
                     cmap="viridis", s=4, label=f"{RACELINE} raceline (vx)")
    if log is not None:
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
    head = (f"global trajectory & lap ({lap_time:.2f} s/lap)" if log
            else "VERIFICATION FAILED -- no CSV written")
    ax.set_title(f"{m}: {head}\n"
                 f"travel dir {orientation.upper()}, start heading "
                 f"{np.degrees(start_heading):+.1f} deg "
                 f"-- check against real car placement")
    ax.set_xlabel("x [m]"); ax.set_ylabel("y [m]")
    fig.tight_layout()
    fig.savefig(OUT_DIR / f"{m}_trajectory{suffix}.png", dpi=150)
    plt.close(fig)
    if log is None:
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
    OUT_DIR.mkdir(parents=True, exist_ok=True)
    safe_dist = SAFE_DIST_BY_MAP.get(m, SAFE_DIST)
    if safe_dist != SAFE_DIST:
        print(f"      [cfg] SAFE_DIST {safe_dist:.2f} m "
              f"(이 맵 전용 오버라이드, 기본값 {SAFE_DIST:.2f})")
    # SP_CURV_W 는 _solve_qp 가 모듈 전역으로 읽으므로 여기서 갈아끼운다.
    # 진입점이 main() 하나뿐이라 이 방식이 안전하다.
    global SP_CURV_W
    if m in SP_CURV_W_BY_MAP:
        SP_CURV_W = SP_CURV_W_BY_MAP[m]
        print(f"      [cfg] SP_CURV_W {SP_CURV_W:.1f} "
              f"(이 맵 전용 오버라이드, 기본값 2.0)")

    # 1) 트랙 추출 (+ 실측 크기 스케일 보정) --------------------------------
    print(f"[1/4] '{m}' 로드 + 트랙 추출 (크기는 yaml resolution 기준) ...")
    outer, inner, resolution, origin, ring, H = load_track(m)
    grid = Grid(ring, resolution, origin, H)
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
        geo = roll_to_start(optimize_raceline_iqp(center, grid,
                                                  safe_dist_m=safe_dist),
                            orientation=orientation)
    elif RACELINE == "sp":
        print("[2/4] SP 최단거리 라인 + 속도 프로파일 ...")
        geo = roll_to_start(optimize_raceline_iqp(center, grid,
                                                  safe_dist_m=safe_dist,
                                                  objective="length"),
                            orientation=orientation)
    else:
        # ⚠️ 이 모드는 검증을 거의 못 통과한다 (설정 블록 주석 참조).
        print("[2/4] 중앙선 라인 + 속도 프로파일 ...")
        geo = center
    traj = build_trajectory(geo)
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
    csv_name = f"{m}_{tag}max{V_MAX:.2f}ms_margin{MARGIN:.2f}m_mu{MU:.2f}.csv"

    traj_sp = sp_name = None
    if RACELINE == "iqp" and MAKE_SP:
        print("      SP 추월 라인 생성 ...")
        geo_sp = roll_to_start(optimize_raceline_iqp(center, grid,
                                                     safe_dist_m=safe_dist,
                                                     objective="length"),
                               orientation=orientation)
        traj_sp = build_trajectory(geo_sp)
        sp_name = f"{m}_sp_max{V_MAX:.2f}ms_margin{MARGIN:.2f}m_mu{MU:.2f}.csv"
        print(f"      경로 길이: IQP {traj['s'][-1]:.1f} m vs "
              f"SP {traj_sp['s'][-1]:.1f} m")

    # 2b) 검증 — 통과 못 하면 CSV를 만들지 않는다 ---------------------------
    # 라인을 실제 occupancy 마스크에 대고 확인하는 유일한 지점. 여기서 막지
    # 않으면 벽을 관통하는 CSV가 그대로 실차로 나간다 (2026-07-27 사고).
    print("      검증: 벽 여유 / 조향 한계 ...")
    ok, msgs = verify_line(RACELINE.upper(), traj, grid)
    if traj_sp is not None:
        ok_sp, msgs_sp = verify_line("SP  ", traj_sp, grid,
                                     curvature_strict=False)
        ok, msgs = ok and ok_sp, msgs + msgs_sp
    for line in msgs:
        print(line)
    if not ok:
        print("\n" + "=" * 72)
        print("  검증 실패 — CSV를 쓰지 않았습니다. 이 라인으로 주행하면 안 됩니다.")
        print("  [벽여유] 위반이면 SAFE_DIST 를 올린다 (현재 %.2f m). 올려도"
              % safe_dist)
        print("           안 늘면 트랙 최협부가 실제로 차폭+여유보다 좁은 것 →")
        print("           맵을 다시 따거나 코스를 넓혀야 한다.")
        print("  [R] 위반이면 SAFE_DIST 를 **내린다**. 값이 클수록 좁은 구간에서")
        print("      라인이 중앙으로 밀려 코너 반경이 같이 줄기 때문이다 —")
        print("      올리는 건 역효과다. 내려도 R<%.2f m 이면 코스 자체가 실차"
              % (WHEELBASE / np.tan(MAX_STEER)))
        print("      조향 한계를 넘는 것이라 헤어핀을 완만하게 다시 깔아야 한다.")
        print("      (V_MAX/MU 를 만져도 R 은 안 바뀐다 — 기하 문제다)")
        print("  위반 지점은 빨간 X 로 표시했습니다 → "
              "outputs/%s_trajectory_FAILED.png" % m)
        print("=" * 72 + "\n")
        if VERIFY_STRICT:
            _save_plots(m, traj, center, outer, inner, orientation,
                        start_heading, grid=grid, suffix="_FAILED")
            raise SystemExit(1)

    for dest in (OUT_DIR / csv_name, CSV_DIR / csv_name):
        write_csv(traj, dest, header_note=f"build_map ({m}, dir={orientation.upper()}, "
                                          f"safe_dist={safe_dist:.2f}m)")
        print(f"      wrote {dest}")
    if traj_sp is not None:
        for dest in (OUT_DIR / sp_name, CSV_DIR / sp_name):
            write_csv(traj_sp, dest,
                      header_note=f"build_map ({m}, SP overtake, "
                                  f"dir={orientation.upper()}, safe_dist={safe_dist:.2f}m)")
    for dest in (OUT_DIR / f"{m}_walls.csv", CENTERLINE_DIR / f"{m}_walls.csv"):
        Path(dest).parent.mkdir(parents=True, exist_ok=True)
        np.savetxt(dest, walls, delimiter="; ", fmt="%.7f",
                   header="x_m; y_m; wall_id(0=outer,1=inner)")
    for dest in (OUT_DIR / f"{m}_centerline.csv",
                 CENTERLINE_DIR / f"{m}_centerline.csv"):
        Path(dest).parent.mkdir(parents=True, exist_ok=True)
        np.savetxt(dest, center["xy"], delimiter="; ", fmt="%.7f",
                   header="x_m; y_m")

    # 3) 폐루프 시뮬 --------------------------------------------------------
    print(f"[3/4] {LAPS}랩 시뮬 (pure pursuit + 자전거 모델) ...")
    log = PurePursuitSim(traj).run(n_laps=LAPS)
    lap_time = log["t"][-1] / max(log["laps"], 1)
    print(f"      lap time {lap_time:.2f} s, v_mean {log['v'].mean():.2f}, "
          f"v_max {log['v'].max():.2f} m/s, |lat err| max "
          f"{np.abs(log['d_err']).max():.3f} m")

    # 4) 플롯 ---------------------------------------------------------------
    print("[4/4] 플롯 저장 ...")
    _save_plots(m, traj, center, outer, inner, orientation, start_heading,
                grid=grid, log=log)

    print(f"\n완료. outputs/{m}_trajectory.png 확인. "
          f"CSV → config/global_trajectory/{csv_name}")


if __name__ == "__main__":
    main()
