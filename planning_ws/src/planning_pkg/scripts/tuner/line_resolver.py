#!/usr/bin/env python3
"""웹 레이스라인 튜너의 물리·CSV 계약 계층 (ROS 없음).

역할 분담
---------
`build_map.py` 가 맵만 보고 만든 **원본** 라인이 초안이고, 사람은 실차를 몰아 본 뒤
"이 코너는 그립이 남았다 / 이 직선은 더 밟아도 된다"를 안다. 이 모듈은 그 판단을
**깎지 않고** 라인에 새기는 쪽이다:

  - 속도는 사람이 넣은 값이 그대로 들어간다. 마찰원/가감속으로 되풀지 않는다.
  - 보라 앵커는 **등호 제약**이다. 코리도로 클램프하지 않는다.
  - 앵커가 아닌 구간만 최소곡률로 다시 풀어 전체 라인을 매끄럽게 잇는다.

물리 모델은 판정이 아니라 **보고**에만 쓴다 (`RacelineState.report()`). 무엇이
한계를 넘었는지는 화면에 빨갛게 보여 주고, 저장은 막지 않는다 — 막을 자리는
`global_trajectory_node` 의 `validation_mode` 이고, 튜닝 라인은 advisory 로 뜬다.

기준 그리드는 **원본 라인** 이며 절대 바뀌지 않는다. 편집 상태는 그 그리드 위의
    xy = xy0 + alpha * normal0
로 표현되므로, 앵커를 왔다 갔다 끌어도 값이 누적 드리프트하지 않고 alpha=0 이
항상 정확히 원본이다.

의존성은 numpy / scipy / yaml / PIL 뿐이다 (시스템 ROS python 에 다 있다).
skimage·matplotlib 가 필요한 `build_map.py` 를 import 하지 않는 이유이기도 하다 —
대신 거기 있는 상수는 AST 로 읽어 사본을 만들지 않는다.
"""

import ast
import hashlib
import io
import json
import math
import os
import sys
import tempfile
import uuid
from pathlib import Path

import numpy as np
import yaml
from PIL import Image
from scipy import ndimage
from scipy.interpolate import splev, splprep
from scipy.optimize import lsq_linear

# scripts/ 를 import 경로에 넣는다. 이 모듈은 웹 서버가 파일 경로로 직접 로드하므로
# 패키지 import 가 아니다 (build_map.py / controller_safety_model.py 와 같은 방식).
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from deployment_profiles import (            # noqa: E402
    A_ACC_MAX, A_DEC_PLANNED, MAX_STEER, PKG_ROOT, WHEELBASE, WORKSPACE_ROOT,
)

G = 9.81

GENERATOR_PATH = PKG_ROOT / "scripts" / "global_traj_sim" / "build_map.py"
CSV_DIR = PKG_ROOT / "config" / "global_trajectory"
CENTERLINE_DIR = PKG_ROOT / "config" / "centerline"
MAPS_SEARCH_DIRS = (WORKSPACE_ROOT / "maps", PKG_ROOT / "maps")

# build_map.py 에서 AST 로 읽어 오는 상수. 여기 숫자를 적지 않는 이유는
# 2026-08-05 리뷰에서 같은 매니페스트가 세 벌이었고 전부 달랐기 때문이다.
_GENERATOR_CONSTANTS = (
    "CAR_HALF_WIDTH", "CAR_HALF_LENGTH", "MU", "MU_PROFILE", "DS_OUT",
)


# ======================================================== 생성기 상수 (AST)
def _eval_restricted(node, namespace):
    """리터럴 / 이미 읽은 이름 / +-*/ 만 계산한다. import 없이 상수만 꺼내려는 것."""
    if isinstance(node, ast.Constant):
        if isinstance(node.value, bool) or not isinstance(
                node.value, (int, float)):
            raise ValueError("숫자 상수가 아님")
        return float(node.value)
    if isinstance(node, ast.Name):
        if node.id not in namespace:
            raise ValueError(f"아직 읽지 않은 이름: {node.id}")
        return namespace[node.id]
    if isinstance(node, ast.UnaryOp) and isinstance(
            node.op, (ast.UAdd, ast.USub)):
        value = _eval_restricted(node.operand, namespace)
        return value if isinstance(node.op, ast.UAdd) else -value
    if isinstance(node, ast.BinOp):
        left = _eval_restricted(node.left, namespace)
        right = _eval_restricted(node.right, namespace)
        if isinstance(node.op, ast.Add):
            return left + right
        if isinstance(node.op, ast.Sub):
            return left - right
        if isinstance(node.op, ast.Mult):
            return left * right
        if isinstance(node.op, ast.Div):
            return left / right
    raise ValueError(f"지원하지 않는 식: {type(node).__name__}")


def generator_constants(path=GENERATOR_PATH, names=_GENERATOR_CONSTANTS):
    """build_map.py 의 스칼라 상수를 이름->값으로 읽는다.

    import 하지 않는 이유: build_map 은 skimage/matplotlib 가 있는 맵 생성 venv 용
    이고 이 모듈은 시스템 ROS python 에서 돈다. 값을 베껴 적지 않는 이유:
    베껴 적는 순간 드리프트가 시작되고, 그게 이 스택에서 실제로 몇 주 동안
    보이지 않은 채로 있었다.
    """
    path = Path(path)
    if not path.is_file():
        raise FileNotFoundError(f"생성기를 찾을 수 없음: {path}")
    tree = ast.parse(path.read_text(encoding="utf-8"), filename=str(path))
    wanted = set(names)
    namespace = {}
    for statement in tree.body:
        targets = (
            statement.targets if isinstance(statement, ast.Assign)
            else [statement.target] if isinstance(statement, ast.AnnAssign)
            else [])
        for target in targets:
            if not isinstance(target, ast.Name):
                continue
            try:
                namespace[target.id] = _eval_restricted(
                    statement.value, namespace)
            except ValueError:
                # 이름/식이 우리 범위를 벗어나는 대입은 그냥 건너뛴다. 필요한
                # 이름이 그래서 안 잡히면 아래에서 확실하게 죽는다.
                continue
    missing = wanted - set(namespace)
    if missing:
        raise ValueError(f"{path.name} 에서 상수를 못 읽음: {sorted(missing)}")
    values = {name: namespace[name] for name in wanted}
    for name, value in values.items():
        if not math.isfinite(value) or value <= 0.0:
            raise ValueError(f"{name} 이 양의 유한값이 아님: {value}")
    return values


# ======================================================== 트랙 맵
class TrackMap:
    """맵 이미지 → 주행 가능 마스크 + 월드↔픽셀 변환.

    **build_map.py 의 `Grid` 와 같은 의미**로 만든다. 예전 RViz 튜너는 map_server 가
    발행한 OccupancyGrid 를 받아 `0 <= v < 65` 를 free 로 봤는데, 그건 yaml 의
    `free_thresh` 를 쓰는 생성기와 미묘하게 다른 기준이었다. 여기서는 생성기와
    똑같이 이미지에서 직접 읽으므로 두 도구가 벽을 동일하게 잰다 (그리고
    map_server / lifecycle_manager 를 띄울 필요가 없어진다).
    """

    def __init__(self, yaml_path):
        yaml_path = Path(yaml_path)
        meta = yaml.safe_load(yaml_path.read_text(encoding="utf-8"))
        if not isinstance(meta, dict) or "image" not in meta:
            raise ValueError(f"맵 yaml 이 이상함: {yaml_path}")
        image = np.array(
            Image.open(yaml_path.parent / meta["image"]).convert("L"),
            dtype=float)
        if meta.get("negate", 0):
            image = 255.0 - image
        occupancy = (255.0 - image) / 255.0        # ROS map_server 관례
        free = occupancy < float(meta.get("free_thresh", 0.196))
        self.res = float(meta["resolution"])
        origin = meta.get("origin")
        if not isinstance(origin, (list, tuple)) or len(origin) < 3:
            raise ValueError("맵 yaml origin 이 [x, y, yaw] 가 아님")
        self.origin = np.asarray(origin[:3], dtype=float)
        if not np.isfinite(self.origin).all() or not math.isfinite(self.res) \
                or self.res <= 0.0:
            raise ValueError("맵 resolution/origin 이 유한하지 않음")
        self.H, self.W = free.shape
        self.name = yaml_path.stem
        self.yaml_path = yaml_path
        self.ring = self._select_ring(free)
        # 화면 배경용 원본 free/occupied 구분 (ring 밖도 그려야 사람이 맵을 알아본다)
        self.free_raw = free

    @staticmethod
    def _select_ring(free):
        """가운데 구멍이 뚫린 free 연결성분(= 트랙 고리)만 남긴다.

        build_map.load_track 과 같은 판정이다. free 마스크를 그대로 쓰면 맵 바깥
        배경까지 '주행 가능'이 되어 바깥 벽 근처 footprint 검사가 통과해 버린다.
        """
        labels, count = ndimage.label(free)
        if count == 0:
            raise RuntimeError("free 영역이 없음")
        sizes = ndimage.sum(free, labels, range(1, count + 1))
        for label in (1 + np.argsort(sizes)[::-1]):
            component = labels == label
            touches_border = (
                component[0, :].any() or component[-1, :].any() or
                component[:, 0].any() or component[:, -1].any())
            if touches_border:                 # 맵 바깥 배경
                continue
            filled = ndimage.binary_fill_holes(component)
            if filled.sum() - component.sum() > component.sum() * 0.02:
                return component
        raise RuntimeError(
            "닫힌 고리(트랙) 모양의 free 영역을 못 찾음 — 실내 corridor 맵이거나 "
            "트랙이 안 닫혔습니다")

    def is_free(self, xy):
        """월드 (N,2) → 그 셀이 주행 가능한가. build_map.Grid.is_free 와 동일 규약."""
        xy = np.atleast_2d(np.asarray(xy, dtype=float))
        dx = xy[:, 0] - self.origin[0]
        dy = xy[:, 1] - self.origin[1]
        yaw = float(self.origin[2])
        cos_yaw, sin_yaw = math.cos(yaw), math.sin(yaw)
        local_x = cos_yaw * dx + sin_yaw * dy
        local_y = -sin_yaw * dx + cos_yaw * dy
        rows = np.round(self.H - 0.5 - local_y / self.res).astype(int)
        cols = np.round(local_x / self.res - 0.5).astype(int)
        inside = ((rows >= 0) & (rows < self.H) &
                  (cols >= 0) & (cols < self.W))
        out = np.zeros(len(xy), dtype=bool)
        out[inside] = self.ring[rows[inside], cols[inside]]
        return out

    def corridor_interval(self, xy, psi, max_d=4.0, step=None):
        """법선을 따라 스캔 → 그 점을 담고 있는 free 구간을 (a_lo, a_hi) [m] 로.

        a_lo < 0 < a_hi 면 점이 코리도 안이고 각각 우/좌 벽까지 거리다. 점이 벽
        안이면 두 끝이 같은 부호로 나온다 (= 그쪽으로 나가야 한다).
        """
        xy = np.atleast_2d(np.asarray(xy, dtype=float))
        psi = np.atleast_1d(np.asarray(psi, dtype=float))
        step = float(step) if step is not None else 0.5 * self.res
        half = int(round(max_d / step))
        alphas = np.arange(-half, half + 1) * step      # alpha=0 은 인덱스 half
        normal = np.column_stack([-np.sin(psi), np.cos(psi)])
        free = np.empty((len(xy), len(alphas)), dtype=bool)
        for index, alpha in enumerate(alphas):
            free[:, index] = self.is_free(xy + alpha * normal)
        a_lo = np.zeros(len(xy))
        a_hi = np.zeros(len(xy))
        for i in range(len(xy)):
            hit_indices = np.flatnonzero(free[i])
            if len(hit_indices) == 0:          # 법선 전체가 막힘
                continue                       # a_lo = a_hi = 0 → 이동 금지
            breaks = np.flatnonzero(np.diff(hit_indices) > 1)
            starts = np.r_[hit_indices[0], hit_indices[breaks + 1]]
            ends = np.r_[hit_indices[breaks], hit_indices[-1]]
            containing = np.flatnonzero((starts <= half) & (half <= ends))
            pick = (containing[0] if len(containing)
                    else int(np.argmin(np.minimum(
                        np.abs(starts - half), np.abs(ends - half)))))
            a_lo[i] = alphas[starts[pick]]
            a_hi[i] = alphas[ends[pick]]
        return a_lo, a_hi

    def clearance(self, xy, psi, **kwargs):
        """좌우 벽까지 거리 중 작은 쪽 [m]. 점이 벽 안이면 음수."""
        a_lo, a_hi = self.corridor_interval(xy, psi, **kwargs)
        inside = (a_lo <= 0.0) & (0.0 <= a_hi)
        nearest = np.minimum(np.abs(a_lo), np.abs(a_hi))
        return np.where(inside, nearest, -nearest)

    def footprint_free(self, xy, psi, half_length, half_width):
        """각 pose 의 직사각 차량 전체가 free 인가. 코너에서 모서리가 벽을 자르는
        경우는 중심점 거리만으로는 안 잡힌다."""
        xy = np.atleast_2d(np.asarray(xy, dtype=float))
        psi = np.atleast_1d(np.asarray(psi, dtype=float))
        step = max(0.5 * self.res, 0.01)
        longitudinal = np.arange(-half_length, half_length + 0.5 * step, step)
        lateral = np.arange(-half_width, half_width + 0.5 * step, step)
        cos_psi, sin_psi = np.cos(psi), np.sin(psi)
        ok = np.ones(len(xy), dtype=bool)
        for dl in longitudinal:
            for dd in lateral:
                ok &= self.is_free(xy + np.column_stack([
                    dl * cos_psi - dd * sin_psi,
                    dl * sin_psi + dd * cos_psi]))
        return ok


def find_map_yaml(map_name, search_dirs=MAPS_SEARCH_DIRS):
    """<맵>.yaml 을 ~/IFAC_ws/maps -> planning_pkg/maps 순으로 찾는다."""
    for directory in search_dirs:
        candidate = Path(directory) / f"{map_name}.yaml"
        if candidate.is_file():
            return candidate
    searched = ", ".join(str(d) for d in search_dirs)
    raise FileNotFoundError(
        f"맵 '{map_name}.yaml' 을 못 찾음. 찾아본 곳: {searched}")


# ======================================================== CSV 입출력
CSV_COLUMNS = ("s_m", "x_m", "y_m", "d_right", "d_left",
               "psi_rad", "kappa_radpm", "vx_mps", "ax_mps2")


def load_csv(path):
    """9열 세미콜론 CSV → (ndarray (n,9), 헤더 주석에서 뽑은 메타 dict)."""
    path = Path(path)
    rows = []
    meta = {}
    for line in path.read_text(encoding="utf-8").splitlines():
        stripped = line.strip()
        if not stripped:
            continue
        if stripped.startswith("#"):
            body = stripped.lstrip("#").strip()
            for chunk in body.split(";"):
                if "=" in chunk:
                    key, _, value = chunk.partition("=")
                    meta.setdefault(key.strip(), value.strip())
            continue
        values = [float(v) for v in stripped.split(";")]
        if len(values) != 9:
            raise ValueError(f"{path.name}: 9열이 아닌 행 ({len(values)}열)")
        rows.append(values)
    array = np.asarray(rows, dtype=float)
    if array.ndim != 2 or array.shape[1] != 9 or len(array) < 8:
        raise ValueError(f"{path.name}: 9열 웨이포인트 8개 이상이어야 함")
    if not np.isfinite(array).all():
        raise ValueError(f"{path.name}: NaN/Inf 포함")
    return array, meta


def as_dict(array):
    """(n,9) → 열 이름 dict. 계산부에서 인덱스를 세지 않게."""
    return {
        "s": array[:, 0], "xy": array[:, 1:3],
        "d_right": array[:, 3], "d_left": array[:, 4],
        "psi": array[:, 5], "kappa": array[:, 6],
        "vx": array[:, 7], "ax": array[:, 8],
    }


def tuned_csv_name(original_name, variant="tuned"):
    """'<맵>_iqp_max...csv' -> '<맵>_tuned_iqp_max...csv'.

    꼬리(_max..ms_margin..m_mu..)를 그대로 두는 이유: launch 의 glob
    `<맵>_iqp_*.csv` 가 원본만 집고 `<맵>_tuned_iqp_*.csv` 는 튜닝본만 집는다.
    파일명에서 margin 을 읽는 경로도 그대로 살아 있다.
    """
    for tag in ("_iqp_", "_sp_"):
        if tag in original_name:
            head, _, tail = original_name.partition(tag)
            if head.endswith(f"_{variant}"):
                return original_name         # 이미 튜닝본이다
            return f"{head}_{variant}{tag}{tail}"
    raise ValueError(
        f"'_iqp_' 도 '_sp_' 도 없는 파일명이라 튜닝본 이름을 만들 수 없음: "
        f"{original_name}")


def write_csv_atomic(path, columns, header_lines):
    """stage → fsync → rename → 디렉터리 fsync. 부분 기록된 CSV 를 남기지 않는다."""
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    stream = io.StringIO()
    for line in header_lines:
        stream.write(f"# {line}\n")
    stream.write("# " + "; ".join(CSV_COLUMNS) + "\n")
    for row in columns:
        stream.write("; ".join(f"{value:.7f}" for value in row) + "\n")
    payload = stream.getvalue().encode("utf-8")
    descriptor, temporary = tempfile.mkstemp(
        prefix=f".{path.name}.", suffix=".tmp", dir=path.parent)
    try:
        with os.fdopen(descriptor, "wb") as handle:
            handle.write(payload)
            handle.flush()
            os.fsync(handle.fileno())
        os.replace(temporary, path)
        flags = os.O_RDONLY | getattr(os, "O_DIRECTORY", 0)
        directory = os.open(path.parent, flags)
        try:
            os.fsync(directory)
        finally:
            os.close(directory)
    finally:
        if os.path.exists(temporary):
            os.unlink(temporary)
    return hashlib.sha256(payload).hexdigest()


# ======================================================== 기하 도우미
def respline_uniform(xy, ds_out, iterations=8):
    """폐곡선을 지나는 보간 스플라인 → 균일 간격 ds_out 으로 재샘플.

    라인을 법선 방향으로 옮기면 간격이 곡률에 비례해 찌그러진다 (안쪽으로 a 만큼
    옮기면 호 길이가 대략 (1 - kappa*a) 배). my_track0807 곡률(최대 0.79)에서
    0.5 m 만 옮겨도 간격이 0.06~0.1 m 로 벌어져 런타임의 '간격 균일'
    검사(max > min*1.25)를 못 넘는다. 그래서 저장 전에 반드시 다시 균일화한다.

    s=0 이므로 스플라인이 입력 점을 정확히 지난다 — 사람이 찍은 앵커 위치가
    재샘플로 밀리지 않는다 (재샘플 격자가 앵커에 딱 떨어지지는 않지만 곡선은
    앵커를 통과한다).
    """
    xy = np.asarray(xy, dtype=float)
    # per=1 은 '마지막 점 == 첫 점'을 기대하고, 아니면 경고를 내면서 마지막 점을
    # 첫 점으로 **덮어쓴다** (scipy _fitpack_impl: "Setting x[0][n]=x[0][0]").
    # 즉 닫는 점을 명시하지 않으면 마지막 웨이포인트가 조용히 버려진다.
    # build_map._respline_closed 도 같은 상태지만, 여기서는 명시적으로 닫는다.
    closed = np.vstack([xy, xy[:1]])
    tck, _ = splprep([closed[:, 0], closed[:, 1]], s=0.0, per=1)
    dense_u = np.linspace(0.0, 1.0, 16 * len(closed), endpoint=False)
    dense_x, dense_y = splev(dense_u, tck)
    segment = np.hypot(np.diff(dense_x, append=dense_x[0]),
                       np.diff(dense_y, append=dense_y[0]))
    length = float(segment.sum())
    count = max(int(round(length / ds_out)), 16)
    dense_s = np.concatenate([[0.0], np.cumsum(segment)])[:-1]
    uniform_u = np.interp(
        np.linspace(0.0, length, count, endpoint=False), dense_s, dense_u)

    # 위까지는 '스플라인 호장'이 균일하다. 그런데 런타임이 검사하는 것은
    # CSV 의 s 열, 즉 **현(弦) 길이** 간격이다 (`ValidateTrajectory` 의
    # `s_m` 은 build_map 도 chord cumsum 으로 만든다). 곡률이 큰 곳에서 현은
    # 호보다 짧아서, 사람이 좌우로 크게 끌어 급한 굽이가 생기면 간격이
    # 0.080 / 0.100 으로 벌어지고 균일성 검사(max <= min*1.25)를 넘겨 버린다.
    # 실측: 앵커를 3.7 m 마다 코리도 90% 로 번갈아 끌면 실제로 그렇게 된다.
    # 그러면 저장은 되는데 차가 아예 뜨지 않는다 — 사람이 트랙에서 알아채는
    # 최악의 실패다. 그래서 현 길이 기준으로 몇 번 더 균일화한다.
    # 같은 극단 편집에서 max/min: 0회 1.2504(거부) · 1회 1.3446 · 2회 1.2010
    # · 4회 1.1278 · 8회 1.0158. 1회에서 한 번 튀므로 여유를 보고 8회로 둔다.
    for _ in range(max(int(iterations), 0)):
        probe_x, probe_y = splev(uniform_u, tck)
        chord = np.hypot(np.roll(probe_x, -1) - probe_x,
                         np.roll(probe_y, -1) - probe_y)
        total = float(chord.sum())
        if total <= 0.0:
            break
        chord_s = np.concatenate([[0.0], np.cumsum(chord)])[:-1]
        # 폐루프라 마지막 구간을 넘어가는 목표점은 u[0]+1 로 감싸 보간한다.
        extended_s = np.concatenate([chord_s, [total]])
        extended_u = np.concatenate([uniform_u, [uniform_u[0] + 1.0]])
        uniform_u = np.interp(
            np.linspace(0.0, total, count, endpoint=False),
            extended_s, extended_u)

    x, y = splev(uniform_u, tck)
    dx, dy = splev(uniform_u, tck, der=1)
    ddx, ddy = splev(uniform_u, tck, der=2)
    psi = np.arctan2(dy, dx)
    kappa = (dx * ddy - dy * ddx) / np.power(dx * dx + dy * dy, 1.5)
    out_xy = np.column_stack([x, y])
    s = np.concatenate([[0.0], np.cumsum(np.hypot(np.diff(x), np.diff(y)))])
    return {"xy": out_xy, "psi": psi, "kappa": kappa, "s": s,
            "length": length}


def closed_segments(xy):
    """폐루프 각 구간 길이 (마지막 → 처음 포함)."""
    xy = np.asarray(xy, dtype=float)
    return np.hypot(np.roll(xy[:, 0], -1) - xy[:, 0],
                    np.roll(xy[:, 1], -1) - xy[:, 1])


def closed_loop_ax(speed, segment_ds):
    """v 에서 ax 를 유도한다 (끝→처음 구간 포함).

    사람이 속도를 직접 넣는 도구이므로 ax 는 사람이 신경 쓸 값이 아니다. 다만
    CSV 가 자기모순이면 런타임 로더가 형식 검사에서 거부하므로 항상 맞춰 둔다.
    """
    speed = np.asarray(speed, dtype=float)
    segment_ds = np.asarray(segment_ds, dtype=float)
    if len(speed) != len(segment_ds) or np.any(segment_ds <= 1e-6):
        raise ValueError("ax 계산 입력이 이상함")
    return (np.roll(speed, -1) ** 2 - speed ** 2) / (2.0 * segment_ds)


def interp_periodic(query_u, base_u, values):
    """폐루프 위 선형 보간. base_u/query_u 는 0~1 정규화 호장."""
    base_u = np.asarray(base_u, dtype=float)
    values = np.asarray(values, dtype=float)
    order = np.argsort(base_u)
    base_u = base_u[order]
    values = values[order]
    extended_u = np.concatenate([base_u, [base_u[0] + 1.0]])
    extended_v = np.concatenate([values, [values[0]]])
    return np.interp(np.mod(query_u, 1.0), extended_u, extended_v)


# ======================================================== 편집 상태
class RacelineState:
    """사람이 소유하는 편집 상태. 기준은 언제나 build_map 원본 라인이다.

    상태는 원본 격자 위의 세 배열뿐이다:
      alpha[n]  — 원본 법선 방향 오프셋. 현재 라인 = xy0 + alpha * normal0
      v[n]      — 속도. 사람이 넣은 값이 그대로 들어간다
      pinned[n] — 사람이 직접 정한 속도인가 (다른 편집이 덮지 않게)
    앵커(anchors: index -> alpha)는 alpha 의 **등호 제약**이다.

    기준 격자를 고정하는 이유: 현재 라인을 매번 새 기준으로 삼으면 앵커를 왔다
    갔다 끌 때 값이 누적 드리프트해서 같은 자리로 돌아오지 않는다. alpha=0 이
    항상 정확히 원본이어야 "전체 리셋"이 의미를 갖는다.
    """

    def __init__(self, base_array, track, corridor_margin=0.08,
                 v_max=None, v_min=0.5, influence_m=None, constants=None,
                 curvature_bias=0.0):
        self.track = track
        self.constants = constants or generator_constants()
        self.half_width = self.constants["CAR_HALF_WIDTH"]
        self.half_length = self.constants["CAR_HALF_LENGTH"]
        self.mu_physical = self.constants["MU"]
        self.mu_planning = self.constants["MU_PROFILE"]
        self.ds_out = self.constants["DS_OUT"]

        self.base = np.asarray(base_array, dtype=float).copy()
        base = as_dict(self.base)
        self.n = len(self.base)
        self.xy0 = base["xy"].copy()
        self.psi0 = base["psi"].copy()
        self.normal0 = np.column_stack([-np.sin(self.psi0), np.cos(self.psi0)])
        self.s0 = base["s"].copy()
        self.length0 = float(
            self.s0[-1] + np.hypot(*(self.xy0[0] - self.xy0[-1])))
        self.ds0 = self.length0 / max(self.n, 1)
        self.corridor_margin = float(corridor_margin)
        self.v_min = float(v_min)
        self.v_max = float(v_max) if v_max else float(base["vx"].max())
        # 앵커가 아닌 구간이 원본에서 얼마나 멀리까지 끌려오는지 정하는 길이.
        # 없으면 전장의 10% (3~8 m) — 예전 튜너의 falloff 기본값과 같은 감각.
        self.influence_m = float(
            influence_m if influence_m
            else min(max(0.10 * self.length0, 3.0), 8.0))
        # 0 = 곡률 변화 최소화 (원본 모양 유지), 1 = 절대 최소곡률. solve() 참조.
        self.curvature_bias = float(np.clip(curvature_bias, 0.0, 1.0))
        # 앵커가 아닌 스테이션 바운드에 더 얹는 여유. 바운드는 스테이션에서만
        # 걸리는데 respline 이 절점 사이에서 바깥으로 부풀 수 있고, 레이캐스트도
        # 격자(res/2) 단위라 딱 맞게 붙이면 출력에서 차 반폭을 스칠 수 있다.
        self.bound_slack_m = 0.5 * track.res

        # 원본 법선 위의 free 구간. alpha 는 이 법선을 따르는 오프셋이므로
        # 여기서 한 번 재면 그대로 alpha 의 상한/하한이 된다 (build_map 처럼
        # 반복해서 다시 잴 필요가 없다).
        self.a_lo, self.a_hi = track.corridor_interval(self.xy0, self.psi0)

        self.alpha = np.zeros(self.n)
        self.v = base["vx"].copy()
        self.pinned = np.zeros(self.n, dtype=bool)
        self.anchors = {}
        self.solve_info = {"solved": False, "anchors": 0}

    # ---------------------------------------------------------------- 편집
    @property
    def xy(self):
        """현재 라인 (원본 격자 위)."""
        return self.xy0 + self.alpha[:, None] * self.normal0

    def alpha_bounds(self):
        """앵커가 아닌 스테이션의 alpha 허용 범위 (차체가 벽에 안 닿는 범위).

        코리도가 차폭보다 좁으면 (2*half_width > 폭) 정중앙으로 고정한다 —
        build_map `_solve_qp` 의 narrow 분기와 같은 처리다. lsq_linear 은
        lo < hi 를 엄격히 요구해서 등호가 들어오면 ValueError 로 죽는다.
        """
        keep_out = self.half_width + self.bound_slack_m
        low = self.a_lo + keep_out
        high = self.a_hi - keep_out
        narrow = low >= high - 1e-9
        middle = 0.5 * (self.a_lo + self.a_hi)
        low = np.where(narrow, middle - 1e-6, low)
        high = np.where(narrow, middle + 1e-6, high)
        return low, high, narrow

    def set_anchor(self, index, alpha):
        """그 스테이션의 오프셋을 사람이 정한 값으로 고정한다 (클램프하지 않음)."""
        index = int(index) % self.n
        self.anchors[index] = float(alpha)
        return index

    def clear_anchor(self, index):
        self.anchors.pop(int(index) % self.n, None)

    def clear_anchors(self):
        self.anchors.clear()

    def solve(self):
        """앵커를 등호 제약으로 두고 라인을 다시 푼다 (최소곡률 IQP).

            min  ‖D2 diag(N) alpha  +  w * D2 xy0‖²  +  lam ‖alpha‖²
            s.t. alpha[앵커] = 사람이 준 값 (클램프 없음)
                 alpha[나머지] ∈ [우벽 + 차반폭, 좌벽 - 차반폭]

        w = curvature_bias.
          w = 0 (기본) — **곡률 변화**의 최소화. D2(라인) - D2(원본) = D2 diag(N) alpha
              이므로 첫 항이 곧 "원본 대비 곡률이 얼마나 흔들렸나"다. 사람이 찍은
              앵커를 지나면서 원본 모양을 최대한 지키는 가장 매끄러운 변형이 나온다.
          w = 1 — build_map 과 같은 **절대** 최소곡률.

        기본이 w=0 인 이유는 실측이다. w=1 로 앵커 하나(-0.20 m)를 걸어 보면
        트랙 전체가 다시 그려지면서 랩이 31.33 -> 29.33 m 로 2 m 짧아지고, 앵커가
        아닌 구간의 이동량이 앵커보다 큰 0.38 m 까지 가고, 벽 여유가 0.125 m 로
        차 반폭 아래까지 내려간다. 원본은 build_map 이 0.5 m 격자 + 적응형
        이격(0.37~0.55 m)으로 푼 답인데 여기 바운드는 차 반폭(0.15 m)이라, 절대
        최소곡률은 전 구간에서 라인을 벽에 붙이는 게 이득이기 때문이다 — 점 하나
        건드렸는데 사람이 좋아했던 라인이 통째로 사라진다. 진짜 절대 최소곡률이
        필요하면 그건 튜너가 아니라 build_map 의 일이다.

        lam 은 앵커의 영향이 미치는 거리로 정한다. w=0 일 때 문제가 동차가 되므로
        연속 근사 alpha'''' + lam_c alpha = 0 의 감쇠 길이 L = lam_c^(-1/4) 이
        그대로 성립하고, 이산화하면 lam = (ds / L)^4 다 (L = influence_m).
        앵커가 없으면 alpha = 0 이 정확한 답이라 아예 풀지 않는다.
        """
        if not self.anchors:
            # 앵커가 없으면 원본이 답이다. 여기서 굳이 풀면 (이산화가 원본과
            # 달라서) 라인이 아무 이유 없이 몇 mm 씩 움직인다.
            self.alpha = np.zeros(self.n)
            self.solve_info = {"solved": False, "anchors": 0,
                               "max_alpha": 0.0, "max_free_alpha": 0.0}
            return self.alpha
        n = self.n
        identity = np.eye(n)
        second = (np.roll(identity, -1, axis=1) - 2.0 * identity +
                  np.roll(identity, 1, axis=1))
        # D2 @ diag(normal) 은 열 스케일이라 브로드캐스트 곱과 같다.
        curvature = np.vstack([second * self.normal0[:, 0],
                               second * self.normal0[:, 1]])
        # curvature_bias=0 이면 target=0 → 곡률 '변화'의 최소화 (동차 문제).
        target = -self.curvature_bias * np.concatenate([
            second @ self.xy0[:, 0], second @ self.xy0[:, 1]])
        lam = (self.ds0 / max(self.influence_m, 1e-6)) ** 4
        matrix = np.vstack([curvature, math.sqrt(lam) * identity])
        rhs = np.concatenate([target, np.zeros(n)])

        anchor_index = np.array(sorted(self.anchors), dtype=int)
        anchor_value = np.array(
            [self.anchors[int(i)] for i in anchor_index], dtype=float)
        free_index = np.setdiff1d(np.arange(n), anchor_index)
        if len(free_index) == 0:
            self.alpha = np.zeros(n)
            self.alpha[anchor_index] = anchor_value
            self.solve_info = {"solved": True, "anchors": len(anchor_index),
                               "max_alpha": float(np.abs(self.alpha).max()),
                               "max_free_alpha": 0.0, "all_anchored": True}
            return self.alpha
        # 앵커는 변수에서 소거한다. lsq_linear 은 lo == hi 를 못 받으므로 등호
        # 제약을 바운드로 표현할 수 없다.
        reduced_rhs = rhs - matrix[:, anchor_index] @ anchor_value
        low, high, narrow = self.alpha_bounds()
        result = lsq_linear(
            matrix[:, free_index], reduced_rhs,
            bounds=(low[free_index], high[free_index]))
        self.alpha = np.zeros(n)
        self.alpha[anchor_index] = anchor_value
        self.alpha[free_index] = result.x
        self.solve_info = {
            "solved": True,
            "anchors": int(len(anchor_index)),
            "max_alpha": float(np.abs(self.alpha).max()),
            "max_free_alpha": float(np.abs(result.x).max()),
            "narrow_pts": int(narrow.sum()),
            "influence_m": self.influence_m,
            "cost": float(result.cost),
        }
        return self.alpha

    def set_speed(self, index, value, falloff_m=None):
        """그 스테이션 속도를 사람이 준 값으로 **그대로** 넣고 주변만 부드럽게.

        투영하지 않는다. 마찰/가감속 한계는 report() 가 보여 주기만 한다.
        falloff 는 좌우로 가장 가까운 pinned 스테이션에서 멈추므로, 다른 코너에서
        이미 손으로 잡아둔 값이 이번 편집에 지워지지 않는다.
        """
        index = int(index) % self.n
        value = float(np.clip(value, self.v_min, self.v_max))
        delta = value - float(self.v[index])
        window = max(int(round(
            (falloff_m if falloff_m else self.influence_m) / self.ds0)), 1)
        self.v[index] = value
        self.pinned[index] = True
        touched = 1
        for direction in (1, -1):
            for offset in range(1, window + 1):
                neighbour = (index + direction * offset) % self.n
                if self.pinned[neighbour] or neighbour == index:
                    break
                weight = 0.5 * (1.0 + math.cos(math.pi * offset / window))
                self.v[neighbour] = float(np.clip(
                    self.v[neighbour] + delta * weight,
                    self.v_min, self.v_max))
                touched += 1
        return {"index": index, "value": value, "delta": delta,
                "touched": touched}

    def unpin_speed(self, index):
        """그 스테이션 속도를 원본 값으로 되돌리고 사람 소유를 푼다."""
        index = int(index) % self.n
        self.pinned[index] = False
        self.v[index] = float(self.base[index, 7])

    def reset_line(self):
        self.anchors.clear()
        self.alpha = np.zeros(self.n)
        self.solve_info = {"solved": False, "anchors": 0}

    def reset_speed(self):
        self.v = self.base[:, 7].copy()
        self.pinned[:] = False

    def reset_all(self):
        """전체 리셋 = build_map 원본. 세션 시작값이 아니라 원본으로 돌아간다."""
        self.reset_line()
        self.reset_speed()

    # ---------------------------------------------------------------- 출력
    def output(self, ds_out=None):
        """저장/표시용 9열. respline 으로 간격을 균일화하고 속도를 실어 옮긴다."""
        ds_out = float(ds_out or self.ds_out)
        moved = self.xy
        line_edited = bool(np.any(self.alpha))
        if not line_edited:
            # 라인을 안 건드렸으면 기하는 **원본 그대로** 쓴다. respline 을 태우면
            # 원본 스플라인과 이산화가 달라 kappa 가 1% 쯤 흔들리고, 그 흔들림이
            # ax 를 plant 한계 위로 밀어 올려 "손대지도 않았는데 위반" 이라는
            # 거짓 경고를 만든다 (실측: ax max 1.800 -> 1.82 > A_ACC_MAX 1.80).
            # 무편집 저장은 기하에 대해 완전한 무연산이어야 한다.
            out_xy = self.xy0.copy()
            out_psi = self.psi0.copy()
            out_kappa = self.base[:, 6].copy()
            out_s = self.s0.copy()
            out_v = self.v.copy()
            geometry = {"length": self.length0}
        else:
            geometry = respline_uniform(moved, ds_out)
            out_xy = geometry["xy"]
            out_psi = geometry["psi"]
            out_kappa = geometry["kappa"]
            out_s = geometry["s"]
            # 속도는 '원본 격자 스테이션이 새 라인 위에서 실제로 어디에 있는지'를
            # 기준으로 옮긴다. 인덱스 비율로 옮기면 간격이 찌그러진 구간에서
            # 속도 프로파일이 코너와 어긋난다.
            moved_segments = closed_segments(moved)
            moved_s = np.concatenate([[0.0], np.cumsum(moved_segments)])[:-1]
            moved_length = float(moved_segments.sum())
            base_u = moved_s / max(moved_length, 1e-9)
            out_u = out_s / max(geometry["length"], 1e-9)
            out_v = interp_periodic(out_u, base_u, self.v)
        out_u = out_s / max(geometry["length"], 1e-9)
        out_segments = closed_segments(out_xy)
        out_ax = closed_loop_ax(out_v, out_segments)
        a_lo, a_hi = self.track.corridor_interval(out_xy, out_psi)
        d_left_raw = np.maximum(a_hi, 0.0)
        d_right_raw = np.maximum(-a_lo, 0.0)
        columns = np.column_stack([
            out_s, out_xy[:, 0], out_xy[:, 1],
            np.maximum(d_right_raw - self.corridor_margin, 0.0),
            np.maximum(d_left_raw - self.corridor_margin, 0.0),
            out_psi, out_kappa, out_v, out_ax])
        return {
            "columns": columns, "xy": out_xy, "psi": out_psi,
            "kappa": out_kappa, "s": out_s, "vx": out_v, "ax": out_ax,
            "segments": out_segments, "length": geometry["length"],
            "d_left_raw": d_left_raw, "d_right_raw": d_right_raw,
            "clearance_raw": np.minimum(d_left_raw, d_right_raw),
            "u": out_u,
        }

    def runtime_format_check(self, out):
        """`global_trajectory_node.ValidateTrajectory` 의 **형식** 검사를 미리 돌린다.

        물리(마찰원·조향)는 advisory 로 통과시킬 수 있지만 형식은 못 한다.
        여기서 loadable=False 가 나오면 저장은 되더라도 그 라인으로는 노드가
        뜨지 않으므로, 화면에 물리 경고와 구분해서 보여 줘야 한다.
        """
        columns = out["columns"]
        problems = []
        spacing = np.diff(columns[:, 0])
        geometric = np.hypot(np.diff(columns[:, 1]), np.diff(columns[:, 2]))
        uniformity = (float(spacing.max() / spacing.min())
                      if len(spacing) and spacing.min() > 0 else float("inf"))
        closure = float(np.hypot(*(columns[0, 1:3] - columns[-1, 1:3])))
        numeric_ax = np.zeros(0)
        if len(spacing):
            numeric_ax = (
                columns[1:, 7] ** 2 - columns[:-1, 7] ** 2) / (2.0 * spacing)
        ax_error = (float(np.abs(numeric_ax - columns[:-1, 8]).max())
                    if len(numeric_ax) else 0.0)
        spacing_error = (float(np.abs(geometric - spacing).max())
                         if len(spacing) else 0.0)
        if not np.isfinite(columns).all():
            problems.append("NaN/Inf")
        if len(columns) < 8:
            problems.append("웨이포인트 8개 미만")
        if len(spacing) and spacing.min() <= 1e-5:
            problems.append("s 가 단조 증가하지 않음")
        if len(spacing) and np.any(
                np.abs(geometric - spacing) > np.maximum(0.02, 0.20 * spacing)):
            problems.append("s 와 기하 간격 불일치")
        if uniformity > 1.25:
            problems.append(f"간격 균일성 {uniformity:.3f} > 1.25")
        if len(spacing) and closure > spacing.max() * 2.5:
            problems.append(f"폐루프 미닫힘 {closure:.3f} m")
        if ax_error > 1.0:
            problems.append(f"ax↔v 불일치 {ax_error:.3f}")
        return {
            "loadable": not problems,
            "problems": problems,
            "spacing_uniformity": uniformity,
            "spacing_min_m": float(spacing.min()) if len(spacing) else 0.0,
            "spacing_max_m": float(spacing.max()) if len(spacing) else 0.0,
            "spacing_error_m": spacing_error,
            "closure_m": closure,
            "ax_error_mps2": ax_error,
        }

    def report(self, output=None):
        """무엇이 한계를 넘었는지 **보고**한다. 저장을 막는 데 쓰지 않는다."""
        out = output or self.output()
        kappa = np.abs(out["kappa"])
        v = out["vx"]
        ax = out["ax"]
        lateral = v ** 2 * kappa
        worst_lateral = np.maximum(lateral, np.roll(lateral, -1))
        combined = np.hypot(worst_lateral, ax)
        required_mu = combined / G
        radius = 1.0 / np.maximum(kappa, 1e-9)
        car_radius = WHEELBASE / math.tan(MAX_STEER)
        clearance = out["clearance_raw"]
        footprint_ok = self.track.footprint_free(
            out["xy"], out["psi"], self.half_length, self.half_width)

        wall_bad = np.flatnonzero(clearance < self.half_width)
        sharp = np.flatnonzero(radius < car_radius)
        friction_bad = np.flatnonzero(required_mu > self.mu_physical)
        footprint_bad = np.flatnonzero(~footprint_ok)
        # 허용오차 1%: ax 는 v 에서 유도하므로 (a) CSV 7자리 양자화, (b) 라인을
        # 옮기면 구간 길이가 바뀌어 같은 v 에서도 ax 가 0.3% 쯤 달라진다. 1e-9 로
        # 보면 손도 안 댔거나 라인만 살짝 옮긴 경우에 "plant 한계 초과" 거짓 경고가
        # 뜨고, 거짓 경고는 진짜 경고를 못 믿게 만든다. plant 한계 자체가 실측
        # p90(±0.3 수준)이라 1% 는 측정 잡음보다 훨씬 작다. 사람이 속도를 실제로
        # 올리면 ax 는 1 m/s² 단위로 움직이므로 이 허용오차에 숨지 않는다.
        accel_tolerance = max(1e-3, 0.01 * A_ACC_MAX)
        decel_tolerance = max(1e-3, 0.01 * A_DEC_PLANNED)
        accel_bad = np.flatnonzero(ax > A_ACC_MAX + accel_tolerance)
        decel_bad = np.flatnonzero(ax < -A_DEC_PLANNED - decel_tolerance)

        # 형식 검사는 물리 검사와 성질이 다르다. advisory 모드는 마찰원/조향만
        # 완화하고 형식은 두 모드 모두 거부하므로, 형식이 깨지면 그 CSV 로는
        # global_trajectory_node 가 아예 안 뜬다 (= 차가 출발하지 않는다).
        # 저장은 막지 않는 도구라서, 이건 반드시 사람 눈에 보여야 한다.
        format_report = self.runtime_format_check(out)

        flags = []
        if not format_report["loadable"]:
            flags.append("format")
        if len(wall_bad) or len(footprint_bad):
            flags.append("wall")
        if len(sharp):
            flags.append("steer_radius")
        if len(friction_bad):
            flags.append("friction")
        if len(accel_bad) or len(decel_bad):
            flags.append("plant_accel")
        return {
            "unsafe_flags": flags,
            "format": format_report,
            "n": int(len(v)),
            "length_m": float(out["length"]),
            "speed": {
                "min": float(v.min()), "mean": float(v.mean()),
                "max": float(v.max()),
                "pinned_pts": int(self.pinned.sum()),
                "at_cap_pts": int(np.sum(v >= self.v_max - 1e-9)),
            },
            "wall": {
                "min_clearance_m": float(clearance.min()),
                "car_half_width_m": self.half_width,
                "violating_pts": int(len(wall_bad)),
                "footprint_violating_pts": int(len(footprint_bad)),
                "worst_s_m": (float(out["s"][int(clearance.argmin())])),
            },
            "steering": {
                "min_radius_m": float(radius.min()),
                "car_min_radius_m": float(car_radius),
                "violating_pts": int(len(sharp)),
            },
            "friction": {
                "max_required_mu": float(required_mu.max()),
                "planning_mu": self.mu_planning,
                "physical_mu": self.mu_physical,
                "over_planning_pts": int(
                    np.sum(required_mu > self.mu_planning)),
                "over_physical_pts": int(len(friction_bad)),
                "worst_s_m": float(out["s"][int(required_mu.argmax())]),
            },
            "longitudinal": {
                "ax_min": float(ax.min()), "ax_max": float(ax.max()),
                "plant_accel_max": A_ACC_MAX,
                "plant_decel_max": A_DEC_PLANNED,
                "over_accel_pts": int(len(accel_bad)),
                "over_decel_pts": int(len(decel_bad)),
            },
            "line": dict(self.solve_info),
            "per_point": {
                "required_mu": required_mu,
                "clearance": clearance,
                "radius": radius,
            },
        }

    # ---------------------------------------------------------------- 저장/이어하기
    def state_payload(self, base_meta=None, base_name=""):
        """사이드카 JSON 내용. 다음 세션이 편집을 **정확히** 이어받게 한다.

        alpha 를 CSV 에서 되짚어 복원할 수도 있지만(투영), 앵커가 어디였는지와
        어느 속도가 사람 소유였는지는 CSV 에 남지 않는다. 그 두 개가 없으면
        이어서 편집할 때 falloff 가 남의 튜닝을 덮어쓴다.
        """
        return {
            "schema_version": 1,
            "base_csv": base_name,
            "base_generation_id": (base_meta or {}).get("generation_id", ""),
            "n": self.n,
            "corridor_margin_m": self.corridor_margin,
            "influence_m": self.influence_m,
            "curvature_bias": self.curvature_bias,
            "v_min": self.v_min,
            "v_max": self.v_max,
            "anchors": {str(k): float(v) for k, v in sorted(self.anchors.items())},
            "speed": [float(value) for value in self.v],
            "pinned": [int(flag) for flag in self.pinned],
        }

    def apply_state(self, payload):
        """사이드카 JSON 을 되돌려 놓는다. 격자가 다르면 조용히 버린다.

        원본을 다시 뽑으면(mkcsv 재실행) 웨이포인트 수가 달라질 수 있는데, 그때
        옛 앵커 인덱스를 그대로 쓰면 엉뚱한 위치를 고정하게 된다. 그 경우
        "이어받지 못했다"를 돌려주고 호출부가 사람에게 알린다.
        """
        if not isinstance(payload, dict):
            return False, "사이드카 형식이 아님"
        if int(payload.get("n", -1)) != self.n:
            return False, (
                f"원본 웨이포인트 수가 달라졌습니다 "
                f"({payload.get('n')} -> {self.n}). 편집 상태를 이어받지 않습니다")
        speed = np.asarray(payload.get("speed", []), dtype=float)
        pinned = np.asarray(payload.get("pinned", []), dtype=bool)
        if len(speed) != self.n or len(pinned) != self.n or \
                not np.isfinite(speed).all():
            return False, "사이드카 속도/pinned 길이가 맞지 않음"
        self.v = np.clip(speed, self.v_min, max(self.v_max, float(speed.max())))
        self.pinned = pinned
        self.anchors = {
            int(key): float(value)
            for key, value in (payload.get("anchors") or {}).items()
            if 0 <= int(key) < self.n and math.isfinite(float(value))}
        self.influence_m = float(
            payload.get("influence_m", self.influence_m) or self.influence_m)
        self.curvature_bias = float(payload.get("curvature_bias", 0.0))
        self.solve()
        return True, ""

    def save(self, csv_path, base_name="", base_meta=None, ds_out=None):
        """튜닝본 CSV + 사이드카를 원자적으로 쓴다. **원본은 건드리지 않는다.**

        물리 위반으로 저장을 막지 않는다 — 그게 이 도구의 요구사항이다. 대신
        무엇을 넘겼는지 헤더에 `unsafe_flags` 로 남겨서, 나중에 파일만 보고도
        "이건 사람이 한계를 넘긴 라인"이라는 걸 알 수 있게 한다.
        """
        csv_path = Path(csv_path)
        out = self.output(ds_out)
        report = self.report(out)
        generation_id = uuid.uuid4().hex
        flags = report["unsafe_flags"]
        header = [
            f"generation_id={generation_id}",
            "tuned by racelinetuner (web): 사람 판단 우선, 물리 검사는 보고만",
            f"base_csv={base_name}; "
            f"base_generation_id={(base_meta or {}).get('generation_id', '')}",
            f"corridor_margin_m={self.corridor_margin:.7f}; "
            f"influence_m={self.influence_m:.4f}; "
            f"curvature_bias={self.curvature_bias:.4f}",
            f"anchors={len(self.anchors)}; speed_pins={int(self.pinned.sum())}",
            f"planning_mu={self.mu_planning:.4f}; "
            f"physical_mu={self.mu_physical:.4f}",
            f"unsafe_flags={','.join(flags) if flags else 'none'}",
            f"min_wall_clearance_m={report['wall']['min_clearance_m']:.4f}; "
            f"min_radius_m={report['steering']['min_radius_m']:.4f}; "
            f"max_required_mu={report['friction']['max_required_mu']:.4f}",
        ]
        digest = write_csv_atomic(csv_path, out["columns"], header)
        payload = self.state_payload(base_meta, base_name)
        payload["generation_id"] = generation_id
        payload["csv_sha256"] = digest
        payload["report"] = {
            key: value for key, value in report.items() if key != "per_point"}
        state_path = Path(str(csv_path) + ".tuner_state.json")
        write_json_atomic(state_path, payload)
        return {"csv": str(csv_path), "state": str(state_path),
                "generation_id": generation_id, "sha256": digest,
                "report": report}

    def snapshot(self, out=None):
        """브라우저로 보낼 JSON 안전 상태."""
        out = out or self.output()
        report = self.report(out)
        return {
            "n": self.n,
            "base": {
                "xy": self.xy0.round(4).tolist(),
                "vx": self.base[:, 7].round(4).tolist(),
                "s": self.s0.round(4).tolist(),
            },
            "line": {
                "xy": self.xy.round(4).tolist(),
                "alpha": self.alpha.round(5).tolist(),
            },
            "speed": self.v.round(4).tolist(),
            "pinned": [int(flag) for flag in self.pinned],
            "anchors": {str(k): round(v, 5) for k, v in sorted(self.anchors.items())},
            "bounds": {
                "low": (self.a_lo + self.half_width).round(4).tolist(),
                "high": (self.a_hi - self.half_width).round(4).tolist(),
            },
            "out": {
                "xy": out["xy"].round(4).tolist(),
                "s": out["s"].round(4).tolist(),
                "vx": out["vx"].round(4).tolist(),
                "ax": out["ax"].round(4).tolist(),
                "kappa": out["kappa"].round(5).tolist(),
                "required_mu": report["per_point"]["required_mu"].round(4).tolist(),
                "clearance": report["per_point"]["clearance"].round(4).tolist(),
                "radius": np.minimum(
                    report["per_point"]["radius"], 99.0).round(3).tolist(),
            },
            "report": {
                key: value for key, value in report.items()
                if key != "per_point"},
            "limits": {
                "v_min": self.v_min, "v_max": self.v_max,
                "influence_m": self.influence_m,
                "curvature_bias": self.curvature_bias,
                "half_width": self.half_width,
                "half_length": self.half_length,
                "mu_planning": self.mu_planning,
                "mu_physical": self.mu_physical,
                "plant_accel": A_ACC_MAX, "plant_decel": A_DEC_PLANNED,
                "car_min_radius": WHEELBASE / math.tan(MAX_STEER),
            },
        }


def write_json_atomic(path, payload):
    """사이드카 JSON 도 CSV 와 같은 원자성으로 쓴다."""
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    data = (json.dumps(payload, indent=2, sort_keys=True) + "\n").encode("utf-8")
    descriptor, temporary = tempfile.mkstemp(
        prefix=f".{path.name}.", suffix=".tmp", dir=path.parent)
    try:
        with os.fdopen(descriptor, "wb") as handle:
            handle.write(data)
            handle.flush()
            os.fsync(handle.fileno())
        os.replace(temporary, path)
        flags = os.O_RDONLY | getattr(os, "O_DIRECTORY", 0)
        directory = os.open(path.parent, flags)
        try:
            os.fsync(directory)
        finally:
            os.close(directory)
    finally:
        if os.path.exists(temporary):
            os.unlink(temporary)

