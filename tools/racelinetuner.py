#!/usr/bin/env python3
"""racelinetuner — 웹 레이스라인 튜너 (RViz/ROS 없이 브라우저에서).

    python3 tools/racelinetuner.py my_track0807
    python3 tools/racelinetuner.py                 # params.yaml 의 현행 라인

무엇을 하는 도구인가
--------------------
`build_map`(mkcsv) 이 맵만 보고 만든 라인이 **초안**이고, 사람은 실차를 몰아 본 뒤
"이 코너는 그립이 남았다 / 이 직선은 어디까지 밟아도 된다"를 안다. 이 도구는 그
판단을 **깎지 않고** 라인에 새긴다:

  - 속도 핸들에 넣은 값이 그대로 들어간다 (마찰원으로 되풀지 않는다)
  - 라인에 찍은 점은 등호 제약이다 (코리도로 클램프하지 않는다)
  - 물리 한계는 화면에 빨갛게 **보여만** 준다

원본은 절대 건드리지 않고 `<맵>_tuned_iqp_*.csv` 로 따로 저장한다:

    mkcsv my_track0808          # 원본 생성
    rc   my_track0808           # 원본으로 주행
    tune my_track0808           # 이 도구 (튜닝본 있으면 이어서 편집)
    ./build_all.sh              # 저장한 CSV 를 install 로
    rc   my_track0808_tuned     # 튜닝본으로 주행 (validation_mode=advisory)

⚠️ 반드시 **소스 트리**의 CSV 를 편집한다 (`planning_ws/src/planning_pkg/config/`).
   install 쪽을 고치면 다음 빌드에 덮여 사라지고, 소스와 어긋난 유령이 남는다.
"""

import argparse
import glob
import importlib.util
import json
import math
import os
import sqlite3
import sys
import threading
import time
import urllib.parse
import webbrowser
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent            # <ws>/tools
WORKSPACE_ROOT = HERE.parent                      # <ws>
SRC_PKG = WORKSPACE_ROOT / "planning_ws" / "src" / "planning_pkg"
RESOLVER_PATH = SRC_PKG / "scripts" / "tuner" / "line_resolver.py"
PARAMS_YAML = SRC_PKG / "config" / "params.yaml"
# 화면은 파이썬 문자열이 아니라 실제 .html 파일로 둔다 — 중괄호/역슬래시 이스케이프
# 사고가 없고, 편집기에서 JS/CSS 하이라이팅이 그대로 먹는다.
PAGE_PATH = HERE / "racelinetuner.html"
# 백을 찾을 곳 (한 단계 아래의 run_*/metadata.yaml 까지 본다).
# bagviz.find_local_bags 는 HOME 과 HOME/IFAC_ws 바로 아래만 보므로
# ~/IFAC_ws/rosbag/run_*/ 를 못 찾는다 — 그래서 여기 따로 둔다.
BAG_ROOTS = (WORKSPACE_ROOT, WORKSPACE_ROOT / "rosbag", Path.home() / "bags")
G = 9.81


# ----------------------------------------------------------- 모듈 로딩
def load_resolver():
    """`line_resolver.py` 를 **소스 트리 경로로** 로드한다.

    ament share(install) 경로로 로드하면 그 안의 PKG_ROOT/WORKSPACE_ROOT 가
    install 트리를 가리켜서, 맵을 `install/maps` 에서 찾고 CSV 를 install 에
    저장한다 — 다음 빌드에 덮여 사라지고 소스와 어긋난 유령이 남는다.
    그래서 경로를 명시하고, 해석 결과가 소스 트리인지 여기서 확인한다.
    """
    if not RESOLVER_PATH.is_file():
        raise SystemExit(
            f"line_resolver.py 를 못 찾았습니다: {RESOLVER_PATH}\n"
            "  tools/ 가 워크스페이스 루트 바로 아래에 있어야 합니다.")
    spec = importlib.util.spec_from_file_location(
        "racelinetuner_line_resolver", RESOLVER_PATH)
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    expected = SRC_PKG / "config" / "global_trajectory"
    if module.CSV_DIR.resolve() != expected.resolve():
        raise SystemExit(
            "line_resolver 가 소스 트리를 가리키지 않습니다:\n"
            f"  CSV_DIR   = {module.CSV_DIR}\n"
            f"  기대값    = {expected}\n"
            "  install 트리를 편집하면 다음 빌드에 사라집니다.")
    return module


def load_bagviz():
    """CDR 리더/디코더를 재사용한다. 없으면 백 오버레이만 비활성."""
    path = HERE / "bagviz.py"
    if not path.is_file():
        return None
    try:
        spec = importlib.util.spec_from_file_location(
            "racelinetuner_bagviz", path)
        module = importlib.util.module_from_spec(spec)
        sys.modules[spec.name] = module
        spec.loader.exec_module(module)
        for name in ("CDR", "dec_odom", "dec_float32", "png_gray_alpha"):
            if not hasattr(module, name):
                return None
        return module
    except Exception:
        return None


# ----------------------------------------------------------- 라인 탐색
def deployed_line_name():
    """params.yaml 이 가리키는 현행 raceline 파일명 (인자 없이 띄울 때의 기본)."""
    try:
        import yaml
        document = yaml.safe_load(PARAMS_YAML.read_text(encoding="utf-8"))
        return (document["global_trajectory_node"]["ros__parameters"]
                ["global_trajectory/trajectory_csv_file"])
    except Exception:
        return ""


def list_lines(resolver):
    """맵 이름 -> {original, tuned} 파일명. 소스 트리 기준."""
    lines = {}
    for path in sorted(resolver.CSV_DIR.glob("*_iqp_*.csv")):
        stem = path.name.split("_iqp_", 1)[0]
        if stem.endswith("_tuned"):
            lines.setdefault(stem[: -len("_tuned")], {})["tuned"] = path.name
        else:
            lines.setdefault(stem, {})["original"] = path.name
    return lines


def resolve_target(resolver, requested):
    """사용자가 준 이름 -> (맵이름, 원본 CSV 파일명).

    받는 형태: 맵 이름(`my_track0808`), `_tuned` 붙은 이름, CSV 파일명 그대로,
    빈 값(= params.yaml 의 현행 라인).
    """
    lines = list_lines(resolver)
    if not lines:
        raise SystemExit(
            f"편집할 raceline 이 없습니다: {resolver.CSV_DIR}\n"
            "  먼저 mkcsv <맵이름> 으로 원본을 만드세요.")
    name = (requested or "").strip()
    if name.endswith(".csv"):
        stem = name.split("_iqp_", 1)[0].split("_sp_", 1)[0]
        name = stem[: -len("_tuned")] if stem.endswith("_tuned") else stem
    if name.endswith("_tuned"):
        name = name[: -len("_tuned")]
    if not name:
        deployed = deployed_line_name()
        if deployed:
            name = deployed.split("_iqp_", 1)[0]
    if name not in lines:
        raise SystemExit(
            f"맵 '{name or '(지정 없음)'}' 의 원본 라인을 못 찾았습니다.\n"
            f"  가능한 맵: {', '.join(sorted(lines))}\n"
            f"  (원본이 없으면 mkcsv <맵이름> 먼저)")
    entry = lines[name]
    if "original" not in entry:
        raise SystemExit(
            f"'{name}' 에 튜닝본만 있고 원본이 없습니다. 원본이 기준이라 편집할 수 "
            "없습니다 — mkcsv 로 원본을 다시 만드세요.")
    return name, entry["original"]


def parse_csv_margin(csv_name, default=0.15):
    """파일명의 `margin<...>m` (build_map 이 d 열에서 뺀 reserve)."""
    import re
    match = re.search(r"margin([0-9]+(?:\.[0-9]+)?)m", csv_name)
    return float(match.group(1)) if match else float(default)


def parse_csv_vmax(csv_name, default=None):
    import re
    match = re.search(r"_max([0-9]+(?:\.[0-9]+)?)ms", csv_name)
    return float(match.group(1)) if match else default


# ----------------------------------------------------------- 세션
class Session:
    """서버가 들고 있는 편집 상태 (단일 사용자 도구라 하나면 충분)."""

    def __init__(self, resolver, map_name, base_csv, ds_out=None):
        self.resolver = resolver
        self.map_name = map_name
        self.base_csv = base_csv
        self.base_path = resolver.CSV_DIR / base_csv
        self.tuned_csv = resolver.tuned_csv_name(base_csv)
        self.tuned_path = resolver.CSV_DIR / self.tuned_csv
        self.state_path = Path(str(self.tuned_path) + ".tuner_state.json")
        self.map_yaml = resolver.find_map_yaml(map_name)
        self.track = resolver.TrackMap(self.map_yaml)
        self.base, self.base_meta = resolver.load_csv(self.base_path)
        self.notes = []
        self.state = resolver.RacelineState(
            self.base, self.track,
            corridor_margin=parse_csv_margin(base_csv),
            v_max=parse_csv_vmax(base_csv))
        self.ds_out = ds_out
        self.undo_stack = []
        self.resumed = False
        self._resume()

    def _resume(self):
        """튜닝본이 이미 있으면 사이드카로 편집을 정확히 이어받는다."""
        if not self.state_path.is_file():
            if self.tuned_path.is_file():
                self.notes.append(
                    f"튜닝본({self.tuned_csv})은 있는데 편집 상태 사이드카가 "
                    "없습니다. 원본에서 새로 시작합니다 (기존 튜닝본은 저장할 때 "
                    "덮어씁니다).")
            return
        try:
            payload = json.loads(self.state_path.read_text(encoding="utf-8"))
        except (OSError, ValueError) as exc:
            self.notes.append(f"사이드카를 읽지 못했습니다: {exc}")
            return
        recorded = payload.get("base_generation_id", "")
        current = self.base_meta.get("generation_id", "")
        if recorded and current and recorded != current:
            self.notes.append(
                "⚠️ 원본이 다시 생성됐습니다 (generation_id 불일치). 예전 편집을 "
                "그대로 얹으면 엉뚱한 위치를 고정하게 되므로 이어받지 않습니다.")
            return
        ok, why = self.state.apply_state(payload)
        self.resumed = ok
        self.notes.append(
            f"튜닝본에서 이어서 편집합니다 ({self.tuned_csv})" if ok
            else f"편집 상태를 이어받지 못했습니다: {why}")

    # ------------------------------------------------------------ undo
    def snapshot_undo(self):
        self.undo_stack.append((
            self.state.alpha.copy(), self.state.v.copy(),
            self.state.pinned.copy(), dict(self.state.anchors)))
        if len(self.undo_stack) > 200:
            self.undo_stack.pop(0)

    def undo(self):
        if not self.undo_stack:
            return False
        alpha, speed, pinned, anchors = self.undo_stack.pop()
        self.state.alpha = alpha
        self.state.v = speed
        self.state.pinned = pinned
        self.state.anchors = anchors
        return True

    # ------------------------------------------------------------ 편집
    def apply(self, operations):
        state = self.state
        messages = []
        needs_solve = False
        for operation in operations:
            kind = operation.get("op")
            if kind == "undo":
                if not self.undo():
                    messages.append("되돌릴 편집이 없습니다")
                continue
            self.snapshot_undo()
            if kind == "anchor_xy":
                index = int(operation["index"]) % state.n
                point = np.array([float(operation["x"]), float(operation["y"])])
                alpha = float(
                    (point - state.xy0[index]) @ state.normal0[index])
                state.set_anchor(index, alpha)
                needs_solve = True
            elif kind == "anchor":
                state.set_anchor(int(operation["index"]),
                                 float(operation["alpha"]))
                needs_solve = True
            elif kind == "anchor_clear":
                state.clear_anchor(int(operation["index"]))
                needs_solve = True
            elif kind == "anchor_clear_all":
                state.reset_line()
            elif kind == "speed":
                state.set_speed(int(operation["index"]),
                                float(operation["value"]))
            elif kind == "speed_unpin":
                state.unpin_speed(int(operation["index"]))
            elif kind == "reset_line":
                state.reset_line()
            elif kind == "reset_speed":
                state.reset_speed()
            elif kind == "reset_all":
                state.reset_all()
            elif kind == "params":
                if "influence_m" in operation:
                    state.influence_m = float(np.clip(
                        float(operation["influence_m"]), 0.3, 30.0))
                if "curvature_bias" in operation:
                    state.curvature_bias = float(np.clip(
                        float(operation["curvature_bias"]), 0.0, 1.0))
                if "v_max" in operation:
                    state.v_max = float(np.clip(
                        float(operation["v_max"]), 0.5, 30.0))
                needs_solve = bool(state.anchors)
            else:
                self.undo_stack.pop()
                messages.append(f"알 수 없는 편집: {kind!r}")
        if needs_solve:
            state.solve()
        return messages

    def save(self):
        result = self.state.save(
            self.tuned_path, base_name=self.base_csv,
            base_meta=self.base_meta, ds_out=self.ds_out)
        return result

    # ------------------------------------------------------------ 전송
    def payload(self, messages=()):
        snapshot = self.state.snapshot()
        snapshot["meta"] = {
            "map": self.map_name,
            "base_csv": self.base_csv,
            "tuned_csv": self.tuned_csv,
            "base_path": str(self.base_path),
            "tuned_path": str(self.tuned_path),
            "map_yaml": str(self.map_yaml),
            "resumed": self.resumed,
            "tuned_exists": self.tuned_path.is_file(),
            "undo_depth": len(self.undo_stack),
            "notes": list(self.notes) + list(messages),
        }
        return snapshot


def map_layers(track, bagviz):
    """배경 occupancy PNG (없으면 벽 셀 좌표로 폴백).

    ⚠️ 행 순서를 건드리지 않는다. map_server 규약대로 **row 0 = world y 최대**다.
    브라우저는 월드 (x0, y_top) 을 화면 좌상단으로 잡고 drawImage 를 아래로
    그리므로, 그 규약이 그대로 맞다. 예전에 여기서 np.flipud 를 걸었더니 맵이
    상하 반전돼 라인이 벽을 지나가는 것처럼 보였다 — 그리고 자체검증은
    '뒤집은 배열' 기준으로 짜여 있어서 그걸 통과시켰다.
    """
    layer = {
        "extent": {
            "x0": float(track.origin[0]), "y0": float(track.origin[1]),
            "width_m": float(track.W * track.res),
            "height_m": float(track.H * track.res),
            "yaw": float(track.origin[2]), "res": float(track.res),
        },
        "wall_px": None,
        "png": None,
    }
    if bagviz is not None:
        # 주행 가능 = 밝게, 벽/unknown = 어둡게. ring 밖은 반투명으로 남겨
        # '트랙으로 인식되지 않은 free 공간'이 눈에 보이게 한다.
        gray = np.where(track.ring, 245, np.where(track.free_raw, 205, 70))
        alpha = np.where(track.ring, 255, np.where(track.free_raw, 90, 210))
        png = bagviz.png_gray_alpha(
            gray.astype(np.uint8), alpha.astype(np.uint8))
        import base64
        layer["png"] = "data:image/png;base64," + \
            base64.b64encode(png).decode("ascii")
    else:
        # PNG 를 못 만들면(bagviz 없음) 벽 셀을 좌표로 보낸다. 월드 좌표라
        # 방향이 틀릴 여지가 없지만 26k 점이라 300 KB 쯤 된다 — 그래서 폴백.
        rows, cols = np.nonzero(~track.ring)
        layer["wall_px"] = {"cols": cols.tolist(), "rows": rows.tolist()}
    return layer


def verify_map_layer_orientation(track):
    """보낸 PNG 를 **브라우저가 실제로 그리는 대로** 샘플해 is_free 와 맞는지 본다.

    스크린샷 없이 맵 뒤집힘을 잡는 유일한 방법이라 시작할 때 한 번 돌린다.
    ⚠️ 이 검사를 '내가 보낸 배열' 기준으로 짜면 아무 의미가 없다 — 예전 버전이
    그래서 상하 반전을 400/400 으로 통과시켰다. 브라우저 쪽 식을 그대로 쓴다:

        브라우저: 월드 (x0, y0+H*res) 를 화면 좌상단에 놓고 drawImage 를
                  오른쪽·아래로 W*res × H*res 만큼 그린다.
        따라서   col = (wx - x0) / res
                 row = (y_top - wy) / res      (row 0 = world y 최대)
    """
    if abs(float(track.origin[2])) > 1e-9:
        return True, "맵 yaw != 0 — 배경 회전은 브라우저 transform 이 처리"
    generator = np.random.default_rng(0)
    world_x = track.origin[0] + generator.uniform(
        0.0, track.W * track.res, 600)
    world_y = track.origin[1] + generator.uniform(
        0.0, track.H * track.res, 600)
    y_top = track.origin[1] + track.H * track.res
    rows = np.floor((y_top - world_y) / track.res).astype(int)
    cols = np.floor((world_x - track.origin[0]) / track.res).astype(int)
    inside = ((rows >= 0) & (rows < track.H) &
              (cols >= 0) & (cols < track.W))
    sampled = np.zeros(len(world_x), dtype=bool)
    sampled[inside] = track.ring[rows[inside], cols[inside]]
    truth = track.is_free(np.column_stack([world_x, world_y]))
    agree = int(np.sum(sampled == truth))
    return agree == len(truth), f"배경 정합 {agree}/{len(truth)}"


# ----------------------------------------------------------- 백 오버레이
def find_bags(roots=BAG_ROOTS, limit=60):
    """rosbag2 백 폴더를 찾는다 (`<root>/**/metadata.yaml`, 2단계까지)."""
    found = {}
    for root in roots:
        root = Path(root)
        if not root.is_dir():
            continue
        for pattern in ("*/metadata.yaml", "*/*/metadata.yaml"):
            for meta in root.glob(pattern):
                directory = meta.parent
                databases = sorted(directory.glob("*.db3"))
                if not databases:
                    continue
                key = str(directory.resolve())
                if key in found:
                    continue
                found[key] = {
                    "path": key,
                    "name": directory.name,
                    "parent": directory.parent.name,
                    "mtime": databases[0].stat().st_mtime,
                    "size_mb": round(sum(
                        db.stat().st_size for db in databases) / 1e6, 1),
                }
    ordered = sorted(found.values(), key=lambda item: -item["mtime"])
    return ordered[:limit]


def decode_odom_with_yaw_rate(bagviz, blob):
    """nav_msgs/Odometry → (x, y, yaw, speed, yaw_rate).

    bagviz.dec_odom 은 twist 의 **선속도까지만** 읽고 (x, y, yaw, speed) 4개를
    돌려준다. 남은 그립을 보려면 yaw rate 가 필요하다 (a_lat = yaw_rate * v).
    그래서 bagviz 를 고치지 않고 그쪽 CDR 프리미티브로 여기서 한 번 더 읽는다.
    필드 순서: pose(x,y,z,q×4) + 공분산 36 + twist linear 3 + angular 3.
    """
    cursor = bagviz.CDR(blob)
    cursor.header()
    cursor.string()                                   # child_frame_id
    x, y, _z, qx, qy, qz, qw = cursor.f64n(7)
    cursor.skip_f64(36)                               # pose covariance
    vx, vy, _vz = cursor.f64n(3)
    _wx, _wy, wz = cursor.f64n(3)
    speed = math.hypot(vx, vy) * (1.0 if vx >= 0.0 else -1.0)
    return x, y, bagviz.quat_yaw(qx, qy, qz, qw), speed, wz


def read_bag(bagviz, bag_dir, line_xy, line_s, max_points=6000):
    """실주행 궤적 + 실측 속도/횡가속/횡오차를 라인 위 s 로 정리한다.

    새 디코더를 만들지 않는다 — bagviz 의 CDR/dec_odom/dec_float32 를 그대로
    쓴다. 횡가속은 IMU 대신 odom 의 (yaw rate x 속도)로 만든다: IMU 장착
    오프셋/바이어스와 무관하고 디코더도 이미 있다.
    """
    if bagviz is None:
        raise ValueError("bagviz.py 가 없어 백을 읽을 수 없습니다")
    databases = sorted(Path(bag_dir).glob("*.db3"))
    if not databases:
        raise ValueError(f"백에 .db3 가 없습니다: {bag_dir}")
    poses = []
    lateral_errors = []
    for database in databases:
        uri = "file:%s?mode=ro" % urllib.parse.quote(str(database))
        connection = sqlite3.connect(uri, uri=True)
        try:
            topics = {
                name: (tid, typ) for tid, name, typ in
                connection.execute("SELECT id, name, type FROM topics")}
            base = connection.execute(
                "SELECT MIN(timestamp) FROM messages").fetchone()[0] or 0
            for topic, sink, decoder in (
                    ("/pf/pose/odom", poses,
                     lambda blob: decode_odom_with_yaw_rate(bagviz, blob)),
                    ("/pitwall/lat_err", lateral_errors, bagviz.dec_float32)):
                if topic not in topics:
                    continue
                for stamp, blob in connection.execute(
                        "SELECT timestamp, data FROM messages "
                        "WHERE topic_id=? ORDER BY timestamp",
                        (topics[topic][0],)):
                    try:
                        sink.append(((stamp - base) / 1e9, decoder(blob)))
                    except Exception:
                        continue
        finally:
            connection.close()
    if not poses:
        raise ValueError(
            "/pf/pose/odom 이 없습니다 — 위치추정이 도는 상태로 딴 백이어야 합니다")

    times = np.array([row[0] for row in poses], dtype=float)
    decoded = [row[1] for row in poses]
    x = np.array([item[0] for item in decoded], dtype=float)
    y = np.array([item[1] for item in decoded], dtype=float)
    speed = np.array([item[3] for item in decoded], dtype=float)
    yaw_rate = np.array([item[4] for item in decoded], dtype=float)
    if not np.any(np.abs(yaw_rate) > 1e-6):
        # PF 가 twist.angular 을 안 채우는 설정도 있다. 그때는 yaw 를 시간으로
        # 미분해서 쓴다 (노이즈는 크지만 0 으로 두면 그립 그래프가 통째로 거짓).
        yaw = np.unwrap(np.array([item[2] for item in decoded], dtype=float))
        stamps = np.array([row[0] for row in poses], dtype=float)
        step = np.gradient(stamps)
        yaw_rate = np.gradient(yaw) / np.where(step > 1e-6, step, np.nan)
        yaw_rate = np.nan_to_num(yaw_rate)
    keep = np.isfinite(x) & np.isfinite(y) & np.isfinite(speed)
    times, x, y, speed, yaw_rate = (
        times[keep], x[keep], y[keep], speed[keep], yaw_rate[keep])
    if len(x) < 10:
        raise ValueError("쓸 만한 pose 표본이 너무 적습니다")

    # 라인 위 s 로 투영 (최근접 스테이션). 랩 분할과 그래프 x축이 s 여야
    # 계획 프로파일과 나란히 비교된다.
    line_xy = np.asarray(line_xy, dtype=float)
    line_s = np.asarray(line_s, dtype=float)
    track_length = float(line_s[-1] + np.hypot(*(line_xy[0] - line_xy[-1])))
    station = np.argmin(
        (x[:, None] - line_xy[None, :, 0]) ** 2 +
        (y[:, None] - line_xy[None, :, 1]) ** 2, axis=1)
    s = line_s[station]
    lateral = yaw_rate * speed
    if lateral_errors:
        error_times = np.array([row[0] for row in lateral_errors], dtype=float)
        error_values = np.array([row[1] for row in lateral_errors], dtype=float)
        lateral_error = np.interp(times, error_times, error_values)
    else:
        lateral_error = np.full(len(times), np.nan)

    laps = []
    start = 0
    for index in range(1, len(s)):
        if s[index - 1] > 0.8 * track_length and s[index] < 0.2 * track_length:
            laps.append((start, index))
            start = index
    laps.append((start, len(s)))
    laps = [(a, b) for a, b in laps if b - a > 20]

    def pack(a, b):
        stride = max(1, (b - a) // max_points)
        section = slice(a, b, stride)
        return {
            "t": np.round(times[section], 3).tolist(),
            "x": np.round(x[section], 4).tolist(),
            "y": np.round(y[section], 4).tolist(),
            "s": np.round(s[section], 3).tolist(),
            "v": np.round(speed[section], 3).tolist(),
            "a_lat": np.round(lateral[section], 3).tolist(),
            "required_mu": np.round(np.abs(lateral[section]) / G, 4).tolist(),
            "lat_err": [
                None if not math.isfinite(value) else round(value, 4)
                for value in lateral_error[section]],
            "lap_time": round(float(times[b - 1] - times[a]), 2),
        }

    return {
        "name": Path(bag_dir).name,
        "path": str(bag_dir),
        "duration": round(float(times[-1] - times[0]), 2),
        "has_lat_err": bool(lateral_errors),
        "laps": [pack(a, b) for a, b in laps],
        "all": pack(0, len(s)),
    }


# ----------------------------------------------------------- HTTP
class Handler(BaseHTTPRequestHandler):
    server_version = "racelinetuner/1.0"
    session = None
    resolver = None
    bagviz = None
    layers = None
    bag_roots = BAG_ROOTS
    lock = threading.Lock()

    def log_message(self, fmt, *args):        # 요청 로그로 콘솔을 덮지 않는다
        pass

    def _send(self, payload, status=200):
        body = json.dumps(payload, allow_nan=False).encode("utf-8")
        self.send_response(status)
        self.send_header("Content-Type", "application/json; charset=utf-8")
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        self.wfile.write(body)

    def _fail(self, message, status=400):
        self._send({"error": str(message)}, status)

    def do_GET(self):
        parsed = urllib.parse.urlparse(self.path)
        query = urllib.parse.parse_qs(parsed.query)
        if parsed.path in ("/", "/index.html"):
            # 매 요청마다 읽는다 — 화면을 고치고 새로고침만 하면 반영된다.
            body = PAGE_PATH.read_bytes()
            self.send_response(200)
            self.send_header("Content-Type", "text/html; charset=utf-8")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)
            return
        try:
            if parsed.path == "/api/open":
                with self.lock:
                    payload = self.session.payload()
                payload["layers"] = self.layers
                payload["bags_available"] = self.bagviz is not None
                self._send(payload)
                return
            if parsed.path == "/api/bags":
                self._send({"bags": find_bags(self.bag_roots)})
                return
            if parsed.path == "/api/bag":
                path = (query.get("path") or [""])[0]
                allowed = any(
                    Path(path).resolve().is_relative_to(Path(root).resolve())
                    for root in self.bag_roots if Path(root).is_dir())
                if not path or not allowed:
                    self._fail("허용된 백 폴더가 아닙니다")
                    return
                with self.lock:
                    out = self.session.state.output()
                self._send(read_bag(
                    self.bagviz, path, out["xy"], out["s"]))
                return
        except Exception as exc:
            self._fail(f"{type(exc).__name__}: {exc}", 500)
            return
        self._fail("없는 경로", 404)

    def do_POST(self):
        parsed = urllib.parse.urlparse(self.path)
        length = int(self.headers.get("Content-Length") or 0)
        try:
            body = json.loads(self.rfile.read(length) or b"{}")
        except ValueError as exc:
            self._fail(f"JSON 파싱 실패: {exc}")
            return
        try:
            if parsed.path == "/api/edit":
                with self.lock:
                    messages = self.session.apply(body.get("ops") or [])
                    payload = self.session.payload(messages)
                self._send(payload)
                return
            if parsed.path == "/api/save":
                with self.lock:
                    result = self.session.save()
                    payload = self.session.payload([
                        f"저장 완료: {result['csv']}",
                        "런타임 반영: cd ~/IFAC_ws && ./build_all.sh  →  "
                        f"rc {self.session.map_name}_tuned",
                    ])
                payload["saved"] = {
                    "csv": result["csv"], "state": result["state"],
                    "generation_id": result["generation_id"],
                    "sha256": result["sha256"][:16],
                }
                self._send(payload)
                return
        except Exception as exc:
            self._fail(f"{type(exc).__name__}: {exc}", 500)
            return
        self._fail("없는 경로", 404)


def main():
    parser = argparse.ArgumentParser(
        description="웹 레이스라인 튜너 (원본 보존, 튜닝본 따로 저장)")
    parser.add_argument(
        "map", nargs="?", default="",
        help="맵 이름 (my_track0808). 비우면 params.yaml 의 현행 라인")
    parser.add_argument("--port", type=int, default=8770)
    parser.add_argument("--no-browser", action="store_true")
    parser.add_argument(
        "--bag-dir", action="append", default=[],
        help="백을 추가로 찾을 폴더 (여러 번 지정 가능)")
    parser.add_argument(
        "--list", action="store_true", help="편집 가능한 라인만 출력하고 종료")
    arguments = parser.parse_args()

    if not PAGE_PATH.is_file():
        raise SystemExit(f"화면 파일이 없습니다: {PAGE_PATH}")
    resolver = load_resolver()
    bagviz = load_bagviz()
    if arguments.list:
        for name, entry in sorted(list_lines(resolver).items()):
            print("%-22s 원본=%s  튜닝본=%s" % (
                name, entry.get("original", "(없음)"),
                entry.get("tuned", "(없음)")))
        return

    map_name, base_csv = resolve_target(resolver, arguments.map)
    session = Session(resolver, map_name, base_csv)
    ok, detail = verify_map_layer_orientation(session.track)
    layers = map_layers(session.track, bagviz)

    print(f"[racelinetuner] 맵      {session.map_yaml}")
    print(f"[racelinetuner] 원본    {session.base_path}")
    print(f"[racelinetuner] 튜닝본  {session.tuned_path}"
          f"{'  (이어서 편집)' if session.resumed else '  (새로 생성)'}")
    print(f"[racelinetuner] 웨이포인트 {session.state.n}개, 랩 "
          f"{session.state.length0:.2f} m, corridor margin "
          f"{session.state.corridor_margin:.2f} m, v_max "
          f"{session.state.v_max:.2f} m/s")
    print(f"[racelinetuner] 배경    {'OK' if ok else '⚠️ 확인 필요'} — {detail}")
    if bagviz is None:
        print("[racelinetuner] ⚠️ tools/bagviz.py 를 못 불러와 실주행 겹쳐보기가 "
              "비활성입니다")
    for note in session.notes:
        print(f"[racelinetuner] {note}")

    Handler.session = session
    Handler.resolver = resolver
    Handler.bagviz = bagviz
    Handler.layers = layers
    Handler.bag_roots = tuple(BAG_ROOTS) + tuple(
        Path(directory).expanduser() for directory in arguments.bag_dir)

    server = ThreadingHTTPServer(("127.0.0.1", arguments.port), Handler)
    url = f"http://127.0.0.1:{arguments.port}/"
    print(f"\n[racelinetuner] {url}  (Ctrl+C 로 종료)")
    print("[racelinetuner] 원격이면: ssh -L "
          f"{arguments.port}:localhost:{arguments.port} <이 기계>")
    # 화면이 없는데(SSH) 브라우저를 열려고 하면 스레드에서 예외가 나 트레이스백이
    # 콘솔을 더럽힌다. 포트포워딩으로 쓰는 게 정상 경로이므로 조용히 건너뛴다.
    headless = sys.platform.startswith("linux") and not (
        os.environ.get("DISPLAY") or os.environ.get("WAYLAND_DISPLAY"))
    if arguments.no_browser:
        pass
    elif headless:
        print("[racelinetuner] DISPLAY 가 없어 브라우저를 열지 않습니다 — "
              "위 ssh -L 안내대로 로컬 브라우저에서 여세요")
    else:
        def open_browser():
            time.sleep(0.4)
            try:
                webbrowser.open(url)
            except Exception as exc:            # 브라우저가 없어도 서버는 돈다
                print(f"[racelinetuner] 브라우저를 못 열었습니다 ({exc}) — "
                      f"직접 {url} 을 여세요")
        threading.Thread(target=open_browser, daemon=True).start()
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        print("\n[racelinetuner] 종료")


if __name__ == "__main__":
    main()
