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
import importlib.util
import json
import os
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


def load_png_encoder():
    """배경 맵 PNG 인코더를 bagviz 에서 빌려온다 (없으면 벽 셀 폴백).

    이 도구가 bagviz 에 기대는 건 이제 이거 하나다 — 백 오버레이는 제거했다.
    """
    path = HERE / "bagviz.py"
    if not path.is_file():
        return None
    try:
        spec = importlib.util.spec_from_file_location(
            "racelinetuner_bagviz", path)
        module = importlib.util.module_from_spec(spec)
        sys.modules[spec.name] = module
        spec.loader.exec_module(module)
        return getattr(module, "png_gray_alpha", None)
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
        # 막대를 잡고 있는 동안의 기준 상태 (speed_drag 참조). None 이면 안 잡음.
        self.drag_base = None
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
    def drag_speed(self, operation):
        """막대 하나를 잡고 끄는 동안 들어오는 연속 편집.

        매 프레임을 독립 편집으로 처리하면 두 가지가 깨진다:
          - 되돌리기 한 번이 마우스 1 px 이 된다 (드래그 하나를 못 무른다)
          - 이전 프레임 결과 위에 falloff 를 또 얹어서 주변이 계단식으로 밀린다
            (손은 제자리인데 옆 막대만 계속 올라간다)
        그래서 잡는 순간의 v/pinned 를 기억해 두고 **매 프레임 거기서 다시**
        set_speed 를 한 번만 건다. 결과는 커서 위치의 함수라 왔다 갔다 해도
        같은 자리면 같은 값이다. 되돌리기 스냅샷은 잡을 때 딱 한 번 쌓는다.
        """
        phase = operation.get("phase", "move")
        if phase == "start" or self.drag_base is None:
            self.snapshot_undo()
            self.drag_base = (self.state.v.copy(), self.state.pinned.copy())
        base_speed, base_pinned = self.drag_base
        self.state.v = base_speed.copy()
        self.state.pinned = base_pinned.copy()
        self.state.set_speed(int(operation["index"]),
                             float(operation["value"]))
        if phase == "end":
            self.drag_base = None

    def apply(self, operations):
        state = self.state
        messages = []
        needs_solve = False
        for operation in operations:
            kind = operation.get("op")
            if kind == "undo":
                self.drag_base = None
                if not self.undo():
                    messages.append("되돌릴 편집이 없습니다")
                continue
            if kind == "speed_drag":
                self.drag_speed(operation)
                continue
            # 드래그 도중에 다른 편집이 끼어들면(단축키 등) 기준 상태를 버린다 —
            # 안 그러면 다음 프레임이 그 편집을 지운 상태에서 다시 시작한다.
            self.drag_base = None
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


def map_layers(track, png_gray_alpha):
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
    if png_gray_alpha is not None:
        # 주행 가능 = 밝게, 벽/unknown = 어둡게. ring 밖은 반투명으로 남겨
        # '트랙으로 인식되지 않은 free 공간'이 눈에 보이게 한다.
        gray = np.where(track.ring, 245, np.where(track.free_raw, 205, 70))
        alpha = np.where(track.ring, 255, np.where(track.free_raw, 90, 210))
        png = png_gray_alpha(gray.astype(np.uint8), alpha.astype(np.uint8))
        import base64
        layer["png"] = "data:image/png;base64," + \
            base64.b64encode(png).decode("ascii")
    else:
        # PNG 를 못 만들면(인코더 없음) 벽 셀을 좌표로 보낸다. 월드 좌표라
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


# ----------------------------------------------------------- HTTP
class Handler(BaseHTTPRequestHandler):
    server_version = "racelinetuner/1.0"
    session = None
    resolver = None
    layers = None
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
                self._send(payload)
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
        "--list", action="store_true", help="편집 가능한 라인만 출력하고 종료")
    arguments = parser.parse_args()

    if not PAGE_PATH.is_file():
        raise SystemExit(f"화면 파일이 없습니다: {PAGE_PATH}")
    resolver = load_resolver()
    png_encoder = load_png_encoder()
    if arguments.list:
        for name, entry in sorted(list_lines(resolver).items()):
            print("%-22s 원본=%s  튜닝본=%s" % (
                name, entry.get("original", "(없음)"),
                entry.get("tuned", "(없음)")))
        return

    map_name, base_csv = resolve_target(resolver, arguments.map)
    session = Session(resolver, map_name, base_csv)
    ok, detail = verify_map_layer_orientation(session.track)
    layers = map_layers(session.track, png_encoder)

    print(f"[racelinetuner] 맵      {session.map_yaml}")
    print(f"[racelinetuner] 원본    {session.base_path}")
    print(f"[racelinetuner] 튜닝본  {session.tuned_path}"
          f"{'  (이어서 편집)' if session.resumed else '  (새로 생성)'}")
    print(f"[racelinetuner] 웨이포인트 {session.state.n}개, 랩 "
          f"{session.state.length0:.2f} m, corridor margin "
          f"{session.state.corridor_margin:.2f} m, v_max "
          f"{session.state.v_max:.2f} m/s")
    print(f"[racelinetuner] 배경    {'OK' if ok else '⚠️ 확인 필요'} — {detail}")
    for note in session.notes:
        print(f"[racelinetuner] {note}")

    Handler.session = session
    Handler.resolver = resolver
    Handler.layers = layers

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
