#!/usr/bin/env python3
"""BagViz — F1TENTH rosbag2(.db3) 시각화 웹앱.

ROS 없이 동작 (sqlite3 + CDR 직접 디코딩, numpy만 필요).

    python3 tools/bagviz.py            # 서버 시작 + 브라우저 오픈
    python3 tools/bagviz.py --inspect <bag_dir|.db3>   # 파싱 테스트

브라우저에 .db3 파일(또는 백 폴더)을 드래그앤드롭하면 바로 시각화됨.
로컬 백 목록에서 클릭해서 열 수도 있음 (업로드 없이 경로로 직접 파싱).
"""
import argparse
import base64
import gzip
import json
import math
import os
import sqlite3
import struct
import sys
import tempfile
import time
import urllib.parse
import webbrowser
import zlib
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path

import numpy as np

HOME = Path.home()

# ---------------------------------------------------------------- CDR reader

class CDR:
    """XCDR1 little-endian reader. 4바이트 encapsulation 헤더 이후부터 정렬 계산."""
    __slots__ = ("b", "o")

    def __init__(self, buf):
        self.b = buf
        self.o = 4

    def align(self, n):
        r = (self.o - 4) % n
        if r:
            self.o += n - r

    def u8(self):
        v = self.b[self.o]
        self.o += 1
        return v

    def i32(self):
        self.align(4)
        v = struct.unpack_from("<i", self.b, self.o)[0]
        self.o += 4
        return v

    def u32(self):
        self.align(4)
        v = struct.unpack_from("<I", self.b, self.o)[0]
        self.o += 4
        return v

    def f32(self):
        self.align(4)
        v = struct.unpack_from("<f", self.b, self.o)[0]
        self.o += 4
        return v

    def f64(self):
        self.align(8)
        v = struct.unpack_from("<d", self.b, self.o)[0]
        self.o += 8
        return v

    def f32a(self, n):
        self.align(4)
        a = np.frombuffer(self.b, dtype="<f4", count=n, offset=self.o)
        self.o += 4 * n
        return a

    def f64n(self, n):
        self.align(8)
        a = struct.unpack_from("<%dd" % n, self.b, self.o)
        self.o += 8 * n
        return a

    def skip_f64(self, n):
        self.align(8)
        self.o += 8 * n

    def string(self):
        n = self.u32()
        s = self.b[self.o:self.o + n - 1].decode("utf-8", "replace") if n > 1 else ""
        self.o += n
        return s

    def time(self):
        sec = self.i32()
        ns = self.u32()
        return sec + ns * 1e-9

    def header(self):
        t = self.time()
        self.string()
        return t


def quat_yaw(qx, qy, qz, qw):
    return math.atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz))


# ------------------------------------------------------------- msg decoders

def dec_odom(b):
    c = CDR(b)
    c.header()
    c.string()                       # child_frame_id
    x, y, _z, qx, qy, qz, qw = c.f64n(7)
    c.skip_f64(36)                   # pose covariance
    vx, vy, _vz = c.f64n(3)
    return x, y, quat_yaw(qx, qy, qz, qw), math.hypot(vx, vy) * (1 if vx >= 0 else -1)


def dec_pose_stamped(b):
    c = CDR(b)
    c.header()
    x, y, _z, qx, qy, qz, qw = c.f64n(7)
    return x, y, quat_yaw(qx, qy, qz, qw)


def dec_point_stamped(b):
    c = CDR(b)
    c.header()
    x, y, _z = c.f64n(3)
    return x, y


def dec_ackermann(b):
    c = CDR(b)
    c.header()
    steer = c.f32()
    c.f32()                          # steering_angle_velocity
    speed = c.f32()
    return steer, speed


def dec_scan(b, beam_stride):
    c = CDR(b)
    c.header()
    amin = c.f32()
    c.f32()                          # angle_max
    ainc = c.f32()
    c.f32(); c.f32()                 # time_increment, scan_time
    rmin = c.f32()
    rmax = c.f32()
    n = c.u32()
    r = np.array(c.f32a(n)[::beam_stride], copy=True)
    bad = ~np.isfinite(r) | (r < rmin) | (r > rmax)
    r[bad] = 0.0
    return amin, ainc * beam_stride, np.round(r, 2)


def dec_float32(b):
    c = CDR(b)
    return c.f32()


def dec_float64(b):
    c = CDR(b)
    return c.f64()


def dec_bool(b):
    return CDR(b).u8() != 0


def dec_string(b):
    return CDR(b).string()


def dec_wpnt_array(b, pt_cap=0):
    """returns list of (s, x, y, vx)"""
    c = CDR(b)
    c.header()
    n = c.u32()
    stride = 1
    if pt_cap and n > pt_cap:
        stride = int(math.ceil(n / pt_cap))
    out = []
    for i in range(n):
        c.i32()                      # id
        f = c.f64n(10)               # s d x y d_r d_l psi kappa vx ax
        if i % stride == 0:
            out.append((f[0], f[2], f[3], f[8]))
    return out


def dec_grid(b):
    c = CDR(b)
    c.header()
    c.time()                         # map_load_time
    res = c.f32()
    w = c.u32()
    h = c.u32()
    ox, oy, _oz = c.f64n(3)
    c.skip_f64(4)                    # origin orientation (assume identity)
    n = c.u32()
    data = np.frombuffer(c.b, dtype=np.int8, count=n, offset=c.o).reshape(h, w)
    return res, ox, oy, w, h, data


def dec_log(b):
    c = CDR(b)
    c.time()
    level = c.u8()
    name = c.string()
    msg = c.string()
    return level, name, msg


def dec_tf_static(b):
    c = CDR(b)
    n = c.u32()
    out = []
    for _ in range(n):
        c.time()
        parent = c.string()
        child = c.string()
        x, y, z = c.f64n(3)
        c.skip_f64(4)
        out.append((parent, child, x, y, z))
    return out


def dec_obstacles(b):
    """f1tenth_perception_msgs/ObstacleArray → [(x, y, radius, is_dynamic)] (laser frame)"""
    c = CDR(b)
    c.header()
    n = c.u32()
    out = []
    for _ in range(min(n, 64)):
        c.i32()
        px, py, _pz = c.f64n(3)
        c.skip_f64(3)                # velocity
        dyn = c.u8() != 0
        c.f32()                      # confidence
        radius = c.f32()
        out.append((round(px, 2), round(py, 2), round(radius, 2), int(dyn)))
    return out


# ---------------------------------------------------------------- PNG writer

def png_gray_alpha(gray, alpha):
    """(h,w) uint8 ×2 → grayscale+alpha PNG bytes (stdlib zlib만 사용)."""
    h, w = gray.shape
    img = np.dstack([gray, alpha]).reshape(h, w * 2)
    raw = np.concatenate([np.zeros((h, 1), np.uint8), img], axis=1).tobytes()

    def chunk(tag, data):
        return (struct.pack(">I", len(data)) + tag + data +
                struct.pack(">I", zlib.crc32(tag + data) & 0xFFFFFFFF))

    ihdr = struct.pack(">IIBBBBB", w, h, 8, 4, 0, 0, 0)
    return (b"\x89PNG\r\n\x1a\n" + chunk(b"IHDR", ihdr) +
            chunk(b"IDAT", zlib.compress(raw, 6)) + chunk(b"IEND", b""))


# ------------------------------------------------------------- bag extraction

def _decimate(rows, cap):
    if cap and len(rows) > cap:
        stride = int(math.ceil(len(rows) / cap))
        return rows[::stride]
    return rows


def extract(db_path):
    t_start = time.time()
    con = sqlite3.connect("file:%s?mode=ro" % urllib.parse.quote(str(db_path)), uri=True)
    cur = con.cursor()
    topics = {}                      # name -> (id, type)
    for tid, name, typ in cur.execute("SELECT id, name, type FROM topics"):
        topics[name] = (tid, typ)

    counts = {}
    for tid, cnt in cur.execute("SELECT topic_id, COUNT(*) FROM messages GROUP BY topic_id"):
        counts[tid] = cnt

    row = cur.execute("SELECT MIN(timestamp), MAX(timestamp) FROM messages").fetchone()
    if not row or row[0] is None:
        raise ValueError("빈 백 파일입니다 (메시지 없음)")
    t0, t1 = row
    dur = (t1 - t0) / 1e9

    def count(name):
        return counts.get(topics[name][0], 0) if name in topics else 0

    def fetch(name, cap=0):
        """[(t_rel, blob)] — cap 지정 시 시간축 균등 다운샘플."""
        if name not in topics:
            return []
        tid = topics[name][0]
        n = counts.get(tid, 0)
        if n == 0:
            return []
        stride = int(math.ceil(n / cap)) if (cap and n > cap) else 1
        rows = []
        for i, (ts, data) in enumerate(cur.execute(
                "SELECT timestamp, data FROM messages WHERE topic_id=? ORDER BY timestamp", (tid,))):
            if i % stride == 0:
                rows.append(((ts - t0) / 1e9, data))
        return rows

    def first(name):
        rows = fetch(name)
        return rows[0] if rows else None

    out = {
        "name": Path(db_path).stem,
        "duration": round(dur, 2),
        "startEpoch": t0 / 1e9,
        "topics": sorted(
            [{"name": n, "type": t[1].split("/")[-1], "count": counts.get(t[0], 0),
              "hz": round(counts.get(t[0], 0) / dur, 1) if dur > 0 else 0}
             for n, t in topics.items()],
            key=lambda d: -d["count"]),
    }

    # --- pose (주행 궤적) : PF 우선, 없으면 /odom
    pose_topic = "/pf/pose/odom" if count("/pf/pose/odom") else "/odom"
    pose = {"t": [], "x": [], "y": [], "yaw": [], "v": []}
    for t, b in _decimate(fetch(pose_topic), 4500):
        try:
            x, y, yaw, v = dec_odom(b)
        except Exception:
            continue
        pose["t"].append(round(t, 3))
        pose["x"].append(round(x, 3))
        pose["y"].append(round(y, 3))
        pose["yaw"].append(round(yaw, 3))
        pose["v"].append(round(v, 2))
    out["pose"] = pose
    out["poseTopic"] = pose_topic

    # --- 실측 속도: /odom (eRPM 기반) 또는 monitor
    act = {"t": [], "v": []}
    if count("/odom"):
        for t, b in _decimate(fetch("/odom"), 2500):
            try:
                _x, _y, _yaw, v = dec_odom(b)
            except Exception:
                continue
            act["t"].append(round(t, 3))
            act["v"].append(round(v, 2))
        out["actTopic"] = "/odom"
    elif count("/monitor/current_speed"):
        for t, b in _decimate(fetch("/monitor/current_speed"), 2500):
            act["t"].append(round(t, 3))
            act["v"].append(round(dec_float32(b), 2))
        out["actTopic"] = "/monitor/current_speed"
    out["act"] = act

    # --- 제어 명령 (최종 mux 출력 우선)
    cmd_topic = next((n for n in ("/ackermann_cmd", "/autonomy/ackermann_cmd", "/drive", "/teleop")
                      if count(n)), None)
    cmd = {"t": [], "steer": [], "speed": []}
    if cmd_topic:
        for t, b in _decimate(fetch(cmd_topic), 2500):
            try:
                steer, speed = dec_ackermann(b)
            except Exception:
                continue
            cmd["t"].append(round(t, 3))
            cmd["steer"].append(round(math.degrees(steer), 2))
            cmd["speed"].append(round(speed, 2))
    out["cmd"] = cmd
    out["cmdTopic"] = cmd_topic or "-"

    # --- 횡오차
    lat = {"t": [], "v": []}
    for t, b in _decimate(fetch("/pitwall/lat_err"), 2500):
        lat["t"].append(round(t, 3))
        lat["v"].append(round(dec_float32(b), 3))
    out["lat"] = lat

    # --- E-stop 구간
    estop = []
    cur_on = None
    rows = fetch("/safety/emergency_stop")
    for t, b in rows:
        on = dec_bool(b)
        if on and cur_on is None:
            cur_on = t
        elif not on and cur_on is not None:
            estop.append([round(cur_on, 2), round(t, 2)])
            cur_on = None
    if cur_on is not None:
        estop.append([round(cur_on, 2), round(dur, 2)])
    out["estop"] = estop

    # --- LiDAR 스캔 (시간·빔 다운샘플)
    scans = []
    n_scan = count("/scan")
    beam_stride = 3
    for t, b in fetch("/scan", 240):
        try:
            amin, ainc, r = dec_scan(b, beam_stride)
        except Exception:
            continue
        scans.append({"t": round(t, 3), "a0": round(float(amin), 4),
                      "ai": round(float(ainc), 5), "r": [float(v) for v in r]})
    out["scans"] = scans
    out["scanTotal"] = n_scan

    # --- 지도
    g = first("/map")
    if g:
        try:
            res, ox, oy, w, h, data = dec_grid(g[1])
            gray = np.full(data.shape, 40, np.uint8)
            gray[data >= 65] = 208
            alpha = np.full(data.shape, 255, np.uint8)
            alpha[data < 0] = 0
            png = png_gray_alpha(gray, alpha)
            out["map"] = {"png": base64.b64encode(png).decode(), "res": res,
                          "ox": round(ox, 3), "oy": round(oy, 3), "w": w, "h": h}
        except Exception:
            out["map"] = None
    else:
        out["map"] = None

    # --- 글로벌 레이스라인 (첫 메시지)
    g = first("/global_waypoints")
    if g:
        wp = dec_wpnt_array(g[1])
        out["glb"] = {"x": [round(w[1], 3) for w in wp], "y": [round(w[2], 3) for w in wp],
                      "vx": [round(w[3], 2) for w in wp]}
    else:
        out["glb"] = None

    # --- 로컬/회피 경로 (시각 t에 해당하는 것만 그림)
    def wpnt_stream(name, msg_cap, pt_cap):
        arr = []
        for t, b in fetch(name, msg_cap):
            try:
                wp = dec_wpnt_array(b, pt_cap)
            except Exception:
                continue
            arr.append({"t": round(t, 3),
                        "p": [[round(w[1], 2), round(w[2], 2)] for w in wp]})
        return arr
    out["loc"] = wpnt_stream("/local_waypoints", 350, 50)
    out["ot"] = wpnt_stream("/planner/avoidance/otwpnts", 300, 50)

    # --- lookahead
    look = {"t": [], "x": [], "y": []}
    for t, b in _decimate(fetch("/control/lookahead_point"), 2000):
        try:
            x, y = dec_point_stamped(b)
        except Exception:
            continue
        look["t"].append(round(t, 3))
        look["x"].append(round(x, 2))
        look["y"].append(round(y, 2))
    out["look"] = look

    # --- 장애물 (laser frame → JS에서 차량 포즈로 변환)
    obs = []
    for t, b in fetch("/perception/obstacles", 600):
        try:
            lst = dec_obstacles(b)
        except Exception:
            continue
        if lst:
            obs.append({"t": round(t, 3), "o": lst})
    out["obs"] = obs

    # --- tf_static → base_link 대비 laser 오프셋
    laser_off = [0.0, 0.0]
    g = first("/tf_static")
    if g:
        try:
            for parent, child, x, y, _z in dec_tf_static(g[1]):
                if child == "laser":
                    laser_off = [round(x, 3), round(y, 3)]
        except Exception:
            pass
    out["laserOff"] = laser_off

    # --- 이벤트 로그 (pitwall + rosout WARN 이상)
    events = []
    for t, b in fetch("/pitwall/events"):
        events.append({"t": round(t, 2), "k": "EVT", "s": dec_string(b)[:160]})
    for t, b in fetch("/rosout"):
        try:
            level, name, msg = dec_log(b)
        except Exception:
            continue
        if level >= 40:
            events.append({"t": round(t, 2), "k": "ERR", "s": "[%s] %s" % (name, msg[:150])})
        elif level >= 30:
            events.append({"t": round(t, 2), "k": "WARN", "s": "[%s] %s" % (name, msg[:150])})
    events.sort(key=lambda e: e["t"])
    # 연속 중복 접기 — 숫자만 다른 반복 메시지(예: PF 경고 스팸)도 하나로 합침
    import re as _re
    norm = lambda s: _re.sub(r"[-+]?\d[\d.]*", "#", s)
    folded = []
    for e in events:
        if folded and folded[-1]["k"] == e["k"] and folded[-1]["_key"] == norm(e["s"]):
            folded[-1]["_n"] += 1
            folded[-1]["t2"] = e["t"]
        else:
            e["_key"] = norm(e["s"])
            e["_n"] = 1
            folded.append(e)
    for e in folded:
        if e["_n"] > 1:
            e["s"] = "%s  (×%d, ~%ss까지)" % (e["s"], e["_n"], e.get("t2", e["t"]))
        del e["_key"], e["_n"]
        e.pop("t2", None)
    out["events"] = folded[:600]

    con.close()
    out["parseSec"] = round(time.time() - t_start, 2)
    return out


# ----------------------------------------------------------- local bag scan

def find_local_bags():
    bags = []
    roots = [HOME, HOME / "IFAC_ws"]
    seen = set()
    for root in roots:
        if not root.is_dir():
            continue
        for meta in root.glob("*/metadata.yaml"):
            d = meta.parent
            if d in seen:
                continue
            seen.add(d)
            dbs = sorted(d.glob("*.db3"))
            if not dbs:
                continue
            db = max(dbs, key=lambda p: p.stat().st_size)
            st = db.stat()
            bags.append({"name": d.name, "path": str(db),
                         "sizeMB": round(st.st_size / 1e6, 1), "mtime": st.st_mtime})
    bags.sort(key=lambda b: -b["mtime"])
    return bags


def resolve_bag_path(p):
    """드롭/선택된 경로 → .db3 파일 경로. 홈 밖 경로는 거부."""
    path = Path(p).resolve()
    if not str(path).startswith(str(HOME)):
        raise ValueError("홈 디렉터리 밖 경로는 열 수 없습니다")
    if path.is_dir():
        dbs = sorted(path.glob("*.db3"))
        if not dbs:
            raise ValueError(".db3 파일이 폴더에 없습니다")
        path = max(dbs, key=lambda q: q.stat().st_size)
    if not path.is_file():
        raise ValueError("파일이 없습니다: %s" % path)
    return path


# -------------------------------------------------------------------- server

class Handler(BaseHTTPRequestHandler):
    def log_message(self, fmt, *args):
        sys.stderr.write("[bagviz] %s\n" % (fmt % args))

    def _send(self, code, body, ctype="application/json; charset=utf-8", gz=False):
        self.send_response(code)
        self.send_header("Content-Type", ctype)
        if gz:
            body = gzip.compress(body, 5)
            self.send_header("Content-Encoding", "gzip")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def _json(self, obj, code=200):
        self._send(code, json.dumps(obj, separators=(",", ":")).encode(), gz=True)

    def do_GET(self):
        url = urllib.parse.urlparse(self.path)
        if url.path == "/":
            self._send(200, PAGE.encode(), "text/html; charset=utf-8")
        elif url.path == "/bags":
            self._json(find_local_bags())
        elif url.path == "/parse_local":
            q = urllib.parse.parse_qs(url.query)
            try:
                path = resolve_bag_path(q["path"][0])
                self._json(extract(path))
            except Exception as e:
                self._json({"error": str(e)}, 400)
        else:
            self._send(404, b"not found", "text/plain")

    def do_POST(self):
        if self.path != "/parse":
            self._send(404, b"not found", "text/plain")
            return
        n = int(self.headers.get("Content-Length", 0))
        tmp = tempfile.NamedTemporaryFile(suffix=".db3", delete=False)
        try:
            remain = n
            while remain > 0:
                chunk = self.rfile.read(min(1 << 20, remain))
                if not chunk:
                    break
                tmp.write(chunk)
                remain -= len(chunk)
            tmp.close()
            d = extract(tmp.name)
            orig = self.headers.get("X-Filename")
            if orig:
                d["name"] = urllib.parse.unquote(orig).rsplit(".db3", 1)[0]
            self._json(d)
        except Exception as e:
            self._json({"error": str(e)}, 400)
        finally:
            try:
                os.unlink(tmp.name)
            except OSError:
                pass


# ---------------------------------------------------------------------- page

PAGE = r"""<!DOCTYPE html>
<html lang="ko"><head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>BagViz — F1TENTH 로스백 뷰어</title>
<style>
:root {
  color-scheme: dark;
  --page: #0d0d0d; --surface: #1a1a19; --surface2: #222220;
  --ink: #ffffff; --ink2: #c3c2b7; --muted: #898781;
  --grid: #2c2c2a; --axis: #383835; --border: rgba(255,255,255,0.10);
  --blue: #3987e5; --orange: #d95926; --aqua: #199e70; --yellow: #c98500;
  --magenta: #d55181; --violet: #9085e9; --red: #e66767;
  --warn: #fab219; --crit: #d03b3b; --good: #0ca30c;
}
* { margin:0; padding:0; box-sizing:border-box; }
html,body { height:100%; }
body { background:var(--page); color:var(--ink);
  font:14px/1.45 system-ui,-apple-system,"Segoe UI",sans-serif; overflow:hidden; }
button { font:inherit; color:var(--ink2); background:var(--surface2);
  border:1px solid var(--border); border-radius:6px; padding:4px 10px; cursor:pointer; }
button:hover { color:var(--ink); border-color:var(--muted); }
button.on { color:var(--ink); background:#2e2e2b; border-color:var(--muted); }

/* ---------- 시작 화면 ---------- */
#welcome { position:fixed; inset:0; display:flex; flex-direction:column;
  align-items:center; justify-content:center; gap:22px; padding:24px; z-index:50;
  background:var(--page); }
#welcome h1 { font-size:26px; font-weight:700; }
#welcome h1 span { color:var(--blue); }
#dropzone { width:min(640px,90vw); padding:52px 30px; text-align:center;
  border:2px dashed var(--axis); border-radius:16px; color:var(--ink2);
  transition:border-color .15s, background .15s; }
#dropzone.hover { border-color:var(--blue); background:rgba(57,135,229,0.08); }
#dropzone .big { font-size:17px; margin-bottom:6px; }
#dropzone .small { color:var(--muted); font-size:13px; }
#baglist { width:min(640px,90vw); max-height:34vh; overflow-y:auto;
  display:flex; flex-direction:column; gap:6px; }
#baglist .bag { display:flex; gap:12px; align-items:baseline; padding:9px 14px;
  background:var(--surface); border:1px solid var(--border); border-radius:8px;
  cursor:pointer; }
#baglist .bag:hover { border-color:var(--blue); }
#baglist .bag b { font-weight:600; }
#baglist .bag .meta { color:var(--muted); font-size:12px; margin-left:auto;
  font-variant-numeric:tabular-nums; }
#status { min-height:22px; color:var(--ink2); font-size:13px; }
#status.err { color:var(--red); }

/* ---------- 대시보드 ---------- */
#app { display:none; height:100%; flex-direction:column; }
#app.show { display:flex; }
header { display:flex; align-items:center; gap:14px; padding:8px 14px;
  background:var(--surface); border-bottom:1px solid var(--border); flex:0 0 auto; }
header .title { font-weight:700; font-size:15px; }
header .title span { color:var(--blue); }
header .sub { color:var(--muted); font-size:12px; }
#stats { display:flex; gap:8px; margin-left:auto; }
.tile { background:var(--surface2); border:1px solid var(--border); border-radius:8px;
  padding:4px 12px; text-align:right; min-width:86px; }
.tile .lb { font-size:10px; color:var(--muted); letter-spacing:.4px; }
.tile .vl { font-size:16px; font-weight:650; font-variant-numeric:tabular-nums; }
.tile .vl small { font-size:11px; color:var(--ink2); font-weight:400; }

#main { flex:1; display:flex; min-height:0; }
#trackWrap { flex:1; position:relative; min-width:0; }
#track { position:absolute; inset:0; width:100%; height:100%; display:block; cursor:grab; }
#track.pan { cursor:grabbing; }
#toggles { position:absolute; top:10px; left:10px; display:flex; flex-wrap:wrap;
  gap:6px; max-width:70%; }
#toggles button { font-size:12px; padding:3px 9px; background:rgba(26,26,25,.85); }
#fitBtn { position:absolute; right:10px; top:10px; font-size:12px;
  background:rgba(26,26,25,.85); }
#legend { position:absolute; left:10px; bottom:10px; background:rgba(26,26,25,.88);
  border:1px solid var(--border); border-radius:8px; padding:8px 12px; font-size:11px;
  color:var(--ink2); display:flex; flex-direction:column; gap:5px; }
#legend .row { display:flex; align-items:center; gap:7px; }
#legend .bar { width:74px; height:8px; border-radius:4px; }
#legend .sw { width:16px; height:3px; border-radius:2px; }
#legend .dot { width:9px; height:9px; border-radius:50%; }

#side { flex:0 0 390px; display:flex; flex-direction:column; min-height:0;
  border-left:1px solid var(--border); background:var(--surface); }
.chart { flex:0 0 auto; padding:8px 10px 2px; }
.chart .head { display:flex; align-items:baseline; gap:10px; margin:0 4px 2px; }
.chart .head .t { font-size:12px; color:var(--ink2); font-weight:600; }
.chart .head .lg { display:flex; gap:10px; font-size:11px; color:var(--muted); }
.chart .head .lg i { display:inline-block; width:12px; height:3px; border-radius:2px;
  vertical-align:middle; margin-right:4px; }
.chart canvas { width:100%; height:96px; display:block; cursor:crosshair; }
#events { flex:1 1 0; min-height:60px; display:flex; flex-direction:column;
  border-top:1px solid var(--border); }
#events .head { padding:7px 14px 4px; font-size:12px; color:var(--ink2); font-weight:600; }
#evlist { flex:1; overflow-y:auto; padding:0 8px 8px; font-size:12px; }
#evlist .ev { display:flex; gap:8px; padding:3px 6px; border-radius:5px; cursor:pointer;
  color:var(--ink2); align-items:baseline; }
#evlist .ev:hover { background:var(--surface2); }
#evlist .ev.cur { background:#26261f; }
#evlist .tt { color:var(--muted); font-variant-numeric:tabular-nums; flex:0 0 52px; }
#evlist .kk { flex:0 0 42px; font-size:10px; font-weight:700; text-align:center;
  border-radius:4px; padding:0 3px; }
#evlist .kk.EVT { color:var(--blue); border:1px solid var(--blue); }
#evlist .kk.WARN { color:var(--warn); border:1px solid var(--warn); }
#evlist .kk.ERR { color:var(--crit); border:1px solid var(--crit); }
#evlist .tx { overflow:hidden; text-overflow:ellipsis; white-space:nowrap; }

#bottom { flex:0 0 auto; display:flex; align-items:center; gap:10px;
  padding:8px 14px; background:var(--surface); border-top:1px solid var(--border); }
#playBtn { width:44px; font-size:15px; }
#speedBtn { min-width:52px; font-variant-numeric:tabular-nums; }
#timeline { flex:1; height:34px; cursor:pointer; }
#clock { font-variant-numeric:tabular-nums; color:var(--ink2); min-width:118px;
  text-align:right; font-size:13px; }
#openBtn { font-size:12px; }
details#topicsBox { position:absolute; right:10px; top:44px; z-index:5;
  background:rgba(26,26,25,.95); border:1px solid var(--border); border-radius:8px;
  max-width:360px; font-size:12px; }
details#topicsBox summary { padding:5px 12px; cursor:pointer; color:var(--ink2);
  list-style:none; }
details#topicsBox[open] summary { border-bottom:1px solid var(--border); }
#topicsTable { max-height:44vh; overflow-y:auto; padding:6px 10px; }
#topicsTable table { border-collapse:collapse; width:100%; }
#topicsTable td { padding:1px 8px 1px 0; color:var(--ink2); white-space:nowrap; }
#topicsTable td.n { font-variant-numeric:tabular-nums; text-align:right; color:var(--muted); }
#dragOverlay { position:fixed; inset:0; display:none; z-index:100;
  background:rgba(13,13,13,.75); border:3px dashed var(--blue);
  align-items:center; justify-content:center; font-size:20px; color:var(--ink); }
#dragOverlay.show { display:flex; }
</style></head>
<body>

<div id="welcome">
  <h1>Bag<span>Viz</span> <small style="font-weight:400;color:var(--muted);font-size:15px">F1TENTH 로스백 뷰어</small></h1>
  <div id="dropzone">
    <div class="big">여기에 <b>.db3</b> 파일 또는 <b>백 폴더</b>를 드래그앤드롭</div>
    <div class="small">클릭해서 파일 선택도 가능 · rosbag2 sqlite3 포맷</div>
    <input type="file" id="fileInput" accept=".db3" style="display:none">
  </div>
  <div id="status"></div>
  <div id="baglist"></div>
</div>

<div id="app">
  <header>
    <div>
      <div class="title">Bag<span>Viz</span> · <span id="bagName"></span></div>
      <div class="sub" id="bagSub"></div>
    </div>
    <button id="openBtn">다른 백 열기</button>
    <div id="stats">
      <div class="tile"><div class="lb">시간</div><div class="vl" id="stT">0.0<small> s</small></div></div>
      <div class="tile"><div class="lb">실측 속도</div><div class="vl" id="stV">–<small> m/s</small></div></div>
      <div class="tile"><div class="lb">명령 속도</div><div class="vl" id="stC">–<small> m/s</small></div></div>
      <div class="tile"><div class="lb">조향</div><div class="vl" id="stS">–<small> °</small></div></div>
      <div class="tile"><div class="lb">횡오차</div><div class="vl" id="stE">–<small> m</small></div></div>
    </div>
  </header>

  <div id="main">
    <div id="trackWrap">
      <canvas id="track"></canvas>
      <div id="toggles"></div>
      <button id="fitBtn">전체 보기 ⤢</button>
      <details id="topicsBox"><summary>토픽 목록</summary><div id="topicsTable"></div></details>
      <div id="legend"></div>
    </div>
    <div id="side">
      <div class="chart" id="ch0"><div class="head"><span class="t">속도 (m/s)</span>
        <span class="lg"><span><i style="background:var(--blue)"></i>명령</span>
        <span><i style="background:var(--orange)"></i>실측</span></span></div><canvas></canvas></div>
      <div class="chart" id="ch1"><div class="head"><span class="t">조향 명령 (°)</span></div><canvas></canvas></div>
      <div class="chart" id="ch2"><div class="head"><span class="t">횡오차 lat_err (m)</span></div><canvas></canvas></div>
      <div id="events"><div class="head">이벤트 · 경고 로그</div><div id="evlist"></div></div>
    </div>
  </div>

  <div id="bottom">
    <button id="playBtn">▶</button>
    <button id="speedBtn">1×</button>
    <canvas id="timeline"></canvas>
    <div id="clock">00:00.0 / 00:00.0</div>
  </div>
</div>

<div id="dragOverlay">새 백 파일을 놓으면 바로 열립니다</div>

<script>
"use strict";
/* ================= 공통 유틸 ================= */
const $ = id => document.getElementById(id);
const CSS = getComputedStyle(document.documentElement);
const C = name => CSS.getPropertyValue("--"+name).trim();
const fmt = (t) => {
  const m = Math.floor(t/60), s = t - m*60;
  return String(m).padStart(2,"0") + ":" + (s<10?"0":"") + s.toFixed(1);
};
const esc = s => s.replace(/&/g,"&amp;").replace(/</g,"&lt;").replace(/"/g,"&quot;");
function bisect(arr, t) {           // arr 오름차순, t 이하 마지막 인덱스
  let lo=0, hi=arr.length-1;
  if (hi<0 || t<arr[0]) return -1;
  while (lo<hi) { const mid=(lo+hi+1)>>1; if (arr[mid]<=t) lo=mid; else hi=mid-1; }
  return lo;
}
function lerpHex(h1,h2,f){
  const a=parseInt(h1.slice(1),16), b=parseInt(h2.slice(1),16);
  const c=[16,8,0].map(s=>Math.round(((a>>s&255)*(1-f)+(b>>s&255)*f)));
  return `rgb(${c[0]},${c[1]},${c[2]})`;
}
// 속도 램프 (어두움=느림 → 밝음=빠름)
const BLUE_STOPS=["#0d366b","#1c5cab","#3987e5","#9ec5f4"];
const ORANGE_STOPS=["#7a2f0f","#b0491e","#e37b4c","#ffc4a8"];
function ramp(stops,f){
  f=Math.max(0,Math.min(1,f)); const n=stops.length-1, i=Math.min(n-1,Math.floor(f*n));
  return lerpHex(stops[i],stops[i+1],f*n-i);
}

/* ================= 상태 ================= */
let D=null;                 // 파싱된 백 데이터
let tCur=0, playing=false, playSpeed=1, lastTs=0;
let vmax=1;                 // 속도 컬러 스케일 상한
const view={s:40,cx:0,cy:0};        // scale px/m, 중심(월드)
let follow=false;
const show={scan:true,glb:true,loc:true,ot:false,look:true,traj:true};
const SPEEDS=[0.5,1,2,4,8];
let mapImg=null;

/* ================= 로딩 ================= */
const statusEl=$("status");
function setStatus(msg,err){ statusEl.textContent=msg; statusEl.className=err?"err":""; }

function loadLocal(path,name){
  setStatus("파싱 중… "+name);
  fetch("/parse_local?path="+encodeURIComponent(path))
    .then(r=>r.json()).then(onData).catch(e=>setStatus("실패: "+e,true));
}
function uploadFile(file){
  setStatus("업로드 중… "+file.name+" ("+(file.size/1e6).toFixed(0)+" MB)");
  const xhr=new XMLHttpRequest();
  xhr.open("POST","/parse");
  xhr.setRequestHeader("X-Filename",encodeURIComponent(file.name));
  xhr.responseType="json";
  xhr.upload.onprogress=e=>{ if(e.lengthComputable)
    setStatus("업로드 "+Math.round(e.loaded/e.total*100)+"% … 이후 파싱까지 잠시 걸립니다"); };
  xhr.onload=()=>onData(xhr.response);
  xhr.onerror=()=>setStatus("업로드 실패",true);
  xhr.send(file);
}
function pickDb3(items){            // DataTransferItemList → .db3 File 찾기
  const files=[];
  const walk=(entry,done)=>{
    if(entry.isFile) entry.file(f=>{files.push(f);done();},done);
    else if(entry.isDirectory){
      entry.createReader().readEntries(es=>{
        let n=es.length; if(!n)return done();
        es.forEach(e=>walk(e,()=>{ if(--n===0) done(); }));
      });
    } else done();
  };
  return new Promise(res=>{
    let n=items.length; if(!n) return res(null);
    const entries=[...items].map(i=>i.webkitGetAsEntry&&i.webkitGetAsEntry()).filter(Boolean);
    if(!entries.length){ // fallback: 일반 파일
      const fs=[...items].map(i=>i.getAsFile&&i.getAsFile()).filter(Boolean);
      return res(fs.find(f=>f.name.endsWith(".db3"))||fs[0]||null);
    }
    n=entries.length;
    entries.forEach(e=>walk(e,()=>{ if(--n===0){
      const db=files.filter(f=>f.name.endsWith(".db3")).sort((a,b)=>b.size-a.size)[0];
      res(db||null);
    }}));
  });
}
function handleDrop(ev){
  ev.preventDefault();
  $("dragOverlay").classList.remove("show");
  $("dropzone").classList.remove("hover");
  pickDb3(ev.dataTransfer.items).then(f=>{
    if(!f){ setStatus(".db3 파일을 찾지 못했습니다",true); return; }
    if(D){ $("welcome").style.display="flex"; $("app").classList.remove("show"); D=null; }
    uploadFile(f);
  });
}
window.addEventListener("dragover",e=>{ e.preventDefault();
  if(D) $("dragOverlay").classList.add("show"); else $("dropzone").classList.add("hover"); });
window.addEventListener("dragleave",e=>{ if(e.relatedTarget===null){
  $("dragOverlay").classList.remove("show"); $("dropzone").classList.remove("hover"); }});
window.addEventListener("drop",handleDrop);
$("dropzone").addEventListener("click",()=>$("fileInput").click());
$("fileInput").addEventListener("change",e=>{ if(e.target.files[0]) uploadFile(e.target.files[0]); });
$("openBtn").addEventListener("click",()=>{
  D=null; playing=false; $("app").classList.remove("show");
  $("welcome").style.display="flex"; setStatus(""); loadBagList();
});
function loadBagList(){
  fetch("/bags").then(r=>r.json()).then(bags=>{
    const el=$("baglist"); el.innerHTML="";
    if(!bags.length) return;
    const h=document.createElement("div");
    h.style.cssText="color:var(--muted);font-size:12px;margin:4px 2px";
    h.textContent="또는 이 컴퓨터의 백에서 선택 (클릭 시 업로드 없이 바로 열림)";
    el.appendChild(h);
    bags.forEach(b=>{
      const d=new Date(b.mtime*1000);
      const row=document.createElement("div"); row.className="bag";
      row.innerHTML="<b>"+b.name+"</b><span class='meta'>"+b.sizeMB+" MB · "
        +d.toLocaleString("ko-KR",{month:"2-digit",day:"2-digit",hour:"2-digit",minute:"2-digit"})+"</span>";
      row.onclick=()=>loadLocal(b.path,b.name);
      el.appendChild(row);
    });
  });
}
loadBagList();

/* ================= 데이터 수신 ================= */
function onData(d){
  if(!d){ setStatus("응답 없음",true); return; }
  if(d.error){ setStatus("오류: "+d.error,true); return; }
  D=d; tCur=0; playing=false; mapImg=null;
  vmax=1;
  if(D.glb) D.glb.vx.forEach(v=>{ if(v>vmax)vmax=v; });
  D.pose.v.forEach(v=>{ if(Math.abs(v)>vmax)vmax=Math.abs(v); });
  vmax=Math.ceil(vmax*10)/10;
  if(D.map){
    mapImg=new Image();
    mapImg.onload=()=>drawAll();
    mapImg.src="data:image/png;base64,"+D.map.png;
  }
  $("welcome").style.display="none";
  $("app").classList.add("show");
  $("bagName").textContent=D.name;
  const st=new Date(D.startEpoch*1000);
  $("bagSub").textContent=fmt(D.duration)+" · "+st.toLocaleString("ko-KR")
    +" · pose:"+D.poseTopic+" · cmd:"+D.cmdTopic+" · 파싱 "+D.parseSec+"s";
  buildTopics(); buildToggles(); buildLegend(); buildEvents();
  fitView(); sizeCanvases(); buildChartData(); drawAll();
}

/* ================= 뷰(지도) ================= */
const track=$("track"), tctx=track.getContext("2d");
function fitView(){
  let xs=[],ys=[];
  if(D.map){ xs.push(D.map.ox, D.map.ox+D.map.w*D.map.res); ys.push(D.map.oy, D.map.oy+D.map.h*D.map.res); }
  if(D.pose.x.length){ xs.push(Math.min(...D.pose.x),Math.max(...D.pose.x));
                       ys.push(Math.min(...D.pose.y),Math.max(...D.pose.y)); }
  if(!xs.length){ xs=[-5,5]; ys=[-5,5]; }
  const w=Math.max(...xs)-Math.min(...xs), h=Math.max(...ys)-Math.min(...ys);
  view.cx=(Math.max(...xs)+Math.min(...xs))/2; view.cy=(Math.max(...ys)+Math.min(...ys))/2;
  const r=track.getBoundingClientRect();
  view.s=Math.min(r.width/(w+2), r.height/(h+2))||40;
  follow=false; updateFollowBtn();
}
const W2SX=x=>(x-view.cx)*view.s+track.width/2/DPR;
const W2SY=y=>track.height/2/DPR-(y-view.cy)*view.s;
let DPR=window.devicePixelRatio||1;

function sizeCanvases(){
  DPR=window.devicePixelRatio||1;
  const r=$("trackWrap").getBoundingClientRect();
  track.width=r.width*DPR; track.height=r.height*DPR;
  tctx.setTransform(DPR,0,0,DPR,0,0);
  charts.forEach(c=>{
    const rc=c.cv.getBoundingClientRect();
    c.cv.width=rc.width*DPR; c.cv.height=rc.height*DPR;
    c.ctx.setTransform(DPR,0,0,DPR,0,0);
    c.w=rc.width; c.h=rc.height; c.base=null;
  });
  const rt=tl.getBoundingClientRect();
  tl.width=rt.width*DPR; tl.height=rt.height*DPR;
  tlctx.setTransform(DPR,0,0,DPR,0,0);
}
window.addEventListener("resize",()=>{ if(D){ sizeCanvases(); drawAll(); }});

// 팬/줌
let dragging=false, dragStart=null;
track.addEventListener("mousedown",e=>{ dragging=true; track.classList.add("pan");
  dragStart={x:e.clientX,y:e.clientY,cx:view.cx,cy:view.cy}; });
window.addEventListener("mousemove",e=>{ if(!dragging)return;
  follow=false; updateFollowBtn();
  view.cx=dragStart.cx-(e.clientX-dragStart.x)/view.s;
  view.cy=dragStart.cy+(e.clientY-dragStart.y)/view.s; drawTrack(); });
window.addEventListener("mouseup",()=>{ dragging=false; track.classList.remove("pan"); });
track.addEventListener("wheel",e=>{ e.preventDefault();
  const r=track.getBoundingClientRect();
  const mx=e.clientX-r.left, my=e.clientY-r.top;
  const wx=(mx-r.width/2)/view.s+view.cx, wy=(r.height/2-my)/view.s+view.cy;
  const f=Math.exp(-e.deltaY*0.0015);
  view.s=Math.max(2,Math.min(600,view.s*f));
  view.cx=wx-(mx-r.width/2)/view.s; view.cy=wy+(my-r.height/2)/view.s;
  drawTrack();
},{passive:false});
track.addEventListener("dblclick",fitView2);
function fitView2(){ fitView(); drawTrack(); }
$("fitBtn").addEventListener("click",fitView2);

/* ================= 토글/범례/토픽 ================= */
let followBtn=null;
function buildToggles(){
  const box=$("toggles"); box.innerHTML="";
  const defs=[
    ["traj","주행 궤적", D.pose.t.length>0],
    ["glb","레이스라인", !!D.glb],
    ["loc","로컬 경로", D.loc.length>0],
    ["ot","회피 경로", D.ot.length>0],
    ["scan","LiDAR", D.scans.length>0],
    ["look","룩어헤드", D.look.t.length>0],
  ];
  defs.forEach(([k,label,avail])=>{
    if(!avail){ show[k]=false; return; }
    const b=document.createElement("button");
    b.textContent=label; b.className=show[k]?"on":"";
    b.onclick=()=>{ show[k]=!show[k]; b.className=show[k]?"on":""; drawTrack(); };
    box.appendChild(b);
  });
  followBtn=document.createElement("button");
  followBtn.textContent="차량 추적";
  followBtn.onclick=()=>{ follow=!follow; updateFollowBtn(); drawTrack(); };
  box.appendChild(followBtn); updateFollowBtn();
}
function updateFollowBtn(){ if(followBtn) followBtn.className=follow?"on":""; }

function buildLegend(){
  const el=$("legend"); el.innerHTML="";
  const rows=[];
  if(show.traj) rows.push(`<div class="row"><span class="bar" style="background:linear-gradient(90deg,${BLUE_STOPS.join(",")})"></span>주행 속도 0–${vmax} m/s</div>`);
  if(D.glb) rows.push(`<div class="row"><span class="bar" style="background:linear-gradient(90deg,${ORANGE_STOPS.join(",")})"></span>레이스라인 목표속도</div>`);
  if(D.loc.length) rows.push(`<div class="row"><span class="sw" style="background:${C("magenta")}"></span>로컬 경로 (추종 중)</div>`);
  if(D.ot.length) rows.push(`<div class="row"><span class="sw" style="background:${C("violet")}"></span>회피 경로 otwpnts</div>`);
  if(D.scans.length) rows.push(`<div class="row"><span class="dot" style="background:${C("ink2")}"></span>LiDAR 스캔</div>`);
  if(D.look.t.length) rows.push(`<div class="row"><span class="dot" style="background:${C("yellow")}"></span>룩어헤드 점</div>`);
  if(D.obs.length) rows.push(`<div class="row"><span class="dot" style="border:2px solid ${C("red")};background:none"></span>장애물</div>`);
  el.innerHTML=rows.join("");
}
function buildTopics(){
  const rows=D.topics.filter(t=>t.count>0).map(t=>
    `<tr><td>${t.name}</td><td class="n">${t.count.toLocaleString()}</td><td class="n">${t.hz} Hz</td></tr>`);
  $("topicsTable").innerHTML="<table>"+rows.join("")+"</table>";
}

/* ================= 트랙 캔버스 ================= */
function drawTrack(){
  const w=track.width/DPR, h=track.height/DPR;
  tctx.clearRect(0,0,w,h);
  if(!D) return;
  if(follow){
    const i=bisect(D.pose.t,tCur);
    if(i>=0){ view.cx=D.pose.x[i]; view.cy=D.pose.y[i]; }
  }
  // 지도
  if(mapImg&&D.map){
    tctx.save();
    tctx.imageSmoothingEnabled=view.s<12;
    tctx.translate(W2SX(D.map.ox), W2SY(D.map.oy));
    tctx.scale(view.s*D.map.res, -view.s*D.map.res);
    tctx.drawImage(mapImg,0,0);
    tctx.restore();
  }
  // 글로벌 레이스라인 (목표속도 램프)
  if(show.glb&&D.glb){
    const g=D.glb;
    for(let i=1;i<g.x.length;i++){
      tctx.strokeStyle=ramp(ORANGE_STOPS,g.vx[i]/vmax);
      tctx.lineWidth=2; tctx.beginPath();
      tctx.moveTo(W2SX(g.x[i-1]),W2SY(g.y[i-1]));
      tctx.lineTo(W2SX(g.x[i]),W2SY(g.y[i])); tctx.stroke();
    }
  }
  // 주행 궤적: 미래 흐리게, 과거 속도 램프
  if(show.traj&&D.pose.t.length){
    const p=D.pose, cur=bisect(p.t,tCur);
    tctx.lineWidth=1.5; tctx.strokeStyle="rgba(137,135,129,0.28)";
    tctx.beginPath();
    for(let i=Math.max(cur,0);i<p.x.length;i++){
      const X=W2SX(p.x[i]),Y=W2SY(p.y[i]);
      i===Math.max(cur,0)?tctx.moveTo(X,Y):tctx.lineTo(X,Y);
    }
    tctx.stroke();
    // 색상 빈(20단계) 단위로 배칭해서 stroke 횟수 최소화
    if(!p._bin){ p._bin=p.v.map(v=>Math.min(19,Math.round(Math.abs(v)/vmax*19))); }
    tctx.lineWidth=3; tctx.lineCap="round";
    let i=1;
    while(i<=cur){
      const b=p._bin[i];
      tctx.strokeStyle=ramp(BLUE_STOPS,b/19);
      tctx.beginPath();
      tctx.moveTo(W2SX(p.x[i-1]),W2SY(p.y[i-1]));
      while(i<=cur&&p._bin[i]===b){ tctx.lineTo(W2SX(p.x[i]),W2SY(p.y[i])); i++; }
      tctx.stroke();
    }
    tctx.lineCap="butt";
  }
  const pi=bisect(D.pose.t,tCur);
  const px=pi>=0?D.pose.x[pi]:0, py=pi>=0?D.pose.y[pi]:0, pyaw=pi>=0?D.pose.yaw[pi]:0;
  // 회피/로컬 경로 (현재 시각)
  [["ot","violet",show.ot,[6,5]],["loc","magenta",show.loc,null]].forEach(([key,col,on,dash])=>{
    if(!on||!D[key].length) return;
    const arr=D[key], ts=arr.map(a=>a.t), i=bisect(ts,tCur);
    if(i<0||tCur-ts[i]>1.5) return;
    tctx.strokeStyle=C(col); tctx.lineWidth=2.5;
    if(dash) tctx.setLineDash(dash);
    tctx.beginPath();
    arr[i].p.forEach((q,j)=>{ const X=W2SX(q[0]),Y=W2SY(q[1]);
      j?tctx.lineTo(X,Y):tctx.moveTo(X,Y); });
    tctx.stroke(); tctx.setLineDash([]);
  });
  // LiDAR 스캔 (차량 포즈 기준 변환)
  if(show.scan&&D.scans.length&&pi>=0){
    const ts=D.scans.map(s=>s.t), i=bisect(ts,tCur);
    if(i>=0&&Math.abs(tCur-ts[i])<2){
      const s=D.scans[i];
      const lx=px+D.laserOff[0]*Math.cos(pyaw)-D.laserOff[1]*Math.sin(pyaw);
      const ly=py+D.laserOff[0]*Math.sin(pyaw)+D.laserOff[1]*Math.cos(pyaw);
      tctx.fillStyle="rgba(195,194,183,0.75)";
      for(let j=0;j<s.r.length;j++){
        const r=s.r[j]; if(!r) continue;
        const a=pyaw+s.a0+s.ai*j;
        tctx.fillRect(W2SX(lx+r*Math.cos(a))-1, W2SY(ly+r*Math.sin(a))-1, 2,2);
      }
    }
  }
  // 장애물
  if(D.obs.length&&pi>=0){
    const ts=D.obs.map(o=>o.t), i=bisect(ts,tCur);
    if(i>=0&&Math.abs(tCur-ts[i])<1){
      tctx.strokeStyle=C("red"); tctx.lineWidth=2;
      D.obs[i].o.forEach(([ox,oy,r,dyn])=>{
        const wx2=px+ox*Math.cos(pyaw)-oy*Math.sin(pyaw);
        const wy2=py+ox*Math.sin(pyaw)+oy*Math.cos(pyaw);
        tctx.beginPath();
        tctx.arc(W2SX(wx2),W2SY(wy2),Math.max(4,(r||0.15)*view.s),0,6.283);
        tctx.setLineDash(dyn?[4,3]:[]); tctx.stroke(); tctx.setLineDash([]);
      });
    }
  }
  // 룩어헤드
  if(show.look&&D.look.t.length&&pi>=0){
    const i=bisect(D.look.t,tCur);
    if(i>=0&&Math.abs(tCur-D.look.t[i])<1){
      const lx=D.look.x[i], ly=D.look.y[i];
      tctx.strokeStyle="rgba(201,133,0,0.5)"; tctx.lineWidth=1.5;
      tctx.beginPath(); tctx.moveTo(W2SX(px),W2SY(py)); tctx.lineTo(W2SX(lx),W2SY(ly)); tctx.stroke();
      tctx.fillStyle=C("yellow");
      tctx.beginPath(); tctx.arc(W2SX(lx),W2SY(ly),5,0,6.283); tctx.fill();
    }
  }
  // 차량
  if(pi>=0){
    const L=0.45*view.s, Wd=0.27*view.s;
    tctx.save();
    tctx.translate(W2SX(px),W2SY(py)); tctx.rotate(-pyaw);
    tctx.fillStyle="#ffffff"; tctx.strokeStyle=C("blue"); tctx.lineWidth=2;
    tctx.beginPath();
    tctx.moveTo(L*0.6,0); tctx.lineTo(-L*0.4,Wd/2); tctx.lineTo(-L*0.25,0);
    tctx.lineTo(-L*0.4,-Wd/2); tctx.closePath();
    tctx.fill(); tctx.stroke();
    tctx.restore();
  }
  // 축척
  const m10=view.s*1;
  tctx.strokeStyle=C("muted"); tctx.lineWidth=2;
  tctx.beginPath(); tctx.moveTo(w-20-m10,h-16); tctx.lineTo(w-20,h-16); tctx.stroke();
  tctx.fillStyle=C("muted"); tctx.font="11px system-ui"; tctx.textAlign="center";
  tctx.fillText("1 m", w-20-m10/2, h-24);
}

/* ================= 차트 ================= */
const charts=[];
function mkChart(id,series,opts){
  const box=$(id), cv=box.querySelector("canvas");
  const c={cv,ctx:cv.getContext("2d"),series,base:null,w:0,h:0,hover:null,
           sym:opts&&opts.sym};
  cv.addEventListener("mousemove",e=>{
    const r=cv.getBoundingClientRect(); c.hover=e.clientX-r.left;
    if(e.buttons&1) seekFromChart(c,e); drawCursor(c);
  });
  cv.addEventListener("mouseleave",()=>{ c.hover=null; drawCursor(c); });
  cv.addEventListener("mousedown",e=>seekFromChart(c,e));
  charts.push(c);
  return c;
}
function seekFromChart(c,e){
  const r=c.cv.getBoundingClientRect();
  seek(Math.max(0,Math.min(1,(e.clientX-r.left-PADL)/(c.w-PADL-4)))*D.duration);
}
const PADL=34;
function buildChartData(){
  charts.length=0;
  mkChart("ch0",[
    {t:D.cmd.t,v:D.cmd.speed,color:C("blue")},
    {t:D.act.t,v:D.act.v,color:C("orange")}]);
  mkChart("ch1",[{t:D.cmd.t,v:D.cmd.steer,color:C("blue")}],{sym:true});
  mkChart("ch2",[{t:D.lat.t,v:D.lat.v,color:C("aqua")}],{sym:true});
  sizeCanvases();
}
function niceStep(range){
  const raw=range/4, p=Math.pow(10,Math.floor(Math.log10(raw||1)));
  for(const m of [1,2,5,10]) if(raw<=m*p) return m*p;
  return 10*p;
}
function drawChartBase(c){
  const {ctx,w,h,series}=c;
  let lo=Infinity, hi=-Infinity;
  series.forEach(s=>s.v.forEach(v=>{ if(v<lo)lo=v; if(v>hi)hi=v; }));
  if(!isFinite(lo)){ lo=0; hi=1; }
  if(c.sym){ const m=Math.max(Math.abs(lo),Math.abs(hi),0.1); lo=-m; hi=m; }
  if(hi-lo<1e-6){ hi+=0.5; lo-=0.5; }
  const pad=(hi-lo)*0.08; lo-=pad; hi+=pad;
  c.lo=lo; c.hi=hi;
  const X=t=>PADL+(t/D.duration)*(w-PADL-4);
  const Y=v=>h-14-((v-lo)/(hi-lo))*(h-20);
  c.X=X; c.Y=Y;
  ctx.clearRect(0,0,w,h);
  // e-stop 음영
  ctx.fillStyle="rgba(208,59,59,0.13)";
  D.estop.forEach(([a,b])=>ctx.fillRect(X(a),4,Math.max(2,X(b)-X(a)),h-18));
  // 그리드+눈금
  ctx.font="10px system-ui"; ctx.fillStyle=C("muted"); ctx.textAlign="right";
  const step=niceStep(hi-lo);
  for(let v=Math.ceil(lo/step)*step; v<=hi; v+=step){
    const y=Y(v);
    ctx.strokeStyle=Math.abs(v)<1e-9?C("axis"):C("grid"); ctx.lineWidth=1;
    ctx.beginPath(); ctx.moveTo(PADL,y); ctx.lineTo(w-4,y); ctx.stroke();
    ctx.fillText(v.toFixed(step<1?1:0), PADL-4, y+3);
  }
  // 시리즈
  series.forEach(s=>{
    ctx.strokeStyle=s.color; ctx.lineWidth=1.6; ctx.beginPath();
    for(let i=0;i<s.t.length;i++){
      const x=X(s.t[i]), y=Y(s.v[i]);
      i?ctx.lineTo(x,y):ctx.moveTo(x,y);
    }
    ctx.stroke();
  });
  c.base=document.createElement("canvas");
  c.base.width=c.cv.width; c.base.height=c.cv.height;
  c.base.getContext("2d").drawImage(c.cv,0,0);
}
function drawCursor(c){
  if(!c.base) drawChartBase(c);
  const {ctx,w,h}=c;
  ctx.clearRect(0,0,w,h);
  ctx.drawImage(c.base,0,0,w,h);
  // 재생 커서
  const x=c.X(tCur);
  ctx.strokeStyle=C("ink2"); ctx.lineWidth=1;
  ctx.beginPath(); ctx.moveTo(x,4); ctx.lineTo(x,h-14); ctx.stroke();
  // 호버 툴팁
  if(c.hover!==null&&c.hover>PADL){
    const t=Math.max(0,Math.min(1,(c.hover-PADL)/(w-PADL-4)))*D.duration;
    ctx.strokeStyle="rgba(137,135,129,0.55)";
    ctx.beginPath(); ctx.moveTo(c.hover,4); ctx.lineTo(c.hover,h-14); ctx.stroke();
    const vals=c.series.map(s=>{
      const i=bisect(s.t,t); return i>=0?s.v[i]:null;
    });
    let tx="t="+t.toFixed(1)+"s";
    vals.forEach(v=>{ if(v!==null) tx+="  "+v.toFixed(2); });
    ctx.font="11px system-ui";
    const tw=ctx.measureText(tx).width+10;
    let bx=c.hover+8; if(bx+tw>w) bx=c.hover-tw-8;
    ctx.fillStyle="rgba(13,13,13,0.9)"; ctx.fillRect(bx,6,tw,16);
    ctx.strokeStyle=C("border"); ctx.strokeRect(bx,6,tw,16);
    ctx.fillStyle=C("ink"); ctx.textAlign="left"; ctx.fillText(tx,bx+5,18);
    // 마크
    c.series.forEach((s,si)=>{
      const i=bisect(s.t,t);
      if(i>=0){ ctx.fillStyle=s.color;
        ctx.beginPath(); ctx.arc(c.hover,c.Y(s.v[i]),3,0,6.283); ctx.fill(); }
    });
  }
  // x축 시간 라벨
  ctx.fillStyle=C("muted"); ctx.font="10px system-ui"; ctx.textAlign="left";
  ctx.fillText("0:00",PADL,h-3);
  ctx.textAlign="right"; ctx.fillText(fmt(D.duration),w-4,h-3);
}

/* ================= 타임라인 ================= */
const tl=$("timeline"), tlctx=tl.getContext("2d");
function drawTimeline(){
  const w=tl.width/DPR, h=tl.height/DPR;
  tlctx.clearRect(0,0,w,h);
  const X=t=>(t/D.duration)*w;
  tlctx.fillStyle=C("surface2"); tlctx.fillRect(0,10,w,h-20);
  // 속도 미니 프로파일
  if(D.act.t.length){
    tlctx.strokeStyle="rgba(217,89,38,0.55)"; tlctx.lineWidth=1;
    tlctx.beginPath();
    let m=0.1; D.act.v.forEach(v=>{ if(Math.abs(v)>m)m=Math.abs(v); });
    for(let i=0;i<D.act.t.length;i++){
      const x=X(D.act.t[i]), y=h-11-(Math.abs(D.act.v[i])/m)*(h-22);
      i?tlctx.lineTo(x,y):tlctx.moveTo(x,y);
    }
    tlctx.stroke();
  }
  tlctx.fillStyle="rgba(208,59,59,0.3)";
  D.estop.forEach(([a,b])=>tlctx.fillRect(X(a),10,Math.max(2,X(b)-X(a)),h-20));
  // 이벤트 틱
  D.events.forEach(e=>{
    tlctx.fillStyle=e.k==="ERR"?C("crit"):e.k==="WARN"?C("warn"):C("blue");
    tlctx.fillRect(X(e.t),h-9,1.5,6);
  });
  // 진행
  tlctx.fillStyle="rgba(57,135,229,0.25)"; tlctx.fillRect(0,10,X(tCur),h-20);
  tlctx.fillStyle=C("ink");
  tlctx.fillRect(X(tCur)-1,4,2,h-8);
}
function tlSeek(e){
  const r=tl.getBoundingClientRect();
  seek(Math.max(0,Math.min(1,(e.clientX-r.left)/r.width))*D.duration);
}
tl.addEventListener("mousedown",e=>{ tlSeek(e);
  const mv=ev=>tlSeek(ev), up=()=>{ window.removeEventListener("mousemove",mv);
    window.removeEventListener("mouseup",up); };
  window.addEventListener("mousemove",mv); window.addEventListener("mouseup",up); });

/* ================= 이벤트 로그 ================= */
function buildEvents(){
  const el=$("evlist"); el.innerHTML="";
  D.events.forEach((e,i)=>{
    const d=document.createElement("div"); d.className="ev"; d.dataset.i=i;
    d.innerHTML=`<span class="tt">${fmt(e.t)}</span><span class="kk ${e.k}">${e.k}</span>`
      +`<span class="tx" title="${esc(e.s)}">${esc(e.s)}</span>`;
    d.onclick=()=>seek(e.t);
    el.appendChild(d);
  });
}
let lastEvIdx=-1;
function highlightEvent(){
  const ts=D.events.map(e=>e.t), i=bisect(ts,tCur);
  if(i===lastEvIdx) return;
  const el=$("evlist");
  if(lastEvIdx>=0&&el.children[lastEvIdx]) el.children[lastEvIdx].classList.remove("cur");
  if(i>=0&&el.children[i]){ el.children[i].classList.add("cur");
    if(playing) el.children[i].scrollIntoView({block:"nearest"}); }
  lastEvIdx=i;
}

/* ================= 재생 ================= */
function seek(t){ tCur=Math.max(0,Math.min(D.duration,t)); drawAll(); }
function drawAll(){
  if(!D) return;
  drawTrack(); charts.forEach(drawCursor); drawTimeline(); updateStats(); highlightEvent();
}
function updateStats(){
  $("stT").innerHTML=tCur.toFixed(1)+"<small> s</small>";
  const g=(o,tArr,vArr,dec,unit)=>{
    const i=bisect(tArr,tCur);
    o.innerHTML=(i>=0?vArr[i].toFixed(dec):"–")+"<small> "+unit+"</small>";
  };
  g($("stV"),D.act.t,D.act.v,2,"m/s");
  g($("stC"),D.cmd.t,D.cmd.speed,2,"m/s");
  g($("stS"),D.cmd.t,D.cmd.steer,1,"°");
  g($("stE"),D.lat.t,D.lat.v,2,"m");
  $("clock").textContent=fmt(tCur)+" / "+fmt(D.duration);
}
function tick(ts){
  if(playing&&D){
    if(lastTs) tCur+=Math.min(0.2,(ts-lastTs)/1000)*playSpeed;
    if(tCur>=D.duration){ tCur=D.duration; setPlay(false); }
    drawAll();
  }
  lastTs=ts;
  requestAnimationFrame(tick);
}
requestAnimationFrame(tick);
function setPlay(p){
  if(p&&D&&tCur>=D.duration) tCur=0;
  playing=p; $("playBtn").textContent=p?"⏸":"▶";
}
$("playBtn").addEventListener("click",()=>setPlay(!playing));
$("speedBtn").addEventListener("click",()=>{
  const i=(SPEEDS.indexOf(playSpeed)+1)%SPEEDS.length;
  playSpeed=SPEEDS[i]; $("speedBtn").textContent=playSpeed+"×";
});
window.addEventListener("keydown",e=>{
  if(!D||e.target.tagName==="INPUT") return;
  if(e.code==="Space"){ e.preventDefault(); setPlay(!playing); }
  else if(e.code==="ArrowRight") seek(tCur+(e.shiftKey?5:1));
  else if(e.code==="ArrowLeft") seek(tCur-(e.shiftKey?5:1));
  else if(e.code==="KeyF"){ follow=!follow; updateFollowBtn(); drawTrack(); }
});
</script>
</body></html>
"""


# ----------------------------------------------------------------------- CLI

def main():
    ap = argparse.ArgumentParser(description="F1TENTH rosbag2 시각화 웹앱")
    ap.add_argument("--port", type=int, default=8998)
    ap.add_argument("--host", default="127.0.0.1",
                    help="다른 PC 브라우저에서 볼 때는 0.0.0.0 (기본: localhost 전용)")
    ap.add_argument("--no-browser", action="store_true")
    ap.add_argument("--inspect", metavar="BAG", help="파싱 테스트 후 요약 출력")
    args = ap.parse_args()

    if args.inspect:
        path = resolve_bag_path(args.inspect)
        d = extract(path)
        blob = json.dumps(d, separators=(",", ":")).encode()
        print("bag       :", d["name"])
        print("duration  :", d["duration"], "s   parse:", d["parseSec"], "s")
        print("pose      :", d["poseTopic"], len(d["pose"]["t"]), "pts")
        print("cmd       :", d["cmdTopic"], len(d["cmd"]["t"]), "pts")
        print("act       :", d.get("actTopic"), len(d["act"]["t"]), "pts")
        print("lat_err   :", len(d["lat"]["t"]), "pts")
        print("scans     :", len(d["scans"]), "/", d["scanTotal"])
        print("map       :", "%dx%d res=%s" % (d["map"]["w"], d["map"]["h"], d["map"]["res"]) if d["map"] else None)
        print("glb wpnts :", len(d["glb"]["x"]) if d["glb"] else None)
        print("loc msgs  :", len(d["loc"]), " ot msgs:", len(d["ot"]))
        print("look      :", len(d["look"]["t"]), "pts")
        print("obstacles :", len(d["obs"]), "msgs")
        print("events    :", len(d["events"]))
        print("estop     :", d["estop"][:5], "..." if len(d["estop"]) > 5 else "")
        print("laser off :", d["laserOff"])
        print("json size : %.1f MB (gzip %.1f MB)" % (len(blob) / 1e6, len(gzip.compress(blob, 5)) / 1e6))
        return

    srv = ThreadingHTTPServer((args.host, args.port), Handler)
    url = "http://127.0.0.1:%d" % args.port
    print("BagViz 실행 중: %s  (Ctrl+C 종료)" % url)
    if not args.no_browser:
        webbrowser.open(url)
    try:
        srv.serve_forever()
    except KeyboardInterrupt:
        print("\n종료")


if __name__ == "__main__":
    main()
