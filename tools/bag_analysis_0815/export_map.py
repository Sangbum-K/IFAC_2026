#!/usr/bin/env python3
"""백의 /map (OccupancyGrid) 을 map_server 가 읽는 pgm + yaml 로 역추출.

run_0815_191056 이 쓴 맵이 저장소에 없다 (388x317, origin(-14.358,-6.962)).
my_track0814 로는 대체 불가 — 스캔-맵 정합 잔차가 0.073 -> 0.490 m 로 6.7배 나빠진다.
맵을 다시 뜰 이유가 없다: 그날 맵이 이미 7cm 로 맞고, 백 안에 원본이 그대로 있다.

출력: maps/my_track0815.pgm + my_track0815.yaml
"""
import sqlite3
import sys
from pathlib import Path

sys.path.insert(0, "/home/sangbum/IFAC_ws/tools")
import numpy as np
from bagviz import CDR

BAG = str(Path(__file__).parent / "bag.db3")
OUT = Path("/home/sangbum/IFAC_ws/maps")
NAME = "my_track0815"


def dec_occupancy_grid(b):
    """nav_msgs/OccupancyGrid — origin 의 쿼터니언까지 전부 뽑는다."""
    c = CDR(b)
    c.header()                                   # stamp + frame_id
    c.time()                                     # map_load_time
    res = c.f32()
    w = c.u32()
    h = c.u32()
    ox, oy, oz = c.f64n(3)
    qx, qy, qz, qw = c.f64n(4)                   # dec_grid 는 여기를 건너뛴다
    n = c.u32()
    data = np.frombuffer(c.b, dtype=np.int8, count=n, offset=c.o).astype(np.int16)
    return dict(res=res, w=w, h=h, ox=ox, oy=oy, oz=oz,
                q=(qx, qy, qz, qw), data=data)


db = sqlite3.connect(BAG)
tid = db.execute("SELECT id FROM topics WHERE name='/map'").fetchone()[0]
raw = db.execute(
    "SELECT data FROM messages WHERE topic_id=? ORDER BY timestamp LIMIT 1",
    (tid,)).fetchone()[0]
g = dec_occupancy_grid(raw)

yaw = np.arctan2(2.0 * (g["q"][3] * g["q"][2] + g["q"][0] * g["q"][1]),
                 1.0 - 2.0 * (g["q"][1] ** 2 + g["q"][2] ** 2))
print(f"/map  {g['w']}x{g['h']}  res={g['res']:.9f}")
print(f"      origin=({g['ox']:.9f}, {g['oy']:.9f}, {g['oz']:.6f})  yaw={np.degrees(yaw):.6f} deg")
vals, cnt = np.unique(g["data"], return_counts=True)
print(f"      셀 값 분포: {list(zip(vals.tolist(), cnt.tolist()))}")
assert len(g["data"]) == g["w"] * g["h"], "data 길이 불일치"
if abs(yaw) > 1e-9:
    print("!! origin yaw 가 0 이 아니다 — pgm 은 yaw 를 표현 못 하므로 yaml 에만 남는다")

# ---- OccupancyGrid -> trinary pgm ----
# map_server 규약(negate=0, occupied_thresh 0.65, free_thresh 0.25):
#   점유 확률 p = (255 - pixel)/255
#   p > 0.65 -> 점유,  p < 0.25 -> 자유,  그 사이 -> 미지
# my_track0814.pgm 의 실제 픽셀값(0 / 205 / 254)과 같은 관례를 따른다.
occ = g["data"].reshape(g["h"], g["w"])
img = np.full((g["h"], g["w"]), 205, dtype=np.uint8)      # 미지(-1)
img[occ == 0] = 254                                        # 자유
img[(occ >= 0) & (occ <= 25)] = 254
img[occ >= 65] = 0                                         # 점유
# ROS 격자는 row 0 이 최소 y (아래), PGM 은 위에서 아래로 쓴다 -> 상하 반전
img = np.flipud(img)

OUT.mkdir(exist_ok=True)
pgm = OUT / f"{NAME}.pgm"
with open(pgm, "wb") as f:
    f.write(b"P5\n")
    f.write(f"# extracted from run_0815_191056_0.db3 /map "
            f"(tools/bag_analysis_0815/export_map.py)\n".encode())
    f.write(f"{g['w']} {g['h']}\n255\n".encode())
    f.write(img.tobytes())

yml = OUT / f"{NAME}.yaml"
sx = g["w"] * g["res"]
sy = g["h"] * g["res"]
yml.write_text(
    f"image: {NAME}.pgm\n"
    f"mode: trinary\n"
    f"resolution: {g['res']:.6f}\n"
    f"origin: [{g['ox']:.6f}, {g['oy']:.6f}, {yaw:.6f}]\n"
    f"negate: 0\n"
    f"occupied_thresh: 0.65\n"
    f"free_thresh: 0.25\n"
    f"\n"
    f"# run_0815_191056 백의 /map 을 그대로 역추출한 것 (재매핑 아님).\n"
    f"# 그날 실린 맵이 저장소에 없었고, my_track0814 로는 대체가 안 된다 —\n"
    f"# 스캔-맵 레이캐스트 |잔차| 중앙값이 0.073 -> 0.490 m 로 6.7배 나빠진다.\n"
    f"# 이 맵은 같은 백의 스캔에 중앙 0.073 m / p90 0.280 m 로 맞는다.\n"
    f"# 이하 참고용 (map_server 는 읽지 않음)\n"
    f"size_px: [{g['w']}, {g['h']}]\n"
    f"size_m: [{sx:.2f}, {sy:.2f}]\n"
    f"bounds_x_m: [{g['ox']:.2f}, {g['ox'] + sx:.2f}]\n"
    f"bounds_y_m: [{g['oy']:.2f}, {g['oy'] + sy:.2f}]\n")

print(f"\n저장: {pgm}  ({pgm.stat().st_size} B)")
print(f"저장: {yml}")

# ---- 왕복 검증: 다시 읽어서 원본 점유와 일치하는지 ----
data = pgm.read_bytes()
tok, i = [], 0
while len(tok) < 4:
    while i < len(data) and data[i:i + 1].isspace():
        i += 1
    if data[i:i + 1] == b"#":
        while i < len(data) and data[i:i + 1] != b"\n":
            i += 1
        continue
    j = i
    while j < len(data) and not data[j:j + 1].isspace():
        j += 1
    tok.append(data[i:j])
    i = j
i += 1
w2, h2 = int(tok[1]), int(tok[2])
back = np.frombuffer(data[i:i + w2 * h2], dtype=np.uint8).reshape(h2, w2)
occ_back = np.flipud(back) < 89.25          # trinary 점유 판정
occ_orig = occ >= 65
print(f"\n왕복 검증: {w2}x{h2}  점유 원본 {int(occ_orig.sum())} / 복원 {int(occ_back.sum())}  "
      f"불일치 셀 {int((occ_back != occ_orig).sum())}")
assert w2 == g["w"] and h2 == g["h"], "크기 불일치"
assert (occ_back == occ_orig).all(), "점유 격자 불일치 — 상하반전/임계값 확인"
print("OK — 백의 /map 과 비트 단위로 같은 점유 격자다")
