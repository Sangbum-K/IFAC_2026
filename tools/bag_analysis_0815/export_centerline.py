#!/usr/bin/env python3
"""my_track0815 센터라인만 생성 (레이스라인은 건드리지 않는다).

build_map.py 를 통째로 돌리면 **그날 실제로 돈 것과 다른 레이스라인**이 하나 더
생긴다 — 맵/레이스라인 짝이 어긋나 이번 조사를 통째로 꼬았던 바로 그 상황이다.
그래서 build_map.py 의 load_track -> Grid -> extract_centerline -> roll_to_start
경로만 그대로 재사용하고 CSV 는 센터라인 하나만 쓴다 (main 의 [1/4] 단계와 동일).
"""
import sys
from pathlib import Path

GTS = Path("/home/sangbum/IFAC_ws/planning_ws/src/planning_pkg/scripts/global_traj_sim")
sys.path.insert(0, str(GTS))

import numpy as np  # noqa: E402
import build_map as bm  # noqa: E402

MAP = "my_track0815"

outer, inner, resolution, origin, raw_ring, skeleton_ring, H = bm.load_track(MAP)
grid = bm.Grid(raw_ring, resolution, origin, H, skeleton_ring=skeleton_ring)
center = bm.extract_centerline(grid)
orientation = bm.resolve_orientation(center, want="ccw")
center = bm.roll_to_start(center, orientation=orientation)

xy = center["xy"]
s = center["s"]
print(f"centerline: {len(s)} pts, 길이 {s[-1]:.3f} m, 방향 {orientation}")
print(f"  x [{xy[:,0].min():.2f}, {xy[:,0].max():.2f}]  y [{xy[:,1].min():.2f}, {xy[:,1].max():.2f}]")

# build_map.py 와 **같은 직렬화**(_savetxt_bytes, '; ', %.7f, header x_m; y_m)
center_bytes = bm._savetxt_bytes(xy, delimiter="; ", fmt="%.7f", header="x_m; y_m")
targets = [
    bm.CENTERLINE_DIR / f"{MAP}_centerline.csv",     # 런타임(config/centerline)
    bm.OUT_DIR / f"{MAP}_centerline.csv",            # 산출물 사본
]
for p in targets:
    p.parent.mkdir(parents=True, exist_ok=True)
    p.write_bytes(center_bytes)
    print(f"저장: {p}  ({len(center_bytes)} B)")

# ---------------- 검증 ----------------
print("\n=== 검증 ===")
fail = []

# (1) 닫힌 고리인가
gap = float(np.hypot(*(xy[0] - xy[-1])))
step = float(np.median(np.hypot(*np.diff(xy, axis=0).T)))
print(f"(1) 닫힘: 첫-끝 간격 {gap:.4f} m (점 간격 중앙 {step:.4f} m)")
if gap > 3.0 * step:
    fail.append("고리가 안 닫혔다")

# (2) 모든 점이 트랙 안에 있는가 (맵 벽에서 떨어져 있는가)
from scipy.ndimage import distance_transform_edt  # noqa: E402

data = (bm.MAPS_DIR / f"{MAP}.pgm").read_bytes() if hasattr(bm, "MAPS_DIR") else None
if data is None:
    data = Path(f"/home/sangbum/IFAC_ws/maps/{MAP}.pgm").read_bytes()
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
w, h = int(tok[1]), int(tok[2])
img = np.flipud(np.frombuffer(data[i:i + w * h], dtype=np.uint8).reshape(h, w))
occ = img < 89.25
edt = distance_transform_edt(~occ) * resolution
col = np.floor((xy[:, 0] - origin[0]) / resolution).astype(int)
row = np.floor((xy[:, 1] - origin[1]) / resolution).astype(int)
inb = (col >= 0) & (col < w) & (row >= 0) & (row < h)
d = np.full(len(xy), -1.0)
d[inb] = edt[row[inb], col[inb]]
print(f"(2) 벽 이격: min {d.min():.3f} m  med {np.median(d):.3f}  "
      f"맵 밖 점 {int((~inb).sum())}개")
if (~inb).any() or d.min() <= resolution:
    fail.append("센터라인이 트랙 밖이거나 벽에 닿는다")

# (3) 레이스라인과 길이가 맞는가 (같은 트랙이므로 ±10% 안)
import pickle  # noqa: E402

Dp = Path(__file__).parent / "data.pkl"
if Dp.exists():
    gw = pickle.load(open(Dp, "rb"))["global_wpnts"][0][1]
    rl = gw[-1, 0]
    print(f"(3) 길이: 센터라인 {s[-1]:.2f} m vs 백 레이스라인 {rl:.2f} m "
          f"(차이 {100*abs(s[-1]-rl)/rl:.1f}%)")
    if abs(s[-1] - rl) / rl > 0.10:
        fail.append("센터라인 길이가 레이스라인과 10% 넘게 다르다")

# (4) 왕복 — 쓴 파일을 다시 읽어 일치
back = np.loadtxt(targets[0], delimiter=";", comments="#")
print(f"(4) 왕복: {len(back)}점, 최대오차 {np.abs(back - xy).max():.2e} m")
if len(back) != len(xy) or np.abs(back - xy).max() > 1e-6:
    fail.append("파일 왕복 불일치")

# (5) 곡률 스파이크 (medial-axis 도입 근거였던 게이트 붕괴 회귀 검사)
dxy = np.gradient(xy, axis=0)
ddxy = np.gradient(dxy, axis=0)
kappa = np.abs(dxy[:, 0] * ddxy[:, 1] - dxy[:, 1] * ddxy[:, 0]) / \
    np.maximum(np.hypot(*dxy.T) ** 3, 1e-9)
print(f"(5) 곡률: med {np.median(kappa):.3f}  p99 {np.percentile(kappa,99):.3f}  "
      f"max {kappa.max():.3f} 1/m  (R_min {1/max(kappa.max(),1e-9):.2f} m)")
if kappa.max() > 5.0:
    fail.append(f"곡률 스파이크 {kappa.max():.2f} (R={1/kappa.max():.2f}m) — 게이트 붕괴 의심")

print()
if fail:
    print("FAIL:")
    for f in fail:
        print("  -", f)
    sys.exit(1)
print("PASS — 5개 검사 전부 통과")
