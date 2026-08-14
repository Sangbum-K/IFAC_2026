#!/usr/bin/env python3
"""물리 장애물: 근거리 관측만으로 xy 클러스터링."""
import pickle
from pathlib import Path
import numpy as np

D = pickle.load(open(Path(__file__).parent / "data.pkl", "rb"))
_, gb = D["global_wpnts"][0]
gs, gdr, gdl, gkap, gvx = gb[:,0], gb[:,4], gb[:,5], gb[:,7], gb[:,8]
TL = 32.07
fre = D["/car_state/frenet/odom"]      # t, s, d, ?, v

def ego_s(t):
    return np.interp(t, fre[:,0], fre[:,1])

rows = []
for t, obs in D["obs_frenet"]:
    es = ego_s(t)
    for o in obs:
        gap = (o["s_center"] - es) % TL
        if gap > TL/2: gap -= TL
        rows.append((t, o["x"], o["y"], o["s_center"], o["d_center"], gap,
                     o["size"], 1 if o["is_static"] else 0, o["id"]))
R = np.array(rows)
near = R[(R[:,5] > -0.5) & (R[:,5] < 3.0)]     # 3m 이내 (정밀)
print(f"전체 {len(R)}  근거리(<3m) {len(near)}")

# xy 그리드 클러스터 (0.45m)
cells = {}
for r in near:
    key = (round(r[1]/0.45), round(r[2]/0.45))
    cells.setdefault(key, []).append(r)
# 인접 셀 병합
keys = sorted(cells, key=lambda k: -len(cells[k]))
used, groups = set(), []
for k in keys:
    if k in used: continue
    g, stack = [], [k]
    while stack:
        c = stack.pop()
        if c in used or c not in cells: continue
        used.add(c); g.extend(cells[c])
        for dx in (-1,0,1):
            for dy in (-1,0,1):
                n=(c[0]+dx,c[1]+dy)
                if n in cells and n not in used: stack.append(n)
    groups.append(np.array(g))
groups = [g for g in groups if len(g) >= 25]
groups.sort(key=lambda g: np.median(g[:,3]))
print(f"\n=== 물리 장애물 {len(groups)}개 (근거리 관측 기준) ===")
OB=[]
for i,g in enumerate(groups,1):
    s_,d_ = np.median(g[:,3]), np.median(g[:,4])
    x_,y_ = np.median(g[:,1]), np.median(g[:,2])
    k = np.interp(s_, gs, gkap)
    dr_, dl_ = np.interp(s_,gs,gdr), np.interp(s_,gs,gdl)
    lap_seen = len(np.unique(np.round(g[:,0]/1.0)))
    OB.append(dict(name=f"O{i}", s=s_, d=d_, x=x_, y=y_, kap=k,
                   dr=dr_, dl=dl_, n=len(g), size=np.median(g[:,6]),
                   stat=g[:,7].mean(), t0=g[:,0].min(), t1=g[:,0].max()))
    # 좌/우 잔여 폭 (차폭 0.30 반경 가정 -> 장애물 반경 size/2)
    half = np.median(g[:,6])/2
    room_L = dl_ - (d_ + half)
    room_R = (d_ - half) - (-dr_)
    print(f" O{i}: s={s_:6.2f} d={d_:+5.2f} xy=({x_:+5.2f},{y_:+5.2f}) n={len(g):4d} "
          f"size={np.median(g[:,6]):.2f} static={g[:,7].mean():.2f} kappa={k:+.3f} "
          f"트랙d[{-dr_:+.2f},{dl_:+.2f}] 좌여유={room_L:+.2f} 우여유={room_R:+.2f} "
          f"v_plan={np.interp(s_,gs,gvx):.2f}")
pickle.dump(OB, open("obst.pkl","wb"))
