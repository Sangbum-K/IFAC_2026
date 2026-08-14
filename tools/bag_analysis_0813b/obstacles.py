#!/usr/bin/env python3
"""물리 장애물 클러스터링 + 랩별 통과 분석."""
import pickle
from collections import defaultdict
from pathlib import Path
import numpy as np

D = pickle.load(open(Path(__file__).parent / "data.pkl", "rb"))
_, gb = D["global_wpnts"][0]
gs, gx, gy, gdr, gdl, gkap, gvx = (gb[:,0], gb[:,2], gb[:,3], gb[:,4], gb[:,5],
                                   gb[:,7], gb[:,8])
TL = 32.07

# ---- 프레네 장애물 전체를 (s_center, d_center) 로 모아 클러스터 ----
pts = []
for t, obs in D["obs_frenet"]:
    for o in obs:
        pts.append((t, o["id"], o["s_center"], o["d_center"], o["x"], o["y"],
                    o["size"], 1 if o["is_static"] else 0, o["s_start"], o["s_end"],
                    o["d_right"], o["d_left"], o["is_visible"]))
P = np.array([(p[0],p[1],p[2],p[3],p[4],p[5],p[6],p[7],p[8],p[9],p[10],p[11]) for p in pts])
print(f"프레네 장애물 관측 {len(P)}개")

# s 축 히스토그램으로 물리 위치 파악
h, edges = np.histogram(P[:,2], bins=int(TL*2), range=(0,TL))
print("\n=== s별 장애물 관측 밀도 (0.5m bin, >30만) ===")
for i,c in enumerate(h):
    if c > 30:
        print(f"  s={edges[i]:5.1f}-{edges[i+1]:5.1f}  {c:5d}  "
              f"d_center p50={np.median(P[(P[:,2]>=edges[i])&(P[:,2]<edges[i+1]),3]):+.2f}")

# 클러스터: s 간격 1.2m 이상이면 다른 장애물
order = np.argsort(P[:,2])
Ps = P[order]
clusters, cur = [], [0]
for i in range(1, len(Ps)):
    if Ps[i,2] - Ps[i-1,2] > 1.2:
        clusters.append(cur); cur = []
    cur.append(i)
clusters.append(cur)
print(f"\n=== 물리 장애물 클러스터 {len(clusters)}개 ===")
OBST = []
for ci, idx in enumerate(clusters):
    if len(idx) < 20: continue
    Q = Ps[idx]
    sc, dc = np.median(Q[:,2]), np.median(Q[:,3])
    k = np.interp(sc, gs, gkap)
    OBST.append(dict(s=sc, d=dc, n=len(idx), kap=k,
                     x=np.median(Q[:,4]), y=np.median(Q[:,5]),
                     size=np.median(Q[:,6]), stat=Q[:,7].mean(),
                     dr=np.median(Q[:,10]), dl=np.median(Q[:,11]),
                     s0=np.median(Q[:,8]), s1=np.median(Q[:,9])))
    o = OBST[-1]
    R = 1/abs(k) if abs(k)>1e-3 else 999
    print(f"  O{len(OBST)}: s={sc:6.2f} d={dc:+5.2f} xy=({o['x']:+5.2f},{o['y']:+5.2f}) "
          f"관측{len(idx):5d} size={o['size']:.2f} static={o['stat']:.2f} "
          f"kappa={k:+.3f}(R={R:4.1f}) obs_d[{o['dr']:+.2f},{o['dl']:+.2f}] "
          f"span s[{o['s0']:.2f},{o['s1']:.2f}] trk_d[{np.interp(sc,gs,-gdr):+.2f},{np.interp(sc,gs,gdl):+.2f}]")
pickle.dump(OBST, open("obst.pkl","wb"))
