#!/usr/bin/env python3
"""S1 최종 판정 — 봉쇄를 이루는 **모든** 물체에 대한 차체 여유를 본다.

⚠️ 상대만 재면 안 된다. 봉쇄는 상대 + 콘으로 만들어지므로, 상대를 피해 콘을
   밟고 지나가도 '여유 +0.37' 로 보인다 (실제로 그랬다).
"""
import sqlite3, glob, sys
import numpy as np
sys.path.insert(0, "/home/sangbum/IFAC_ws/tools")
from bagviz import CDR, dec_odom, dec_ackermann

L, HALF = 36.84, 0.15
db = sqlite3.connect(glob.glob(sys.argv[1] + "/*.db3")[0])
top = {n: i for i, n, _ in db.execute("SELECT id,name,type FROM topics")}
t0 = db.execute("SELECT MIN(timestamp) FROM messages").fetchone()[0] / 1e9
def rows(n):
    return [] if n not in top else [(ts/1e9-t0, d) for ts, d in db.execute(
        "SELECT timestamp,data FROM messages WHERE topic_id=? ORDER BY timestamp", (top[n],))]
def dec_obs(b):
    c = CDR(b); c.header(); n = c.u32(); out = []
    for _ in range(n):
        oid = c.i32(); c.f64n(3); f = c.f64n(14)
        st = c.u8() != 0; c.u8(); c.i32()
        out.append(dict(id=oid, s=f[0], d=f[1], ss=f[2], se=f[3],
                        dr=f[4], dl=f[5], vs=f[6], static=st))
    return out

ego = np.array([(t,) + dec_odom(d)[1:] for t, d in rows("/car_state/frenet/odom")])
drv = np.array([(t, dec_ackermann(d)[1]) for t, d in rows("/drive")])
frn = [(t, dec_obs(d)) for t, d in rows("/perception/obstacles_frenet")]
opp = [(t, dec_obs(d)) for t, d in rows("/perception/opponent")]

ot = np.array([(t, o[0]["s"], o[0]["d"]) for t, o in opp if o])
still = np.abs(np.diff(ot[:, 1])) < 0.002
runs, cur = [], []
for i, m in enumerate(still):
    if m: cur.append(i)
    elif cur: runs.append(cur); cur = []
if cur: runs.append(cur)
r = max(runs, key=len); ta, tb = ot[r[0], 0], ot[r[-1]+1, 0]
s_opp = ot[r[0], 1]
print(f"상대 정지 t={ta:.1f}~{tb:.1f} ({tb-ta:.1f}s) s={s_opp:.2f} d={ot[r[0],2]:+.2f}")

# 매 프레임: 봉쇄 구역(정지지점 ±1.5 m) 안의 모든 장애물에 대한 최소 차체여유
res = []
for t, obs in frn:
    if not (ta <= t <= tb): continue
    k = np.argmin(np.abs(ego[:, 0] - t))
    if abs(ego[k, 0] - t) > 0.1: continue
    es, ed, ev = ego[k, 1], ego[k, 2], ego[k, 4]
    best = None
    for o in obs:
        ds = ((o["s"] - es + L/2) % L) - L/2
        if abs(ds) > 0.45: continue                 # 종방향으로 나란할 때만
        slack = abs(o["d"] - ed) - (o["dl"] - o["dr"]) / 2.0 - HALF
        if best is None or slack < best[0]:
            best = (slack, o["id"], o["static"], o["d"])
    if best: res.append((t, es, ed, ev, *best))
if not res:
    print("나란히 선 프레임 없음"); sys.exit()
R = np.array([(x[0], x[3], x[4]) for x in res])     # t, v, slack
print(f"\n나란히 선 프레임 {len(res)}개 (봉쇄 물체 전부 대상)")
print(f"  차체여유 최소 {R[:,2].min():+.3f} m   p10 {np.percentile(R[:,2],10):+.3f}")
n_hit = int((R[:, 2] <= 0).sum())
print(f"  접촉(<=0) 프레임 {n_hit}  -> **{'접촉 발생' if n_hit else '접촉 없음'}**")
if n_hit:
    i = int(np.argmin(R[:, 2]))
    x = res[i]
    print(f"  최악: t={x[0]:.2f} 자차 s={x[1]:.2f} d={x[2]:+.2f} v={x[3]:.2f} "
          f"vs 물체 id={x[5]} d={x[7]:+.2f} static={x[6]}")
md = (drv[:, 0] >= ta) & (drv[:, 0] <= tb)
print(f"  /drive 명령: 최저 {drv[md,1].min():.2f}  p10 {np.percentile(drv[md,1],10):.2f} m/s")
print(f"  자차 속도  : 최저 {ego[(ego[:,0]>=ta)&(ego[:,0]<=tb),4].min():.2f} m/s")
