#!/usr/bin/env python3
"""S1 판정 — 상대 정지 구간의 **모든** 접근을 본다.

첫 접근은 콘 배치 전이라 봉쇄가 성립하지 않는다(실측: 자차가 콘보다 4초 먼저
도착했다). 상대는 hold 동안 계속 서 있으므로 이후 랩마다 접근이 반복된다.
"""
import sqlite3, glob, sys
import numpy as np
sys.path.insert(0, "/home/sangbum/IFAC_ws/tools")
from bagviz import CDR, dec_odom, dec_ackermann

L = 36.84
db = sqlite3.connect(glob.glob(sys.argv[1] + "/*.db3")[0])
top = {n: i for i, n, _ in db.execute("SELECT id,name,type FROM topics")}
t0 = db.execute("SELECT MIN(timestamp) FROM messages").fetchone()[0] / 1e9
def rows(n):
    return [] if n not in top else [(ts/1e9-t0, d) for ts, d in db.execute(
        "SELECT timestamp,data FROM messages WHERE topic_id=? ORDER BY timestamp", (top[n],))]

ego = np.array([(t,) + dec_odom(d)[1:] for t, d in rows("/car_state/frenet/odom")])  # t,s,d,yaw,v
drv = np.array([(t, dec_ackermann(d)[1]) for t, d in rows("/drive")])

def dec_obs(b):
    c = CDR(b); c.header(); n = c.u32(); out = []
    for _ in range(n):
        oid = c.i32(); c.f64n(3); f = c.f64n(14)
        st = c.u8() != 0; c.u8(); c.i32()
        out.append((oid, f[0], f[1], f[5], f[6], st))   # id,s_c,d_c,d_r,d_l,static
    return out
frn = [(t, dec_obs(d)) for t, d in rows("/perception/obstacles_frenet")]
opp = [(t, dec_obs(d)) for t, d in rows("/perception/opponent")]

ot = np.array([(t, o[0][1], o[0][2]) for t, o in opp if o])
still = np.abs(np.diff(ot[:, 1])) < 0.002
runs, cur = [], []
for i, m in enumerate(still):
    if m: cur.append(i)
    elif cur: runs.append(cur); cur = []
if cur: runs.append(cur)
r = max(runs, key=len)
ta, tb = ot[r[0], 0], ot[r[-1]+1, 0]
s_opp, d_opp = ot[r[0], 1], ot[r[0], 2]
print(f"상대 정지: t={ta:.1f}~{tb:.1f} ({tb-ta:.1f}s) s={s_opp:.2f} d={d_opp:+.2f}")

# 콘이 인지에 잡히기 시작한 시각 = 정지지점 ±2 m 안에 정적 트랙이 처음 뜬 때
t_cone = None
for t, obs in frn:
    if t < ta: continue
    for oid, sc, dc, dr, dl, st in obs:
        if st and abs(((sc - s_opp + L/2) % L) - L/2) < 2.0:
            t_cone = t; break
    if t_cone: break
print(f"콘이 인지에 처음 잡힌 시각: {t_cone if t_cone else '없음'}")

# 접근 = 자차 gap 이 +3 -> -1 로 교차하는 사건
gap = (s_opp - ego[:, 1]) % L
gap = np.where(gap > L/2, gap - L, gap)
cross = [i for i in range(1, len(gap)) if gap[i-1] > 0 >= gap[i] and ta <= ego[i, 0] <= tb]
print(f"\n정지 구간 중 자차 통과 {len(cross)}회")
print(f"{'#':>2} {'t':>7} {'콘있음':>7} {'최소간격':>8} {'그때 자차d':>10} {'횡거리':>7} {'차체여유':>9} {'접근속도p90':>11} {'최저명령':>8}")
for n, i in enumerate(cross, 1):
    w = (ego[:, 0] > ego[i, 0] - 3.0) & (ego[:, 0] < ego[i, 0] + 0.5)
    g = (s_opp - ego[w, 1]) % L; g = np.where(g > L/2, g - L, g)
    k = np.argmin(np.abs(g))
    lat = abs(d_opp - ego[w][k, 2])
    md = (drv[:, 0] > ego[i, 0] - 3.0) & (drv[:, 0] < ego[i, 0] + 0.5)
    has = "O" if (t_cone is not None and ego[i, 0] > t_cone) else "X"
    print(f"{n:2d} {ego[i,0]:7.2f} {has:>7} {np.abs(g).min():8.2f} {ego[w][k,2]:+10.2f} "
          f"{lat:7.2f} {lat-0.30:+9.2f} {np.percentile(ego[w,4],90):11.2f} "
          f"{drv[md,1].min() if md.sum() else float('nan'):8.2f}")
