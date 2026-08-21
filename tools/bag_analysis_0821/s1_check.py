#!/usr/bin/env python3
"""S1(봉쇄 정지) 결과 판정 — 백에서 직접 읽는다 (시나리오 자기보고 불신)."""
import sqlite3, glob, sys
import numpy as np
sys.path.insert(0, "/home/sangbum/IFAC_ws/tools")
from bagviz import CDR, dec_odom, dec_ackermann

BAG = sys.argv[1]
db = sqlite3.connect(glob.glob(BAG + "/*.db3")[0])
top = {n: i for i, n, _ in db.execute("SELECT id,name,type FROM topics")}
t0 = db.execute("SELECT MIN(timestamp) FROM messages").fetchone()[0] / 1e9

def rows(name):
    if name not in top: return []
    return [(ts / 1e9 - t0, d) for ts, d in db.execute(
        "SELECT timestamp,data FROM messages WHERE topic_id=? ORDER BY timestamp", (top[name],))]

def dec_drive(b):
    # AckermannDriveStamped 는 float32 다 (bagviz.dec_ackermann).
    steer, speed = dec_ackermann(b)
    return speed, steer

ego = np.array([(t,) + dec_odom(d)[1:] for t, d in rows("/car_state/frenet/odom")])
drv = np.array([(t,) + dec_drive(d) for t, d in rows("/drive")]) if "/drive" in top else np.zeros((0, 3))

def dec_obs(b):
    c = CDR(b); c.header(); n = c.u32(); out = []
    for _ in range(n):
        oid = c.i32(); c.f64n(3)
        f = c.f64n(14)
        st = c.u8() != 0; vis = c.u8() != 0; c.i32()
        out.append((oid, f[0], f[1], f[2], f[3], f[4], f[5], f[6], st))
    return out            # id, s_c, d_c, s_s, s_e, d_r, d_l, vs, is_static
opp = [(t, dec_obs(d)) for t, d in rows("/perception/opponent")]

print(f"기록 {ego[-1,0]:.1f}s, 자차 s {ego[:,1].min():.1f}~{ego[:,1].max():.1f}")
# 상대 궤적
ot = np.array([(t, o[0][1], o[0][2], o[0][7]) for t, o in opp if o])
if len(ot):
    still = np.abs(np.diff(ot[:, 1])) < 0.002
    print(f"상대: s {ot[:,1].min():.2f}~{ot[:,1].max():.2f}")
    # 정지 구간 = s 변화가 거의 없는 최장 구간
    runs, cur = [], []
    for i, m in enumerate(still):
        if m: cur.append(i)
        elif cur: runs.append(cur); cur = []
    if cur: runs.append(cur)
    if runs:
        r = max(runs, key=len)
        ta, tb = ot[r[0], 0], ot[r[-1] + 1, 0]
        print(f"  최장 정지 구간 t={ta:.1f}~{tb:.1f} ({tb-ta:.1f}s) at s={ot[r[0],1]:.2f} d={ot[r[0],2]:+.2f}")
        # 그 동안 자차
        m = (ego[:, 0] >= ta) & (ego[:, 0] <= tb)
        if m.sum():
            gap = (ot[r[0], 1] - ego[m, 1]) % 36.84
            gap = np.where(gap > 18, gap - 36.84, gap)
            k = np.argmin(np.abs(gap))
            print(f"  그동안 자차 s {ego[m,1].min():.2f}~{ego[m,1].max():.2f}, "
                  f"최소 간격 {np.abs(gap).min():.2f} m (그때 자차 v={ego[m][k,4]:.2f})")
            print(f"  자차 속도 그 구간: p50 {np.median(ego[m,4]):.2f} 최저 {ego[m,4].min():.2f} m/s")
            lat = np.abs(ot[r[0], 2] - ego[m][k, 2])
            print(f"  최근접 시 횡거리 {lat:.2f} m -> 차체여유 {lat-0.15-0.15:+.2f} m")
        md = (drv[:, 0] >= ta) & (drv[:, 0] <= tb)
        if md.sum():
            print(f"  /drive 명령속도: p50 {np.median(drv[md,1]):.2f} 최저 {drv[md,1].min():.2f} m/s")

# ── 접근 구간 시계열 ──────────────────────────────────────────────────────
print("\n=== 접근 구간 (최근접 전 6초) ===")
if len(ot):
    r = max(runs, key=len)
    ta, tb = ot[r[0], 0], ot[r[-1] + 1, 0]
    s_opp, d_opp = ot[r[0], 1], ot[r[0], 2]
    m = (ego[:, 0] >= ta) & (ego[:, 0] <= tb)
    E = ego[m]
    gap = (s_opp - E[:, 1]) % 36.84
    gap = np.where(gap > 18, gap - 36.84, gap)
    # 처음으로 gap 이 1.0 아래로 내려간 시점
    close = np.where(np.abs(gap) < 1.0)[0]
    if len(close):
        k0 = close[0]
        t_hit = E[k0, 0]
        sel = (E[:, 0] > t_hit - 6.0) & (E[:, 0] < t_hit + 1.5)
        print(f"{'t':>6} {'자차s':>7} {'간격':>7} {'자차d':>7} {'속도':>6} {'명령':>6}")
        idx = np.where(sel)[0]
        for i in idx[::max(1, len(idx)//18)]:
            j = np.argmin(np.abs(drv[:, 0] - E[i, 0])) if len(drv) else None
            cmd = drv[j, 1] if j is not None else float('nan')
            g2 = (s_opp - E[i, 1]) % 36.84
            if g2 > 18: g2 -= 36.84
            print(f"{E[i,0]:6.2f} {E[i,1]:7.2f} {g2:+7.2f} {E[i,2]:+7.2f} {E[i,4]:6.2f} {cmd:6.2f}")
        print(f"\n상대 정지 위치 s={s_opp:.2f} d={d_opp:+.2f}")
