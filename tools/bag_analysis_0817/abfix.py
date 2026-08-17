"""BuildTrailing 횡목표 선택을 OLD/NEW 로 오프라인 재현 (run_0817_124904 59.3~59.95s).

재현 범위: d_goal 결정 + ramp_ok 경계 판정만. 속도 프로파일/충돌검사는 무관.
파라미터는 config/params.yaml + environments/real_car.yaml 실값.
"""
import pickle, numpy as np
from pathlib import Path
D = pickle.load(open(Path(__file__).parent / "d124904.pkl", "rb"))
fre = D["/car_state/frenet/odom"]; gw = D["global_wpnts"][0][1]
DR, DL = gw[:, 4], gw[:, 5]
WP = gw[1, 0] - gw[0, 0]
NG = len(gw)
TRACK = gw[-1, 0] - gw[0, 0] + np.hypot(gw[0, 2]-gw[-1, 2], gw[0, 3]-gw[-1, 3])
def MARGIN_AT(v):
    return max(0.20, 0.15 + min(0.10 + 0.08 * max(v - 3.5, 0.0), 0.22))
A_LAT = 8.83           # blend_lat_acc_max
GAP = 1.5              # trailing_gap_m
EXTEND = 5.0
RETRY = 0.577          # 59.22s 자유스플라인 apex (evasion_d_apex 실측)
RETRY_T, RETRY_HOLD = 59.22, 0.7

def gi_of(s): return int((s % TRACK) / WP) % NG
def blend_dist(A, v): 
    A = abs(A)
    return 0.0 if A < 1e-3 else v * np.sqrt(6.0 * A / A_LAT)

def make(cur_s, cur_d, cur_v, obs_s_start, obs_s_end, goal):
    span = (obs_s_start - GAP - cur_s) % TRACK
    if span > 12.0 + 5.0: span = 0.0
    n = max(int(min(span + EXTEND, TRACK - WP) / WP), 2)
    ramp = max(1.0, blend_dist(goal - cur_d, cur_v))
    back = max(1.0, blend_dist(goal, cur_v))
    hold_until = (obs_s_end - cur_s) % TRACK + 0.5
    def d_at(dist):
        if dist <= ramp: return cur_d + (goal - cur_d) * min(dist / max(ramp, 1e-3), 1.0)
        if dist <= hold_until or abs(goal) <= 0.02: return goal
        return goal * (1.0 - min((dist - hold_until) / max(back, 1e-3), 1.0))
    return n, d_at

def ramp_ok(cur_s, cur_d, cur_v, oss, ose, goal, new_rule):
    n, d_at = make(cur_s, cur_d, cur_v, oss, ose, goal)
    for i in range(n):
        dist = i * WP
        gi = gi_of(cur_s + dist)
        d = d_at(dist)
        M = MARGIN_AT(max(cur_v, 1.5))
        dmin, dmax = -abs(DR[gi]) + M, abs(DL[gi]) - M
        if dmin <= d <= dmax: continue
        if not new_rule: return False, (cur_s + dist) % TRACK, d
        if dmin > dmax: return False, (cur_s + dist) % TRACK, d
        if d < dmin and cur_d < dmin and d >= cur_d: continue   # fail-open (같은 쪽, 더 나빠지지 않음)
        if d > dmax and cur_d > dmax and d <= cur_d: continue
        return False, (cur_s + dist) % TRACK, d
    return True, None, None

def solve(cur_s, cur_d, cur_v, oss, ose, new_rule, new_interval):
    full = RETRY
    ok, bs, bd = ramp_ok(cur_s, cur_d, cur_v, oss, ose, full, new_rule)
    if ok: return full, "전량 채택", bs
    lo, hi = 0.0, 1.0
    for _ in range(12):
        mid = 0.5 * (lo + hi)
        cand = (cur_d + (full - cur_d) * mid) if new_interval else (full * mid)
        if ramp_ok(cur_s, cur_d, cur_v, oss, ose, cand, new_rule)[0]: lo = mid
        else: hi = mid
    g = (cur_d + (full - cur_d) * lo) if new_interval else (full * lo)
    if new_interval:
        prog = abs(g - cur_d) > 0.02
        return (g if prog else cur_d), ("축소 채택" if prog else "진행 0 → 현재 유지"), bs
    else:
        if abs(g) <= 0.02: g = 0.0
        return g, ("축소 채택" if abs(g) > 0.02 else "포기 → d 동결"), bs

# 그 구간 장애물 #185 (실측 발행값 사용)
def obs_at(t):
    best = None
    for tt, obs in D["obs_frenet"]:
        if abs(tt - t) > 0.06: continue
        for o in obs:
            if 32.9 <= o["s_center"] <= 34.3: best = (o["s_start"], o["s_end"])
    return best

print("run_0817_124904 — 트레일링 사이클별 횡목표 (회피목표 +0.577, 자차 오른쪽 d≈-0.2x)")
print("  t     자차s   자차d  v    | OLD(현행)                 | NEW(Fix A+B)              | 횡이동 확보")
prev = None
for t, W in D["otwpnts"]:
    if not (59.33 <= t <= 59.95): continue
    if t - (prev or 0) < 0.055: continue
    prev = t
    o = obs_at(t)
    if o is None: continue
    cs = np.interp(t, fre[:,0], fre[:,1]); cd = np.interp(t, fre[:,0], fre[:,2])
    cv = np.interp(t, fre[:,0], fre[:,4])
    if not (0 <= t - RETRY_T <= RETRY_HOLD): continue
    g_old, w_old, _ = solve(cs, cd, cv, o[0], o[1], False, False)
    g_new, w_new, _ = solve(cs, cd, cv, o[0], o[1], True, True)
    print(f"  {t:5.2f} {cs:6.2f} {cd:+5.2f} {cv:4.2f} | d*={g_old:+5.2f} {w_old:<14s} | d*={g_new:+5.2f} {w_new:<14s} | {g_new-g_old:+.2f} m")
