#!/usr/bin/env python3
"""P0 계측 결과 — 재생본(/pitwall/* + /rosout)과 원본 백의 카운터를 대조한다."""
import pickle, re, sqlite3, sys, collections
import numpy as np
sys.path.insert(0, "/home/sangbum/IFAC_ws/tools")
from bagviz import CDR, dec_float32

REPLAY = sys.argv[1] if len(sys.argv) > 1 else "p0"

def dec_log2(b):
    c = CDR(b); t = c.time(); lvl = c.u8(); name = c.string(); msg = c.string()
    return t, lvl, name, msg

import glob, os
db_path = glob.glob(os.path.join(REPLAY, "*.db3"))[0]
db = sqlite3.connect(db_path)
topics = {n: (i, ty) for i, n, ty in db.execute("SELECT id,name,type FROM topics")}
final = {}
for n, (tid, ty) in topics.items():
    if not n.startswith("/pitwall/"):
        continue
    r = db.execute("SELECT data FROM messages WHERE topic_id=? ORDER BY timestamp DESC LIMIT 1",
                   (tid,)).fetchone()
    if r:
        try: final[n] = dec_float32(r[0])
        except Exception: pass

# rosout -> trace (메시지 자체 stamp 로 재타임스탬프)
out = []
if "/rosout" in topics:
    t0 = db.execute("SELECT MIN(timestamp) FROM messages").fetchone()[0] / 1e9
    for ts, d in db.execute("SELECT timestamp,data FROM messages WHERE topic_id=? ORDER BY timestamp",
                            (topics["/rosout"][0],)):
        try:
            t, lvl, name, msg = dec_log2(d)
            out.append((t - t0, msg))
        except Exception: pass
    out.sort()

pat = re.compile(r"\[trace\] .*\| (\w+) \| (.*)$")
cyc = []
for t, msg in out:
    m = pat.search(msg)
    if m:
        cyc.append((t, m.group(1), set(x.lstrip('+') for x in m.group(2).split(',') if x)))
print(f"재생본 trace 사이클 {len(cyc)}개")

def runs(flag):
    """flag 가 연속으로 선 구간 길이(사이클 수) 목록."""
    r, cur = [], 0
    prev_t = None
    for t, mode, tags in cyc:
        hit = flag in tags
        if hit and (prev_t is None or t - prev_t < 0.15):
            cur += 1
        elif hit:
            if cur: r.append(cur)
            cur = 1
        else:
            if cur: r.append(cur)
            cur = 0
        prev_t = t
    if cur: r.append(cur)
    return r

print("\n=== P0-1  blocked_both_sides (MoreSpace 양쪽 통과 불가) ===")
n = sum(1 for _, _, tg in cyc if "blocked_both_sides" in tg)
print(f"  발화 사이클 {n} / {len(cyc)} ({n/max(len(cyc),1)*100:.1f}%)   카운터 최종 {final.get('/pitwall/outcome_blocked_both_sides','-')}")
r = runs("blocked_both_sides")
if r:
    r = np.array(r)
    print(f"  연속 구간 {len(r)}개: 길이(사이클) p50 {np.median(r):.0f} p90 {np.percentile(r,90):.0f} max {r.max()}")
    for N in (2, 3, 5, 8, 12, 20):
        print(f"    지속 게이트 N={N:2d} -> 남는 구간 {int((r>=N).sum()):3d}개 "
              f"(전체의 {(r>=N).sum()/len(r)*100:3.0f}%), 사이클 {int(r[r>=N].sum()):4d}")
    mode = collections.Counter(m for _, m, tg in cyc if "blocked_both_sides" in tg)
    print(f"  그때 모드: {dict(mode)}")

print("\n=== P0-2  escape arc ===")
att = sum(1 for _, _, tg in cyc if "escape_attempted" in tg)
gate = sum(1 for _, _, tg in cyc if "escape_gate_not_geom" in tg)
print(f"  진입(escape_attempted) {att} 사이클   진입게이트가 막음(escape_gate_not_geom) {gate}")
print(f"  카운터: attempted {final.get('/pitwall/outcome_escape_attempted','-')} / "
      f"gate_not_geom {final.get('/pitwall/outcome_escape_gate_not_geom','-')}")
for k in sorted(final):
    if "escape" in k and k != "/pitwall/outcome_escape_attempted":
        print(f"    {k.split('/')[-1]:38s} {final[k]:.0f}")

print("\n=== P0-3  정지 권한 박탈 ===")
for f in ("trailing_retry_clamp_raised", "trailing_stop_denied_ever_dynamic",
          "trailing_no_return_geom_denied_stop", "trailing_no_return_ever_dynamic"):
    n = sum(1 for _, _, tg in cyc if f in tg)
    print(f"  {f:38s} {n:5d} 사이클  (카운터 {final.get('/pitwall/outcome_'+f,'-')})")
both = sum(1 for _, _, tg in cyc
           if "blocked_both_sides" in tg and "trailing_retry_clamp_raised" in tg)
print(f"  ** 양쪽막힘 AND 정지금지 동시: {both} 사이클 ** = P1-1 이 실제로 바꿀 사이클")

print("\n=== 재생 충실도 (원본 백 대비) ===")
D = pickle.load(open("d.pkl", "rb"))
print(f"  {'카운터':46s} {'원본':>8} {'재생':>8}")
for k in ("outcome_trailing", "outcome_ok_lane_switch", "outcome_cluster_merge_blocks_track",
          "reject_lane_wall_no_room", "reject_lane_geom_floor", "reject_trailing_ramp_collision",
          "reject_escape_geom_floor", "reject_escape_not_visible_static"):
    o = D.get("/pitwall/" + k)
    ov = o[-1, 1] if o is not None and len(o) else float('nan')
    rv = final.get("/pitwall/" + k, float('nan'))
    print(f"  {k:46s} {ov:8.0f} {rv:8.0f}")
