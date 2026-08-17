#!/usr/bin/env python3
"""carve_replay 용 최소 추출: /scan 풀해상도 + /pf/pose/odom + 기억 주입 스냅샷."""
import pickle
import sqlite3
import sys
from pathlib import Path

sys.path.insert(0, "/home/miru/IFAC_ws/tools")
import numpy as np
from bagviz import dec_odom, dec_scan
from extract import dec_frenet_obs  # noqa: E402  (같은 디렉터리)

BAG = sys.argv[1]
OUT = Path(__file__).parent / sys.argv[2]

db = sqlite3.connect(BAG)
topics = {n: tid for tid, n, _ in db.execute("SELECT id,name,type FROM topics")}
t0 = db.execute("SELECT MIN(timestamp) FROM messages").fetchone()[0]


def rows(name):
    return db.execute(
        "SELECT timestamp,data FROM messages WHERE topic_id=? ORDER BY timestamp",
        (topics[name],)).fetchall()


D = {}
out = []
for ts, d in rows("/scan"):
    ht, amin, ainc, tinc, rr = dec_scan(d, 1)          # stride 1 = 풀해상도
    out.append(((ts - t0) / 1e9, amin, ainc, rr.astype(np.float32)))
D["scan"] = out
out = []
for ts, d in rows("/pf/pose/odom"):
    ht, x, y, yaw, v = dec_odom(d)
    out.append(((ts - t0) / 1e9, x, y, yaw, v))
D["pf"] = np.array(out)
D["obs_frenet"] = [((ts - t0) / 1e9, dec_frenet_obs(d)[2])
                   for ts, d in rows("/perception/obstacles_frenet")]
with open(OUT, "wb") as f:
    pickle.dump(D, f)
print(f"saved {OUT}: scan={len(D['scan'])} pf={len(D['pf'])} "
      f"obs={len(D['obs_frenet'])}")
