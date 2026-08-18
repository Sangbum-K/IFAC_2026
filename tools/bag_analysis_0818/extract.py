#!/usr/bin/env python3
"""백(또는 재생 결과 백)에서 /perception/obstacles + /pf/pose/odom 을 뽑는다.

트랙 필드를 **전부** 디코드한다 (bagviz.dec_obstacles 는 id/velocity/is_visible 을
버린다). 시간축은 bag 수신시각이 아니라 **헤더 시각**을 쓴다 — 재생 결과와
원본을 같은 축에서 비교해야 하기 때문이다.
"""
import pickle, sqlite3, sys
from pathlib import Path
sys.path.insert(0, "/home/miru/IFAC_ws/tools")
import numpy as np
from bagviz import CDR, dec_odom


def dec_obs_full(b):
    c = CDR(b); ht, frame = c.header(); n = c.u32(); out = []
    for _ in range(n):
        oid = c.i32()
        px, py, _pz = c.f64n(3)
        vx, vy, _vz = c.f64n(3)
        dyn = c.u8() != 0
        conf = c.f32(); rad = c.f32(); vis = c.u8() != 0
        out.append((oid, px, py, vx, vy, int(dyn), conf, rad, int(vis)))
    return ht, out, len(b) - c.o


def db3(path):
    p = Path(path)
    if p.is_dir():
        cands = sorted(p.glob("*.db3"))
        if not cands:
            raise SystemExit("no .db3 in %s" % p)
        return str(cands[0])
    return str(p)


def load(path, pose_from=None):
    db = sqlite3.connect(db3(path))
    topics = {n: t for t, n, _ in db.execute("SELECT id,name,type FROM topics")}
    def rows(name):
        if name not in topics:
            return []
        return db.execute(
            "SELECT timestamp,data FROM messages WHERE topic_id=? ORDER BY timestamp",
            (topics[name],)).fetchall()
    D = {}
    obs, left = [], set()
    for _ts, d in rows("/perception/obstacles"):
        ht, o, lo = dec_obs_full(d); left.add(lo); obs.append((ht, o))
    D["obs"] = obs
    if left - {0, 1, 2, 3}:
        print("⚠ ObstacleArray 디코더가 %s B 를 남겼습니다" % sorted(left))
    src = pose_from or path
    dbp = sqlite3.connect(db3(src))
    tp = {n: t for t, n, _ in dbp.execute("SELECT id,name,type FROM topics")}
    pf = []
    for _ts, d in dbp.execute(
            "SELECT timestamp,data FROM messages WHERE topic_id=? ORDER BY timestamp",
            (tp["/pf/pose/odom"],)).fetchall():
        ht, x, y, yaw, v = dec_odom(d)
        pf.append((ht, x, y, yaw, v))
    D["pf"] = np.array(pf)
    return D


if __name__ == "__main__":
    out = sys.argv[2]
    D = load(sys.argv[1], sys.argv[3] if len(sys.argv) > 3 else None)
    pickle.dump(D, open(out, "wb"))
    print("%s: obs frames %d, pf %d" % (out, len(D["obs"]), len(D["pf"])))
