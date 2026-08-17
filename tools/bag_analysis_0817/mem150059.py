#!/usr/bin/env python3
"""기억(주입) 장애물 및 이벤트 타임라인."""
import pickle
import sys
import numpy as np

D = pickle.load(open("d150059.pkl", "rb"))
MEM_BASE = 900000

print("=== events ===")
for t, s in D["events"]:
    print(f"{t:7.2f}  {s}")

print("\n=== integration status ===")
for t, s in D["integration"][:6]:
    print(f"{t:7.2f}  {s[:300]}")

print("\n=== rosout: bridge / memory 관련 ===")
for t, lvl, name, msg in D["rosout"]:
    lo = msg.lower()
    if "bridge" in name or "memory" in lo or "기억" in msg or "anchor" in lo:
        print(f"{t:7.2f} [{lvl}] {name}: {msg[:220]}")

print("\n=== obs_frenet: 기억 id(>=900000) 등장 ===")
mem_frames = 0
first = None
ids_seen = {}
for t, obs in D["obs_frenet"]:
    mem = [o for o in obs if o["id"] >= MEM_BASE]
    if mem:
        mem_frames += 1
        if first is None:
            first = t
        for o in mem:
            ids_seen.setdefault(o["id"], []).append((t, o["s_center"], o["d_center"], o["is_visible"], o["lap"]))
print(f"frames with memory obs: {mem_frames}/{len(D['obs_frenet'])}, first t={first}")
for oid, rec in sorted(ids_seen.items()):
    ts = [r[0] for r in rec]
    print(f"  id={oid} n={len(rec)} t={ts[0]:.2f}~{ts[-1]:.2f} "
          f"s={np.mean([r[1] for r in rec]):.2f} d={np.mean([r[2] for r in rec]):.2f}")

print("\n=== obs_frenet: 전체 id 요약 ===")
allids = {}
for t, obs in D["obs_frenet"]:
    for o in obs:
        allids.setdefault(o["id"], []).append((t, o["s_center"], o["d_center"], o["is_static"], o["is_visible"], o["lap"]))
for oid, rec in sorted(allids.items()):
    ts = [r[0] for r in rec]
    ss = [r[1] for r in rec]
    dd = [r[2] for r in rec]
    stat = np.mean([r[3] for r in rec])
    vis = np.mean([r[4] for r in rec])
    laps = sorted(set(r[5] for r in rec))
    print(f"  id={oid:7d} n={len(rec):4d} t={ts[0]:6.2f}~{ts[-1]:6.2f} "
          f"s={np.mean(ss):6.2f}±{np.std(ss):4.2f} d={np.mean(dd):6.2f}±{np.std(dd):4.2f} "
          f"static={stat:.2f} vis={vis:.2f} laps={laps}")
