#!/usr/bin/env python3
"""P0(계측만) vs P1(계측+ever_moved 정지권한) 재생 A/B.

핵심 질문: '잠깐 느려진 상대'를 정지로 오인해 급감속하는 사이클이 생겼는가.

사이클 정렬 키는 (장애물 id, nobs) 다. nobs 는 그 트랙의 관측 수라 트랙 안에서
단조 증가하므로, 두 재생이 같은 프레임을 봤다는 뜻이 된다. s 반올림으로 맞추면
랩마다 같은 s 가 나와 충돌한다(9417 -> 2539).
"""
import re, sys, sqlite3, glob, collections
import numpy as np
sys.path.insert(0, "/home/sangbum/IFAC_ws/tools")
from bagviz import CDR, dec_float32

PAT = re.compile(
    r"\[trace\] s=([-\d.]+) v=([-\d.]+) d=([-\d.]+) \| obs id=(-?\d+) "
    r"dist=([-\d.]+) d\[([-\d.]+),([-\d.]+)\] static=(\d) nobs=(-?\d+)"
    r".*\| (\w+) \| (.*)$")

def load(prefix):
    db = sqlite3.connect(glob.glob(prefix + "/*.db3")[0])
    top = {n: (i, ty) for i, n, ty in db.execute("SELECT id,name,type FROM topics")}
    cnt = {}
    for n, (tid, _ty) in top.items():
        if not n.startswith("/pitwall/"): continue
        r = db.execute("SELECT data FROM messages WHERE topic_id=? "
                       "ORDER BY timestamp DESC LIMIT 1", (tid,)).fetchone()
        if r:
            try: cnt[n] = dec_float32(r[0])
            except Exception: pass
    def dec(b):
        c = CDR(b); t = c.time(); c.u8(); c.string(); return t, c.string()
    t0 = db.execute("SELECT MIN(timestamp) FROM messages").fetchone()[0] / 1e9
    cy, logs = {}, []
    for _, d in db.execute("SELECT timestamp,data FROM messages WHERE topic_id=? "
                           "ORDER BY timestamp", (top["/rosout"][0],)):
        try: t, m = dec(d)
        except Exception: continue
        logs.append(m)
        g = PAT.search(m)
        if g:
            key = (int(g.group(4)), int(g.group(9)))     # (obs id, nobs)
            cy[key] = dict(t=t - t0, s=float(g.group(1)), v=float(g.group(2)),
                           dist=float(g.group(5)), static=int(g.group(8)),
                           mode=g.group(10), tags=g.group(11))
    return cnt, cy, logs

def main():
    A_CNT, A, A_LOG = load(sys.argv[1])
    B_CNT, B, B_LOG = load(sys.argv[2])
    com = sorted(set(A) & set(B))
    print(f"P0 사이클 {len(A)}  P1 {len(B)}  공통 키 {len(com)} "
          f"({len(com)/max(len(A),1)*100:.0f}%)")

    print("\n=== 카운터 변화 (|Δ|>3%, 30 이상) ===")
    for k in sorted(set(A_CNT) | set(B_CNT)):
        a, b = A_CNT.get(k, 0), B_CNT.get(k, 0)
        if max(a, b) < 30: continue
        d = (b - a) / max(a, 1) * 100
        if abs(d) > 3:
            print(f"  {k.split('/')[-1]:44s} {a:7.0f} -> {b:7.0f} ({d:+5.0f}%)")

    ch = [k for k in com if A[k]["mode"] != B[k]["mode"]]
    print(f"\n=== 같은 프레임에서 모드가 바뀐 사이클: {len(ch)} / {len(com)} "
          f"({len(ch)/max(len(com),1)*100:.1f}%) ===")
    if ch:
        c = collections.Counter((A[k]["mode"], B[k]["mode"]) for k in ch)
        for (a, b), n in c.most_common(8):
            print(f"    {a:14s} -> {b:14s} {n}")

    # 정적 회귀: 콘(ever_moved 아님) 프레임은 비트 단위로 같아야 한다
    print("\n=== 정적 프레임(=콘) 불변 확인 ===")
    st = [k for k in com if A[k]["static"] == 1]
    diff = [k for k in st if A[k]["mode"] != B[k]["mode"] or A[k]["tags"] != B[k]["tags"]]
    print(f"  static=1 공통 프레임 {len(st)}개 중 모드/사유가 다른 것 {len(diff)}개")
    for k in diff[:6]:
        print(f"    id={k[0]} nobs={k[1]}  {A[k]['mode']} -> {B[k]['mode']}")

    print("\n=== 정지 권한이 열린 프레임 (P0 에서 stop_denied_ever_dynamic) ===")
    sel = [k for k in com if "trailing_stop_denied_ever_dynamic" in A[k]["tags"]]
    print(f"  {len(sel)}개 프레임")
    if sel:
        print(f"  그때 상대까지 거리 {min(A[k]['dist'] for k in sel):.2f}~{max(A[k]['dist'] for k in sel):.2f} m,"
              f" 자차 속도 {min(A[k]['v'] for k in sel):.2f}~{max(A[k]['v'] for k in sel):.2f} m/s")
        print(f"  P1 에서 같은 프레임의 모드: "
              f"{dict(collections.Counter(B[k]['mode'] for k in sel))}")

    for nm, L in (("P0", A_LOG), ("P1", B_LOG)):
        print(f"  {nm}: 정지 워치독 {sum(1 for m in L if '정지 워치독' in m)}")


if __name__ == "__main__":
    main()
