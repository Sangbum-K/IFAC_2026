import pickle, re, collections, numpy as np, sys
RX=re.compile(r"\[trace\] s=([-\d.]+) v=([-\d.]+) d=([-\d.]+) \| obs id=(\d+) dist=([-\d.]+) d\[([-\d.]+),([-\d.]+)\] static=(\d) nobs=(\d+) \| evade=(\d) sector=(\d) commit=(\d) \| (\w+) \| (.*)$")
for tag in ('d43','d47'):
    D=pickle.load(open(tag+'.pkl','rb'))
    rows=[]
    for t,lvl,name,msg in D['rosout']:
        m=RX.search(msg)
        if m: rows.append((t,)+m.groups())
    print("\n=== %s  트레이스 사이클 %d"%(tag,len(rows)))
    mode=collections.Counter(r[13] for r in rows)
    print("  모드:", dict(mode))
    # lane 단계에서 죽은 첫 이유
    lane=collections.Counter(); spl=collections.Counter()
    for r in rows:
        rj=r[14].split(',')
        lr=[x for x in rj if x.startswith('lane_')]
        sr=[x for x in rj if x.startswith('spline_')]
        lane[lr[0] if lr else ('(레인 성공/미시도)' if not lr else '')]+=1
        spl[sr[0] if sr else '(스플라인 성공/미시도)']+=1
    print("  레인스위치 1차 사망원인:")
    for k,v in lane.most_common(8): print("     %-42s %5d (%.1f%%)"%(k,v,100*v/len(rows)))
    print("  프리스플라인 1차 사망원인:")
    for k,v in spl.most_common(8): print("     %-42s %5d (%.1f%%)"%(k,v,100*v/len(rows)))
    # 회피 가능했던 프레임 = lane/spline reject가 하나도 없는
    ok=sum(1 for r in rows if not any(x.startswith(('lane_','spline_')) for x in r[14].split(',')))
    print("  회피 채택(거부 0) 사이클: %d (%.1f%%)"%(ok,100*ok/len(rows)))
    # 속도 분포 in trailing
    v=np.array([float(r[2]) for r in rows]); dist=np.array([float(r[5]) for r in rows])
    print("  트레일링 중 속도 p50/p90 %.2f/%.2f, 장애물 거리 p50 %.2f"%(np.percentile(v,50),np.percentile(v,90),np.percentile(dist,50)))
