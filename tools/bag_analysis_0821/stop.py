import pickle,numpy as np,collections
D=pickle.load(open("d.pkl","rb"))
rows=pickle.load(open("trace.pkl","rb"))
ack=D["/ackermann_cmd"]; pf=D["/pf/pose/odom"]
def at(a,t,c):
    i=np.searchsorted(a[:,0],t); i=min(max(i,0),len(a)-1); return a[i,c]
# 명령 0 근처 구간
c=ack[:,2]; t=ack[:,0]
lo=c<0.3
seg=[];cur=None
for i in range(len(t)):
    if lo[i]:
        cur=(t[i],t[i]) if cur is None else (cur[0],t[i])
    elif cur:
        if cur[1]-cur[0]>0.3: seg.append(cur)
        cur=None
print(f"명령속도 < 0.3 m/s 가 0.3초 이상 지속된 구간 {len(seg)}개")
rt=np.array([r[0] for r in rows])
for a,b in seg:
    m=(rt>a-0.3)&(rt<b+0.1)
    tags=collections.Counter()
    modes=collections.Counter()
    dmin=[]
    for r in [rows[i] for i in np.where(m)[0]]:
        modes[r[13]]+=1; dmin.append(r[5])
        for x in r[14].split(","):
            if x and not x.startswith("+"): tags[x]+=1
    v0=at(pf,a,4)
    print(f"  {a:7.2f}~{b:7.2f} ({b-a:.2f}s) v={v0:.2f} 모드={dict(modes) if modes else '(trace 없음)'} dist_min={min(dmin) if dmin else float('nan'):.2f}")
    if tags: print(f"      사유: {', '.join(k for k,_ in tags.most_common(6))}")
# 양쪽 막힘 카운터가 실제로 튄 시각
for k in ("/pitwall/reject_side_both_blocked","/pitwall/reject_escape_both_blocked"):
    a=D[k]; d=np.diff(a[:,1]); idx=np.where(d>0)[0]
    print(f"\n{k}: 최종 {a[-1,1]:.0f}, 증가 이벤트 {len(idx)}개, 시각 {[round(a[i+1,0],1) for i in idx[:15]]}")
