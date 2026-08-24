import pickle,re,numpy as np
RX=re.compile(r"차체 여유 최대 후보로 이동 \(d ([-\d.]+) -> ([-\d.]+), 차체 여유 ([-\d.]+) m\)")
for tag in ('d43','d47'):
    D=pickle.load(open(tag+'.pkl','rb'))
    rows=[]
    for t,lvl,name,msg in D['rosout']:
        m=RX.search(msg)
        if m: rows.append((t,float(m.group(1)),float(m.group(2)),float(m.group(3))))
    print("=== %s  '차체 여유 최대 후보로 이동' %d회"%(tag,len(rows)))
    neg=[r for r in rows if r[3]<0]
    jump=np.array([abs(r[2]-r[1]) for r in rows])
    print("   그중 여유가 음수(=이동해도 접촉) %d회 (%.0f%%)"%(len(neg),100*len(neg)/max(len(rows),1)))
    print("   횡목표 점프량 p50/max = %.2f/%.2f m"%(np.median(jump),jump.max()))
    print("   점프가 0.30 m 초과 %d회"%(jump>0.30).sum())
    for r in rows[:50]:
        print("     t=%7.2f  d %+.2f -> %+.2f (점프 %.2f), 차체여유 %+.3f %s"%(
            r[0],r[1],r[2],abs(r[2]-r[1]),r[3], "** 음수" if r[3]<0 else ""))
