import pickle,sys,numpy as np
R=pickle.load(open(sys.argv[1],'rb'))
R=[r for r in R if r['static']]
enc=[];cur=[]
for r in R:
    if cur and (r['oid']!=cur[-1]['oid'] or r['t']-cur[-1]['t']>0.5): enc.append(cur);cur=[]
    cur.append(r)
if cur: enc.append(cur)
HW=0.15
print(" t0     id  통과시 d   장애물 d[..]   차체여유(m)  판정  |횡이동|  진입v  최소여유거리")
res=[]
for e in enc:
    if len(e)<5: continue
    # dist 가 0 을 지나는 지점
    k=None
    for i in range(1,len(e)):
        if e[i-1]['dist']>0>=e[i]['dist']: k=i;break
    if k is None: continue
    r=e[k]
    slack=min(abs(r['d']-r['dl']),abs(r['d']-r['dr']))
    inside = r['dr']-HW <= r['d'] <= r['dl']+HW
    clear = 0.0 if inside else slack-HW
    d0=e[0]['d']; move=abs(r['d']-d0); v0=e[0]['v']
    res.append((inside,move,v0,clear))
    print(f"{e[0]['t']:7.2f} {r['oid']:4d}  {r['d']:+.2f}   [{r['dr']:+.2f},{r['dl']:+.2f}]   {clear:+.3f}   {'충돌' if inside else '통과':4s}  {move:.2f}   {v0:.2f}")
res=np.array([(1 if a else 0,b,c,d) for a,b,c,d in res])
hit=res[res[:,0]==1]; ok=res[res[:,0]==0]
print(f"\n충돌판정 {len(hit)} / 통과 {len(ok)}")
if len(hit): print(f"  충돌: |횡이동| med {np.median(hit[:,1]):.2f}  진입속도 med {np.median(hit[:,2]):.2f}")
if len(ok):  print(f"  통과: |횡이동| med {np.median(ok[:,1]):.2f}  진입속도 med {np.median(ok[:,2]):.2f}  여유 med {np.median(ok[:,3]):.3f}")
