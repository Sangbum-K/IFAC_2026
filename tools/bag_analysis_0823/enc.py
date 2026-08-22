import pickle,sys,numpy as np
R=pickle.load(open(sys.argv[1],'rb'))
R=[r for r in R if r['static']]
# encounter = 같은 oid 연속 (0.5s 이상 끊기면 새 조우)
enc=[];cur=[]
for r in R:
    if cur and (r['oid']!=cur[-1]['oid'] or r['t']-cur[-1]['t']>0.5):
        enc.append(cur);cur=[]
    cur.append(r)
if cur:enc.append(cur)
enc=[e for e in enc if len(e)>=5]
print(f"조우 {len(enc)}건")
nf=0;nlate=0;nnever=0;flips=0
rows=[]
for e in enc:
    openL=[r for r in e if r['defL']<=0]
    openR=[r for r in e if r['defR']<=0]
    # 첫 관측 거리
    d0=e[0]['dist']
    firstopen=max([r["dist"] for r in e if r["defL"]<=0 or r["defR"]<=0],default=None)
    # 열린 쪽이 도중에 바뀌었나
    seq=[]
    for r in e:
        s=('L' if r['defL']<=0 else '')+('R' if r['defR']<=0 else '')
        seq.append(s or '-')
    ch=sum(1 for i in range(1,len(seq)) if seq[i]!=seq[i-1])
    flips+=ch
    rows.append((e[0]['t'],e[0]['oid'],d0,firstopen,len(e),
                 100*len(openL)/len(e),100*len(openR)/len(e),ch))
    if firstopen is None: nnever+=1
print(" t0     id  첫관측m  첫개방m  n  L열림%  R열림%  개방상태변화")
for r in rows:
    fo = f"{r[3]:6.2f}" if r[3] is not None else "  없음"
    print(f"{r[0]:7.2f} {r[1]:4d} {r[2]:7.2f} {fo} {r[4]:4d} {r[5]:6.0f} {r[6]:6.0f} {r[7]:6d}")
print(f"\n한 번도 열리지 않은 조우: {nnever}/{len(enc)}")
print(f"개방상태(L/R/양쪽/없음) 변화 총 {flips}회, 조우당 평균 {flips/len(enc):.1f}회")
