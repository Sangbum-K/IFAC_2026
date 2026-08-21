import pickle,numpy as np,collections
rows=pickle.load(open("trace.pkl","rb"))
eps=[];cur=[]
for r in rows:
    if cur and r[0]-cur[-1][0]>0.7: eps.append(cur);cur=[]
    cur.append(r)
eps.append(cur)
c_early=collections.Counter(); c_all=collections.Counter(); n_e=0;n_a=0
mode_early=collections.Counter()
for e in eps:
    if e[-1][0]-e[0][0]<1.5: continue
    if np.mean([r[8] for r in e])>0.5: continue   # 동적만
    t0=e[0][0]
    for r in e:
        tags=[x for x in r[14].split(",") if x and not x.startswith("+")]
        early = r[0]-t0 <= 1.0
        if early:
            n_e+=1; mode_early[r[13]]+=1
            for t in set(tags): c_early[t]+=1
        n_a+=1
        for t in set(tags): c_all[t]+=1
print(f"동적 에피소드 — 탐지 후 1초 이내 사이클 {n_e}개, 전체 {n_a}개")
print("탐지 후 1초 이내 모드:", dict(mode_early))
print(f"\n{'사유':44s} {'≤1s':>7} {'전체':>7}")
for k,v in c_early.most_common(18):
    print(f"{k:44s} {v/n_e*100:6.0f}% {c_all[k]/n_a*100:6.0f}%")
