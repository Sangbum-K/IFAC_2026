#!/usr/bin/env python3
"""ever_moved 발동 트랙과 저속 트레일링의 상관"""
import pickle,re,numpy as np
D=pickle.load(open("d17.pkl","rb")); L=pickle.load(open("log17.pkl","rb"))
ack=D["/ackermann_cmd"]
tr={}
for t,o in D["obs_frenet"]:
    if not(4<=t<=170): continue
    for ob in o: tr.setdefault(ob["id"],[]).append((ob["x"],ob["y"]))
EV=set()
for i,v in tr.items():
    a=np.array(v)
    if len(a)<2: continue
    dm=np.hypot(a[:,0][:,None]-a[:,0][None,:],a[:,1][:,None]-a[:,1][None,:])
    if dm.max()>=0.5: EV.add(i)
print("ever_moved(변위>=0.5 m) 트랙:",sorted(EV))
pat=re.compile(r"v=([-\d.]+) .*obs id=(-?\d+) dist=([-\d.]+).*\| (\w+) \|")
rows=[]
for t,lvl,n,m in L:
    if "[trace]" not in m or not(4<=t<=170): continue
    g=pat.search(m)
    if g: rows.append((t,float(g[1]),int(g[2]),float(g[3]),g[4]))
def cmd(t): return ack[np.argmin(np.abs(ack[:,0]-t))][2]
for nm,sel in (("ever_moved 트랙이 closest",lambda r:r[2] in EV),
               ("일반 정적 트랙",lambda r:r[2] not in EV)):
    g=[r for r in rows if sel(r)]
    tr_=[r for r in g if r[4]=="trailing"]
    c=[cmd(r[0]) for r in tr_]
    print("\n%s: 전체 %d 사이클, trailing %d (%.0f%%)"%(nm,len(g),len(tr_),100*len(tr_)/max(len(g),1)))
    if c:
        c=np.array(c)
        print("  그 trailing 의 명령속도 p10 %.2f p50 %.2f p90 %.2f | <1.5 m/s %.0f%% | <0.5 %.0f%%"%(
            *np.percentile(c,[10,50,90]),100*np.mean(c<1.5),100*np.mean(c<0.5)))
