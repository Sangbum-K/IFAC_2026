#!/usr/bin/env python3
"""조우별 손실 시간 계산 + 최악 조우의 [trace] 타임라인"""
import pickle,re,numpy as np
D=pickle.load(open("d17.pkl","rb")); L=pickle.load(open("log17.pkl","rb"))
pf=D["/pf/pose/odom"]; fre=D["/car_state/frenet/odom"]; gw=D["global_wpnts"][0][1]; TL=gw[-1,0]
pat=re.compile(r"s=([-\d.]+) v=([-\d.]+) d=([-\d.]+) \| obs id=(-?\d+) dist=([-\d.]+) d\[([-\d.]+),([-\d.]+)\] static=(\d) nobs=(\d+) \| evade=(\d) sector=(\d) commit=(\d) \| (\w+) \| (.*)$")
R=[]
for t,lvl,n,m in L:
    if "[trace]" not in m or not(4<=t<=170): continue
    g=pat.search(m)
    if g: R.append((t,g.groups(),m))
R.sort()
blocks=[];cur=[R[0]]
for a,b in zip(R,R[1:]):
    if b[0]-a[0]<0.3: cur.append(b)
    else: blocks.append(cur); cur=[b]
blocks.append(cur)
def gbv(s): return gw[np.argmin(np.abs(gw[:,0]-(s%TL))),8]
out=[]
for b in blocks:
    t0,t1=b[0][0],b[-1][0]
    m=(pf[:,0]>=t0-0.3)&(pf[:,0]<=t1+1.0)
    P=pf[m]
    if len(P)<3: continue
    # 이동거리 / 실제시간 vs 같은 거리를 0.89*gb 로 갔을 때
    F=[fre[np.argmin(np.abs(fre[:,0]-t)),1] for t in P[:,0]]
    ds=np.diff([f for f in F]); ds=np.where(ds<-TL/2,ds+TL,ds); ds=np.clip(ds,0,1)
    dist=ds.sum()
    dt=P[-1,0]-P[0,0]
    vref=np.array([0.89*gbv(f) for f in F[:-1]])
    dt_ref=np.sum(ds/np.maximum(vref,0.3))
    out.append((dt-dt_ref,t0,t1,dist,dt,dt_ref,np.median(P[:,4]),b))
out.sort(reverse=True)
print("조우 %d회 | 총 손실 %.1f s (기준: 회피 없는 랩의 실측 페이스 0.89*gb)"%(
    len(out),sum(o[0] for o in out)))
print("\n손실 상위 8:")
for loss,t0,t1,dist,dt,dtr,vm,b in out[:8]:
    print("  t %6.2f-%6.2f  손실 %+5.2fs (실제 %.2fs / 기준 %.2fs, %.1fm, v중앙 %.2f)"%(
        t0,t1,loss,dt,dtr,dist,vm))
print("\n=== 최악 조우 타임라인 ===")
for t,g,m in out[0][7]:
    print("  %.2f %s"%(t,m.split("[trace] ")[1][:165]))
