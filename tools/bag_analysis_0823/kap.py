import pickle, numpy as np
D=pickle.load(open('d43.pkl','rb'))
gw=D['global_wpnts'][0][1]
s,kap,vx=gw[:,0],gw[:,7],gw[:,8]
L=0.33; import math
kmax=math.tan(0.3220)/L
print("조향 한계 kappa = %.3f 1/m (δ=0.322, wheelbase 0.33)"%kmax)
print("레이스라인 |κ| p50/p90/max = %.3f/%.3f/%.3f"%(*np.percentile(abs(kap),[50,90]),abs(kap).max()))
print("|κ| > 0.5*kmax 구간 길이 = %.1f m / %.1f m (%.0f%%)"%((abs(kap)>0.5*kmax).sum()*0.1, s[-1], 100*(abs(kap)>0.5*kmax).mean()))
print("|κ| > 0.7*kmax = %.1f m"%((abs(kap)>0.7*kmax).sum()*0.1))
# 남는 조향 여유 -> 오프셋 경로에 쓸 수 있는 곡률
spare = 0.8*np.maximum(kmax-abs(kap), 0.15*kmax)
print("geom_floor spare p10/p50 = %.3f/%.3f"%tuple(np.percentile(spare,[10,50])))
# 어디에 장애물이 있었나
R=pickle.load(open('o43.pkl','rb'))
cs=[o['s_center'] for t,obs in R for o in obs if o['is_static']]
import collections
h=np.histogram(cs,bins=np.arange(0,35,2))
print("정적 장애물 s 분포(2m bin):", dict(zip(np.round(h[1][:-1]),h[0])))
for lo,hi in ((0,4),(30,34.4)):
    m=(s>=lo)&(s<hi)
    print("  s %g~%g: |κ| p50 %.3f max %.3f, 계획속도 p50 %.1f"%(lo,hi,np.median(abs(kap[m])),abs(kap[m]).max(),np.median(vx[m])))
