import lib2, numpy as np
from collections import defaultdict
d=lib2.D()
hist=defaultdict(list)
for t,obs in d['obs_frenet']:
    for o in obs:
        hist[o['id']].append((t, o['d_left']-o['d_right']))
jumps=[]; per=[]
for i,h in hist.items():
    if len(h)<5: continue
    w=np.array([x[1] for x in h]); tt=np.array([x[0] for x in h])
    dw=np.abs(np.diff(w)); dt=np.diff(tt)
    m=dt<0.15                      # 연속 프레임만
    if m.sum()<4: continue
    per.append((i,len(h),w.min(),np.median(w),w.max(),w.max()/max(w.min(),1e-3)))
    jumps.append(dw[m])
J=np.concatenate(jumps)
print(f"트랙 {len(per)}개 / 연속프레임 폭변화 {len(J)}건")
print(f"  |Δ폭| p50 {np.median(J):.3f}  p90 {np.percentile(J,90):.3f}  p99 {np.percentile(J,99):.3f}  max {J.max():.3f} m")
for th in (0.15,0.30,0.50):
    print(f"  한 프레임에 폭이 {th:.2f} m 이상 튄 비율: {100*np.mean(J>th):.2f}%")
print("\n폭 변동이 큰 트랙 (max/min 비):")
for i,n,mn,md,mx,r in sorted(per,key=lambda z:-z[5])[:10]:
    print(f"  id{i:4d} n={n:4d} 폭 min {mn:.2f} / p50 {md:.2f} / max {mx:.2f}  비 {r:.1f}x")
print("\n=== id18 / id19 폭 시계열 (23.1~24.3) ===")
for i in (18,19):
    s=[(round(t,3),round(w,2)) for t,w in hist[i] if 23.1<=t<=24.3]
    print(f" id{i}: "+" ".join(f"{w:.2f}" for _,w in s))
