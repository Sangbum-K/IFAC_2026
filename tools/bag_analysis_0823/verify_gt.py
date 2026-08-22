"""global_trajectory_node 의 새 BuildSteerSpeedCap 을 그대로 파이썬으로 재현해
실제 CSV 에 대한 효과(감속량·랩타임)를 잰다."""
import numpy as np, sys
CSV=sys.argv[1]
BRK=np.array([1.5,2.0,2.5,3.0,3.5,4.0,4.5,5.0,5.5,6.0,6.5,7.0])
ACC=np.array([2.27,3.63,5.26,6.72,8.14,9.69,9.60,8.27,6.41,6.00,5.49,4.93])
MARGIN=1.0
VSCALE=float(sys.argv[2]) if len(sys.argv)>2 else 1.0
KIN=np.tan(0.3220)/0.33      # validation_max_steer_rad 기본값
def lat_at(v):
    return MARGIN*np.interp(np.abs(v),BRK,ACC)   # 양끝 클램프 = np.interp 기본
def cap_for(k):
    k=abs(k)
    if k<1e-4: return np.inf
    if k>KIN:  return 0.0
    best=0.0; v=0.02
    while v<=BRK[-1]+1e-9:
        if k*v*v>lat_at(v): break
        best=v; v+=0.02
    if best>=BRK[-1]-1e-9: return np.sqrt(ACC[-1]*MARGIN/k)
    return best
rows=[]
for line in open(CSV):
    line=line.strip()
    if not line or line.startswith('#'): continue
    p=[float(x) for x in line.replace(';',',').split(',')]
    if len(p)==9: rows.append(p)
T=np.array(rows); n=len(T)
s,kap,vx,ax=T[:,0],T[:,6],T[:,7],T[:,8]
print(f"{CSV.split('/')[-1]}  n={n}  L={s[-1]:.2f} m  vx {vx.min():.2f}~{vx.max():.2f}")
cap=np.array([cap_for(k) for k in kap])
a_acc=max(ax.max(),0.5); a_dec=max((-ax).max(),0.5)
for _ in range(2):
    for k in range(1,n+1):
        i,p=k%n,(k-1)%n; ds=abs(s[i]-s[p])
        if not (1e-6<ds<=1.0): continue
        cap[i]=min(cap[i],np.sqrt(cap[p]**2+2*a_acc*ds))
    for k in range(n,0,-1):
        i,nx=(k-1)%n,k%n; ds=abs(s[nx]-s[i])
        if not (1e-6<ds<=1.0): continue
        cap[i]=min(cap[i],np.sqrt(cap[nx]**2+2*a_dec*ds))
pub_old=vx*VSCALE
pub_new=np.minimum(pub_old,cap)
ds=np.diff(np.append(s,s[-1]+ (s[1]-s[0])))
t_old=np.sum(ds/np.maximum(pub_old,0.05)); t_new=np.sum(ds/np.maximum(pub_new,0.05))
cut=(pub_new<pub_old-1e-6)
print(f"velocity_scale={VSCALE}")
print(f"  감속된 점 {cut.sum()}/{n} ({100*cut.mean():.1f}%)   최소 발행속도 {pub_new.min():.2f} m/s")
print(f"  랩타임 {t_old:.2f} -> {t_new:.2f} s  ({100*(t_new-t_old)/t_old:+.1f}%)")
if cut.any():
    i=np.argmin(pub_new-pub_old)
    print(f"  최악 s={s[i]:.2f} kappa={kap[i]:.2f}  {pub_old[i]:.2f} -> {pub_new[i]:.2f} m/s")
    # 구간 묶기
    seg=[];cur=None
    for i,b in enumerate(cut):
        if b and cur is None: cur=i
        if not b and cur is not None: seg.append((cur,i-1)); cur=None
    if cur is not None: seg.append((cur,n-1))
    print("  감속 구간:")
    for a,b in seg:
        print(f"    s {s[a]:5.2f}~{s[b]:5.2f}  |kappa|max {np.abs(kap[a:b+1]).max():.2f}  "
              f"v {pub_old[a:b+1].mean():.2f} -> {pub_new[a:b+1].mean():.2f}")
