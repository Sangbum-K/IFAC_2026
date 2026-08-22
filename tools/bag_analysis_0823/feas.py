import pickle,sys,numpy as np
LUT='/home/mina/IFAC_ws/control_ws/src/control_pkg/config/steering_lookup/IFAC_realcar_v3_pooled11bag_pacejka_lookup_table.csv'
A=np.loadtxt(LUT,delimiter=','); sp=A[0,1:]; ang=A[1:,0]; tab=A[1:,1:]
def kappa_max(v,dmax=0.3578):
    j=np.clip(np.searchsorted(sp,v)-1,0,len(sp)-2)
    w=(v-sp[j])/(sp[j+1]-sp[j])
    out=[]
    for jj,ww in ((j,1-w),(j+1,w)):
        col=tab[:,jj]; fin=np.isfinite(col)
        a=np.nanmax(np.where(fin&(ang<=dmax),col,-np.inf))
        out.append(a*ww)
    return sum(out)/max(v*v,1e-6)
D=pickle.load(open(sys.argv[1],'rb'))
gb=D['global_wpnts'][0][1]; S=gb[:,0]; K=np.abs(gb[:,7]); V=gb[:,8]
scale=float(sys.argv[2]) if len(sys.argv)>2 else 1.0
V=V*scale
km=np.array([kappa_max(v) for v in V])
ratio=K/np.maximum(km,1e-6)
bad=ratio>1.0
print(f"속도배율 {scale:.2f}: 레이스라인 {len(S)}점 중 조향으로 낼 수 없는 점 {bad.sum()} ({100*bad.mean():.1f}%)")
if bad.any():
    print("  요구/가능 비 max %.2f  (그 지점 s=%.1f kappa=%.2f v=%.2f 가능kappa=%.2f)"%(
        ratio.max(),S[ratio.argmax()],K[ratio.argmax()],V[ratio.argmax()],km[ratio.argmax()]))
    # 구간 묶기
    seg=[];cur=None
    for i,b in enumerate(bad):
        if b and cur is None: cur=i
        if not b and cur is not None: seg.append((cur,i-1)); cur=None
    if cur is not None: seg.append((cur,len(bad)-1))
    for a,b in seg:
        print(f"    s {S[a]:5.2f}~{S[b]:5.2f}  kappa {K[a:b+1].max():.2f}  v {V[a:b+1].mean():.2f}  요구/가능 {ratio[a:b+1].max():.2f}")
