import pickle,numpy as np
LUT='/home/mina/IFAC_ws/control_ws/src/control_pkg/config/steering_lookup/IFAC_realcar_v3_pooled11bag_pacejka_lookup_table.csv'
A=np.loadtxt(LUT,delimiter=','); sp=A[0,1:]; ang=A[1:,0]; tab=A[1:,1:]
def amax_at(v,dmax=0.3578):
    j=int(np.clip(np.searchsorted(sp,v)-1,0,len(sp)-2)); w=(v-sp[j])/(sp[j+1]-sp[j])
    def col_a(jj):
        c=tab[:,jj]; f=np.isfinite(c)&(ang<=dmax)
        return np.max(c[f]) if f.any() else 0.0
    return (1-w)*col_a(j)+w*col_a(j+1)
def vmax_for(k,dmax=0.3578):
    if k<1e-4: return 9.0
    lo,hi=0.5,8.5
    for _ in range(40):
        m=0.5*(lo+hi)
        if amax_at(m,dmax)/ (m*m) >= k: lo=m
        else: hi=m
    return lo
D=pickle.load(open('d43.pkl','rb'))
gb=D['global_wpnts'][0][1]; S=gb[:,0]; K=np.abs(gb[:,7]); V=gb[:,8]
step=S[1]-S[0]
vf=np.array([vmax_for(k) for k in K])
for margin,name in [(1.00,'조향한계 정확히'),(0.90,'10% 여유'),(0.85,'15% 여유')]:
    vnew=np.minimum(V, vf*margin)
    # 감속/가속 전파(전진·후진 패스)
    a_acc,a_dec=3.0,2.38
    v=vnew.copy()
    for _ in range(3):
        for i in range(1,len(v)): v[i]=min(v[i],np.sqrt(v[i-1]**2+2*a_acc*step))
        for i in range(len(v)-2,-1,-1): v[i]=min(v[i],np.sqrt(v[i+1]**2+2*a_dec*step))
        v[0]=min(v[0],np.sqrt(v[-1]**2+2*a_acc*step))
    t0=np.sum(step/V); t1=np.sum(step/v)
    print(f"{name:14s}: 랩타임 {t0:.2f}s -> {t1:.2f}s  ({100*(t1-t0)/t0:+.1f}%)  평균속도 {S[-1]/t0:.2f} -> {S[-1]/t1:.2f} m/s")
    print(f"                 s=0~4.5 구간 계획속도 {V[S<4.5].mean():.2f} -> {v[S<4.5].mean():.2f} m/s")
print()
print("코너별 조향 가능 최고속도 (여유 없음):")
for a,b in [(0,4.5),(7.5,10),(13.5,21),(31.5,34.4)]:
    m=(S>=a)&(S<b)
    print(f"  s {a:4.1f}~{b:4.1f}: kappa max {K[m].max():.2f} -> v_max {vf[m].min():.2f} m/s (현재 계획 {V[m].mean():.2f})")
