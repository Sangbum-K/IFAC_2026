# 3" 궤적 생성 마진 0.08 m — 되돌릴 때의 득실
import lib2, numpy as np
d=lib2.D(); m=lib2.Map(d); g=lib2.gw(d)
S,X,Y,DR,DL,PSI=g[:,0],g[:,2],g[:,3],g[:,4],g[:,5],g[:,6]
TL=S[-1]+(S[1]-S[0])
def mod(x): return x%TL
def bidx(s): return int(np.argmin(np.abs(S-mod(s))))
CLR=0.27; BND=0.20; NEED=CLR+BND

# ── 이득: 마진을 Δ 되돌리면 통로가 몇 건이나 열리나 ──────────────────────
print("=== 3\" 이득: 경계를 Δ 만큼 넓혔을 때 열리는 통로 ===")
print(" Δ[m]   좌통로 열림   우통로 열림   어느 쪽이든 열림(신규)")
base=None
for D_ in (0.0,0.02,0.04,0.06,0.08,0.10):
    L=R=E=0; tot=0
    for t,obs in d['obs_frenet']:
        for o in obs:
            if not np.isfinite(o['s_center']): continue
            bi=bidx(o['s_center']); wl,wr=DL[bi]+D_,-(DR[bi]+D_)
            tot+=1
            l=(wl-o['d_left'])>=NEED; r=(o['d_right']-wr)>=NEED
            L+=l; R+=r; E+=(l or r)
    if base is None: base=(L,R,E)
    print(f" {D_:.2f}   {L:6d}       {R:6d}       {E-base[2]:+6d}   (전체 {tot})")

# ── 손실: 실제로 차가 벽에 얼마나 붙어 달렸나 (맵 EDT 기준) ──────────────
pf=d['/pf/pose/odom']; fr=d['/car_state/frenet/odom']
mv=np.abs(fr[:,4])>0.5
tmv=fr[mv,0]
sel=np.isin(np.round(pf[:,0],2), np.round(tmv,2))
px,py=pf[:,1],pf[:,2]
ok=(pf[:,0]>3.2)&(pf[:,0]<24.6)|((pf[:,0]>82.4)&(pf[:,0]<152.7))
dist=m.dist(px[ok],py[ok])          # 차 중심 -> 가장 가까운 맵 벽
half=0.15
clear=dist-half                     # 차체 표면 여유
print("\n=== 3\" 손실: 주행 중 차체 표면 ~ 실제 맵 벽 여유 ===")
for q in (0.1,1,5,25,50):
    print(f"  p{q:<4}: {np.percentile(clear,q):.3f} m")
for th in (0.0,0.05,0.10,0.15,0.20):
    print(f"  여유 < {th:.2f} m 인 시간 비율: {100*np.mean(clear<th):.1f}%")
