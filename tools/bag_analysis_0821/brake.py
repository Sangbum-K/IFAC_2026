import pickle,numpy as np,collections
D=pickle.load(open("d.pkl","rb"))
L=36.833
gw=D["global_wpnts"][0][1]   # s d x y dr dl psi kappa vx ax
gs=gw[:,0]; gv=gw[:,8]
def vref(s): return np.interp(s%L,gs,gv)
fo=D["/car_state/frenet/odom"]; ack=D["/ackermann_cmd"]
rows=pickle.load(open("trace.pkl","rb"))
# 동적 에피소드
eps=[];cur=[]
for r in rows:
    if cur and r[0]-cur[-1][0]>0.7: eps.append(cur);cur=[]
    cur.append(r)
eps.append(cur)
def cmdv(t):
    i=np.searchsorted(ack[:,0],t); i=min(max(i,0),len(ack)-1); return ack[i,2]
print(f"{'t0':>7} {'dist0':>6} {'v0':>5} {'vref0':>6} | {'제동시작 dist':>12} {'그때 v':>7} {'최저명령':>8} {'최저 dist':>9} {'통과?':>5}")
out=[]
for e in eps:
    if e[-1][0]-e[0][0]<1.5: continue
    dyn=1-np.mean([r[8] for r in e])
    if dyn<0.5: continue
    v0=e[0][2]; d0=e[0][5]; s0=e[0][1]
    vr0=vref(s0)
    bt=None
    for r in e:
        c=cmdv(r[0])
        if c < 0.8*vref(r[1]):
            bt=r; break
    dists=[r[5] for r in e]
    cmds=[cmdv(r[0]) for r in e]
    passed = min(dists)<-0.5
    out.append((e[0][0],d0,v0,vr0,bt[5] if bt else np.nan,bt[2] if bt else np.nan,min(cmds),min(dists),passed))
    print(f"{e[0][0]:7.1f} {d0:6.2f} {v0:5.2f} {vr0:6.2f} | {(bt[5] if bt else float('nan')):12.2f} {(bt[2] if bt else float('nan')):7.2f} {min(cmds):8.2f} {min(dists):9.2f} {'O' if passed else 'X':>5}")
a=np.array([(o[1],o[4],o[6],o[7]) for o in out],dtype=float)
print(f"\n동적 에피소드 {len(out)}개")
print(f"  최초탐지 거리 p50 {np.nanmedian(a[:,0]):.2f} m")
print(f"  '레이스라인 80% 미만으로 명령이 떨어진' 시점의 거리 p50 {np.nanmedian(a[:,1]):.2f} m  (탐지 후 {np.nanmedian(a[:,0]-a[:,1]):.2f} m 만에)")
print(f"  통과 성공(장애물 뒤로 -0.5m 이상 지나감) {sum(1 for o in out if o[8])}/{len(out)}")
