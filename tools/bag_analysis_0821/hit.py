import pickle,numpy as np
D=pickle.load(open("d.pkl","rb"))
imu=D["/sensors/imu/raw"]   # t,yaw,wx,wy,wz,ax,ay,az
print("imu",imu.shape)
t=imu[:,0]; wz=imu[:,4]; ax=imu[:,5]; ay=imu[:,6]
# yaw rate 급반전 (0.1s 내 부호 반전 + 크기)
ev=[]
for i in range(5,len(t)-5):
    if abs(wz[i-3])>3 and abs(wz[i+3])>3 and np.sign(wz[i-3])!=np.sign(wz[i+3]):
        ev.append(t[i])
ev=np.array(ev)
# 군집화
if len(ev):
    grp=[[ev[0]]]
    for x in ev[1:]:
        if x-grp[-1][-1]>1.5: grp.append([])
        grp[-1].append(x)
    print(f"yaw rate 급반전 군집 {len(grp)}개")
fo=D["/car_state/frenet/odom"]; pf=D["/pf/pose/odom"]
# 급감속: 0.3초 안에 1.5 m/s 이상 감소
v=pf[:,4]; tv=pf[:,0]
dec=[]
for i in range(len(tv)):
    j=np.searchsorted(tv,tv[i]+0.4)
    if j>=len(tv): break
    if v[i]-v[j] > 1.8 and v[i]>1.5: dec.append((tv[i],v[i],v[j]))
grp=[]
for x in dec:
    if not grp or x[0]-grp[-1][-1][0]>2.0: grp.append([])
    grp[-1].append(x)
print(f"\n급감속(0.4s 내 -1.8 m/s) 군집 {len(grp)}개:")
rows=pickle.load(open("trace.pkl","rb"))
rt=np.array([r[0] for r in rows]); rd=np.array([r[5] for r in rows]); rst=np.array([r[8] for r in rows])
for g in grp:
    t0=g[0][0]
    k=np.searchsorted(rt,t0)
    near=(rt>t0-1.0)&(rt<t0+0.5)
    d=rd[near]; s=rst[near]
    tag=""
    if near.sum(): tag=f"직전 dist min {d.min():.2f} static={'D' if s.mean()<0.5 else 'S'}"
    print(f"  t={t0:7.2f} v {g[0][1]:.2f}->{g[-1][2]:.2f}  {tag}")
