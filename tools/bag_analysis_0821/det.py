import pickle, numpy as np, sys
from collections import defaultdict
D=pickle.load(open("d.pkl","rb"))
L=36.833
fo=D["/car_state/frenet/odom"]
def ego(t):
    i=np.searchsorted(fo[:,0],t); i=min(max(i,0),len(fo)-1); return fo[i]
tr=defaultdict(list)
for t,o in D["obs_frenet"]:
    e=ego(t)
    for ob in o:
        ds=(ob["s_center"]-e[1])%L
        if ds>L/2: ds-=L
        tr[ob["id"]].append((t,ds,ob["d_center"],ob["vs"],ob["vd"],ob["is_static"],ob["is_visible"],ob["size"],ob["s_start"],e[1],e[4]))
print(f"tracks={len(tr)}  frames={len(D['obs_frenet'])}")
rows=[]
for i,v in tr.items():
    a=np.array([(x[0],x[1],x[2],x[3],x[5],x[6],x[7]) for x in v])
    life=a[-1,0]-a[0,0]
    st=a[:,4]
    flips=int(np.sum(np.abs(np.diff(st))>0.5))
    dyn_frac=1-st.mean()
    rows.append((i,len(v),life,a[0,0],a[0,1],dyn_frac,flips,a[:,2].std(),np.abs(a[:,3]).mean()))
rows.sort(key=lambda r:-r[1])
print(f"{'id':>6} {'n':>5} {'life':>6} {'t0':>7} {'ds0':>6} {'dyn%':>5} {'flip':>4} {'std_d':>6} {'|vs|':>5}")
for r in rows[:30]:
    print(f"{r[0]:6d} {r[1]:5d} {r[2]:6.2f} {r[3]:7.1f} {r[4]:+6.2f} {r[5]*100:5.0f} {r[6]:4d} {r[7]:6.3f} {r[8]:5.2f}")
A=np.array([(r[1],r[2],r[4],r[5],r[6]) for r in rows])
print(f"\n총 트랙 {len(rows)}, 관측>=5 인 트랙 {int((A[:,0]>=5).sum())}")
print(f"수명 p50 {np.median(A[:,1]):.2f}s p90 {np.percentile(A[:,1],90):.2f}s  1초 미만 비율 {(A[:,1]<1).mean()*100:.0f}%")
m=A[:,0]>=10
print(f"관측>=10 트랙 {m.sum()}개: 최초관측거리 p50 {np.median(A[m,2]):.2f} p10 {np.percentile(A[m,2],10):.2f} p90 {np.percentile(A[m,2],90):.2f} m")
print(f"  분류 flip 합 {A[m,4].sum():.0f}, flip>=1 트랙 {(A[m,4]>=1).sum()}개")
