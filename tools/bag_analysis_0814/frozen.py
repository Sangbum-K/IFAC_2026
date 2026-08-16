import pickle,numpy as np,re
D=pickle.load(open('data.pkl','rb'))
ot=D['otwpnts']; fr=D['/car_state/frenet/odom']
ft=fr[:,0]; fv=fr[:,4]
rows=[]
for t,l,n,m in D['rosout']:
    if n=='spliner_node' and '[trace]' in m:
        mm=re.search(r'\| ([a-z_]+) \|',m)
        if mm: rows.append((t,mm.group(1)))
rt=np.array([r[0] for r in rows]); rm=np.array([r[1] for r in rows])
hold=(rm=='committed_hold').astype(int)
ch=np.diff(hold); st=rt[1:][ch==1]; en=rt[1:][ch==-1]
if len(en) and len(st) and en[0]<st[0]: en=en[1:]
N=min(len(st),len(en))
res=[]
for t,a in ot:
    if len(a)<20 or t<40 or t>478: continue
    k=np.searchsorted(st[:N],t)-1
    if k<0 or t>en[k] or t<st[k]: continue
    if t-st[k] < 0.15: continue          # 커밋 직후는 제외
    v_now=float(np.interp(t,ft,fv)); v_at_commit=float(np.interp(st[k],ft,fv))
    res.append((a[0,8], v_now, v_at_commit, t-st[k]))
r=np.array(res)
print("committed_hold 유지 중 사이클 %d개" % len(r))
print()
print("                              중앙값")
print("  경로 머리 속도 (동결값)      %.2f" % np.median(r[:,0]))
print("  지금 차속                    %.2f" % np.median(r[:,1]))
print("  커밋 생성 시점 차속          %.2f" % np.median(r[:,2]))
print()
for a,b,lab in [(0,0.5,'커밋 후 0~0.5초'),(0.5,1.5,'0.5~1.5초'),(1.5,99,'1.5초 이상')]:
    m=(r[:,3]>=a)&(r[:,3]<b)
    if m.sum()<30: continue
    print("%-16s n=%5d | 머리 %.2f  차속 %.2f  생성시차속 %.2f | 머리-생성시차속 상관 %.2f"
          % (lab,m.sum(),np.median(r[m,0]),np.median(r[m,1]),np.median(r[m,2]),
             np.corrcoef(r[m,0],r[m,2])[0,1]))
print()
print("머리 속도와의 상관:  지금 차속 %.2f   vs   커밋 생성시 차속 %.2f"
      % (np.corrcoef(r[:,0],r[:,1])[0,1], np.corrcoef(r[:,0],r[:,2])[0,1]))
