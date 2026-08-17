import pickle,numpy as np,re
D=pickle.load(open('data.pkl','rb'))
rows=[]
for t,l,n,m in D['rosout']:
    if n=='spliner_node' and '[trace]' in m:
        tail=m.split('|')[-1]; v=re.search(r'v=([-\d.]+)',m)
        md=re.search(r'\| ([a-z_]+) \|',m)
        if v and md: rows.append((float(v.group(1)), md.group(1),
                                  'trailing_d_goal_zeroed' in tail,
                                  'trailing_d_goal_shrunk' in tail))
r=np.array([(a,c,d) for a,b,c,d in rows if b=='trailing'])
print("trailing 사이클 %d개" % len(r))
z=r[:,1]>0.5; s=r[:,2]>0.5; ok=~z&~s
print("  횡목표 유지 %d (%.0f%%) | 축소 %d (%.0f%%) | 포기 %d (%.0f%%)"
      % (ok.sum(),100*ok.mean(),s.sum(),100*s.mean(),z.sum(),100*z.mean()))
print()
print("차속별 '횡목표 포기' 비율  ← 속도가 오르면 포기가 늘어나는가?")
for lo,hi in [(0,1.0),(1.0,1.5),(1.5,2.0),(2.0,2.5),(2.5,3.0),(3.0,9)]:
    m=(r[:,0]>=lo)&(r[:,0]<hi)
    if m.sum()<20: continue
    print("  v %.1f~%.1f  n=%4d   포기 %5.1f%%   축소 %5.1f%%"
          % (lo,hi,m.sum(),100*r[m,1].mean(),100*r[m,2].mean()))
