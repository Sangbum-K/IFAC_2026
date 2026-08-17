import pickle,numpy as np,re,collections
D=pickle.load(open('data.pkl','rb'))
ot=D['otwpnts']; fr=D['/car_state/frenet/odom']
ft=fr[:,0]; fv=fr[:,4]
rows=[]
for t,l,n,m in D['rosout']:
    if n=='spliner_node' and '[trace]' in m:
        mm=re.search(r'\| ([a-z_]+) \|',m)
        if mm: rows.append((t,mm.group(1)))
rt=np.array([r[0] for r in rows]); rm=np.array([r[1] for r in rows])
dip=collections.Counter(); tot=collections.Counter()
idx10=[]
for t,a in ot:
    if len(a)<20 or t<40 or t>478: continue
    j=int(np.argmin(abs(rt-t)))
    if abs(rt[j]-t)>0.1: continue
    md=rm[j]; v0=float(np.interp(t,ft,fv))
    tot[md]+=1
    if a[0,8] < 0.7*max(v0,.1):
        dip[md]+=1
        idx10.append((a[0,8], a[min(10,len(a)-1),8], v0))
print("머리(idx0) 낙폭 사이클의 모드 분포")
for k,v in dip.most_common():
    print("  %-16s %5d / %5d  (%4.1f%% of mode)" % (k,v,tot[k],100*v/tot[k]))
q=np.array(idx10)
print()
print("낙폭 사이클에서:  idx0=%.2f  ->  idx10(컨트롤러가 읽는 ~1m)=%.2f   차속=%.2f"
      % (np.median(q[:,0]), np.median(q[:,1]), np.median(q[:,2])))
print("  idx10 도 차속의 70%% 미만인 비율: %.1f%%" % (100*np.mean(q[:,1]<0.7*q[:,2])))
