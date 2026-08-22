#!/usr/bin/env python3
"""구간 내 outcome/reject 카운터 증가량"""
import pickle,sys,numpy as np
D=pickle.load(open(sys.argv[1],"rb")); t0,t1=float(sys.argv[2]),float(sys.argv[3])
rows=[]
for k,v in D.items():
    if not isinstance(k,str) or not k.startswith("/pitwall/"): continue
    if not isinstance(v,np.ndarray) or v.ndim!=2 or len(v)==0: continue
    m=(v[:,0]>=t0)&(v[:,0]<=t1)
    if m.sum()<1: continue
    seg=v[m]
    d=seg[-1,1]-seg[0,1]
    rows.append((d,k,seg[0,1],seg[-1,1],len(seg)))
rows.sort(reverse=True)
for d,k,a,b,n in rows:
    if abs(d)>0: print("%+8.0f  %-58s %.0f -> %.0f  (n%d)"%(d,k.replace("/pitwall/",""),a,b,n))
