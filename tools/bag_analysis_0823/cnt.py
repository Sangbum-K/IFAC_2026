import pickle, numpy as np, sys
for tag in ('d43','d47'):
    D=pickle.load(open(tag+'.pkl','rb'))
    print("=== %s"%tag)
    rows=[]
    for k,v in D.items():
        if isinstance(k,str) and k.startswith('/pitwall/') and isinstance(v,np.ndarray) and v.ndim==2 and v.shape[1]==2 and len(v):
            rows.append((k, len(v), float(v[:,1].max()), float(v[-1,1])))
    for k,n,mx,last in sorted(rows,key=lambda r:-r[3]):
        if 'reject' in k or 'outcome' in k or 'cluster' in k or 'passed' in k:
            print("  %-58s last=%8.0f n=%d"%(k.replace('/pitwall/',''),last,n))
