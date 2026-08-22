import pickle,sys,re
D=pickle.load(open(sys.argv[1],'rb'))
t0,t1=float(sys.argv[2]),float(sys.argv[3])
pat=sys.argv[4] if len(sys.argv)>4 else None
for t,l,n,m in D['rosout']:
    if t0<=t<=t1:
        if 'rosbag2' in n or 'Frenet]' in m: continue
        if pat and not re.search(pat,n+m): continue
        print(f'{t:7.2f} {n[:18]:18s}| {m[:230]}')
