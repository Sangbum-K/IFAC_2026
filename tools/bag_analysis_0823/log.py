import pickle, sys
D=pickle.load(open(sys.argv[3] if len(sys.argv)>3 else 'd43.pkl','rb'))
t0,t1=float(sys.argv[1]),float(sys.argv[2])
for t,lvl,name,msg in D['rosout']:
    if t0<=t<=t1 and 'spliner' in name.lower() or (t0<=t<=t1 and lvl>=30):
        if t0<=t<=t1: print("%7.2f [%s] %s"%(t,name.split('.')[-1],msg))
