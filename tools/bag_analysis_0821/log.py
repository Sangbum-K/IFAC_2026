import pickle,sys,re
R=pickle.load(open("rosout2.pkl","rb"))
t0,t1=float(sys.argv[1]),float(sys.argv[2])
pat=sys.argv[3] if len(sys.argv)>3 else None
for t,l,name,msg in R:
    if t0<=t<=t1 and (pat is None or re.search(pat,msg) or re.search(pat,name)):
        print(f"{t:7.2f} {name[:20]:20s} {msg}")
