import pickle,re,sys,numpy as np
RE=re.compile(r'\[trace\] s=([-\d.]+) v=([-\d.]+) d=([-\d.]+) \| obs id=(-?\d+) dist=([-\d.]+) d\[([-\d.]+),([-\d.]+)\] static=(\d) nobs=(\d+) \| evade=(\d) sector=(\d) commit=(\d) \| (\w+) \| (.*)')
def load(p):
    D=pickle.load(open(p,'rb'))
    out=[]
    for t,l,n,m in D['rosout']:
        if n!='spliner_node' or '[trace]' not in m: continue
        g=RE.search(m)
        if not g: continue
        out.append(dict(t=t,s=float(g.group(1)),v=float(g.group(2)),d=float(g.group(3)),
            oid=int(g.group(4)),dist=float(g.group(5)),dl=float(g.group(7)),dr=float(g.group(6)),
            static=int(g.group(8)),nobs=int(g.group(9)),evade=int(g.group(10)),
            commit=int(g.group(12)),outcome=g.group(13),rej=g.group(14)))
    return D,out
if __name__=='__main__':
    D,T=load(sys.argv[1])
    print(len(T),"traces")
    from collections import Counter
    c=Counter(x['outcome'] for x in T); print(c)
