import pickle,sqlite3,sys,numpy as np
sys.path.insert(0,'/home/miru/IFAC_ws/tools')
from bagviz import CDR
def dec(b):
    c=CDR(b); ht,frame=c.header(); n=c.u32(); out=[]
    for _ in range(n):
        oid=c.i32(); px,py,_=c.f64n(3)
        v=c.f64n(14)
        st=c.u8()!=0; vis=c.u8()!=0; lap=c.i32()
        out.append(dict(id=oid,s_center=v[0],d_center=v[1],s_start=v[2],s_end=v[3],
            d_right=v[4],d_left=v[5],vs=v[6],vd=v[7],s_var=v[8],d_var=v[9],
            vs_var=v[10],vd_var=v[11],size=v[12],theta=v[13],is_static=st,is_visible=vis))
    return out
for bag,out in ((sys.argv[1],sys.argv[2]),):
    db=sqlite3.connect(bag)
    tid=db.execute("select id from topics where name='/perception/obstacles_frenet'").fetchone()[0]
    t0=db.execute("select min(timestamp) from messages").fetchone()[0]
    R=[((ts-t0)/1e9, dec(d)) for ts,d in db.execute("select timestamp,data from messages where topic_id=? order by timestamp",(tid,))]
    pickle.dump(R,open(out,'wb')); print(out,len(R))
    dv=[o['d_var'] for _,obs in R for o in obs]; vv=[o['vd_var'] for _,obs in R for o in obs]
    dv=np.array(dv); vv=np.array(vv)
    sig=3.0*np.sqrt(dv+0.09*vv)
    print(" d_var p50/p90/max %.4f %.4f %.4f"%tuple(np.percentile(dv,[50,90,100])))
    print(" 3σ(static) p50/p90/max %.3f %.3f %.3f  capped@0.12 비율=%.1f%%"%(
        *np.percentile(sig,[50,90,100]), 100*(sig>=0.12).mean()))
