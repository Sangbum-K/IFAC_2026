#!/usr/bin/env python3
"""새 게이트의 오프라인 검증: 정지 사이클마다 옛/새 게이트와 그때의 명령을 재계산.
컨트롤러 계산을 그대로 흉내낸다 — v=0 이면 조회점은 발밑, 게이트가 열리면
경로를 따라 0.25 m 앞."""
import pickle,re,sys,numpy as np
D=pickle.load(open(sys.argv[1],"rb")); L=pickle.load(open(sys.argv[2],"rb"))
MIN_M=0.25
lw=D["local_wpnts"]; ack=D["/ackermann_cmd"]; cmt=D["/planner/avoidance/evade_committed"]
pat=re.compile(r"s=([-\d.]+) v=([-\d.]+) d=([-\d.]+) \| obs id=(-?\d+) dist=([-\d.]+).*?\| (\w+) \| (.*)$")
rows=[]
for t,lvl,n,m in L:
    if "[trace]" not in m: continue
    g=pat.search(m)
    if g: rows.append((t,float(g.group(2)),float(g.group(5)),g.group(6),g.group(7)))
def at(l,t,tol=0.15):
    b=None;bd=9
    for tt,o in l:
        if abs(tt-t)<bd: bd=abs(tt-t);b=o
    return b if bd<tol else None
def near(a,t):
    return a[np.argmin(np.abs(a[:,0]-t))] if len(a) else None
old_open=new_open=0; n=0
cmd_old=[];cmd_new=[]; still0=0; risky=0
for t,v,dist,mode,tags in rows:
    if abs(v)>=0.10: continue          # 정지 사이클만
    w=at(lw,t)
    if w is None or len(w)==0: continue
    n+=1
    C=near(cmt,t)
    og = C is not None and abs(C[0]-t)<0.2 and C[1]>0.5
    # 새 게이트: 옛 게이트 OR (트레일링 && 정지권한 박탈)
    ng = og or (mode=="trailing" and "trailing_retry_clamp_raised" in tags)
    old_open+=og; new_open+=ng
    ds=np.r_[0,np.cumsum(np.hypot(np.diff(w[:,2]),np.diff(w[:,3])))]
    v_foot=w[0,8]
    k=int(np.searchsorted(ds,MIN_M)); k=min(k,len(w)-1)
    v25=w[k,8]
    co = v25 if og else v_foot
    cn = v25 if ng else v_foot
    cmd_old.append(co); cmd_new.append(cn)
    if cn<0.05: still0+=1
    if cn>=0.05 and dist>=0 and dist<0.78: risky+=1
cmd_old=np.array(cmd_old); cmd_new=np.array(cmd_new)
print("정지 사이클 %d"%n)
print("  게이트 열림:  옛 %d (%.1f%%)  ->  새 %d (%.1f%%)"%(
    old_open,100*old_open/n,new_open,100*new_open/n))
print("  명령 0 비율:  옛 %.1f%%  ->  새 %.1f%%"%(
    100*np.mean(cmd_old<0.05),100*np.mean(cmd_new<0.05)))
print("  명령 중앙값:  옛 %.2f  ->  새 %.2f m/s"%(np.median(cmd_old),np.median(cmd_new)))
print("  풀린 사이클 %d (남은 0: %d)"%(int(np.sum((cmd_old<0.05)&(cmd_new>=0.05))),still0))
print("  발진하는데 장애물 0.78 m 이내: %d (%.1f%%)"%(risky,100*risky/max(new_open,1)))
