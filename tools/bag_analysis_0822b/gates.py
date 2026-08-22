#!/usr/bin/env python3
"""새 게이트 3종의 실제 발화: ever_moved / corridor_block / commit_lat_dev"""
import pickle,numpy as np
D=pickle.load(open("d17.pkl","rb"))
T0,T1=4,170
tr={}
for t,o in D["obs_frenet"]:
    if not(T0<=t<=T1): continue
    for ob in o: tr.setdefault(ob["id"],[]).append((t,ob["x"],ob["y"],ob["is_static"]))
disp={}
for i,v in tr.items():
    a=np.array([[x[1],x[2]] for x in v])
    d=np.max(np.hypot(a[:,0]-a[0,0],a[:,1]-a[0,1])) if len(a)>1 else 0.0
    # 최대 상호 변위(첫점 기준이 아니라 전체 span)
    span=np.max(np.hypot(a[:,0][:,None]-a[:,0][None,:],a[:,1][:,None]-a[:,1][None,:])) if len(a)<400 else d
    disp[i]=(d,span,len(v),all(x[3] for x in v))
dd=np.array([v[1] for v in disp.values()])
print("[ever_moved 게이트: ever_moved_min_disp_m = 0.5]")
print("  트랙 %d개  최대변위 span p50 %.2f p90 %.2f max %.2f"%(len(dd),np.median(dd),np.percentile(dd,90),dd.max()))
ev=[i for i,v in disp.items() if v[1]>=0.5]
inst=sum(disp[i][2] for i in ev)
tot=sum(v[2] for v in disp.values())
print("  span>=0.5 인 트랙 %d개 (%.0f%%), 발행 인스턴스 %d/%d (%.0f%%)"%(
    len(ev),100*len(ev)/len(disp),inst,tot,100*inst/tot))
print("  그중 항상 is_static 이던 트랙: %d개"%sum(1 for i in ev if disp[i][3]))
for k in ("/planner/corridor_blocked","/planner/avoidance/evade_committed"):
    a=D.get(k)
    if a is None or len(a)==0: print(k,"없음"); continue
    m=(a[:,0]>T0)&(a[:,0]<T1); x=a[m]
    print("\n[%s] n=%d  true %.1f%%"%(k,len(x),100*x[:,1].mean()))
    # 연속 true 구간
    on=x[:,1]>0.5; segs=[];i=0
    while i<len(x):
        if on[i]:
            j=i
            while j+1<len(x) and on[j+1]: j+=1
            segs.append((x[i,0],x[j,0])); i=j+1
        else: i+=1
    segs=[s for s in segs if s[1]-s[0]>0.05]
    print("  연속구간 %d개, 총 %.1fs, 최장 %.2fs"%(len(segs),sum(b-a for a,b in segs),max((b-a for a,b in segs),default=0)))
    for a2,b2 in sorted(segs,key=lambda s:-(s[1]-s[0]))[:6]:
        print("    %.2f-%.2f (%.2fs)"%(a2,b2,b2-a2))
a=D.get("/pitwall/commit_lat_dev")
if a is not None and len(a):
    m=(a[:,0]>T0)&(a[:,0]<T1); x=a[m,1]
    print("\n[commit_lat_dev] n=%d p50 %.3f p90 %.3f p99 %.3f max %.3f | >=0.40 %.1f%%"%(
        len(x),np.median(x),np.percentile(x,90),np.percentile(x,99),x.max(),100*np.mean(x>=0.40)))
for k in ("/pitwall/outcome_corridor_block_stop_latched","/pitwall/outcome_cluster_merge_blocks_track",
          "/pitwall/avoidance_loss_hold","/pitwall/overtake_active","/pitwall/follow_active"):
    a=D.get(k)
    if a is None or len(a)==0: continue
    m=(a[:,0]>T0)&(a[:,0]<T1); x=a[m]
    print("%-52s n=%4d  중앙 %.2f max %.2f"%(k,len(x),np.median(x[:,1]) if len(x) else -1,x[:,1].max() if len(x) else -1))
