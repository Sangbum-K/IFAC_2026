"""perception 전단 오프라인 재현: 벽필터 -> 트랙필터 -> 적응 클러스터링 -> 반경 게이트."""
import pickle, sqlite3, sys, numpy as np
from pathlib import Path
sys.path.insert(0,"/home/miru/IFAC_ws/tools")
from bagviz import dec_scan
from scipy.ndimage import distance_transform_edt, median_filter
P=Path(__file__).parent
D=pickle.load(open(P/"d124904.pkl","rb"))
BAG="/home/miru/IFAC_ws/rosbag0817/run_0817_124904/run_0817_124904_0.db3"
db=sqlite3.connect(BAG); t0=db.execute("SELECT MIN(timestamp) FROM messages").fetchone()[0]
tid=dict((n,i) for i,n,_ in db.execute("SELECT id,name,type FROM topics"))["/scan"]
pf=D["/pf/pose/odom"]; fre=D["/car_state/frenet/odom"]; gw=D["global_wpnts"][0][1]
mp=D["map"]; res=mp["res"]; ox,oy=mp["ox"],mp["oy"]
grid=np.array(mp["data"]).reshape(mp["h"],mp["w"]); occ=grid>50
EDT=distance_transform_edt(~occ)*res            # free-space distance to nearest wall
GX,GY,GS,GPSI,GDR,GDL=gw[:,2],gw[:,3],gw[:,0],gw[:,6],gw[:,4],gw[:,5]
TRACK_LEN=37.51
LX=0.28
RANGE_MIN,RANGE_MAX=0.1,12.0; FOV=np.radians(80.0)
WALL_MIN=0.25; INFL=0.10; VIEW_A, VIEW_B=8.0,2.0
LAM=np.radians(10.0); SIG=0.015; MINC=8; MINC_OPEN=4; OPEN_D=0.50
MAXR=0.35
def pose(tt):
    return (np.interp(tt,pf[:,0],pf[:,1]), np.interp(tt,pf[:,0],pf[:,2]),
            np.arctan2(np.interp(tt,pf[:,0],np.sin(pf[:,3])),np.interp(tt,pf[:,0],np.cos(pf[:,3]))))
def edt_at(X,Y):
    ix=np.clip(((X-ox)/res).astype(int),0,mp["w"]-1); iy=np.clip(((Y-oy)/res).astype(int),0,mp["h"]-1)
    return EDT[iy,ix]
def frenet(X,Y):
    d2=(X[:,None]-GX[None,:])**2+(Y[:,None]-GY[None,:])**2; j=d2.argmin(1)
    nx,ny=-np.sin(GPSI[j]),np.cos(GPSI[j])
    return GS[j],(X-GX[j])*nx+(Y-GY[j])*ny, GDR[j], GDL[j]
def cluster(pts, rng, dphi, minsize):
    n=len(pts)
    if n<minsize: return []
    denom=max(np.sin(LAM-dphi),1e-6); sdp=np.sin(dphi)
    d=np.linalg.norm(np.diff(pts,axis=0),axis=1)
    rp=0.5*(rng[:-1]+rng[1:]); dmax=(rp*sdp/denom+3*SIG)*0.5
    brk=np.where(d>dmax)[0]+1
    out=[];beg=0
    for b in list(brk)+[n]:
        if b-beg>=minsize:
            seg=pts[beg:b]; c=seg.mean(0); r=np.sqrt(((seg-c)**2).sum(1).max())
            out.append((c,r,b-beg,seg))
        beg=b
    return out
rows=[]
for ts,d in db.execute("SELECT timestamp,data FROM messages WHERE topic_id=? ORDER BY timestamp",(tid,)):
    tt=(ts-t0)/1e9
    if not (57.5<=tt<=60.3): continue
    ht,amin,ainc,tinc,r=dec_scan(d,1); r=np.asarray(r,float)
    ang=amin+ainc*np.arange(len(r))
    r=median_filter(r,3)
    ok=np.isfinite(r)&(r>=RANGE_MIN)&(r<=RANGE_MAX)&(np.abs(ang)<=FOV)
    r=r[ok]; ang=ang[ok]
    x,y=r*np.cos(ang)+LX, r*np.sin(ang)
    px,py,yw=pose(tt); c,sn=np.cos(yw),np.sin(yw)
    X=px+c*x-sn*y; Y=py+sn*x+c*y
    keep=edt_at(X,Y)>=WALL_MIN
    S,Dd,dr,dl=frenet(X,Y)
    ego_s=np.interp(tt,fre[:,0],fre[:,1])
    gap=(S-ego_s)%TRACK_LEN; gap=np.where(gap>TRACK_LEN/2,gap-TRACK_LEN,gap)
    keep &= (Dd> -(dr-INFL)) & (Dd< (dl-INFL)) & (gap<=VIEW_A) & (gap>=-VIEW_B)
    XX,YY,RR=X[keep],Y[keep],r[keep]
    cl=cluster(np.stack([XX,YY],1),RR,ainc,MINC)
    big=[k for k in cl if k[1]>MAXR]
    surv=[k for k in cl if k[1]<=MAXR]
    # 자차 전방에 있는 것만
    def fwd(k):
        s,dd,_,_=frenet(np.array([k[0][0]]),np.array([k[0][1]]))
        g=(s[0]-ego_s)%TRACK_LEN; g=g-TRACK_LEN if g>TRACK_LEN/2 else g
        return g,dd[0]
    rows.append((tt,ego_s,cl,big,surv,fwd))
print("  t     ego_s  | 거부된 초대형 클러스터 (반경, 점수, 전방거리, d)      | 통과 클러스터")
for tt,ego_s,cl,big,surv,fwd in rows:
    if not (58.2<=tt<=60.2): continue
    bs=" ".join(f"[r={k[1]:.2f} n={k[2]} @{fwd(k)[0]:+.2f}m d={fwd(k)[1]:+.2f}]" for k in big)
    ss=" ".join(f"(r={k[1]:.2f} n={k[2]} @{fwd(k)[0]:+.2f}m d={fwd(k)[1]:+.2f})" for k in surv)
    print(f"  {tt:5.2f} {ego_s:6.2f} | {bs:70s} | {ss}")
