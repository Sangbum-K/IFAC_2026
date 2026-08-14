#!/usr/bin/env python3
import pickle, numpy as np
import matplotlib; matplotlib.use("Agg"); import matplotlib.pyplot as plt
import matplotlib.font_manager as fm
for f in ("NanumGothic","Noto Sans CJK KR","Malgun Gothic","UnDotum"):
    if any(f in x.name for x in fm.fontManager.ttflist): plt.rcParams["font.family"]=f; break
plt.rcParams["axes.unicode_minus"]=False
print("font:",plt.rcParams["font.family"])
D=pickle.load(open("data.pkl","rb")); TL=32.07
fre=D["/car_state/frenet/odom"]; t_e,s_e,d_e,v_e=fre[:,0],fre[:,1],fre[:,2],fre[:,4]
_,gb=D["global_wpnts"][0]; gs,gdr,gdl,gvx,gkap=gb[:,0],gb[:,4],gb[:,5],gb[:,8],gb[:,7]
wrap=np.where(np.diff(s_e)<-TL*0.5)[0]; lapb=np.concatenate([[t_e[0]],t_e[wrap],[t_e[-1]]])
CONE=[("O2 square-corner (right)",6.63,-0.28),("O6 hairpin (left)",22.11,-0.32),
      ("O7 straight",28.86,+0.22),("O3 top",10.45,-0.52)]
fig,axs=plt.subplots(2,4,figsize=(22,9))
for k,(nm,so,do) in enumerate(CONE):
    axd,axv=axs[0,k],axs[1,k]
    for li in range(1,len(lapb)-2):
        m=(t_e>=lapb[li])&(t_e<lapb[li+1])&(np.abs(s_e-so)<4.0)
        if m.sum()<5: continue
        col="tab:red" if li<15 else "tab:blue"
        axd.plot(s_e[m],d_e[m],lw=.9,color=col,alpha=.55)
        axv.plot(s_e[m],v_e[m],lw=.9,color=col,alpha=.55)
    ss=np.linspace(so-4,so+4,200)
    axd.plot(ss,np.interp(ss,gs,gdl),"g--",lw=1); axd.plot(ss,-np.interp(ss,gs,gdr),"g--",lw=1)
    axd.axhline(0,color="k",lw=.7,ls=":")
    axd.add_patch(plt.Rectangle((so-0.15,do-0.15),0.30,0.30,color="orange",alpha=.9,zorder=5))
    axd.add_patch(plt.Rectangle((so-0.35,do-0.40),0.70,0.80,fc="none",ec="orange",ls=":",lw=1.2))
    axd.set_title(nm); axd.set_ylabel("d [m]"); axd.grid(alpha=.3); axd.set_xlim(so-4,so+4)
    axv.plot(ss,np.interp(ss,gs,gvx)*0.96,"k--",lw=1.6,label="plan v x0.96")
    axv.axvline(so,color="orange",lw=2,alpha=.7)
    axv.set_xlabel("s [m]"); axv.set_ylabel("v [m/s]"); axv.grid(alpha=.3); axv.set_ylim(0,6.5)
    axv.set_xlim(so-4,so+4)
    if k==0: axv.legend(fontsize=8)
axs[0,0].plot([],[],color="tab:red",label="lap1-14 (cone present)")
axs[0,0].plot([],[],color="tab:blue",label="lap15-18 (O2 cone gone)")
axs[0,0].legend(fontsize=8)
plt.tight_layout(); plt.savefig("fig_cones.png",dpi=95)
print("fig_cones.png")

# 2) 전체 랩 속도 프로파일
fig,ax=plt.subplots(2,1,figsize=(15,8),sharex=True)
for li in range(1,len(lapb)-2):
    m=(t_e>=lapb[li])&(t_e<lapb[li+1])
    col="tab:red" if li<15 else "tab:blue"
    sx=s_e[m].copy(); vy=v_e[m].copy(); dy=d_e[m].copy()
    brk=np.where(np.abs(np.diff(sx))>2.0)[0]
    sx=np.insert(sx,brk+1,np.nan); vy=np.insert(vy,brk+1,np.nan); dy=np.insert(dy,brk+1,np.nan)
    ax[0].plot(sx,vy,lw=.8,color=col,alpha=.5)
    ax[1].plot(sx,dy,lw=.8,color=col,alpha=.5)
ax[0].plot(gs,gvx*0.96,"k--",lw=2,label="plan (velocity_scale 0.96)")
ax[0].set_ylabel("speed [m/s]"); ax[0].grid(alpha=.3); ax[0].legend()
ax[1].plot(gs,gdl,"g--",lw=1); ax[1].plot(gs,-gdr,"g--",lw=1); ax[1].axhline(0,color="k",lw=.6,ls=":")
ax[1].set_ylabel("d [m]"); ax[1].set_xlabel("s [m]"); ax[1].grid(alpha=.3)
for nm,so,do in [("O2",6.63,0),("O3",10.45,0),("O4",14.99,0),("O6",22.11,0),("O7",28.86,0)]:
    for a in ax: a.axvline(so,color="orange",lw=1.5,alpha=.6)
    ax[0].text(so,6.3,nm,color="darkorange",ha="center",fontsize=10,weight="bold")
ax[0].set_title("run_0813_221645 — per-lap speed & lateral offset (red=lap1-14, blue=lap15-18)")
plt.tight_layout(); plt.savefig("fig_speed.png",dpi=95); print("fig_speed.png")
