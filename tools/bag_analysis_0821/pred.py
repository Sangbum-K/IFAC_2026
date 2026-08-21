import pickle, numpy as np
D=pickle.load(open("d.pkl","rb"))
P=D["obs_pred"]
n=len(P); ne=sum(1 for t,o in P if len(o)>0)
print(f"opponent_prediction 발행 {n}, 비어있지 않은 것 {ne} ({ne/max(n,1)*100:.1f}%)")
for t,o in P:
    if len(o)>0:
        print(f"  t={t:.2f} n={len(o)} {[(x['id'],round(x['s_center'],2),round(x['d_center'],2),round(x['vs'],2)) for x in o]}")
        break
# rosout: prediction 관련
R=pickle.load(open("rosout2.pkl","rb"))
import re
from collections import Counter
c=Counter()
for t,l,name,msg in R:
    c[(name, re.sub(r'[-+0-9.]+','N',msg)[:100])]+=1
print("\n=== rosout 상위 40 ===")
for (nm,m),k in c.most_common(40):
    print(f"{k:6d}  {nm:28s} {m}")
