import pickle, numpy as np, sys
def load(tag):
    D=pickle.load(open(f'{tag}.pkl','rb'))
    D['rosout2']=pickle.load(open(f'{tag}_rosout.pkl','rb'))
    gw=D['global_wpnts'][0][1]
    D['L']=gw[-1,0]+ (gw[-1,0]-gw[-2,0])
    D['gw']=gw
    return D
def at(arr,t):
    if len(arr)==0: return None
    i=np.searchsorted(arr[:,0],t); return arr[min(max(i,0),len(arr)-1)]
def wrap(ds,L):
    ds=ds%L
    return ds-L if ds>L/2 else ds
