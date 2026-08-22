import pickle, numpy as np
from scipy import ndimage

_D=None
def D():
    global _D
    if _D is None:
        _D=pickle.load(open('d.pkl','rb'))
    return _D

class Map:
    def __init__(self,d):
        m=d['map']; self.res=m['res']; self.ox=m['ox']; self.oy=m['oy']
        self.w=m['w']; self.h=m['h']
        g=np.array(m['data']).reshape(m['h'],m['w'])
        self.occ = g>50           # wall
        self.unknown = g<0
        free = ~(self.occ|self.unknown)
        # distance to nearest occupied-or-unknown cell (metres)
        self.edt = ndimage.distance_transform_edt(~(self.occ|self.unknown))*self.res
    def dist(self,x,y):
        x=np.atleast_1d(np.asarray(x,float)); y=np.atleast_1d(np.asarray(y,float))
        j=np.clip(((x-self.ox)/self.res).astype(int),0,self.w-1)
        i=np.clip(((y-self.oy)/self.res).astype(int),0,self.h-1)
        return self.edt[i,j]

def gw(d):
    return d['global_wpnts'][0][1]   # s d x y d_r d_l psi kappa vx ax

def at_idx(arr,t):
    return int(np.clip(np.searchsorted(arr[:,0],t),0,len(arr)-1))

def near(lst,t):
    """lst = [(t, payload)] -> payload nearest t"""
    ts=np.array([x[0] for x in lst])
    i=int(np.clip(np.searchsorted(ts,t),0,len(ts)-1))
    return lst[i]
