import sys, json, time
sys.path.insert(0,".")
import numpy as np
from dfp import run
from measure_proxy import shells, edges, crossings
D,H=0.002,0.001
def sheet(n,size,z=0.0):
    xs=np.linspace(0,size,n+1); V=np.array([[x,y,z] for x in xs for y in xs]); F=[]
    for i in range(n):
        for j in range(n):
            a=i*(n+1)+j; b=a+1; c=a+n+1; d=c+1; F+= [[a,c,b],[b,c,d]]
    return V,np.array(F)
def join(parts):
    V=[];F=[];o=0
    for v,f in parts: V.append(v); F.append(f+o); o+=len(v)
    return np.vstack(V),np.vstack(F)
def rot(V,rng):
    q=rng.normal(size=4); q/=np.linalg.norm(q); w,x,y,z=q
    R=np.array([[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)],[2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],[2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]])
    return V@R.T
def tube(rng,n=24,m=8):
    r=rng.uniform(0.010,0.018); L=0.03; V=[];F=[]
    for j in range(m+1):
        for i in range(n): t=2*np.pi*i/n; V.append([r*np.cos(t),L*j/m,r*np.sin(t)])
    for j in range(m):
        for i in range(n):
            a=j*n+i; b=j*n+(i+1)%n; c=a+n; d=b+n; F+=[[a,c,b],[b,c,d]]
    return np.array(V),np.array(F)
def make(level,rng):
    if level=="flat, on the grid":
        V,F=sheet(8,0.03+0.001*rng.integers(0,5)); return V,F,1
    if level=="flat, off the grid":
        V,F=sheet(8,0.03); V=V+rng.uniform(0,0.001,3)*[0,0,1]; return V,F,1
    if level=="open tube, rotated":
        V,F=tube(rng); return rot(V,rng)+rng.uniform(0,0.001,3),F,1
    if level=="two layers 3 mm apart":
        a=sheet(8,0.03); b=sheet(8,0.03,0.003+rng.uniform(-0.0004,0.0004)); V,F=join([a,b]); return V+rng.uniform(0,0.001,3),F,1
    if level=="two layers 6 mm apart":
        a=sheet(8,0.03); b=sheet(8,0.03,0.006+rng.uniform(-0.0004,0.0004)); V,F=join([a,b]); return V+rng.uniform(0,0.001,3),F,2
    if level=="fold at 90 degrees":
        a=sheet(8,0.03); v,f=sheet(8,0.03); v=np.stack([v[:,0]*0,v[:,1],v[:,0]],1)
        V,F=join([a,(v,f)]); return rot(V,rng)+rng.uniform(0,0.001,3),F,None
LEVELS=["flat, on the grid","flat, off the grid","open tube, rotated","two layers 3 mm apart","two layers 6 mm apart","fold at 90 degrees"]
def checks(V,T,expect):
    if len(T)==0: return dict(closed=False,distinct=False,area=False,shells=False,cross=False)
    o,m,nm=edges(T)
    k=np.round(V/1e-6).astype(np.int64); _,cnt=np.unique(k,axis=0,return_counts=True)
    a=0.5*np.linalg.norm(np.cross(V[T[:,1]]-V[T[:,0]],V[T[:,2]]-V[T[:,0]]),axis=1)
    sh=shells(T,len(V))
    cr=crossings(V,T,0.002)[1] if len(T)<60000 else -1
    return dict(closed=(o==0 and nm==0),distinct=bool((cnt==1).all()),area=bool((a>=1e-12).all()),shells=(expect is None or sh==expect),cross=(cr==0))
def evaluate(variant,seeds):
    res={}
    for lv in LEVELS:
        outs=[]
        for s in seeds:
            rng=np.random.default_rng(1000*s+LEVELS.index(lv)); V,F,ex=make(lv,rng)
            VP,T=run(V,F,D,H,variant); c=checks(VP,T,ex); outs.append(c)
        res[lv]=outs
    return res
def summary(res):
    tot=0; rows=[]
    for lv,outs in res.items():
        s=sum(all(c.values()) for c in outs); n=len(outs); tot+=s
        fails={k:sum(not c[k] for c in outs) for k in outs[0]}
        rows.append((lv,s,n,1.0*(s>=1)-s/n,fails))
    return tot,rows
if __name__=="__main__":
    seeds=list(range(3)); t0=time.time(); log=[]
    order=["base","planted","nets","nets_centre"]
    best=None; kept=[]
    for v in order:
        res=evaluate(v,seeds); tot,rows=summary(res)
        print("== variant",v,"frontier success %d of %d  (%.0fs)"%(tot,len(LEVELS)*len(seeds),time.time()-t0),flush=True)
        for lv,s,n,reg,fails in rows: print("   %-26s s=%d/%d regret=%+.2f  failing checks per 8: %s"%(lv,s,n,reg,{k:x for k,x in fails.items() if x}),flush=True)
        log.append((v,tot))
