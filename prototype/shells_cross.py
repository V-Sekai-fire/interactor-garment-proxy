# Throwaway measurement (pure Python, no numpy): welded shells, edge use, and exact edge-vs-triangle crossings.
import sys, collections
def load(p):
    V=[];F=[]
    for l in open(p):
        t=l.split()
        if not t: continue
        if t[0]=="v": V.append(tuple(float(x) for x in t[1:4]))
        elif t[0]=="f":
            ids=[int(x.split("/")[0])-1 for x in t[1:]]
            for k in range(1,len(ids)-1): F.append((ids[0],ids[k],ids[k+1]))
    return V,F
def sub(a,b): return (a[0]-b[0],a[1]-b[1],a[2]-b[2])
def cross(a,b): return (a[1]*b[2]-a[2]*b[1],a[2]*b[0]-a[0]*b[2],a[0]*b[1]-a[1]*b[0])
def dot(a,b): return a[0]*b[0]+a[1]*b[1]+a[2]*b[2]
def seg_tri(p,q,a,b,c):
    # Moller-Trumbore on the closed segment p-q; strict interior of the segment and triangle
    d=sub(q,p); e1=sub(b,a); e2=sub(c,a)
    h=cross(d,e2); det=dot(e1,h)
    if det==0.0: return False
    f=1.0/det; s=sub(p,a); u=f*dot(s,h)
    if u<=0.0 or u>=1.0: return False
    qq=cross(s,e1); v=f*dot(d,qq)
    if v<=0.0 or u+v>=1.0: return False
    t=f*dot(e2,qq)
    return 0.0<t<1.0
def analyse(path):
    V,F=load(path)
    lo=[min(v[i] for v in V) for i in range(3)]; hi=[max(v[i] for v in V) for i in range(3)]
    size=max(hi[i]-lo[i] for i in range(3)); q=size*1e-5
    ids={}; rm=[]
    for v in V: rm.append(ids.setdefault(tuple(round(c/q) for c in v),len(ids)))
    P=[None]*len(ids)
    for i,v in enumerate(V): P[rm[i]]=v
    T=[]; seen=set()
    for a,b,c in F:
        t=(rm[a],rm[b],rm[c])
        if len(set(t))<3: continue
        k=tuple(sorted(t))
        if k in seen: continue
        seen.add(k); T.append(t)
    par=list(range(len(P)))
    def f(x):
        while par[x]!=x: par[x]=par[par[x]]; x=par[x]
        return x
    for a,b,c in T: par[f(a)]=f(b); par[f(b)]=f(c)
    shell={}
    for a,b,c in T: shell[(a,b,c)]=f(a)
    nsh=len(set(shell.values()))
    e=collections.Counter()
    for a,b,c in T:
        for u,v in ((a,b),(b,c),(c,a)): e[(min(u,v),max(u,v))]+=1
    nm=sum(1 for n in e.values() if n>=3); op=sum(1 for n in e.values() if n==1)
    # grid broad phase on triangles
    cell=size/40.0
    grid=collections.defaultdict(list)
    bb=[]
    for i,(a,b,c) in enumerate(T):
        pts=(P[a],P[b],P[c]); mn=[min(p[k] for p in pts) for k in range(3)]; mx=[max(p[k] for p in pts) for k in range(3)]
        bb.append((mn,mx))
        for x in range(int((mn[0]-lo[0])//cell),int((mx[0]-lo[0])//cell)+1):
            for y in range(int((mn[1]-lo[1])//cell),int((mx[1]-lo[1])//cell)+1):
                for z in range(int((mn[2]-lo[2])//cell),int((mx[2]-lo[2])//cell)+1):
                    grid[(x,y,z)].append(i)
    pairs=set()
    for lst in grid.values():
        for ii in range(len(lst)):
            for jj in range(ii+1,len(lst)):
                i,j=lst[ii],lst[jj]
                if i>j: i,j=j,i
                if (i,j) in pairs: continue
                if set(T[i])&set(T[j]): continue
                mi,xi=bb[i]; mj,xj=bb[j]
                if any(xi[k]<mj[k] or xj[k]<mi[k] for k in range(3)): continue
                pairs.add((i,j))
    cross_pairs=0; same=0
    for i,j in pairs:
        hit=False
        for (A,B) in ((i,j),(j,i)):
            a,b,c=(P[x] for x in T[B])
            for u,v in ((T[A][0],T[A][1]),(T[A][1],T[A][2]),(T[A][2],T[A][0])):
                if seg_tri(P[u],P[v],a,b,c): hit=True; break
            if hit: break
        if hit:
            cross_pairs+=1
            if shell[T[i]]==shell[T[j]]: same+=1
    return dict(verts=len(P),tris=len(T),shells=nsh,sizes=sorted(collections.Counter(shell.values()).values(),reverse=True)[:5],nonmanifold=nm,open_edges=op,crossing_pairs=cross_pairs,same_shell=same,cross_shell=cross_pairs-same)
if __name__=="__main__":
    for p in sys.argv[1:]: print(p.split("/")[-1], analyse(p))
