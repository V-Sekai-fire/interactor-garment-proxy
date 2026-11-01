# Prototype measurement (numpy): shells, edge use, and sampled edge-vs-triangle crossings.
import sys, time
import numpy as np
def load(p):
    V=[];F=[]
    for l in open(p):
        if l.startswith("v "): V.append([float(x) for x in l.split()[1:4]])
        elif l.startswith("f "): F.append([int(x.split("/")[0])-1 for x in l.split()[1:4]])
    return np.array(V),np.array(F)
def shells(F,n):
    par=np.arange(n)
    def find(x):
        while par[x]!=x:
            par[x]=par[par[x]]; x=par[x]
        return x
    for a,b,c in F.tolist():
        ra,rb=find(a),find(b)
        if ra!=rb: par[ra]=rb
        rc=find(c); rb=find(b)
        if rc!=rb: par[rc]=rb
    roots=np.array([find(i) for i in np.unique(F)])
    return len(np.unique(roots))
def edges(F):
    E=np.concatenate([F[:,[0,1]],F[:,[1,2]],F[:,[2,0]]]); E.sort(1)
    k=E[:,0].astype(np.int64)*(F.max()+1)+E[:,1]
    _,c=np.unique(k,return_counts=True)
    return int((c==1).sum()),int((c==2).sum()),int((c>=3).sum())
def seg_tri(p,q,a,b,c):
    d=q-p; e1=b-a; e2=c-a
    h=np.cross(d,e2); det=(e1*h).sum(1)
    ok=np.abs(det)>1e-18
    f=np.where(ok,1.0/np.where(ok,det,1),0); s=p-a
    u=f*(s*h).sum(1); qq=np.cross(s,e1); v=f*(d*qq).sum(1); t=f*(e2*qq).sum(1)
    return ok&(u>0)&(u<1)&(v>0)&(u+v<1)&(t>0)&(t<1)
def crossings(V,F,cell):
    cen=(V[F[:,0]]+V[F[:,1]]+V[F[:,2]])/3
    lo=V.min(0); ci=np.floor((cen-lo)/cell).astype(np.int64)
    M=ci.max(0)+3; key=(ci[:,0]+1)*M[1]*M[2]+(ci[:,1]+1)*M[2]+(ci[:,2]+1)
    order=np.argsort(key); ks=key[order]
    pairs=[]
    for dx in (-1,0,1):
      for dy in (-1,0,1):
        for dz in (-1,0,1):
          off=dx*M[1]*M[2]+dy*M[2]+dz
          tgt=key+off
          lo_i=np.searchsorted(ks,tgt,"left"); hi_i=np.searchsorted(ks,tgt,"right")
          cnt=hi_i-lo_i
          rep=np.repeat(np.arange(len(F)),cnt)
          if len(rep)==0: continue
          starts=np.repeat(lo_i,cnt); within=np.arange(len(rep))-np.repeat(np.cumsum(cnt)-cnt,cnt)
          other=order[starts+within]
          m=rep<other
          pairs.append(np.stack([rep[m],other[m]],1))
    P=np.unique(np.concatenate(pairs),axis=0)
    share=np.zeros(len(P),bool)
    for i in range(3):
        for j in range(3): share|=F[P[:,0],i]==F[P[:,1],j]
    P=P[~share]
    hit=np.zeros(len(P),bool)
    for s in range(0,len(P),400000):
        pp=P[s:s+400000]
        A=V[F[pp[:,0]]]; B=V[F[pp[:,1]]]
        h=np.zeros(len(pp),bool)
        for X,Y in ((A,B),(B,A)):
            for k in range(3):
                h|=seg_tri(X[:,k],X[:,(k+1)%3],Y[:,0],Y[:,1],Y[:,2])
        hit[s:s+400000]=h
    return len(P),int(hit.sum())
if __name__=="__main__":
    V,F=load(sys.argv[1]); t0=time.time()
    print("vertices",len(V),"triangles",len(F))
    print("shells",shells(F,len(V)),"(%.0fs)"%(time.time()-t0),flush=True)
    print("open / manifold / non-manifold edges",edges(F),flush=True)
    # planted second shell control: a far-away triangle must raise the count by 1
    V2=np.vstack([V,V.max(0)+[1,1,1],V.max(0)+[1,1.01,1],V.max(0)+[1,1,1.01]]); F2=np.vstack([F,[len(V),len(V)+1,len(V)+2]])
    print("control, planted far triangle -> shells",shells(F2,len(V2)),"(must be one more)",flush=True)
    rng=np.random.default_rng(1); lo,hi=V.min(0),V.max(0); cen=(V[F[:,0]]+V[F[:,1]]+V[F[:,2]])/3
    tot=0;hits=0
    for r in range(4):
        c=cen[rng.integers(len(cen))]; m=np.all(np.abs(cen-c)<0.02,axis=1)
        sub=F[m]; used=np.unique(sub); mp=-np.ones(len(V),int); mp[used]=np.arange(len(used))
        n,h=crossings(V[used],mp[sub],0.003); tot+=n; hits+=h
        print("  box %d: %d triangles, %d candidate pairs, %d crossing"%(r,len(sub),n,h),flush=True)
    print("sampled: %d candidate pairs, %d crossing pairs"%(tot,hits))
    # control: one planted piercing triangle in the last box
    c=cen[0]; m=np.all(np.abs(cen-c)<0.02,axis=1); sub=F[m]; used=np.unique(sub); mp=-np.ones(len(V),int); mp[used]=np.arange(len(used))
    Vs=V[used]; Fs=mp[sub]; t=Vs[Fs[0]]; n0=t.mean(0); nrm=np.cross(t[1]-t[0],t[2]-t[0]); nrm/=np.linalg.norm(nrm)
    pv=np.array([n0+nrm*0.004+[0.0002,0,0],n0-nrm*0.004+[0.0002,0,0],n0+np.cross(nrm,[0,0,1])*0.004+nrm*0.001])
    Vp=np.vstack([Vs,pv]); Fp=np.vstack([Fs,[len(Vs),len(Vs)+1,len(Vs)+2]])
    print("control, planted piercing triangle -> crossing pairs",crossings(Vp,Fp,0.003)[1],"(must be >= 1)")
