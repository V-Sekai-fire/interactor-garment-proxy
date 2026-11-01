# Prototype only (numpy is blocklisted for the final pipeline): level surface of the unsigned distance to a mesh.
import sys, time
import numpy as np
src, out, D, H = sys.argv[1], sys.argv[2], float(sys.argv[3]), float(sys.argv[4])
V=[];F=[]
for l in open(src):
    t=l.split()
    if not t: continue
    if t[0]=="v": V.append([float(x) for x in t[1:4]])
    elif t[0]=="f":
        ids=[int(x.split("/")[0])-1 for x in t[1:]]
        for k in range(1,len(ids)-1): F.append([ids[0],ids[k],ids[k+1]])
V=np.array(V); F=np.array(F)
pad=D+3*H
org=V.min(0)-pad; dims=np.ceil((V.max(0)+pad-org)/H).astype(int)+1
nx,ny,nz=dims; print("grid",nx,ny,nz,"=%.1fM"%(nx*ny*nz/1e6),flush=True)
G=np.full((nx,ny,nz),np.float32(1e9),dtype=np.float32)

def closest(P,a,b,c):
    ab=b-a; ac=c-a; ap=P-a
    d1=ap@ab; d2=ap@ac
    bp=P-b; d3=bp@ab; d4=bp@ac
    cp=P-c; d5=cp@ab; d6=cp@ac
    vc=d1*d4-d3*d2; vb=d5*d2-d1*d6; va=d3*d6-d5*d4
    out=np.empty_like(P)
    # face region by default
    den=va+vb+vc
    den=np.where(den==0,1e-30,den)
    v=vb/den; w=vc/den
    out=a+ab*v[:,None]+ac*w[:,None]
    m=(d1<=0)&(d2<=0); out[m]=a
    m=(d3>=0)&(d4<=d3); out[m]=b
    m=(d6>=0)&(d5<=d6); out[m]=c
    m=(vc<=0)&(d1>=0)&(d3<=0); t=d1/np.where(d1-d3==0,1e-30,d1-d3); out[m]=(a+ab*t[:,None])[m]
    m=(vb<=0)&(d2>=0)&(d6<=0); t=d2/np.where(d2-d6==0,1e-30,d2-d6); out[m]=(a+ac*t[:,None])[m]
    m=(va<=0)&((d4-d3)>=0)&((d5-d6)>=0); t=(d4-d3)/np.where((d4-d3)+(d5-d6)==0,1e-30,(d4-d3)+(d5-d6)); out[m]=(b+(c-b)*t[:,None])[m]
    return np.linalg.norm(P-out,axis=1)

t0=time.time(); R=D+1.5*H
for i,(ia,ib,ic) in enumerate(F):
    a,b,c=V[ia],V[ib],V[ic]
    lo=np.minimum(np.minimum(a,b),c)-R; hi=np.maximum(np.maximum(a,b),c)+R
    i0=np.maximum(np.floor((lo-org)/H).astype(int),0); i1=np.minimum(np.ceil((hi-org)/H).astype(int),dims-1)
    xs=np.arange(i0[0],i1[0]+1); ys=np.arange(i0[1],i1[1]+1); zs=np.arange(i0[2],i1[2]+1)
    X,Y,Z=np.meshgrid(xs,ys,zs,indexing="ij")
    P=np.stack([X.ravel(),Y.ravel(),Z.ravel()],1)*H+org
    dist=closest(P,a,b,c).astype(np.float32).reshape(X.shape)
    sl=G[i0[0]:i1[0]+1,i0[1]:i1[1]+1,i0[2]:i1[2]+1]
    np.minimum(sl,dist,out=sl)
print("distance field %.1fs"%(time.time()-t0),flush=True)

f=G-np.float32(D)
inside=f<0
cx=np.zeros((nx,ny,nz),bool)
s=np.zeros((nx-1,ny-1,nz-1),np.int8)
for ox in (0,1):
  for oy in (0,1):
    for oz in (0,1):
      s+=inside[ox:nx-1+ox,oy:ny-1+oy,oz:nz-1+oz]
act=np.argwhere((s>0)&(s<8))
print("active cubes",len(act),flush=True)
corner=np.array([[0,0,0],[1,0,0],[1,1,0],[0,1,0],[0,0,1],[1,0,1],[1,1,1],[0,1,1]])
tets=[(0,5,1,6),(0,1,2,6),(0,2,3,6),(0,3,7,6),(0,7,4,6),(0,4,5,6)]
cid=lambda p: (p[...,0]*ny+p[...,1])*nz+p[...,2]
tri_edges=[]
for tt in tets:
    cp=np.stack([act+corner[k] for k in tt],1)          # (n,4,3)
    val=f[cp[...,0],cp[...,1],cp[...,2]]                  # (n,4)
    gid=cid(cp)                                           # (n,4)
    ins=val<0
    nin=ins.sum(1)
    for mask in range(1,15):
        bits=[(mask>>k)&1 for k in range(4)]
        sel=np.all(ins==np.array(bits,bool),axis=1)
        if not sel.any(): continue
        I=[k for k in range(4) if bits[k]]; O=[k for k in range(4) if not bits[k]]
        if len(I)==1 or len(O)==1:
            lone,others=(I[0],O) if len(I)==1 else (O[0],I)
            e=[(lone,o) for o in others]
            tri_edges.append((gid[sel],val[sel],cp[sel],[(e[0],e[1],e[2])]))
        else:
            a,b=I; c,dd=O
            tri_edges.append((gid[sel],val[sel],cp[sel],[((a,c),(a,dd),(b,dd)),((a,c),(b,dd),(b,c))]))
keys=[];tris=[]
epos={}
allk=[];allp=[]
for gid,val,cp,tl in tri_edges:
    for t in tl:
        ks=[];ps=[]
        for (p,q) in t:
            g0=gid[:,p];g1=gid[:,q]
            lo_=np.minimum(g0,g1);hi_=np.maximum(g0,g1)
            ks.append(lo_*np.int64(nx*ny*nz)+hi_)
            v0=val[:,p];v1=val[:,q]; tt_=(v0/(v0-v1))[:,None]
            ps.append(cp[:,p]*(1-tt_)+cp[:,q]*tt_)
        allk.append(np.stack(ks,1)); allp.append(np.stack(ps,1))
K=np.concatenate(allk); P=np.concatenate(allp)            # (m,3),(m,3,3) in grid units
uk,first,inv=np.unique(K.ravel(),return_index=True,return_inverse=True)
VP=P.reshape(-1,3)[first]*H+org
T=inv.reshape(-1,3)
ok=(T[:,0]!=T[:,1])&(T[:,1]!=T[:,2])&(T[:,0]!=T[:,2]); T=T[ok]
print("proxy: %d vertices, %d triangles"%(len(VP),len(T)),flush=True)
with open(out,"w") as o:
    for p in VP: o.write("v %.6f %.6f %.6f\n"%tuple(p))
    for t in T: o.write("f %d %d %d\n"%(t[0]+1,t[1]+1,t[2]+1))
