import sys
import numpy as np
src,out=sys.argv[1],sys.argv[2]
V=[];F=[]
for l in open(src):
    if l.startswith("v "): V.append([float(x) for x in l.split()[1:4]])
    elif l.startswith("f "):
        ids=[int(x.split("/")[0])-1 for x in l.split()[1:]]
        for k in range(1,len(ids)-1): F.append([ids[0],ids[k],ids[k+1]])
V=np.array(V);F=np.array(F)
q=np.round(V/1e-6).astype(np.int64)
_,first,inv=np.unique(q,axis=0,return_index=True,return_inverse=True)
inv=inv.ravel(); W=V[first]; G=inv[F]
ok=(G[:,0]!=G[:,1])&(G[:,1]!=G[:,2])&(G[:,0]!=G[:,2]); G=G[ok]
fn=np.cross(W[G[:,1]]-W[G[:,0]],W[G[:,2]]-W[G[:,0]])
N=np.zeros_like(W)
for k in range(3): np.add.at(N,G[:,k],fn)
L=np.linalg.norm(N,axis=1); L[L==0]=1; N/=L[:,None]
lo=W.min(0); hi=W.max(0)
with open(out,"wb") as f:
    np.array([len(W),len(G)],dtype=np.int32).tofile(f)
    np.hstack([W,N]).astype(np.float32).tofile(f)
    G.astype(np.int32).tofile(f)
print(src.split("/")[-1],"-> welded vertices",len(W),"triangles",len(G),"bounds",lo.round(3).tolist(),hi.round(3).tolist())
