import sys
import numpy as np
V=[];F=[]
for l in open(sys.argv[1]):
    if l.startswith("v "): V.append([float(x) for x in l.split()[1:4]])
    else: F.append([int(x)-1 for x in l.split()[1:4]])
V=np.array(V);F=np.array(F)
q=np.round(V/1e-6).astype(np.int64)
_,inv,cnt=np.unique(q,axis=0,return_inverse=True,return_counts=True)
print("vertices",len(V),"distinct at 1e-6 m:",len(cnt),"-> vertices sharing a position:",int((cnt[inv]>1).sum()))
a=0.5*np.linalg.norm(np.cross(V[F[:,1]]-V[F[:,0]],V[F[:,2]]-V[F[:,0]]),axis=1)
z=a<1e-12
print("triangles",len(F),"zero-area (<1e-12 m2):",int(z.sum()))
Fq=inv.ravel()[F]
coll=(Fq[:,0]==Fq[:,1])|(Fq[:,1]==Fq[:,2])|(Fq[:,0]==Fq[:,2])
print("zero-area triangles with two corners at one position:",int((z&coll).sum()),"of",int(z.sum()))
Q=np.round(np.vstack([V[:5],V[:1]])/1e-6).astype(np.int64)
print("control: planted duplicate position ->",int((np.unique(Q,axis=0,return_counts=True)[1]>1).sum()),"(must be 1)")
