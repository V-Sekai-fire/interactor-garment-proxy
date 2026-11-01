import sys, colorsys
import numpy as np
def load(p):
    V=[];F=[]
    for l in open(p):
        if l.startswith("v "): V.append([float(x) for x in l.split()[1:4]])
        elif l.startswith("f "):
            ids=[int(x.split("/")[0])-1 for x in l.split()[1:]]
            for k in range(1,len(ids)-1): F.append([ids[0],ids[k],ids[k+1]])
    return np.array(V),np.array(F)
def labels(F,n):
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
    lab=np.array([find(i) for i in range(n)]); return lab
def weld(V,F):
    k=np.round(V/1e-6).astype(np.int64); _,first,inv=np.unique(k,axis=0,return_index=True,return_inverse=True); inv=inv.ravel()
    G=inv[F]; ok=(G[:,0]!=G[:,1])&(G[:,1]!=G[:,2])&(G[:,0]!=G[:,2]); return V[first],G[ok],inv
def write(path,W,G,col):
    fn=np.cross(W[G[:,1]]-W[G[:,0]],W[G[:,2]]-W[G[:,0]]); N=np.zeros_like(W)
    for k in range(3): np.add.at(N,G[:,k],fn)
    L=np.linalg.norm(N,axis=1); L[L==0]=1; N/=L[:,None]
    with open(path,"wb") as f:
        np.array([len(W),len(G)],dtype=np.int32).tofile(f)
        np.hstack([W,N,col]).astype(np.float32).tofile(f)
        G.astype(np.int32).tofile(f)
def shell_colors(G,n):
    lab=labels(G,n); used=np.unique(G); u,cnt=np.unique(lab[used],return_counts=True)
    main=u[np.argmax(cnt)]; col=np.zeros((n,3))+0.62
    rng=np.random.default_rng(3)
    for i,r in enumerate(u):
        if r==main: continue
        h=(i*0.61803)%1.0; col[lab==r]=colorsys.hsv_to_rgb(h,0.85,1.0)
    return col,len(u),float(cnt.max())/cnt.sum()
def markers(pts,r):
    oct_v=np.array([[1,0,0],[-1,0,0],[0,1,0],[0,-1,0],[0,0,1],[0,0,-1]],float)*r
    oct_f=np.array([[0,2,4],[2,1,4],[1,3,4],[3,0,4],[2,0,5],[1,2,5],[3,1,5],[0,3,5]])
    V=(pts[:,None,:]+oct_v[None]).reshape(-1,3); F=(oct_f[None]+(np.arange(len(pts))*6)[:,None,None]).reshape(-1,3)
    return V,F
out=sys.argv[1]
gV,gF=load("export/garment_APants.obj"); gW,gG,_=weld(gV,gF)
c,n,fr=shell_colors(gG,len(gW)); write(out+"/g_col.bin",gW,gG,c); print("garment shells",n,"main share %.3f"%fr)
pV,pF=load("proxy/APants_d2mm.obj")
a=0.5*np.linalg.norm(np.cross(pV[pF[:,1]]-pV[pF[:,0]],pV[pF[:,2]]-pV[pF[:,0]]),axis=1); zero=a<1e-12
zc=(pV[pF[zero,0]]+pV[pF[zero,1]]+pV[pF[zero,2]])/3
k=np.round(pV/1e-6).astype(np.int64); _,inv,cnt=np.unique(k,axis=0,return_inverse=True,return_counts=True); inv=inv.ravel()
sv=pV[cnt[inv]>1]
pW,pG,_=weld(pV,pF)
c,n,fr=shell_colors(pG,len(pW)); write(out+"/p_col.bin",pW,pG,c); print("proxy shells",n,"main share %.4f"%fr)
mp=np.vstack([zc,sv]); mV,mF=markers(mp,0.007)
write(out+"/defects.bin",mV,mF,np.tile([1.0,0.0,0.0],(len(mV),1)))
print("zero-area triangles",int(zero.sum()),"shared-position vertices",len(sv),"markers",len(mp))
