# Prototype (numpy; retire when ported to .sgd): unsigned-distance level surface, extractor with variants.
import numpy as np

def closest(P,a,b,c):
    ab=b-a; ac=c-a; ap=P-a
    d1=ap@ab; d2=ap@ac; bp=P-b; d3=bp@ab; d4=bp@ac; cp=P-c; d5=cp@ab; d6=cp@ac
    vc=d1*d4-d3*d2; vb=d5*d2-d1*d6; va=d3*d6-d5*d4
    den=va+vb+vc; den=np.where(den==0,1e-30,den); v=vb/den; w=vc/den
    out=a+ab*v[:,None]+ac*w[:,None]
    m=(d1<=0)&(d2<=0); out[m]=a
    m=(d3>=0)&(d4<=d3); out[m]=b
    m=(d6>=0)&(d5<=d6); out[m]=c
    m=(vc<=0)&(d1>=0)&(d3<=0); t=d1/np.where(d1-d3==0,1e-30,d1-d3); out[m]=(a+ab*t[:,None])[m]
    m=(vb<=0)&(d2>=0)&(d6<=0); t=d2/np.where(d2-d6==0,1e-30,d2-d6); out[m]=(a+ac*t[:,None])[m]
    m=(va<=0)&((d4-d3)>=0)&((d5-d6)>=0); t=(d4-d3)/np.where((d4-d3)+(d5-d6)==0,1e-30,(d4-d3)+(d5-d6)); out[m]=(b+(c-b)*t[:,None])[m]
    return np.linalg.norm(P-out,axis=1)

def field(V,F,D,H):
    pad=D+3*H; org=V.min(0)-pad; dims=np.ceil((V.max(0)+pad-org)/H).astype(int)+1
    G=np.full(tuple(dims),np.float32(1e9),dtype=np.float32); R=D+1.5*H
    for ia,ib,ic in F:
        a,b,c=V[ia],V[ib],V[ic]
        lo=np.minimum(np.minimum(a,b),c)-R; hi=np.maximum(np.maximum(a,b),c)+R
        i0=np.maximum(np.floor((lo-org)/H).astype(int),0); i1=np.minimum(np.ceil((hi-org)/H).astype(int),dims-1)
        X,Y,Z=np.meshgrid(*[np.arange(i0[k],i1[k]+1) for k in range(3)],indexing="ij")
        P=np.stack([X.ravel(),Y.ravel(),Z.ravel()],1)*H+org
        dist=closest(P,a,b,c).astype(np.float32).reshape(X.shape)
        sl=G[i0[0]:i1[0]+1,i0[1]:i1[1]+1,i0[2]:i1[2]+1]; np.minimum(sl,dist,out=sl)
    return G,org

def extract(f,org,H):
    nx,ny,nz=f.shape; inside=f<0
    s=np.zeros((nx-1,ny-1,nz-1),np.int8)
    for ox in (0,1):
        for oy in (0,1):
            for oz in (0,1): s+=inside[ox:nx-1+ox,oy:ny-1+oy,oz:nz-1+oz]
    act=np.argwhere((s>0)&(s<8))
    corner=np.array([[0,0,0],[1,0,0],[1,1,0],[0,1,0],[0,0,1],[1,0,1],[1,1,1],[0,1,1]])
    tets=[(0,5,1,6),(0,1,2,6),(0,2,3,6),(0,3,7,6),(0,7,4,6),(0,4,5,6)]
    cid=lambda p:(p[...,0]*ny+p[...,1])*nz+p[...,2]
    allk=[];allp=[]
    for tt in tets:
        cp=np.stack([act+corner[k] for k in tt],1); val=f[cp[...,0],cp[...,1],cp[...,2]]; gid=cid(cp); ins=val<0
        for mask in range(1,15):
            bits=[(mask>>k)&1 for k in range(4)]
            sel=np.all(ins==np.array(bits,bool),axis=1)
            if not sel.any(): continue
            I=[k for k in range(4) if bits[k]]; O=[k for k in range(4) if not bits[k]]
            if len(I)==1 or len(O)==1:
                lone,others=(I[0],O) if len(I)==1 else (O[0],I); e=[(lone,o) for o in others]; tl=[(e[0],e[1],e[2])]
            else:
                a,b=I; c,dd=O; tl=[((a,c),(a,dd),(b,dd)),((a,c),(b,dd),(b,c))]
            for t in tl:
                ks=[];ps=[]
                for (p,q) in t:
                    g0=gid[sel][:,p];g1=gid[sel][:,q]
                    ks.append(np.minimum(g0,g1)*np.int64(nx*ny*nz)+np.maximum(g0,g1))
                    v0=val[sel][:,p];v1=val[sel][:,q]; tt_=(v0/(v0-v1))[:,None]
                    ps.append(cp[sel][:,p]*(1-tt_)+cp[sel][:,q]*tt_)
                allk.append(np.stack(ks,1)); allp.append(np.stack(ps,1))
    if not allk: return np.zeros((0,3)),np.zeros((0,3),int)
    K=np.concatenate(allk); P=np.concatenate(allp)
    uk,first,inv=np.unique(K.ravel(),return_index=True,return_inverse=True)
    VP=P.reshape(-1,3)[first]*H+org; T=inv.ravel().reshape(-1,3)
    ok=(T[:,0]!=T[:,1])&(T[:,1]!=T[:,2])&(T[:,0]!=T[:,2])
    return VP,T[ok]

EDGES=[(0,1),(1,2),(2,3),(3,0),(4,5),(5,6),(6,7),(7,4),(0,4),(1,5),(2,6),(3,7)]
CORNER=np.array([[0,0,0],[1,0,0],[1,1,0],[0,1,0],[0,0,1],[1,0,1],[1,1,1],[0,1,1]])

def extract_nets(f,org,H,centre=False):
    nx,ny,nz=f.shape; ins=f<0
    s=np.zeros((nx-1,ny-1,nz-1),np.int8)
    for ox in (0,1):
        for oy in (0,1):
            for oz in (0,1): s+=ins[ox:nx-1+ox,oy:ny-1+oy,oz:nz-1+oz]
    act=np.argwhere((s>0)&(s<8)); n=len(act)
    cidx=-np.ones((nx-1,ny-1,nz-1),np.int64); cidx[tuple(act.T)]=np.arange(n)
    if centre: pos=act+0.5
    else:
        sp=np.zeros((n,3)); cnt=np.zeros(n)
        for p,q in EDGES:
            cp=act+CORNER[p]; cq=act+CORNER[q]
            v0=f[tuple(cp.T)]; v1=f[tuple(cq.T)]; m=(v0<0)!=(v1<0)
            t=np.where(m,v0/np.where(v0==v1,1,v0-v1),0)
            sp[m]+=(cp+(cq-cp)*t[:,None])[m]; cnt[m]+=1
        pos=sp/cnt[:,None]
    VP=pos*H+org
    tris=[]
    off=[(-1,-1),(0,-1),(0,0),(-1,0)]
    for a in range(3):
        b=(a+1)%3; c=(a+2)%3
        sl0=[slice(None)]*3; sl1=[slice(None)]*3
        sl0[a]=slice(0,-1); sl1[a]=slice(1,None)
        A=ins[tuple(sl0)]; B=ins[tuple(sl1)]; m=A!=B
        e=np.argwhere(m)
        e=e[(e[:,b]>=1)&(e[:,c]>=1)&(e[:,b]<=ny-2 if b==1 else True)]
        flip=A[tuple(e.T)]
        q=[]
        for ob,oc in off:
            cell=e.copy(); cell[:,b]+=ob; cell[:,c]+=oc
            q.append(cidx[tuple(cell.T)])
        q=np.stack(q,1); good=(q>=0).all(1); q=q[good]; flip=flip[good]
        T1=q[:,[0,1,2]]; T2=q[:,[0,2,3]]
        T1[flip]=T1[flip][:,::-1]; T2[flip]=T2[flip][:,::-1]
        tris+= [T1,T2]
    return VP,np.vstack(tris)

def run(V,F,D,H,variant):
    G,org=field(V,F,D,H); f=G.astype(np.float64)-D
    if variant=="nets": VP,T=extract_nets(f,org,H)
    elif variant=="nets_centre": VP,T=extract_nets(f,org,H,centre=True)
    else: VP,T=extract(f,org,H)
    if variant=="planted" and len(T)>0: T=T[1:]
    return VP,T
