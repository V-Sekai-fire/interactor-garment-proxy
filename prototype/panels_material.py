# Label every triangle with a panel id: panels are the connected pieces of the surface that stay after cutting along long
# crease chains (dihedral over THR degrees, chains of at least MINLEN). Feed the file to xatlas_cli as materials=<file>:
# xatlas never makes a chart across two materials, so every chart border it makes must follow a panel border (a crease).
# Tiny panels (under MINTRI triangles) are merged into the neighbour they share the most edges with.
# python panels_material.py <in.obj> <out.txt> [THR=40] [MINLEN in file units=0.6] [MINTRI=60]
import sys, math, numpy as np
src, out = sys.argv[1], sys.argv[2]
THR = float(sys.argv[3]) if len(sys.argv) > 3 else 40.0; MINLEN = float(sys.argv[4]) if len(sys.argv) > 4 else 0.6; MINTRI = int(sys.argv[5]) if len(sys.argv) > 5 else 60
V, F = [], []
for l in open(src):
    t = l.split()
    if t and t[0] == "v": V.append([float(x) for x in t[1:4]])
    elif t and t[0] == "f": F.append([int(x.split("/")[0]) - 1 for x in t[1:4]])
V = np.array(V); F = np.array(F)
N = np.cross(V[F[:, 1]] - V[F[:, 0]], V[F[:, 2]] - V[F[:, 0]]); N /= np.maximum(np.linalg.norm(N, axis=1, keepdims=True), 1e-20)
edges = {}
for t, f in enumerate(F):
    for k in range(3):
        a, b = int(f[k]), int(f[(k + 1) % 3]); edges.setdefault((min(a, b), max(a, b)), []).append(t)
cosT = math.cos(math.radians(THR))
cr = [k for k, ts in edges.items() if len(ts) == 2 and np.dot(N[ts[0]], N[ts[1]]) < cosT]
vp = {}
def vf(x):
    while vp.setdefault(x, x) != x: vp[x] = vp[vp[x]]; x = vp[x]
    return x
for a, b in cr: vp[vf(a)] = vf(b)
chain = {}
for a, b in cr: chain[vf(a)] = chain.get(vf(a), 0.0) + float(np.linalg.norm(V[a] - V[b]))
keep = {k for k in cr if chain[vf(k[0])] >= MINLEN}
par = list(range(len(F)))
def find(x):
    while par[x] != x: par[x] = par[par[x]]; x = par[x]
    return x
for k, ts in edges.items():
    if len(ts) == 2 and k not in keep: par[find(ts[0])] = find(ts[1])
lab = np.array([find(t) for t in range(len(F))])
for _ in range(3):                                   # merge tiny panels into the neighbour with the most shared edges
    ids, cnt = np.unique(lab, return_counts=True); small = {int(i) for i, c in zip(ids, cnt) if c < MINTRI}
    if not small: break
    for i in small:
        share = {}
        for k, ts in edges.items():
            if len(ts) == 2 and (lab[ts[0]] == i) != (lab[ts[1]] == i):
                o = lab[ts[1]] if lab[ts[0]] == i else lab[ts[0]]; share[int(o)] = share.get(int(o), 0) + 1
        if share: lab[lab == i] = max(share, key=share.get)
ids, inv = np.unique(lab, return_inverse=True)
open(out, "w").write("\n".join(str(int(x)) for x in inv) + "\n")
sizes = np.bincount(inv)
print(f"{src.split('/')[-1]}: {len(keep)} crease edges kept of {len(cr)} (chains of at least {MINLEN} units), {len(ids)} panels; largest {sizes.max()} triangles, smallest {sizes.min()}")
