# Put the seams where the surface creases. Duplicates the vertices along every crease edge (the two triangles of the edge
# meet at an angle sharper than THR degrees), so the crease is an open boundary and xatlas cannot make a chart across it:
# every chart border has to run along a crease (xatlas may still add cuts inside a panel where a chart would stretch).
# Noise edges are dropped: only crease chains (crease edges that touch end to end) at least MINLEN long are cut.
# python crease_split.py <in.obj> <out.obj> [THR degrees = 40] [MINLEN in the file's units = 0]   -> out.obj (same units), out.obj.orig.json (original vertex per new vertex)
import sys, json, math, numpy as np
src, out = sys.argv[1], sys.argv[2]; THR = float(sys.argv[3]) if len(sys.argv) > 3 else 40.0; MINLEN = float(sys.argv[4]) if len(sys.argv) > 4 else 0.0
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
cosT = math.cos(math.radians(THR)); crease = 0; interior = 0
cr = [(k, ts) for k, ts in edges.items() if len(ts) == 2 and np.dot(N[ts[0]], N[ts[1]]) < cosT]
vp = {}
def vf(x):
    while vp.setdefault(x, x) != x: vp[x] = vp[vp[x]]; x = vp[x]
    return x
for (a_, b_), ts in cr: vp[vf(a_)] = vf(b_)
chain = {}
for (a_, b_), ts in cr: chain[vf(a_)] = chain.get(vf(a_), 0.0) + float(np.linalg.norm(V[a_] - V[b_]))
keep = {k for k, ts in cr if chain[vf(k[0])] >= MINLEN}
print(f"crease chains: {len(chain)}; kept {sum(1 for c in chain.values() if c >= MINLEN)} of at least {MINLEN:.2f} units ({len(keep)} of {len(cr)} crease edges)")
parent = np.arange(len(F) * 3)
def find(a):
    while parent[a] != a: parent[a] = parent[parent[a]]; a = parent[a]
    return a
def slot(t, v): return t * 3 + [int(x) for x in F[t]].index(v)
for (a, b), ts in edges.items():
    if len(ts) != 2: continue
    interior += 1
    if (a, b) in keep: crease += 1; continue
    for v in (a, b):
        ra, rb = find(slot(ts[0], v)), find(slot(ts[1], v))
        if ra != rb: parent[rb] = ra
newid = {}; orig = []; Fn = np.zeros_like(F)
for t, f in enumerate(F):
    for k in range(3):
        key = (int(f[k]), find(t * 3 + k))
        if key not in newid: newid[key] = len(orig); orig.append(int(f[k]))
        Fn[t, k] = newid[key]
# xatlas welds vertices that share a position, so a duplicate would be merged straight back: move every duplicated vertex
# a hair (NUDGE units, 0.02 mm at 0.2 m/unit) along its own panel's normal so the crease stays an open boundary
NUDGE = 1e-4
Vn = V[orig].copy(); acc = np.zeros_like(Vn)
for t in range(len(F)):
    for k in range(3): acc[Fn[t, k]] += N[t]
count = np.bincount(orig, minlength=len(V)); dup = count[orig] > 1
acc /= np.maximum(np.linalg.norm(acc, axis=1, keepdims=True), 1e-20); Vn[dup] += acc[dup] * NUDGE
with open(out, "w") as fh:
    for o in Vn: fh.write("v %.9f %.9f %.9f\n" % tuple(o))
    for a, b, c in Fn: fh.write("f %d %d %d\n" % (a + 1, b + 1, c + 1))
json.dump(orig, open(out + ".orig.json", "w"))
print(f"{src.split('/')[-1]}: {len(F)} triangles, {interior} interior edges, {crease} crease edges (> {THR:.0f} deg) = {100.0 * crease / max(interior, 1):.1f}%, vertices {len(V)} -> {len(orig)}")
