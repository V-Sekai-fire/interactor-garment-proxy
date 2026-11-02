# Rigid versus soft POLYGONS of a skinned glb (the labels are per triangle: they become UsdGeomSubset faces).
# H = the glb's joints that match a humanoid bone of a rigged VRM of the same mesh; matching is by polygons: each vertex is weighted by its share of triangle area,
# so a densely tessellated patch (hair) cannot outvote a large smooth one, and a joint matches the bone whose weighted skin region it overlaps most (cosine >= 0.5).
# A spring subtree hangs off the body, has no humanoid joint below it, is >= 4 joints and >= 3 deep, and reaches >= 5 cm (hair, skirt, ribbons; face and helper joints fall out).
# A triangle is soft when the mean skin weight of its three vertices on spring joints is >= 0.5.
# python rigid_soft.py <model.glb> <rigged.vrm> <out.npz>
import sys, struct, json, numpy as np
glb, vrm, out = sys.argv[1:4]; MINSZ, MINDEPTH, MINREACH = 4, 3, 0.05
def load(p):
    d = open(p, "rb").read(); n = struct.unpack("<I", d[12:16])[0]; j = json.loads(d[20:20 + n]); o = 20 + n; bl = struct.unpack("<I", d[o:o + 4])[0]; return j, d[o + 8:o + 8 + bl]
def world(j):
    nodes = j["nodes"]; par = {c: i for i, nd in enumerate(nodes) for c in nd.get("children", [])}; W = {}
    def loc(nd):
        if "matrix" in nd: return np.array(nd["matrix"]).reshape(4, 4).T
        t = np.array(nd.get("translation", [0, 0, 0])); x, y, z, w = nd.get("rotation", [0, 0, 0, 1]); s = np.array(nd.get("scale", [1, 1, 1]))
        R = np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)], [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)], [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])
        M = np.eye(4); M[:3, :3] = R * s; M[:3, 3] = t; return M
    def w(i):
        if i not in W: W[i] = (w(par[i]) if i in par else np.eye(4)) @ loc(nodes[i])
        return W[i]
    return lambda i: w(i)[:3, 3]
def acc(j, B, i):
    a = j["accessors"][i]; bv = j["bufferViews"][a["bufferView"]]; n = {"SCALAR": 1, "VEC2": 2, "VEC3": 3, "VEC4": 4}[a["type"]]; dt = {5121: np.uint8, 5123: np.uint16, 5125: np.uint32, 5126: np.float32}[a["componentType"]]
    o = bv.get("byteOffset", 0) + a.get("byteOffset", 0); isz = np.dtype(dt).itemsize; st = bv.get("byteStride") or n * isz
    if st == n * isz: return np.frombuffer(B, dt, count=a["count"] * n, offset=o).reshape(a["count"], n)
    return np.frombuffer(B, np.uint8, count=a["count"] * st, offset=o).reshape(a["count"], st)[:, :n * isz].copy().view(dt).reshape(a["count"], n)
def geometry(j, B):   # positions, triangles (global vertex ids), mesh id per triangle, skin weights (vertices x joints of skin 0)
    nj = len(j["skins"][0]["joints"]); P, T, MT, G = [], [], [], []; off = 0
    for mi, m in enumerate(j["meshes"]):
        for pr in m["primitives"]:
            at = pr["attributes"]; p = acc(j, B, at["POSITION"]).astype(np.float64); n = len(p); P.append(p)
            t = (acc(j, B, pr["indices"]).astype(np.int64).reshape(-1, 3) if "indices" in pr else np.arange(n).reshape(-1, 3)) + off; T.append(t); MT.append(np.full(len(t), mi)); off += n
            M = np.zeros((n, nj), np.float32); k = 0
            while f"JOINTS_{k}" in at:
                J = acc(j, B, at[f"JOINTS_{k}"]).astype(int); W = acc(j, B, at[f"WEIGHTS_{k}"]).astype(np.float64)
                W = W / (255.0 if 1.5 < W.max() <= 255 else 65535.0 if W.max() > 255 else 1.0); np.add.at(M, (np.repeat(np.arange(n), J.shape[1]), J.ravel()), W.ravel().astype(np.float32)); k += 1
            G.append(M)
    return np.vstack(P), np.vstack(T), np.concatenate(MT), np.vstack(G)
jg, Bg = load(glb); jv, Bv = load(vrm); Pg, Tg, MTg, G = geometry(jg, Bg); Pv, Tv, MTv, V = geometry(jv, Bv)
kg, kv = np.sort(Tg, axis=1), np.sort(Tv, axis=1); og, ov = np.lexsort(kg.T[::-1]), np.lexsort(kv.T[::-1])   # a polygon is its vertex set: the two files start each triangle at a different corner
same_topology = Tg.shape == Tv.shape and bool((kg[og] == kv[ov]).all()); print("GATE polygons match one to one by vertex set between the glb and the rigged VRM:", same_topology, f"({len(Tg)} triangles, {len(Pg)} vertices)")
assert same_topology, "the two files do not share polygons; matching by polygons needs the same topology"
cr = np.cross(Pg[Tg[:, 1]] - Pg[Tg[:, 0]], Pg[Tg[:, 2]] - Pg[Tg[:, 0]]); area = 0.5 * np.linalg.norm(cr, axis=1); Av = np.zeros(len(Pg)); np.add.at(Av, Tg.ravel(), np.repeat(area / 3, 3))
joints = jg["skins"][0]["joints"]; jidx = {n: k for k, n in enumerate(joints)}; hb = jv["extensions"]["VRMC_vrm"]["humanoid"]["humanBones"]; vj = jv["skins"][0]["joints"]
name_of = {v["node"]: k for k, v in hb.items()}; vnames = [name_of.get(n, "?") for n in vj]; sw = np.sqrt(Av)[:, None].astype(np.float32)
Gn = (G * sw) / np.maximum(np.linalg.norm(G * sw, axis=0, keepdims=True), 1e-9); Vn = (V * sw) / np.maximum(np.linalg.norm(V * sw, axis=0, keepdims=True), 1e-9)
C = Vn.T @ Gn; H = {}; used = set(); cos = []
for k in np.argsort(-C.max(1)):
    c = C[k].copy(); c[list(used)] = -1; i = int(c.argmax())
    if c[i] >= 0.5: H[vnames[k]] = i; used.add(i); cos.append(float(c[i]))
isH = np.zeros(len(joints), bool); isH[list(H.values())] = True
print(f"humanoid bones matched by area-weighted skin cosine >= 0.5: {len(H)} of {len(vj)} (median {np.median(cos):.2f}, min {min(cos):.2f}); unmatched: {[n for n in vnames if n not in H][:8]}")
nodes = jg["nodes"]; jset = set(joints); kids = {n: [c for c in nodes[n].get("children", []) if c in jset] for n in joints}; parent = {c: n for n in joints for c in kids[n]}; wg = world(jg); hfree = {}
def free(n):
    if n not in hfree: hfree[n] = (not isH[jidx[n]]) and all(free(c) for c in kids[n])
    return hfree[n]
def sub(n): return [n] + [m for c in kids[n] for m in sub(c)]
def depth(n): return 1 + max([depth(c) for c in kids[n]] or [0])
isE = np.zeros(len(joints), bool); sizes = []
for n in joints:
    if free(n) and not (n in parent and free(parent[n])):
        s = sub(n)
        if len(s) >= MINSZ and depth(n) >= MINDEPTH and max(float(np.linalg.norm(wg(m) - wg(n))) for m in s) >= MINREACH:
            for m in s: isE[jidx[m]] = True
            sizes.append(len(s))
print(f"spring joints {int(isE.sum())} in {len(sizes)} subtrees (size median {int(np.median(sizes))}, max {max(sizes)}); other non-humanoid joints (face, twist, helpers): {int((~isH & ~isE).sum())}")
def labels(spring):
    Gs = G / np.maximum(G.sum(1, keepdims=True), 1e-9); e = (Gs * spring).sum(1); return e[Tg].mean(1) >= 0.5
soft = labels(isE); np.savez(out, soft=soft, T=Tg, mesh=MTg, isE=isE, isH=isH, hum_names=np.array(list(H)), hum_joint=np.array(list(H.values())), joint_nodes=np.array(joints))
print("mesh | triangles | soft by count | soft by area"); rows = []
for mi in range(len(jg["meshes"])):
    k = MTg == mi; rows.append(f"m{mi} {int(k.sum())} {100*soft[k].mean():.0f}% {100*area[k][soft[k]].sum()/area[k].sum():.0f}%")
print(" ; ".join(rows)); print(f"all: {len(Tg)} triangles, soft {int(soft.sum())} ({100*soft.mean():.0f}% by count, {100*area[soft].sum()/area.sum():.0f}% by area), rigid {int((~soft).sum())}")
rng = np.random.RandomState(0); alt = isE.copy(); alt[rng.choice(np.flatnonzero(isE), max(int(isE.sum()) // 10, 1), replace=False)] = False
rnd = np.zeros(len(isE), bool); rnd[rng.choice(len(isE), int(isE.sum()), replace=False)] = True
print(f"CONTROL dropping 10% of the spring joints flips {int((labels(alt) != soft).sum())} polygons; a random joint set of the same size flips {int((labels(rnd) != soft).sum())} (both must be > 0)")
