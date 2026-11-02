# Make a VRM 1.0 out of a skinned glb's OWN rig: the binary chunk (every joint and weight) is copied unchanged, only the JSON gains VRMC_vrm and VRMC_springBone.
# The auto-rig step re-predicts weights and drops the spring chains, so it cannot keep them; this keeps them.
# Humanoid map H and spring joints E come from rigid_soft.py (its npz). Meta comes from the glb's asset.extras; nothing is invented.
# Spring parameters and collider radii are chosen constants (SPRING, COLLIDER below), not read from the file.
# python vrm_annotate.py <model.glb> <rigid_soft.npz> <out.vrm>
import sys, struct, json, re, numpy as np
glb, npz, out = sys.argv[1:4]
SPRING = dict(stiffness=1.0, dragForce=0.4, gravityPower=0.1, gravityDir=[0, -1, 0], hitRadius=0.02)
COLLIDER = dict(head=0.10, torso=0.12, upperLeg=0.08)   # metres
d = open(glb, "rb").read(); n = struct.unpack("<I", d[12:16])[0]; j = json.loads(d[20:20 + n]); o = 20 + n; bl = struct.unpack("<I", d[o:o + 4])[0]; BIN = d[o + 8:o + 8 + bl]
z = np.load(npz, allow_pickle=True); joints = [int(x) for x in z["joint_nodes"]]; H = {str(k): joints[int(v)] for k, v in zip(z["hum_names"], z["hum_joint"])}; isE = z["isE"]
nodes = j["nodes"]; par = {c: i for i, nd in enumerate(nodes) for c in nd.get("children", [])}; jset = set(joints)
kids = {n_: [c for c in nodes[n_].get("children", []) if c in jset] for n_ in joints}
def loc(nd):
    if "matrix" in nd: return np.array(nd["matrix"]).reshape(4, 4).T
    t = np.array(nd.get("translation", [0, 0, 0])); x, y, zz, w = nd.get("rotation", [0, 0, 0, 1]); s = np.array(nd.get("scale", [1, 1, 1]))
    R = np.array([[1 - 2 * (y * y + zz * zz), 2 * (x * y - zz * w), 2 * (x * zz + y * w)], [2 * (x * y + zz * w), 1 - 2 * (x * x + zz * zz), 2 * (y * zz - x * w)], [2 * (x * zz - y * w), 2 * (y * zz + x * w), 1 - 2 * (x * x + y * y)]])
    M = np.eye(4); M[:3, :3] = R * s; M[:3, 3] = t; return M
Wc = {}
def W(i):
    if i not in Wc: Wc[i] = (W(par[i]) if i in par else np.eye(4)) @ loc(nodes[i])
    return Wc[i]
def anc(i):
    a = [i]
    while a[-1] in par: a.append(par[a[-1]])
    return a
# --- humanoid map: the matched bones, plus the unmatched required ones from the skeleton's shape
chain = {"leftIndex": 0}
for s in ("left", "right"):
    if s + "LowerLeg" in H and s + "UpperLeg" not in H: H[s + "UpperLeg"] = par[H[s + "LowerLeg"]]   # an upper leg is the parent of its lower leg
if "hips" not in H:   # hips: the nearest common ancestor of both legs, the spine and the head (a rig may hang the legs off a lower-body bone, with the spine beside it)
    sets = [set(anc(H[k])) for k in ("leftUpperLeg", "rightUpperLeg", "spine", "head")]
    H["hips"] = next(a for a in anc(H["leftUpperLeg"]) if all(a in t for t in sets))
for s in ("left", "right"):   # a finger bone needs the one before it in its chain
    for f in ("Thumb", "Index", "Middle", "Ring", "Little"):
        names = [s + f + k for k in (("Metacarpal", "Proximal", "Distal") if f == "Thumb" else ("Proximal", "Intermediate", "Distal"))]
        for a, b in zip(names, names[1:]):
            if b in H and a not in H: del H[b]
H.pop("upperChest", None)   # optional in VRM 1.0
REQ = "hips spine head leftUpperLeg leftLowerLeg leftFoot rightUpperLeg rightLowerLeg rightFoot leftUpperArm leftLowerArm leftHand rightUpperArm rightLowerArm rightHand".split()
missing = [r for r in REQ if r not in H]; assert not missing, f"required humanoid bones without a joint: {missing}"
assert len(set(H.values())) == len(H), "two humanoid bones on one node"
# --- springs: each E joint belongs to exactly one spring; a spring is a chain (VRM springs do not branch), so a subtree is cut into heavy paths
Ej = {joints[i] for i in np.flatnonzero(isE)}
def size(n_): return 1 + sum(size(c) for c in kids[n_] if c in Ej)
def paths(n_, out_):
    p = [n_]
    while True:
        k = sorted((c for c in kids[p[-1]] if c in Ej), key=lambda c: -size(c))
        for c in k[1:]: paths(c, out_)
        if not k: break
        p.append(k[0])
    out_.append(p)
roots = [n_ for n_ in Ej if par.get(n_) not in Ej]; springs = []
for r in sorted(roots): paths(r, springs)
def check(sp):
    flat = [n_ for p in sp for n_ in p]
    return len(flat) == len(set(flat)) and set(flat) == Ej and all(par[b] == a for p in sp for a, b in zip(p, p[1:]))
assert check(springs), "springs do not cover the spring joints exactly once"
# --- colliders on rigid parts: a head sphere, a torso capsule hips->head, upper-leg capsules to the knee; offsets are in the node's own space
def inv_pt(node, p): return (np.linalg.inv(W(node)) @ np.append(p, 1))[:3].tolist()
def pos(i): return W(i)[:3, 3]
cols = []
def add(node, shape): cols.append({"node": node, **shape})
def cap(node, a, b, r): add(node, {"shape": {"capsule": {"offset": inv_pt(node, a), "radius": r, "tail": inv_pt(node, b)}}})
add(H["head"], {"shape": {"sphere": {"offset": inv_pt(H["head"], pos(H["head"])), "radius": COLLIDER["head"]}}})
cap(H["hips"], pos(H["hips"]), pos(H["spine"]) if "chest" not in H else pos(H["chest"]), COLLIDER["torso"])
for s in ("left", "right"): cap(H[s + "UpperLeg"], pos(H[s + "UpperLeg"]), pos(H[s + "LowerLeg"]), COLLIDER["upperLeg"])
# --- meta from the file
ex = j.get("asset", {}).get("extras", {}); sp_ = lambda t: re.match(r"^(.*?)\s*\((https?://[^)]*)\)\s*$", str(t))
assert "author" in ex and "license" in ex, "asset.extras has no author or licence"
meta = {"name": ex.get("title", "model"), "version": "1.0", "authors": [sp_(ex["author"]).group(1) if sp_(ex["author"]) else ex["author"]], "licenseUrl": sp_(ex["license"]).group(2), "copyrightInformation": f"{ex['author']}, {ex['license']}", "references": [ex["source"]] if "source" in ex else []}
vrm = {"specVersion": "1.0", "meta": meta, "humanoid": {"humanBones": {k: {"node": v} for k, v in H.items()}}}
sb = {"specVersion": "1.0", "colliders": [{"node": c["node"], "shape": c["shape"]} for c in cols], "colliderGroups": [{"name": "body", "colliders": list(range(len(cols)))}],
      "springs": [{"name": f"spring{i}", "joints": [{"node": n_, **{k: SPRING[k] for k in ("hitRadius", "stiffness", "gravityPower", "gravityDir", "dragForce")}} for n_ in p], "colliderGroups": [0]} for i, p in enumerate(springs)]}
for m in j.get("materials", []): m.setdefault("extensions", {})["VRMC_materials_mtoon"] = {"specVersion": "1.0"}   # VRM players expect MToon; its defaults, the pbr colours stay
j.setdefault("extensions", {}).update({"VRMC_vrm": vrm, "VRMC_springBone": sb}); j["extensionsUsed"] = sorted(set(j.get("extensionsUsed", [])) | {"VRMC_vrm", "VRMC_springBone", "VRMC_materials_mtoon"})
js = json.dumps(j, separators=(",", ":")).encode(); js += b" " * (-len(js) % 4)
body = struct.pack("<I4s", len(js), b"JSON") + js + struct.pack("<I4s", len(BIN), b"BIN\0") + BIN
open(out, "wb").write(struct.pack("<4sII", b"glTF", 2, 12 + len(body)) + body)
# --- gates
r = open(out, "rb").read(); n2 = struct.unpack("<I", r[12:16])[0]; j2 = json.loads(r[20:20 + n2]); o2 = 20 + n2; bl2 = struct.unpack("<I", r[o2:o2 + 4])[0]
print(f"GATE binary chunk (all weights and joints) identical: {r[o2 + 8:o2 + 8 + bl2] == BIN}")
print(f"GATE skin joints in {len(joints)} = out {len(j2['skins'][0]['joints'])}: {j2['skins'][0]['joints'] == j['skins'][0]['joints']}")
print(f"GATE every spring joint in exactly one spring, chained: {check(springs)}  ({len(Ej)} joints, {len(roots)} subtrees, {len(springs)} springs)")
print(f"GATE meta names {meta['authors']} and {meta['licenseUrl']}")
print(f"humanoid bones {len(H)} (matched by polygons {len(z['hum_names'])}, added from the skeleton: {[k for k in H if k not in set(map(str, z['hum_names']))]})")
print("CONTROL a spring chain removed fails the coverage check:", not check(springs[1:]))
print("CONTROL a broken chain (two joints swapped) fails:", not check([next(p for p in springs if len(p) > 1)[::-1]] + [p for p in springs if p is not next(q for q in springs if len(q) > 1)]))
