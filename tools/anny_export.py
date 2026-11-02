# Export the baked ANNY-soma-x body (solve/core/soma_rig.h: 18056 vertices, 78 joints, 10 skin weights per vertex) to plain files
# the Godot tools read: <out>/anny_bind.obj (metres, Y up), <out>/anny_skeleton.json (names, parents, rest and inverse-bind
# matrices) and <out>/anny_weights.npz (joint index and weight, 10 per vertex).
# Gates (each must pass, and the vertex-change control must FAIL): counts, face indices in range, weights sum to 1, the rest pose
# times the inverse bind is the identity, linear-blend skinning at the bind pose returns the bind mesh, and a one-vertex change in
# the written OBJ is caught on read-back.
# python anny_export.py <soma_rig.h> <out dir>
import re, sys, json, os, numpy as np
src, out = sys.argv[1], sys.argv[2]; os.makedirs(out, exist_ok=True)
text = open(src).read()
def arr(name, dtype):
    m = re.search(r"static const %s %s\[[^\]]*\]\s*=\s*\{(.*?)\};" % ("float" if dtype == np.float32 else "int", name), text, re.S)
    assert m, name
    return np.array(m.group(1).replace("\n", " ").split(","), dtype=object)[:-1 if m.group(1).strip().endswith(",") else None].astype(dtype) if False else np.array([x for x in re.split(r"[,\s]+", m.group(1).strip()) if x], dtype=dtype)
V = int(re.search(r"#define SOMA_V (\d+)", text).group(1)); J = int(re.search(r"#define SOMA_J (\d+)", text).group(1)); F = int(re.search(r"#define SOMA_F (\d+)", text).group(1)); K = int(re.search(r"#define SOMA_K (\d+)", text).group(1))
bind = arr("soma_bind", np.float32).reshape(V, 3); widx = arr("soma_widx", np.int32).reshape(V, K); wval = arr("soma_wval", np.float32).reshape(V, K)
inv = arr("soma_invbind", np.float32).reshape(J, 4, 4); rest = arr("soma_restWorld", np.float32).reshape(J, 4, 4)
parents = arr("soma_parents", np.int32); faces = arr("soma_faces", np.int32).reshape(F, 3)
names = re.findall(r'"([^"]+)"', re.search(r"soma_jointNames\[SOMA_J\]\s*=\s*\{(.*?)\};", text, re.S).group(1))
ok = True
def gate(name, cond, detail=""):
    global ok; ok &= bool(cond); print(("PASS " if cond else "FAIL ") + name + (" " + detail if detail else ""))
gate("counts", bind.shape == (18056, 3) and faces.shape == (36108, 3) and len(names) == 78 and len(parents) == 78, f"V {V} F {F} J {J} K {K}")
gate("face indices in range", faces.min() >= 0 and faces.max() < V)
gate("joint indices in range", widx.min() >= 0 and widx.max() < J)
gate("weights sum to 1", np.abs(wval.sum(1) - 1).max() < 1e-3, f"max deviation {np.abs(wval.sum(1) - 1).max():.2e}")
roots = [i for i, p in enumerate(parents) if p == i or p < 0]   # this rig marks the root as its own parent
gate("one root, parents precede children", len(roots) == 1 and all(p < i for i, p in enumerate(parents) if p != i and p >= 0), f"root joint {roots[0] if roots else None} ({names[roots[0]] if roots else None})")
# 4x4 layout: try row-major and column-major, keep the one where rest @ inverse bind is the identity
def ident_err(R, I): return max(np.abs(R[j] @ I[j] - np.eye(4)).max() for j in range(J))
e_row = ident_err(rest, inv); e_col = ident_err(rest.transpose(0, 2, 1), inv.transpose(0, 2, 1)); layout = "row-major" if e_row <= e_col else "column-major"
if layout == "column-major": rest, inv = rest.transpose(0, 2, 1).copy(), inv.transpose(0, 2, 1).copy()
gate("rest pose x inverse bind = identity", min(e_row, e_col) < 1e-3, f"{layout}, max error {min(e_row, e_col):.2e}")
# linear-blend skinning at the bind pose (row vectors or column vectors: use the convention that worked above)
M = np.einsum("jab,jbc->jac", rest, inv)                     # per joint skinning matrix at the bind pose
vh = np.concatenate([bind, np.ones((V, 1), np.float32)], 1)
skin = np.zeros((V, 3), np.float64)
for k in range(K): skin += wval[:, k:k + 1] * np.einsum("vab,vb->va", M[widx[:, k]], vh)[:, :3]
gate("bind-pose skinning returns the bind mesh", np.abs(skin - bind).max() < 1e-4, f"max error {np.abs(skin - bind).max():.2e} m")
ext = bind.max(0) - bind.min(0)
print(f"bounding box {ext[0]:.3f} x {ext[1]:.3f} x {ext[2]:.3f} m (height is Y: {ext[1]:.3f}); bind min y {bind[:,1].min():.3f}")
with open(os.path.join(out, "anny_bind.obj"), "w") as f:
    for v in bind: f.write("v %.7f %.7f %.7f\n" % tuple(v))
    for a, b, c in faces: f.write("f %d %d %d\n" % (a + 1, b + 1, c + 1))
json.dump({"names": names, "parents": [-1 if int(p) == i else int(p) for i, p in enumerate(parents)], "root": int(roots[0]), "rest_world": rest.tolist(), "inverse_bind": inv.tolist(), "layout": layout}, open(os.path.join(out, "anny_skeleton.json"), "w"))
np.savez_compressed(os.path.join(out, "anny_weights.npz"), joint=widx, weight=wval)
# read-back gate and the planted one-vertex control
def read_obj(p): return np.array([[float(x) for x in l.split()[1:4]] for l in open(p) if l.startswith("v ")])
rb = read_obj(os.path.join(out, "anny_bind.obj")); gate("OBJ reads back equal", np.abs(rb - bind).max() < 1e-6)
bad = rb.copy(); bad[1234, 1] += 0.01
caught = np.abs(bad - bind).max() > 1e-6
print(("PASS " if caught else "FAIL ") + "control: a one-vertex change (1 cm) is caught"); ok &= caught
sys.exit(0 if ok else 1)
