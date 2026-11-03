# The 23 body frames and the 17 MuscleMimic sites of a humanoid VRM, as rotation-star frames.
# Frame = origin at the joint in the rest pose, X along the bone to its child (the last joint of a chain keeps its parent's X),
# Z toward the body's forward (the horizontal direction the toes point, from the VRM's own toes and feet), Y = Z x X.
# Each frame is a 3x3 rotation and its 7-point star (origin plus the six unit-axis ends); the star goes through the
# rotation-star solver (rs_star_kabsch) and must give the same rotation back.
# python body_frames.py <humanoid.vrm> <out.json>
import ctypes, json, os, struct, subprocess, sys, tempfile
import numpy as np
vrm, out = sys.argv[1:3]
HERE = os.path.dirname(os.path.abspath(__file__))
RS = os.path.normpath(os.path.join(HERE, "../../../adapters/third-party/solve/core/rotation-star/c/rotation_star.c"))
lib_path = os.path.join(tempfile.mkdtemp(), "librs.dylib" if sys.platform == "darwin" else "librs.so")
subprocess.run(["cc", "-O2", "-shared", "-fPIC", "-o", lib_path, RS, "-lm"], check=True)
L = ctypes.CDLL(lib_path)
D = lambda a: np.ascontiguousarray(a, np.float64).ctypes.data_as(ctypes.POINTER(ctypes.c_double))
def star(R, o):
    p = np.zeros(21); L.rs_star(D(R.reshape(9)), D(o), D(p)); return p
def kabsch(tgt, src):
    r = np.zeros(9); L.rs_star_kabsch(D(tgt), D(src), D(r)); return r.reshape(3, 3)
d = open(vrm, "rb").read(); n = struct.unpack("<I", d[12:16])[0]; j = json.loads(d[20:20 + n]); nodes = j["nodes"]
hb = {k: v["node"] for k, v in j["extensions"]["VRMC_vrm"]["humanoid"]["humanBones"].items()}
par = {c: i for i, nd in enumerate(nodes) for c in nd.get("children", [])}
def loc(nd):
    if "matrix" in nd: return np.array(nd["matrix"]).reshape(4, 4).T
    t = np.array(nd.get("translation", [0, 0, 0])); x, y, z, w = nd.get("rotation", [0, 0, 0, 1]); s = np.array(nd.get("scale", [1, 1, 1]))
    R = np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)], [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)], [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])
    M = np.eye(4); M[:3, :3] = R * s; M[:3, 3] = t; return M
cache = {}
def W(i):
    if i not in cache: cache[i] = (W(par[i]) if i in par else np.eye(4)) @ loc(nodes[i])
    return cache[i]
P = lambda role: W(hb[role])[:3, 3]
# SOMA30 body joint -> VRM role (None: halfway between its neighbours, the VRM has no such bone), child in the chain
SOMA = [("Hips", "hips", "Spine1"), ("Spine1", "spine", "Spine2"), ("Spine2", None, "Chest"), ("Chest", "chest", "Neck1"), ("Neck1", "neck", "Neck2"),
        ("Neck2", None, "Head"), ("Head", "head", None)]
for s_, r_ in (("Left", "left"), ("Right", "right")):
    SOMA += [(s_ + "Shoulder", r_ + "Shoulder", s_ + "Arm"), (s_ + "Arm", r_ + "UpperArm", s_ + "ForeArm"), (s_ + "ForeArm", r_ + "LowerArm", s_ + "Hand"), (s_ + "Hand", r_ + "Hand", None),
             (s_ + "Leg", r_ + "UpperLeg", s_ + "Shin"), (s_ + "Shin", r_ + "LowerLeg", s_ + "Foot"), (s_ + "Foot", r_ + "Foot", s_ + "ToeBase"), (s_ + "ToeBase", r_ + "Toes", None)]
assert len(SOMA) == 23
pos = {}
for name, role, _ in SOMA:
    if role: pos[name] = P(role)
for name, role, child in SOMA:   # a joint with no VRM bone sits halfway between the joints on either side of it
    if role is None:
        prev = [n for n, _, c in SOMA if c == name][0]; pos[name] = 0.5 * (pos[prev] + pos[child])
forward = np.zeros(3)
for s_ in ("left", "right"): forward += P(s_ + "Toes") - P(s_ + "Foot")
forward[1] = 0.0; forward /= np.linalg.norm(forward)
frames = {}; last_x = {}
for name, role, child in SOMA:
    x = (pos[child] - pos[name]) if child else last_x[[n for n, _, c in SOMA if c == name][0]]
    x = x / np.linalg.norm(x); last_x[name] = x
    z = forward - x * (x @ forward)
    if np.linalg.norm(z) < 1e-3: z = np.array([0.0, 0.0, 1.0]) - x * x[2]     # a bone along the facing direction: fall back to the VRM +Z
    z /= np.linalg.norm(z); y = np.cross(z, x); R = np.stack([x, y, z], 1)
    frames[name] = (R, pos[name])
# --- gates
errs = {"orthonormal": 0.0, "det": 0.0, "star_roundtrip_deg": 0.0, "x_to_child": 0.0}
for name, role, child in SOMA:
    R, p = frames[name]
    errs["orthonormal"] = max(errs["orthonormal"], float(np.abs(R.T @ R - np.eye(3)).max())); errs["det"] = max(errs["det"], abs(float(np.linalg.det(R)) - 1.0))
    back = kabsch(star(R, p), star(np.eye(3), np.zeros(3)))   # the solver turns the identity star into the frame's star
    errs["star_roundtrip_deg"] = max(errs["star_roundtrip_deg"], float(np.degrees(np.arccos(np.clip((np.trace(R.T @ back) - 1) / 2, -1, 1)))))
    if child: errs["x_to_child"] = max(errs["x_to_child"], float(np.linalg.norm(R[:, 0] - (pos[child] - p) / np.linalg.norm(pos[child] - p))))
print("GATE frames", len(frames), "== 23:", len(frames) == 23)
print("GATE rotations: orthonormal err %.1e, det err %.1e, star solver round trip %.2e deg, x to child %.1e" % (errs["orthonormal"], errs["det"], errs["star_roundtrip_deg"], errs["x_to_child"]))
SIDE = ["Shoulder", "Arm", "ForeArm", "Hand", "Leg", "Shin", "Foot", "ToeBase"]
up = np.array([0.0, 1.0, 0.0]); lateral = np.cross(up, forward)   # the character's left (VRM: forward +Z, left +X)
def side_ok(names):   # left joints on the left of the hips, right joints on the right
    c = pos["Hips"] @ lateral; return all((pos["Left" + n] @ lateral - c) > 0 > (pos["Right" + n] @ lateral - c) for n in names)
def mirror_err(names):   # the left joint is the right one reflected across the body's middle plane
    c = pos["Hips"] @ lateral; return max(float(abs((pos["Left" + n] @ lateral - c) + (pos["Right" + n] @ lateral - c))) + float(abs(pos["Left" + n][1] - pos["Right" + n][1])) for n in names)
ok = side_ok(SIDE); e = mirror_err(SIDE)
print("GATE left joints are left, right joints right: %s; left mirrors right: max error %.4f m (tolerance 0.01)" % (ok, e)); assert ok and e < 0.01, "the two sides do not mirror: a joint is mislabelled"
saved = dict(pos); pos["LeftArm"], pos["RightArm"] = saved["RightArm"], saved["LeftArm"]
print("CONTROL left and right upper arms swapped: sides test fails as it must: %s" % (not side_ok(SIDE))); pos.update(saved)
SITES = {"pelvis": "Hips", "spine1": "Spine1", "head": "Head"}
for s_, r_ in (("left", "Left"), ("right", "Right")):
    SITES.update({s_ + "_hip": r_ + "Leg", s_ + "_knee": r_ + "Shin", s_ + "_ankle": r_ + "Foot", s_ + "_foot": r_ + "ToeBase", s_ + "_wrist": r_ + "Hand", s_ + "_shoulder": r_ + "Arm", s_ + "_elbow": r_ + "ForeArm"})
assert len(SITES) == 17
height = float(max(P("head")[1], pos["Head"][1]) - min(P("leftFoot")[1], P("rightFoot")[1]))
json.dump({"source": os.path.basename(vrm), "height_hint_m": height, "forward": forward.tolist(),
           "frames": {k: {"position": v[1].tolist(), "R": v[0].tolist(), "star": star(v[0], v[1]).tolist()} for k, v in frames.items()},
           "sites": {k: {"joint": v, "position": frames[v][1].tolist()} for k, v in SITES.items()}}, open(out, "w"), indent=1)
print("17 sites written to", out)
