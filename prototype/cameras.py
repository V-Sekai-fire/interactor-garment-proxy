# Studio-style camera plan: choose the fewest cameras that look at every marker head-on, like the capture studio of the
# SOMA paper (it reads 82% of its markers over its motions). A candidate camera sits on each marker's outward normal at
# DIST metres and looks at the marker; it covers every marker inside its view cone (cos of the angle to the axis) and
# whose own normal faces it within MAX_TILT degrees. Greedy set cover until all are covered COPIES times (each copy
# from a different camera). Writes a JSON list of {pos, target, up, fov} (Godot frame) for godot/fit_decals.gd.
# python cameras.py <markers.json> <out.json> [dist m] [fov deg] [max tilt deg] [copies]
import sys, json, math, numpy as np
m = json.load(open(sys.argv[1])); out = sys.argv[2]
DIST = float(sys.argv[3]) if len(sys.argv) > 3 else 1.2
FOV = float(sys.argv[4]) if len(sys.argv) > 4 else 40.0
TILT = float(sys.argv[5]) if len(sys.argv) > 5 else 50.0
COPIES = int(sys.argv[6]) if len(sys.argv) > 6 else 2
P = np.array([x["P"] for x in m]); N = np.array([x["n"] for x in m]); n = len(P)
pos = P + N * DIST                               # one candidate camera per marker
half = math.radians(FOV * 0.5) * 0.85             # keep markers away from the frame edge
cosv = math.cos(half); cost = math.cos(math.radians(TILT))
d = P[None, :, :] - pos[:, None, :]               # [camera, marker, 3]: from the camera to the marker
d = d / np.maximum(np.linalg.norm(d, axis=2, keepdims=True), 1e-9)
view = d[np.arange(n), np.arange(n)]              # a camera's own axis points at its own marker
in_cone = np.einsum("ckj,cj->ck", d, view) >= cosv
faces = np.einsum("kj,ckj->ck", N, -d) >= cost    # the marker's normal faces the camera
cover = in_cone & faces
need = np.full(n, COPIES); avail = np.ones(n, bool); chosen = []
while (need > 0).any():
    gain = (cover & (need > 0)[None, :]).sum(1) * avail
    c = int(np.argmax(gain))
    if gain[c] == 0: break
    chosen.append(c); avail[c] = False
    need = need - (cover[c] & (need > 0))
uncovered = int((need > 0).sum())
cams = []
for c in chosen:
    up = np.array([0.0, 1.0, 0.0]); v = view[c]
    if abs(v[1]) > 0.97: up = np.array([1.0, 0.0, 0.0])
    cams.append({"pos": [float(x) for x in pos[c]], "target": [float(x) for x in P[c]], "up": [float(x) for x in up], "fov": FOV})
json.dump(cams, open(out, "w"))
print(f"{n} markers, {len(cams)} cameras (dist {DIST} m, fov {FOV}, tilt <= {TILT} deg, {COPIES} copies); markers short of {COPIES} views: {uncovered}")
