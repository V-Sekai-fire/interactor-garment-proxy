# Paint the markers into the texture of UVs we already have, so that no marker is clipped by a seam: for every UV island find the grid
# offset (the grid is the suit's: squares S px wide, a marker with its quiet zone in every second square) that fits the most markers whose
# whole square lies inside the island, paint the checker and the markers into a new atlas texture, and write the ground truth in atlas pixels.
# The UVs, and so the surface size of every square, are untouched (S = 45 px is 30 mm * MARKER_SCALE at the atlas's texel density).
# Gates (pass-fail, printed first): every marker square lies inside one island; markers do not overlap; ids unique; the clean atlas reads
# back every marker with its id. Measures (markers, share of island area covered) follow.
# python texture_markers.py <textured.obj with vt normalised to BWH = 2205 x 2925 (decals.py --uvmap output)> <out dir> [dictionary name = suit_dictionary.txt] [margin px = 3] [supersample = 2]
import os, sys, json, math, numpy as np, cv2, cv2.aruco as aruco
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__))); import tagfamily
src, out = sys.argv[1], sys.argv[2]; os.makedirs(out, exist_ok=True)
DICT = sys.argv[3] if len(sys.argv) > 3 else open("suit_dictionary.txt").read().strip()
MARGIN = int(sys.argv[4]) if len(sys.argv) > 4 else 3; RS = int(sys.argv[5]) if len(sys.argv) > 5 else 2
S, MK = 45, 33; INSET = (S - MK) // 2; BWH = np.array([49 * 45, 65 * 45], float)
V, VT, F = [], [], []
for l in open(src):
    t = l.split()
    if not t: continue
    if t[0] == "v": V.append([float(x) for x in t[1:4]])
    elif t[0] == "vt": VT.append([float(x) for x in t[1:3]])
    elif t[0] == "f": F.append([int(x.split("/")[0]) - 1 for x in t[1:4]])
V = np.array(V); F = np.array(F); uv = np.array(VT) * BWH
W, H = int(math.ceil(uv[:, 0].max())) + 2, int(math.ceil(uv[:, 1].max())) + 2
parent = list(range(len(V)))
def find(x):
    while parent[x] != x: parent[x] = parent[parent[x]]; x = parent[x]
    return x
for f in F: parent[find(f[1])] = find(f[0]); parent[find(f[2])] = find(f[0])
roots = {}; chart_of = np.array([roots.setdefault(find(f[0]), len(roots)) for f in F]); NCH = len(roots)
lab = np.full((H, W), -1, np.int32)
tri = np.full((H, W), -1, np.int32)
for t, f in enumerate(F):
    cv2.fillConvexPoly(lab, np.round(uv[f]).astype(np.int32), int(chart_of[t])); cv2.fillConvexPoly(tri, np.round(uv[f]).astype(np.int32), t)
d = tagfamily.get(DICT); NID = len(d.bytesList); SHIFT = int(os.environ.get("ID_SHIFT", "0")); DROP = set(json.load(open(os.environ["DROP_IDS"]))) if os.environ.get("DROP_IDS") else set()   # DROP_IDS: json list of ids that failed the dead-on gate   # ID_SHIFT moves every id to another position (a test that tells pattern faults from position faults)
canvas = np.full((H * RS, W * RS), 70, np.uint8); markers = []; dropped = []; area_total = int((lab >= 0).sum())
for c in range(NCH):
    ys, xs = np.nonzero(lab == c)
    if len(xs) < S * S: continue
    x0, x1, y0, y1 = xs.min(), xs.max() + 1, ys.min(), ys.max() + 1
    m = (lab[y0:y1, x0:x1] == c).astype(np.uint8)
    me = cv2.erode(m, np.ones((2 * MARGIN + 1, 2 * MARGIN + 1), np.uint8)); ii = cv2.integral(me)
    best = (-1, 0, 0, None)
    for dy in range(0, 2 * S, 5):
        for dx in range(0, 2 * S, 5):
            gx = np.arange(-dx, x1 - x0 + S, S) + dx - (dx // S) * S * 0   # cell origins in chart coordinates
            ox = np.arange(dx - 2 * S, m.shape[1], S); oy = np.arange(dy - 2 * S, m.shape[0], S)
            I, J = np.meshgrid(np.arange(len(ox)), np.arange(len(oy)))
            X = ox[I]; Y = oy[J]; par = ((I + J) % 2 == 0)
            inb = (X >= 0) & (Y >= 0) & (X + S <= m.shape[1]) & (Y + S <= m.shape[0]) & par
            Xi, Yi = X[inb], Y[inb]
            if len(Xi) == 0: continue
            tot = ii[Yi + S, Xi + S] - ii[Yi, Xi + S] - ii[Yi + S, Xi] + ii[Yi, Xi]
            n = int((tot == S * S).sum())
            if n > best[0]: best = (n, dx, dy, (Xi[tot == S * S], Yi[tot == S * S]))
    n, dx, dy, cells = best
    # paint the island: checker by cell parity, as on the real board: white (255) and black (0), no grey
    yy, xx = np.mgrid[y0:y1, x0:x1]; i = (xx - x0 - dx + 2 * S) // S; j = (yy - y0 - dy + 2 * S) // S
    col = np.where((i + j) % 2 == 0, 255, 0).astype(np.uint8); mask = (lab[y0:y1, x0:x1] == c)
    big = np.kron(col, np.ones((RS, RS), np.uint8)); bm = np.kron(mask.astype(np.uint8), np.ones((RS, RS), np.uint8)).astype(bool)
    sub = canvas[y0 * RS:y1 * RS, x0 * RS:x1 * RS]; sub[bm] = big[bm]
    if cells is not None:
        for X, Y in zip(*cells):
            k = len(markers) + len(dropped)   # a dropped marker keeps its id as a hole, so every other marker stays as it was
            if k >= NID: break
            if (k + SHIFT) % NID in DROP:
                dropped.append(k); continue
            px, py = x0 + int(X), y0 + int(Y)
            canvas[py * RS:(py + S) * RS, px * RS:(px + S) * RS] = 255
            canvas[(py + INSET) * RS:(py + INSET + MK) * RS, (px + INSET) * RS:(px + INSET + MK) * RS] = aruco.generateImageMarker(d, (k + SHIFT) % NID, MK * RS, borderBits=1)
            qx, qy = px + INSET, py + INSET
            markers.append({"idx": k, "id": (k + SHIFT) % NID, "chart": int(c), "square": [int(px), int(py)], "quad": [[int(qx), int(qy)], [int(qx) + MK, int(qy)], [int(qx) + MK, int(qy) + MK], [int(qx), int(qy) + MK]]})
# ---- gates ----
# real size: the squares are S px, so the surface must be unwrapped at S px per (30 mm * MARKER_SCALE); xatlas shrinks any island bigger than a fixed sheet, so check it
MS = float(os.environ.get("MARKER_SCALE", "1.0")); target = 1500.0 / MS
a3 = 0.5 * np.linalg.norm(np.cross(V[F[:, 1]] - V[F[:, 0]], V[F[:, 2]] - V[F[:, 0]]), axis=1)
e1 = uv[F[:, 1]] - uv[F[:, 0]]; e2 = uv[F[:, 2]] - uv[F[:, 0]]; a2 = 0.5 * np.abs(e1[:, 0] * e2[:, 1] - e1[:, 1] * e2[:, 0])
okt = a3 > 1e-12; dens = np.sqrt(a2[okt] / a3[okt]); wts = a3[okt]
within = float(wts[np.abs(dens / target - 1) < 0.15].sum() / wts.sum()); med = float(np.median(dens))

fail = 0
def gate(name, ok, detail=""):
    global fail
    print(("PASS " if ok else "FAIL ") + name + (" " + detail if detail else "")); fail += 0 if ok else 1
REALSIZE = (within >= 0.85)
inside = all(len(set(np.unique(lab[m_["square"][1]:m_["square"][1] + S, m_["square"][0]:m_["square"][0] + S]).tolist())) == 1 and lab[m_["square"][1], m_["square"][0]] == m_["chart"] for m_ in markers)
gate("real size: at least 85% of the surface is within 15% of the target texel density", REALSIZE, f"({100 * within:.0f}% within; median {med:.0f} px/m, target {target:.0f})")
gate("every marker square lies inside one island", inside, f"({len(markers)} markers, {NCH} islands)")
occ = np.zeros((H, W), np.int32)
for m_ in markers: occ[m_["square"][1]:m_["square"][1] + S, m_["square"][0]:m_["square"][0] + S] += 1
gate("markers do not overlap", int(occ.max()) <= 1)
gate("ids unique", len({m_["id"] for m_ in markers}) == len(markers) and len(markers) <= NID, f"({len(markers)} of {NID} ids in the family)")
prm = aruco.DetectorParameters(); prm.minMarkerPerimeterRate = 0.002; prm.adaptiveThreshWinSizeMax = 103; prm.adaptiveThreshWinSizeStep = 10   # the atlas is 12000 px wide: the default 3% perimeter would reject a 66 px marker
det = aruco.ArucoDetector(d, prm); big = cv2.copyMakeBorder(canvas, 60, 60, 60, 60, cv2.BORDER_CONSTANT, value=70)
cs, ids, _ = det.detectMarkers(big); got = set() if ids is None else {int(x) for x in ids.flatten()}
gate("the clean atlas reads back every marker", got >= {m_["id"] for m_ in markers}, f"({len({m_['id'] for m_ in markers} & got)} of {len(markers)} read)")
blank = det.detectMarkers(np.full_like(big, 70))[1]
gate("a blank atlas reads nothing", blank is None)
print("ALL GATES PASS" if fail == 0 else f"{fail} GATE(S) FAILED")
cov = sum(S * S for _ in markers) / max(area_total, 1)
print(f"measure: {len(markers)} markers; marker squares cover {100 * cov:.1f}% of island area (one marker per two squares caps this at 50%); islands {NCH}; atlas {W} x {H} px")
vol = float(np.einsum("ij,ij->i", V[F[:, 0]], np.cross(V[F[:, 1]], V[F[:, 2]])).sum() / 6.0); so = 1.0 if vol > 0 else -1.0
placed = []
for m_ in markers:
    cx, cy = np.mean(m_["quad"], axis=0); t = int(tri[int(cy), int(cx)])
    if t < 0: continue
    q = uv[F[t]]; M = np.array([q[1] - q[0], q[2] - q[0]]).T
    if abs(np.linalg.det(M)) < 1e-9: continue
    bb = np.linalg.solve(M, np.array([cx, cy]) - q[0]); w = np.array([1 - bb.sum(), bb[0], bb[1]]); w = np.clip(w, 0, None); w /= w.sum()
    n = np.cross(V[F[t, 1]] - V[F[t, 0]], V[F[t, 2]] - V[F[t, 0]]); n = so * n / max(np.linalg.norm(n), 1e-20)
    placed.append({"id": m_["id"], "P": [float(c) for c in (w[:, None] * V[F[t]]).sum(0)], "n": [float(c) for c in n]})
json.dump(placed, open(os.path.join(out, "markers_c0.json"), "w")); json.dump([], open(os.path.join(out, "cams_c1.json"), "w"))
cv2.imwrite(os.path.join(out, "atlas.png"), canvas)
json.dump(markers, open(os.path.join(out, "suit_markers.json"), "w")); json.dump({"w": W, "h": H}, open(os.path.join(out, "suit_board_size.json"), "w"))
json.dump({"body": {"w": W, "h": H, "first": 0, "count": len(markers)}}, open(os.path.join(out, "boards_meta.json"), "w")); open(os.path.join(out, "suit_dictionary.txt"), "w").write(DICT)
with open(os.path.join(out, "anny_uv.obj"), "w") as f:
    for v in V: f.write("v %.6f %.6f %.6f\n" % tuple(v))
    for u in uv / [W, H]: f.write("vt %.6f %.6f\n" % tuple(u))
    for a, b, c_ in F: f.write("f %d/%d %d/%d %d/%d\n" % (a + 1, a + 1, b + 1, b + 1, c_ + 1, c_ + 1))
sys.exit(1 if fail else 0)
