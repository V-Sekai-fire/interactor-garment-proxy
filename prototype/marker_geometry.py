# Geometry of every painted marker on the surface: lifts a 5x5 grid over each marker's quad from atlas pixels to 3D and reports how square and flat it is.
# python marker_geometry.py <textured.obj> <dir with suit_markers.json, reads.json> [marker scale]  -> prints the table of unread vs read markers
import sys, os, json, math, numpy as np, cv2
src, d = sys.argv[1], sys.argv[2]; MS = float(sys.argv[3]) if len(sys.argv) > 3 else 1.0
BWH = np.array([49 * 45, 65 * 45], float); V, VT, F = [], [], []
for l in open(src):
    t = l.split()
    if not t: continue
    if t[0] == "v": V.append([float(x) for x in t[1:4]])
    elif t[0] == "vt": VT.append([float(x) for x in t[1:3]])
    elif t[0] == "f": F.append([int(x.split("/")[0]) - 1 for x in t[1:4]])
V = np.array(V); F = np.array(F); uv = np.array(VT) * BWH
W, H = int(math.ceil(uv[:, 0].max())) + 2, int(math.ceil(uv[:, 1].max())) + 2
tri = np.full((H, W), -1, np.int32)
for t, f in enumerate(F): cv2.fillConvexPoly(tri, np.round(uv[f]).astype(np.int32), t)
def lift(x, y):
    t = tri[min(max(int(y), 0), H - 1), min(max(int(x), 0), W - 1)]
    if t < 0: return None
    a, b, c = uv[F[t]]; M = np.array([[b[0] - a[0], c[0] - a[0]], [b[1] - a[1], c[1] - a[1]]])
    if abs(np.linalg.det(M)) < 1e-9: return None
    u, v = np.linalg.solve(M, [x - a[0], y - a[1]]); return V[F[t, 0]] * (1 - u - v) + V[F[t, 1]] * u + V[F[t, 2]] * v
def stats(m, MK=33):
    qx, qy = m["quad"][0]; g = np.linspace(0, MK, 5); P = np.zeros((5, 5, 3))
    for i, yy in enumerate(g):
        for j, xx in enumerate(g):
            p = lift(qx + xx, qy + yy)
            if p is None: return None
            P[i, j] = p
    exp = 0.022 * MS
    sides = [np.linalg.norm(P[0, 4] - P[0, 0]), np.linalg.norm(P[4, 4] - P[0, 4]), np.linalg.norm(P[4, 4] - P[4, 0]), np.linalg.norm(P[4, 0] - P[0, 0])]
    size = max(abs(s / exp - 1) for s in sides)
    c = P.reshape(-1, 3); c0 = c.mean(0); n = np.linalg.svd(c - c0)[2][2]; flat = np.abs((c - c0) @ n).max() / exp
    def ang(a, b, cc): u, v = a - b, cc - b; return math.degrees(math.acos(np.clip(u @ v / (np.linalg.norm(u) * np.linalg.norm(v) + 1e-12), -1, 1)))
    sq = max(abs(ang(P[0, 4], P[0, 0], P[4, 0]) - 90), abs(ang(P[0, 0], P[0, 4], P[4, 4]) - 90), abs(ang(P[0, 4], P[4, 4], P[4, 0]) - 90), abs(ang(P[4, 4], P[4, 0], P[0, 0]) - 90))
    return size, sq, flat
if __name__ == "__main__":
    mk = json.load(open(f"{d}/suit_markers.json")); r = json.load(open(f"{d}/reads.json"))["0:body"]
    rd = {int(k): v for k, v in r["reads"].items()}; wl = {int(k): v for k, v in r["well"].items()}
    rows = {"gap": [], "read": []}
    for m in mk:
        s = stats(m)
        if s is None: continue
        i = m["idx"]
        if wl.get(i, 0) >= 2: rows["read" if rd.get(i, 0) >= 2 else "gap"].append(s)
    for k, v in rows.items():
        a = np.array(v); print(f"{k:5s} n={len(a)}  size err p50/p95/max {np.median(a[:,0]):.3f}/{np.percentile(a[:,0],95):.3f}/{a[:,0].max():.3f}   corner err deg p50/p95/max {np.median(a[:,1]):.1f}/{np.percentile(a[:,1],95):.1f}/{a[:,1].max():.1f}   flatness p50/p95/max {np.median(a[:,2]):.3f}/{np.percentile(a[:,2],95):.3f}/{a[:,2].max():.3f}")
    np.save(f"{d}/geom_stats.npy", np.array([[m["idx"], *(stats(m) or (np.nan,)*3), (1 if rd.get(m["idx"],0)>=2 else 0), wl.get(m["idx"],0)] for m in mk]))
