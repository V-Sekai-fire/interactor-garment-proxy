# Per-marker decals on a sewn-suit grid layout. Each ChArUco marker of a mesh's board keeps its place on the surface
# (taken from the mesh's xatlas unwrap, godot/unwrap_tiles.gd with K=1) but gets its own chart: a 30 mm square patch built
# by straight geodesic walks across the triangles from the marker centre (an exponential map: a polar chart), so no seam
# can cut it and its size is true. Patches are lifted LIFT m along the surface normal and textured from the board with the
# patch's own board coordinates, so a marker's id is where it sits on the board.
# python decals.py <orig.obj> <unwrap.obj> <m per unit> <board.png> <out prefix> [<fitted.obj> <fitted m per unit>]
#   <unwrap.obj> is godot/unwrap_tiles.gd output (with its .json). With a fitted OBJ that has the same faces as <orig.obj>
#   the same markers are walked on the fitted surface too (out prefix + "_fit"). Needs numpy, opencv (cv2 5.0).
import os, sys, json, math, numpy as np, cv2
MSCALE = float(os.environ.get("MARKER_SCALE", "1.0"))   # 1.0 = 30 mm squares and 22 mm markers; 0.7071 = twice the markers per area
SQ_MM, MK_MM, PXM = 30.0 * MSCALE, 22.0 * MSCALE, 1500.0 / MSCALE
GRID, LIFT, MIN_SEP, FILL_SEP = 12, 0.0002, 0.0315, 0.040
T = (lambda v: v) if os.environ.get("YUP") == "1" else (lambda v: np.stack([v[:, 0], v[:, 2], -v[:, 1]], 1))   # solver Z-up -> Godot Y-up; YUP=1 for meshes that are Y up already (ANNY)
def read_obj(path):
    V, F = [], []
    for l in open(path):
        t = l.split()
        if t and t[0] == "v": V.append([float(x) for x in t[1:4]])
        elif t and t[0] == "f": F.append([int(x.split("/")[0]) - 1 for x in t[1:4]])
    return np.array(V), np.array(F)
def read_uv_obj(path):
    V, VT, F = [], [], []
    for l in open(path):
        t = l.split()
        if not t: continue
        if t[0] == "v": V.append([float(x) for x in t[1:4]])
        elif t[0] == "vt": VT.append([float(x) for x in t[1:3]])
        elif t[0] == "f": F.append([int(x.split("/")[0]) - 1 for x in t[1:4]])
    return np.array(V), np.array(VT), np.array(F)
class Walker:
    def __init__(self, V, F):
        self.V, self.F = V, F
        self.stat = dict(walks=0, boundary=0, noedge=0, exhausted=0)
        e = {}
        for t, f in enumerate(F):
            for k in range(3):
                a, b = int(f[k]), int(f[(k + 1) % 3]); e.setdefault((min(a, b), max(a, b)), []).append(t)
        self.edge = e
        n = np.cross(V[F[:, 1]] - V[F[:, 0]], V[F[:, 2]] - V[F[:, 0]]); self.N = n / np.maximum(np.linalg.norm(n, axis=1, keepdims=True), 1e-20)
    def walk(self, t, p, d, L):
        """Straight geodesic: start in triangle t at point p, unit direction d in the plane, length L. -> (t, p)"""
        V, F, N = self.V, self.F, self.N; skip = None; nudges = 0; self.stat['walks'] += 1
        for _ in range(24):
            n = N[t]; tri = F[t]; best = None
            for k in range(3):
                a, b = int(tri[k]), int(tri[(k + 1) % 3])
                if skip == (min(a, b), max(a, b)): continue
                P0, e = V[a], V[b] - V[a]; den = np.dot(np.cross(d, e), n)
                if abs(den) < 1e-14: continue
                s = np.dot(np.cross(P0 - p, e), n) / den; tt = np.dot(np.cross(P0 - p, d), n) / den
                if s > 1e-12 and -1e-6 <= tt <= 1 + 1e-6 and (best is None or s < best[0]): best = (s, a, b)
            if best is None:
                if nudges < 3:
                    nudges += 1; p = p + (V[tri].mean(0) - p) * 1e-3; skip = None; continue
                self.stat['noedge'] += 1; return t, p + d * L
            if best[0] >= L: return t, p + d * L
            s, a, b = best; p = p + d * s; L -= s; key = (min(a, b), max(a, b)); nb = [u for u in self.edge[key] if u != t]
            if not nb: self.stat['boundary'] += 1; return t, p
            u = nb[0]; eh = (V[b] - V[a]); eh = eh / np.linalg.norm(eh)
            opp = [int(x) for x in F[u] if int(x) not in (a, b)][0]; w = V[opp] - V[a]; w = w - np.dot(w, eh) * eh; w = w / np.linalg.norm(w)
            par = np.dot(d, eh); perp = math.sqrt(max(1 - par * par, 0.0)); d = par * eh + perp * w; d = d / np.linalg.norm(d)
            t, skip = u, key
        self.stat['exhausted'] += 1
        return t, p

    def walk_carry(self, t, p, d, L, carry, crease_cos):
        """Like walk, but carries extra in-plane vectors across the edges (a rotation about each crossed edge) and stops
        (ok = False) at an edge whose two triangles meet at a sharper angle than crease_cos (a crease) or at a boundary."""
        V, F, N = self.V, self.F, self.N; skip = None; nudges = 0
        for _ in range(24):
            n = N[t]; tri = F[t]; best = None
            for k in range(3):
                a, b = int(tri[k]), int(tri[(k + 1) % 3])
                if skip == (min(a, b), max(a, b)): continue
                P0, e = V[a], V[b] - V[a]; den = np.dot(np.cross(d, e), n)
                if abs(den) < 1e-14: continue
                s = np.dot(np.cross(P0 - p, e), n) / den; tt = np.dot(np.cross(P0 - p, d), n) / den
                if s > 1e-12 and -1e-6 <= tt <= 1 + 1e-6 and (best is None or s < best[0]): best = (s, a, b)
            if best is None:
                if nudges < 3: nudges += 1; p = p + (V[tri].mean(0) - p) * 1e-3; skip = None; continue
                return False, t, p, d, carry
            if best[0] >= L: return True, t, p + d * L, d, carry
            s, a, b = best; p = p + d * s; L -= s; key = (min(a, b), max(a, b)); nb = [u for u in self.edge[key] if u != t]
            if not nb: return False, t, p, d, carry
            u = nb[0]
            if np.dot(N[t], N[u]) < crease_cos: return False, t, p, d, carry
            eh = V[b] - V[a]; eh = eh / np.linalg.norm(eh)
            phi = math.atan2(np.dot(np.cross(N[t], N[u]), eh), np.dot(N[t], N[u])); cp, sp = math.cos(phi), math.sin(phi)
            rot = lambda v: v * cp + np.cross(eh, v) * sp + eh * np.dot(eh, v) * (1 - cp)
            d = rot(d); d = d / np.linalg.norm(d); carry = [rot(c) for c in carry]; t, skip = u, key
        return False, t, p, d, carry

def markers_of(board_png):
    """The real suit board (prototype/make_suit_board.py): [(idx, quad px)] over all 1592 markers, and the board size."""
    return [(m["idx"], np.array(m["quad"], np.float32)) for m in json.load(open("suit_markers.json"))], (49 * 45, 65 * 45)


def uvmap():
    """--uvmap <orig.obj> <unwrap.obj> <m per unit> <out prefix> [<fitted.obj> <fitted m per unit>]
    The real suit board on the xatlas UVs (seams where crease_split.py forced them): writes <prefix>_uv.obj (textured, Godot frame),
    <prefix>_markers.json (every marker whose centre lies on the mesh: P, outward n, intact = all 9 sample points in one chart)
    and, with a fitted OBJ of the same faces, the same for the fitted surface (<prefix>_fit_*). Prints the seam report."""
    a = sys.argv; orig, unw, sc, prefix = a[2], a[3], float(a[4]), a[5]
    V, F = read_obj(orig); V = T(V * sc); V2, VT2, F2 = read_uv_obj(unw); V2 = T(V2)
    meta = json.load(open(unw + ".json"))[0]; BWH = np.array([49 * 45, 65 * 45], float)
    VTb = VT2 * [meta["board_w"], meta["board_h"]]
    def outward(Vs, Fs):
        n = np.cross(Vs[Fs[:, 1]] - Vs[Fs[:, 0]], Vs[Fs[:, 2]] - Vs[Fs[:, 0]]); n /= np.maximum(np.linalg.norm(n, axis=1, keepdims=True), 1e-20)
        vol = float(np.einsum("ij,ij->i", Vs[Fs[:, 0]], np.cross(Vs[Fs[:, 1]], Vs[Fs[:, 2]])).sum() / 6.0)
        return n * (1.0 if vol > 0 else -1.0)
    N2 = outward(V2, F2)
    parent = list(range(len(V2)))
    def find(x):
        while parent[x] != x: parent[x] = parent[parent[x]]; x = parent[x]
        return x
    for f in F2:
        parent[find(f[1])] = find(f[0]); parent[find(f[2])] = find(f[0])
    chart = np.array([find(f[0]) for f in F2])
    # A chart whose UV orientation is mirrored prints mirrored markers (unreadable, or misread). Seen from outside, the board's
    # right and down axes must satisfy (dP/du x dP/dv) . n_out = -1; flip every chart that does not (u -> umin + umax - u).
    e1 = V2[F2[:, 1]] - V2[F2[:, 0]]; e2 = V2[F2[:, 2]] - V2[F2[:, 0]]
    du1, dv1 = (VTb[F2[:, 1]] - VTb[F2[:, 0]]).T; du2, dv2 = (VTb[F2[:, 2]] - VTb[F2[:, 0]]).T; det = du1 * dv2 - du2 * dv1
    c0 = (e1 * dv2[:, None] - e2 * dv1[:, None]) / np.where(np.abs(det) < 1e-12, 1e-12, det)[:, None]
    c1 = (-e1 * du2[:, None] + e2 * du1[:, None]) / np.where(np.abs(det) < 1e-12, 1e-12, det)[:, None]
    hand = np.einsum("ij,ij->i", np.cross(c0, c1), N2); wgt = np.abs(det)
    flipped = 0
    for ch in set(chart.tolist()):
        sel_t = np.nonzero(chart == ch)[0]
        if float((np.sign(hand[sel_t]) * wgt[sel_t]).sum()) > 0:       # mirrored
            vi = np.unique(F2[sel_t]); lo_, hi_ = VTb[vi, 0].min(), VTb[vi, 0].max(); VTb[vi, 0] = lo_ + hi_ - VTb[vi, 0]; flipped += 1
    print(f"charts flipped to remove mirroring: {flipped} of {len(set(chart.tolist()))}")
    # Tile geometry: do the board's squares stay squares of the real size on the surface? Per triangle, the singular values
    # (s1 >= s2) of the UV-to-surface Jacobian, scaled so 1.0 is the real size (1500 px/m): size factor s1*s2, shear s1/s2.
    du1, dv1 = (VTb[F2[:, 1]] - VTb[F2[:, 0]]).T; du2, dv2 = (VTb[F2[:, 2]] - VTb[F2[:, 0]]).T; det = du1 * dv2 - du2 * dv1
    sd = np.where(np.abs(det) < 1e-12, 1e-12, det)[:, None]
    c0 = (e1 * dv2[:, None] - e2 * dv1[:, None]) / sd; c1 = (-e1 * du2[:, None] + e2 * du1[:, None]) / sd
    ga, gb, gd = np.einsum("ij,ij->i", c0, c0), np.einsum("ij,ij->i", c0, c1), np.einsum("ij,ij->i", c1, c1)
    root = np.sqrt(np.maximum((ga - gd) ** 2 + 4 * gb ** 2, 0)); l1 = (ga + gd + root) / 2; l2 = np.maximum((ga + gd - root) / 2, 1e-18)
    s1, s2 = np.sqrt(l1) * PXM, np.sqrt(l2) * PXM; shear = s1 / s2; size = s1 * s2
    print(f"tile geometry per triangle: shear s1/s2 median {np.median(shear):.2f}, p95 {np.percentile(shear, 95):.2f}, {100.0 * (shear <= 1.2).mean():.0f}% within 1.2; size factor p5 {np.percentile(size, 5):.2f}, p95 {np.percentile(size, 95):.2f}, {100.0 * ((size > 0.85) & (size < 1.18)).mean():.0f}% within 15%")
    # seam report: a 3D edge whose two triangles do not share the same pair of vertex ids is a seam
    q = lambda p: tuple(np.round(p * 2000).astype(int))        # 0.5 mm bins
    ed = {}
    for t, f in enumerate(F2):
        for k in range(3):
            i, j = int(f[k]), int(f[(k + 1) % 3]); key = tuple(sorted((q(V2[i]), q(V2[j])))); ed.setdefault(key, []).append((t, (min(i, j), max(i, j))))
    cosT = math.cos(math.radians(40.0)); seam_len = crease_seam = crease_len = 0.0; nseam = 0
    for key, lst in ed.items():
        if len(lst) != 2: continue
        t0, e0 = lst[0]; t1, e1 = lst[1]; i, j = e0; ln = float(np.linalg.norm(V2[i] - V2[j])); is_crease = float(np.dot(N2[t0], N2[t1])) < cosT
        if is_crease: crease_len += ln
        if e0 != e1:
            seam_len += ln; nseam += 1
            if is_crease: crease_seam += ln
    print(f"charts {len(set(chart.tolist()))}; seam length {seam_len:.1f} m in {nseam} edges; {100.0 * crease_seam / max(seam_len, 1e-9):.1f}% of the seam length is on creases; {100.0 * crease_seam / max(crease_len, 1e-9):.1f}% of the crease length is a seam")
    # At human scale the atlas can be larger than one printed sheet: repeat the board across it (the paper reuses ids too, as an
    # "instance"). Marker idx = position on the sheet, instance = which copy of the sheet.
    ATL = np.maximum(np.ceil(VTb.max(0) / BWH).astype(int), 1)
    mkall = [(idx, cy * int(ATL[0]) + cx, qd + np.array([cx * BWH[0], cy * BWH[1]], np.float32)) for cy in range(int(ATL[1])) for cx in range(int(ATL[0])) for idx, qd in markers_of(None)[0]]
    print(f"atlas {VTb.max(0)[0]:.0f} x {VTb.max(0)[1]:.0f} px, board copies {int(ATL[0])} x {int(ATL[1])}, markers to test {len(mkall)}")
    lab = np.full((int(ATL[1] * BWH[1]) + 1, int(ATL[0] * BWH[0]) + 1), -1, np.int32)
    for t, f in enumerate(F2): cv2.fillConvexPoly(lab, np.round(VTb[f]).astype(np.int32), t)
    def to3d(px):
        x, y = int(round(px[0])), int(round(px[1]))
        if not (0 <= y < lab.shape[0] and 0 <= x < lab.shape[1]) or lab[y, x] < 0: return None
        t = int(lab[y, x]); uv = VTb[F2[t]]; M = np.array([uv[1] - uv[0], uv[2] - uv[0]]).T
        if abs(np.linalg.det(M)) < 1e-9: return None
        bb = np.linalg.solve(M, np.asarray(px) - uv[0]); w = np.array([1 - bb.sum(), bb[0], bb[1]]); w = np.clip(w, 0, None); w /= w.sum()
        return (w[:, None] * V2[F2[t]]).sum(0)
    sq_ok = sq_all = 0
    for idx_, inst_, qd_ in mkall:
        c_ = qd_.mean(0); cs_ = [to3d(c_ + 0.85 * (p - c_)) for p in qd_]
        if any(v is None for v in cs_): continue
        sq_all += 1; sides = [np.linalg.norm(cs_[i] - cs_[(i + 1) % 4]) / 0.85 for i in range(4)]
        angs = [math.degrees(math.acos(np.clip(np.dot(cs_[(i + 1) % 4] - cs_[i], cs_[(i - 1) % 4] - cs_[i]) / max(np.linalg.norm(cs_[(i + 1) % 4] - cs_[i]) * np.linalg.norm(cs_[(i - 1) % 4] - cs_[i]), 1e-12), -1, 1))) for i in range(4)]
        if max(abs(x / (MK_MM / 1000.0) - 1) for x in sides) <= 0.10 and max(abs(a_ - 90) for a_ in angs) <= 8: sq_ok += 1
    print(f"markers that stay square on the surface (sides within 10% of 22 mm and corners within 8 deg of 90, whole markers only): {sq_ok} of {sq_all} ({100.0 * sq_ok / max(sq_all, 1):.0f}%)")
    out = []; stat = dict(intact=0, cut=0, void=0)
    g = np.linspace(0.1, 0.9, 3)
    for idx, inst, qd in mkall:
        pts = [(qd[0] * (1 - u) + qd[1] * u) * (1 - w) + (qd[3] * (1 - u) + qd[2] * u) * w for u in g for w in g]
        ts = [int(lab[int(np.clip(round(p[1]), 0, lab.shape[0] - 1)), int(np.clip(round(p[0]), 0, lab.shape[1] - 1))]) for p in pts]
        if all(t >= 0 for t in ts) and len({int(chart[t]) for t in ts}) == 1: stat["intact"] += 1; intact = True
        elif any(t >= 0 for t in ts): stat["cut"] += 1; intact = False
        else: stat["void"] += 1; continue
        t = ts[4]
        if t < 0: continue                                   # a cut marker whose centre is in a gap: no point to aim a camera at
        uv = VTb[F2[t]]; M = np.array([uv[1] - uv[0], uv[2] - uv[0]]).T
        if abs(np.linalg.det(M)) < 1e-9: continue
        bb = np.linalg.solve(M, qd.mean(0) - uv[0]); b = np.array([1 - bb.sum(), bb[0], bb[1]]); b = np.clip(b, 0, None); b /= b.sum()
        out.append(dict(idx=idx, inst=inst, t=t, b=b, intact=intact))
    print(f"markers of the board on the mesh: {stat['intact'] + stat['cut']} ({stat['intact']} intact, {stat['cut']} cut by a seam or the chart edge); {100.0 * stat['intact'] / max(stat['intact'] + stat['cut'], 1):.1f}% intact")
    def write(Vs, name):
        Ns = outward(Vs, F2)
        with open(prefix + name + "_uv.obj", "w") as fh:
            for v in Vs: fh.write("v %.6f %.6f %.6f\n" % tuple(v))
            for u in VTb / BWH: fh.write("vt %.6f %.6f\n" % tuple(u))
            for x, y, z in F2: fh.write("f %d/%d %d/%d %d/%d\n" % (x + 1, x + 1, y + 1, y + 1, z + 1, z + 1))
        json.dump([{"id": m["idx"], "instance": m["inst"], "P": [float(c) for c in (m["b"][:, None] * Vs[F2[m["t"]]]).sum(0)], "n": [float(c) for c in Ns[m["t"]]], "intact": m["intact"]} for m in out], open(prefix + name + "_markers.json", "w"))
    write(V2, "")
    if len(a) > 7:
        Vf, Ff = read_obj(a[6]); Vf = T(Vf * float(a[7])); assert (Ff == F).all(), "the fitted mesh must have the same faces"
        qv = lambda p: tuple(np.round(p * 1e4).astype(int)); lookup = {qv(p): i for i, p in enumerate(V)}; miss = 0; Vf2 = np.zeros_like(V2)
        for i, p in enumerate(V2):
            k = qv(p); hit = None
            for dx in (0, -1, 1):
                for dy in (0, -1, 1):
                    for dz in (0, -1, 1):
                        kk = (k[0] + dx, k[1] + dy, k[2] + dz)
                        if hit is None and kk in lookup: hit = lookup[kk]
            if hit is None: miss += 1; hit = 0
            Vf2[i] = Vf[hit]
        print("fitted surface: vertices without a match", miss, "of", len(V2)); write(Vf2, "_fit")

def main():
    if len(sys.argv) > 1 and sys.argv[1] == "--uvmap": return uvmap()
    orig, unw, sc, board, prefix = sys.argv[1], sys.argv[2], float(sys.argv[3]), sys.argv[4], sys.argv[5]
    V, F = read_obj(orig); V = T(V * sc)
    V2, VT2, F2 = read_uv_obj(unw); V2 = T(V2); meta = json.load(open(unw + ".json"))[0]
    VT2 = VT2 * [meta["board_w"], meta["board_h"]]
    mk, (BW, BH) = markers_of(board)
    meta_r = json.load(open("boards_meta.json"))[os.environ.get("MESH", "body")]; LO, CAP = meta_r["first"], meta_r["count"]   # this mesh's idx range on the board
    quad_of = {i: q for i, q in mk}
    # xatlas triangle -> original triangle, by centroid
    key = lambda c: tuple(np.round(c * 1e4).astype(int))
    cen_o = {key(V[f].mean(0)): t for t, f in enumerate(F)}
    tri_o = {}
    for t2, f in enumerate(F2):
        k = key(V2[f].mean(0))
        for dx in (0, -1, 1):
            for dy in (0, -1, 1):
                for dz in (0, -1, 1):
                    kk = (k[0] + dx, k[1] + dy, k[2] + dz)
                    if kk in cen_o and t2 not in tri_o: tri_o[t2] = cen_o[kk]
    print("xatlas triangles matched to the original:", len(tri_o), "of", len(F2))
    lab = np.full((meta["board_h"] + 1, meta["board_w"] + 1), -1, np.int32)
    for t2, f in enumerate(F2): cv2.fillConvexPoly(lab, np.round(VT2[f]).astype(np.int32), t2)
    W = Walker(V, F); recs = []
    for mid, q in mk:
        c = q.mean(0); x, y = int(round(c[0])), int(round(c[1]))
        if not (0 <= y < lab.shape[0] and 0 <= x < lab.shape[1]) or lab[y, x] < 0 or lab[y, x] not in tri_o: continue
        t2 = lab[y, x]; uv = VT2[F2[t2]]; M = np.array([uv[1] - uv[0], uv[2] - uv[0]]).T
        if abs(np.linalg.det(M)) < 1e-9: continue
        bary = np.linalg.solve(M, c - uv[0]); b = np.array([1 - bary.sum(), bary[0], bary[1]])
        b = np.clip(b, 1e-4, None); b = b / b.sum()   # the centre pixel can sit just outside its triangle
        t = tri_o[t2]; tri = F[t]; P = (b[:, None] * V[tri]).sum(0)
        E = np.array([V2[F2[t2][1]] - V2[F2[t2][0]], V2[F2[t2][2]] - V2[F2[t2][0]]]).T  # 3D edges of the xatlas triangle
        J = E @ np.linalg.inv(M)                  # d(3D)/d(uv px)
        n = W.N[t]; eu = J[:, 0] - np.dot(J[:, 0], n) * n; eu /= np.linalg.norm(eu)
        ev = J[:, 1] - np.dot(J[:, 1], n) * n; ev = ev - np.dot(ev, eu) * eu; ev /= np.linalg.norm(ev)
        sgn = 1.0 if np.dot(np.cross(eu, ev), n) > 0 else -1.0
        # angle of eu in the triangle plane relative to its first edge, so the same marker can be placed on a fitted copy
        e0 = V[tri[1]] - V[tri[0]]; e0 /= np.linalg.norm(e0); ang = math.atan2(np.dot(np.cross(e0, eu), n), np.dot(e0, eu))
        recs.append(dict(id=mid, t=t, b=b, ang=ang, sgn=sgn, P=P, c=c, eu3=eu))
    print("markers whose centre is on the mesh:", len(recs), "of", len(mk))
    # greedy: keep a marker if it is at least MIN_SEP from every kept one
    kept, grid = [], {}
    for r in recs:
        g = tuple((r["P"] // MIN_SEP).astype(int)); ok = True
        for dx in (-1, 0, 1):
            for dy in (-1, 0, 1):
                for dz in (-1, 0, 1):
                    for o in grid.get((g[0] + dx, g[1] + dy, g[2] + dz), []):
                        if np.linalg.norm(o["P"] - r["P"]) < MIN_SEP: ok = False
        if ok: kept.append(r); grid.setdefault(g, []).append(r)
    print("kept after overlap filter:", len(kept))
    # fill the gaps (seams drop markers): farthest-point seeding on the surface with the board markers that are still
    # unused, so every id stays unique and the layout keeps the board's density (one marker per two 30 mm squares)
    free = list(range(max(CAP - len(kept), 0)))   # room left in this mesh's idx range
    cen = V[F].mean(1); dmin = np.full(len(F), 1e9)
    for r in kept: dmin = np.minimum(dmin, np.linalg.norm(cen - r["P"], axis=1))
    sg = 1.0 if sum(r["sgn"] for r in kept) >= 0 else -1.0; nfill = 0
    while free:
        i = int(np.argmax(dmin))
        if dmin[i] < FILL_SEP: break
        P = cen[i]; n = W.N[i]; j = int(np.argmin([np.linalg.norm(r["P"] - P) for r in kept]))
        eu = kept[j]["eu3"] - np.dot(kept[j]["eu3"], n) * n
        if np.linalg.norm(eu) < 1e-6: eu = np.cross(n, [1.0, 0.0, 0.0]) if abs(n[0]) < 0.9 else np.cross(n, [0.0, 1.0, 0.0])
        eu /= np.linalg.norm(eu); tri = F[i]; e0 = V[tri[1]] - V[tri[0]]; e0 /= np.linalg.norm(e0)
        ang = math.atan2(np.dot(np.cross(e0, eu), n), np.dot(e0, eu)); free.pop(0); mid, c = -1, np.zeros(2)
        kept.append(dict(id=mid, t=i, b=np.array([1 / 3.0] * 3), ang=ang, sgn=sg, P=P, c=c, eu3=eu)); nfill += 1
        dmin = np.minimum(dmin, np.linalg.norm(cen - P, axis=1))
    print("gap fillers added:", nfill, "; markers now", len(kept), "; unused ids left", len(free), "; largest remaining gap (mm):", round(float(dmin.max()) * 1000, 1))
    kept = kept[:CAP]
    for j, r in enumerate(kept):          # each decal takes the next marker of this mesh's range: idx = board position, id = idx % 1000
        r["id"] = LO + j; r["c"] = quad_of[LO + j].mean(0)
    print("assigned board markers idx", LO, "to", LO + len(kept) - 1, "(ids", LO % 1000, "..., repeating past 1000)")
    def build(Vs, Fs, name):
        Ws = Walker(Vs, Fs); verts, uvs, faces, areas, errs, centres, normals = [], [], [], [], [], [], []
        # Handedness is fixed by the geometry, not copied from the chart a marker was laid out on (xatlas charts can be flipped,
        # which mirrored the markers): seen from outside, the board's right and down axes (eu, ev) must satisfy (eu x ev) . n_out = -1.
        so = 1.0 if float(np.einsum("ij,ij->i", Vs[Fs[:, 0]], np.cross(Vs[Fs[:, 1]], Vs[Fs[:, 2]])).sum() / 6.0) > 0 else -1.0
        half = SQ_MM / 2000.0
        for r in kept:
            tri = Fs[r["t"]]; n = Ws.N[r["t"]]; P = (r["b"][:, None] * Vs[tri]).sum(0)
            e0 = Vs[tri[1]] - Vs[tri[0]]; e0 /= np.linalg.norm(e0)
            eu = math.cos(r["ang"]) * e0 + math.sin(r["ang"]) * np.cross(n, e0); ev = (-so) * np.cross(n, eu)
            base = len(verts); pts = np.zeros((GRID + 1, GRID + 1, 3))
            for iy in range(GRID + 1):
                for ix in range(GRID + 1):
                    a = -half + SQ_MM / 1000.0 * ix / GRID; bb = -half + SQ_MM / 1000.0 * iy / GRID; dist = math.hypot(a, bb)
                    if dist < 1e-9: t, p = r["t"], P
                    else:
                        d = (a * eu + bb * ev) / dist; t, p = Ws.walk(r["t"], P, d, dist)
                    pts[iy, ix] = p + Ws.N[t] * LIFT
                    verts.append(pts[iy, ix]); uvs.append(((r["c"][0] + (ix / GRID - 0.5) * SQ_MM * PXM / 1000.0) / BW, (r["c"][1] + (iy / GRID - 0.5) * SQ_MM * PXM / 1000.0) / BH))
            for iy in range(GRID):
                for ix in range(GRID):
                    i0 = base + iy * (GRID + 1) + ix; faces += [(i0, i0 + 1, i0 + GRID + 2), (i0, i0 + GRID + 2, i0 + GRID + 1)]
            ar = sum(0.5 * np.linalg.norm(np.cross(pts[iy, ix + 1] - pts[iy, ix], pts[iy + 1, ix] - pts[iy, ix])) + 0.5 * np.linalg.norm(np.cross(pts[iy + 1, ix + 1] - pts[iy, ix + 1], pts[iy + 1, ix] - pts[iy + 1, ix + 1])) for iy in range(GRID) for ix in range(GRID))
            areas.append(ar / (SQ_MM / 1000.0) ** 2); centres.append(pts[GRID // 2, GRID // 2]); normals.append(Ws.N[r["t"]])
            errs.append(max(abs(np.linalg.norm(pts[0, GRID] - pts[0, 0]) / (SQ_MM / 1000.0) - 1), abs(np.linalg.norm(pts[GRID, 0] - pts[0, 0]) / (SQ_MM / 1000.0) - 1)))
        with open(prefix + name + "_decals.obj", "w") as f:
            for v in verts: f.write("v %.6f %.6f %.6f\n" % tuple(v))
            for u in uvs: f.write("vt %.6f %.6f\n" % u)
            for a, b, c in faces: f.write("f %d/%d %d/%d %d/%d\n" % (a + 1, a + 1, b + 1, b + 1, c + 1, c + 1))
        with open(prefix + name + "_base.obj", "w") as f:
            for v in Vs: f.write("v %.6f %.6f %.6f\n" % tuple(v))
            for a, b, c in Fs: f.write("f %d %d %d\n" % (a + 1, b + 1, c + 1))
        vol = float(np.einsum("ij,ij->i", Vs[Fs[:, 0]], np.cross(Vs[Fs[:, 1]], Vs[Fs[:, 2]])).sum() / 6.0); so = 1.0 if vol > 0 else -1.0
        json.dump([{"id": r["id"], "P": [float(x) for x in cp], "n": [float(x * so) for x in nn]} for r, cp, nn in zip(kept, centres, normals)], open(prefix + name + "_markers.json", "w"))
        ar = np.array(areas); er = np.array(errs); print(name or 'input', 'walk stats', Ws.stat, '; patches with a side more than 10% off:', int((er > 0.1).sum()), 'of', len(er))
        print(f"{name or 'input'}: {len(kept)} decals, patch area / 30 mm square: mean {ar.mean():.3f} p5 {np.percentile(ar,5):.3f} p95 {np.percentile(ar,95):.3f}; side length error: mean {100*er.mean():.1f}% p95 {100*np.percentile(er,95):.1f}%")
        json.dump([r["id"] for r in kept], open(prefix + name + "_ids.json", "w"))
    build(V, F, "")
    if len(sys.argv) > 7:
        Vf, Ff = read_obj(sys.argv[6]); Vf = T(Vf * float(sys.argv[7]))
        assert (Ff == F).all(), "the fitted mesh must have the same faces"
        build(Vf, Ff, "_fit")
main()
