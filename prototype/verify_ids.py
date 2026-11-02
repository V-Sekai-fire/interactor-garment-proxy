# Proof that a detected marker is the right one: decode the UV at each detected corner (u and v as 16 bits in the
# _u / _v images from godot/fit_sheet.gd) and compare with where that id sits on its own board, in mm. Correct = mean
# error under 4 mm, max under 8 mm, and the id belongs to the mesh the pixel colour says. A neighbouring id is the
# planted control and must never be accepted. Run in the folder holding dict6x6_2000.npy, boards_meta.json and the
# board PNGs: python prototype/verify_ids.py <cells dir> <overview.png>
import sys, numpy as np, cv2, cv2.aruco as aruco
folder, out = sys.argv[1:3]
import json
PAD = 100
d = aruco.Dictionary(np.load("dict6x6_2000.npy"), 6, 4)
det = aruco.ArucoDetector(d, aruco.DetectorParameters())
META = json.load(open("boards_meta.json"))
def mesh_of(i):
    for n, m in META.items():
        if m["first"] <= i < m["first"] + m["count"]: return n
    return None
# ground truth: where each id sits on the clean sheet (sheet px, detector corner order)
truth = {}
for name in META:
    bi = cv2.copyMakeBorder(cv2.imread(f"board_{name}.png", 0), PAD, PAD, PAD, PAD, cv2.BORDER_CONSTANT, value=255)
    c0, i0, _, _ = det.detectMarkersWithConfidence(bi)
    for q, i in zip(c0, i0.flatten()):
        truth.setdefault(int(i), []).append(q.reshape(4, 2) - PAD)
print("ids located:", len(truth), "placements:", sum(len(v) for v in truth.values()), "(each id once:", all(len(v) == 1 for v in truth.values()), ")")
def err_mm(sheet_pts, idx, inset_w, SZ):
    """smallest over placements of idx of the mean/max corner error, in mm (toroidal)."""
    best = (1e9, 1e9)
    for q in truth.get(idx, []):
        c = q.mean(axis=0)
        exp = c + inset_w * (q - c)
        dlt = sheet_pts - exp
        dlt -= np.round(dlt / SZ) * SZ
        e = np.linalg.norm(dlt, axis=1) / 1.5     # 1500 px/m -> 1.5 px/mm
        if e.mean() < best[0]: best = (float(e.mean()), float(e.max()))
    return best
INSET = 0.7
def decode(U, V, M, pt, WH):
    x, y = int(round(pt[0])), int(round(pt[1]))
    if y < 2 or x < 2 or y >= M.shape[0]-2 or x >= M.shape[1]-2 or not M[y-2:y+3, x-2:x+3].all():
        return None
    u = (U[y, x, 2]*256 + U[y, x, 1]) / 65535.0
    v = (V[y, x, 2]*256 + V[y, x, 1]) / 65535.0
    return np.array([u, v]) * WH
def run(c, v):
    im = cv2.imread(f"{folder}/v{v}_c{c}.png")
    U = cv2.imread(f"{folder}/v{v}_c{c}_u.png").astype(int)
    V = cv2.imread(f"{folder}/v{v}_c{c}_v.png").astype(int)
    bg = np.array([0.20, 0.20, 0.24]) * 255
    M = np.abs(im[..., ::-1].astype(float) - bg).max(axis=2) >= 2
    corners, ids, _, _ = det.detectMarkersWithConfidence(cv2.cvtColor(im, cv2.COLOR_BGR2GRAY))
    rows = []
    if ids is None: return im, rows
    for q, i in zip(corners, ids.flatten()):
        q = q.reshape(4, 2); ctr = q.mean(axis=0)
        px = q.astype(np.int32); m = np.zeros(M.shape, np.uint8); cv2.fillPoly(m, [px], 1)
        sel = im[m > 0].astype(int)
        br = sel[sel.sum(axis=1) >= np.percentile(sel.sum(axis=1), 60)]
        mesh = "skirt" if br[:, 2].mean() > br[:, 0].mean() + 20 else "body"
        mid = mesh_of(int(i))
        if mid is None:
            rows.append((mesh, int(i), "WRONG", (999.0, 999.0, False, False), q)); continue
        WH = np.array([META[mid]["w"], META[mid]["h"]], float)
        pts = [decode(U, V, M, ctr + INSET * (p - ctr), WH) for p in q]
        if any(p is None for p in pts):
            rows.append((mesh, int(i), "unverifiable", None, q)); continue
        sp = np.array(pts)
        mean, mx = err_mm(sp, int(i), INSET, WH)
        f0, n0 = META[mid]["first"], META[mid]["count"]
        ctl = err_mm(sp, f0 + ((int(i) - f0 + 1) % n0), INSET, WH)   # planted control: neighbouring id of the same mesh
        ok = mean < 4.0 and mx < 8.0 and mid == mesh
        okc = ctl[0] < 4.0 and ctl[1] < 8.0
        rows.append((mesh, int(i), "correct" if ok else "WRONG", (mean, mx, okc, mid == mesh), q))
    return im, rows
tiles = {}
tot = {}
print("view col  mesh  detected correct WRONG unverifiable  mean_err_mm(correct)  planted-control-accepted  id/colour-mismatch")
for v in range(4):
    for c in range(2):
        im, rows = run(c, v)
        ov = im.copy()
        for mesh, i, st, info, q in rows:
            col = {"correct": (60, 220, 60), "WRONG": (40, 40, 240), "unverifiable": (0, 200, 255)}[st]
            cv2.polylines(ov, [q.astype(np.int32)], True, col, 5)
            cv2.putText(ov, str(i), tuple(q.mean(axis=0).astype(int) + np.array([-18, 6])), cv2.FONT_HERSHEY_SIMPLEX, 0.7, col, 2)
        bgc = np.array([0.20, 0.20, 0.24]) * 255
        mk = np.abs(im[..., ::-1].astype(float) - bgc).max(axis=2) >= 2
        ys, xs = np.where(mk)
        y0, y1, x0, x1 = max(ys.min()-40, 0), min(ys.max()+40, im.shape[0]), max(xs.min()-40, 0), min(xs.max()+40, im.shape[1])
        crop = ov[y0:y1, x0:x1]
        TW, TH = 760, 900
        sc = min(TW / crop.shape[1], TH / crop.shape[0])
        crop = cv2.resize(crop, None, fx=sc, fy=sc, interpolation=cv2.INTER_AREA)
        tile = np.full((TH, TW, 3), 18, np.uint8)
        oy, ox = (TH - crop.shape[0]) // 2, (TW - crop.shape[1]) // 2
        tile[oy:oy+crop.shape[0], ox:ox+crop.shape[1]] = crop
        g = sum(r[2] == "correct" for r in rows); b = sum(r[2] == "WRONG" for r in rows)
        cv2.putText(tile, f"view {v}  {'input' if c==0 else 'fitted'}:  {g} right, {b} wrong", (12, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2)
        tiles[(v, c)] = tile
        for mesh in ("body", "skirt"):
            rr = [r for r in rows if r[0] == mesh]
            if not rr: continue
            n = len(rr); k = sum(r[2] == "correct" for r in rr); w = sum(r[2] == "WRONG" for r in rr); u = sum(r[2] == "unverifiable" for r in rr)
            me = np.mean([r[3][0] for r in rr if r[2] == "correct"]) if k else float("nan")
            cc = sum(1 for r in rr if r[3] and r[3][2])
            mm = sum(1 for r in rr if r[3] and len(r[3]) > 3 and not r[3][3])
            print(f"{v}   {'start' if c==0 else 'fit  '} {mesh:5s} {n:5d} {k:7d} {w:6d} {u:6d}   {me:6.2f}   {cc}   {mm}")
            t = tot.setdefault((c, mesh), [0, 0, 0, 0, 0, 0]); t[0]+=n; t[1]+=k; t[2]+=w; t[3]+=u; t[4]+=cc; t[5]+=mm
for (c, mesh), t in sorted(tot.items()):
    print(f"TOTAL {'start' if c==0 else 'fit  '} {mesh:5s} detected {t[0]} correct {t[1]} wrong {t[2]} unverifiable {t[3]} control-accepted {t[4]} id/colour-mismatch {t[5]}")
h, w = tiles[(0, 0)].shape[:2]
sheet = np.full((h*2 + 70, w*4, 3), 18, np.uint8)
cv2.putText(sheet, "Unique ids per mesh (body 0-1149, skirt 1150-1814): green = right id and place, red = wrong", (12, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (240, 240, 240), 2)
cv2.putText(sheet, "left: cloth-fit input    right: fitted (same UV layout on the skirt)", (12, 58), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (200, 200, 200), 1)
for (v, c), t in tiles.items():
    r_, k_ = divmod(v, 2)
    sheet[70 + r_*h: 70 + (r_+1)*h, (k_*2 + c)*w:(k_*2 + c + 1)*w] = t
cv2.imwrite(out, sheet)
