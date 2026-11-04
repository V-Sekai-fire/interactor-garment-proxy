# Rotation-star labels for the marker tiles, from the sim's own cameras (exact intrinsics, no depth model).
# A marker's frame is X = corner1 - corner0, Y = corner0 - corner3 (unit, pointing up the printed marker), Z = X x Y (out of the surface, OpenCV's ArUco marker convention: X right, Y up, Z toward the viewer); its star is the origin and the ends of +-X, +-Y, +-Z at radius R = half the marker side:
# 7 points, as in contract-rotation-star. Each star point is projected through the render's camera (Godot Camera3D: look_at, vertical fov).
# Gate first: the same projection of the four 3D corners must reproduce the existing corner labels (median error under 1.5 px), or no star labels are written.
# python star_labels.py <textured obj> <dir with suit_markers.json, cams_c0.json> <renders folder> <coco export dir> <out coco json> [marker scale = 0.5]
import sys, os, re, json, math, numpy as np, cv2
src, d, cells, coco, out = sys.argv[1:6]; MS = float(sys.argv[6]) if len(sys.argv) > 6 else 0.5
sys.argv = ["x", src, d, str(MS)]; sys.path.insert(0, os.path.dirname(os.path.abspath(__file__))); import marker_geometry as mg
mk = {m["id"]: m for m in json.load(open(f"{d}/suit_markers.json"))}; cams = json.load(open(f"{d}/cams_c0.json")); R = 0.011 * MS / 2   # half the 22 mm * scale marker
frame = cv2.imread(f"{cells}/v0_c0.png"); H, W = frame.shape[:2]
def corners3d(m):
    qx, qy = m["quad"][0]; MK = 33; pts = [mg.lift(qx + a, qy + b) for a, b in ((0, 0), (MK, 0), (MK, MK), (0, MK))]
    return None if any(p is None for p in pts) else np.array(pts)
def square3d(m):
    px, py = m["square"]; S = 45; e = 0.5   # the grid square's four corners, pulled in half a texel so the lift stays inside the island
    pts = [mg.lift(px + a, py + b) for a, b in ((e, e), (S - e, e), (S - e, S - e), (e, S - e))]
    return None if any(p is None for p in pts) else np.array(pts)
def star(c):
    X = c[1] - c[0]; Y = c[0] - c[3]; X /= np.linalg.norm(X); Y -= X * (X @ Y); Y /= np.linalg.norm(Y); Z = np.cross(X, Y); o = c.mean(0)   # X right, Y UP (the printed top edge minus the bottom edge), Z = X x Y points out of the surface
    return np.array([o, o + R * X, o - R * X, o + R * Y, o - R * Y, o + R * Z, o - R * Z])
def project(P, cam):
    pos = np.array(cam["pos"]); tgt = np.array(cam["target"]); up = np.array(cam["up"]); z = pos - tgt; z /= np.linalg.norm(z)
    x = np.cross(up, z); x /= np.linalg.norm(x); y = np.cross(z, x); pc = (P - pos) @ np.stack([x, y, z], 1); f = 1.0 / math.tan(math.radians(cam["fov"]) / 2)
    return np.stack([(pc[:, 0] / -pc[:, 2]) * f * (H / 2) + W / 2, H / 2 - (pc[:, 1] / -pc[:, 2]) * f * (H / 2)], 1), pc[:, 2]
parse = lambda fn: tuple(int(v) for v in re.match(r"v(\d+)_c0_(\d+)_(\d+)\.jpg", fn).groups())
errs = []
for sp in ("train", "valid"):
    dd = json.load(open(f"{coco}/{sp}/_annotations.coco.json")) if os.path.exists(f"{coco}/{sp}/_annotations.coco.json") else None
    if dd is None: continue
    imgs = {im["id"]: im for im in dd["images"]}; rng = np.random.RandomState(0); sample = rng.choice(len(dd["annotations"]), min(400, len(dd["annotations"])), replace=False)
    for ai in sample:
        a = dd["annotations"][ai]
        if a["num_keypoints"] != 4 or a["marker_id"] not in mk: continue
        k, x0, y0 = parse(imgs[a["image_id"]]["file_name"]); c = corners3d(mk[a["marker_id"]])
        if c is None or k >= len(cams): continue
        uv, depth = project(c, cams[k]); lab = np.array(a["keypoints"]).reshape(4, 3)[:, :2] + [x0, y0]; errs.append(np.linalg.norm(uv - lab, axis=1).mean())
nout = {m["id"]: np.array(m["n"]) for m in json.load(open(f"{d}/markers_c0.json"))}
zs = []
for i, m in mk.items():
    c = corners3d(m)
    if c is not None and i in nout: zs.append(float(star(c)[5] - star(c)[0]) @ nout[i] / np.linalg.norm(nout[i]) / R) if False else zs.append(float((star(c)[5] - star(c)[0]) @ nout[i] / np.linalg.norm(nout[i]) / R))
zs = np.array(zs); outward = float((zs > 0).mean()); print(f"outward gate: Z agrees with the outward surface normal for {100 * outward:.1f}% of {len(zs)} markers (mean cosine {zs.mean():.2f})"); assert outward > 0.99, "star Z points into the body"
errs = np.array(errs); print(f"frame {W}x{H}; corner gate on {len(errs)} labelled markers: median error {np.median(errs):.2f} px, p90 {np.percentile(errs, 90):.2f}, max {errs.max():.2f}")
ok = len(errs) > 50 and np.median(errs) < 1.5; print("GATE", "PASS" if ok else "FAIL (projection does not reproduce the corner labels; no star labels written)")
if ok:
    for sp in ("train", "valid"):
        dd = json.load(open(f"{coco}/{sp}/_annotations.coco.json")); imgs = {im["id"]: im for im in dd["images"]}; n = 0; skipped = 0
        for a in dd["annotations"]:
            kp = np.array(a["keypoints"]).reshape(4, 3).tolist(); st = [[0.0, 0.0, 0.0]] * 11
            if a["num_keypoints"] == 4 and a["marker_id"] in mk:
                k, x0, y0 = parse(imgs[a["image_id"]]["file_name"]); c = corners3d(mk[a["marker_id"]])
                if c is not None and k < len(cams):
                    cuv, cz = project(c, cams[k]); lab = np.array(a["keypoints"]).reshape(4, 3)[:, :2] + [x0, y0]
                    if np.linalg.norm(cuv - lab, axis=1).mean() < 4.0 and (cz < 0).all():   # the projection must agree with this marker's own corner labels, or it gets no star
                        sq = square3d(mk[a["marker_id"]]); pts3 = star(c) if sq is None else np.vstack([star(c), sq])
                        uv, _ = project(pts3, cams[k]); uv = uv - [x0, y0]; T = imgs[a["image_id"]]["width"]
                        st = [[float(p[0]), float(p[1]), 2.0 if 0 <= p[0] < T and 0 <= p[1] < T else 0.0] for p in uv]; st += [[0.0, 0.0, 0.0]] * (11 - len(st)); n += 1
                    else: skipped += 1
            a["keypoints"] = [v for p in kp + st for v in p]; a["num_keypoints"] = sum(1 for i in range(2, len(a["keypoints"]), 3) if a["keypoints"][i] > 0)
        dd["categories"] = [{"id": 1, "name": "marker", "supercategory": "marker", "keypoints": ["c0", "c1", "c2", "c3", "o", "xp", "xn", "yp", "yn", "zp", "zn", "g0", "g1", "g2", "g3"], "skeleton": [[1, 2], [2, 3], [3, 4], [4, 1], [5, 6], [5, 8], [5, 10], [12, 13], [13, 14], [14, 15], [15, 12]]}]
        os.makedirs(f"{out}/{sp}", exist_ok=True); json.dump(dd, open(f"{out}/{sp}/_annotations.coco.json", "w")); print(sp, "markers with a star:", n, "| left without (projection disagrees with the corner labels):", skipped)
