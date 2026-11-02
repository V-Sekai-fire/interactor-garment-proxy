# Which detector setting recovers the most markers we KNOW are there? Ground truth: markers that are large (>= MINPX screen pixels) and wholly
# visible, from the UV passes (largest connected piece per marker idx). Each variant reads the full render; a marker counts when its id comes back.
# python detector_battery.py <cells dir> [N cameras = 12] [MINPX = 1500]      (run where suit_markers.json is)
import sys as _sys, os as _os; _sys.path.insert(0, _os.path.dirname(_os.path.abspath(__file__))); import tagfamily
import os, sys, json, glob, numpy as np, cv2, cv2.aruco as aruco
folder = sys.argv[1]; NCAM = int(sys.argv[2]) if len(sys.argv) > 2 else 12; MINPX = int(sys.argv[3]) if len(sys.argv) > 3 else 1500
d = tagfamily.load_dictionary(); suit = json.load(open("suit_markers.json")); W, H = tagfamily.board_size()[0], tagfamily.board_size()[1]
lab_all = np.full((H, W), -1, np.int32); quad = {}; ids_of = {}
for m in suit: q = np.array(m["quad"], np.float32); quad[m["idx"]] = q; ids_of[m["idx"]] = m["id"]; cv2.fillPoly(lab_all, [np.round(q).astype(np.int32)], m["idx"])
def P(**kw):
    p = aruco.DetectorParameters()
    for k, v in kw.items(): setattr(p, k, v)
    return p
V = {"default": (P(), None),
     "ArUco3 detection": (P(useAruco3Detection=True, minSideLengthCanonicalImg=16, minMarkerLengthRatioOriginalImg=0.0), None),
     "AprilTag corner refinement": (P(cornerRefinementMethod=aruco.CORNER_REFINE_APRILTAG), None),
     "wider adaptive window": (P(adaptiveThreshWinSizeMin=3, adaptiveThreshWinSizeMax=103, adaptiveThreshWinSizeStep=10), None),
     "looser polygon, borders and bits": (P(polygonalApproxAccuracyRate=0.06, maxErroneousBitsInBorderRate=0.6, errorCorrectionRate=1.0, minMarkerPerimeterRate=0.01), None),
     "CLAHE contrast": (P(), "clahe"), "upscale x1.5": (P(), "up"),
     "all recovery settings": (P(useAruco3Detection=True, minSideLengthCanonicalImg=16, minMarkerLengthRatioOriginalImg=0.0, cornerRefinementMethod=aruco.CORNER_REFINE_APRILTAG, adaptiveThreshWinSizeMin=3, adaptiveThreshWinSizeMax=103, adaptiveThreshWinSizeStep=10, polygonalApproxAccuracyRate=0.06, maxErroneousBitsInBorderRate=0.6, errorCorrectionRate=1.0), None)}
cams = sorted({int(os.path.basename(p).split("_")[0][1:]) for p in glob.glob(f"{folder}/v*_c0.png")})[:NCAM]
hit = {k: 0 for k in V}; wrong = {k: 0 for k in V}; gt_total = 0
for v in cams:
    im = cv2.imread(f"{folder}/v{v}_c0.png"); U = cv2.imread(f"{folder}/v{v}_c0_u.png").astype(np.int64); Vv = cv2.imread(f"{folder}/v{v}_c0_v.png").astype(np.int64)
    bg = np.array([0.20, 0.20, 0.24]) * 255; M = np.abs(im[..., ::-1].astype(float) - bg).max(axis=2) >= 2
    u = (U[..., 2] * 256 + U[..., 1]) / 65535.0; vv = (Vv[..., 2] * 256 + Vv[..., 1]) / 65535.0
    ys, xs = np.nonzero(M); tx = np.minimum((u[M] * W).astype(np.int64), W - 1); ty = np.minimum((vv[M] * H).astype(np.int64), H - 1); lab = lab_all[ty, tx]; ok = lab >= 0
    gt = set()
    for k in np.unique(lab[ok]):
        s0 = ok & (lab == k)
        if int(s0.sum()) < MINPX: continue
        y0_, x0_ = ys[s0], xs[s0]; mk = np.zeros((y0_.max() - y0_.min() + 3, x0_.max() - x0_.min() + 3), np.uint8); mk[y0_ - y0_.min() + 1, x0_ - x0_.min() + 1] = 1
        nc, cc = cv2.connectedComponents(mk, connectivity=8)
        keep = (cc[y0_ - y0_.min() + 1, x0_ - x0_.min() + 1] == (1 + int(np.argmax(np.bincount(cc.ravel())[1:])))) if nc > 2 else np.ones(len(y0_), bool)
        if int(keep.sum()) < MINPX: continue
        bx, by = tx[s0][keep], ty[s0][keep]
        if (bx.max() - bx.min()) >= 0.9 * np.ptp(quad[int(k)][:, 0]) and (by.max() - by.min()) >= 0.9 * np.ptp(quad[int(k)][:, 1]): gt.add(ids_of[int(k)])
    gt_total += len(gt); g = cv2.cvtColor(im, cv2.COLOR_BGR2GRAY)
    for name, (prm, pre) in V.items():
        img = g
        if pre == "clahe": img = cv2.createCLAHE(clipLimit=3.0, tileGridSize=(8, 8)).apply(g)
        if pre == "up": img = cv2.resize(g, None, fx=1.5, fy=1.5, interpolation=cv2.INTER_CUBIC)
        _, ids, _ = aruco.ArucoDetector(d, prm).detectMarkers(img)[:3] if False else (None, aruco.ArucoDetector(d, prm).detectMarkers(img)[1], None)
        got = set() if ids is None else {int(i) for i in ids.flatten()}
        hit[name] += len(got & gt); wrong[name] += len(got - gt)
print(f"{len(cams)} cameras, {gt_total} known large, wholly visible markers")
for name in V: print(f"{name:34s} recovered {hit[name]:5d} ({100.0 * hit[name] / max(gt_total, 1):5.1f}%)   reads of ids not in the known set: {wrong[name]}")
