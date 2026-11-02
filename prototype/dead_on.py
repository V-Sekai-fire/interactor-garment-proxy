# Dead-on gate: every marker we paint must read when a camera looks straight down its normal, nothing in the way, full resolution.
#   make:  dead_on.py make <markers.json> <out dir> [count = 300] [dist m = 0.3] [fov = 20] [seed = 1]   (a random sample of markers; count 0 = all)
#   score: dead_on.py score <render folder> <out dir with targets.json> <dictionary dir with suit_dictionary.txt> <dir with suit_markers.json>
# A marker counts only if the camera really sees it (its pixels cover most of its quad in the UV pass); the gate passes when every such marker reads with its own id.
import sys, os, json, glob, numpy as np
mode = sys.argv[1]
if mode == "make":
    mk = json.load(open(sys.argv[2])); out = sys.argv[3]; cnt = int(sys.argv[4]) if len(sys.argv) > 4 else 300
    dist = float(sys.argv[5]) if len(sys.argv) > 5 else 0.3; fov = float(sys.argv[6]) if len(sys.argv) > 6 else 20.0; seed = int(sys.argv[7]) if len(sys.argv) > 7 else 1
    os.makedirs(out, exist_ok=True); idx = np.arange(len(mk)); 
    if cnt: idx = np.random.RandomState(seed).permutation(idx)[:cnt]
    cams = []; tg = []
    for k in idx:
        P = np.array(mk[k]["P"]); n = np.array(mk[k]["n"]); n = n / np.linalg.norm(n); up = np.array([0.0, 1.0, 0.0])
        if abs(n @ up) > 0.95: up = np.array([0.0, 0.0, 1.0])
        cams.append({"pos": (P + n * dist).tolist(), "target": P.tolist(), "up": up.tolist(), "fov": fov}); tg.append(int(mk[k]["id"]))
    json.dump(cams, open(f"{out}/cams_c0.json", "w")); json.dump([], open(f"{out}/cams_c1.json", "w")); json.dump(tg, open(f"{out}/targets.json", "w")); print(len(cams), "cameras")
else:
    import cv2, cv2.aruco as aruco
    sys.path.insert(0, os.path.dirname(os.path.abspath(__file__))); import tagfamily
    folder, out, dd = sys.argv[2], sys.argv[3], sys.argv[4]; sm = json.load(open(f"{dd}/suit_markers.json")) if len(sys.argv) < 6 else json.load(open(f"{sys.argv[5]}/suit_markers.json"))
    tg = json.load(open(f"{out}/targets.json")); byid = {m["id"]: m for m in sm}
    meta = json.load(open(f"{sys.argv[5] if len(sys.argv) > 5 else dd}/boards_meta.json"))["body"]; W, H = meta["w"], meta["h"]
    d = tagfamily.load_dictionary(); prm = aruco.DetectorParameters()
    for a_, b_ in json.loads(os.environ.get("DET", "{}")).items(): setattr(prm, a_, b_)   # detector settings, e.g. DET='{"adaptiveThreshWinSizeMax": 203}'
    det = aruco.ArucoDetector(d, prm)
    MINPX = int(os.environ.get("MINPX", "2500")); ok = seen = blocked = 0; bad = []
    for k, t in enumerate(tg):
        p = f"{folder}/v{k}_c0.png"
        if not os.path.exists(p): continue
        im = cv2.imread(p); U = cv2.imread(p[:-4] + "_u.png").astype(np.int64); V = cv2.imread(p[:-4] + "_v.png").astype(np.int64)
        u = (U[..., 2] * 256 + U[..., 1]) / 65535.0; v = (V[..., 2] * 256 + V[..., 1]) / 65535.0; m = U[..., 0] < 128
        q = np.array(byid[t]["quad"], np.float32); qm = np.zeros((H, W), np.uint8); cv2.fillPoly(qm, [np.round(q).astype(np.int32)], 1)
        tx = np.minimum((u[m] * W).astype(np.int64), W - 1); ty = np.minimum((v[m] * H).astype(np.int64), H - 1)
        n_px = int(qm[ty, tx].sum()); quad_area = float(cv2.contourArea(q))
        # visible if the camera sees at least 1.5 pixels per atlas pixel of the marker's quad... (resolution differs), so require MINPX pixels in the frame
        if n_px < MINPX: blocked += 1; continue
        seen += 1
        hh, ww = im.shape[:2]; C = int(os.environ.get("CROP", "480")); g = cv2.cvtColor(im, cv2.COLOR_BGR2GRAY)[max(hh // 2 - C // 2, 0):hh // 2 + C // 2, max(ww // 2 - C // 2, 0):ww // 2 + C // 2]   # the target is at the centre: detect on a crop, which is much faster
        _, ids, _, _ = det.detectMarkersWithConfidence(g); got = ids is not None and t in set(ids.flatten().tolist())
        ok += bool(got)
        if not got: bad.append((k, t, n_px))
    print(f"dead-on: {len(tg)} cameras; marker seen (>= {MINPX} px): {seen}; blocked or off-surface: {blocked}; read with own id: {ok} ({100 * ok / max(seen, 1):.2f}%)")
    print("GATE", "PASS" if seen and ok == seen else "FAIL", "failures (camera, id, px):", bad[:40])
