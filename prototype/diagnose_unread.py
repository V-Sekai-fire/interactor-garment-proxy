# Why are well-seen markers not read? Lists markers that are large (at least MINPX screen pixels), fully visible (their pixels cover
# the whole marker on the board) and still unread, and cuts out a crop of each with the ground-truth outline drawn (green = read,
# red = unread), so a placement bug (mirroring, bleeding, wrong squares) can be seen by eye.
# python diagnose_unread.py <cells dir> <out.png> [MINPX=1500] [max crops=24] [first N cameras=40]   (run where suit_markers.json is)
import os, sys, json, glob, numpy as np, cv2, cv2.aruco as aruco
folder, out = sys.argv[1], sys.argv[2]
MINPX = int(sys.argv[3]) if len(sys.argv) > 3 else 1500; MAXC = int(sys.argv[4]) if len(sys.argv) > 4 else 24; NCAM = int(sys.argv[5]) if len(sys.argv) > 5 else 40
d = aruco.getPredefinedDictionary(getattr(aruco, open("suit_dictionary.txt").read().strip()) if os.path.exists("suit_dictionary.txt") else aruco.DICT_4X4_1000); det = aruco.ArucoDetector(d, aruco.DetectorParameters())
suit = json.load(open("suit_markers.json")); W, H = 49 * 45, 65 * 45
lab_all = np.full((H, W), -1, np.int32); quad = {}; ids_of = {}
for m in suit: q = np.array(m["quad"], np.float32); quad[m["idx"]] = q; ids_of[m["idx"]] = m["id"]; cv2.fillPoly(lab_all, [np.round(q).astype(np.int32)], m["idx"])
cams = sorted({int(os.path.basename(p).split("_")[0][1:]) for p in glob.glob(f"{folder}/v*_c0.png")})[:NCAM]
audit = []; verdicts = {}; tot = unread = 0; crops = []; reasons = {"read": 0, "unread, upright and square": 0}
for v in cams:
    im = cv2.imread(f"{folder}/v{v}_c0.png"); U = cv2.imread(f"{folder}/v{v}_c0_u.png").astype(np.int64); Vv = cv2.imread(f"{folder}/v{v}_c0_v.png").astype(np.int64)
    bg = np.array([0.20, 0.20, 0.24]) * 255; M = np.abs(im[..., ::-1].astype(float) - bg).max(axis=2) >= 2
    u = (U[..., 2] * 256 + U[..., 1]) / 65535.0; vv = (Vv[..., 2] * 256 + Vv[..., 1]) / 65535.0
    ys, xs = np.nonzero(M); tx = np.minimum((u[M] * W).astype(np.int64), W - 1); ty = np.minimum((vv[M] * H).astype(np.int64), H - 1)
    lab = lab_all[ty, tx]; ok = lab >= 0
    corners, rid, _, _ = det.detectMarkersWithConfidence(cv2.cvtColor(im, cv2.COLOR_BGR2GRAY)); read = {} if rid is None else {int(i): c.reshape(4, 2) for c, i in zip(corners, rid.flatten())}
    for k in np.unique(lab[ok]):
        sel0 = ok & (lab == k)
        if int(sel0.sum()) < MINPX: continue
        # the board repeats (instances), so one idx can show at several places on the body: keep the largest connected piece in the image
        ys0, xs0 = ys[sel0], xs[sel0]; x0_, y0_ = xs0.min(), ys0.min(); mk_ = np.zeros((ys0.max() - y0_ + 3, xs0.max() - x0_ + 3), np.uint8); mk_[ys0 - y0_ + 1, xs0 - x0_ + 1] = 1
        nc, cc = cv2.connectedComponents(mk_, connectivity=8)
        if nc > 2: big = 1 + int(np.argmax(np.bincount(cc.ravel())[1:])); keep = cc[ys0 - y0_ + 1, xs0 - x0_ + 1] == big
        else: keep = np.ones(len(xs0), bool)
        sel = sel0.copy(); sel[np.nonzero(sel0)[0]] = keep if False else sel[np.nonzero(sel0)[0]]
        idxs = np.nonzero(sel0)[0]; sel = np.zeros_like(sel0); sel[idxs[keep]] = True; n = int(sel.sum())
        if n < MINPX: continue
        px, py = xs[sel], ys[sel]; bxs, bys = tx[sel], ty[sel]; q = quad[int(k)]
        if (bxs.max() - bxs.min()) < 0.9 * np.ptp(q[:, 0]) or (bys.max() - bys.min()) < 0.9 * np.ptp(q[:, 1]): continue    # not wholly visible
        tot += 1; isread = ids_of[int(k)] in read
        Hq, _ = cv2.findHomography(np.stack([u[M][sel] * W, vv[M][sel] * H], 1).astype(np.float32), np.stack([px, py], 1).astype(np.float32), cv2.RANSAC, 2.0)
        verdict = "no homography"
        if Hq is not None:
            qq = cv2.perspectiveTransform(q[None], Hq)[0]; sq = np.array([[40, 40], [200, 40], [200, 200], [40, 200]], np.float32)   # marker corners -> a 240 px patch with a white margin
            patch = cv2.warpPerspective(cv2.cvtColor(im, cv2.COLOR_BGR2GRAY), cv2.getPerspectiveTransform(qq.astype(np.float32), sq), (240, 240), flags=cv2.INTER_AREA, borderValue=255)
            def has(img): return aruco.ArucoDetector(d, aruco.DetectorParameters()).detectMarkers(img)[1] is not None
            # what marker is actually on the surface here? read the 6x6 cell grid of the rectified patch and find the closest dictionary code
            g = cv2.threshold(patch, 128, 1, cv2.THRESH_BINARY)[1]; cell = 160 // 6; bits = np.zeros((6, 6), np.uint8)
            for r_ in range(6):
                for c_ in range(6): bits[r_, c_] = np.median(g[40 + r_ * 160 // 6 + 3: 40 + (r_ + 1) * 160 // 6 - 3, 40 + c_ * 160 // 6 + 3: 40 + (c_ + 1) * 160 // 6 - 3]) > 0
            inner = bits[1:5, 1:5]; border_black = (bits[0].sum() + bits[5].sum() + bits[:, 0].sum() + bits[:, 5].sum()) == 0
            best = (99, -1)
            for i_ in range(1000):
                code = np.unpackbits(d.bytesList[i_][0].astype(np.uint8))[:0]
                m_ = cv2.aruco.generateImageMarker(d, i_, 6, borderBits=1) // 255 if False else None
            if border_black:
                from_dict = [cv2.aruco.generateImageMarker(d, i_, 6) // 255 for i_ in range(1000)]
                for i_, mm_ in enumerate(from_dict):
                    for rot in range(4):
                        dist_ = int((np.rot90(mm_[1:5, 1:5], rot) != inner).sum())
                        if dist_ < best[0]: best = (dist_, i_)
                if not isread: audit.append((int(k), ids_of[int(k)], best[1], best[0], border_black))
            elif not isread: audit.append((int(k), ids_of[int(k)], -1, 99, border_black))
            verdict = "reads when rectified" if has(patch) else ("reads MIRRORED" if has(cv2.flip(patch, 1)) or has(cv2.flip(patch, 0)) else "not a valid marker either way")
        verdicts[verdict] = verdicts.get(verdict, 0) + (0 if isread else 1)
        if isread: reasons["read"] += 1; continue
        unread += 1; reasons["unread, upright and square"] += 1
        if len(crops) < MAXC:
            Hm, _ = cv2.findHomography(np.stack([u[M][sel] * W, vv[M][sel] * H], 1).astype(np.float32), np.stack([px, py], 1).astype(np.float32), cv2.RANSAC, 2.0)
            if Hm is None: continue
            qs = cv2.perspectiveTransform(q[None], Hm)[0]; c = qs.mean(0); r = int(max(np.ptp(qs[:, 0]), np.ptp(qs[:, 1])) * 1.0) + 24
            x0, y0 = int(np.clip(c[0] - r, 0, im.shape[1] - 2 * r)), int(np.clip(c[1] - r, 0, im.shape[0] - 2 * r)); cr = im[y0:y0 + 2 * r, x0:x0 + 2 * r].copy()
            if cr.size == 0: continue
            cv2.polylines(cr, [np.round(qs - [x0, y0]).astype(np.int32)], True, (0, 0, 255), 2); cr = cv2.resize(cr, (260, 260)); cv2.putText(cr, f"v{v} idx{int(k)} id{ids_of[int(k)]} px{n}", (4, 16), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 255, 255), 1); crops.append(cr)
print(f"{len(cams)} cameras: {tot} large, wholly visible markers; read {reasons['read']} ({100.0 * reasons['read'] / max(tot, 1):.1f}%), unread {unread}")
if crops:
    rows = [np.hstack(crops[i:i + 6] + [np.zeros((260, 260, 3), np.uint8)] * (6 - len(crops[i:i + 6]))) for i in range(0, len(crops), 6)]; cv2.imwrite(out, np.vstack(rows)); print("unread markers, after rectifying each one:", verdicts); print("crops:", len(crops), "->", out)
ok_b = [a_ for a_ in audit if a_[4]]; same = [a_ for a_ in ok_b if a_[2] == a_[1]]; near = [a_ for a_ in ok_b if a_[3] <= 2]
print(f"of {len(audit)} unread markers: {len(ok_b)} have a black border; {len(near)} of those match some dictionary code within 2 bits; {len(same)} are the code the board says is there (idx < 1000: {sum(1 for a_ in same if a_[0] < 1000)}, idx >= 1000: {sum(1 for a_ in same if a_[0] >= 1000)}); other code: {len(near) - len(same)}")
print("examples (idx, expected id, code found, bit distance):", [a_[:4] for a_ in audit[:8]])
