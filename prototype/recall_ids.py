# How many markers are well seen, and how many of those the detector reads (recall). A marker is well seen in a
# view when at least MIN_PX screen pixels show it and those pixels cover at least 85% of its extent on the board in
# both directions (a seam or a silhouette cuts the rest). Pixels are mapped to markers through the UV passes.
# python prototype/recall_ids.py <cells dir> [min px]   (run in the folder with suit_markers.json and boards_meta.json)
import sys as _sys, os as _os; _sys.path.insert(0, _os.path.dirname(_os.path.abspath(__file__))); import tagfamily
import os, sys, json, glob, numpy as np, cv2, cv2.aruco as aruco
folder = sys.argv[1]
MIN_PX = int(sys.argv[2]) if len(sys.argv) > 2 else 500
d = tagfamily.load_dictionary()
det = aruco.ArucoDetector(d, aruco.DetectorParameters())
META = json.load(open("boards_meta.json"))
suit = json.load(open("suit_markers.json"))
cand = {}                                   # detected id -> board idx candidates (ids repeat past 1000)
extent = {}
lab_all = np.full((tagfamily.board_size()[1], tagfamily.board_size()[0]), -1, np.int32)
for m in suit:
    q = np.array(m["quad"]); cv2.fillPoly(lab_all, [np.round(q).astype(np.int32)], m["idx"])
    extent[m["idx"]] = (np.ptp(q[:, 0]), np.ptp(q[:, 1])); cand.setdefault(m["id"], []).append(m["idx"])
label = {name: lab_all for name in META}
def meshes_of(i):
    return [n for n, m in META.items() if m["first"] <= i < m["first"] + m["count"]]
def mesh_of(i):
    for n, m in META.items():
        if m["first"] <= i < m["first"] + m["count"]: return n
tot = {}
union = {}
reads = {}
wellc = {}
NV = max(len(glob.glob(f"{folder}/v*_c0.png")), len(glob.glob(f"{folder}/v*_c1.png")))
print("cameras:", NV)
print("view col  mesh   well-seen  read(of well-seen)  recall   read-but-not-well-seen")
for v in range(NV):
    for c in range(2):
        if not os.path.exists(f"{folder}/v{v}_c{c}.png"): continue
        im = cv2.imread(f"{folder}/v{v}_c{c}.png")
        U = cv2.imread(f"{folder}/v{v}_c{c}_u.png").astype(np.int64)
        V = cv2.imread(f"{folder}/v{v}_c{c}_v.png").astype(np.int64)
        bg = np.array([0.20, 0.20, 0.24]) * 255
        M = np.abs(im[..., ::-1].astype(float) - bg).max(axis=2) >= 2
        b, r = im[..., 0].astype(int), im[..., 2].astype(int)
        cls = {"body": M & (U[..., 0] < 128), "skirt": M & (U[..., 0] >= 128)}   # mesh id is in the UV pass blue channel
        _, ids, _, _ = det.detectMarkersWithConfidence(cv2.cvtColor(im, cv2.COLOR_BGR2GRAY))
        read = set() if ids is None else set(int(x) for x in ids.flatten())
        u = (U[..., 2] * 256 + U[..., 1]) / 65535.0
        vv = (V[..., 2] * 256 + V[..., 1]) / 65535.0
        for name, m in META.items():
            sel = cls[name]
            tx = np.minimum((u[sel] * m["w"]).astype(np.int64), m["w"] - 1)
            ty = np.minimum((vv[sel] * m["h"]).astype(np.int64), m["h"] - 1)
            lab = label[name][ty, tx]
            ok = lab >= 0
            lab, tx, ty = lab[ok], tx[ok], ty[ok]
            present = set(int(x) for x in np.unique(lab))
            well = set()
            if lab.size:
                order = np.argsort(lab, kind="stable")
                lab, tx, ty = lab[order], tx[order], ty[order]
                keys, start, cnt = np.unique(lab, return_index=True, return_counts=True)
                xmin = np.minimum.reduceat(tx, start); xmax = np.maximum.reduceat(tx, start)
                ymin = np.minimum.reduceat(ty, start); ymax = np.maximum.reduceat(ty, start)
                for k, n, x0, x1, y0, y1 in zip(keys, cnt, xmin, xmax, ymin, ymax):
                    ex, ey = extent[int(k)]
                    if n >= MIN_PX and (x1 - x0) >= 0.85 * ex and (y1 - y0) >= 0.85 * ey:
                        well.add(int(k))
            mine = set()
            for k in read:
                for idx in cand.get(k, []):
                    if name in meshes_of(idx) and idx in present: mine.add(idx)
            hit = mine & well
            wcnt = wellc.setdefault((c, name), {})
            for k_ in well: wcnt[k_] = wcnt.get(k_, 0) + 1   # in how many cameras this marker is well seen (ground truth)
            t = tot.setdefault((c, name), [0, 0, 0, 0]); t[0] += len(well); t[1] += len(hit); t[2] += len(mine - well); t[3] += len(mine)
            rc_ = reads.setdefault((c, name), {})
            for idx_ in mine: rc_[idx_] = rc_.get(idx_, 0) + 1   # in how many cameras this marker was read correctly
            u_ = union.setdefault((c, name), [set(), set(), set()]); u_[0] |= set(int(k) for k in np.unique(lab)) if lab.size else set(); u_[1] |= well; u_[2] |= mine
for (c, name), t in sorted(tot.items()):
    rc = 100.0 * t[1] / t[0] if t[0] else float("nan")
    print(f"PER-CAMERA {'input ' if c == 0 else 'fitted'} {name:5s} well-seen {t[0]} read {t[1]} recall {rc:.1f}%  read-but-not-well-seen {t[2]}")
for (c, name), (on, well, rd) in sorted(union.items()):
    rcount = reads.get((c, name), {})
    two = sum(1 for k in on if rcount.get(k, 0) >= 2); one = sum(1 for k in on if rcount.get(k, 0) >= 1); wl = sum(1 for k in well if rcount.get(k, 0) >= 2)
    print(f"TRACKED {'input ' if c == 0 else 'fitted'} {name:5s} markers on the mesh {len(on)}: read in 1+ cameras {one} ({100.0 * one / max(len(on), 1):.1f}%), in 2+ cameras (can be triangulated) {two} ({100.0 * two / max(len(on), 1):.1f}%); of the {len(well)} well seen by some camera, 2+ reads: {wl} ({100.0 * wl / max(len(well), 1):.1f}%)")
    print(f"ALL CAMERAS {'input ' if c == 0 else 'fitted'} {name:5s} markers on the mesh {len(on)}, read by some camera {len(rd & on)} ({100.0 * len(rd & on) / max(len(on), 1):.1f}%); well seen by some camera {len(well)}, of those read {len(rd & well)} ({100.0 * len(rd & well) / max(len(well), 1):.1f}%)")

import json as _json   # per-marker read counts, for the neighbour-prediction check
_json.dump({f"{c}:{name}": {"on": sorted(int(k) for k in on), "reads": {str(k): int(v) for k, v in reads.get((c, name), {}).items()}, "well": {str(k): int(v) for k, v in wellc.get((c, name), {}).items()}} for (c, name), (on, well, rd) in union.items()}, open("reads.json", "w"))
