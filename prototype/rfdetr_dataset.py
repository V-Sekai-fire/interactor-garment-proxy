# Training data for RF-DETR from the marker renders: every visible marker gets a box, 4 corner keypoints (the
# marker's corners in board order, so the corner order carries the orientation), a polygon mask, its id and the share of
# it that is visible. Labels come from the UV passes, so they are exact, not detected: each pixel's UV names the board
# position it shows, the board position names the marker, and a homography board -> screen gives the corners even where
# part of the marker is hidden. Writes COCO JSON (images + annotations; keypoints, segmentation, marker_id, visible_frac)
# with tiles of TILE px cut from the full-resolution renders (a 55 px marker would shrink to about 15 px if the 2000 px
# image were resized to the model's input), and optionally a parquet with the rf-detr-keypoint-data columns.
# python rfdetr_dataset.py <cells dir> <out dir> <name> [tile px=640] [stride=480]   (in the folder with suit_markers.json, boards_meta.json)
# marker_id is the marker's board position (unique); aruco_id = marker_id % 1000 is what the printed suit carries.
import os, sys, json, glob, numpy as np, cv2, cv2.aruco as aruco
cells, out, name = sys.argv[1:4]
TILE = int(sys.argv[4]) if len(sys.argv) > 4 else 640; STRIDE = int(sys.argv[5]) if len(sys.argv) > 5 else 480
META = json.load(open("boards_meta.json"))
suit = json.load(open("suit_markers.json"))    # the real suit board (prototype/make_suit_board.py): every marker by board position idx
quad = {m["idx"]: np.array(m["quad"], np.float32) for m in suit}
lab_all = np.full((65 * 45, 49 * 45), -1, np.int32)
for m in suit: cv2.fillPoly(lab_all, [np.round(quad[m["idx"]]).astype(np.int32)], m["idx"])
label = {mname: lab_all for mname in META}
os.makedirs(f"{out}/{name}", exist_ok=True)
images, anns, aid = [], [], 1
BG = np.array([0.20, 0.20, 0.24]) * 255
def annotate(cell):
    """-> list of dict(id, bbox, kps, poly, vis, px) in full-image pixels"""
    im = cv2.imread(f"{cells}/{cell}.png"); U = cv2.imread(f"{cells}/{cell}_u.png").astype(np.int64); V = cv2.imread(f"{cells}/{cell}_v.png").astype(np.int64)
    M = np.abs(im[..., ::-1].astype(float) - BG).max(axis=2) >= 2; b, r = im[..., 0].astype(int), im[..., 2].astype(int)
    u = (U[..., 2] * 256 + U[..., 1]) / 65535.0; v = (V[..., 2] * 256 + V[..., 1]) / 65535.0
    ids = np.full(M.shape, -1, np.int32); bx = np.zeros(M.shape, np.float32); by = np.zeros(M.shape, np.float32)
    for mname, m in META.items():
        sel = M & ((U[..., 0] < 128) if mname == "body" else (U[..., 0] >= 128))   # mesh id is in the UV pass blue channel
        tx = np.minimum((u[sel] * m["w"]).astype(np.int64), m["w"] - 1); ty = np.minimum((v[sel] * m["h"]).astype(np.int64), m["h"] - 1)
        ids[sel] = label[mname][ty, tx]; bx[sel] = u[sel] * m["w"]; by[sel] = v[sel] * m["h"]
    res = []
    for k in np.unique(ids[ids >= 0]):
        ys, xs = np.nonzero(ids == k)
        if len(xs) < 30: continue
        sel = np.random.default_rng(int(k)).choice(len(xs), min(len(xs), 400), replace=False)
        Hm, _ = cv2.findHomography(np.stack([bx[ys[sel], xs[sel]], by[ys[sel], xs[sel]]], 1), np.stack([xs[sel], ys[sel]], 1).astype(np.float32), cv2.RANSAC, 2.0)
        if Hm is None: continue
        q = cv2.perspectiveTransform(quad[int(k)][None], Hm)[0]
        area = abs(cv2.contourArea(q.astype(np.float32)))
        if area < 20: continue
        kps = []
        for (x, y) in q:
            xi, yi = int(round(x)), int(round(y))
            if 0 <= xi < ids.shape[1] and 0 <= yi < ids.shape[0]: kps += [float(x), float(y), 2 if ids[yi, xi] == k else 1]
            else: kps += [float(x), float(y), 0]
        mk = (ids == k).astype(np.uint8); cs, _ = cv2.findContours(mk, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        poly = max(cs, key=cv2.contourArea).reshape(-1, 2).astype(float) if cs else np.zeros((0, 2))
        res.append(dict(id=int(k), kps=kps, poly=poly, vis=float(min(len(xs) / max(area, 1.0), 1.0)), box=[float(xs.min()), float(ys.min()), float(xs.max() - xs.min() + 1), float(ys.max() - ys.min() + 1)]))
    return im, res
cell_names = sorted({os.path.basename(p)[:-4] for p in glob.glob(f"{cells}/v*_c?.png")}, key=lambda s: (int(s.split("_")[0][1:]), s))
iid = 1
for cell in cell_names:
    im, res = annotate(cell); H, W = im.shape[:2]
    for y0 in range(0, max(H - TILE, 0) + STRIDE, STRIDE):
        for x0 in range(0, max(W - TILE, 0) + STRIDE, STRIDE):
            x0c, y0c = min(x0, W - TILE), min(y0, H - TILE); tile_anns = []
            for a in res:
                kp = np.array(a["kps"]).reshape(4, 3).copy(); kp[:, 0] -= x0c; kp[:, 1] -= y0c
                inside = (kp[:, 0] >= 0) & (kp[:, 0] < TILE) & (kp[:, 1] >= 0) & (kp[:, 1] < TILE)
                if inside.sum() < 3: continue          # keep markers that are mostly in this tile
                kp[~inside, 2] = 0
                x, y, w, h = a["box"]; bx0, by0 = max(x - x0c, 0), max(y - y0c, 0); bx1, by1 = min(x + w - x0c, TILE), min(y + h - y0c, TILE)
                if bx1 - bx0 < 4 or by1 - by0 < 4: continue
                p = a["poly"] - [x0c, y0c]; p = np.clip(p, 0, TILE)
                tile_anns.append(dict(id=aid, image_id=iid, category_id=1, bbox=[bx0, by0, bx1 - bx0, by1 - by0], area=float((bx1 - bx0) * (by1 - by0)), iscrowd=0, keypoints=[round(float(v), 2) for v in kp.flatten()], num_keypoints=int((kp[:, 2] > 0).sum()),
                                      segmentation=[[round(float(v), 1) for v in p.flatten()]] if len(p) >= 3 else [], marker_id=a["id"], aruco_id=a["id"] % 1000, visible_frac=round(a["vis"], 3))); aid += 1
            if not tile_anns and np.random.default_rng(iid).random() > 0.1: continue   # keep 10% of empty tiles as negatives
            fn = f"{cell}_{x0c}_{y0c}.jpg"; cv2.imwrite(f"{out}/{name}/{fn}", im[y0c:y0c + TILE, x0c:x0c + TILE], [cv2.IMWRITE_JPEG_QUALITY, 95])
            images.append(dict(id=iid, file_name=fn, width=TILE, height=TILE, cell=cell)); anns += tile_anns; iid += 1
cats = [dict(id=1, name="marker", supercategory="marker", keypoints=["c0", "c1", "c2", "c3"], skeleton=[[1, 2], [2, 3], [3, 4], [4, 1]])]
json.dump(dict(images=images, annotations=anns, categories=cats), open(f"{out}/{name}/_annotations.coco.json", "w"))
print(f"{name}: {len(cell_names)} renders -> {len(images)} tiles, {len(anns)} marker labels; mean visible share {np.mean([a['visible_frac'] for a in anns]):.2f}; keypoints with v=2: {100.0 * np.mean([a['keypoints'][2::3].count(2) / 4 for a in anns]):.0f}%")
