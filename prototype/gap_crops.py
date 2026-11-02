# Crops of markers that are well seen in 2+ cameras yet read in fewer than 2: the best view of each, as a contact sheet, to see why they fail.
# python gap_crops.py <render folder> <dir with suit_markers.json, reads.json> <out.png> [count = 30]
import sys, glob, os, json, numpy as np, cv2
folder, d, out = sys.argv[1:4]; N = int(sys.argv[4]) if len(sys.argv) > 4 else 30
mk = {m["idx"]: m for m in json.load(open(f"{d}/suit_markers.json"))}; r = json.load(open(f"{d}/reads.json"))["0:body"]
rd = {int(k): v for k, v in r["reads"].items()}; wl = {int(k): v for k, v in r["well"].items()}
gap = [i for i in r["on"] if wl.get(i, 0) >= 2 and rd.get(i, 0) < 2]; rng = np.random.RandomState(1); rng.shuffle(gap); gap = gap[:N]
bd = json.load(open(f"{d}/boards_meta.json")); W, H = [(v["w"], v["h"]) for v in bd.values()][0] if isinstance(bd, dict) and "w" not in bd else (bd["w"], bd["h"])
lab = np.full((H, W), -1, np.int32)
for i in gap: cv2.fillPoly(lab, [np.round(np.array(mk[i]["quad"], np.float32)).astype(np.int32)], i)
best = {i: (0, None) for i in gap}
for p in sorted(glob.glob(f"{folder}/v*_c0.png")):
    if p.endswith("_u.png") or p.endswith("_v.png"): continue
    U = cv2.imread(p[:-4] + "_u.png").astype(np.int64); Vv = cv2.imread(p[:-4] + "_v.png").astype(np.int64)
    u = (U[..., 2] * 256 + U[..., 1]) / 65535.0; v = (Vv[..., 2] * 256 + Vv[..., 1]) / 65535.0
    ok = U[..., 0] < 128; ys, xs = np.nonzero(ok)
    l = lab[np.minimum((v[ok] * H).astype(np.int64), H - 1), np.minimum((u[ok] * W).astype(np.int64), W - 1)]
    for i in np.unique(l[l >= 0]):
        n = int((l == i).sum())
        if n > best[int(i)][0]: sel = l == i; best[int(i)] = (n, (p, xs[sel].min(), xs[sel].max(), ys[sel].min(), ys[sel].max()))
tiles = []
for i in gap:
    n, b = best[i]
    if b is None: continue
    im = cv2.imread(b[0]); x0, x1, y0, y1 = b[1:]; cx, cy = (x0 + x1) // 2, (y0 + y1) // 2; h = max(x1 - x0, y1 - y0) // 2 + 20
    c = cv2.copyMakeBorder(im, h, h, h, h, cv2.BORDER_CONSTANT); c = c[cy:cy + 2 * h, cx:cx + 2 * h]; c = cv2.resize(c, (200, 200), interpolation=cv2.INTER_CUBIC)
    cv2.putText(c, f"{i} {n}px", (4, 16), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1); tiles.append(c)
cols = 6; rows = [np.hstack(tiles[k:k + cols] + [np.zeros_like(tiles[0])] * (cols - len(tiles[k:k + cols]))) for k in range(0, len(tiles), cols)]
cv2.imwrite(out, np.vstack(rows)); print("tiles", len(tiles)); json.dump({str(i): best[i][1][0] for i in gap if best[i][1]}, open(out + ".json", "w"))
