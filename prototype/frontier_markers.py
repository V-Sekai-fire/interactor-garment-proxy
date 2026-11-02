# Marker frontier: detector variants x input families, scored against the UV-pass ground truth, with planted controls (like frontier.py).
# python frontier_markers.py <render folder> <dir with suit_markers.json, boards_meta.json, suit_dictionary.txt (cwd must hold it)> <families.json> <out.json> [frames per family = 10] [variants = all]
# Families come from the cameras (distance, tilt). A marker is "well seen" when its pixels are at least MINPX and inside the frame. Per cell: read share of the
# well-seen markers, wrong reads (a read id that is not on screen), and a control (the same frames flipped must read nothing).
import sys, os, json, glob, time, numpy as np, cv2, cv2.aruco as aruco
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__))); import tagfamily
folder, d, famf, out = sys.argv[1:5]; NF = int(sys.argv[5]) if len(sys.argv) > 5 else 10; MINPX = 400
fam = json.load(open(famf)); meta = json.load(open(f"{d}/boards_meta.json"))["body"]; W, H = meta["w"], meta["h"]
mk = json.load(open(f"{d}/suit_markers.json")); lab = np.full((H, W), -1, np.int32)
for m in mk: cv2.fillPoly(lab, [np.round(np.array(m["quad"], np.float32)).astype(np.int32)], m["idx"])
dic = tagfamily.load_dictionary()
def P(**kw):
    p = aruco.DetectorParameters()
    for a, b in kw.items(): setattr(p, a, b)
    return p
VARIANTS = {"default": P(), "window 3-103": P(adaptiveThreshWinSizeMax=103, adaptiveThreshWinSizeStep=10), "window 3-203": P(adaptiveThreshWinSizeMax=203, adaptiveThreshWinSizeStep=20),
            "aruco3": P(useAruco3Detection=True), "window 3-203 + const 3": P(adaptiveThreshWinSizeMax=203, adaptiveThreshWinSizeStep=20, adaptiveThreshConstant=3)}
byfam = {}
for k, f in enumerate(fam): byfam.setdefault(f, []).append(k)
rng = np.random.RandomState(0); pick = {f: sorted(rng.choice(v, min(NF, len(v)), replace=False).tolist()) for f, v in byfam.items()}
def truth(k):
    p = f"{folder}/v{k}_c0.png"; im = cv2.imread(p); U = cv2.imread(p[:-4] + "_u.png").astype(np.int64); V = cv2.imread(p[:-4] + "_v.png").astype(np.int64)
    u = (U[..., 2] * 256 + U[..., 1]) / 65535.0; v = (V[..., 2] * 256 + V[..., 1]) / 65535.0; m = U[..., 0] < 128; ys, xs = np.nonzero(m)
    l = lab[np.minimum((v[m] * H).astype(np.int64), H - 1), np.minimum((u[m] * W).astype(np.int64), W - 1)]; h, w = m.shape; ok = l >= 0
    present = set(int(i) for i in np.unique(l[ok])); well = set()
    for i in present:
        s = l == i; n = int(s.sum())
        if n >= MINPX and xs[s].min() > 5 and ys[s].min() > 5 and xs[s].max() < w - 6 and ys[s].max() < h - 6: well.add(i)
    return cv2.cvtColor(im, cv2.COLOR_BGR2GRAY), present, well
res = {}; t0 = time.time(); total = sum(len(v) for v in pick.values()); done = 0
for f, ks in pick.items():
    for k in ks:
        g, present, well = truth(k); done += 1
        for vn, prm in VARIANTS.items():
            det = aruco.ArucoDetector(dic, prm); _, ids, _, _ = det.detectMarkersWithConfidence(g); got = set() if ids is None else set(int(x) for x in ids.flatten())
            _, ids2, _, _ = det.detectMarkersWithConfidence(g[:, ::-1].copy()); got2 = set() if ids2 is None else set(int(x) for x in ids2.flatten())
            c = res.setdefault((vn, f), [0, 0, 0, 0, 0]); c[0] += len(well); c[1] += len(well & got); c[2] += len(got - present); c[3] += len(got2 & present); c[4] += 1
        print(f"frame {done}/{total} {f} k={k} well {len(well)} elapsed {time.time() - t0:.0f}s", flush=True)
fams = sorted(byfam); print("\nread share of the well-seen markers (wrong reads in brackets; control = flipped frames that still read an id on screen)")
print(f"{'variant':26s} " + " | ".join(f"{f:16s}" for f in fams))
rows = {}
for vn in VARIANTS:
    cells = []
    for f in fams:
        c = res[(vn, f)]; cells.append(f"{100 * c[1] / max(c[0], 1):5.1f}% ({c[2]}) c{c[3]}"); rows[(vn, f)] = c
    print(f"{vn:26s} " + " | ".join(f"{x:16s}" for x in cells))
tot = {vn: sum(res[(vn, f)][1] for f in fams) / max(sum(res[(vn, f)][0] for f in fams), 1) for vn in VARIANTS}
wrong = {vn: sum(res[(vn, f)][2] for f in fams) for vn in VARIANTS}
front = [vn for vn in VARIANTS if not any(all(rows[(o, f)][1] / max(rows[(o, f)][0], 1) >= rows[(vn, f)][1] / max(rows[(vn, f)][0], 1) for f in fams) and wrong[o] <= wrong[vn] and (any(rows[(o, f)][1] / max(rows[(o, f)][0], 1) > rows[(vn, f)][1] / max(rows[(vn, f)][0], 1) for f in fams) or wrong[o] < wrong[vn]) for o in VARIANTS if o != vn)]
print("\nfrontier (no other variant reads at least as much in every family with no more wrong reads):", front)
print("overall read share:", {k: round(100 * v, 1) for k, v in tot.items()}, "| wrong reads:", wrong)
print("CONTROL", "PASS" if sum(c[3] for c in res.values()) == 0 else "FAIL (flipped frames read an id that is on screen)")
json.dump({"|".join(k): v for k, v in res.items()}, open(out, "w"))
