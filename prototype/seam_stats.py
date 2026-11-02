# How many suit markers a UV unwrap cuts. Reads godot/unwrap_tiles.gd output (<obj> and <obj>.json), rasterises each
# tile's charts (connected UV pieces) into a label map at board resolution, lays the tile's ChArUco board over it, and
# counts per marker: intact (all 9 sample points inside one chart), cut (some inside a chart, the rest in another
# chart or in empty space), empty (none inside a chart). Run in the folder holding dict6x6_2000.npy.
# python seam_stats.py <obj> [<obj> ...]
import sys, json, numpy as np, cv2, cv2.aruco as aruco
d = aruco.Dictionary(np.load("dict6x6_2000.npy"), 6, 4)
det = aruco.ArucoDetector(d, aruco.DetectorParameters())
PX, PAD = 45, 100
def parse(path):
    v, vt, tiles, cur = [], [], {}, None
    for l in open(path):
        t = l.split()
        if not t: continue
        if t[0] == "usemtl": cur = int(t[1].split("_")[1]); tiles.setdefault(cur, [])
        elif t[0] == "vt": vt.append((float(t[1]), float(t[2])))
        elif t[0] == "f": tiles[cur].append([int(x.split("/")[0]) - 1 for x in t[1:4]])
    return np.array(vt), {k: np.array(f) for k, f in tiles.items()}
def board_quads(bw, bh):
    n = len(aruco.CharucoBoard((bw // PX, bh // PX), 0.03, 0.022, d).getIds())
    b = aruco.CharucoBoard((bw // PX, bh // PX), 0.03, 0.022, d, np.arange(n)); b.setLegacyPattern(True)
    img = cv2.copyMakeBorder(b.generateImage((bw, bh), marginSize=0, borderBits=1), PAD, PAD, PAD, PAD, cv2.BORDER_CONSTANT, value=255)
    c, i, _, _ = det.detectMarkersWithConfidence(img)
    return n, [q.reshape(4, 2) - PAD for q in c]
def run(path):
    vt, tiles = parse(path); meta = {m["tile"]: m for m in json.load(open(path + ".json"))}
    T = dict(charts=0, markers=0, intact=0, cut=0, empty=0, tris=0)
    for k, tri in tiles.items():
        bw, bh = meta[k]["board_w"], meta[k]["board_h"]
        parent = np.arange(len(vt))
        def find(a):
            while parent[a] != a: parent[a] = parent[parent[a]]; a = parent[a]
            return a
        for a, b, c in tri:
            ra, rb, rc = find(a), find(b), find(c); parent[rb] = ra; parent[find(c)] = ra
        lab = np.full((bh, bw), -1, np.int32); roots = {}
        for a, b, c in tri:
            r = roots.setdefault(find(a), len(roots))
            pts = np.round(vt[[a, b, c]] * [bw, bh]).astype(np.int32)
            cv2.fillConvexPoly(lab, pts, r)
        n, quads = board_quads(bw, bh)
        g = np.linspace(0.1, 0.9, 3)
        for q in quads:
            pts = [(q[0] * (1 - u) + q[1] * u) * (1 - w) + (q[3] * (1 - u) + q[2] * u) * w for u in g for w in g]
            ls = [lab[int(np.clip(round(p[1]), 0, bh - 1)), int(np.clip(round(p[0]), 0, bw - 1))] for p in pts]
            if all(l >= 0 and l == ls[0] for l in ls): T["intact"] += 1
            elif any(l >= 0 for l in ls): T["cut"] += 1
            else: T["empty"] += 1
        T["charts"] += len(roots); T["markers"] += len(quads); T["tris"] += len(tri)
    used = T["intact"] + T["cut"]
    print(f"{path.split('/')[-1]:14s} tiles {len(tiles)}  charts {T['charts']:4d}  markers on boards {T['markers']:5d}  on the mesh {used:5d}  intact {T['intact']:5d}  cut by a seam {T['cut']:5d}  intact share {100.0 * T['intact'] / max(used, 1):.1f}%")
for p in sys.argv[1:]: run(p)
