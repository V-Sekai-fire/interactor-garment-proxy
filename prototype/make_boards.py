# Unique ArUco ids per mesh: one custom 6x6 dictionary (2000 markers, made once and kept in dict6x6_2000.npy), one
# ChArUco board per mesh with its own disjoint id range, sized to the mesh's UV atlas / 45 px per square.
# Gates per board: every id reads back once, none extra, a blank sheet reads none. Writes board_<name>.png and
# boards_meta.json. Needs opencv-python-headless (cv2 5.0).
import cv2, cv2.aruco as aruco, numpy as np, json, sys
PX = 45
import os
if not os.path.exists("dict6x6_2000.npy"):
    np.save("dict6x6_2000.npy", aruco.extendDictionary(2000, 6, aruco.Dictionary(np.zeros((0, 5, 4), np.uint8), 6), 0).bytesList)
bl = np.load("dict6x6_2000.npy")
d = aruco.Dictionary(bl, 6, 4)
specs = {"body": (50, 46), "skirt": (38, 35)}
start = 0; meta = {}
det = aruco.ArucoDetector(d, aruco.DetectorParameters())
allids = {}
for name, (w, h) in specs.items():
    n = len(aruco.CharucoBoard((w, h), 0.03, 0.022, d).getIds())
    ids = np.arange(start, start + n)
    b = aruco.CharucoBoard((w, h), 0.03, 0.022, d, ids); b.setLegacyPattern(True)
    img = b.generateImage((w*PX, h*PX), marginSize=0, borderBits=1)
    cv2.imwrite(f"board_{name}.png", img)
    pad = cv2.copyMakeBorder(img, 100, 100, 100, 100, cv2.BORDER_CONSTANT, value=255)
    c, i, _, _ = det.detectMarkersWithConfidence(pad)
    got = set() if i is None else set(int(x) for x in i.flatten())
    want = set(int(x) for x in ids)
    blank = det.detectMarkersWithConfidence(np.full_like(pad, 255))[1]
    print(name, (w*PX, h*PX), "markers", n, "ids", start, "to", start+n-1, "read back", len(got & want), "missing", len(want-got), "extra", len(got-want), "blank-control reads", 0 if blank is None else len(blank))
    allids[name] = want; meta[name] = {"w": w*PX, "h": h*PX, "first": start, "count": n}
    start += n
print("disjoint:", not (allids["body"] & allids["skirt"]), "total ids used", start, "of", len(bl))
json.dump(meta, open("boards_meta.json", "w"))
