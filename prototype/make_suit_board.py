# The real SOMA suit board, not a custom one: CharucoBoard((49, 65), 0.03, 0.022, DICT_4X4_1000) with the legacy pattern and
# ids = arange(1592) % 1000, so ids repeat after 1000 exactly as on the printed suit (3 cm squares, 22 mm markers). Each
# marker is told apart by its board position (idx, row-major over the detected quads), the way the paper uses an instance
# index. Gates: every placement reads back with its id, ids wrap, a blank sheet reads nothing.
# Writes board_suit.png (+ board_body.png / board_skirt.png copies for the renderers), suit_markers.json
# [{idx, id, quad}] and boards_meta.json (meshes get disjoint idx ranges: body 0..BODY-1, skirt BODY..).
# python make_suit_board.py [body markers = 950] [dictionary = DICT_4X4_1000] [render px per square = 45]   (needs opencv-python-headless, cv2 5.0)
# Any OpenCV dictionary works (DICT_APRILTAG_36h10 has 2320 unique tags, so ids never repeat); the choice is written to suit_dictionary.txt.
import sys, json, shutil, numpy as np, cv2, cv2.aruco as aruco
PX, PAD = 45, 100
BODY = int(sys.argv[1]) if len(sys.argv) > 1 else 950
DICT = sys.argv[2] if len(sys.argv) > 2 else "DICT_4X4_1000"; RPX = int(sys.argv[3]) if len(sys.argv) > 3 else 45     # RPX: drawn resolution; the layout and quads stay in 45 px squares
import sys as _sys, os as _os; _sys.path.insert(0, _os.path.dirname(_os.path.abspath(__file__))); import tagfamily
d = tagfamily.get(DICT); NID = len(d.bytesList)
ids = np.arange(1592) % NID
open("suit_dictionary.txt", "w").write(DICT)
b = aruco.CharucoBoard((49, 65), 0.03, 0.022, d, ids); b.setLegacyPattern(True)
img = b.generateImage((49 * RPX, 65 * RPX), marginSize=0, borderBits=1)
cv2.imwrite("board_suit.png", img)
for n in ("body", "skirt"): shutil.copy("board_suit.png", f"board_{n}.png")
det = aruco.ArucoDetector(d, aruco.DetectorParameters())
PADR = PAD * RPX // PX; pad = cv2.copyMakeBorder(img, PADR, PADR, PADR, PADR, cv2.BORDER_CONSTANT, value=255)
c, i, _, _ = det.detectMarkersWithConfidence(pad)
quads = [(int(k), (q.reshape(4, 2) - PADR) * PX / RPX) for q, k in zip(c, i.flatten())]
quads.sort(key=lambda t: (round(float(t[1][:, 1].mean()) / 20), float(t[1][:, 0].mean())))     # row-major by marker centre
suit = [{"idx": n, "id": k, "quad": [[float(x), float(y)] for x, y in q]} for n, (k, q) in enumerate(quads)]
json.dump(suit, open("suit_markers.json", "w"))
want = sorted(ids.tolist()); got = sorted(k for k, _ in quads)
blank = det.detectMarkersWithConfidence(np.full_like(pad, 255))[1]
print("board", img.shape[::-1], "markers read back", len(quads), "of", len(ids), "; ids multiset equal:", want == got, "; distinct ids", len(set(got)), "; ids repeat once past 1000:", len(quads) - len(set(got)), "; blank-sheet reads", 0 if blank is None else len(blank))
n = len(suit)
json.dump({"body": {"w": 49 * PX, "h": 65 * PX, "first": 0, "count": BODY}, "skirt": {"w": 49 * PX, "h": 65 * PX, "first": BODY, "count": n - BODY}}, open("boards_meta.json", "w"))
print("body idx 0..%d, skirt idx %d..%d" % (BODY - 1, BODY, n - 1))
