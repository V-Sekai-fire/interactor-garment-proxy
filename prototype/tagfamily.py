# One place that turns a dictionary name into an OpenCV dictionary, so the board maker and every reader agree.
#   DICT_4X4_1000, DICT_APRILTAG_36h10, ...  an OpenCV predefined dictionary
#   custom:<family.json>                      a family made by lean/MarkerDetect (tagfamily_gen): side, minDistance, codes (hex, bit k = row k // side, column k % side, 1 = white)
# load_dictionary() reads the name from suit_dictionary.txt in the current folder (default DICT_4X4_1000).
import os, json, numpy as np, cv2, cv2.aruco as aruco

def from_family(path):
    f = json.load(open(path)); side = f["side"]; d = f["minDistance"]; rows = []
    for h in f["codes"]:
        v = int(h, 16); bits = np.array([(v >> k) & 1 for k in range(side * side)], np.uint8).reshape(side, side)
        rows.append(aruco.Dictionary.getByteListFromBits(bits)[0])   # (nbytes, 4): the four rotations, as OpenCV stores them
    return aruco.Dictionary(np.stack(rows).astype(np.uint8), side, (d - 1) // 2)

def get(name):
    if name.startswith("custom:"): return from_family(name[len("custom:"):])
    return aruco.getPredefinedDictionary(getattr(aruco, name))

def load_dictionary(path="suit_dictionary.txt"):
    return get(open(path).read().strip()) if os.path.exists(path) else aruco.getPredefinedDictionary(aruco.DICT_4X4_1000)

def board_size(path="suit_board_size.json"):
    """(width, height) in px of the texture the markers are painted on: the printed sheet (49 x 65 squares of 45 px) unless suit_board_size.json says otherwise."""
    return (json.load(open(path))["w"], json.load(open(path))["h"]) if os.path.exists(path) else (49 * 45, 65 * 45)

def px_per_mm():
    """Texels per millimetre of surface: 1.5 at the repo's 30 mm squares, 1.5 / MARKER_SCALE for smaller markers."""
    return 1.5 / float(os.environ.get("MARKER_SCALE", "1.0"))
