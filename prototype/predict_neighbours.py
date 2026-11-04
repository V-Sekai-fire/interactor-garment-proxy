# How many markers that were not triangulated (fewer than 2 reads) sit in an island with enough triangulated neighbours to be placed without being seen?
# Usage: predict_neighbours.py <dir with suit_markers.json and reads.json> [steps]   (a neighbour is within <steps> squares in the same island)
import sys, json, numpy as np
d = sys.argv[1]; steps = float(sys.argv[2]) if len(sys.argv) > 2 else 3.0
mk = json.load(open(f"{d}/suit_markers.json")); S = 45
mk = mk if isinstance(mk, list) else mk.get("markers", mk)
rd = json.load(open(f"{d}/reads.json"))["0:body"]; cnt = {int(k): v for k, v in rd["reads"].items()}
on = rd["on"]; pos = {m["idx"]: (m["chart"], np.array(m["square"], float)) for m in mk}
by = {}
for i, (c, p) in pos.items(): by.setdefault(c, []).append(i)
tri = {i for i in on if cnt.get(i, 0) >= 2}; miss = [i for i in on if i not in tri]
out = {}
for need in (1, 2, 3, 4):
    ok = 0
    for i in miss:
        c, p = pos[i]; n = sum(1 for j in by[c] if j in tri and j != i and np.abs(pos[j][1] - p).max() <= steps * S)
        ok += n >= need
    out[need] = ok
print(f"markers on mesh {len(on)}; triangulated {len(tri)} ({100 * len(tri) / len(on):.1f}%); not triangulated {len(miss)}")
for need, ok in out.items():
    print(f"  not triangulated but with >= {need} triangulated neighbours within {steps:g} squares in the same island: {ok} -> coverage {100 * (len(tri) + ok) / len(on):.1f}%")
