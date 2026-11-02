# Scan test for a tag family: flat sheets that carry every tag at the suit's real square size, plus cameras at several distances and tilts.
# make:  tag_bench.py make <out dir> [planes = 5] [marker scale = 0.5]   -> <out>/anny_uv.obj (sheets, UVs in board units), cams_c0.json, cams_c1.json
# judge: tag_bench.py judge <dir with reads.json from recall_ids.py> <family json> <out blocklist.json> [planted ids, comma separated]
#   a tag is broken when some view saw it well and it did not read there; the planted ids (known broken) must be flagged or the test is not trusted.
import sys, os, json, math, numpy as np
mode = sys.argv[1]
if mode == "make":
    out = sys.argv[2]; NP = int(sys.argv[3]) if len(sys.argv) > 3 else 5; MS = float(sys.argv[4]) if len(sys.argv) > 4 else 0.5
    os.makedirs(out, exist_ok=True); S = 45; BW, BH = 49 * S, 65 * S; ppm = 1500.0 / MS / (S / 45.0)   # texels per metre (30 mm * MS per square)
    w, h = BW / ppm, BH / ppm; V = []; VT = []; F = []; cams = []; fams = []
    for j in range(NP):
        x0 = j * (w + 1.0); base = len(V)
        for (px, py) in ((0, 0), (BW, 0), (BW, BH), (0, BH)):
            V.append([x0 + px / ppm, (BH - py) / ppm, 0.0]); VT.append([(j * BW + px) / BW, py / BH])   # atlas rows run down, world y runs up: without the flip the sheets render mirrored and nothing reads
        F += [[base, base + 1, base + 2], [base, base + 2, base + 3]]
        for D, fov, tilt in ((0.8, 30.0, 0.0), (0.55, 30.0, 0.0), (1.2, 30.0, 0.0), (0.8, 30.0, 40.0), (0.8, 30.0, -40.0)):
            fp = 2 * D * math.tan(math.radians(fov / 2)) * 0.85; nx = max(1, math.ceil(w / fp)); ny = max(1, math.ceil(h / fp))
            for a in range(nx):
                for b in range(ny):
                    tx = x0 + (a + 0.5) * w / nx; ty = (b + 0.5) * h / ny; t = math.radians(tilt)
                    cams.append({"pos": [tx + D * math.sin(t), ty, D * math.cos(t)], "target": [tx, ty, 0.0], "up": [0.0, 1.0, 0.0], "fov": fov}); fams.append(f"{D} m, tilt {tilt:+.0f}")
    with open(f"{out}/anny_uv.obj", "w") as f:
        for v in V: f.write("v %f %f %f\n" % tuple(v))
        for v in VT: f.write("vt %f %f\n" % tuple(v))
        for t in F: f.write("f " + " ".join(f"{i + 1}/{i + 1}" for i in t) + "\n")
    json.dump(cams, open(f"{out}/cams_c0.json", "w")); json.dump([], open(f"{out}/cams_c1.json", "w")); json.dump(fams, open(f"{out}/families.json", "w")); print(NP, "sheets,", len(cams), "cameras")
else:
    d, fam, out = sys.argv[2:5]; planted = [int(x) for x in sys.argv[5].split(",")] if len(sys.argv) > 5 else []
    r = json.load(open(f"{d}/reads.json"))["0:body"]; rd = {int(k): v for k, v in r["reads"].items()}; wl = {int(k): v for k, v in r["well"].items()}
    codes = json.load(open(fam))["codes"]
    broken = sorted(i for i in wl if rd.get(i, 0) < wl[i]); unseen = [i for i in range(len(codes)) if wl.get(i, 0) == 0]
    flagged = [p for p in planted if p in broken]
    json.dump({"broken": broken, "codes": [codes[i] for i in broken], "never_seen": unseen}, open(out, "w"))
    print(f"tags {len(codes)}; seen well in at least one view: {len(wl)}; never seen: {len(unseen)}; broken (some well-seen view did not read): {len(broken)}")
    if planted: print("planted known-broken ids flagged:", flagged, "of", planted, "-> TEST", "TRUSTED" if len(flagged) == len(planted) else "NOT TRUSTED (misses a known-broken tag)")
