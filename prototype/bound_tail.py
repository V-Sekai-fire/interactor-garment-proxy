# Splits the markers that were not triangulated into: never well seen, well seen in one camera only, well seen in 2+ cameras but read in fewer than 2 (the closable gap).
import sys, json
d = sys.argv[1]; r = json.load(open(f"{d}/reads.json"))["0:body"]
on = r["on"]; rd = {int(k): v for k, v in r["reads"].items()}; wl = {int(k): v for k, v in r["well"].items()}
tri = [i for i in on if rd.get(i, 0) >= 2]; miss = [i for i in on if rd.get(i, 0) < 2]
A = [i for i in miss if wl.get(i, 0) == 0]; B = [i for i in miss if wl.get(i, 0) == 1]; C = [i for i in miss if wl.get(i, 0) >= 2]
n = len(on); print(f"markers on mesh {n}; triangulated {len(tri)} ({100 * len(tri) / n:.2f}%)")
print(f"  never well seen by the planned cameras: {len(A)} ({100 * len(A) / n:.2f}%)")
print(f"  well seen in one camera only:           {len(B)} ({100 * len(B) / n:.2f}%)")
print(f"  well seen in 2+ cameras, read in <2:    {len(C)} ({100 * len(C) / n:.2f}%)   <- the detector gap")
print(f"upper bound with these cameras (well seen in 2+): {100 * (len(tri) + 0 + len(C)) / n:.2f}% ; triangulated of those: {100 * len(tri) / (len(tri) + len(C)):.2f}%")
