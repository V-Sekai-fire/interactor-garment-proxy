# Planted control for df_proxy.py --flood: a cube with a smaller closed cube inside it (an air bubble).
# Without --flood every closed cube leaves an outer and an inner offset surface (4 shells); with it only the outer envelope remains (1 shell). Run: uv run --no-project --with numpy prototype/planted_flood.py
import os, subprocess, sys, tempfile
here = os.path.dirname(os.path.abspath(__file__))
def cube(lo, hi, base):
    v = [(x, y, z) for x in (lo, hi) for y in (lo, hi) for z in (lo, hi)]
    q = [(0,1,3,2),(4,6,7,5),(0,4,5,1),(2,3,7,6),(0,2,6,4),(1,5,7,3)]
    f = [t for a,b,c,d in q for t in ((a,b,c),(a,c,d))]
    return v, [(base+a+1, base+b+1, base+c+1) for a,b,c in f]
v1, f1 = cube(0.0, 1.0, 0)
v2, f2 = cube(0.4, 0.6, 8)
def shells(path):
    V, F = [], []
    for l in open(path):
        t = l.split()
        if t and t[0] == "v": V.append(tuple(t[1:4]))
        elif t and t[0] == "f": F.append([int(x.split("/")[0]) - 1 for x in t[1:]])
    parent = list(range(len(V)))
    def find(a):
        while parent[a] != a: parent[a] = parent[parent[a]]; a = parent[a]
        return a
    for f in F:
        for a in f[1:]: parent[find(a)] = find(f[0])
    used = {find(a) for f in F for a in f}
    return len(used)
with tempfile.TemporaryDirectory() as d:
    src = os.path.join(d, "nested.obj")
    with open(src, "w") as fh:
        for v in v1 + v2: fh.write("v %g %g %g\n" % v)
        for f in f1 + f2: fh.write("f %d %d %d\n" % f)
    res = {}
    for flag in ([], ["--flood"]):
        out = os.path.join(d, "out%d.obj" % len(flag))
        subprocess.run([sys.executable, os.path.join(here, "df_proxy.py"), src, out, "0.02", "0.02"] + flag, check=True, capture_output=True)
        res[bool(flag)] = shells(out)
print("shells without flood:", res[False], " with flood:", res[True])
assert res[False] == 4 and res[True] == 1, "flood fill must leave only the outer envelope, and its absence must keep every offset shell"
print("ok: the control can fail (without --flood all four offset shells stay)")
