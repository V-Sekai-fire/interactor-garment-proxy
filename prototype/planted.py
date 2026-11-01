import sys; sys.path.insert(0,".")
from shells_cross import *
import tempfile
def mk(s):
    t=tempfile.NamedTemporaryFile("w",suffix=".obj",delete=False); t.write(s); t.close(); return t.name
# two triangles, one pierced by the other's edge; and the same pair moved apart
hit="v 0 0 0\nv 4 0 0\nv 0 4 0\nv 1 1 -1\nv 1 1 1\nv 3 3 1\nf 1 2 3\nf 4 5 6\n"
miss="v 0 0 0\nv 4 0 0\nv 0 4 0\nv 1 1 2\nv 1 1 3\nv 3 3 3\nf 1 2 3\nf 4 5 6\n"
print("planted crossing ->",analyse(mk(hit))["crossing_pairs"],"(must be 1)")
print("planted separate ->",analyse(mk(miss))["crossing_pairs"],"(must be 0)")
