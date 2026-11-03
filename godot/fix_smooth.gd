extends SceneTree
const CL = preload("res://crosslib.gd")
# Resolve pinch crossings by local Laplacian smoothing: vertices of crossing triangles plus a growing ring around them
# move halfway to the mean of their edge neighbours, repeated until the mesh has no crossing pair (index adjacency).
# usage: -- <in.obj> <out.obj> <max_iter> <log.txt>


func _init() -> void:
	var a := OS.get_cmdline_user_args()
	var raw := _read(a[0])
	var P: PackedVector3Array = raw[0]
	var idx: PackedInt32Array = raw[1]
	var nt := idx.size() / 3
	var nb: Array = []
	nb.resize(P.size())
	for i in P.size():
		nb[i] = {}
	for t in nt:
		for k in 3:
			nb[idx[t * 3 + k]][idx[t * 3 + (k + 1) % 3]] = true
			nb[idx[t * 3 + (k + 1) % 3]][idx[t * 3 + k]] = true
	var orig := P.duplicate()
	var lg := FileAccess.open(a[3], FileAccess.WRITE)
	var touched := {}
	var it := 0
	var rings := 1
	while true:
		var pairs := _crossings(P, idx)
		lg.store_line("iteration %d: %d crossing pairs, %d vertices touched so far" % [it, pairs.size(), touched.size()])
		lg.flush()
		if pairs.is_empty() or it >= int(a[2]):
			break
		var mark := {}
		for key in pairs:
			for t in [key / 4194304, key % 4194304]:
				for k in 3:
					mark[idx[t * 3 + k]] = true
		for r in rings:
			var add := {}
			for v in mark:
				for n in nb[v]:
					add[n] = true
			mark.merge(add)
		rings = mini(rings + (1 if it % 3 == 2 else 0), 4)
		var newp := P.duplicate()
		for v in mark:
			var m := Vector3.ZERO
			for n in nb[v]:
				m += P[n]
			m /= float(nb[v].size())
			newp[v] = P[v].lerp(m, 0.5)
			touched[v] = true
		P = newp
		it += 1
	var mx := 0.0
	var mean := 0.0
	for v in touched:
		var d := P[v].distance_to(orig[v])
		mx = maxf(mx, d)
		mean += d
	lg.store_line("final: %d vertices moved, max move %.6f, mean %.6f (solver units)" % [touched.size(), mx, mean / maxf(1.0, touched.size())])
	lg.close()
	var f := FileAccess.open(a[1], FileAccess.WRITE)
	for v in P:
		f.store_line("v %.9f %.9f %.9f" % [v.x, v.y, v.z])
	for t in nt:
		f.store_line("f %d %d %d" % [idx[t * 3] + 1, idx[t * 3 + 1] + 1, idx[t * 3 + 2] + 1])
	f.close()
	quit(0)
func _crossings(P: PackedVector3Array, idx: PackedInt32Array) -> Dictionary:
	var nt := idx.size() / 3
	var tot := 0.0
	for t in nt:
		tot += P[idx[t * 3]].distance_to(P[idx[t * 3 + 1]])
	var cell := 2.0 * tot / float(nt)
	var grid := {}
	for t in nt:
		var A := P[idx[t * 3]]
		var B := P[idx[t * 3 + 1]]
		var C := P[idx[t * 3 + 2]]
		var lo := A.min(B).min(C)
		var hi := A.max(B).max(C)
		for x in range(floori(lo.x / cell), floori(hi.x / cell) + 1):
			for y in range(floori(lo.y / cell), floori(hi.y / cell) + 1):
				for z in range(floori(lo.z / cell), floori(hi.z / cell) + 1):
					var k := Vector3i(x, y, z)
					if not grid.has(k):
						grid[k] = []
					grid[k].append(t)
	var pairs := {}
	var checked := {}
	for k in grid:
		var l: Array = grid[k]
		for i in l.size():
			for j in range(i + 1, l.size()):
				var ta: int = l[i]
				var tb: int = l[j]
				var key: int = ta * 4194304 + tb if ta < tb else tb * 4194304 + ta
				if checked.has(key):
					continue
				checked[key] = true
				var shared := false
				for p in 3:
					for q in 3:
						if idx[ta * 3 + p] == idx[tb * 3 + q]:
							shared = true
				if shared:
					continue
				if _tri_cross(P, idx, ta, tb) or _tri_cross(P, idx, tb, ta):
					pairs[key] = true
	return pairs


func _tri_cross(P: PackedVector3Array, idx: PackedInt32Array, ta: int, tb: int) -> bool:
	var a := P[idx[ta * 3]]
	var b := P[idx[ta * 3 + 1]]
	var c := P[idx[ta * 3 + 2]]
	var q0 := P[idx[tb * 3]]
	var q1 := P[idx[tb * 3 + 1]]
	var q2 := P[idx[tb * 3 + 2]]
	return CL.seg_tri(q0, q1, a, b, c) or CL.seg_tri(q1, q2, a, b, c) or CL.seg_tri(q2, q0, a, b, c)


func _find(par: PackedInt32Array, x: int) -> int:
	while par[x] != x:
		par[x] = par[par[x]]
		x = par[x]
	return x


func _union(par: PackedInt32Array, a: int, b: int) -> void:
	var ra := _find(par, a)
	var rb := _find(par, b)
	if ra != rb:
		par[ra] = rb


func _read(path: String) -> Array:
	var v := PackedVector3Array()
	var idx := PackedInt32Array()
	for l in FileAccess.get_file_as_string(path).split("\n"):
		if l.begins_with("v "):
			var t := l.split(" ", false)
			v.append(Vector3(float(t[1]), float(t[2]), float(t[3])))
		elif l.begins_with("f "):
			var t := l.split(" ", false)
			for k in 3:
				idx.append(int(t[k + 1].split("/")[0]) - 1)
	return [v, idx]
