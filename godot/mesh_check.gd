extends SceneTree
const CL = preload("res://crosslib.gd")
# Closedness and crossing check for one or more OBJ meshes (each welded separately, then merged for crossings).
# Reports per file: vertices, triangles, shells, open edges (used once), non-manifold edges (used >2 or same direction twice),
# signed volume; then crossing triangle pairs within each file and between files.
# Pairs that share a welded vertex are skipped. A planted control (--control) moves file 1 by 1 m along x... not used; see
# --shift=x,y,z which translates the LAST file so cross-file crossings must appear.
# usage: godot --headless --path <proj> -s res://mesh_check.gd -- a.obj [b.obj] [--shift=x,y,z]


var NOWELD := false


func _init() -> void:
	var files: Array = []
	var shift := Vector3.ZERO
	for s in OS.get_cmdline_user_args():
		if s == "--noweld":
			NOWELD = true
		elif s.begins_with("--shift="):
			var p := s.substr(8).split(",")
			shift = Vector3(float(p[0]), float(p[1]), float(p[2]))
		else:
			files.append(s)
	var all_p := PackedVector3Array()
	var all_t: Array = []
	var tri_file: Array = []
	var tri_shell: Array = []
	for fi in files.size():
		var raw := _read(files[fi])
		var pts: PackedVector3Array = raw[0]
		if fi == files.size() - 1 and shift != Vector3.ZERO:
			for i in pts.size():
				pts[i] += shift
		var w := _weld(pts, raw[1])
		var wp: PackedVector3Array = w[0]
		var T: Array = w[1]
		# edges
		var dir_count := {}
		for t in T:
			for e in [Vector2i(t.x, t.y), Vector2i(t.y, t.z), Vector2i(t.z, t.x)]:
				dir_count[e] = int(dir_count.get(e, 0)) + 1
		var open_e := 0
		var nonman := 0
		var done := {}
		for e in dir_count:
			var k := Vector2i(mini(e.x, e.y), maxi(e.x, e.y))
			if done.has(k):
				continue
			done[k] = true
			var f1: int = int(dir_count.get(Vector2i(k.x, k.y), 0))
			var f2: int = int(dir_count.get(Vector2i(k.y, k.x), 0))
			if f1 == 1 and f2 == 1:
				continue
			if f1 + f2 == 1:
				open_e += 1
			else:
				nonman += 1
		var par := PackedInt32Array()
		for i in wp.size():
			par.append(i)
		for t in T:
			_union(par, t.x, t.y)
			_union(par, t.y, t.z)
		var roots := {}
		var vol := 0.0
		for t in T:
			roots[_find(par, t.x)] = true
			vol += wp[t.x].dot(wp[t.y].cross(wp[t.z])) / 6.0
		print("%s: %d verts, %d tris, %d shells, %d open edges, %d non-manifold edges, signed volume %.6f" % [files[fi].get_file(), wp.size(), T.size(), roots.size(), open_e, nonman, vol])
		var base := all_p.size()
		all_p.append_array(wp)
		for t in T:
			all_t.append(Vector3i(t.x + base, t.y + base, t.z + base))
			tri_file.append(fi)
			tri_shell.append(_find(par, t.x) + base)
	# crossings
	var box := AABB(all_p[0], Vector3.ZERO)
	for p in all_p:
		box = box.expand(p)
	var tot_edge := 0.0
	for t in all_t:
		tot_edge += all_p[t.x].distance_to(all_p[t.y])
	var cell: float = 2.0 * tot_edge / float(all_t.size())
	var grid := {}
	for ti in all_t.size():
		var t: Vector3i = all_t[ti]
		var lo: Vector3 = all_p[t.x].min(all_p[t.y]).min(all_p[t.z])
		var hi: Vector3 = all_p[t.x].max(all_p[t.y]).max(all_p[t.z])
		for x in range(floori(lo.x / cell), floori(hi.x / cell) + 1):
			for y in range(floori(lo.y / cell), floori(hi.y / cell) + 1):
				for z in range(floori(lo.z / cell), floori(hi.z / cell) + 1):
					var k := Vector3i(x, y, z)
					if not grid.has(k):
						grid[k] = []
					grid[k].append(ti)
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
				var A: Vector3i = all_t[ta]
				var B: Vector3i = all_t[tb]
				if A.x == B.x or A.x == B.y or A.x == B.z or A.y == B.x or A.y == B.y or A.y == B.z or A.z == B.x or A.z == B.y or A.z == B.z:
					continue
				if _tri_cross(all_p, A, B) or _tri_cross(all_p, B, A):
					pairs[key] = true
	var same := {}
	var cross_file := 0
	var same_shell := 0
	for key in pairs:
		var ta: int = key / 4194304
		var tb: int = key % 4194304
		if tri_file[ta] != tri_file[tb]:
			cross_file += 1
		elif tri_shell[ta] == tri_shell[tb]:
			same_shell += 1
	print("CROSSINGS: %d pairs total; %d between files; %d inside one shell; %d between shells of one file" % [pairs.size(), cross_file, same_shell, pairs.size() - cross_file - same_shell])
	quit(0)


func _tri_cross(P: PackedVector3Array, A: Vector3i, B: Vector3i) -> bool:
	var a := P[A.x]
	var b := P[A.y]
	var c := P[A.z]
	var q0 := P[B.x]
	var q1 := P[B.y]
	var q2 := P[B.z]
	return CL.seg_tri(q0, q1, a, b, c) or CL.seg_tri(q1, q2, a, b, c) or CL.seg_tri(q2, q0, a, b, c)


func _weld(pts: PackedVector3Array, tris: PackedInt32Array) -> Array:
	var q := 1e-6
	if NOWELD:
		q = 1e-12
	var ids := {}
	var wp := PackedVector3Array()
	var remap := PackedInt32Array()
	for p in pts:
		var k := Vector3i(roundi(p.x / q), roundi(p.y / q), roundi(p.z / q))
		if not ids.has(k):
			ids[k] = wp.size()
			wp.append(p)
		remap.append(ids[k])
	var T: Array = []
	for t in tris.size() / 3:
		var x := remap[tris[t * 3]]
		var y := remap[tris[t * 3 + 1]]
		var z := remap[tris[t * 3 + 2]]
		if x == y or y == z or x == z:
			continue
		T.append(Vector3i(x, y, z))
	return [wp, T]


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
	var f := FileAccess.open(path, FileAccess.READ)
	while not f.eof_reached():
		var l := f.get_line()
		if l.begins_with("v "):
			var t := l.split(" ", false)
			v.append(Vector3(float(t[1]), float(t[2]), float(t[3])))
		elif l.begins_with("f "):
			var t := l.split(" ", false)
			for k in 3:
				idx.append(int(t[k + 1].split("/")[0]) - 1)
	return [v, idx]
