extends SceneTree
# Thicken each welded shell of a garment into a solid (inner copy + rim strip), CSG-union them, write OBJ.

const THICK := 0.004


func _init() -> void:
	var a := OS.get_cmdline_user_args()
	var src: String = a[0]
	var out: String = a[1]
	var thick: float = float(a[2]) if a.size() > 2 else THICK
	var raw := _read(src)
	var pts: PackedVector3Array = raw[0]
	var tris: PackedInt32Array = raw[1]
	# weld at 1e-5 of the bounding box
	var box := AABB(pts[0], Vector3.ZERO)
	for p in pts:
		box = box.expand(p)
	var q: float = box.get_longest_axis_size() * 1e-5
	var ids := {}
	var wp := PackedVector3Array()
	var remap := PackedInt32Array()
	for p in pts:
		var k := Vector3i(roundi(p.x / q), roundi(p.y / q), roundi(p.z / q))
		if not ids.has(k):
			ids[k] = wp.size()
			wp.append(p)
		remap.append(ids[k])
	var seen := {}
	var T: Array = []
	for t in tris.size() / 3:
		var x := remap[tris[t * 3]]
		var y := remap[tris[t * 3 + 1]]
		var z := remap[tris[t * 3 + 2]]
		if x == y or y == z or x == z:
			continue
		var s := [x, y, z]
		s.sort()
		if seen.has(s):
			continue
		seen[s] = true
		T.append(Vector3i(x, y, z))
	# shells
	var par := PackedInt32Array()
	for i in wp.size():
		par.append(i)
	for t in T:
		_union(par, t.x, t.y)
		_union(par, t.y, t.z)
	var shells := {}
	for t in T:
		var r := _find(par, t.x)
		if not shells.has(r):
			shells[r] = []
		shells[r].append(t)
	print("welded ", wp.size(), " vertices, ", T.size(), " triangles, ", shells.size(), " shells")
	var comb := CSGCombiner3D.new()
	root.add_child(comb)
	var made := 0
	var bad := 0
	var in_tris := 0
	for r in shells:
		var m := _solid(wp, shells[r], thick)
		var chk := _check(m)
		if chk[0] != 0 or chk[1] != 0 or chk[2] <= 0.0:
			bad += 1
			if bad <= 4:
				print("  input solid ", made, " (", shells[r].size(), " tris): unmatched directed edges ", chk[0], ", edges used >1 same way ", chk[1], ", signed volume ", chk[2])
		in_tris += (m.surface_get_arrays(0)[Mesh.ARRAY_INDEX] as PackedInt32Array).size() / 3
		var c := CSGMesh3D.new()
		c.mesh = m
		c.operation = CSGShape3D.OPERATION_UNION
		comb.add_child(c)
		made += 1
	print("solids ", made, ", invalid inputs ", bad, ", input triangles ", in_tris)
	await process_frame
	await process_frame
	var res: Array = comb.get_meshes()
	if res.is_empty():
		print("RESULT: empty (union refused)")
		quit(1)
		return
	var mesh: Mesh = res[1]
	var arr := mesh.surface_get_arrays(0)
	var rv: PackedVector3Array = arr[Mesh.ARRAY_VERTEX]
	var ri = arr[Mesh.ARRAY_INDEX]
	var f := FileAccess.open(out, FileAccess.WRITE)
	for p in rv:
		f.store_line("v %.9f %.9f %.9f" % [p.x, p.y, p.z])
	if ri == null or ri.size() == 0:
		for t in rv.size() / 3:
			f.store_line("f %d %d %d" % [t * 3 + 1, t * 3 + 2, t * 3 + 3])
		print("RESULT: ", rv.size() / 3, " triangles (unindexed)")
	else:
		for t in ri.size() / 3:
			f.store_line("f %d %d %d" % [ri[t * 3] + 1, ri[t * 3 + 1] + 1, ri[t * 3 + 2] + 1])
		print("RESULT: ", ri.size() / 3, " triangles, ", rv.size(), " vertices")
	quit(0)


func _solid(wp: PackedVector3Array, T: Array, thick: float) -> ArrayMesh:
	var used := {}
	for t in T:
		used[t.x] = true
		used[t.y] = true
		used[t.z] = true
	var order: Array = used.keys()
	order.sort()
	var idx := {}
	for i in order.size():
		idx[order[i]] = i
	var n := order.size()
	var nor := PackedVector3Array()
	nor.resize(n)
	for t in T:
		var fn := (wp[t.y] - wp[t.x]).cross(wp[t.z] - wp[t.x])
		nor[idx[t.x]] += fn
		nor[idx[t.y]] += fn
		nor[idx[t.z]] += fn
	var v := PackedVector3Array()
	for i in n:
		v.append(wp[order[i]])
	for i in n:
		v.append(wp[order[i]] - nor[i].normalized() * thick)
	var ix := PackedInt32Array()
	var edges := {}
	for t in T:
		var a: int = idx[t.x]
		var b: int = idx[t.y]
		var c: int = idx[t.z]
		ix.append_array(PackedInt32Array([a, b, c]))
		ix.append_array(PackedInt32Array([a + n, c + n, b + n]))
		for e in [Vector2i(a, b), Vector2i(b, c), Vector2i(c, a)]:
			edges[e] = true
	for e in edges:
		if not edges.has(Vector2i(e.y, e.x)):
			var a: int = e.x
			var b: int = e.y
			ix.append_array(PackedInt32Array([b, a, a + n]))
			ix.append_array(PackedInt32Array([b, a + n, b + n]))
	var arr := []
	arr.resize(Mesh.ARRAY_MAX)
	arr[Mesh.ARRAY_VERTEX] = v
	arr[Mesh.ARRAY_INDEX] = ix
	var m := ArrayMesh.new()
	m.add_surface_from_arrays(Mesh.PRIMITIVE_TRIANGLES, arr)
	return m


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


func _check(m: ArrayMesh) -> Array:
	var arr := m.surface_get_arrays(0)
	var v: PackedVector3Array = arr[Mesh.ARRAY_VERTEX]
	var ix: PackedInt32Array = arr[Mesh.ARRAY_INDEX]
	var e := {}
	var dup := 0
	var vol := 0.0
	for t in ix.size() / 3:
		var a := ix[t * 3]
		var b := ix[t * 3 + 1]
		var c := ix[t * 3 + 2]
		vol += v[a].dot(v[b].cross(v[c])) / 6.0
		for k in [Vector2i(a, b), Vector2i(b, c), Vector2i(c, a)]:
			if e.has(k):
				dup += 1
			e[k] = true
	var un := 0
	for k in e:
		if not e.has(Vector2i(k.y, k.x)):
			un += 1
	return [un, dup, vol]
