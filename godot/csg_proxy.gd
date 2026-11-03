extends SceneTree
# CSG proxy: weld, split into shells, thicken each shell inward into a closed solid, CSG-union them (Manifold,
# dev/csg-vertex-attributes), and write the union surface as the closed proxy. No flood fill.
# usage: godot --headless --path <proj> -s res://csg_proxy.gd -- <in.obj> <out.obj> <thickness in the mesh's units>


func _init() -> void:
	var a := OS.get_cmdline_user_args()
	var raw := _read(a[0])
	var thick := float(a[2])
	var pts: PackedVector3Array = raw[0]
	var tris: PackedInt32Array = raw[1]
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
	var keys: Array = shells.keys()
	var n_shells := keys.size()
	print("welded ", wp.size(), " verts, ", T.size(), " tris, ", n_shells, " shells")
	var comb := CSGCombiner3D.new()
	root.add_child(comb)
	for i in n_shells:
		var c := CSGMesh3D.new()
		c.mesh = _solid(wp, shells[keys[i]], thick, i, n_shells)
		c.operation = CSGShape3D.OPERATION_UNION
		comb.add_child(c)
	for i in 4:
		await process_frame
	var got: Array = comb.get_meshes()
	if got.is_empty():
		print("UNION REFUSED")
		quit(1)
		return
	var arr = (got[1] as Mesh).surface_get_arrays(0)
	var vv: PackedVector3Array = arr[Mesh.ARRAY_VERTEX]
	var ii = arr[Mesh.ARRAY_INDEX]
	var ov := PackedVector3Array()
	var oi := PackedInt32Array()
	var oid := {}
	var qq := q
	var src_i: PackedInt32Array = ii if ii != null and ii.size() > 0 else PackedInt32Array(range(vv.size()))
	for k in src_i:
		var p := vv[k]
		var key := Vector3i(roundi(p.x / qq), roundi(p.y / qq), roundi(p.z / qq))
		if not oid.has(key):
			oid[key] = ov.size()
			ov.append(p)
		oi.append(oid[key])
	var f := FileAccess.open(a[1], FileAccess.WRITE)
	for p in ov:
		f.store_line("v %.9f %.9f %.9f" % [p.x, p.y, p.z])
	for t in oi.size() / 3:
		f.store_line("f %d %d %d" % [oi[t * 3] + 1, oi[t * 3 + 1] + 1, oi[t * 3 + 2] + 1])
	f.close()
	print("proxy: ", ov.size(), " verts, ", oi.size() / 3, " tris -> ", a[1])
	quit(0)


func _solid(wp: PackedVector3Array, T: Array, thick: float, sid: int, n_shells: int) -> ArrayMesh:
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
	var colors := PackedColorArray()
	colors.resize(v.size())
	colors.fill(Color(float(sid + 1) / float(n_shells + 1), 0.0, 0.0, 1.0))
	var arr := []
	arr.resize(Mesh.ARRAY_MAX)
	arr[Mesh.ARRAY_VERTEX] = v
	arr[Mesh.ARRAY_COLOR] = colors
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
