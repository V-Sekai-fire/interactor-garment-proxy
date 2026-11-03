extends SceneTree
# CSG-union proxy: weld, split into shells, thicken every shell inward into a closed solid, union them with Manifold,
# weld the union surface, drop surface components that are not visible from outside (enclosed / air-bubble parts),
# write the result as a scratch OBJ (PolyFEM input) and report.
# usage: godot --headless --path <proj> -s res://union_proxy.gd -- <usd.elf> <in.obj|in.usdz> <out.obj> <thickness_m>


func _init() -> void:
	var a := OS.get_cmdline_user_args()
	var elf := FileAccess.get_file_as_bytes(a[0])
	var src: String = a[1]
	var out: String = a[2]
	var thick: float = float(a[3])
	var raw: Array = _read_usdz(elf, src) if src.ends_with(".usdz") else _read(src)
	var pts: PackedVector3Array = raw[0]
	var tris: PackedInt32Array = raw[1]
	print("input: %d points, %d triangles" % [pts.size(), tris.size() / 3])
	var w := _weld(pts, tris)
	var wp: PackedVector3Array = w[0]
	var T: Array = w[1]
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
	print("welded: %d points, %d triangles, %d shells" % [wp.size(), T.size(), keys.size()])
	var comb := CSGCombiner3D.new()
	root.add_child(comb)
	for i in keys.size():
		var c := CSGMesh3D.new()
		c.mesh = _solid(wp, shells[keys[i]], thick)
		c.operation = CSGShape3D.OPERATION_UNION
		comb.add_child(c)
	await process_frame
	await process_frame
	var got: Array = comb.get_meshes()
	if got.is_empty():
		print("RESULT: FAIL union empty/refused")
		quit(1)
		return
	var arr = (got[1] as Mesh).surface_get_arrays(0)
	var up: PackedVector3Array = arr[Mesh.ARRAY_VERTEX]
	var ui = arr[Mesh.ARRAY_INDEX]
	var utris := PackedInt32Array()
	if ui == null or (ui as PackedInt32Array).is_empty():
		for i in up.size():
			utris.append(i)
	else:
		utris = ui
	print("union surface raw: %d verts, %d triangles" % [up.size(), utris.size() / 3])
	var w2 := _weld(up, utris)
	var vp: PackedVector3Array = w2[0]
	var UT: Array = w2[1]
	# components
	var par2 := PackedInt32Array()
	for i in vp.size():
		par2.append(i)
	for t in UT:
		_union(par2, t.x, t.y)
		_union(par2, t.y, t.z)
	var comp_of_tri := PackedInt32Array()
	var comps := {}
	for ti in UT.size():
		var r := _find(par2, UT[ti].x)
		if not comps.has(r):
			comps[r] = []
		comps[r].append(ti)
	var ckeys: Array = comps.keys()
	print("union components: %d" % ckeys.size())
	# exposure test: 26 directions, ray from the extreme vertex of the component
	var am := ArrayMesh.new()
	var arr2 := []
	arr2.resize(Mesh.ARRAY_MAX)
	arr2[Mesh.ARRAY_VERTEX] = vp
	var flat := PackedInt32Array()
	for t in UT:
		flat.append(t.x)
		flat.append(t.y)
		flat.append(t.z)
	arr2[Mesh.ARRAY_INDEX] = flat
	am.add_surface_from_arrays(Mesh.PRIMITIVE_TRIANGLES, arr2)
	var tm := am.generate_triangle_mesh()
	var dirs: Array = []
	for x in [-1, 0, 1]:
		for y in [-1, 0, 1]:
			for z in [-1, 0, 1]:
				if x != 0 or y != 0 or z != 0:
					dirs.append(Vector3(x, y, z).normalized())
	var keep := {}
	var kept_tris := 0
	var dropped := 0
	for ck in ckeys:
		var tl: Array = comps[ck]
		var verts := {}
		for ti in tl:
			verts[UT[ti].x] = true
			verts[UT[ti].y] = true
			verts[UT[ti].z] = true
		var exposed := false
		for d in dirs:
			var best := -INF
			var bv := Vector3.ZERO
			for vi in verts:
				var s: float = vp[vi].dot(d)
				if s > best:
					best = s
					bv = vp[vi]
			var hit: Dictionary = tm.intersect_ray(bv + d * 2e-4, d)
			if hit.is_empty():
				exposed = true
				break
		if exposed:
			keep[ck] = true
			kept_tris += tl.size()
		else:
			dropped += 1
			print("  dropped enclosed component with %d triangles" % tl.size())
	print("components kept %d, dropped %d, kept triangles %d" % [keep.size(), dropped, kept_tris])
	# write OBJ (scratch)
	var newid := {}
	var ov := PackedVector3Array()
	var of: Array = []
	for ck in ckeys:
		if not keep.has(ck):
			continue
		for ti in comps[ck]:
			var t: Vector3i = UT[ti]
			var f3 := []
			for vi in [t.x, t.y, t.z]:
				if not newid.has(vi):
					newid[vi] = ov.size()
					ov.append(vp[vi])
				f3.append(int(newid[vi]) + 1)
			of.append(f3)
	var f := FileAccess.open(out, FileAccess.WRITE)
	for v in ov:
		f.store_line("v %.9f %.9f %.9f" % [v.x, v.y, v.z])
	for t in of:
		f.store_line("f %d %d %d" % t)
	f.close()
	print("wrote %s: %d vertices, %d triangles" % [out, ov.size(), of.size()])
	print("RESULT: PASS")
	quit(0)


func _weld(pts: PackedVector3Array, tris: PackedInt32Array) -> Array:
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
	return [wp, T]


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


func _read_usdz(elf: PackedByteArray, path: String) -> Array:
	var sb = ClassDB.instantiate("Sandbox")
	sb.memory_max = 1024
	sb.references_max = 4096
	sb.allocations_max = 1000000
	sb.execution_timeout = 600000
	root.add_child(sb)
	sb.load_buffer(elf)
	sb.vmcall("usd_init")
	var pkg := FileAccess.get_file_as_bytes(path)
	var at := 0
	while at < pkg.size():
		sb.vmcall("usd_push", pkg.slice(at, min(at + 524288, pkg.size())))
		at += 524288
	sb.vmcall("usd_open_staged")
	var bp: PackedFloat32Array = sb.vmcall("usd_mesh_points", 0)
	var bi: PackedInt32Array = sb.vmcall("usd_mesh_indices", 0)
	var v := PackedVector3Array()
	for i in bp.size() / 3:
		v.append(Vector3(bp[i * 3], bp[i * 3 + 1], bp[i * 3 + 2]))
	sb.queue_free()
	return [v, bi]
