extends SceneTree
# Per-garment CSG union, then the cleaned garment as OpenUSD.
# For each garment: weld, split into shells, thicken each shell into a closed solid tagged with a colour id,
# CSG-union all solids (dev/csg-vertex-attributes carries the colour through Manifold), and drop the shells
# whose id is missing from the union surface: those lie wholly inside the union of the others.
# The kept welded triangles are written with usd.elf's usd_write_mesh as a .usdz, read back with
# usd_open_staged, and compared bit for bit. A planted one-ulp change must be caught.
# A garment whose union is refused by Manifold keeps all its shells and is flagged.
#
# usage: godot --headless --path <proj> -s res://csg_clean_usd.gd -- <usd.elf> <out_dir> <thickness_m> <garment.obj> [...]
# (garment .obj files are the Unity exporter's existing output; they are only read.)


func _init() -> void:
	var a := OS.get_cmdline_user_args()
	var elf := FileAccess.get_file_as_bytes(a[0])
	var out_dir: String = a[1]
	var thick: float = float(a[2])
	DirAccess.make_dir_recursive_absolute(out_dir)
	var rep := FileAccess.open(out_dir.path_join("report.txt"), FileAccess.WRITE)
	rep.store_line("garment\tshells\tenclosed\ttriangles_in\ttriangles_out\tusdz_bytes\tunion\tusd_readback\tcontrol")
	var fails := 0
	print("%-12s %6s %8s %9s %9s %10s  %-8s %-10s %-8s" % ["garment", "shells", "enclosed", "tris_in", "tris_out", "usdz_bytes", "union", "readback", "control"])
	for gi in range(3, a.size()):
		var name: String = String(a[gi]).get_file().get_basename().trim_prefix("garment_")
		var r: Dictionary = await _garment(elf, name, a[gi], thick, out_dir)
		var ok: bool = r["identical"] and r["control"]
		if not ok:
			fails += 1
		var line := "%-12s %6d %8d %9d %9d %10d  %-8s %-10s %-8s" % [name, r["shells"], r["enclosed"], r["tris_in"], r["tris_out"], r["bytes"],
				r["union"], "identical" if r["identical"] else "DIFFERS", "caught" if r["control"] else "MISSED"]
		print(line)
		rep.store_line("%s\t%d\t%d\t%d\t%d\t%d\t%s\t%s\t%s" % [name, r["shells"], r["enclosed"], r["tris_in"], r["tris_out"], r["bytes"],
				r["union"], "identical" if r["identical"] else "DIFFERS", "caught" if r["control"] else "MISSED"])
	rep.close()
	print("TOTAL: ", a.size() - 3, " garments, ", fails, " failed")
	quit(1 if fails > 0 else 0)


func _garment(elf: PackedByteArray, name: String, src: String, thick: float, out_dir: String) -> Dictionary:
	var res := {"shells": 0, "enclosed": 0, "tris_in": 0, "tris_out": 0, "bytes": 0, "union": "ok", "identical": false, "control": false}
	var raw := _read(src)
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
	res["shells"] = n_shells
	res["tris_in"] = T.size()
	# CSG union with colour ids.
	var enclosed := {}
	if n_shells > 1:
		var comb := CSGCombiner3D.new()
		root.add_child(comb)
		for i in n_shells:
			var c := CSGMesh3D.new()
			c.mesh = _solid(wp, shells[keys[i]], thick, i, n_shells)
			c.operation = CSGShape3D.OPERATION_UNION
			comb.add_child(c)
		await process_frame
		await process_frame
		var got: Array = comb.get_meshes()
		var colors = null
		if not got.is_empty():
			var arr = (got[1] as Mesh).surface_get_arrays(0)
			colors = arr[Mesh.ARRAY_COLOR]
			var nv: int = (arr[Mesh.ARRAY_VERTEX] as PackedVector3Array).size()
			if colors == null or colors.size() != nv:
				colors = null
		if colors == null:
			res["union"] = "refused"
		else:
			var alive := {}
			for c in colors:
				var id := roundi(c.r * float(n_shells + 1)) - 1
				if id >= 0 and id < n_shells:
					alive[id] = true
			for i in n_shells:
				if not alive.has(i):
					enclosed[i] = true
		comb.queue_free()
	else:
		res["union"] = "single"
	res["enclosed"] = enclosed.size()
	# Kept welded mesh: positions as float32, triangles of the surviving shells.
	var newid := {}
	var kp := PackedFloat32Array()
	var kt := PackedInt32Array()
	for i in n_shells:
		if enclosed.has(i):
			continue
		for t in shells[keys[i]]:
			for vi in [t.x, t.y, t.z]:
				if not newid.has(vi):
					newid[vi] = kp.size() / 3
					kp.append(wp[vi].x)
					kp.append(wp[vi].y)
					kp.append(wp[vi].z)
				kt.append(newid[vi])
	res["tris_out"] = kt.size() / 3
	# OpenUSD write-out and read-back.
	var sb = ClassDB.instantiate("Sandbox")
	sb.memory_max = 1024
	sb.references_max = 4096
	sb.allocations_max = 1000000
	sb.execution_timeout = 600000
	root.add_child(sb)
	sb.load_buffer(elf)
	sb.vmcall("usd_init")
	var nf := kt.size() / 3
	var ints := PackedInt32Array()
	ints.resize(nf)
	ints.fill(3)
	ints.append_array(kt)
	var header := "%s\nnp=%d\nnuv=0\nnrest=0\nnf=%d\nni=%d\nsource=Maple garment, CSG union cleaned (%d of %d shells enclosed)" % [name, kp.size() / 3, nf, kt.size(), enclosed.size(), n_shells]
	var pkg = sb.vmcall("usd_write_mesh", kp, ints, header, "")
	if typeof(pkg) != TYPE_PACKED_BYTE_ARRAY:
		sb.queue_free()
		return res
	res["bytes"] = pkg.size()
	var f := FileAccess.open(out_dir.path_join(name + ".usdz"), FileAccess.WRITE)
	f.store_buffer(pkg)
	f.close()
	var at := 0
	while at < pkg.size():
		sb.vmcall("usd_push", pkg.slice(at, min(at + 524288, pkg.size())))
		at += 524288
	sb.vmcall("usd_open_staged")
	var bp = sb.vmcall("usd_mesh_points", 0)
	var bi = sb.vmcall("usd_mesh_indices", 0)
	if typeof(bp) == TYPE_PACKED_FLOAT32_ARRAY and typeof(bi) == TYPE_PACKED_INT32_ARRAY:
		res["identical"] = bp.to_byte_array() == kp.to_byte_array() and bi == kt
		if kp.size() > 0:
			var b := kp.to_byte_array()
			var k2 := kp.size() / 2
			b[k2 * 4] = b[k2 * 4] ^ 1
			res["control"] = bp.to_byte_array() != b
	sb.queue_free()
	return res


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
