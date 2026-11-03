extends SceneTree
# Find the shells of a garment that are wholly enclosed by other shells.
# Each welded shell is thickened into a closed solid (inner copy + rim strip), tagged with a colour id,
# and all solids are CSG-unioned. The engine carries vertex colour through the Manifold boolean
# (dev/csg-vertex-attributes), so an id that is missing from the union surface belongs to a shell
# that lies entirely inside the union of the others. The shell count is whatever the mesh has.
#
# usage: godot --headless --path <proj> -s res://csg_enclosed.gd -- <garment.obj> <report.txt> [thickness_m]

const THICK := 0.004


func _init() -> void:
	var a := OS.get_cmdline_user_args()
	var src: String = a[0]
	var out: String = a[1]
	var thick: float = float(a[2]) if a.size() > 2 else THICK
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
	var n_shells: int = shells.size()
	print("welded ", wp.size(), " vertices, ", T.size(), " triangles, ", n_shells, " shells")
	var comb := CSGCombiner3D.new()
	root.add_child(comb)
	var shell_tris := PackedInt32Array()
	var bad := 0
	var sid := 0
	for r in shells:
		var m := _solid(wp, shells[r], thick, sid, n_shells)
		var chk := _check(m)
		if chk[0] != 0 or chk[1] != 0 or chk[2] <= 0.0:
			bad += 1
		shell_tris.append(shells[r].size())
		var c := CSGMesh3D.new()
		c.mesh = m
		c.operation = CSGShape3D.OPERATION_UNION
		comb.add_child(c)
		sid += 1
	print("solids ", sid, ", invalid inputs ", bad)
	await process_frame
	await process_frame
	var res: Array = comb.get_meshes()
	if res.is_empty():
		print("RESULT: empty (union refused)")
		quit(1)
		return
	var mesh: Mesh = res[1]
	var arr := mesh.surface_get_arrays(0)
	var col = arr[Mesh.ARRAY_COLOR]
	var ri = arr[Mesh.ARRAY_INDEX]
	var rv: PackedVector3Array = arr[Mesh.ARRAY_VERTEX]
	if col == null or col.size() != rv.size():
		print("RESULT: union carried no vertex colour (is this the dev/csg-vertex-attributes engine?)")
		quit(2)
		return
	var alive := {}
	var unknown := 0
	for c in col:
		var id := roundi(c.r * float(n_shells + 1)) - 1
		if id < 0 or id >= n_shells:
			unknown += 1
		else:
			alive[id] = true
	var enclosed: Array = []
	for i in n_shells:
		if not alive.has(i):
			enclosed.append(i)
	var out_tris: int = (ri.size() / 3) if (ri != null and ri.size() > 0) else (rv.size() / 3)
	print("RESULT: union has ", out_tris, " triangles; ", alive.size(), " of ", n_shells, " shells survive; ",
			enclosed.size(), " enclosed; vertices with an unreadable id ", unknown, " of ", col.size())
	# Cleaned garment: the welded mesh without the triangles of the enclosed shells.
	if a.size() > 3:
		var drop := {}
		for i in enclosed:
			drop[i] = true
		var keys: Array = shells.keys()
		var newid := {}
		var kv := PackedVector3Array()
		var kt: Array = []
		for i in keys.size():
			if drop.has(i):
				continue
			for t in shells[keys[i]]:
				var tri := PackedInt32Array()
				for vi in [t.x, t.y, t.z]:
					if not newid.has(vi):
						newid[vi] = kv.size()
						kv.append(wp[vi])
					tri.append(newid[vi])
				kt.append(tri)
		# Saved as a Godot mesh resource (.res), not a text interchange file: the curvenet extractor
		# (modules/cassie) runs in the engine and loads it directly.
		var cix := PackedInt32Array()
		for tri in kt:
			cix.append_array(tri)
		var carr := []
		carr.resize(Mesh.ARRAY_MAX)
		carr[Mesh.ARRAY_VERTEX] = kv
		carr[Mesh.ARRAY_INDEX] = cix
		var cm := ArrayMesh.new()
		cm.add_surface_from_arrays(Mesh.PRIMITIVE_TRIANGLES, carr)
		var err := ResourceSaver.save(cm, a[3])
		print("CLEAN: saved ", a[3].get_file(), " (err ", err, "): ", kt.size(), " triangles, ", kv.size(), " vertices (dropped ", T.size() - kt.size(), " triangles of ", enclosed.size(), " enclosed shells)")
	var f := FileAccess.open(out, FileAccess.WRITE)
	f.store_line("garment %s" % src.get_file())
	f.store_line("shells %d" % n_shells)
	f.store_line("invalid_input_solids %d" % bad)
	f.store_line("surviving %d" % alive.size())
	f.store_line("enclosed %d" % enclosed.size())
	f.store_line("thickness_m %f" % thick)
	f.store_line("# shell_id triangles (enclosed shells only)")
	for i in enclosed:
		f.store_line("%d %d" % [i, shell_tris[i]])
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
	# Shell id, scaled into (0, 1] so it survives as a float colour: r = (id + 1) / (shells + 1).
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
