extends SceneTree
# Clipping measure with Godot's Manifold CSG: body INTERSECT garment. Manifold refuses a mesh that is not a closed
# manifold, so no GDScript edge pass is needed; the answer is read in constant time from the result:
# overlap yes/no, its triangle count and its bounding box. --volume adds an O(triangles) volume sum.
# godot --headless --path <proj> -s res://csg_clip.gd -- <body.obj> <garment.obj> [--scale=<m per unit>] [--shift=x,y,z] [--volume]
# --shift moves the garment first (planted control: a shift into the body must make the overlap grow).
# Exit 1 if CSG refuses the inputs.
func _init() -> void:
	var a := OS.get_cmdline_user_args()
	var sc := 1.0
	var shift := Vector3.ZERO
	var want_volume := false
	for s in a.slice(2):
		if s.begins_with("--scale="):
			sc = float(s.substr(8))
		elif s.begins_with("--shift="):
			var t := s.substr(8).split(",")
			shift = Vector3(float(t[0]), float(t[1]), float(t[2]))
		elif s == "--volume":
			want_volume = true
	var t0 := Time.get_ticks_msec()
	var body := _mesh(a[0], sc, Vector3.ZERO)
	var garm := _mesh(a[1], sc, shift)
	print("read OBJs %d ms" % (Time.get_ticks_msec() - t0))
	var comb := CSGCombiner3D.new()
	root.add_child(comb)
	var cb := CSGMesh3D.new()
	cb.mesh = body
	comb.add_child(cb)
	var cg := CSGMesh3D.new()
	cg.mesh = garm
	cg.operation = CSGShape3D.OPERATION_INTERSECTION
	comb.add_child(cg)
	await process_frame
	await process_frame
	var t1 := Time.get_ticks_msec()
	var res: Array = comb.get_meshes()
	print("intersection %d ms" % (Time.get_ticks_msec() - t1))
	if res.is_empty() or (res[1] as Mesh).get_surface_count() == 0:
		print("overlap: none (empty intersection; or CSG refused a non-manifold input)")
		quit(0)
		return
	var arr: Array = (res[1] as Mesh).surface_get_arrays(0)
	var v: PackedVector3Array = arr[Mesh.ARRAY_VERTEX]
	var ix = arr[Mesh.ARRAY_INDEX]
	var tris: int = v.size() / 3 if (ix == null or ix.size() == 0) else ix.size() / 3
	var box := comb.get_aabb()
	print("overlap: %d triangles, bounding box %.4f x %.4f x %.4f m" % [tris, box.size.x, box.size.y, box.size.z])
	if want_volume:
		var vol := 0.0
		for t in tris:
			if ix == null or ix.size() == 0:
				vol += v[t * 3].dot(v[t * 3 + 1].cross(v[t * 3 + 2])) / 6.0
			else:
				vol += v[ix[t * 3]].dot(v[ix[t * 3 + 1]].cross(v[ix[t * 3 + 2]])) / 6.0
		print("overlap volume %.6f m^3" % absf(vol))
	quit(0)

func _mesh(path: String, sc: float, shift: Vector3) -> ArrayMesh:
	var v := PackedVector3Array()
	var ix := PackedInt32Array()
	for l in FileAccess.get_file_as_string(path).split("\n"):
		if l.begins_with("v "):
			var t := l.split(" ", false)
			v.append(Vector3(float(t[1]), float(t[2]), float(t[3])) * sc + shift)
		elif l.begins_with("f "):
			var t := l.split(" ", false)
			for k in 3:
				ix.append(int(t[k + 1].split("/")[0]) - 1)
	var arr := []
	arr.resize(Mesh.ARRAY_MAX)
	arr[Mesh.ARRAY_VERTEX] = v
	arr[Mesh.ARRAY_INDEX] = ix
	var m := ArrayMesh.new()
	m.add_surface_from_arrays(Mesh.PRIMITIVE_TRIANGLES, arr)
	return m
