extends SceneTree
# Run the real curvenet extractor (modules/cassie CassieCurvenetExtractor) on the cleaned garments, with
# tolerances taken from the body mesh, and write each garment's curves as OpenUSD (BasisCurves, .usdz).
# Garments come from csg_clean_usd.gd (.usdz, wholly enclosed shells already dropped); they are read with usd.elf.
#
# usage: godot --headless --path <proj> -s res://curvenet_extract.gd -- <usd.elf> <out_dir> <h_m> <curvature_weight> <target_curves> <garment.usdz> [...]
# rdp_error = fit_error = h_m (the body's median edge length).


func _init() -> void:
	var a := OS.get_cmdline_user_args()
	var elf := FileAccess.get_file_as_bytes(a[0])
	var out_dir: String = a[1]
	var h: float = float(a[2])
	var cw: float = float(a[3])
	var target: int = int(a[4])
	DirAccess.make_dir_recursive_absolute(out_dir)
	var rep := FileAccess.open(out_dir.path_join("report.txt"), FileAccess.WRITE)
	rep.store_line("# rdp_error=fit_error=%f m, curvature_weight=%f, target_curve_count=%d" % [h, cw, target])
	rep.store_line("garment\tverts\ttris\tcurves\tcurve_points\tnodes\tsecs\tusdz_bytes\treadback")
	print("%-12s %7s %7s %7s %9s %6s %7s %10s  %s" % ["garment", "verts", "tris", "curves", "curve_pts", "nodes", "secs", "usdz_bytes", "readback"])
	for gi in range(5, a.size()):
		var name: String = String(a[gi]).get_file().get_basename()
		var row := _one(elf, a[gi], name, h, cw, target, out_dir)
		print("%-12s %7d %7d %7d %9d %6d %7.1f %10d  %s" % [name, row[0], row[1], row[2], row[3], row[4], row[5], row[6], row[7]])
		rep.store_line("%s\t%d\t%d\t%d\t%d\t%d\t%.1f\t%d\t%s" % [name, row[0], row[1], row[2], row[3], row[4], row[5], row[6], row[7]])
	rep.close()
	quit(0)


func _sandbox(elf: PackedByteArray) -> Object:
	var sb = ClassDB.instantiate("Sandbox")
	sb.memory_max = 1024
	sb.references_max = 4096
	sb.allocations_max = 1000000
	sb.execution_timeout = 600000
	root.add_child(sb)
	sb.load_buffer(elf)
	sb.vmcall("usd_init")
	return sb


func _one(elf: PackedByteArray, path: String, name: String, h: float, cw: float, target: int, out_dir: String) -> Array:
	var sb = _sandbox(elf)
	var pkg := FileAccess.get_file_as_bytes(path)
	var at := 0
	while at < pkg.size():
		sb.vmcall("usd_push", pkg.slice(at, min(at + 524288, pkg.size())))
		at += 524288
	sb.vmcall("usd_open_staged")
	var pts = sb.vmcall("usd_mesh_points", 0)
	var idx = sb.vmcall("usd_mesh_indices", 0)
	sb.queue_free()
	if typeof(pts) != TYPE_PACKED_FLOAT32_ARRAY or typeof(idx) != TYPE_PACKED_INT32_ARRAY:
		return [0, 0, 0, 0, 0, 0.0, 0, "garment read failed: " + str(pts)]
	var v := PackedVector3Array()
	for i in pts.size() / 3:
		v.append(Vector3(pts[i * 3], pts[i * 3 + 1], pts[i * 3 + 2]))
	var arr := []
	arr.resize(Mesh.ARRAY_MAX)
	arr[Mesh.ARRAY_VERTEX] = v
	arr[Mesh.ARRAY_INDEX] = idx
	var mesh := ArrayMesh.new()
	mesh.add_surface_from_arrays(Mesh.PRIMITIVE_TRIANGLES, arr)
	var patch := CassieSurfacePatch.new()
	patch.mesh = mesh
	var t0 := Time.get_ticks_msec()
	var g: Dictionary = CassieCurvenetExtractor.extract_graph_data(patch, target, h, h, cw)
	var secs := (Time.get_ticks_msec() - t0) / 1000.0
	var curves: Array = g.get("curves", [])
	var nodes: Array = g.get("nodes", [])
	# Curves as polylines for OpenUSD BasisCurves: points, per-curve counts.
	var cp := PackedFloat32Array()
	var counts := PackedInt32Array()
	var names := PackedStringArray()
	var total := 0
	for k in curves.size():
		var c: Curve3D = curves[k].get_curve()
		var poly := c.tessellate(5, 0.0005)
		if poly.size() < 2:
			continue
		for p in poly:
			cp.append(p.x)
			cp.append(p.y)
			cp.append(p.z)
		counts.append(poly.size())
		names.append("c%04d" % k)
		total += poly.size()
	var nbytes := 0
	var rb := "none"
	if counts.size() > 0:
		var sb2 = _sandbox(elf)
		var out = sb2.vmcall("usd_write_curves", cp, counts, PackedInt32Array(), "\n".join(names),
				"source=%s\nrdp_error=%f\nfit_error=%f\ncurvature_weight=%f\ntarget_curve_count=%d" % [name, h, h, cw, target])
		if typeof(out) != TYPE_STRING or String(out).begins_with("ERR"):
			rb = "ERR: " + String(out).substr(0, 60)
		else:
			# usd_write_curves answers .usda text; store it as the layer next to the report.
			rb = "usda written (not read back)"
		var layer_path := out_dir.path_join(name + ".curves.usda")
		var f := FileAccess.open(layer_path, FileAccess.WRITE)
		f.store_string(String(out))
		f.close()
		nbytes = String(out).length()
		sb2.queue_free()
	return [v.size(), idx.size() / 3, curves.size(), total, nodes.size(), secs, nbytes, rb]
