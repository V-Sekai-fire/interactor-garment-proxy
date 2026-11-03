# Simplify an OBJ with remesh.elf's simplify (meshoptimizer, original vertices kept).
#   godot --headless --path rproj --script run_remesh.gd -- <remesh.elf> <in.obj> <out.obj> <target_triangles> [target_error] [options]
extends SceneTree

func _initialize() -> void:
	var a := OS.get_cmdline_user_args()
	if a.size() < 4:
		push_error("usage: -- <remesh.elf> <in.obj> <out.obj> <target_triangles> [target_error] [options]")
		quit(2); return
	var pos := PackedVector3Array()
	var idx := PackedInt32Array()
	for line in FileAccess.get_file_as_string(a[1]).split("\n"):
		var t := line.split(" ", false)
		if t.size() >= 4 and t[0] == "v":
			pos.append(Vector3(float(t[1]), float(t[2]), float(t[3])))
		elif t.size() >= 4 and t[0] == "f":
			idx.append(int(t[1].split("/")[0]) - 1); idx.append(int(t[2].split("/")[0]) - 1); idx.append(int(t[3].split("/")[0]) - 1)
	var sb = ClassDB.instantiate("Sandbox")
	sb.memory_max = 1024
	sb.execution_timeout = 600000
	sb.references_max = 4096
	sb.allocations_max = 1000000
	sb.load_buffer(FileAccess.get_file_as_bytes(a[0]))
	var target_err := float(a[4]) if a.size() > 4 else 1.0
	var opts := int(a[5]) if a.size() > 5 else 8   # meshopt_SimplifyPrune
	print("in: %d vertices, %d triangles" % [pos.size(), idx.size() / 3])
	print("remesh.elf: ", sb.vmcall("version"))
	var mode := OS.get_environment("REMESH_MODE")
	var r
	if mode == "remesh":
		# remesh(positions, indices, resolution, options, target_triangles) -> [positions, indices, error]
		r = sb.vmcall("remesh", pos, idx, int(OS.get_environment("REMESH_RES")), opts, int(a[3]))
		if r == null or typeof(r) != TYPE_ARRAY:
			push_error("remesh returned %s" % [str(r)]); quit(1); return
		pos = r[0]
		r = [r[1], r[2]]
	else:
		r = sb.vmcall("simplify", pos, idx, PackedFloat32Array(), PackedFloat32Array(), int(a[3]) * 3, target_err, opts)
		if r == null or typeof(r) != TYPE_ARRAY:
			push_error("simplify returned %s" % [str(r)]); quit(1); return
	var out_idx: PackedInt32Array = r[1] if false else r[0]
	var remap := {}
	var verts := PackedVector3Array()
	var f := FileAccess.open(a[2], FileAccess.WRITE)
	var faces := PackedStringArray()
	for i in range(0, out_idx.size(), 3):
		var ids := []
		for k in 3:
			var v := out_idx[i + k]
			if not remap.has(v):
				remap[v] = verts.size(); verts.append(pos[v])
			ids.append(int(remap[v]) + 1)
		faces.append("f %d %d %d" % ids)
	for v in verts: f.store_line("v %.9f %.9f %.9f" % [v.x, v.y, v.z])
	for s in faces: f.store_line(s)
	f.close()
	print("out: %d vertices, %d triangles, relative error %s" % [verts.size(), out_idx.size() / 3, str(r[1])])
	print("RESULT: PASS")
	quit(0)
