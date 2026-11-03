extends SceneTree
# OBJ (scratch interchange) -> .usdz with usd.elf usd_write_mesh, read back and compared bit for bit.
# usage: -- <usd.elf> <in.obj> <out.usdz> <name> [--solver=<scale>]   (--solver: input is in the cloth-fit frame, X=s*x, Y=-s*z, Z=s*y; written in Unity metres)


func _init() -> void:
	var a := OS.get_cmdline_user_args()
	var sc := 0.0
	for s in a:
		if s.begins_with("--solver="):
			sc = float(s.substr(9))
	var kp := PackedFloat32Array()
	var kt := PackedInt32Array()
	for l in FileAccess.get_file_as_string(a[1]).split("\n"):
		if l.begins_with("v "):
			var t := l.split(" ", false)
			var x := float(t[1])
			var y := float(t[2])
			var z := float(t[3])
			if sc > 0.0:
				var ux := x / sc
				var uy := z / sc
				var uz := -y / sc
				x = ux
				y = uy
				z = uz
			kp.append(x)
			kp.append(y)
			kp.append(z)
		elif l.begins_with("f "):
			var t := l.split(" ", false)
			for k in 3:
				kt.append(int(t[k + 1].split("/")[0]) - 1)
	var sb = ClassDB.instantiate("Sandbox")
	sb.memory_max = 1024
	sb.references_max = 4096
	sb.allocations_max = 1000000
	sb.execution_timeout = 600000
	root.add_child(sb)
	sb.load_buffer(FileAccess.get_file_as_bytes(a[0]))
	sb.vmcall("usd_init")
	var nf := kt.size() / 3
	var ints := PackedInt32Array()
	ints.resize(nf)
	ints.fill(3)
	ints.append_array(kt)
	var header := "%s\nnp=%d\nnuv=0\nnrest=0\nnf=%d\nni=%d\nsource=%s" % [a[3], kp.size() / 3, nf, kt.size(), a[3]]
	var pkg = sb.vmcall("usd_write_mesh", kp, ints, header, "")
	var f := FileAccess.open(a[2], FileAccess.WRITE)
	f.store_buffer(pkg)
	f.close()
	var at := 0
	while at < pkg.size():
		sb.vmcall("usd_push", pkg.slice(at, min(at + 524288, pkg.size())))
		at += 524288
	sb.vmcall("usd_open_staged")
	var bp = sb.vmcall("usd_mesh_points", 0)
	var bi = sb.vmcall("usd_mesh_indices", 0)
	print("%s: %d points, %d triangles, %d bytes; readback identical: %s" % [a[2].get_file(), kp.size() / 3, nf, pkg.size(), str(bp.to_byte_array() == kp.to_byte_array() and bi == kt)])
	quit(0)
