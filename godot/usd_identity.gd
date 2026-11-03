extends SceneTree
# Identity write-out through OpenUSD: each garment goes through usd.elf's usd_write_mesh unchanged (no
# welding, no shell removal), the package is read back with usd_open_staged, and every point and index
# is compared bit for bit. A planted one-ulp change must be caught, or the check proves nothing.
# The .usdz is saved next to the report. No text interchange format is written.
#
# usage: godot --headless --path <proj> -s res://usd_identity.gd -- <usd.elf> <out_dir> <garment.obj> [<garment.obj> ...]
# (the garment .obj files are the Unity exporter's existing output; they are only read.)


func _init() -> void:
	var a := OS.get_cmdline_user_args()
	var elf_path: String = a[0]
	var out_dir: String = a[1]
	DirAccess.make_dir_recursive_absolute(out_dir)
	var elf := FileAccess.get_file_as_bytes(elf_path)
	var failed := 0
	print("%-14s %8s %8s %10s  %-9s %-9s %s" % ["garment", "points", "tris", "usdz_bytes", "identity", "control", "result"])
	for gi in range(2, a.size()):
		var name: String = String(a[gi]).get_file().get_basename().trim_prefix("garment_")
		var raw := _read_obj(a[gi])
		var pts: PackedFloat32Array = raw[0]
		var tris: PackedInt32Array = raw[1]
		var res := _roundtrip(elf, name, pts, tris, out_dir)
		var pass_ok: bool = res["identical"] and res["control_caught"]
		if not pass_ok:
			failed += 1
		print("%-14s %8d %8d %10d  %-9s %-9s %s" % [name, pts.size() / 3, tris.size() / 3, res["bytes"],
				"identical" if res["identical"] else "DIFFERS", "caught" if res["control_caught"] else "MISSED",
				"PASS" if pass_ok else ("FAIL: " + String(res["why"]))])
	print("TOTAL: ", a.size() - 2, " garments, ", failed, " failed")
	quit(1 if failed > 0 else 0)


func _roundtrip(elf: PackedByteArray, name: String, pts: PackedFloat32Array, tris: PackedInt32Array, out_dir: String) -> Dictionary:
	var r := {"identical": false, "control_caught": false, "bytes": 0, "why": ""}
	var sb = ClassDB.instantiate("Sandbox")
	sb.memory_max = 1024
	sb.references_max = 4096
	sb.allocations_max = 1000000
	sb.execution_timeout = 600000
	root.add_child(sb)
	sb.load_buffer(elf)
	sb.vmcall("usd_init")
	var nf := tris.size() / 3
	var ints := PackedInt32Array()
	ints.resize(nf)
	ints.fill(3)
	ints.append_array(tris)
	var header := "%s\nnp=%d\nnuv=0\nnrest=0\nnf=%d\nni=%d\nsource=Unity fit export, identity write-out" % [name, pts.size() / 3, nf, tris.size()]
	var pkg = sb.vmcall("usd_write_mesh", pts, ints, header, "")
	if typeof(pkg) != TYPE_PACKED_BYTE_ARRAY:
		r["why"] = "usd_write_mesh answered: " + str(pkg)
		sb.queue_free()
		return r
	r["bytes"] = pkg.size()
	var f := FileAccess.open(out_dir.path_join(name + ".usdz"), FileAccess.WRITE)
	f.store_buffer(pkg)
	f.close()
	var chunk := 512 * 1024
	var at := 0
	while at < pkg.size():
		sb.vmcall("usd_push", pkg.slice(at, min(at + chunk, pkg.size())))
		at += chunk
	var opened = sb.vmcall("usd_open_staged")
	var bp = sb.vmcall("usd_mesh_points", 0)
	var bi = sb.vmcall("usd_mesh_indices", 0)
	if typeof(bp) != TYPE_PACKED_FLOAT32_ARRAY or typeof(bi) != TYPE_PACKED_INT32_ARRAY:
		r["why"] = "read-back types: " + str(opened) + " | " + str(bp)
		sb.queue_free()
		return r
	r["identical"] = _same_bits(bp, pts) and bi == tris
	if not r["identical"]:
		r["why"] = "points %d/%d, indices %d/%d" % [bp.size(), pts.size(), bi.size(), tris.size()]
	# Planted control: flip the lowest mantissa bit of one input float; the comparison must notice.
	if pts.size() > 0:
		var k := pts.size() / 2
		var bytes := pts.to_byte_array()
		bytes[k * 4] = bytes[k * 4] ^ 1
		var planted := bytes.to_float32_array()
		r["control_caught"] = not _same_bits(bp, planted)
	sb.queue_free()
	return r


func _same_bits(x: PackedFloat32Array, y: PackedFloat32Array) -> bool:
	return x.size() == y.size() and x.to_byte_array() == y.to_byte_array()


func _read_obj(path: String) -> Array:
	var v := PackedFloat32Array()
	var idx := PackedInt32Array()
	var f := FileAccess.open(path, FileAccess.READ)
	while not f.eof_reached():
		var l := f.get_line()
		if l.begins_with("v "):
			var t := l.split(" ", false)
			v.append(float(t[1]))
			v.append(float(t[2]))
			v.append(float(t[3]))
		elif l.begins_with("f "):
			var t := l.split(" ", false)
			for k in 3:
				idx.append(int(t[k + 1].split("/")[0]) - 1)
	return [v, idx]
