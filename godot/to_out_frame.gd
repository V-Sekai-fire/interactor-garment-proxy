extends SceneTree
# Put an OBJ (solver input frame) into the solver output frame: out = s * (in - ca) + ct, fitted from avatar_red -> target_avatar.
# usage: -- <avatar_red.obj> <target_avatar.obj> <in.obj> <out.obj> [<positions_from.obj>]
# With the 5th argument the output takes its vertex positions from that file (faces from <in.obj>).


func _init() -> void:
	var a := OS.get_cmdline_user_args()
	var AV: PackedVector3Array = _read(a[0])[0]
	var TA: PackedVector3Array = _read(a[1])[0]
	var ca := Vector3.ZERO
	var ct := Vector3.ZERO
	for i in AV.size():
		ca += AV[i]
		ct += TA[i]
	ca /= AV.size()
	ct /= AV.size()
	var num := 0.0
	var den := 0.0
	for i in AV.size():
		num += (AV[i] - ca).dot(TA[i] - ct)
		den += (AV[i] - ca).length_squared()
	var s := num / den
	var r := _read(a[2])
	var P: PackedVector3Array = r[0]
	if a.size() > 4:
		P = _read(a[4])[0]
	else:
		for i in P.size():
			P[i] = s * (P[i] - ca) + ct
	var f := FileAccess.open(a[3], FileAccess.WRITE)
	for v in P:
		f.store_line("v %.12f %.12f %.12f" % [v.x, v.y, v.z])
	for t in r[1].size() / 3:
		f.store_line("f %d %d %d" % [r[1][t * 3] + 1, r[1][t * 3 + 1] + 1, r[1][t * 3 + 2] + 1])
	quit(0)


func _read(path: String) -> Array:
	var v := PackedVector3Array()
	var idx := PackedInt32Array()
	for l in FileAccess.get_file_as_string(path).split("\n"):
		if l.begins_with("v "):
			var t := l.split(" ", false)
			v.append(Vector3(float(t[1]), float(t[2]), float(t[3])))
		elif l.begins_with("f "):
			var t := l.split(" ", false)
			for k in 3:
				idx.append(int(t[k + 1].split("/")[0]) - 1)
	return [v, idx]
