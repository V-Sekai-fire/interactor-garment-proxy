extends SceneTree
# How much of a cloth-fit displacement is the target skeleton's rest-pose offset: o_v = sum_j W[j][v] * (target_j - source_j)
# (the exporter's 21-joint weights and skeleton OBJs), compared with the delta d_v. Reading OBJ/JSON inputs only.
# usage: godot --headless --path <proj> -s res://rest_offset.gd -- <export dir> <piece e.g. ASkirt> <deltas.json>
func _verts(path: String) -> PackedVector3Array:
	var o := PackedVector3Array()
	for l in FileAccess.get_file_as_string(path).split("\n"):
		if l.begins_with("v "):
			var p := l.split(" ")
			o.append(Vector3(float(p[1]), float(p[2]), float(p[3])))
	return o
func _init() -> void:
	var a := OS.get_cmdline_user_args()
	var src := _verts(a[0].path_join("source_skeleton.obj"))
	var tgt := _verts(a[0].path_join("target_skeleton.obj"))
	var names := FileAccess.get_file_as_string(a[0].path_join("joints.txt")).split("\n", false)
	var rows := FileAccess.get_file_as_string(a[0].path_join("garment_%s_weights.txt" % a[1])).split("\n", false)
	var js = JSON.parse_string(FileAccess.get_file_as_string(a[2]))
	var dl: Array = js["deltas_unity_m"]
	var n: int = dl.size()
	print("joint offsets target - source (m):")
	for j in src.size():
		var o := tgt[j] - src[j]
		print("  %-12s %+.4f %+.4f %+.4f" % [names[j], o.x, o.y, o.z])
	var off := PackedVector3Array()
	off.resize(n)
	for j in rows.size():
		var w := rows[j].split(" ")
		var o := tgt[j] - src[j]
		for v in n:
			off[v] += o * float(w[v])
	var sd := 0.0
	var sr := 0.0
	var mo := Vector3.ZERO
	var md := Vector3.ZERO
	var sw := 0.0
	for v in n:
		var d := Vector3(dl[v][0], dl[v][1], dl[v][2])
		sd += d.length_squared()
		sr += (d - off[v]).length_squared()
		mo += off[v]
		md += d
	print("vertices %d: mean delta (%.4f, %.4f, %.4f), mean skeleton-offset displacement (%.4f, %.4f, %.4f)" % [n, md.x / n, md.y / n, md.z / n, mo.x / n, mo.y / n, mo.z / n])
	print("sum |d|^2 %.4f, sum |d - o|^2 %.4f: the skeleton rest offset alone explains %.2f percent of the squared displacement" % [sd, sr, 100.0 * (1.0 - sr / sd)])
	var ys := PackedFloat64Array()
	for v in n:
		ys.append(absf(off[v].y))
	ys.sort()
	print("|offset y| per vertex: median %.4f max %.4f" % [ys[n / 2], ys[n - 1]])
	quit()
