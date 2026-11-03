extends SceneTree
# cloth-fit no_fit_spec_path: 0-based ids of the garment vertices that need not fit tightly (the loose, hanging parts), on one line.
# The garment is in the cloth-fit frame (Z up). Rule: Z below a height. Either --below-z=<Z> or --height-fraction=<f>
# (Z < zmin + f * (zmax - zmin), the "below 84% of the skirt's height" rule of the garment-data example).
# usage: -- <garment_red.obj> <out.txt> (--below-z=<Z> | --height-fraction=<f>)
# prints "vertices N, no-fit M, threshold Z"


func _init() -> void:
	var a := OS.get_cmdline_user_args()
	var zs := PackedFloat64Array()
	for l in FileAccess.get_file_as_string(a[0]).split("\n"):
		if l.begins_with("v "):
			zs.append(float(l.split(" ", false)[3]))
	var thr := 0.0
	for s in a:
		if s.begins_with("--below-z="):
			thr = float(s.substr(10))
		elif s.begins_with("--height-fraction="):
			var lo := INF
			var hi := -INF
			for z in zs:
				lo = minf(lo, z)
				hi = maxf(hi, z)
			thr = lo + float(s.substr(18)) * (hi - lo)
	var ids := PackedStringArray()
	for i in zs.size():
		if zs[i] < thr:
			ids.append(str(i))
	if ids.is_empty():
		ids.append("0")
	var f := FileAccess.open(a[1], FileAccess.WRITE)
	f.store_line(" ".join(ids))
	f.close()
	print("vertices %d, no-fit %d, threshold %.6f" % [zs.size(), ids.size(), thr])
	quit(0)
