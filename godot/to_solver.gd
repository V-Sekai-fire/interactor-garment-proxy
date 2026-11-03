extends SceneTree
# Unity metres (Y up) -> cloth-fit solver frame: (X,Y,Z) = scale * (x, -z, y). Non-vertex lines are copied.
# usage: -- <in.obj> <out.obj> <scale> [--nofit=<out.txt> --below=<unity y metres>]
# --nofit writes 0-based ids of vertices with Unity y < below on one line (cloth-fit no_fit_spec_path format).


func _init() -> void:
	var pos: Array = []
	var nofit := ""
	var below := -INF
	for s in OS.get_cmdline_user_args():
		if s.begins_with("--nofit="):
			nofit = s.substr(8)
		elif s.begins_with("--below="):
			below = float(s.substr(8))
		else:
			pos.append(s)
	var sc := float(pos[2])
	var o := FileAccess.open(pos[1], FileAccess.WRITE)
	var ids := PackedStringArray()
	var n := 0
	for l in FileAccess.get_file_as_string(pos[0]).split("\n"):
		if l.begins_with("v "):
			var t := l.split(" ", false)
			var x := float(t[1])
			var y := float(t[2])
			var z := float(t[3])
			o.store_line("v %.9f %.9f %.9f" % [sc * x, -sc * z, sc * y])
			if y < below:
				ids.append(str(n))
			n += 1
		elif l.strip_edges() != "":
			o.store_line(l)
	o.close()
	if nofit != "":
		var f := FileAccess.open(nofit, FileAccess.WRITE)
		f.store_line(" ".join(ids))
		f.close()
	print("vertices %d, no-fit %d" % [n, ids.size()])
	quit(0)
