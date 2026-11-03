extends SceneTree
# Write the shells of an OBJ (index adjacency) whose rank by triangle count (0 = largest) is in the list.
# usage: -- <in.obj> <out.obj> <rank,rank,...> | --drop=<rank,...>   (keep these ranks, or keep all but these; ranks are written to <out.obj>.ranks)
# prints "shells in: N, kept: M, triangles out: T"


func _init() -> void:
	var a := OS.get_cmdline_user_args()
	var P := PackedVector3Array()
	var idx := PackedInt32Array()
	for l in FileAccess.get_file_as_string(a[0]).split("\n"):
		if l.begins_with("v "):
			var t := l.split(" ", false)
			P.append(Vector3(float(t[1]), float(t[2]), float(t[3])))
		elif l.begins_with("f "):
			var t := l.split(" ", false)
			for k in 3:
				idx.append(int(t[k + 1].split("/")[0]) - 1)
	var par := PackedInt32Array()
	for i in P.size():
		par.append(i)
	for t in idx.size() / 3:
		_union(par, idx[t * 3], idx[t * 3 + 1])
		_union(par, idx[t * 3 + 1], idx[t * 3 + 2])
	var size := {}
	for t in idx.size() / 3:
		var r := _find(par, idx[t * 3])
		size[r] = int(size.get(r, 0)) + 1
	var order: Array = size.keys()
	order.sort_custom(func(x, y): return size[x] > size[y] or (size[x] == size[y] and x < y))
	var rank := {}
	var rf := FileAccess.open(a[1] + ".ranks", FileAccess.WRITE)
	for i in order.size():
		rank[order[i]] = i
		rf.store_line("rank %d: %d triangles" % [i, size[order[i]]])
	rf.close()
	var want := {}
	if a[2].begins_with("--drop="):
		for i in order.size():
			want[i] = true
		for s in a[2].substr(7).split(",", false):
			want.erase(int(s))
	else:
		for s in a[2].split(",", false):
			want[int(s)] = true
	var newid := {}
	var vl := PackedStringArray()
	var fl := PackedStringArray()
	for t in idx.size() / 3:
		if not want.has(rank[_find(par, idx[t * 3])]):
			continue
		var ids := []
		for k in 3:
			var v := idx[t * 3 + k]
			if not newid.has(v):
				newid[v] = newid.size()
				vl.append("v %.9f %.9f %.9f" % [P[v].x, P[v].y, P[v].z])
			ids.append(int(newid[v]) + 1)
		fl.append("f %d %d %d" % ids)
	print("shells in: %d, kept: %d, triangles out: %d" % [order.size(), want.size(), fl.size()])
	var f := FileAccess.open(a[1], FileAccess.WRITE)
	for l in vl:
		f.store_line(l)
	for l in fl:
		f.store_line(l)
	quit(0)


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
