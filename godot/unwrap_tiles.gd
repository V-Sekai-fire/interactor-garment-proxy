extends SceneTree
# UDIM-style unwrap: cut a mesh into K regions (k-means on triangle centres, so limbs and torso separate), unwrap
# each region on its own with Godot's xatlas, and give each region its own tile (its own 0-1 UV space, written as
# a `usemtl tile_<i>` group, i.e. already unpacked the way fbx-udim-unpack leaves a UDIM model).
# godot --headless --path <proj> -s res://unwrap_tiles.gd -- <in.obj> <out.obj> <m per unit> <K> [px per metre]
# Writes <out.obj> (metres, one vt per vertex) and <out.obj>.json: [{tile, atlas_w, atlas_h, board_w, board_h, triangles}]
# board size = atlas size rounded up to whole 45 px squares; vt is already scaled to the board.
func _init() -> void:
	var a := OS.get_cmdline_user_args()
	var sc := float(a[2])
	var K := int(a[3])
	var pxm := float(a[4]) if a.size() > 4 else 1500.0
	var v := PackedVector3Array()
	var ix := PackedInt32Array()
	for l in FileAccess.get_file_as_string(a[0]).split("\n"):
		if l.begins_with("v "):
			var t := l.split(" ", false)
			v.append(Vector3(float(t[1]), float(t[2]), float(t[3])) * sc)
		elif l.begins_with("f "):
			var t := l.split(" ", false)
			for k in 3:
				ix.append(int(t[k + 1].split("/")[0]) - 1)
	var nt := ix.size() / 3
	var cen := PackedVector3Array()
	cen.resize(nt)
	for t in nt:
		cen[t] = (v[ix[t * 3]] + v[ix[t * 3 + 1]] + v[ix[t * 3 + 2]]) / 3.0
	# farthest-first seeds, then Lloyd
	var seeds: Array = [cen[0]]
	while seeds.size() < K:
		var best := -1
		var bd := -1.0
		for t in range(0, nt, 7):
			var dmin := 1e30
			for s in seeds:
				dmin = minf(dmin, cen[t].distance_squared_to(s))
			if dmin > bd:
				bd = dmin
				best = t
		seeds.append(cen[best])
	var lab := PackedInt32Array()
	lab.resize(nt)
	for it in 10:
		var sum: Array = []
		var cnt: Array = []
		for k in K:
			sum.append(Vector3.ZERO)
			cnt.append(0)
		for t in nt:
			var bk := 0
			var bd := 1e30
			for k in K:
				var d := cen[t].distance_squared_to(seeds[k])
				if d < bd:
					bd = d
					bk = k
			lab[t] = bk
			sum[bk] += cen[t]
			cnt[bk] += 1
		for k in K:
			if cnt[k] > 0:
				seeds[k] = sum[k] / cnt[k]
	var out := FileAccess.open(a[1], FileAccess.WRITE)
	var meta := []
	var vbase := 0
	for k in K:
		var remap := {}
		var pv := PackedVector3Array()
		var pi := PackedInt32Array()
		for t in nt:
			if lab[t] != k:
				continue
			for c in 3:
				var o := ix[t * 3 + c]
				if not remap.has(o):
					remap[o] = pv.size()
					pv.append(v[o])
				pi.append(remap[o])
		if pi.is_empty():
			continue
		var arr := []
		arr.resize(Mesh.ARRAY_MAX)
		arr[Mesh.ARRAY_VERTEX] = pv
		arr[Mesh.ARRAY_INDEX] = pi
		var m0 := ArrayMesh.new()
		m0.add_surface_from_arrays(Mesh.PRIMITIVE_TRIANGLES, arr)
		var sm := SurfaceTool.new()
		sm.create_from(m0, 0)
		sm.generate_normals()
		var mesh := sm.commit()
		mesh.lightmap_unwrap(Transform3D.IDENTITY, 1.0 / pxm)
		var ar := mesh.surface_get_arrays(0)
		var ov: PackedVector3Array = ar[Mesh.ARRAY_VERTEX]
		var oi: PackedInt32Array = ar[Mesh.ARRAY_INDEX]
		var uv2: PackedVector2Array = ar[Mesh.ARRAY_TEX_UV2]
		var hint := Vector2(mesh.lightmap_size_hint)
		var bw := int(ceil(hint.x / 45.0)) * 45
		var bh := int(ceil(hint.y / 45.0)) * 45
		out.store_line("usemtl tile_%d" % k)
		for p in ov:
			out.store_line("v %.6f %.6f %.6f" % [p.x, p.y, p.z])
		for u in uv2:
			out.store_line("vt %.6f %.6f" % [u.x * hint.x / bw, u.y * hint.y / bh])
		for t in oi.size() / 3:
			var i0 := oi[t * 3] + 1 + vbase
			var i1 := oi[t * 3 + 1] + 1 + vbase
			var i2 := oi[t * 3 + 2] + 1 + vbase
			out.store_line("f %d/%d %d/%d %d/%d" % [i0, i0, i1, i1, i2, i2])
		vbase += ov.size()
		meta.append({"tile": k, "atlas_w": hint.x, "atlas_h": hint.y, "board_w": bw, "board_h": bh, "triangles": oi.size() / 3})
		print("tile ", k, ": ", oi.size() / 3, " tris, atlas ", hint, " verts ", ov.size())
	out.close()
	var jf := FileAccess.open(a[1] + ".json", FileAccess.WRITE)
	jf.store_string(JSON.stringify(meta))
	quit(0)
