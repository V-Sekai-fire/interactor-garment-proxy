extends SceneTree
# UDIM-style unwrap with one continuous chart per region: cut a mesh into K regions (k-means on triangle centres, so
# limbs and torso separate), develop each region onto a plane at real size (a cylinder, or a cone when the radius
# changes along the axis: both flatten with no stretch, with one wrap seam placed in the emptiest direction), and give
# each region its own tile (written as a `usemtl tile_<i>` group, the form fbx-udim-unpack leaves a UDIM model in).
# godot --headless --path <proj> -s res://unwrap_tiles.gd -- <in.obj> <out.obj> <m per unit> <K> [px per metre]
# Writes <out.obj> (metres, one vt per vertex) and <out.obj>.json: [{tile, atlas_w, atlas_h, board_w, board_h, triangles}]
# board size = chart size rounded up to whole 45 px squares; vt is already scaled to the board. Prints the area stretch
# (UV area / surface area) as mean and 5th/95th percentile.
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
		var res := _develop(pv, pi, pxm)
		var ov: PackedVector3Array = res[0]
		var oi: PackedInt32Array = res[1]
		var uvp: PackedVector2Array = res[2]   # metres on the development plane, origin at the tile corner
		var hint := Vector2(ceil(res[3].x * pxm), ceil(res[3].y * pxm))
		var bw := int(ceil(hint.x / 45.0)) * 45
		var bh := int(ceil(hint.y / 45.0)) * 45
		out.store_line("usemtl tile_%d" % k)
		for p in ov:
			out.store_line("v %.6f %.6f %.6f" % [p.x, p.y, p.z])
		for u in uvp:
			out.store_line("vt %.6f %.6f" % [u.x * pxm / bw, u.y * pxm / bh])
		for t in oi.size() / 3:
			var i0 := oi[t * 3] + 1 + vbase
			var i1 := oi[t * 3 + 1] + 1 + vbase
			var i2 := oi[t * 3 + 2] + 1 + vbase
			out.store_line("f %d/%d %d/%d %d/%d" % [i0, i0, i1, i1, i2, i2])
		vbase += ov.size()
		meta.append({"tile": k, "atlas_w": hint.x, "atlas_h": hint.y, "board_w": bw, "board_h": bh, "triangles": oi.size() / 3})
		print("tile ", k, ": ", oi.size() / 3, " tris, chart ", hint, " px, verts ", ov.size(), ", ", res[4])
	out.close()
	var jf := FileAccess.open(a[1] + ".json", FileAccess.WRITE)
	jf.store_string(JSON.stringify(meta))
	quit(0)


# Develops one region onto a plane. Returns [vertices, indices, uv (metres), extent (metres), stats text].
func _develop(pv: PackedVector3Array, pi: PackedInt32Array, pxm: float) -> Array:
	var n := pv.size()
	var c := Vector3.ZERO
	for p in pv:
		c += p
	c /= n
	# principal axis by power iteration on the covariance
	var cov := [[0.0, 0.0, 0.0], [0.0, 0.0, 0.0], [0.0, 0.0, 0.0]]
	for p in pv:
		var d := p - c
		for r in 3:
			for q in 3:
				cov[r][q] += d[r] * d[q]
	var ax := Vector3(1, 0.3, 0.2).normalized()
	for it in 40:
		var nx := Vector3(0, 0, 0)
		for r in 3:
			nx[r] = cov[r][0] * ax.x + cov[r][1] * ax.y + cov[r][2] * ax.z
		ax = nx.normalized()
	var e2 := ax.cross(Vector3.UP if absf(ax.dot(Vector3.UP)) < 0.9 else Vector3.RIGHT).normalized()
	var e3 := ax.cross(e2)
	var s := PackedFloat32Array()
	var th := PackedFloat32Array()
	var rad := PackedFloat32Array()
	s.resize(n)
	th.resize(n)
	rad.resize(n)
	var hist := PackedInt32Array()
	hist.resize(36)
	for i in n:
		var d := pv[i] - c
		s[i] = d.dot(ax)
		var a2 := d.dot(e2)
		var a3 := d.dot(e3)
		th[i] = atan2(a3, a2)
		rad[i] = sqrt(a2 * a2 + a3 * a3)
		hist[clampi(int((th[i] + PI) / TAU * 36.0), 0, 35)] += 1
	# seam in the emptiest direction (widest run of empty bins, else the emptiest bin)
	var best := 0
	var bestc := 1 << 30
	for b in 36:
		var cnt := hist[b] * 4 + hist[(b + 1) % 36] + hist[(b + 35) % 36]
		if cnt < bestc:
			bestc = cnt
			best = b
	var seam := -PI + (best + 0.5) / 36.0 * TAU
	for i in n:
		th[i] = fposmod(th[i] - seam, TAU)   # 0..TAU, the wrap seam sits at 0 / TAU
	# radius along the axis: r = r0 + k s (least squares); a cone when the slope is real
	var sm := 0.0
	var rm := 0.0
	for i in n:
		sm += s[i]
		rm += rad[i]
	sm /= n
	rm /= n
	var sxx := 0.0
	var sxr := 0.0
	for i in n:
		sxx += (s[i] - sm) * (s[i] - sm)
		sxr += (s[i] - sm) * (rad[i] - rm)
	var kslope := sxr / maxf(sxx, 1e-9)
	var r0 := rm - kslope * sm
	var cone := absf(kslope) > 0.05
	var s_apex := -r0 / kslope if cone else 0.0
	var sin_a := 1.0
	if cone:
		var acc := 0.0
		for i in n:
			var rho := sqrt((s[i] - s_apex) * (s[i] - s_apex) + rad[i] * rad[i])
			acc += rad[i] / maxf(rho, 1e-9)
		sin_a = acc / n
	# split vertices that sit across the wrap: a triangle spanning more than PI gets the low angles shifted by TAU
	var outv := PackedVector3Array()
	var outuv := PackedVector2Array()
	var key := {}
	var oi := PackedInt32Array()
	for t in pi.size() / 3:
		var ids := [pi[t * 3], pi[t * 3 + 1], pi[t * 3 + 2]]
		var tmin := minf(th[ids[0]], minf(th[ids[1]], th[ids[2]]))
		var tmax := maxf(th[ids[0]], maxf(th[ids[1]], th[ids[2]]))
		var wrap := (tmax - tmin) > PI
		for id in ids:
			var shift: int = 1 if (wrap and th[id] < PI) else 0
			var kk: int = id * 2 + shift
			if not key.has(kk):
				key[kk] = outv.size()
				var ang: float = th[id] + TAU * shift
				var uv: Vector2
				if cone:
					var rho := sqrt((s[id] - s_apex) * (s[id] - s_apex) + rad[id] * rad[id])
					var phi := ang * sin_a
					uv = Vector2(rho * cos(phi), rho * sin(phi))
				else:
					uv = Vector2(ang * rm, s[id])
				outv.append(pv[id])
				outuv.append(uv)
			oi.append(key[kk])
	var lo := Vector2(1e30, 1e30)
	var hi := Vector2(-1e30, -1e30)
	for uv in outuv:
		lo = Vector2(minf(lo.x, uv.x), minf(lo.y, uv.y))
		hi = Vector2(maxf(hi.x, uv.x), maxf(hi.y, uv.y))
	for i in outuv.size():
		outuv[i] -= lo - Vector2(0.01, 0.01)
	var ext := hi - lo + Vector2(0.02, 0.02)
	# stretch: UV area over surface area per triangle
	var ratios := PackedFloat32Array()
	for t in oi.size() / 3:
		var a := outv[oi[t * 3]]
		var b := outv[oi[t * 3 + 1]]
		var cc := outv[oi[t * 3 + 2]]
		var area3 := 0.5 * (b - a).cross(cc - a).length()
		var ua := outuv[oi[t * 3]]
		var ub := outuv[oi[t * 3 + 1]]
		var uc := outuv[oi[t * 3 + 2]]
		var area2 := 0.5 * absf((ub - ua).cross(uc - ua))
		if area3 > 1e-12:
			ratios.append(area2 / area3)
	ratios.sort()
	var mean := 0.0
	for r in ratios:
		mean += r
	mean /= maxi(ratios.size(), 1)
	var txt := "%s, stretch mean %.2f p5 %.2f p95 %.2f" % ["cone" if cone else "cylinder", mean, ratios[int(ratios.size() * 0.05)], ratios[int(ratios.size() * 0.95)]]
	return [outv, oi, outuv, ext, txt]
