extends SceneTree
# Map the cloth-fit displacement from the garment proxy back onto the original garment vertices (Unity metres).
# Frame: the solver output is a similarity of its input: out = s * (in - ca) + ct (scale and translation; fitted from
# avatar_red.obj against target_avatar.obj, which keep the input vertex order; residual printed).
# Per original vertex: closest point on the INITIAL proxy (remesh.elf closest) -> barycentrics -> interpolate the proxy displacement.
# usage: -- <remesh.elf> <proxy_unity.obj> <fitted_proxy_solver.obj> <avatar_red.obj> <target_avatar.obj> <garment_own.bin> <out_deltas.bin> <scale>
# garment_own.bin layout (ChibifireFitExport): int32 nv, int32 ntri, float32 xyz * nv, int32 idx * 3 * ntri.


func _init() -> void:
	var a := OS.get_cmdline_user_args()
	var elf := FileAccess.get_file_as_bytes(a[0])
	var P0s: PackedVector3Array = _read(a[1])[0]
	var F: PackedInt32Array = _read(a[1])[1]
	var PFs: PackedVector3Array = _read(a[2])[0]
	var AV: PackedVector3Array = _read(a[3])[0]
	var TA: PackedVector3Array = _read(a[4])[0]
	var sc := float(a[7])
	var P0 := PackedVector3Array()
	for p in P0s:
		P0.append(Vector3(p.x / sc, p.z / sc, -p.y / sc))
	assert(AV.size() == TA.size() and PFs.size() == P0.size())
	# similarity fit
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
	var res := 0.0
	for i in AV.size():
		res = maxf(res, (s * (AV[i] - ca) + ct - TA[i]).length())
	print("frame: s=%.9f ca=%s ct=%s max residual %.9f (solver units)" % [s, ca, ct, res])
	# fitted proxy back in the input frame, then to Unity metres
	var PF := PackedVector3Array()
	for i in PFs.size():
		var x := (PFs[i] - ct) / s + ca
		PF.append(Vector3(x.x / sc, x.z / sc, -x.y / sc))
	var D := PackedVector3Array()
	var dm := 0.0
	var dmean := 0.0
	for i in P0.size():
		D.append(PF[i] - P0[i])
		dm = maxf(dm, D[i].length())
		dmean += D[i].length()
	print("proxy displacement (Unity m): mean %.4f max %.4f over %d proxy vertices" % [dmean / P0.size(), dm, P0.size()])
	# original vertices
	var fb := FileAccess.open(a[5], FileAccess.READ)
	var nv := fb.get_32()
	var nt := fb.get_32()
	var orig := PackedVector3Array()
	for i in nv:
		orig.append(Vector3(fb.get_float(), fb.get_float(), fb.get_float()))
	print("original garment: %d vertices, %d triangles" % [nv, nt])
	var sb = ClassDB.instantiate("Sandbox")
	sb.memory_max = 1024
	sb.references_max = 4096
	sb.allocations_max = 1000000
	sb.execution_timeout = 600000
	root.add_child(sb)
	sb.load_buffer(elf)
	var cl: Array = sb.vmcall("closest", P0, F, orig)
	var tri: PackedInt32Array = cl[0]
	var bary: PackedFloat32Array = cl[1]
	var out := PackedFloat32Array()
	var dist_max := 0.0
	var dist_mean := 0.0
	var mag_max := 0.0
	var mag_mean := 0.0
	var dists: Array = []
	var ymin := INF
	var ymax := -INF
	for i in nv:
		var t := tri[i]
		var d := Vector3.ZERO
		var q := Vector3.ZERO
		for k in 3:
			var vi := F[t * 3 + k]
			d += bary[i * 3 + k] * D[vi]
			q += bary[i * 3 + k] * P0[vi]
		var dist := q.distance_to(orig[i])
		dists.append(dist)
		dist_max = maxf(dist_max, dist)
		dist_mean += dist
		mag_max = maxf(mag_max, d.length())
		mag_mean += d.length()
		ymin = minf(ymin, d.y)
		ymax = maxf(ymax, d.y)
		out.append(d.x)
		out.append(d.y)
		out.append(d.z)
	dists.sort()
	print("query->proxy distance percentiles (m): p50 %.4f p90 %.4f p99 %.4f; vertices farther than 0.03 m: %d" % [dists[nv / 2], dists[nv * 9 / 10], dists[nv * 99 / 100], dists.filter(func(x): return x > 0.03).size()])
	print("query->proxy distance (m): max %.4f mean %.4f" % [dist_max, dist_mean / nv])
	print("garment delta (m): max %.4f mean %.4f; dy min %.4f max %.4f" % [mag_max, mag_mean / nv, ymin, ymax])
	# planted control: query = proxy vertices themselves must give distance ~0 and delta = D
	var sub := PackedVector3Array()
	for i in range(0, P0.size(), 7):
		sub.append(P0[i])
	var cl2: Array = sb.vmcall("closest", P0, F, sub)
	var worst_d := 0.0
	var worst_pos := 0.0
	for j in sub.size():
		var t: int = cl2[0][j]
		var d := Vector3.ZERO
		var q := Vector3.ZERO
		for k in 3:
			var vi := F[t * 3 + k]
			d += cl2[1][j * 3 + k] * D[vi]
			q += cl2[1][j * 3 + k] * P0[vi]
		worst_d = maxf(worst_d, (d - D[j * 7]).length())
		worst_pos = maxf(worst_pos, q.distance_to(sub[j]))
	print("control (proxy vertices as queries): max position miss %.9f m, max delta miss %.9f m" % [worst_pos, worst_d])
	var fo := FileAccess.open(a[6], FileAccess.WRITE)
	fo.store_buffer(out.to_byte_array())
	fo.close()
	print("wrote %s (%d float32)" % [a[6], out.size()])
	# target positions in the exporter .bin layout (original + delta), triangles copied
	var fb2 := FileAccess.open(a[5], FileAccess.READ)
	var raw_all := fb2.get_buffer(fb2.get_length())
	var tgt := FileAccess.open(a[6].get_basename() + "_target.bin", FileAccess.WRITE)
	tgt.store_32(nv)
	tgt.store_32(nt)
	for i in nv:
		tgt.store_float(orig[i].x + out[i * 3])
		tgt.store_float(orig[i].y + out[i * 3 + 1])
		tgt.store_float(orig[i].z + out[i * 3 + 2])
	tgt.store_buffer(raw_all.slice(8 + nv * 12))
	tgt.close()
	# left/right mirror symmetry of the displacement: partner of vertex i = nearest original vertex to (-x, y, z)
	var cell := 0.004
	var grid := {}
	for i in nv:
		var k := Vector3i(floori(orig[i].x / cell), floori(orig[i].y / cell), floori(orig[i].z / cell))
		if not grid.has(k):
			grid[k] = []
		grid[k].append(i)
	var n_pairs := 0
	var sum_dx := 0.0
	var sum_abs_dx := 0.0
	var sum_abs_dy := 0.0
	var sum_abs_dz := 0.0
	var sum_mir_x := 0.0
	var n_right := 0
	var sum_dx_right := 0.0
	var sum_dx_left := 0.0
	var n_left := 0
	for i in nv:
		if orig[i].x <= 0.002:
			continue
		var q := Vector3(-orig[i].x, orig[i].y, orig[i].z)
		var best := -1
		var bd := 0.003
		var c0 := Vector3i(floori(q.x / cell), floori(q.y / cell), floori(q.z / cell))
		for dx in range(-1, 2):
			for dy in range(-1, 2):
				for dz in range(-1, 2):
					var kk := Vector3i(c0.x + dx, c0.y + dy, c0.z + dz)
					if grid.has(kk):
						for j in grid[kk]:
							var d := orig[j].distance_to(q)
							if d < bd:
								bd = d
								best = j
		if best < 0:
			continue
		n_pairs += 1
		var di := Vector3(out[i * 3], out[i * 3 + 1], out[i * 3 + 2])
		var dj := Vector3(out[best * 3], out[best * 3 + 1], out[best * 3 + 2])
		sum_abs_dx += absf(dj.x + di.x)
		sum_abs_dy += absf(dj.y - di.y)
		sum_abs_dz += absf(dj.z - di.z)
		sum_dx_right += di.x
		sum_dx_left += dj.x
		sum_mir_x += absf(di.x)
	print("symmetry: %d mirror pairs (x>0 vertex and its partner at (-x,y,z) within 3 mm); mean dx of x>0 side %.5f m, mean dx of mirrored x<0 side %.5f m" % [n_pairs, sum_dx_right / maxf(1, n_pairs), sum_dx_left / maxf(1, n_pairs)])
	print("symmetry: mean |dx_left - mirrored dx_right| = mean |dx_L + dx_R| = %.5f m; mean |dy_L - dy_R| = %.5f m; mean |dz_L - dz_R| = %.5f m; mean |dx| = %.5f m" % [sum_abs_dx / maxf(1, n_pairs), sum_abs_dy / maxf(1, n_pairs), sum_abs_dz / maxf(1, n_pairs), sum_mir_x / maxf(1, n_pairs)])
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
