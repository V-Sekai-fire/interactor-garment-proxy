extends SceneTree
# How much of a skirt cloth-fit displacement is a few global parameters about the waist axis (no bones)?
# Models (p = vertex, a = waist axis point, R = yaw about +y, all in Unity world metres, y up):
#   M1  p' = a + t + R diag(sx, 1, sz) (p - a)                        6 parameters
#   M2  same with sy free                                              7
#   M3  sx = sz = s(h), s piecewise linear over 8 height bands         4 + 8 = 12
#   M3b M3 with sy free                                                13
#   M3c sx(h), sz(h) independent per band                              4 + 16 = 20
#   M4  p' = a + t + A (p - a), A a free 3x3 matrix                    12 (closed form, shows shear)
# h = 0 at the waist (top of the mesh), 1 at the hem (lowest vertex).
# Reads the exporter .bin and the cloth-fit deltas JSON. No file is written. Nonlinear models: Levenberg-Marquardt on numeric Jacobians.
# usage: godot --headless --path <proj> -s res://skirt_global_fit.gd -- <garment_own_X.bin> <deltas.json> [bands]

var P0: PackedVector3Array
var T: PackedVector3Array
var a_pt := Vector3.ZERO
var y_top := 0.0
var y_low := 0.0
var nbands := 8

func _init() -> void:
	var args := OS.get_cmdline_user_args()
	var f := FileAccess.open(args[0], FileAccess.READ)
	var n := f.get_32()
	f.get_32()
	P0.resize(n)
	for i in n:
		P0[i] = Vector3(f.get_float(), f.get_float(), f.get_float())
	var js = JSON.parse_string(FileAccess.get_file_as_string(args[1]))
	T.resize(n)
	for i in n:
		T[i] = P0[i] + Vector3(js["deltas_unity_m"][i][0], js["deltas_unity_m"][i][1], js["deltas_unity_m"][i][2])
	if args.size() > 2:
		nbands = int(args[2])
	y_top = -1e9
	y_low = 1e9
	for p in P0:
		y_top = maxf(y_top, p.y)
		y_low = minf(y_low, p.y)
	# waist axis: vertical (+y) through the x,z centroid of the highest 5 percent of the vertices (the waist ring)
	var ys := PackedFloat64Array()
	for p in P0:
		ys.append(p.y)
	ys.sort()
	var thr := ys[int(0.95 * (n - 1))]
	var c := Vector3.ZERO
	var cnt := 0
	for p in P0:
		if p.y >= thr:
			c += p
			cnt += 1
	c /= cnt
	a_pt = Vector3(c.x, y_top, c.z)
	print("skirt %d vertices; y from %.4f (hem) to %.4f (waist), height %.4f m; waist axis through x=%.4f z=%.4f (centroid of the top 5%% of vertices, %d of them), vertical = +y" % [n, y_low, y_top, y_top - y_low, c.x, c.z, cnt])
	var base := 0.0
	for i in n:
		base += (T[i] - P0[i]).length_squared()
	print("cloth-fit movement: mean %.4f m, mean vector (%.4f, %.4f, %.4f)" % [_mean_len(P0, T), _mean_vec().x, _mean_vec().y, _mean_vec().z])
	# M4 closed form
	_affine(base)
	# nonlinear models
	_run("M1  t(3) yaw sx sz (sy=1)", 6, base)
	_run("M2  t(3) yaw sx sy sz", 7, base)
	_run("M3  t(3) yaw s(h) x%d bands (sx=sz, sy=1)" % nbands, 4 + nbands, base)
	_run("M3b t(3) yaw s(h) x%d bands + sy" % nbands, 5 + nbands, base)
	_run("M3c t(3) yaw sx(h), sz(h) x%d bands" % nbands, 4 + 2 * nbands, base)
	quit()

func _mean_len(a: PackedVector3Array, b: PackedVector3Array) -> float:
	var s := 0.0
	for i in a.size():
		s += a[i].distance_to(b[i])
	return s / a.size()

func _mean_vec() -> Vector3:
	var m := Vector3.ZERO
	for i in P0.size():
		m += T[i] - P0[i]
	return m / P0.size()

func _hband(y: float) -> float:
	return clampf((y_top - y) / maxf(y_top - y_low, 1e-9), 0.0, 1.0)

# piecewise-linear s(h) through nbands values at band centres
func _prof(vals: PackedFloat64Array, off: int, h: float) -> float:
	var x := h * nbands - 0.5
	var i0 := clampi(floori(x), 0, nbands - 1)
	var i1 := clampi(i0 + 1, 0, nbands - 1)
	var w := clampf(x - float(i0), 0.0, 1.0)
	if x < 0.0:
		return vals[off]
	return vals[off + i0] * (1.0 - w) + vals[off + i1] * w

func _apply(model: int, p: PackedFloat64Array, v: Vector3) -> Vector3:
	var t := Vector3(p[0], p[1], p[2])
	var yaw := p[3]
	var sx := 1.0
	var sy := 1.0
	var sz := 1.0
	var d := v - a_pt
	var h := _hband(v.y)
	match model:
		6:
			sx = p[4]
			sz = p[5]
		7:
			sx = p[4]
			sy = p[5]
			sz = p[6]
		_:
			pass
	if model == 4 + nbands:
		sx = _prof(p, 4, h)
		sz = sx
	elif model == 5 + nbands:
		sx = _prof(p, 4, h)
		sz = sx
		sy = p[4 + nbands]
	elif model == 4 + 2 * nbands:
		sx = _prof(p, 4, h)
		sz = _prof(p, 4 + nbands, h)
	var s := Vector3(d.x * sx, d.y * sy, d.z * sz)
	var cy := cos(yaw)
	var sn := sin(yaw)
	var r := Vector3(cy * s.x + sn * s.z, s.y, -sn * s.x + cy * s.z)
	return a_pt + t + r

func _cost(model: int, p: PackedFloat64Array) -> float:
	var c := 0.0
	for i in P0.size():
		c += (_apply(model, p, P0[i]) - T[i]).length_squared()
	return c

func _init_params(model: int) -> PackedFloat64Array:
	var p := PackedFloat64Array()
	p.resize(model)
	for i in model:
		p[i] = 1.0 if i >= 4 else 0.0
	return p

func _run(label: String, model: int, base: float) -> void:
	var p := _init_params(model)
	var lam := 1e-3
	var cost := _cost(model, p)
	var it := 0
	for _k in 80:
		it += 1
		var np := model
		var jtj := PackedFloat64Array()
		jtj.resize(np * np)
		var jtr := PackedFloat64Array()
		jtr.resize(np)
		var n := P0.size()
		var cols: Array = []
		for k in np:
			var q := p.duplicate()
			var h := 1e-6
			q[k] += h
			var col := PackedVector3Array()
			col.resize(n)
			for i in n:
				col[i] = (_apply(model, q, P0[i]) - _apply(model, p, P0[i])) / h
			cols.append(col)
		for i in n:
			var r := _apply(model, p, P0[i]) - T[i]
			for a in np:
				var ja: Vector3 = cols[a][i]
				jtr[a] += ja.dot(r)
				for b in range(a, np):
					jtj[a * np + b] += ja.dot(cols[b][i])
		for a in np:
			for b in range(a):
				jtj[a * np + b] = jtj[b * np + a]
		var improved := false
		for _t in 12:
			var m := jtj.duplicate()
			for a in np:
				m[a * np + a] += lam * maxf(jtj[a * np + a], 1e-12)
			var dlt := _solve(m, jtr, np)
			var q := p.duplicate()
			for a in np:
				q[a] -= dlt[a]
			var c2 := _cost(model, q)
			if c2 < cost:
				var rel := (cost - c2) / cost
				p = q
				cost = c2
				lam = maxf(lam * 0.3, 1e-9)
				improved = true
				if rel < 1e-9:
					improved = false
				break
			lam *= 4.0
		if not improved:
			break
	var e := PackedFloat64Array()
	for i in P0.size():
		e.append((_apply(model, p, P0[i]) - T[i]).length())
	e.sort()
	var mean := 0.0
	for v in e:
		mean += v
	mean /= e.size()
	print("%-52s %2d params, %2d iterations: explained %.2f%%  error mean %.4f p99 %.4f max %.4f m" % [label, model, it, 100.0 * (1.0 - cost / base), mean, e[int(0.99 * (e.size() - 1))], e[e.size() - 1]])
	var s := "      t = (%.4f, %.4f, %.4f) m, yaw %.3f deg" % [p[0], p[1], p[2], rad_to_deg(p[3])]
	if model == 6:
		s += ", sx %.4f sz %.4f" % [p[4], p[5]]
	elif model == 7:
		s += ", sx %.4f sy %.4f sz %.4f" % [p[4], p[5], p[6]]
	elif model == 4 + nbands or model == 5 + nbands:
		var bs := PackedStringArray()
		for k in nbands:
			bs.append("%.3f" % p[4 + k])
		s += ", s(waist..hem) = " + ", ".join(bs)
		if model == 5 + nbands:
			s += ", sy %.4f" % p[4 + nbands]
	else:
		var bx := PackedStringArray()
		var bz := PackedStringArray()
		for k in nbands:
			bx.append("%.3f" % p[4 + k])
			bz.append("%.3f" % p[4 + nbands + k])
		s += ", sx(waist..hem) = " + ", ".join(bx) + "; sz = " + ", ".join(bz)
	print(s)
	# hem drop and side differences of the residual
	var hem := Vector3(a_pt.x, y_low, a_pt.z)
	var moved := _apply(model, p, hem)
	print("      the hem point under the waist axis goes from y %.4f to y %.4f (drop %.4f m), radial offset %.4f m" % [hem.y, moved.y, hem.y - moved.y, Vector2(moved.x - a_pt.x, moved.z - a_pt.z).length()])
	var left := Vector3.ZERO
	var right := Vector3.ZERO
	var nl := 0
	var nr := 0
	for i in P0.size():
		var r := T[i] - _apply(model, p, P0[i])
		if P0[i].x > a_pt.x:
			left += r
			nl += 1
		else:
			right += r
			nr += 1
	left /= nl
	right /= nr
	print("      mean residual (target - model) by side: x>axis (%+.4f, %+.4f, %+.4f), x<axis (%+.4f, %+.4f, %+.4f) m" % [left.x, left.y, left.z, right.x, right.y, right.z])

func _affine(base: float) -> void:
	# p' = a + t + A (p - a): per output coordinate a 4-vector (A row, t) by normal equations
	var n := P0.size()
	var m := PackedFloat64Array()
	m.resize(16)
	var rhs: Array = [PackedFloat64Array([0, 0, 0, 0]), PackedFloat64Array([0, 0, 0, 0]), PackedFloat64Array([0, 0, 0, 0])]
	for i in n:
		var d := P0[i] - a_pt
		var x := [d.x, d.y, d.z, 1.0]
		var tg := T[i] - a_pt
		for r in 4:
			for c in 4:
				m[r * 4 + c] += x[r] * x[c]
			for k in 3:
				rhs[k][r] += x[r] * tg[k]
	var sol: Array = []
	for k in 3:
		sol.append(_solve(m, rhs[k], 4))
	var cost := 0.0
	var e := PackedFloat64Array()
	for i in n:
		var d := P0[i] - a_pt
		var q := Vector3(sol[0][0] * d.x + sol[0][1] * d.y + sol[0][2] * d.z + sol[0][3], sol[1][0] * d.x + sol[1][1] * d.y + sol[1][2] * d.z + sol[1][3], sol[2][0] * d.x + sol[2][1] * d.y + sol[2][2] * d.z + sol[2][3]) + a_pt
		cost += (q - T[i]).length_squared()
		e.append((q - T[i]).length())
	e.sort()
	var mean := 0.0
	for v in e:
		mean += v
	mean /= n
	print("%-52s 12 params (closed form): explained %.2f%%  error mean %.4f p99 %.4f max %.4f m" % ["M4  free 3x3 matrix + t (shows shear)", 100.0 * (1.0 - cost / base), mean, e[int(0.99 * (n - 1))], e[n - 1]])
	print("      A (rows x,y,z; columns x,y,z): [%.4f %.4f %.4f] [%.4f %.4f %.4f] [%.4f %.4f %.4f], t = (%.4f, %.4f, %.4f) m" % [sol[0][0], sol[0][1], sol[0][2], sol[1][0], sol[1][1], sol[1][2], sol[2][0], sol[2][1], sol[2][2], sol[0][3], sol[1][3], sol[2][3]])
	print("      horizontal block [[Axx Axz],[Azx Azz]] = [[%.4f %.4f],[%.4f %.4f]]; antisymmetric part (yaw) %.4f, symmetric off-diagonal (shear) %.4f; vertical coupling Ayx %.4f Ayz %.4f, Axy %.4f Azy %.4f" % [sol[0][0], sol[0][2], sol[2][0], sol[2][2], 0.5 * (sol[0][2] - sol[2][0]), 0.5 * (sol[0][2] + sol[2][0]), sol[1][0], sol[1][2], sol[0][1], sol[2][1]])

func _solve(m: PackedFloat64Array, b: PackedFloat64Array, n: int) -> PackedFloat64Array:
	var a := m.duplicate()
	var x := b.duplicate()
	for c in n:
		var piv := c
		for r in range(c + 1, n):
			if absf(a[r * n + c]) > absf(a[piv * n + c]):
				piv = r
		if piv != c:
			for k in n:
				var t := a[c * n + k]
				a[c * n + k] = a[piv * n + k]
				a[piv * n + k] = t
			var tb := x[c]
			x[c] = x[piv]
			x[piv] = tb
		var d := a[c * n + c]
		if absf(d) < 1e-300:
			continue
		for r in range(c + 1, n):
			var f := a[r * n + c] / d
			if f == 0.0:
				continue
			for k in range(c, n):
				a[r * n + k] -= f * a[c * n + k]
			x[r] -= f * x[c]
	for c in range(n - 1, -1, -1):
		var s := x[c]
		for k in range(c + 1, n):
			s -= a[c * n + k] * x[k]
		x[c] = s / a[c * n + c] if absf(a[c * n + c]) > 1e-300 else 0.0
	return x
