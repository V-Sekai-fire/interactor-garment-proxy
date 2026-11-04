extends SceneTree
# Orbit-view contact sheet that makes the proxy's defects visible: shells coloured, defects in red.
# godot --path . -s res://contact_sheet2.gd -- <dir with g_col.bin p_col.bin defects.bin> <out.png> [--control=blank]

const THUMB := Vector2i(480, 480)
const LABEL_W := 250
const HEAD_H := 190
const COL_H := 52
const BG := Color(0.20, 0.20, 0.24)
const VIEWS := 8
const FOV := 30.0

func _init() -> void:
	var a := OS.get_cmdline_user_args()
	var dir: String = a[0]
	var out: String = a[1]
	var control: bool = a.size() > 2 and a[2] == "--control=blank"
	var g := _load(dir + "/g_col.bin")
	var p := _load(dir + "/p_col.bin")
	var dfx := _load(dir + "/defects.bin")
	# column 0: garment by shell; column 1: proxy by shell; column 2: proxy grey plus red defects
	var cols := [
		{"title": "garment: each shell its own colour (main shell grey)", "meshes": [[g[0], true, false]]},
		{"title": "proxy: each shell its own colour (main shell grey)", "meshes": [[p[0], true, false]]},
		{"title": "proxy grey; red = zero-area triangle or shared position", "meshes": [[p[0], false, false], [dfx[0], true, true]]},
	]
	var box: AABB = g[0].get_aabb().merge(p[0].get_aabb())
	var centre: Vector3 = box.get_center()
	var dist: float = box.size.length() * 0.5 / sin(deg_to_rad(FOV * 0.5)) * 0.8
	var views := _hammersley(VIEWS)
	var cells: Array = []
	for c in cols.size():
		var vp := SubViewport.new()
		vp.size = THUMB
		vp.own_world_3d = true
		vp.render_target_update_mode = SubViewport.UPDATE_DISABLED
		var env := Environment.new()
		env.background_mode = Environment.BG_COLOR
		env.background_color = BG
		env.ambient_light_source = Environment.AMBIENT_SOURCE_COLOR
		env.ambient_light_color = Color(0.5, 0.5, 0.55)
		var we := WorldEnvironment.new()
		we.environment = env
		vp.add_child(we)
		for m in cols[c].meshes:
			var mi := MeshInstance3D.new()
			mi.mesh = m[0]
			var mat := StandardMaterial3D.new()
			mat.cull_mode = BaseMaterial3D.CULL_DISABLED
			mat.vertex_color_use_as_albedo = m[1]
			mat.albedo_color = Color(0.62, 0.62, 0.62) if not m[1] else Color(1, 1, 1)
			if m[2]:
				mat.shading_mode = BaseMaterial3D.SHADING_MODE_UNSHADED
			mi.material_override = mat
			vp.add_child(mi)
		var cam := Camera3D.new()
		cam.fov = FOV
		cam.near = 0.05
		cam.far = 20.0
		vp.add_child(cam)
		var light := DirectionalLight3D.new()
		cam.add_child(light)
		root.add_child(vp)
		cells.append([vp, cam])
	var rows: Array = []
	for v in views.size():
		var az: float = views[v][0]
		var el: float = views[v][1]
		var dirv := Vector3(sin(deg_to_rad(az)) * cos(deg_to_rad(el)), sin(deg_to_rad(el)), -cos(deg_to_rad(az)) * cos(deg_to_rad(el)))   # Pixal3D is Z-up: (x, y, z) -> (x, z, -y) for this Y-up scene
		var row := {"view": v, "az": az, "el": el, "img": [], "lit": [], "red": 0}
		for c in cols.size():
			var cam: Camera3D = cells[c][1]
			var eye: Vector3 = centre + dirv * dist
			cam.look_at_from_position(eye, centre if not (control and c == 2) else eye + dirv, Vector3.UP if absf(dirv.y) < 0.999 else Vector3(0, 0, 1))
			var img: Image
			var lit := 0
			var red := 0
			var seen := 0
			for attempt in 3:
				cells[c][0].render_target_update_mode = SubViewport.UPDATE_ONCE
				for f in 12:
					await process_frame
				await RenderingServer.frame_post_draw
				img = cells[c][0].get_texture().get_image()
				img.convert(Image.FORMAT_RGB8)
				lit = 0
				red = 0
				seen = 0
				for y in range(0, img.get_height(), 2):
					for x in range(0, img.get_width(), 2):
						seen += 1
						var px := img.get_pixel(x, y)
						if absf(px.r - BG.r) > 0.004 or absf(px.g - BG.g) > 0.004 or absf(px.b - BG.b) > 0.004:
							lit += 1
						if px.r > 0.8 and px.g < 0.45 and px.b < 0.45:
							red += 1
				if lit > 0:
					break
			row.img.append(img)
			row.lit.append(100.0 * lit / maxi(1, seen))
			if c == 2:
				row.red = red
		rows.append(row)
	rows.sort_custom(func(x, y): return x.red > y.red)
	var w := LABEL_W + THUMB.x * cols.size()
	var h := HEAD_H + COL_H + THUMB.y * rows.size()
	var sheet := SubViewport.new()
	sheet.size = Vector2i(w, h)
	sheet.render_target_update_mode = SubViewport.UPDATE_ALWAYS
	root.add_child(sheet)
	_rect(sheet, Vector2.ZERO, Vector2(w, h), Color(0.12, 0.12, 0.14))
	var failed := 0
	for r in rows:
		for c in cols.size():
			if r.lit[c] <= 0.0:
				failed += 1
		if r.red <= 0:
			failed += 1
	_label(sheet, "pants proxy at 2 mm: what is wrong with it   %d of %d cells drawn, %d of %d views show a red defect" % [rows.size() * 3 - failed, rows.size() * 3, rows.filter(func(r): return r.red > 0).size(), rows.size()], Vector2(16, 10), 26, 0)
	_label(sheet, "The solver needs ONE connected shell with no self-crossings. The garment has 53 shells, the proxy 71 (main shell 99.6% of triangles, 70 small islands in colour).", Vector2(16, 48), 16, 0)
	_label(sheet, "Red: 5,822 zero-area triangles and 2,469 vertices sharing a position, both from the extractor. A further 20 crossing pairs were found in 4 sampled boxes and are not drawn.", Vector2(16, 72), 16, 0)
	_label(sheet, "garment: 4014 welded vertices, 7543 triangles.   proxy: 2,414,324 welded vertices, 4,828,856 triangles (too heavy to fit).", Vector2(16, 96), 16, 0)
	_label(sheet, "layout RFD66606-2370 (row per view, column per source, failures first: here the views with the most red). sidecars RFD66606-2240. skill avatar-garment-fitting.", Vector2(16, 124), 14, 0)
	_label(sheet, "cameras: sphere_hammersley_sequence, 8 views, fov 30, orbit about the centre of both meshes (pattern of transport-meshing-pen tools/orbit_views.sgd)", Vector2(16, 144), 14, 0)
	_label(sheet, "cell label: share of pixels not background; red-pixel count in the right column; a cell at 0 FAILs", Vector2(16, 164), 14, 0)
	for c in cols.size():
		_label(sheet, cols[c].title, Vector2(LABEL_W + c * THUMB.x + 8, HEAD_H + 6), 15, THUMB.x - 16)
	for k in rows.size():
		var r: Dictionary = rows[k]
		var y := HEAD_H + COL_H + k * THUMB.y
		_label(sheet, "view %d\nazimuth %.1f deg\nelevation %.1f deg\nred pixels %d" % [r.view, r.az, r.el, r.red * 4], Vector2(10, y + 8), 17, LABEL_W - 20)
		for c in cols.size():
			var tr := TextureRect.new()
			tr.texture = ImageTexture.create_from_image(r.img[c])
			tr.position = Vector2(LABEL_W + c * THUMB.x, y)
			tr.size = Vector2(THUMB)
			sheet.add_child(tr)
			var tag: String = "%.1f%% of frame" % r.lit[c]
			if r.lit[c] <= 0.0:
				tag = "FAIL: blank view"
			elif c == 2 and r.red <= 0:
				tag = "FAIL: no defect visible"
			_label(sheet, tag, Vector2(LABEL_W + c * THUMB.x + 8, y + THUMB.y - 26), 16, 0)
	for f in 4:
		await process_frame
	var simg: Image = sheet.get_texture().get_image()
	var err := simg.save_png(out)
	var f := FileAccess.open(out.get_basename() + ".tsv", FileAccess.WRITE)
	f.store_line("view\tazimuth_deg\televation_deg\tgarment_percent\tproxy_percent\tdefect_percent\tred_pixels_in_defect_cell\tstatus")
	for r in rows:
		f.store_line("%d\t%.2f\t%.2f\t%.3f\t%.3f\t%.3f\t%d\t%s" % [r.view, r.az, r.el, r.lit[0], r.lit[1], r.lit[2], r.red * 4, "ok" if (r.lit[0] > 0 and r.lit[1] > 0 and r.lit[2] > 0 and r.red > 0) else "FAIL"])
	f.close()
	print("saved ", out, " err=", err, " size=", simg.get_size(), " failed cells ", failed)
	quit(1 if failed > 0 else 0)


func _load(path: String) -> Array:
	var b := FileAccess.get_file_as_bytes(path)
	var nv := b.decode_s32(0)
	var nt := b.decode_s32(4)
	var fl := b.slice(8, 8 + nv * 36).to_float32_array()
	var pos := PackedVector3Array()
	var nor := PackedVector3Array()
	var col := PackedColorArray()
	pos.resize(nv)
	nor.resize(nv)
	col.resize(nv)
	for i in nv:
		pos[i] = Vector3(fl[i * 9], fl[i * 9 + 1], fl[i * 9 + 2])
		nor[i] = Vector3(fl[i * 9 + 3], fl[i * 9 + 4], fl[i * 9 + 5])
		col[i] = Color(fl[i * 9 + 6], fl[i * 9 + 7], fl[i * 9 + 8])
	var idx := b.slice(8 + nv * 36, 8 + nv * 36 + nt * 12).to_int32_array()
	var arr := []
	arr.resize(Mesh.ARRAY_MAX)
	arr[Mesh.ARRAY_VERTEX] = pos
	arr[Mesh.ARRAY_NORMAL] = nor
	arr[Mesh.ARRAY_COLOR] = col
	arr[Mesh.ARRAY_INDEX] = idx
	var m := ArrayMesh.new()
	m.add_surface_from_arrays(Mesh.PRIMITIVE_TRIANGLES, arr)
	return [m, nv, nt]


func _rect(p: Node, at: Vector2, size: Vector2, c: Color) -> void:
	var r := ColorRect.new()
	r.color = c
	r.position = at
	r.size = size
	p.add_child(r)


func _label(p: Node, text: String, at: Vector2, px: int, width: int) -> void:
	var l := Label.new()
	l.text = text
	l.position = at
	l.add_theme_font_size_override("font_size", px)
	l.add_theme_color_override("font_color", Color(0.93, 0.93, 0.95))
	if width > 0:
		l.custom_minimum_size = Vector2(width, 0)
		l.autowrap_mode = TextServer.AUTOWRAP_WORD_SMART
	p.add_child(l)


static func _hammersley(n: int) -> Array:
	var out := []
	for i in n:
		var u := float(i) / n
		var v := 0.0
		var f := 0.5
		var k := i
		while k > 0:
			v += (k & 1) * f
			k >>= 1
			f *= 0.5
		# Pixal3D sphere_hammersley_sequence(n, N), offset (0, 0), remap False, as render_multiview calls it: no remap, no elevation clamp. View 0 is straight below.
		var el := rad_to_deg(acos(1.0 - 2.0 * u) - PI / 2.0)
		out.append([v * 360.0, el])
	return out
