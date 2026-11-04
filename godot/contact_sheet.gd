extends SceneTree
# Orbit-view contact sheet of two meshes: one row per camera, one column per source, failures first.
# godot --path . -s res://contact_sheet.gd -- <a.bin> <b.bin> <out.png> [--control=blank]

const THUMB := Vector2i(560, 315)
const LABEL_W := 300
const HEAD_H := 150
const COL_H := 34
const BG := Color(0.20, 0.20, 0.24)
const VIEWS := 8
const FOV := 40.0

var _sources: Array = []


func _init() -> void:
	var a := OS.get_cmdline_user_args()
	var out: String = a[2]
	var control: bool = a.size() > 3 and a[3] == "--control=blank"
	var names: Array = [a[0].get_file().get_basename(), a[1].get_file().get_basename()]
	var titles: Array = ["pants garment, as exported", "pants proxy, 2 mm level surface"]
	var meshes: Array = []
	var stats: Array = []
	var box := AABB()
	for i in 2:
		var m := _load(a[i])
		meshes.append(m[0])
		stats.append(m[1])
		box = box.merge(m[0].get_aabb()) if i > 0 else m[0].get_aabb()
	var centre: Vector3 = box.get_center()
	var radius: float = box.size.length() * 0.5
	var dist: float = radius / sin(deg_to_rad(FOV * 0.5)) * 0.9
	var views := _hammersley(VIEWS)
	var cells: Array = []
	for i in 2:
		var vp := SubViewport.new()
		vp.size = THUMB
		vp.own_world_3d = true
		vp.render_target_update_mode = SubViewport.UPDATE_DISABLED
		var env := Environment.new()
		env.background_mode = Environment.BG_COLOR
		env.background_color = BG
		env.ambient_light_source = Environment.AMBIENT_SOURCE_COLOR
		env.ambient_light_color = Color(0.45, 0.45, 0.5)
		var we := WorldEnvironment.new()
		we.environment = env
		vp.add_child(we)
		var mi := MeshInstance3D.new()
		mi.mesh = meshes[i]
		var mat := StandardMaterial3D.new()
		mat.albedo_color = Color(0.82, 0.76, 0.66)
		mat.cull_mode = BaseMaterial3D.CULL_DISABLED
		mat.roughness = 0.8
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
		var dir := Vector3(sin(deg_to_rad(az)) * cos(deg_to_rad(el)), sin(deg_to_rad(el)), -cos(deg_to_rad(az)) * cos(deg_to_rad(el)))   # Pixal3D is Z-up: (x, y, z) -> (x, z, -y) for this Y-up scene
		var row := {"view": v, "az": az, "el": el, "img": [], "lit": []}
		for i in 2:
			var cam: Camera3D = cells[i][1]
			var eye: Vector3 = centre + dir * dist
			cam.look_at_from_position(eye, centre if not (control and i == 1) else eye + dir, Vector3.UP if absf(dir.y) < 0.999 else Vector3(0, 0, 1))
			var img: Image
			var lit := 0
			var seen := 0
			for attempt in 3:
				cells[i][0].render_target_update_mode = SubViewport.UPDATE_ONCE
				for f in 12:
					await process_frame
				await RenderingServer.frame_post_draw
				img = cells[i][0].get_texture().get_image()
				img.convert(Image.FORMAT_RGB8)
				lit = 0
				seen = 0
				for y in range(0, img.get_height(), 2):
					for x in range(0, img.get_width(), 2):
						seen += 1
						var c := img.get_pixel(x, y)
						if absf(c.r - BG.r) > 0.004 or absf(c.g - BG.g) > 0.004 or absf(c.b - BG.b) > 0.004:
							lit += 1
				if lit > 0:
					break
			row.img.append(img)
			row.lit.append(100.0 * lit / seen)
		rows.append(row)
	for r in rows:
		r["worst"] = minf(r.lit[0], r.lit[1])
	rows.sort_custom(func(x, y): return x.worst < y.worst)
	var w := LABEL_W + THUMB.x * 2
	var h := HEAD_H + COL_H + THUMB.y * rows.size()
	var sheet := SubViewport.new()
	sheet.size = Vector2i(w, h)
	sheet.render_target_update_mode = SubViewport.UPDATE_ALWAYS
	root.add_child(sheet)
	_rect(sheet, Vector2.ZERO, Vector2(w, h), Color(0.12, 0.12, 0.14))
	var failed := 0
	for r in rows:
		for i in 2:
			if r.lit[i] <= 0.0:
				failed += 1
	_label(sheet, "pants proxy: garment against its 2 mm distance surface   %d of %d views drawn" % [rows.size() * 2 - failed, rows.size() * 2], Vector2(16, 10), 28, 0)
	_label(sheet, "layout: RFD66606-2370 (one row per case, one column per source, failures first); sidecars: RFD66606-2240; skill: avatar-garment-fitting", Vector2(16, 50), 15, 0)
	_label(sheet, "cameras: sphere_hammersley_sequence, %d views, fov %d, orbit about the centre of both meshes' bounds (pattern of transport-meshing-pen tools/orbit_views.sgd)" % [VIEWS, int(FOV)], Vector2(16, 70), 15, 0)
	_label(sheet, "garment: %s" % stats[0], Vector2(16, 94), 17, 0)
	_label(sheet, "proxy: %s" % stats[1], Vector2(16, 114), 17, 0)
	_label(sheet, "cell label: share of pixels that are not background; a cell at 0 is a FAIL" , Vector2(16, 134), 14, 0)
	for i in 2:
		_label(sheet, titles[i], Vector2(LABEL_W + i * THUMB.x + 8, HEAD_H + 6), 17, 0)
	for k in rows.size():
		var r: Dictionary = rows[k]
		var y := HEAD_H + COL_H + k * THUMB.y
		_label(sheet, "view %d\nazimuth %.1f deg\nelevation %.1f deg" % [r.view, r.az, r.el], Vector2(10, y + 8), 17, LABEL_W - 20)
		for i in 2:
			var tr := TextureRect.new()
			tr.texture = ImageTexture.create_from_image(r.img[i])
			tr.position = Vector2(LABEL_W + i * THUMB.x, y)
			tr.size = Vector2(THUMB)
			sheet.add_child(tr)
			var tag: String = "%.1f%% of frame" % r.lit[i]
			if r.lit[i] <= 0.0:
				tag = "FAIL: blank view"
			_label(sheet, tag, Vector2(LABEL_W + i * THUMB.x + 8, y + THUMB.y - 28), 17, 0)
	for f in 4:
		await process_frame
	var simg: Image = sheet.get_texture().get_image()
	var err := simg.save_png(out)
	var f := FileAccess.open(out.get_basename() + ".tsv", FileAccess.WRITE)
	f.store_line("view\tazimuth_deg\televation_deg\tsource\tpercent_of_frame\tstatus")
	for r in rows:
		for i in 2:
			f.store_line("%d\t%.2f\t%.2f\t%s\t%.3f\t%s" % [r.view, r.az, r.el, names[i], r.lit[i], "ok" if r.lit[i] > 0.0 else "FAIL"])
	f.close()
	print("saved ", out, " err=", err, " size=", simg.get_size(), " views drawn ", rows.size() * 2 - failed, "/", rows.size() * 2)
	quit(1 if failed > 0 else 0)


func _load(path: String) -> Array:
	var b := FileAccess.get_file_as_bytes(path)
	var nv := b.decode_s32(0)
	var nt := b.decode_s32(4)
	var fl := b.slice(8, 8 + nv * 24).to_float32_array()
	var pos := PackedVector3Array()
	var nor := PackedVector3Array()
	pos.resize(nv)
	nor.resize(nv)
	for i in nv:
		pos[i] = Vector3(fl[i * 6], fl[i * 6 + 1], fl[i * 6 + 2])
		nor[i] = Vector3(fl[i * 6 + 3], fl[i * 6 + 4], fl[i * 6 + 5])
	var idx := b.slice(8 + nv * 24, 8 + nv * 24 + nt * 12).to_int32_array()
	var arr := []
	arr.resize(Mesh.ARRAY_MAX)
	arr[Mesh.ARRAY_VERTEX] = pos
	arr[Mesh.ARRAY_NORMAL] = nor
	arr[Mesh.ARRAY_INDEX] = idx
	var m := ArrayMesh.new()
	m.add_surface_from_arrays(Mesh.PRIMITIVE_TRIANGLES, arr)
	return [m, "%d welded vertices, %d triangles" % [nv, nt]]


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
