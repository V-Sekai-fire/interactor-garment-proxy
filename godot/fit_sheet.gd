# Orbit contact sheet of a cloth-fit result: cloth-fit input (left) against the fitted step (right), body and
# garment textured with their own ArUco boards on real UVs (Godot's xatlas), 4 Hammersley cameras. The main window
# shows each render live. With CELLS=<dir> it also writes v<N>_c<0|1>.png plus _u/_v UV-pass images that
# prototype/verify_ids.py reads back against the board.
# godot --path <proj> -s res://fit_sheet.gd -- <avatar_in.obj> <garment_in.obj> <avatar_fit.obj> <garment_fit.obj> <out.png> x
#   env: BOARD_BODY, BOARD_SKIRT (prototype/make_boards.py output), CELLS, LIVEDIR, LIVE_HOLD (seconds per view)
# Constants for the Maple skirt: input OBJs are 0.2 m/unit, solver-frame OBJs 0.5187 m/unit (`scales`), solver
# frame is Z-up, body board 2250x2070 px and skirt board 1710x1575 px (`sheet_px`). The fitted garment must share
# the input garment's faces: it reuses the input's UVs, so the change in readable markers comes from the fit alone.
extends SceneTree
# Orbit contact sheet: body (grey) + skirt (orange) at start vs fitted. Rows = Hammersley cameras.
# args: <avatar_a> <garment_a> <avatar_b> <garment_b> <out.png> <board.png> [--control=blank]; scale_a=0.2 m/unit, scale_b=0.5187
const THUMB := Vector2i(2000, 2200)
const LABEL_W := 200
const HEAD_H := 90
const BG := Color(0.20, 0.20, 0.24)
const VIEWS := 4
const GRID := 48
const PX_PER_M := 1500.0
var texs := []
var ref_map := PackedInt32Array()
var ref_arrs := []
var want_ref := false
const UVSHADER := """shader_type spatial;
render_mode unshaded, cull_disabled;
uniform int mode = 0;
float s2l(float c) { return c <= 0.04045 ? c / 12.92 : pow((c + 0.055) / 1.055, 2.4); }
void fragment() {
	vec2 uv = fract(UV);
	float v = mode == 0 ? uv.x : uv.y;
	float t = floor(clamp(v, 0.0, 1.0) * 65535.0 + 0.5);
	float hi = floor(t / 256.0);
	float lo = t - hi * 256.0;
	ALBEDO = vec3(s2l(hi / 255.0), s2l(lo / 255.0), 0.0);
}
"""
var sheet_px := Vector2(2205, 2925)
var scales := [0.2, 0.5187]
const FOV := 40.0
func _init() -> void:
	var a := OS.get_cmdline_user_args()
	var out: String = a[4]
	var control: bool = a.size() > 6 and a[6] == "--control=blank"
	for pth in [OS.get_environment("BOARD_BODY"), OS.get_environment("BOARD_SKIRT")]:
		var im0 := Image.load_from_file(pth)
		im0.generate_mipmaps()
		texs.append(ImageTexture.create_from_image(im0))
	var cols := [[a[0], a[1]], [a[2], a[3]]]
	var titles := ["start (cloth-fit input)", "fitted (step 241)"]
	var boxes := []
	var meshes := []
	for c in cols:
		var pair := []
		var bx := AABB()
		for p in c:
			var is_g: bool = pair.size() == 1
			sheet_px = Vector2(1710, 1575) if is_g else Vector2(2250, 2070)
			var m: ArrayMesh
			if is_g and boxes.size() == 1:
				m = _load_same(p, scales[1])
			else:
				want_ref = is_g and boxes.size() == 0
				m = _load(p, scales[boxes.size()])
			bx = m.get_aabb() if pair.is_empty() else bx.merge(m.get_aabb())
			pair.append(m)
		boxes.append(bx)
		meshes.append(pair)
	var vps := []
	var shader := Shader.new()
	shader.code = UVSHADER
	var uvmats := []
	for md in 2:
		var sm_ := ShaderMaterial.new()
		sm_.shader = shader
		sm_.set_shader_parameter("mode", md)
		uvmats.append(sm_)
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
		var mis := []
		var cols_rgb := [Color(0.75, 0.85, 1.0), Color(1.0, 0.62, 0.25)]
		for k in 2:
			var mi := MeshInstance3D.new()
			mi.mesh = meshes[i][k]
			var mat := StandardMaterial3D.new()
			mat.albedo_color = cols_rgb[k]
			mat.cull_mode = BaseMaterial3D.CULL_DISABLED
			mat.roughness = 0.9
			mat.albedo_texture = texs[k]
			mat.texture_repeat = true
			mat.texture_filter = BaseMaterial3D.TEXTURE_FILTER_LINEAR_WITH_MIPMAPS_ANISOTROPIC
			mi.material_override = mat
			mis.append([mi, mat])
			vp.add_child(mi)
		var cam := Camera3D.new()
		cam.fov = FOV
		cam.near = 0.05
		cam.far = 200.0
		vp.add_child(cam)
		var light := DirectionalLight3D.new()
		cam.add_child(light)
		root.add_child(vp)
		vps.append([vp, cam, mis])
	var live_bg := ColorRect.new()
	live_bg.color = Color(0.07, 0.07, 0.08)
	live_bg.size = Vector2(1152, 648)
	root.add_child(live_bg)
	root.size = Vector2i(1152, 648)
	root.content_scale_size = Vector2i(1152, 648)
	root.content_scale_mode = Window.CONTENT_SCALE_MODE_CANVAS_ITEMS
	root.content_scale_aspect = Window.CONTENT_SCALE_ASPECT_KEEP
	var status := Label.new()
	status.position = Vector2(14, 10)
	status.add_theme_font_size_override("font_size", 22)
	root.add_child(status)
	var live_titles := ["cloth-fit input", "fitted (step 241)"]
	for i in 2:
		var tr := TextureRect.new()
		tr.expand_mode = TextureRect.EXPAND_IGNORE_SIZE
		tr.texture = vps[i][0].get_texture()
		tr.position = Vector2(10 + i * 570, 50)
		tr.size = Vector2(560, 590)
		tr.stretch_mode = TextureRect.STRETCH_KEEP_ASPECT_CENTERED if false else TextureRect.STRETCH_KEEP_ASPECT
		root.add_child(tr)
		var tl := Label.new()
		tl.text = live_titles[i]
		tl.position = Vector2(20 + i * 570, 52)
		tl.add_theme_font_size_override("font_size", 20)
		root.add_child(tl)
	var hold := float(OS.get_environment("LIVE_HOLD")) if OS.get_environment("LIVE_HOLD") != "" else 0.0
	var rows := []
	var views := _hammersley(VIEWS)
	for v in VIEWS:
		var az: float = views[v][0]
		var el: float = views[v][1]
		var dir := Vector3(sin(deg_to_rad(az)) * cos(deg_to_rad(el)), sin(deg_to_rad(el)), cos(deg_to_rad(az)) * cos(deg_to_rad(el)))
		var row := {"v": v, "az": az, "el": el, "img": [], "lit": []}
		for i in 2:
			var cam: Camera3D = vps[i][1]
			status.text = "view %d of %d, %s: textured render" % [v + 1, VIEWS, live_titles[i]]
			var centre: Vector3 = boxes[i].get_center()
			var dist: float = boxes[i].size.length() * 0.5 / sin(deg_to_rad(FOV * 0.5)) * 0.9
			var eye := centre + dir * dist
			cam.look_at_from_position(eye, centre if not (control and i == 1) else eye + dir, Vector3.UP if absf(dir.y) < 0.99 else Vector3.RIGHT)
			var img: Image
			var lit := 0
			var seen := 0
			for attempt in 3:
				vps[i][0].render_target_update_mode = SubViewport.UPDATE_ONCE
				for f in 12:
					await process_frame
				await RenderingServer.frame_post_draw
				img = vps[i][0].get_texture().get_image()
				img.convert(Image.FORMAT_RGB8)
				lit = 0
				seen = 0
				# constant time: GRID x GRID samples whatever the image size
				var iw := img.get_width()
				var ih := img.get_height()
				for gy in GRID:
					for gx in GRID:
						seen += 1
						var c := img.get_pixel((gx * iw) / GRID + iw / (2 * GRID), (gy * ih) / GRID + ih / (2 * GRID))
						if absf(c.r - BG.r) > 0.004 or absf(c.g - BG.g) > 0.004 or absf(c.b - BG.b) > 0.004:
							lit += 1
				if lit > 0:
					break
			if OS.get_environment("CELLS") != "":
				img.save_png("%s/v%d_c%d.png" % [OS.get_environment("CELLS"), v, i])
			if OS.get_environment("CELLS") != "":
				for md in 2:
					status.text = "view %d of %d, %s: UV pass %s (ground truth for the readback)" % [v + 1, VIEWS, live_titles[i], "u" if md == 0 else "v"]
					for pr in vps[i][2]:
						pr[0].material_override = uvmats[md]
					vps[i][0].render_target_update_mode = SubViewport.UPDATE_ONCE
					for f in 6:
						await process_frame
					await RenderingServer.frame_post_draw
					var ui: Image = vps[i][0].get_texture().get_image()
					ui.convert(Image.FORMAT_RGB8)
					ui.save_png("%s/v%d_c%d_%s.png" % [OS.get_environment("CELLS"), v, i, "u" if md == 0 else "v"])
				for pr in vps[i][2]:
					pr[0].material_override = pr[1]
				status.text = "view %d of %d, %s: textured model" % [v + 1, VIEWS, live_titles[i]]
				vps[i][0].render_target_update_mode = SubViewport.UPDATE_ONCE
				for f in 3:
					await process_frame
			row.img.append(img)
			row.lit.append(100.0 * lit / seen)
		rows.append(row)
		if hold > 0.0:
			status.text = "view %d of %d done" % [v + 1, VIEWS]
			await process_frame
			await RenderingServer.frame_post_draw
			root.get_texture().get_image().save_png("%s/live_view%d.png" % [OS.get_environment("LIVEDIR"), v])
			await create_timer(hold).timeout
	var w := LABEL_W + THUMB.x * 2
	var h := HEAD_H + THUMB.y * rows.size()
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
	_label(sheet, "Maple skirt cloth-fit, SOMA suit board on real UVs (xatlas): blue = body, orange = skirt   %d of %d views drawn" % [rows.size() * 2 - failed, rows.size() * 2], Vector2(16, 10), 24, 0)
	_label(sheet, "cameras: sphere_hammersley_sequence, %d views, fov %d; cell tag: share of pixels not background (0 = FAIL)" % [VIEWS, int(FOV)], Vector2(16, 44), 14, 0)
	for i in 2:
		_label(sheet, titles[i], Vector2(LABEL_W + i * THUMB.x + 8, HEAD_H - 28), 17, 0)
	for k in rows.size():
		var r: Dictionary = rows[k]
		var y := HEAD_H + k * THUMB.y
		_label(sheet, "view %d\naz %.1f\nel %.1f" % [r.v, r.az, r.el], Vector2(10, y + 8), 17, LABEL_W - 20)
		for i in 2:
			var tr := TextureRect.new()
			tr.texture = ImageTexture.create_from_image(r.img[i])
			tr.position = Vector2(LABEL_W + i * THUMB.x, y)
			tr.size = Vector2(THUMB)
			sheet.add_child(tr)
			_label(sheet, "FAIL: blank view" if r.lit[i] <= 0.0 else "%.1f%% of frame" % r.lit[i], Vector2(LABEL_W + i * THUMB.x + 8, y + THUMB.y - 28), 15, 0)
	for f in 4:
		await process_frame
	var err := sheet.get_texture().get_image().save_png(out)
	print("saved ", out, " err=", err, " views drawn ", rows.size() * 2 - failed, "/", rows.size() * 2)
	quit(1 if failed > 0 else 0)

func _load(path: String, sc: float) -> ArrayMesh:
	var pos := PackedVector3Array()
	var idx := PackedInt32Array()
	for line in FileAccess.get_file_as_string(path).split("\n"):
		if line.begins_with("v "):
			var t := line.split(" ", false)
			pos.append(Vector3(float(t[1]), float(t[2]), float(t[3])))
		elif line.begins_with("f "):
			var t := line.split(" ", false)
			var f := []
			for k in range(1, t.size()):
				f.append(int(t[k].split("/")[0]) - 1)
			for k in range(1, f.size() - 1):
				idx.append_array([f[0], f[k], f[k + 1]])
	# solver is Z-up; rotate to Y-up for the camera
	for i in pos.size():
		pos[i] = Vector3(pos[i].x, pos[i].z, -pos[i].y) * sc
	var arr := []
	arr.resize(Mesh.ARRAY_MAX)
	arr[Mesh.ARRAY_VERTEX] = pos
	arr[Mesh.ARRAY_INDEX] = idx
	var m := ArrayMesh.new()
	m.add_surface_from_arrays(Mesh.PRIMITIVE_TRIANGLES, arr)
	var nm := ArrayMesh.new()
	var st := SurfaceTool.new()
	st.create_from_triangle_arrays(m.surface_get_arrays(0)) if false else null
	var sm := SurfaceTool.new()
	sm.create_from(m, 0)
	sm.generate_normals()
	var orig := pos.duplicate()
	var mesh := sm.commit()
	var err := mesh.lightmap_unwrap(Transform3D.IDENTITY, 1.0 / PX_PER_M)
	if err != OK:
		push_error("lightmap_unwrap failed %d" % err)
	var arrs := mesh.surface_get_arrays(0)
	var uv2: PackedVector2Array = arrs[Mesh.ARRAY_TEX_UV2]
	var hint := Vector2(mesh.lightmap_size_hint)
	var uv := PackedVector2Array()
	uv.resize(uv2.size())
	for i in uv2.size():
		uv[i] = Vector2(uv2[i].x * hint.x / sheet_px.x, uv2[i].y * hint.y / sheet_px.y)
	arrs[Mesh.ARRAY_TEX_UV] = uv
	if want_ref:
		var key := {}
		for i in orig.size():
			key[_k(orig[i])] = i
		var vv: PackedVector3Array = arrs[Mesh.ARRAY_VERTEX]
		ref_map.resize(vv.size())
		var miss := 0
		for i in vv.size():
			var kk := _k(vv[i])
			var hit := -1
			for dx in [0, -1, 1]:
				for dy in [0, -1, 1]:
					for dz in [0, -1, 1]:
						if hit < 0 and key.has(kk + Vector3i(dx, dy, dz)):
							hit = key[kk + Vector3i(dx, dy, dz)]
			ref_map[i] = hit
			if hit < 0:
				miss += 1
		print("ref map built, unmatched ", miss, " of ", vv.size())
		ref_arrs = arrs.duplicate()
	var out := ArrayMesh.new()
	out.add_surface_from_arrays(Mesh.PRIMITIVE_TRIANGLES, arrs)
	print("unwrap ", path.get_file(), " verts ", uv.size(), " atlas ", hint)
	return out

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
		u = 2.0 * u if u < 0.25 else 2.0 / 3.0 * u + 1.0 / 3.0
		var el := rad_to_deg(acos(1.0 - 2.0 * u) - PI / 2.0)
		out.append([v * 360.0, clampf(el, -85.0, 85.0)])
	return out


func _load_same(path: String, sc: float) -> ArrayMesh:
	var pos := PackedVector3Array()
	for line in FileAccess.get_file_as_string(path).split("\n"):
		if line.begins_with("v "):
			var t := line.split(" ", false)
			pos.append(Vector3(float(t[1]), float(t[2]), float(t[3])))
	for i in pos.size():
		pos[i] = Vector3(pos[i].x, pos[i].z, -pos[i].y) * sc
	var arrs := ref_arrs.duplicate()
	var nv := PackedVector3Array()
	nv.resize(ref_map.size())
	for i in ref_map.size():
		nv[i] = pos[ref_map[i]]
	arrs[Mesh.ARRAY_VERTEX] = nv
	arrs[Mesh.ARRAY_NORMAL] = null
	arrs[Mesh.ARRAY_TEX_UV2] = null
	var m := ArrayMesh.new()
	m.add_surface_from_arrays(Mesh.PRIMITIVE_TRIANGLES, arrs)
	var sm := SurfaceTool.new()
	sm.create_from(m, 0)
	sm.generate_normals()
	print("same-UV skirt loaded ", path.get_file())
	return sm.commit()


func _k(v: Vector3) -> Vector3i:
	return Vector3i(roundi(v.x * 1e5), roundi(v.y * 1e5), roundi(v.z * 1e5))
