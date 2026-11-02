extends SceneTree
# Contact-sheet cells for per-marker decals (prototype/decals.py): the cloth-fit input (column 0) and the fitted result
# (column 1), each with a plain base mesh and a decal mesh per body/skirt, textured from its ArUco board. 16 Hammersley
# cameras; the window shows each render. Writes v<N>_c<0|1>.png plus _u/_v UV-pass images (decal UV as 16 bits) for
# prototype/verify_ids.py and recall_ids.py. The base mesh reads UV 0,0 in the UV passes, which is empty board space.
# godot --path <proj> -s res://fit_decals.gd -- <dir with *_base.obj / *_decals.obj> <body in> <skirt in> <body fit> <skirt fit> [cams0.json cams1.json]
#   (names are the file prefixes, e.g. bodyin skirt bodyfit skirt_fit); env BOARD_BODY, BOARD_SKIRT, CELLS
#   With two camera lists (prototype/cameras.py) each column renders its own list (a studio-style plan, narrow fov, head-on);
#   without them, 16 Hammersley orbit cameras around each column's bounds.
const THUMB := Vector2i(2000, 2200)
const BG := Color(0.20, 0.20, 0.24)
const VIEWS := 16
const FOV := 40.0
const UVSHADER := """shader_type spatial;
render_mode unshaded, cull_disabled;
uniform int mode = 0;
uniform float mesh_id = 0.0;   // 0 body, 1 skirt: lets the readers tell the meshes apart without colour
float s2l(float c) { return c <= 0.04045 ? c / 12.92 : pow((c + 0.055) / 1.055, 2.4); }
void fragment() {
	vec2 uv = fract(UV);   // the board repeats across an atlas larger than one sheet
	float v = mode == 0 ? uv.x : uv.y;
	float t = floor(clamp(v, 0.0, 1.0) * 65535.0 + 0.5);
	float hi = floor(t / 256.0);
	float lo = t - hi * 256.0;
	ALBEDO = vec3(s2l(hi / 255.0), s2l(lo / 255.0), mesh_id);
}
"""
func _init() -> void:
	var a := OS.get_cmdline_user_args()
	var dir: String = a[0]
	var names := [[a[1], a[2]], [a[3], a[4]]]   # per column: body prefix, skirt prefix
	var texs := []
	for pth in [OS.get_environment("BOARD_BODY"), OS.get_environment("BOARD_SKIRT")]:
		var im0 := Image.load_from_file(pth)
		var mx := maxi(im0.get_width(), im0.get_height())
		if mx > 16000:   # the GPU texture limit is 16384
			im0.resize(int(im0.get_width() * 16000.0 / mx), int(im0.get_height() * 16000.0 / mx), Image.INTERPOLATE_LANCZOS)
		im0.generate_mipmaps()
		texs.append(ImageTexture.create_from_image(im0))
	var tints := [Color(0.75, 0.85, 1.0), Color(1.0, 0.62, 0.25)]
	var shader := Shader.new()
	shader.code = UVSHADER
	var uvmats := []   # [mode][mesh]
	for md in 3:
		var row := []
		for k in 2:
			var m_ := ShaderMaterial.new()
			m_.shader = shader
			m_.set_shader_parameter("mode", md)
			m_.set_shader_parameter("mesh_id", float(k))
			row.append(m_)
		uvmats.append(row)
	var vps := []
	var boxes := []
	var all_mis := []
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
		var box := AABB()
		var first := true
		var mis := []
		for k in 2:
			var uvp := "%s/%s_uv.obj" % [dir, names[i][k]]
			if FileAccess.file_exists(uvp):   # the real board on UVs: one textured mesh, black and white as printed (TINT=1 colours it)
				var mu := _load(uvp, true)
				var matu := StandardMaterial3D.new()
				matu.albedo_color = tints[k] if OS.get_environment("TINT") == "1" else Color(1, 1, 1)
				matu.cull_mode = BaseMaterial3D.CULL_DISABLED
				matu.roughness = 0.9
				matu.albedo_texture = texs[k]
				matu.texture_filter = BaseMaterial3D.TEXTURE_FILTER_LINEAR_WITH_MIPMAPS_ANISOTROPIC
				var miu := MeshInstance3D.new()
				miu.mesh = mu
				miu.material_override = matu
				vp.add_child(miu)
				mis.append([miu, matu, true, k])
				box = mu.get_aabb() if first else box.merge(mu.get_aabb())
				first = false
				continue
			if not FileAccess.file_exists("%s/%s_base.obj" % [dir, names[i][k]]):
				continue   # no mesh for this slot (e.g. a body-only run)
			for decal in [false, true]:
				var path := "%s/%s_%s.obj" % [dir, names[i][k], "decals" if decal else "base"]
				var m := _load(path, decal)
				var mat := StandardMaterial3D.new()
				mat.albedo_color = tints[k]
				mat.cull_mode = BaseMaterial3D.CULL_DISABLED
				mat.roughness = 0.9
				if decal:
					mat.albedo_texture = texs[k]
					mat.texture_filter = BaseMaterial3D.TEXTURE_FILTER_LINEAR_WITH_MIPMAPS_ANISOTROPIC
				else:
					mat.albedo_color = tints[k] * 0.55
				var mi := MeshInstance3D.new()
				mi.mesh = m
				mi.material_override = mat
				vp.add_child(mi)
				mis.append([mi, mat, decal, k])
				if not decal:
					box = m.get_aabb() if first else box.merge(m.get_aabb())
					first = false
		var cam := Camera3D.new()
		cam.fov = FOV
		cam.near = 0.05
		cam.far = 200.0
		vp.add_child(cam)
		var light := DirectionalLight3D.new()
		cam.add_child(light)
		root.add_child(vp)
		vps.append([vp, cam])
		boxes.append(box)
		all_mis.append(mis)
	var bgr := ColorRect.new()
	bgr.color = Color(0.07, 0.07, 0.08)
	bgr.size = Vector2(1152, 648)
	root.add_child(bgr)
	root.size = Vector2i(1152, 648)
	root.always_on_top = true   # a covered window stops drawing, and a render that waits on it stalls
	var status := Label.new()
	status.position = Vector2(14, 10)
	status.add_theme_font_size_override("font_size", 22)
	root.add_child(status)
	var trs := []   # the preview shows the last colour frame, not the live viewport, which spends two thirds of its time on the UV passes
	for i in 2:
		var tr := TextureRect.new()
		trs.append(tr)
		tr.expand_mode = TextureRect.EXPAND_IGNORE_SIZE
		tr.texture = vps[i][0].get_texture()
		tr.position = Vector2(10 + i * 570, 50)
		tr.size = Vector2(560, 590)
		tr.stretch_mode = TextureRect.STRETCH_KEEP_ASPECT
		root.add_child(tr)
	var titles := ["cloth-fit input", "fitted"]
	var lists := [[], []]
	if a.size() > 6:
		for i in 2:
			for cm in JSON.parse_string(FileAccess.get_file_as_string(a[5 + i])):
				lists[i].append(cm)
	else:
		var views := _hammersley(VIEWS)
		for i in 2:
			for v in VIEWS:
				var az: float = views[v][0]
				var el: float = views[v][1]
				var dv := Vector3(sin(deg_to_rad(az)) * cos(deg_to_rad(el)), sin(deg_to_rad(el)), cos(deg_to_rad(az)) * cos(deg_to_rad(el)))
				var centre: Vector3 = boxes[i].get_center()
				var dist: float = boxes[i].size.length() * 0.5 / sin(deg_to_rad(FOV * 0.5)) * 0.9
				var upv := [0.0, 1.0, 0.0] if absf(dv.y) < 0.99 else [1.0, 0.0, 0.0]
				lists[i].append({"pos": [centre.x + dv.x * dist, centre.y + dv.y * dist, centre.z + dv.z * dist], "target": [centre.x, centre.y, centre.z], "up": upv, "fov": FOV})
	print("cameras per column: ", lists[0].size(), " ", lists[1].size())
	var nmax: int = maxi(lists[0].size(), lists[1].size())
	for v in nmax:
		for i in 2:
			if v >= lists[i].size():
				continue
			var base := "%s/v%d_c%d" % [OS.get_environment("CELLS"), v, i]
			if FileAccess.file_exists(base + ".png") and FileAccess.file_exists(base + "_u.png") and FileAccess.file_exists(base + "_v.png"):
				continue   # resume: this camera is already rendered
			var cm: Dictionary = lists[i][v]
			var cam: Camera3D = vps[i][1]
			cam.fov = float(cm["fov"])
			var up := Vector3(cm["up"][0], cm["up"][1], cm["up"][2])
			cam.look_at_from_position(Vector3(cm["pos"][0], cm["pos"][1], cm["pos"][2]), Vector3(cm["target"][0], cm["target"][1], cm["target"][2]), up)
			status.text = "camera %d of %d, %s: decals" % [v + 1, lists[i].size(), titles[i]]
			for f in 10:
				await process_frame   # linger: let the camera move and the materials settle before anything is drawn
			var img: Image
			var passes := []
			for attempt in 3:
				for pr in all_mis[i]:
					pr[0].material_override = pr[1]
				img = await _shot(vps[i][0], 8)
				trs[i].texture = ImageTexture.create_from_image(img)
				passes = []
				for md in 2:
					for pr in all_mis[i]:
						pr[0].material_override = uvmats[md][pr[3]] if pr[2] else uvmats[2][pr[3]]
					status.text = "camera %d of %d, %s: UV pass %s" % [v + 1, lists[i].size(), titles[i], "u" if md == 0 else "v"]
					passes.append(await _shot(vps[i][0], 6))
				for pr in all_mis[i]:
					pr[0].material_override = pr[1]
				if not _same(img, passes[0]) and not _same(img, passes[1]):   # a colour frame equal to a UV pass was captured before the materials changed back
					break
				print("retry camera ", v, ": colour frame matches a UV pass")
				for f in 20:
					await process_frame
			img.save_png("%s/v%d_c%d.png" % [OS.get_environment("CELLS"), v, i])
			passes[0].save_png("%s/v%d_c%d_u.png" % [OS.get_environment("CELLS"), v, i])
			passes[1].save_png("%s/v%d_c%d_v.png" % [OS.get_environment("CELLS"), v, i])
			for f in 6:
				await process_frame
	print("saved cells")
	quit(0)

# Draws the viewport twice and keeps the second image: the first draw can still show the state before the last change.
func _shot(vp: SubViewport, frames: int) -> Image:
	for rep in 2:
		vp.render_target_update_mode = SubViewport.UPDATE_ONCE
		for f in frames:
			await process_frame
		await _drawn()
	var im: Image = vp.get_texture().get_image()
	im.convert(Image.FORMAT_RGB8)
	return im

func _same(a: Image, b: Image) -> bool:
	if a.get_size() != b.get_size():
		return false
	var x: Image = a.duplicate()
	var y: Image = b.duplicate()
	x.resize(64, 64)
	y.resize(64, 64)
	return x.get_data() == y.get_data()

# Waits for the frame to be drawn, but not for ever: a window that is covered or hidden never draws.
func _drawn() -> void:
	var done := [false]
	RenderingServer.frame_post_draw.connect(func(): done[0] = true, CONNECT_ONE_SHOT)
	var t0 := Time.get_ticks_msec()
	while not done[0] and Time.get_ticks_msec() - t0 < 4000:
		await process_frame

func _load(path: String, with_uv: bool) -> ArrayMesh:
	var pos := PackedVector3Array()
	var uv := PackedVector2Array()
	var idx := PackedInt32Array()
	for l in FileAccess.get_file_as_string(path).split("\n"):
		if l.begins_with("v "):
			var t := l.split(" ", false)
			pos.append(Vector3(float(t[1]), float(t[2]), float(t[3])))
		elif l.begins_with("vt "):
			var t := l.split(" ", false)
			uv.append(Vector2(float(t[1]), float(t[2])))
		elif l.begins_with("f "):
			var t := l.split(" ", false)
			for k in 3:
				idx.append(int(t[k + 1].split("/")[0]) - 1)
	var arr := []
	arr.resize(Mesh.ARRAY_MAX)
	arr[Mesh.ARRAY_VERTEX] = pos
	arr[Mesh.ARRAY_INDEX] = idx
	if with_uv:
		arr[Mesh.ARRAY_TEX_UV] = uv
	var m0 := ArrayMesh.new()
	m0.add_surface_from_arrays(Mesh.PRIMITIVE_TRIANGLES, arr)
	var sm := SurfaceTool.new()
	sm.create_from(m0, 0)
	sm.generate_normals()
	return sm.commit()

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
