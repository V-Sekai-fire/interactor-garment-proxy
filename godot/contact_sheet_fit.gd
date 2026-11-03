extends SceneTree
# Contact sheet of fit results: one column per variant, one row per camera angle, every cell the garment on the body.
# Meshes are OpenUSD (.usdz) read through usd.elf (usd_mesh_points / usd_mesh_indices).
# Needs a window (not --headless): godot --path <proj> --resolution WxH -s res://contact_sheet_fit.gd -- args
#
# usage: ... -- <usd.elf> <out.png> <body.usdz> <focus_index> "<label 1>|<label 2>|..." <garment1.usdz> <garment2.usdz> ...
# focus_index: which garment column frames the cameras (0-based).

const CELL := Vector2i(360, 360)
const HEADER := 64
# The orbit is contact_sheet.gd's: sphere_hammersley_sequence views, FOV 40, distance radius / sin(FOV / 2) * 0.9
# about the centre of the bounds. Its static _hammersley is called, not copied.
const Orbit := preload("res://contact_sheet.gd")
const N_VIEWS := 8
const FOV := 40.0


func _init() -> void:
	var a := OS.get_cmdline_user_args()
	var elf := FileAccess.get_file_as_bytes(a[0])
	var out_png: String = a[1]
	var body := _load_usdz(elf, a[2])
	var focus := int(a[3])
	var labels := String(a[4]).split("|")
	var garments: Array = []
	for i in range(5, a.size()):
		garments.append(_load_usdz(elf, a[i]))
	var cols := garments.size()
	var box := (garments[focus] as ArrayMesh).get_aabb()
	var center := box.get_center()
	var dist: float = (box.size.length() * 0.5) / sin(deg_to_rad(FOV * 0.5)) * 0.9
	var views: Array = Orbit._hammersley(N_VIEWS)

	var win := Vector2i(cols * CELL.x, HEADER + views.size() * CELL.y)
	var stage := SubViewport.new()
	stage.size = win
	stage.transparent_bg = false
	stage.render_target_update_mode = SubViewport.UPDATE_ALWAYS
	root.add_child(stage)

	var bg := ColorRect.new()
	bg.color = Color(0.09, 0.09, 0.11)
	bg.size = Vector2(win)
	stage.add_child(bg)

	for c in cols:
		var lab := Label.new()
		lab.text = labels[c] if c < labels.size() else "col %d" % c
		lab.position = Vector2(c * CELL.x + 10, 8)
		lab.size = Vector2(CELL.x - 20, HEADER - 12)
		lab.autowrap_mode = TextServer.AUTOWRAP_WORD_SMART
		lab.add_theme_font_size_override("font_size", 20)
		lab.add_theme_color_override("font_color", Color(0.95, 0.95, 0.95))
		stage.add_child(lab)
		for r in views.size():
			var cont := SubViewportContainer.new()
			cont.position = Vector2(c * CELL.x, HEADER + r * CELL.y)
			cont.size = Vector2(CELL)
			cont.stretch = true
			var vp := SubViewport.new()
			vp.size = CELL
			vp.own_world_3d = true
			vp.msaa_3d = Viewport.MSAA_4X
			vp.render_target_update_mode = SubViewport.UPDATE_ALWAYS
			cont.add_child(vp)
			stage.add_child(cont)

			var env := Environment.new()
			env.background_mode = Environment.BG_COLOR
			env.background_color = Color(0.13, 0.13, 0.16)
			env.ambient_light_source = Environment.AMBIENT_SOURCE_COLOR
			env.ambient_light_color = Color(0.62, 0.62, 0.66)
			env.ambient_light_energy = 0.9
			var cam := Camera3D.new()
			cam.environment = env
			cam.fov = FOV
			cam.near = 0.05
			cam.far = 20.0
			vp.add_child(cam)
			var az: float = views[r][0]
			var el: float = views[r][1]
			var dir := Vector3(sin(deg_to_rad(az)) * cos(deg_to_rad(el)), sin(deg_to_rad(el)), cos(deg_to_rad(az)) * cos(deg_to_rad(el)))
			cam.look_at_from_position(center + dir * dist, center, Vector3.UP)
			if c == 0:
				var vl := Label.new()
				vl.text = "view %d   az %.0f   el %.0f" % [r, az, el]
				vl.position = cont.position + Vector2(6, 4)
				vl.add_theme_font_size_override("font_size", 14)
				vl.add_theme_color_override("font_color", Color(0.85, 0.85, 0.9))
				stage.add_child(vl)
			var sun := DirectionalLight3D.new()
			sun.light_energy = 1.1
			vp.add_child(sun)
			sun.global_position = cam.position
			sun.look_at(center, Vector3.UP)

			var bm := MeshInstance3D.new()
			bm.mesh = body
			var bmat := StandardMaterial3D.new()
			bmat.albedo_color = Color(0.70, 0.66, 0.62)
			bmat.roughness = 0.9
			bmat.cull_mode = BaseMaterial3D.CULL_DISABLED
			bm.material_override = bmat
			vp.add_child(bm)
			var gm := MeshInstance3D.new()
			gm.mesh = garments[c]
			var gmat := StandardMaterial3D.new()
			gmat.albedo_color = Color(0.94, 0.42, 0.18)
			gmat.roughness = 0.7
			gmat.cull_mode = BaseMaterial3D.CULL_DISABLED
			gm.material_override = gmat
			vp.add_child(gm)

	for i in 8:
		await process_frame
	var img := stage.get_texture().get_image()
	var err := img.save_png(out_png)
	print("saved ", out_png, " ", img.get_size(), " err ", err)
	quit(0)


func _load_usdz(elf: PackedByteArray, path: String) -> ArrayMesh:
	var sb = ClassDB.instantiate("Sandbox")
	sb.memory_max = 1024
	sb.references_max = 4096
	sb.allocations_max = 1000000
	sb.execution_timeout = 600000
	root.add_child(sb)
	sb.load_buffer(elf)
	sb.vmcall("usd_init")
	var pkg := FileAccess.get_file_as_bytes(path)
	var at := 0
	while at < pkg.size():
		sb.vmcall("usd_push", pkg.slice(at, min(at + 524288, pkg.size())))
		at += 524288
	sb.vmcall("usd_open_staged")
	var pts = sb.vmcall("usd_mesh_points", 0)
	var idx = sb.vmcall("usd_mesh_indices", 0)
	sb.queue_free()
	var v := PackedVector3Array()
	for i in pts.size() / 3:
		v.append(Vector3(pts[i * 3], pts[i * 3 + 1], pts[i * 3 + 2]))
	var st := SurfaceTool.new()
	st.begin(Mesh.PRIMITIVE_TRIANGLES)
	for k in idx.size():
		st.add_vertex(v[idx[k]])
	st.generate_normals()
	print("loaded ", path.get_file(), ": ", v.size(), " points, ", idx.size() / 3, " triangles")
	return st.commit()
