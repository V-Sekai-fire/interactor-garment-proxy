# Host for model_explode's explode.gd (copy it next to this script): loads each OBJ as one mesh, explodes, prints
# print_layout() and renders 4 views. Non-skinned meshes are never cut, so a body and a skirt give 2 pieces; the layout
# is box against box, so it shows pieces separate cleanly and does not measure clipping.
# godot --path <proj> -s res://explode_host.gd -- res://explode.gd <out.png> <zup:0|1> <a.obj> <b.obj> ...
extends SceneTree
# args: <explode.gd> <out.png> <zup:0|1> <obj>...   Loads each OBJ as one mesh, explodes, renders 4 views.
const THUMB := Vector2i(640, 640)
const BG := Color(0.20, 0.20, 0.24)
const PAL := [Color(0.62,0.66,0.72), Color(0.95,0.55,0.20), Color(0.35,0.7,0.95), Color(0.9,0.35,0.5), Color(0.5,0.85,0.4), Color(0.9,0.85,0.3), Color(0.7,0.5,0.9), Color(0.4,0.85,0.8), Color(0.85,0.6,0.45)]
func _init() -> void:
	var a := OS.get_cmdline_user_args()
	var ex = load(a[0]).new()
	var out: String = a[1]
	var zup: bool = a[2] == "1"
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
	var models := Node3D.new()
	vp.add_child(models)
	var legend := []
	for k in range(3, a.size()):
		var m := _load(a[k], zup)
		var mat := StandardMaterial3D.new()
		mat.albedo_color = PAL[(k - 3) % PAL.size()]
		mat.cull_mode = BaseMaterial3D.CULL_DISABLED
		mat.roughness = 0.8
		m.surface_set_material(0, mat)
		var mi := MeshInstance3D.new()
		mi.name = a[k].get_file().get_basename()
		mi.mesh = m
		models.add_child(mi)
		legend.append("%s = %s" % [mi.name, ["grey-blue","orange","blue","pink","green","yellow","violet","teal","tan"][(k-3) % 9]])
	var cam := Camera3D.new()
	cam.near = 0.05
	cam.far = 500.0
	cam.fov = 40.0
	vp.add_child(cam)
	var light := DirectionalLight3D.new()
	cam.add_child(light)
	root.add_child(vp)
	await process_frame
	ex.explode([models], vp, false)
	print("EXPLODED pieces=", ex.pieces.size(), " gap=", ex.gap)
	ex.print_layout()
	var bb := AABB()
	var first := true
	for i in ex.pieces.size():
		var b: AABB = ex.part_box[i]
		var c: Vector3 = ex.centres[i] + ex.dirs[i] * ex.moves[i]
		var nb := AABB(c - b.size * 0.5, b.size)
		bb = nb if first else bb.merge(nb)
		first = false
	var centre := bb.get_center()
	var dist := bb.size.length() * 0.5 / sin(deg_to_rad(20.0)) * 0.95
	var views := [[0.0, 10.0], [90.0, 10.0], [45.0, 35.0], [0.0, 85.0]]
	var imgs := []
	var failed := 0
	for v in views:
		var az: float = v[0]
		var el: float = v[1]
		var dir := Vector3(sin(deg_to_rad(az)) * cos(deg_to_rad(el)), sin(deg_to_rad(el)), cos(deg_to_rad(az)) * cos(deg_to_rad(el)))
		cam.look_at_from_position(centre + dir * dist, centre, Vector3.UP if el < 80 else Vector3.RIGHT)
		var img: Image
		var lit := 0
		for attempt in 3:
			vp.render_target_update_mode = SubViewport.UPDATE_ONCE
			for f in 12:
				await process_frame
			await RenderingServer.frame_post_draw
			img = vp.get_texture().get_image()
			img.convert(Image.FORMAT_RGB8)
			lit = 0
			for y in range(0, img.get_height(), 4):
				for x in range(0, img.get_width(), 4):
					var c := img.get_pixel(x, y)
					if absf(c.r - BG.r) > 0.004 or absf(c.g - BG.g) > 0.004 or absf(c.b - BG.b) > 0.004:
						lit += 1
			if lit > 0:
				break
		if lit == 0:
			failed += 1
		imgs.append(img)
	var sheet := SubViewport.new()
	sheet.size = Vector2i(THUMB.x * 2, THUMB.y * 2 + 60)
	sheet.render_target_update_mode = SubViewport.UPDATE_ALWAYS
	root.add_child(sheet)
	var bg := ColorRect.new()
	bg.color = Color(0.12, 0.12, 0.14)
	bg.size = Vector2(sheet.size)
	sheet.add_child(bg)
	var l := Label.new()
	l.text = "explode.gd: %d pieces, %d of 4 views drawn.  " % [ex.pieces.size(), 4 - failed] + ", ".join(legend)
	l.position = Vector2(10, 8)
	l.add_theme_font_size_override("font_size", 15)
	l.autowrap_mode = TextServer.AUTOWRAP_WORD_SMART
	l.custom_minimum_size = Vector2(THUMB.x * 2 - 20, 0)
	sheet.add_child(l)
	for i in 4:
		var tr := TextureRect.new()
		tr.texture = ImageTexture.create_from_image(imgs[i])
		tr.position = Vector2((i % 2) * THUMB.x, 60 + (i / 2) * THUMB.y)
		tr.size = Vector2(THUMB)
		sheet.add_child(tr)
	for f in 4:
		await process_frame
	print("saved ", out, " err=", sheet.get_texture().get_image().save_png(out), " failed=", failed)
	quit(1 if failed > 0 else 0)

func _load(path: String, zup: bool) -> ArrayMesh:
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
	if zup:
		for i in pos.size():
			pos[i] = Vector3(pos[i].x, pos[i].z, -pos[i].y)
	var arr := []
	arr.resize(Mesh.ARRAY_MAX)
	arr[Mesh.ARRAY_VERTEX] = pos
	arr[Mesh.ARRAY_INDEX] = idx
	var m := ArrayMesh.new()
	m.add_surface_from_arrays(Mesh.PRIMITIVE_TRIANGLES, arr)
	var sm := SurfaceTool.new()
	sm.create_from(m, 0)
	sm.generate_normals()
	return sm.commit()
