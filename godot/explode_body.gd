# Builds a skinned body from the Unity export (OBJ + 21-joint weights + rest skeleton), runs model_explode's explode.gd on it
# (a skinned body is cut at its skeleton's big branches: legs, arms, head), and writes each piece as an OBJ at its rest position,
# plus pieces.json (name, box, joint, axis, cut depth) for fitting a body model piece by piece.
# godot --headless --path <proj> -s res://explode_body.gd -- <explode.gd> <maple-fit dir> <out dir> [first vertex of the weighted mesh = 7065] [weighted vertex count = 9660]
extends SceneTree
const PARENT := {"Spine": "Hips", "Chest": "Spine", "Neck": "Chest", "Head": "Neck", "Shoulder.L": "Chest", "Upper_arm.L": "Shoulder.L", "Lower_arm.L": "Upper_arm.L", "Hand.L": "Lower_arm.L",
	"Shoulder.R": "Chest", "Upper_arm.R": "Shoulder.R", "Lower_arm.R": "Upper_arm.R", "Hand.R": "Lower_arm.R", "Upper_leg.L": "Hips", "Lower_leg.L": "Upper_leg.L", "Foot.L": "Lower_leg.L", "Toe.L": "Foot.L",
	"Upper_leg.R": "Hips", "Lower_leg.R": "Upper_leg.R", "Foot.R": "Lower_leg.R", "Toe.R": "Foot.R"}
func _init() -> void:
	var a := OS.get_cmdline_user_args()
	var ex = load(a[0]).new()
	var dir: String = a[1]
	var out: String = a[2]
	var v0 := int(a[3]) if a.size() > 3 else 7065
	var nv := int(a[4]) if a.size() > 4 else 9660
	DirAccess.make_dir_recursive_absolute(out)
	var names := FileAccess.get_file_as_string(dir + "/joints.txt").strip_edges().split("\n")
	var jpos := PackedVector3Array()
	for l in FileAccess.get_file_as_string(dir + "/target_skeleton.obj").split("\n"):
		var t := l.split(" ", false)
		if t.size() >= 4 and t[0] == "v":
			jpos.append(Vector3(float(t[1]), float(t[2]), float(t[3])))
	var wrows: Array = []
	for l in FileAccess.get_file_as_string(dir + "/avatar_weights.txt").strip_edges().split("\n"):
		var f := PackedFloat32Array()
		for s in l.split(" ", false):
			f.append(float(s))
		wrows.append(f)
	var verts := PackedVector3Array()
	var tris := PackedInt32Array()
	var all_v := PackedVector3Array()
	var all_f: Array = []
	for l in FileAccess.get_file_as_string(dir + "/avatar.obj").split("\n"):
		if l.begins_with("v "):
			var t := l.split(" ", false)
			all_v.append(Vector3(float(t[1]), float(t[2]), float(t[3])))
		elif l.begins_with("f "):
			var t := l.split(" ", false)
			all_f.append([int(t[1].split("/")[0]) - 1, int(t[2].split("/")[0]) - 1, int(t[3].split("/")[0]) - 1])
	for i in nv:
		verts.append(all_v[v0 + i])
	for f in all_f:
		if f[0] >= v0 and f[1] >= v0 and f[2] >= v0:
			tris.append_array(PackedInt32Array([f[0] - v0, f[1] - v0, f[2] - v0]))
	print("weighted body: ", verts.size(), " vertices, ", tris.size() / 3, " triangles, ", names.size(), " joints")
	var model := Node3D.new()
	var sk := Skeleton3D.new()
	sk.name = "Skeleton"
	model.add_child(sk)
	for n in names:
		sk.add_bone(n)
	for i in names.size():
		var p: String = PARENT.get(names[i], "")
		if p != "":
			sk.set_bone_parent(i, names.find(p))
			sk.set_bone_rest(i, Transform3D(Basis(), jpos[i] - jpos[names.find(p)]))
		else:
			sk.set_bone_rest(i, Transform3D(Basis(), jpos[i]))
	sk.reset_bone_poses()
	var skin := Skin.new()
	for i in names.size():
		skin.add_bind(i, sk.get_bone_global_rest(i).affine_inverse())
	var bones := PackedInt32Array()
	var wts := PackedFloat32Array()
	bones.resize(verts.size() * 4)
	wts.resize(verts.size() * 4)
	for vi in verts.size():
		var best: Array = []   # top four (weight, bone)
		for j in names.size():
			var w: float = wrows[j][vi]
			if w > 0.0:
				best.append([w, j])
		best.sort_custom(func(x, y): return x[0] > y[0])
		var sum := 0.0
		for k in mini(4, best.size()):
			sum += best[k][0]
		for k in 4:
			bones[vi * 4 + k] = best[k][1] if k < best.size() else 0
			wts[vi * 4 + k] = best[k][0] / sum if (k < best.size() and sum > 0.0) else 0.0
	var arr := []
	arr.resize(Mesh.ARRAY_MAX)
	arr[Mesh.ARRAY_VERTEX] = verts
	arr[Mesh.ARRAY_INDEX] = tris
	arr[Mesh.ARRAY_BONES] = bones
	arr[Mesh.ARRAY_WEIGHTS] = wts
	var am := ArrayMesh.new()
	am.add_surface_from_arrays(Mesh.PRIMITIVE_TRIANGLES, arr)
	var mi := MeshInstance3D.new()
	mi.name = "BaseBody"
	mi.mesh = am
	sk.add_child(mi)
	root.add_child(model)
	var holder := Node3D.new()
	root.add_child(holder)
	await process_frame
	mi.skeleton = mi.get_path_to(sk)   # set once the nodes are in the tree, so the skin registers with the skeleton
	mi.skin = skin
	for f in 3:
		await process_frame
	print("skin reference valid: ", mi.get_skin_reference() != null)
	ex.explode([model], holder, false)
	ex.print_layout()
	var sb = ClassDB.instantiate("Sandbox")   # interactor-usd-guest's usd.elf writes the OpenUSD packages
	sb.memory_max = 240
	sb.allocations_max = 1000000
	sb.references_max = 4096
	sb.execution_timeout = 400
	sb.program = load("res://usd.elf")
	root.add_child(sb)
	var info := []
	for i in ex.pieces.size():
		var piece: Node3D = ex.pieces[i]
		var pts := PackedFloat32Array()
		var cor := PackedInt32Array()
		var base := 0
		for m in piece.find_children("*", "MeshInstance3D", true, false):
			var msh: Mesh = (m as MeshInstance3D).mesh
			for sidx in msh.get_surface_count():
				var ar := msh.surface_get_arrays(sidx)
				var pv: PackedVector3Array = ar[Mesh.ARRAY_VERTEX]
				for p in pv:
					pts.append_array(PackedFloat32Array([p.x, p.y, p.z]))   # the piece node carries the layout move, so mesh space is the rest position
				var idx = ar[Mesh.ARRAY_INDEX]
				if idx != null and idx.size() > 0:
					for k in idx.size():
						cor.append(base + idx[k])
				else:   # a cut surface can have no index buffer: its vertices are already one triangle after another
					for k in (pv.size() / 3) * 3:
						cor.append(base + k)
				base += pv.size()
		var counts := PackedInt32Array()
		counts.resize(cor.size() / 3)
		counts.fill(3)
		var ints := counts.duplicate()
		ints.append_array(cor)
		var header := "piece_%02d
np=%d
nuv=0
nrest=0
nf=%d
ni=%d
source=explode.gd
piece=%d" % [i, pts.size() / 3, counts.size(), cor.size(), i]
		var z = sb.vmcall("usd_write_mesh", pts, ints, header, "")
		if z is PackedByteArray:
			var f := FileAccess.open("%s/piece_%02d.usdz" % [out, i], FileAccess.WRITE)
			f.store_buffer(z)
			f.close()
		else:
			print("usd_write_mesh failed for piece ", i, ": ", str(z).substr(0, 160))
		var box: AABB = ex.part_box[i]
		info.append({"piece": i, "name": String(piece.name), "box_min": [box.position.x, box.position.y, box.position.z], "box_size": [box.size.x, box.size.y, box.size.z],
			"joint": [] if ex.part_joint[i] == null else [ex.part_joint[i].x, ex.part_joint[i].y, ex.part_joint[i].z],
			"axis": [] if ex.part_axis[i] == null else [ex.part_axis[i].x, ex.part_axis[i].y, ex.part_axis[i].z], "depth": ex.part_depth[i]})
	var jf := FileAccess.open(out + "/pieces.json", FileAccess.WRITE)
	jf.store_string(JSON.stringify(info, "  "))
	jf.close()
	# Show it: colour each piece, light and frame the exploded layout, and save a front and a side render.
	var pal := [Color(0.90, 0.30, 0.30), Color(0.35, 0.80, 0.35), Color(0.30, 0.50, 0.95), Color(0.95, 0.80, 0.25), Color(0.80, 0.35, 0.80), Color(0.25, 0.85, 0.85), Color(0.75, 0.75, 0.75)]
	for i in ex.pieces.size():
		for m in (ex.pieces[i] as Node3D).find_children("*", "MeshInstance3D", true, false):
			var mat := StandardMaterial3D.new()
			mat.albedo_color = pal[i % pal.size()]
			mat.cull_mode = BaseMaterial3D.CULL_DISABLED
			mat.roughness = 0.7
			(m as MeshInstance3D).material_override = mat
	var env := Environment.new()
	env.background_mode = Environment.BG_COLOR
	env.background_color = Color(0.13, 0.13, 0.16)
	env.ambient_light_source = Environment.AMBIENT_SOURCE_COLOR
	env.ambient_light_color = Color(0.55, 0.55, 0.6)
	var we := WorldEnvironment.new()
	we.environment = env
	root.add_child(we)
	var sun := DirectionalLight3D.new()
	sun.rotation_degrees = Vector3(-35, 30, 0)
	root.add_child(sun)
	var lo := Vector3(1e9, 1e9, 1e9)
	var hi := Vector3(-1e9, -1e9, -1e9)
	for i in ex.centres.size():
		lo = lo.min((ex.centres[i] as Vector3) + (ex.dirs[i] as Vector3) * float(ex.moves[i]) - ex.halves[i])
		hi = hi.max((ex.centres[i] as Vector3) + (ex.dirs[i] as Vector3) * float(ex.moves[i]) + ex.halves[i])
	var c := (lo + hi) * 0.5
	var r := (hi - lo).length() * 0.5
	var cam := Camera3D.new()
	cam.fov = 40
	root.add_child(cam)
	for view in [["front", Vector3(0, 0, 1)], ["side", Vector3(1, 0, 0)]]:
		cam.look_at_from_position(c + (view[1] as Vector3) * r * 2.6, c, Vector3.UP)
		for f in 6:
			await process_frame
		await RenderingServer.frame_post_draw
		root.get_viewport().get_texture().get_image().save_png("%s/render_exploded_%s.png" % [out, view[0]])
	print("pieces: ", ex.pieces.size(), " -> ", out)
	quit(0)
