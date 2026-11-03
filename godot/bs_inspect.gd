extends SceneTree
# Inspect one FBX mesh's blend shapes: surfaces, vertices, blend_shape_mode, and whether the shape
# arrays look like offsets (small) or absolute positions (as large as the base).
# usage: godot --headless --path <proj> -s res://bs_inspect.gd -- <fbx> <mesh node name>


func _init() -> void:
	var a := OS.get_cmdline_user_args()
	var doc := FBXDocument.new()
	var st := FBXState.new()
	print("append_from_file err=", doc.append_from_file(a[0], st))
	var root_node: Node = doc.generate_scene(st)
	var mi := _find(root_node, a[1])
	if mi == null:
		print("no mesh named ", a[1])
		quit(1)
		return
	var m: ArrayMesh = mi.mesh
	print("mesh ", a[1], " surfaces=", m.get_surface_count(), " shapes=", m.get_blend_shape_count(), " blend_shape_mode=", m.blend_shape_mode)
	var total := 0
	for s in m.get_surface_count():
		var arr := m.surface_get_arrays(s)
		var v: PackedVector3Array = arr[Mesh.ARRAY_VERTEX]
		var idx = arr[Mesh.ARRAY_INDEX]
		var bs := m.surface_get_blend_shape_arrays(s)
		total += v.size()
		print("surface ", s, " vertices=", v.size(), " indices=", (idx.size() if idx != null else -1), " shape arrays=", bs.size(),
				" has_normal=", arr[Mesh.ARRAY_NORMAL] != null and (arr[Mesh.ARRAY_NORMAL] as PackedVector3Array).size() > 0)
		if bs.size() > 0:
			var sv: PackedVector3Array = bs[0][Mesh.ARRAY_VERTEX]
			var mb := 0.0
			var ms := 0.0
			for i in range(min(v.size(), sv.size())):
				mb = max(mb, v[i].abs().max_axis_index() * 0.0 + max(abs(v[i].x), max(abs(v[i].y), abs(v[i].z))))
				ms = max(ms, max(abs(sv[i].x), max(abs(sv[i].y), abs(sv[i].z))))
			print("  shape 0 '", m.get_blend_shape_name(0), "' vertex array size=", sv.size(), " max|base coord|=", mb, " max|shape coord|=", ms)
	print("total vertices ", total)
	var names := PackedStringArray()
	for i in m.get_blend_shape_count():
		names.append(str(m.get_blend_shape_name(i)))
	print("names with a '.' or other non-identifier char: ")
	var bad := 0
	for n in names:
		if not n.is_valid_ascii_identifier():
			bad += 1
			if bad <= 12:
				print("  ", n)
	print("non-identifier names: ", bad, " of ", names.size())
	quit()


func _find(n: Node, name: String) -> MeshInstance3D:
	if n is MeshInstance3D and n.name == name and n.mesh is ArrayMesh:
		return n
	for c in n.get_children():
		var r := _find(c, name)
		if r != null:
			return r
	return null
