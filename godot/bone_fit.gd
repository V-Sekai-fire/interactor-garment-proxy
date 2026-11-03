extends SceneTree
# bone_fit.gd: fit an outfit mesh's bone transforms so its EXISTING linear blend skinning reproduces target vertex positions.
# Weights and bind poses are held fixed (read from the outfit FBX); each weighted bone gets a world affine (12 unknowns) by
# ridge-regularised weighted least squares (the transform update of Dem Bones, closed form), then the affines are projected to
# Unity-style local TRS top-down through the bone hierarchy (the prefab YAML), with the error measured through the projected
# hierarchy, not through the affines. No Python; OpenUSD out (clip .usda, meshes .usdz).
#
# Spaces: Unity world (metres, Y up, left handed), as the exporter ChibifireFitExport.cs bakes. The FBX is read with Godot's
# FBXDocument (right handed); Unity = Godot with x negated (flip tried both ways, the one that reproduces the bake wins).
# The pose the bake was taken in is the prefab's: bone world matrices come from the prefab YAML (flat prefab, Transform docs).
#
# usage (double-precision engine, headless, from any project dir):
#   godot --headless --path <proj> -s res://bone_fit.gd -- \
#     --fbx <outfit.fbx> --mesh "(A)Skirt" --orig <garment_own_X.bin> (--target <positions.bin> | --deltas <deltas.json>) \
#     --out <dir> [--prefab <outfit.prefab>] [--usd-elf <usd.elf>] [--name ASkirt] [--lambda 1e-6] [--flip auto|-1|1]
#   --orig / --target: ChibifireFitExport .bin layout (i32 vertex count, i32 triangle count, float32 xyz per vertex, i32 triangle
#   corners), Unity world metres, in the Unity mesh's vertex order. --deltas: JSON {"deltas_unity_m": [[dx,dy,dz], ...]} added to --orig.
#   --max-influences N (default 4: Unity skins with at most 4 bones a vertex; 0 = all the FBX gives) --sweeps N (rigid sweeps, 200) --lambda L (fixed ridge; default a sweep when <= 175 bones, else 1e-3)
#   DEFAULT: only bones that no other mesh of the FBX weights and that carry no Miroir bone name may move (--exclusive 0 --analysis-not-applied 1 = analysis, writes no clip). --same-piece "(B)Kimono_Big" (variants of the same garment do not count as other pieces) --freeze "Name,Name" --miroir-prefab <Miroir-Re.prefab> --stage1-analysis <export dir> (analysis only: re-seat shared bones onto the Miroir skeleton, never written as a clip)
#   --prefab default: Nutty Mapley_SHINANO.prefab next to the FBX's folder. --usd-elf default: usd.post-blendshape.elf in scratchpad.
# Outputs in --out: <name>_bones_clip_trs.usda (rest frame 0, fitted frame 1), <name>_bones_clip_rt.usda (rigid only),
#   <name>_fitted_target.usdz, <name>_skinned_newbones.usdz, <name>_bind_skinned.usdz (bind-pose skinned mesh for OpenUSD LBS),
#   <name>_bone_fit_report.txt (all numbers), <name>_bones_local_trs.txt (per bone original and fitted local pos / quat / scale).
# Exit code 0 when the run completed and the reproduction/control checks passed.

var A := {}
var out_lines := PackedStringArray()
var fails := 0

func say(s: String) -> void:
	print(s)
	out_lines.append(s)

func check(label: String, ok: bool, detail := "") -> void:
	if not ok:
		fails += 1
	say("%s %s%s" % ["PASS" if ok else "FAIL", label, ("  [" + detail + "]") if detail != "" else ""])

func _init() -> void:
	var ua := OS.get_cmdline_user_args()
	var i := 0
	while i < ua.size():
		if ua[i].begins_with("--") and i + 1 < ua.size():
			if ua[i] == "--piece":
				piece_args.append(ua[i + 1])
			else:
				A[ua[i].substr(2)] = ua[i + 1]
			i += 2
		else:
			i += 1
	for k in ["fbx", "mesh", "orig", "out"]:
		if not A.has(k):
			print("missing --", k)
			quit(2)
			return
	if not (A.has("target") or A.has("deltas")):
		print("need --target or --deltas")
		quit(2)
		return
	run()
	quit(1 if fails > 0 else 0)

# ------------------------------------------------------------------ inputs

func read_bin(path: String) -> Dictionary:
	var f := FileAccess.open(path, FileAccess.READ)
	var n := f.get_32()
	var t := f.get_32()
	var p := PackedVector3Array()
	p.resize(n)
	for i in n:
		p[i] = Vector3(f.get_float(), f.get_float(), f.get_float())
	var tri := PackedInt32Array()
	tri.resize(t * 3)
	for i in t * 3:
		tri[i] = f.get_32()
	return {"p": p, "tri": tri}

func fid(line: String) -> int:
	var k := line.find("fileID: ")
	if k < 0:
		return 0
	var s := line.substr(k + 8)
	var e := 0
	while e < s.length() and (s[e] == "-" or (s[e] >= "0" and s[e] <= "9")):
		e += 1
	return int(s.substr(0, e))

func vec(line: String) -> PackedFloat64Array:
	var a := line.find("{")
	var b := line.find("}")
	var out := PackedFloat64Array()
	for part in line.substr(a + 1, b - a - 1).split(","):
		out.append(float(part.split(":")[1]))
	return out

# Flat prefab: {tf: {id: {go, rot, pos, scl, father}}, name: {goid: name}, smr: {goid: {bones: [ids]}}}
func parse_prefab(path: String) -> Dictionary:
	var tf := {}
	var names := {}
	var smr := {}
	var gocomp := {}
	var mono := {}
	var in_ch := false
	var typ := 0
	var id := 0
	var in_bones := false
	# YAML wraps long flow mappings ({x: ..., \n    w: ...}): join them back into one line first
	var merged := PackedStringArray()
	var pending := ""
	for raw in FileAccess.get_file_as_string(path).split("\n"):
		if pending != "":
			pending += " " + raw.strip_edges()
			if raw.contains("}"):
				merged.append(pending)
				pending = ""
		elif raw.contains("{") and not raw.contains("}"):
			pending = raw
		else:
			merged.append(raw)
	for line in merged:
		if line.begins_with("--- !u!"):
			var parts := line.split(" ")
			typ = int(parts[1].substr(3))
			id = int(parts[2].substr(1))
			in_bones = false
			if typ == 4:
				tf[id] = {"go": 0, "rot": Quaternion.IDENTITY, "pos": Vector3.ZERO, "scl": Vector3.ONE, "father": 0, "children": []}
			elif typ == 1:
				gocomp[id] = []
			elif typ == 114:
				mono[id] = {"go": 0, "fid": 0, "guid": "", "ref": "", "prefix": "", "suffix": ""}
			in_ch = false
			if typ == 137:
				smr[id] = {"go": 0, "bones": []}
			continue
		if typ == 1 and line.begins_with("  m_Name:"):
			names[id] = line.substr(9).strip_edges().trim_prefix("'").trim_suffix("'").trim_prefix("\"").trim_suffix("\"")
		elif typ == 1 and line.begins_with("  - component:"):
			gocomp[id].append(fid(line))
		elif typ == 114:
			if line.begins_with("  m_GameObject:"):
				mono[id]["go"] = fid(line)
			elif line.begins_with("  m_Script:"):
				mono[id]["fid"] = fid(line)
				var gi := line.find("guid: ")
				mono[id]["guid"] = line.substr(gi + 6, 32)
			elif line.begins_with("    referencePath:") and mono[id]["ref"] == "":
				mono[id]["ref"] = line.substr(18).strip_edges()
			elif line.begins_with("  prefix:"):
				mono[id]["prefix"] = line.substr(9).strip_edges()
			elif line.begins_with("  suffix:"):
				mono[id]["suffix"] = line.substr(9).strip_edges()
		elif typ == 4:
			if in_ch:
				if line.begins_with("  - {fileID:"):
					tf[id]["children"].append(fid(line))
					continue
				in_ch = false
			if line.begins_with("  m_Children:") and not line.contains("[]"):
				in_ch = true
			if line.begins_with("  m_GameObject:"):
				tf[id]["go"] = fid(line)
			elif line.begins_with("  m_LocalRotation:"):
				var q := vec(line)
				tf[id]["rot"] = Quaternion(q[0], q[1], q[2], q[3])
			elif line.begins_with("  m_LocalPosition:"):
				var q := vec(line)
				tf[id]["pos"] = Vector3(q[0], q[1], q[2])
			elif line.begins_with("  m_LocalScale:"):
				var q := vec(line)
				tf[id]["scl"] = Vector3(q[0], q[1], q[2])
			elif line.begins_with("  m_Father:"):
				tf[id]["father"] = fid(line)
		elif typ == 137:
			if line.begins_with("  m_GameObject:"):
				smr[id]["go"] = fid(line)
			if line.begins_with("  m_Bones:"):
				in_bones = true
			elif in_bones:
				if line.begins_with("  - {fileID:"):
					smr[id]["bones"].append(fid(line))
				else:
					in_bones = false
	return {"tf": tf, "name": names, "smr": smr, "gocomp": gocomp, "mono": mono}

var _wcache := {}
func world_of(P: Dictionary, id: int) -> Transform3D:
	if _wcache.has(id):
		return _wcache[id]
	var t: Dictionary = P["tf"][id]
	var local := Transform3D(Basis(t["rot"]) * Basis.from_scale(t["scl"]), t["pos"])
	var w := local
	if t["father"] != 0 and P["tf"].has(t["father"]):
		w = world_of(P, t["father"]) * local
	_wcache[id] = w
	return w

func tf_name(P: Dictionary, id: int) -> String:
	return P["name"].get(P["tf"][id]["go"], "?")

func find_mesh(n: Node, nm: String) -> MeshInstance3D:
	if n is MeshInstance3D and n.name == nm:
		return n
	for c in n.get_children():
		var r := find_mesh(c, nm)
		if r != null:
			return r
	return null

# ------------------------------------------------------------------ math

func gget(b: Basis, k: int, c: int) -> float:
	return (b.x if c == 0 else (b.y if c == 1 else b.z))[k]

func polar(m: Basis) -> Basis:
	var r := m
	for i in 60:
		var rit := r.inverse().transposed()
		var nr := Basis((r.x + rit.x) * 0.5, (r.y + rit.y) * 0.5, (r.z + rit.z) * 0.5)
		var d := (nr.x - r.x).length() + (nr.y - r.y).length() + (nr.z - r.z).length()
		r = nr
		if d < 1e-15:
			break
	return r

# Quaternion of a proper rotation matrix.
func quat_of(r: Basis) -> Quaternion:
	return r.get_rotation_quaternion()

func floats12(t: Transform3D) -> PackedFloat32Array:
	var o := PackedFloat32Array()
	for k in 3:
		for c in 3:
			o.append(gget(t.basis, k, c))
	o.append(t.origin.x)
	o.append(t.origin.y)
	o.append(t.origin.z)
	return o

func stats(e: PackedFloat64Array) -> Dictionary:
	var s := e.duplicate()
	s.sort()
	var sum := 0.0
	for v in e:
		sum += v
	var n := e.size()
	return {"max": s[n - 1], "mean": sum / n, "p99": s[int(0.99 * (n - 1))], "p50": s[n / 2]}

func fmt(d: Dictionary) -> String:
	return "max %.9f  mean %.9f  p99 %.9f  median %.9f" % [d["max"], d["mean"], d["p99"], d["p50"]]

# ------------------------------------------------------------------ skinning model

var nv := 0          # Unity vertex count
var gmap := PackedInt32Array()      # Unity vertex -> Godot vertex
var g_bones := PackedInt32Array()   # Godot vertex*stride + s -> bind index
var g_w := PackedFloat32Array()
var g_q := PackedVector3Array()     # bone-local bind position in Unity bone axes
var stride := 4

# world: Array[Transform3D] by bind index. Returns Unity-world positions.
func lbs(world: Array) -> PackedVector3Array:
	var out := PackedVector3Array()
	out.resize(nv)
	for u in nv:
		var g := gmap[u] * stride
		var p := Vector3.ZERO
		for s in stride:
			var w := g_w[g + s]
			if w > 0.0:
				p += (world[g_bones[g + s]] as Transform3D) * g_q[g + s] * w
		out[u] = p
	return out

func err_stats(a: PackedVector3Array, b: PackedVector3Array) -> Dictionary:
	var e := PackedFloat64Array()
	e.resize(a.size())
	for i in a.size():
		e[i] = a[i].distance_to(b[i])
	return stats(e)

func sumsq(a: PackedVector3Array, b: PackedVector3Array) -> float:
	var s := 0.0
	for i in a.size():
		s += a[i].distance_squared_to(b[i])
	return s

# ------------------------------------------------------------------ main

func run() -> void:
	var t0 := Time.get_ticks_msec()
	var fbx: String = A["fbx"]
	var name: String = A.get("name", String(A["mesh"]).replace("(", "").replace(")", "").replace(" ", "_"))
	var out_dir: String = A["out"]
	DirAccess.make_dir_recursive_absolute(out_dir)
	var prefab_path: String = A.get("prefab", fbx.get_base_dir().get_base_dir().path_join(fbx.get_base_dir().get_base_dir().get_file() + ".prefab"))
	var lam: float = float(A.get("lambda", "1e-6"))
	say("bone_fit: fbx=%s mesh=%s prefab=%s lambda=%.9f" % [fbx.get_file(), A["mesh"], prefab_path.get_file(), lam])

	# --- Unity side: orig + target positions
	var ob := read_bin(A["orig"])
	var orig: PackedVector3Array = ob["p"]
	nv = orig.size()
	var target := PackedVector3Array()
	if A.has("target"):
		var tb := read_bin(A["target"])
		target = tb["p"]
	else:
		var js = JSON.parse_string(FileAccess.get_file_as_string(A["deltas"]))
		var dl: Array = js["deltas_unity_m"]
		target.resize(nv)
		for i in nv:
			target[i] = orig[i] + Vector3(dl[i][0], dl[i][1], dl[i][2])
	if target.size() != nv:
		say("FAIL target has %d vertices, orig %d" % [target.size(), nv])
		fails += 1
		return
	say("vertices %d, triangles %d" % [nv, ob["tri"].size() / 3])

	# --- prefab: pose of the bake
	var P := parse_prefab(prefab_path)
	var smr_id := 0
	for id in P["smr"]:
		if P["name"].get(P["smr"][id]["go"], "") == A["mesh"]:
			smr_id = id
	if smr_id == 0:
		say("FAIL no SkinnedMeshRenderer on a GameObject named " + str(A["mesh"]))
		fails += 1
		return
	var ubones: Array = P["smr"][smr_id]["bones"]
	var uname := {}
	for tid in ubones:
		if tid == 0:
			continue
		var n := tf_name(P, tid)
		if uname.has(n):
			say("FAIL duplicate Unity bone name " + n)
			fails += 1
			return
		uname[n] = tid
	say("prefab: %d transforms, SMR bones %d" % [P["tf"].size(), ubones.size()])

	# --- Godot side: FBX skin
	var doc := FBXDocument.new()
	var st := FBXState.new()
	if doc.append_from_file(fbx, st) != OK:
		say("FAIL FBX did not load")
		fails += 1
		return
	var scene: Node = doc.generate_scene(st)
	var mi := find_mesh(scene, A["mesh"])
	if mi == null or mi.skin == null:
		say("FAIL FBX has no skinned mesh named " + str(A["mesh"]))
		fails += 1
		return
	var sk: Skeleton3D = mi.get_node(mi.skeleton)
	var skin: Skin = mi.skin
	var nb := skin.get_bind_count()
	var bind_tid := PackedInt64Array()  # bind index -> prefab transform id
	var binds: Array = []
	var bnames := PackedStringArray()
	for j in nb:
		var bn: String = skin.get_bind_name(j)
		if bn == "" and skin.get_bind_bone(j) >= 0:
			bn = sk.get_bone_name(skin.get_bind_bone(j))
		if not uname.has(bn):
			say("FAIL FBX bind %d '%s' is not an SMR bone in the prefab" % [j, bn])
			fails += 1
			return
		bind_tid.append(uname[bn])
		bnames.append(bn)
		binds.append(skin.get_bind_pose(j))
	# The Miroir body, its skeleton and its bones are READ-ONLY and are never written. Which outfit bones are the Miroir's
	# bones is decided by simulating Modular Avatar's MergeArmature (see ma_simulate for the code it follows).
	# Modes: default "exclusive" (also freeze every bone another piece weights and the ancestors of such bones);
	# --clothing-free 1 (every clothing bone moves except the MA-merged ones); --exclusive 0 --analysis-not-applied 1
	# (nothing frozen, analysis only, writes no clip).
	var cfree: bool = A.has("clothing-free")
	var excl: bool = (not cfree) and String(A.get("exclusive", "1")) != "0"
	analysis_only = ((not excl) and (not cfree)) or A.has("stage1-analysis")
	if not excl and not cfree and not A.has("analysis-not-applied"):
		say("FAIL --exclusive 0 moves Miroir-merged bones; it is analysis only: add --analysis-not-applied 1 (or use --clothing-free 1)")
		fails += 1
		return
	var frozen := {}
	shared_names = {}
	var stack: Array = [scene]
	var piece_nodes: Array = []
	var all_other_nodes: Array = []
	var same_list: PackedStringArray = String(A.get("same-piece", "")).split(",", false)
	var extra_names: PackedStringArray = PackedStringArray()
	for pa in piece_args:
		extra_names.append(String(pa).split(":")[0])
	while not stack.is_empty():
		var nd: Node = stack.pop_back()
		stack.append_array(nd.get_children())
		if nd is MeshInstance3D and nd != mi and nd.skin != null and nd.mesh is ArrayMesh:
			all_other_nodes.append(nd)
			if str(nd.name) in same_list or str(nd.name) in extra_names:
				continue
			piece_nodes.append(nd)
			for s in (nd.mesh as ArrayMesh).get_surface_count():
				var ar := (nd.mesh as ArrayMesh).surface_get_arrays(s)
				var bb2: PackedInt32Array = ar[Mesh.ARRAY_BONES]
				var ww2: PackedFloat32Array = ar[Mesh.ARRAY_WEIGHTS]
				for k in bb2.size():
					if ww2[k] > 0.0:
						var bi2: int = (nd.skin as Skin).get_bind_bone(bb2[k])
						shared_names[sk.get_bone_name(bi2) if bi2 >= 0 else (nd.skin as Skin).get_bind_name(bb2[k])] = true
	say("%d other skinned pieces in the FBX (%d count as 'other' for sharing); %d bones are weighted by at least one of them" % [all_other_nodes.size(), piece_nodes.size(), shared_names.size()])
	var miroir_path: String = A.get("miroir-prefab", fbx.get_base_dir().get_base_dir().get_base_dir().get_base_dir().path_join("BikitanAvatars/Miroir-Re/Miroir-Re.prefab"))
	miroir_names = {}
	if not FileAccess.file_exists(miroir_path):
		say("FAIL Miroir prefab not found at %s: pass --miroir-prefab" % miroir_path)
		fails += 1
		return
	var MP := parse_prefab(miroir_path)
	for id in MP["tf"]:
		miroir_names[tf_name(MP, id)] = true
	say("Miroir skeleton (READ-ONLY reference): %d transforms in %s" % [miroir_names.size(), miroir_path.get_file()])
	for l in merge_info(prefab_path):
		say("MergeArmature in the outfit prefab: " + l)
	var ma := ma_simulate(P, MP)
	if not ma["found"]:
		say("FAIL no MergeArmature component or no Miroir merge target found: cannot decide which bones are the Miroir's")
		fails += 1
		return
	ma_merged = ma["merged"]
	var merged_list := PackedStringArray()
	var name_only := PackedStringArray()
	for tid in ma_merged:
		merged_list.append(tf_name(P, tid))
	for tid in P["tf"]:
		if miroir_names.has(tf_name(P, tid)) and not ma_merged.has(tid) and bind_tid.has(tid):
			name_only.append(tf_name(P, tid))
	say("MA merge simulation (path match from the armature root, as ModularAvatarMergeArmature.GetBonesMapping): %d outfit transforms ARE Miroir bones (merged, frozen): %s" % [merged_list.size(), ", ".join(merged_list)])
	say("   outfit bones with a Miroir bone's NAME that are NOT merged (their parent path does not match, so MA keeps them as the outfit's own bones, free to move): %d: %s" % [name_only.size(), ", ".join(name_only)])
	say("   merged bones carrying a VRCPhysBone: %s; merged bones carrying other components: %s" % [str(ma["pb"]), str(ma["extra"])])
	var fz: PackedStringArray = String(A.get("freeze", "")).split(",", false)
	for j in nb:
		var nm_j: String = bnames[j]
		if ma_merged.has(bind_tid[j]) and not (not excl and not cfree):
			frozen[j] = true
		if excl and shared_names.has(nm_j):
			frozen[j] = true
		if nm_j in fz:
			frozen[j] = true
	if excl:
		# exclusive mode only: a movable bone drags its descendants, so a bone with an other-piece bone below it stays too
		var cannot := {}
		for tid in P["tf"]:
			if shared_names.has(tf_name(P, tid)):
				var c: int = P["tf"][tid]["father"]
				while c != 0 and P["tf"].has(c) and not cannot.has(c):
					cannot[c] = true
					c = P["tf"][c]["father"]
		var anc_frozen := PackedStringArray()
		for j in nb:
			if cannot.has(bind_tid[j]) and not frozen.has(j):
				frozen[j] = true
				anc_frozen.append(bnames[j])
		say("exclusive mode: frozen because an other-piece bone hangs below them: %d: %s" % [anc_frozen.size(), ", ".join(anc_frozen)])
	var fl := PackedStringArray()
	for j in frozen:
		fl.append(bnames[j])
	say("mode: %s. frozen (kept at their prefab transform): %d of %d bones" % ["clothing-free" if cfree else ("exclusive" if excl else "UNCONSTRAINED ANALYSIS"), frozen.size(), nb])
	# Godot vertices of every fitted piece, concatenated (the primary mesh first). Several pieces = ONE joint fit of the one
	# shared set of bone transforms: each piece keeps its own bind poses (for q) and vertex ranges; bone indices are remapped
	# to the primary mesh's bind indices (the same bones, matched by name) so one set of unknowns serves all of them.
	var maxinf := int(A.get("max-influences", "4"))
	var specs: Array = [{"mesh": A["mesh"], "primary": true, "weight": float(A.get("weight", "1"))}]
	for pa in piece_args:
		var ps := String(pa).split(":")
		if ps.size() < 3:
			say("FAIL --piece needs name:orig.bin:target.bin (or :deltas.json, or :reseat@<export dir> for a synthetic target), got " + String(pa))
			fails += 1
			return
		specs.append({"mesh": ps[0], "orig_path": ps[1], "tspec": ps[2], "primary": false, "weight": float(ps[3]) if ps.size() > 3 else 1.0})
	var tid2j := {}
	for j in nb:
		tid2j[bind_tid[j]] = j
	var gv := PackedVector3Array()
	var gw := PackedFloat32Array()
	var gb_all := PackedInt32Array()
	var gown_all := PackedInt32Array()
	var orig_all := PackedVector3Array()
	var target_all := PackedVector3Array()
	var tri_all := PackedInt32Array()
	seg_g0 = PackedInt32Array([0])
	seg_u0 = PackedInt32Array([0])
	seg_binds = []
	seg_names = PackedStringArray()
	seg_synth = []
	u_weight = PackedFloat64Array()
	for sp in specs:
		var node: MeshInstance3D = mi if sp["primary"] else find_mesh(scene, sp["mesh"])
		if node == null or node.skin == null:
			say("FAIL FBX has no skinned mesh named " + str(sp["mesh"]))
			fails += 1
			return
		var p_orig: PackedVector3Array
		var p_tri: PackedInt32Array
		var p_target: PackedVector3Array
		var synth := false
		if sp["primary"]:
			p_orig = orig
			p_tri = ob["tri"]
			p_target = target
		else:
			var pob := read_bin(sp["orig_path"])
			p_orig = pob["p"]
			p_tri = pob["tri"]
			var ts: String = sp["tspec"]
			if ts.begins_with("reseat@"):
				p_target = p_orig.duplicate()
				synth = true
				reseat_dir = ts.substr(7)
			elif ts.ends_with(".json"):
				var js2 = JSON.parse_string(FileAccess.get_file_as_string(ts))
				p_target = PackedVector3Array()
				for i in p_orig.size():
					p_target.append(p_orig[i] + Vector3(js2["deltas_unity_m"][i][0], js2["deltas_unity_m"][i][1], js2["deltas_unity_m"][i][2]))
			else:
				p_target = read_bin(ts)["p"]
			if p_target.size() != p_orig.size():
				say("FAIL piece %s: target has %d vertices, orig %d" % [sp["mesh"], p_target.size(), p_orig.size()])
				fails += 1
				return
		var sk2: Skeleton3D = node.get_node(node.skeleton)
		var skin2: Skin = node.skin
		var own_binds: Array = []
		var rm := PackedInt32Array()
		for j in skin2.get_bind_count():
			var bn2: String = skin2.get_bind_name(j)
			if bn2 == "" and skin2.get_bind_bone(j) >= 0:
				bn2 = sk2.get_bone_name(skin2.get_bind_bone(j))
			if not uname.has(bn2) or not tid2j.has(uname[bn2]):
				say("FAIL piece %s: bone '%s' is not one of %s's bones" % [sp["mesh"], bn2, A["mesh"]])
				fails += 1
				return
			own_binds.append(skin2.get_bind_pose(j))
			rm.append(tid2j[uname[bn2]])
		var pv := PackedVector3Array()
		var pb := PackedInt32Array()
		var pw := PackedFloat32Array()
		var m2: ArrayMesh = node.mesh
		for s in m2.get_surface_count():
			var arr := m2.surface_get_arrays(s)
			var v: PackedVector3Array = arr[Mesh.ARRAY_VERTEX]
			var bb: PackedInt32Array = arr[Mesh.ARRAY_BONES]
			var ww: PackedFloat32Array = arr[Mesh.ARRAY_WEIGHTS]
			var st2 := bb.size() / v.size()
			if gv.size() + pv.size() > 0 and st2 != stride:
				say("FAIL piece %s has %d influence slots per vertex, the first piece %d" % [sp["mesh"], st2, stride])
				fails += 1
				return
			stride = st2
			pv.append_array(v)
			pb.append_array(bb)
			pw.append_array(ww)
		if maxinf > 0:
			for i in pv.size():
				var order: Array = range(stride)
				order.sort_custom(func(x, y): return pw[i * stride + x] > pw[i * stride + y])
				var keep := 0.0
				for r in range(maxinf):
					keep += pw[i * stride + order[r]]
				for r in range(maxinf, stride):
					pw[i * stride + order[r]] = 0.0
				if keep > 0.0:
					for r in range(maxinf):
						pw[i * stride + order[r]] /= keep
		for k in pb.size():
			gb_all.append(rm[pb[k]])
			gown_all.append(pb[k])
		gv.append_array(pv)
		gw.append_array(pw)
		tri_all.append_array(PackedInt32Array(Array(p_tri).map(func(x): return x + orig_all.size())))
		orig_all.append_array(p_orig)
		target_all.append_array(p_target)
		seg_g0.append(gv.size())
		seg_u0.append(orig_all.size())
		seg_binds.append(own_binds)
		seg_names.append(str(sp["mesh"]))
		seg_synth.append(synth)
		for _u in p_orig.size():
			u_weight.append(float(sp["weight"]))
		say("piece %s: %d Unity vertices, %d Godot vertices%s" % [sp["mesh"], p_orig.size(), pv.size(), " (target = synthetic re-seat, set below)" if synth else ""])
	orig = orig_all
	target = target_all
	nv = orig.size()
	ob = {"p": orig_all, "tri": tri_all}
	g_bones = gb_all
	g_own = gown_all
	g_w = gw
	if maxinf > 0:
		say("weights truncated to the %d largest influences per vertex and renormalised" % maxinf)
	say("FBX: %d Godot vertices in %d piece(s), %d binds, %d influences per vertex; %d Unity vertices in total" % [gv.size(), specs.size(), nb, stride, nv])
	var wsum_dev := 0.0
	var more4 := 0
	for i in gv.size():
		var s := 0.0
		var nzc := 0
		for k in stride:
			s += gw[i * stride + k]
			if gw[i * stride + k] > 0.0:
				nzc += 1
		wsum_dev = maxf(wsum_dev, absf(s - 1.0))
		if nzc > 4:
			more4 += 1
	say("max |sum of weights - 1| = %.9f; vertices with more than 4 non-zero influences: %d of %d" % [wsum_dev, more4, gv.size()])

	# --- P matrices (Unity world of each bind's bone in the prefab pose)
	_wcache.clear()
	var Pm: Array = []
	for j in nb:
		Pm.append(world_of(P, bind_tid[j]))

	# --- correspondence + sign of the x flip: try each, keep the one that reproduces the bake best
	var best := {}
	var flips := [-1.0, 1.0] if A.get("flip", "auto") == "auto" else [float(A["flip"])]
	for sgn in flips:
		build_q(gv, binds, sgn)
		identity_map(gv.size())
		var how := "identity order where the counts agree"
		var pos_id := lbs(Pm)
		for k in seg_names.size():
			var worst := 1e30
			if seg_g0[k + 1] - seg_g0[k] == seg_u0[k + 1] - seg_u0[k]:
				worst = 0.0
				for u in range(seg_u0[k], seg_u0[k + 1]):
					worst = maxf(worst, pos_id[u].distance_to(orig[u]))
			if worst > 1e-4:
				build_map(gv, Pm, orig, k)
				how = "matched by posed position (Godot and Unity vertex orders differ)"
		var e := err_stats(lbs(Pm), orig)
		say("flip x by %+d: reproduction of the bake (%s): %s" % [int(sgn), how, fmt(e)])
		if best.is_empty() or e["max"] < best["e"]["max"]:
			best = {"sgn": sgn, "e": e, "map": gmap.duplicate()}
	build_q(gv, binds, best["sgn"])
	gmap = best["map"]
	var pose_orig := lbs(Pm)
	var e0 := err_stats(pose_orig, orig)
	say("bake reproduction (FBX skin at the prefab pose vs garment_own bin, all fitted pieces): %s  [x sign %+d]" % [fmt(e0), int(best["sgn"])])
	check("skin at the prefab pose reproduces the baked vertices to 1e-4 m (max)", e0["max"] < 1e-4, fmt(e0))
	if seg_names.size() > 1:
		for k in seg_names.size():
			var ek := err_stats(pose_orig.slice(seg_u0[k], seg_u0[k + 1]), orig.slice(seg_u0[k], seg_u0[k + 1]))
			say("   piece %s: bake reproduction %s" % [seg_names[k], fmt(ek)])
	var chain := build_chain(P, ubones)
	for k in seg_names.size():
		if seg_synth[k]:
			var rs := reseat_worlds(P, chain, reseat_dir)
			var Pr: Array = []
			for j in nb:
				Pr.append(rs[bind_tid[j]])
			var pos_rs := lbs(Pr)
			var mv_sum := 0.0
			for u in range(seg_u0[k], seg_u0[k + 1]):
				target[u] = orig[u] + pos_rs[u] - pose_orig[u]
				mv_sum += (pos_rs[u] - pose_orig[u]).length()
			say("SYNTHETIC target for piece %s (not a cloth-fit result): its original vertices skinned with the translation re-seat of the paired joints; mean movement %.4f m. It exercises the joint fit only." % [seg_names[k], mv_sum / (seg_u0[k + 1] - seg_u0[k])])
	if A.has("stage1-analysis"):
		# ANALYSIS ONLY, never applied: re-seat the clothing skeleton onto the Miroir skeleton. The Miroir is only the
		# target reference. Per-bone translation of the paired joints (joints.txt names, source_skeleton.obj = clothing,
		# target_skeleton.obj = Miroir), carried to every descendant (a child moves with its nearest paired ancestor).
		say("=== STAGE 1 ANALYSIS (re-seat the shared clothing skeleton on the Miroir skeleton): ANALYSIS NOT APPLIED, nothing here is a deliverable ===")
		var ed: String = A["stage1-analysis"]
		var jn := FileAccess.get_file_as_string(ed.path_join("joints.txt")).split("\n", false)
		var sv := obj_verts(ed.path_join("source_skeleton.obj"))
		var tv := obj_verts(ed.path_join("target_skeleton.obj"))
		var off := {}
		for i in jn.size():
			off[jn[i].strip_edges()] = tv[i] - sv[i]
			say("   paired joint %-12s clothing -> Miroir offset (%+.4f, %+.4f, %+.4f) m" % [jn[i].strip_edges(), off[jn[i].strip_edges()].x, off[jn[i].strip_edges()].y, off[jn[i].strip_edges()].z])
		var sane := 0.0
		for tid in chain:
			var nmt := tf_name(P, tid)
			if off.has(nmt):
				sane = maxf(sane, (world_of(P, tid).origin - sv[jn.find(nmt)]).length())
		say("   check: clothing joints in the prefab vs source_skeleton.obj, max distance %.9f m" % sane)
		var moved_n := 0
		for tid in chain:
			var c: int = tid
			var o := Vector3.ZERO
			while c != 0 and P["tf"].has(c):
				if off.has(tf_name(P, c)):
					o = off[tf_name(P, c)]
					break
				c = P["tf"][c]["father"]
			var w0 := world_of(P, tid)
			base_w[tid] = Transform3D(w0.basis, w0.origin + o)
			if o != Vector3.ZERO:
				moved_n += 1
		say("   stage 1 translates %d of %d hierarchy joints (nearest paired ancestor's offset)" % [moved_n, chain.size()])
		var Pm1: Array = []
		for j in nb:
			Pm1.append(base_w[bind_tid[j]])
		var s1 := lbs(Pm1)
		var e_s1 := err_stats(s1, target)
		var bsq := sumsq(target, orig)
		say("   STAGE 1 ALONE, skirt: error vs cloth-fit target: %s  explained %.4f%% (unfitted movement mean %.6f)" % [fmt(e_s1), 100.0 * (1.0 - sumsq(s1, target) / bsq), err_stats(orig, target)["mean"]])
		var p0m := {}
		for tid in chain:
			p0m[tid] = world_of(P, tid)
		for nd in piece_nodes:
			var pm := piece_movement(nd, sk, uname, p0m, base_w, best["sgn"], int(A.get("max-influences", "4")))
			say("   stage 1 moves piece %-14s: %d vertices, movement mean %.4f m max %.4f m, mean vector (%+.4f, %+.4f, %+.4f)" % [nd.name, pm["n"], pm["mean"], pm["max"], pm["vec"].x, pm["vec"].y, pm["vec"].z])
		var skm := piece_movement(mi, sk, uname, p0m, base_w, best["sgn"], int(A.get("max-influences", "4")))
		say("   (the skirt itself, stage 1 movement: mean %.4f m max %.4f m)" % [skm["mean"], skm["max"]])
		Pm = Pm1
		pose_orig = lbs(Pm)
		say("=== STAGE 2 ANALYSIS: exclusive bones, rigid, fitted on top of stage 1 (analysis not applied) ===")

	# --- bones with weight
	var used := PackedInt32Array()
	var slot := {}
	var wtot := PackedFloat64Array()
	wtot.resize(nb)
	for u in nv:
		var g := gmap[u] * stride
		for s in stride:
			if g_w[g + s] > 0.0:
				wtot[g_bones[g + s]] += g_w[g + s] * u_weight[u]
	for j in nb:
		if wtot[j] > 0.0 and not frozen.has(j):
			slot[j] = used.size()
			used.append(j)
	if used.is_empty():
		say("FAIL no movable bone carries weight for this mesh (every weighted bone is shared with another piece or merged onto the Miroir). If another mesh of the FBX is a variant of the same garment, pass --same-piece \"(B)Kimono_Big\".")
		fails += 1
		var rf0 := FileAccess.open(out_dir.path_join(name + "_bone_fit_report.txt"), FileAccess.WRITE)
		rf0.store_string("\n".join(out_lines) + "\n")
		rf0.close()
		return
	var n := used.size()
	var d := 4 * n
	say("bones in skin %d, with weight %d  ->  %d unknowns (12 per bone), %d equations (3 per vertex)" % [nb, n, 12 * n, 3 * nv])

	# --- normal equations (the three output coordinates share the matrix)
	var ata := PackedFloat64Array()
	ata.resize(d * d)
	var rhs := [PackedFloat64Array(), PackedFloat64Array(), PackedFloat64Array()]
	for k in 3:
		rhs[k].resize(d)
	var res := PackedVector3Array()
	res.resize(nv)
	for u in nv:
		res[u] = target[u] - pose_orig[u]
	for u in nv:
		var g := gmap[u] * stride
		var uw: float = u_weight[u]
		var sl := PackedInt32Array()
		var ww := PackedFloat64Array()
		var rr: Array = []
		for s in stride:
			var w := g_w[g + s]
			if w > 0.0 and slot.has(g_bones[g + s]):
				sl.append(slot[g_bones[g + s]])
				ww.append(w)
				var q := g_q[g + s]
				rr.append([q.x, q.y, q.z, 1.0])
		for a in sl.size():
			var ra: Array = rr[a]
			for k in 3:
				var rv: float = res[u][k]
				for p in 4:
					rhs[k][sl[a] * 4 + p] += uw * ww[a] * ra[p] * rv
			for b in sl.size():
				var rb: Array = rr[b]
				var coef: float = uw * ww[a] * ww[b]
				for p in 4:
					var base: int = (sl[a] * 4 + p) * d + sl[b] * 4
					var cp: float = coef * ra[p]
					ata[base] += cp * rb[0]
					ata[base + 1] += cp * rb[1]
					ata[base + 2] += cp * rb[2]
					ata[base + 3] += cp * rb[3]
	var tr := 0.0
	for i in d:
		tr += ata[i * d + i]
	say("normal matrix %dx%d, trace %.9f" % [d, d, tr])
	say("hierarchy: %d joints (the SMR bones plus their ancestors)" % chain.size())
	var delta_mag := PackedFloat64Array()
	delta_mag.resize(nv)
	for u in nv:
		delta_mag[u] = (target[u] - orig[u]).length()
	var base_sq := sumsq(target, orig)
	say("--- movement: |target - baked original|: %s" % fmt(stats(delta_mag)))
	say("unfitted (original bones): error vs target = the movement above (explained 0 percent)")

	# --- A: affine (12 DOF per bone) by ridge least squares, ridge swept; projection to TRS through the hierarchy
	var lams: Array = [lam] if A.has("lambda") else ([1e-6, 1e-5, 1e-4, 1e-3, 1e-2, 1e-1, 1.0] if d <= 700 else [1e-3])
	var best_aff := {}
	say("--- ridge sweep (ridge = lambda * trace / dim; affine = world affine as solved; TRS = projected to local TRS top-down and skinned through the hierarchy)")
	for L in lams:
		var Nl := solve_affine(ata.duplicate(), rhs, d, L * tr / d, slot, used, Pm)
		var ea := err_stats(lbs(Nl), target)
		var rzl := realize(P, chain, bind_tid, Nl, used, "trs")
		var wbl: Array = []
		for j in nb:
			wbl.append(rzl["world"][bind_tid[j]])
		var et := err_stats(lbs(wbl), target)
		say("  lambda %.9f: affine mean %.6f max %.6f | TRS mean %.6f max %.6f | max shear %.4f, max |scale-1| %.4f" % [L, ea["mean"], ea["max"], et["mean"], et["max"], rzl["shear"], rzl["scale_dev"]])
		if best_aff.is_empty() or et["mean"] < best_aff["et"]["mean"]:
			best_aff = {"L": L, "N": Nl, "ea": ea, "et": et}
	var Nw: Array = best_aff["N"]
	say("chosen lambda %.9f (lowest TRS mean error)" % best_aff["L"])
	say("full affine (12 DOF per bone, world): error vs target: %s  explained %.4f%%" % [fmt(best_aff["ea"]), 100.0 * (1.0 - sumsq(lbs(Nw), target) / base_sq)])
	seg_report("affine", lbs(Nw), target, orig)
	say("solved in %d ms" % (Time.get_ticks_msec() - t0))

	# --- B: rigid only (rotation + translation per bone), block coordinate descent, weights fixed (what Dem Bones does)
	var Nr := rigid_fit(Nw, used, target, nb, int(A.get("sweeps", "200")))
	var results := {}
	var variants := {"trs": Nw, "rt": Nr}
	for mode in ["trs", "rt"]:
		var rz := realize(P, chain, bind_tid, variants[mode], used, mode)
		var W: Dictionary = rz["world"]
		var wb: Array = []
		for j in nb:
			wb.append(W[bind_tid[j]])
		var pos := lbs(wb)
		var es := err_stats(pos, target)
		var expl := 100.0 * (1.0 - sumsq(pos, target) / base_sq)
		results[mode] = {"pos": pos, "rz": rz, "e": es, "wb": wb}
		say("%s (through the local hierarchy): error vs target: %s  explained %.4f%%" % [("TRS from the affine fit: rotation + per-axis scale + translation" if mode == "trs" else "RT, fitted rigid directly: rotation + translation, scale 1"), fmt(es), expl])
		say("   vs original positions: moved %s" % fmt(err_stats(pos, orig)))
		seg_report(mode, pos, target, orig)
		say("   projection stats (affine to TRS): max shear (relative) %.9f, max |scale-1| %.9f, max non-uniform scale spread %.9f" % [rz["shear"], rz["scale_dev"], rz["nonuni"]])

	var mt_max := 0.0
	var mt_sum := 0.0
	var ang_max := 0.0
	var ang_sum := 0.0
	var tops := []
	for j in used:
		var dt := (Nr[j] as Transform3D).origin.distance_to((Pm[j] as Transform3D).origin)
		var rel: Basis = (polar((Pm[j] as Transform3D).basis)).inverse() * (Nr[j] as Transform3D).basis
		var an := rad_to_deg(rel.get_rotation_quaternion().get_angle())
		mt_max = maxf(mt_max, dt)
		mt_sum += dt
		ang_max = maxf(ang_max, an)
		ang_sum += an
		tops.append([wtot[j], tf_name(P, bind_tid[j]), dt, an])
	tops.sort_custom(func(x, y): return x[0] > y[0])
	say("--- rigid bone motion (world, %d weighted bones): origin moves mean %.4f m max %.4f m; rotation change mean %.2f deg max %.2f deg" % [used.size(), mt_sum / used.size(), mt_max, ang_sum / used.size(), ang_max])
	for e in mini(8, tops.size()):
		say("   heaviest bone %d: %s (total weight %.1f): origin moves %.4f m, rotates %.2f deg" % [e + 1, tops[e][1], tops[e][0], tops[e][2], tops[e][3]])

	# --- which bones moved, and the Miroir / shared assertions
	var rzr: Dictionary = results[primary_key]["rz"]
	var moved_names := PackedStringArray()
	var moved_shared := PackedStringArray()
	var follower_names := PackedStringArray()
	var bad_miroir := PackedStringArray()
	var bad_shared := PackedStringArray()
	var untouched_dev := 0.0
	for tid in chain:
		var tpf: Dictionary = P["tf"][tid]
		var l: Dictionary = rzr["local"][tid]
		var dloc: float = (l["pos"] - tpf["pos"]).length() + (absf((l["quat"] as Quaternion).x - (tpf["rot"] as Quaternion).x) + absf((l["quat"] as Quaternion).y - (tpf["rot"] as Quaternion).y) + absf((l["quat"] as Quaternion).z - (tpf["rot"] as Quaternion).z) + absf((l["quat"] as Quaternion).w - (tpf["rot"] as Quaternion).w)) + (l["scl"] - tpf["scl"]).length()
		var nmx := tf_name(P, tid)
		var wd: float = ((rzr["world"][tid] as Transform3D).origin - (base_w.get(tid, world_of(P, tid)) as Transform3D).origin).length()
		if dloc > 1e-6:
			moved_names.append(nmx)
			if ma_merged.has(tid):
				bad_miroir.append(nmx)
			if shared_names.has(nmx):
				bad_shared.append(nmx)
				moved_shared.append(nmx)
		elif wd > 1e-7:
			follower_names.append(nmx)
			if ma_merged.has(tid):
				bad_miroir.append(nmx + " (follower)")
			if shared_names.has(nmx):
				bad_shared.append(nmx + " (follower)")
				moved_shared.append(nmx + " (follower)")
		else:
			untouched_dev = maxf(untouched_dev, dloc)
	say("--- BONES MOVED (rigid fit): %d: %s" % [moved_names.size(), ", ".join(moved_names)])
	say("--- of which weighted by another piece of the outfit (shared): %d: %s" % [moved_shared.size(), ", ".join(moved_shared)])
	say("--- of which MA-merged (the Miroir's bones, must be 0): %d" % bad_miroir.size())
	say("--- bones that follow a moved parent unchanged in local space (world moves, nothing authored): %d: %s" % [follower_names.size(), ", ".join(follower_names)])
	check("ASSERT (a) no moved or following bone is a Miroir bone: none of the %d MA-merged outfit transforms changed, and every written joint is an outfit transform (the Miroir prefab is only read)" % ma_merged.size(), bad_miroir.is_empty(), ", ".join(bad_miroir))
	if excl:
		check("ASSERT (b, exclusive mode) no moved or following bone is weighted by another piece of the outfit", bad_shared.is_empty(), ", ".join(bad_shared))
	check("ASSERT every other joint keeps its prefab local transform (max deviation %.9f)" % untouched_dev, untouched_dev < 1e-6)
	# coverage by movable weight
	var edges := [0.0, 0.001, 0.25, 0.5, 0.75, 0.999]
	var lab := ["= 0", "(0, 0.25]", "(0.25, 0.5]", "(0.5, 0.75]", "(0.75, 0.999)", ">= 0.999"]
	var grp_n := [0, 0, 0, 0, 0, 0]
	var grp_e := [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
	var grp_s := [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
	var grp_m := [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
	var tot_s := 0.0
	var tot_m := 0.0
	for u in nv:
		var wm := 0.0
		var g := gmap[u] * stride
		for s in stride:
			if g_w[g + s] > 0.0 and slot.has(g_bones[g + s]):
				wm += g_w[g + s]
		var gi := 5 if wm >= 0.999 else (4 if wm > 0.75 else (3 if wm > 0.5 else (2 if wm > 0.25 else (1 if wm > 0.001 else 0))))
		var ee: float = results[primary_key]["pos"][u].distance_to(target[u])
		var mv: float = (target[u] - orig[u]).length_squared()
		grp_n[gi] += 1
		grp_e[gi] += ee
		grp_s[gi] += ee * ee
		grp_m[gi] += mv
		tot_s += ee * ee
		tot_m += mv
	say("--- movable weight per vertex (sum of the weights of bones allowed to move), %d vertices:" % nv)
	for gi in 6:
		say("   movable weight %-14s: %5d vertices, mean error %.6f m, share of the residual squared error %5.1f%%, share of the cloth-fit movement squared %5.1f%%" % [lab[gi], grp_n[gi], grp_e[gi] / maxi(grp_n[gi], 1), 100.0 * grp_s[gi] / maxf(tot_s, 1e-12), 100.0 * grp_m[gi] / maxf(tot_m, 1e-12)])
	# what the moved bones do to the other pieces of the FBX (their original vertices skinned with the new bones)
	var p0m := {}
	for tid in chain:
		p0m[tid] = world_of(P, tid)
	for nd in all_other_nodes:
		var pmv := piece_movement(nd, sk, uname, p0m, rzr["world"], best["sgn"], int(A.get("max-influences", "4")))
		var fitted_here: bool = str(nd.name) in extra_names
		say("   effect on piece %-14s %s: %d vertices, movement mean %.4f m max %.4f m, mean vector (%+.4f, %+.4f, %+.4f)" % [nd.name, "(joint-fitted, has a target)" if fitted_here else "(movement only, no cloth-fit target)", pmv["n"], pmv["mean"], pmv["max"], pmv["vec"].x, pmv["vec"].y, pmv["vec"].z])


	# --- planted control: move the heaviest bone 1 cm along x in world; the skinned vertices must move by weight * 1 cm
	var heavy: int = used[0]
	for j in used:
		if wtot[j] > wtot[heavy]:
			heavy = j
	var wb2: Array = results[primary_key]["wb"].duplicate()
	wb2[heavy] = Transform3D(wb2[heavy].basis, wb2[heavy].origin + Vector3(0.01, 0, 0))
	var pert := lbs(wb2)
	var maxpred := 0.0
	var maxact := 0.0
	var mism := 0.0
	for u in nv:
		var g := gmap[u] * stride
		var wh := 0.0
		for s in stride:
			if g_bones[g + s] == heavy:
				wh += g_w[g + s]
		var act: float = (pert[u] - results[primary_key]["pos"][u]).length()
		maxpred = maxf(maxpred, wh * 0.01)
		maxact = maxf(maxact, act)
		mism = maxf(mism, absf(act - wh * 0.01))
	var e_pert := err_stats(pert, target)
	var mw_before := 0.0
	var mw_after := 0.0
	var mw_n := 0.0
	for u in nv:
		if u_weight[u] > 0.0:
			mw_before += results[primary_key]["pos"][u].distance_to(target[u]) * u_weight[u]
			mw_after += pert[u].distance_to(target[u]) * u_weight[u]
			mw_n += u_weight[u]
	check("planted control: bone %s off by 1 cm in x moves vertices by weight * 1 cm (max predicted %.9f, measured %.9f, worst mismatch %.9f)" % [tf_name(P, bind_tid[heavy]), maxpred, maxact, mism], mism < 1e-7 and maxact > 0.0)
	check("planted control: the (weighted) mean error vs target rose (%.9f -> %.9f)" % [mw_before / maxf(mw_n, 1e-12), mw_after / maxf(mw_n, 1e-12)], mw_after > mw_before)

	# --- rest-pose offset share: how much of the movement one uniform translation explains
	var mean_d := Vector3.ZERO
	for u in nv:
		mean_d += target[u] - orig[u]
	mean_d /= nv
	var res_u := 0.0
	for u in nv:
		res_u += (target[u] - orig[u] - mean_d).length_squared()
	say("--- rest-pose offset: mean movement (%.4f, %.4f, %.4f) m; one uniform translation explains %.2f%% of the squared movement" % [mean_d.x, mean_d.y, mean_d.z, 100.0 * (1.0 - res_u / base_sq)])
	var mt := Vector3.ZERO
	var cnt := 0
	for j in used:
		mt += (Nw[j] as Transform3D).origin - (Pm[j] as Transform3D).origin
		cnt += 1
	mt /= maxi(cnt, 1)
	say("   mean world displacement of the weighted bone origins (affine fit): (%.4f, %.4f, %.4f) m over %d bones" % [mt.x, mt.y, mt.z, cnt])

	# --- deliverables
	var elf_path: String = A.get("usd-elf", "/private/tmp/claude-502/-Users-ernest-lee-Desktop-entities-fabric/b8beae81-4e98-4e20-8054-433eb7abd11b/scratchpad/usd.post-blendshape.elf")
	var report_name := name + "_bone_fit_report.txt"
	if analysis_only:
		report_name = name + "_analysis_not_applied_report.txt"
		say("ANALYSIS NOT APPLIED: no clip, table or mesh is written for this run")
	else:
		write_local_table(out_dir.path_join(name + "_bones_local_%s.txt" % primary_key), P, chain, results[primary_key]["rz"])
		write_local_table(out_dir.path_join(name + "_bones_local_trs.txt"), P, chain, results["trs"]["rz"])
		write_usd(out_dir, name, elf_path, P, chain, bind_tid, binds, best["sgn"], mi, gv, ob, orig, target, results, nb)
	var rf := FileAccess.open(out_dir.path_join(report_name), FileAccess.WRITE)
	rf.store_string("\n".join(out_lines) + "\n")
	rf.close()
	say("done in %d ms, %d failed checks" % [Time.get_ticks_msec() - t0, fails])

func solve_affine(a: PackedFloat64Array, rhs: Array, d: int, ridge: float, slot: Dictionary, used: PackedInt32Array, Pm: Array) -> Array:
	for i in d:
		a[i * d + i] += ridge
	var X := cholesky_solve(a, rhs, d)
	var Nw: Array = Pm.duplicate()
	for j in used:
		var s: int = slot[j]
		var cols: Array = []
		for c in 3:
			var col := Vector3()
			for k in 3:
				col[k] = gget(Pm[j].basis, k, c) + X[k][s * 4 + c]
			cols.append(col)
		var org := Vector3()
		for k in 3:
			org[k] = Pm[j].origin[k] + X[k][s * 4 + 3]
		Nw[j] = Transform3D(Basis(cols[0], cols[1], cols[2]), org)
	return Nw

# Rigid-only fit: per bone a rotation + translation, the other bones held, weighted Procrustes (weight w^2) on the
# residual target y = (target - rest_of_the_skin) / w; sweeps over the bones until nothing moves. Starts from the
# rotation (polar part) of the given affines.
func rigid_fit(N0: Array, used: PackedInt32Array, target: PackedVector3Array, nb: int, sweeps: int) -> Array:
	var N: Array = N0.duplicate()
	for j in used:
		var t: Transform3D = N[j]
		N[j] = Transform3D(polar(t.basis), t.origin)
	var lu := {}
	var lw := {}
	var lq := {}
	for j in used:
		lu[j] = PackedInt32Array()
		lw[j] = PackedFloat64Array()
		lq[j] = PackedVector3Array()
	for u in nv:
		var g := gmap[u] * stride
		for s in stride:
			var w := g_w[g + s]
			if w > 0.0 and lu.has(g_bones[g + s]):
				var j := g_bones[g + s]
				lu[j].append(u)
				lw[j].append(w)
				lq[j].append(g_q[g + s])
	# the translation start: refit t for the polar rotation, then sweep
	var cur := lbs(N)
	for sw in sweeps:
		var moved := 0.0
		for j in used:
			var us: PackedInt32Array = lu[j]
			var ws: PackedFloat64Array = lw[j]
			var qs: PackedVector3Array = lq[j]
			var old: Transform3D = N[j]
			var sw_ := 0.0
			var qb := Vector3.ZERO
			var yb := Vector3.ZERO
			var ys := PackedVector3Array()
			ys.resize(us.size())
			for e in us.size():
				var w := ws[e]
				var y := (target[us[e]] - cur[us[e]] + old * qs[e] * w) / w
				ys[e] = y
				var om := w * w * u_weight[us[e]]
				sw_ += om
				qb += qs[e] * om
				yb += y * om
			if sw_ <= 0.0:
				continue
			qb /= sw_
			yb /= sw_
			var c0 := Vector3.ZERO
			var c1 := Vector3.ZERO
			var c2 := Vector3.ZERO
			var spread := 0.0
			for e in us.size():
				var om := ws[e] * ws[e] * u_weight[us[e]]
				var dq := qs[e] - qb
				var dy := ys[e] - yb
				c0 += dy * (om * dq.x)
				c1 += dy * (om * dq.y)
				c2 += dy * (om * dq.z)
				spread += om * dq.length_squared()
			var rprev := polar(old.basis)
			var mu := 1e-3 * spread
			var M := Basis(c0 + rprev.x * mu, c1 + rprev.y * mu, c2 + rprev.z * mu)
			var R := polar(M)
			if R.determinant() < 0.0:
				R = rprev
			var nt := Transform3D(R, yb - R * qb)
			for e in us.size():
				cur[us[e]] += ((nt * qs[e]) - (old * qs[e])) * ws[e]
			moved = maxf(moved, nt.origin.distance_to(old.origin))
			N[j] = nt
		if sw == 0 or sw == 4 or sw == 9 or sw == 19 or sw == 39 or sw == sweeps - 1:
			say("  rigid sweep %d: mean error vs target %.6f max %.6f, largest bone translation change %.9f" % [sw + 1, err_stats(cur, target)["mean"], err_stats(cur, target)["max"], moved])
	return N

func scene_xf(n: Node) -> Transform3D:
	var t := Transform3D.IDENTITY
	var c := n
	while c != null and c is Node3D:
		t = (c as Node3D).transform * t
		c = c.get_parent()
	return t

func identity_map(_n: int) -> void:
	gmap = PackedInt32Array()
	gmap.resize(nv)
	for k in seg_names.size():
		var cnt_g := seg_g0[k + 1] - seg_g0[k]
		for u in range(seg_u0[k], seg_u0[k + 1]):
			gmap[u] = seg_g0[k] + mini(u - seg_u0[k], cnt_g - 1)

# q = C * (bind * v), C = diag(sgn, 1, 1): the vertex in its bone's Unity axes (each piece with its own bind poses)
func build_q(gv: PackedVector3Array, _binds: Array, sgn: float) -> void:
	g_q = PackedVector3Array()
	g_q.resize(gv.size() * stride)
	for k in seg_names.size():
		var bk: Array = seg_binds[k]
		for i in range(seg_g0[k], seg_g0[k + 1]):
			for s in stride:
				var q: Vector3 = (bk[g_own[i * stride + s]] as Transform3D) * gv[i]
				g_q[i * stride + s] = Vector3(q.x * sgn, q.y, q.z)

# Godot vertex counts can exceed Unity's (split vertices) and the orders differ: match by posed position, piece by piece.
func build_map(gv: PackedVector3Array, Pm: Array, orig: PackedVector3Array, seg: int) -> void:
	var g0 := seg_g0[seg]
	var g1 := seg_g0[seg + 1]
	var u0 := seg_u0[seg]
	var u1 := seg_u0[seg + 1]
	var gp := PackedVector3Array()
	gp.resize(g1 - g0)
	for i in range(g0, g1):
		var p := Vector3.ZERO
		for s in stride:
			var w := g_w[i * stride + s]
			if w > 0.0:
				p += (Pm[g_bones[i * stride + s]] as Transform3D) * g_q[i * stride + s] * w
		gp[i - g0] = p
	var cell := 5e-3
	var grid := {}
	for i in gp.size():
		var k := Vector3i(floori(gp[i].x / cell), floori(gp[i].y / cell), floori(gp[i].z / cell))
		if not grid.has(k):
			grid[k] = PackedInt32Array()
		grid[k].append(i)
	if gmap.size() != nv:
		gmap = PackedInt32Array()
		gmap.resize(nv)
	var bad := 0
	var ambig := 0
	for u in range(u0, u1):
		var o := orig[u]
		var k := Vector3i(floori(o.x / cell), floori(o.y / cell), floori(o.z / cell))
		var bi := -1
		var bd := 1e30
		for dx in range(-1, 2):
			for dy in range(-1, 2):
				for dz in range(-1, 2):
					var kk := Vector3i(k.x + dx, k.y + dy, k.z + dz)
					if grid.has(kk):
						for i in grid[kk]:
							var dd := gp[i].distance_squared_to(o)
							if dd < bd:
								bd = dd
								bi = i
		if bi < 0:
			bad += 1
			bi = 0
		gmap[u] = g0 + bi
		if bd < 1e-10:
			var wmax := 0.0
			for dx in range(-1, 2):
				for dy in range(-1, 2):
					for dz in range(-1, 2):
						var kk := Vector3i(k.x + dx, k.y + dy, k.z + dz)
						if grid.has(kk):
							for i in grid[kk]:
								if i != bi and gp[i].distance_squared_to(o) < 1e-10:
									wmax = maxf(wmax, wdiff(g0 + i, g0 + bi))
			if wmax > 1e-4:
				ambig += 1
	if bad > 0:
		say("  (candidate pass, %s: %d Unity vertices had no Godot vertex within 5 mm of the posed position)" % [seg_names[seg], bad])
	var used_g := {}
	var dup := 0
	for u in range(u0, u1):
		if used_g.has(gmap[u]):
			dup += 1
		used_g[gmap[u]] = true
	say("position match (%s): %d Unity vertices, %d distinct Godot vertices used, %d reused (coincident seam vertices); %d Unity vertices have a coincident Godot vertex with different weights" % [seg_names[seg], u1 - u0, used_g.size(), dup, ambig])

func cholesky_solve(a: PackedFloat64Array, rhs: Array, d: int) -> Array:
	for j in d:
		var s := a[j * d + j]
		var rj := j * d
		for k in j:
			s -= a[rj + k] * a[rj + k]
		var ljj := sqrt(s)
		a[rj + j] = ljj
		for i in range(j + 1, d):
			var ri := i * d
			var t := a[ri + j]
			for k in j:
				t -= a[ri + k] * a[rj + k]
			a[ri + j] = t / ljj
	var out: Array = []
	for c in 3:
		var y := PackedFloat64Array(rhs[c])
		for i in d:
			var t := y[i]
			for k in i:
				t -= a[i * d + k] * y[k]
			y[i] = t / a[i * d + i]
		for i in range(d - 1, -1, -1):
			var t := y[i]
			for k in range(i + 1, d):
				t -= a[k * d + i] * y[k]
			y[i] = t / a[i * d + i]
		out.append(y)
	return out

# Joints = the SMR bones plus all their ancestors, parents before children.
func build_chain(P: Dictionary, ubones: Array) -> Array:
	var set := {}
	for tid in ubones:
		var t: int = tid
		while t != 0 and P["tf"].has(t) and not set.has(t):
			set[t] = true
			t = P["tf"][t]["father"]
	var depth := {}
	for t in set:
		var dd := 0
		var c: int = t
		while P["tf"].has(c) and P["tf"][c]["father"] != 0 and P["tf"].has(P["tf"][c]["father"]):
			c = P["tf"][c]["father"]
			dd += 1
		depth[t] = dd
	var ids: Array = set.keys()
	ids.sort_custom(func(x, y): return depth[x] < depth[y] or (depth[x] == depth[y] and x < y))
	return ids

# Top-down projection of the world targets onto local TRS (mode trs / rt). Returns the realised worlds and locals.

func realize(P: Dictionary, chain: Array, bind_tid: PackedInt64Array, Nw: Array, used: PackedInt32Array, mode: String) -> Dictionary:
	var target := {}
	var changed := {}
	for j in used:
		target[bind_tid[j]] = Nw[j]
		changed[bind_tid[j]] = true
	var W := {}
	var local := {}
	var shear := 0.0
	var sdev := 0.0
	var nonuni := 0.0
	for tid in chain:
		var fa: int = P["tf"][tid]["father"]
		var bw: Transform3D = base_w.get(tid, world_of(P, tid))
		var t0: Dictionary = P["tf"][tid]
		if fa == 0 or not W.has(fa):
			W[tid] = bw
			local[tid] = {"pos": t0["pos"], "quat": t0["rot"], "scl": t0["scl"]}
			continue
		if not changed.has(tid):
			# not moved: keeps its own local transform, so it follows a moved parent
			var bl: Transform3D = (base_w.get(fa, world_of(P, fa)) as Transform3D).affine_inverse() * bw
			if base_w.is_empty():
				W[tid] = (W[fa] as Transform3D) * Transform3D(Basis(t0["rot"]) * Basis.from_scale(t0["scl"]), t0["pos"])
				local[tid] = {"pos": t0["pos"], "quat": t0["rot"], "scl": t0["scl"]}
			else:
				var R0 := polar(bl.basis)
				var S0 := R0.transposed() * bl.basis
				W[tid] = (W[fa] as Transform3D) * bl
				local[tid] = {"pos": bl.origin, "quat": quat_of(R0), "scl": Vector3(S0.x.x, S0.y.y, S0.z.z)}
			continue
		var L: Transform3D = (W[fa] as Transform3D).affine_inverse() * (target[tid] as Transform3D)
		var R := polar(L.basis)
		var S := R.transposed() * L.basis
		var scl := Vector3(S.x.x, S.y.y, S.z.z)
		var off := maxf(maxf(absf(S.x.y), absf(S.x.z)), maxf(absf(S.y.z), absf(S.y.x)))
		off = maxf(off, maxf(absf(S.z.x), absf(S.z.y)))
		shear = maxf(shear, off / maxf((scl.x + scl.y + scl.z) / 3.0, 1e-9))
		sdev = maxf(sdev, maxf(absf(scl.x - 1.0), maxf(absf(scl.y - 1.0), absf(scl.z - 1.0))))
		var mn := (scl.x + scl.y + scl.z) / 3.0
		nonuni = maxf(nonuni, maxf(absf(scl.x - mn), maxf(absf(scl.y - mn), absf(scl.z - mn))))
		if mode == "rt":
			scl = Vector3.ONE
		var lt := Transform3D(R * Basis.from_scale(scl), L.origin)
		W[tid] = (W[fa] as Transform3D) * lt
		local[tid] = {"pos": L.origin, "quat": quat_of(R), "scl": scl}
	return {"world": W, "local": local, "shear": shear, "scale_dev": sdev, "nonuni": nonuni}

func local_t(l: Dictionary) -> Transform3D:
	return Transform3D(Basis(l["quat"]) * Basis.from_scale(l["scl"]), l["pos"])

func write_local_table(path: String, P: Dictionary, chain: Array, rz: Dictionary) -> void:
	var f := FileAccess.open(path, FileAccess.WRITE)
	f.store_line("# joint | original local pos xyz | quat xyzw | scale | fitted local pos xyz | quat xyzw | scale   (Unity local space, left handed)")
	for tid in chain:
		var t: Dictionary = P["tf"][tid]
		var l: Dictionary = rz["local"][tid]
		var o: Quaternion = t["rot"]
		var q: Quaternion = l["quat"]
		f.store_line("%s | %.9f %.9f %.9f | %.9f %.9f %.9f %.9f | %.9f %.9f %.9f | %.9f %.9f %.9f | %.9f %.9f %.9f %.9f | %.9f %.9f %.9f" % [
			tf_name(P, tid), t["pos"].x, t["pos"].y, t["pos"].z, o.x, o.y, o.z, o.w, t["scl"].x, t["scl"].y, t["scl"].z,
			l["pos"].x, l["pos"].y, l["pos"].z, q.x, q.y, q.z, q.w, l["scl"].x, l["scl"].y, l["scl"].z])
	f.close()

# ------------------------------------------------------------------ OpenUSD out

func new_sb(elf: PackedByteArray):
	var sb = ClassDB.instantiate("Sandbox")
	sb.memory_max = 1024
	sb.references_max = 4096
	sb.allocations_max = 1000000
	sb.execution_timeout = 600000
	root.add_child(sb)
	sb.load_buffer(elf)
	sb.vmcall("usd_init")
	return sb

func pts_floats(p: PackedVector3Array) -> PackedFloat32Array:
	var o := PackedFloat32Array()
	o.resize(p.size() * 3)
	for i in p.size():
		o[i * 3] = p[i].x
		o[i * 3 + 1] = p[i].y
		o[i * 3 + 2] = p[i].z
	return o

func save_bytes(path: String, b: PackedByteArray) -> void:
	var f := FileAccess.open(path, FileAccess.WRITE)
	f.store_buffer(b)
	f.close()

func write_mesh_plain(sb, nm: String, p: PackedVector3Array, tri: PackedInt32Array) -> Variant:
	var counts := PackedInt32Array()
	counts.resize(tri.size() / 3)
	counts.fill(3)
	var ints := PackedInt32Array(counts)
	ints.append_array(tri)
	var header := "%s\nnp=%d\nnuv=0\nnrest=0\nnf=%d\nni=%d\nsource=bone_fit.gd" % [nm, p.size(), counts.size(), tri.size()]
	return sb.vmcall("usd_write_mesh", pts_floats(p), ints, header, "")

var clip_back := {}

func write_usd(out_dir: String, name: String, elf_path: String, P: Dictionary, chain: Array, bind_tid: PackedInt64Array, binds: Array, sgn: float, mi: MeshInstance3D, gv: PackedVector3Array, ob: Dictionary, orig: PackedVector3Array, target: PackedVector3Array, results: Dictionary, nb: int) -> void:
	if not FileAccess.file_exists(elf_path):
		say("usd.elf not found at %s: USD outputs skipped" % elf_path)
		fails += 1
		return
	var sb = new_sb(FileAccess.get_file_as_bytes(elf_path))
	var tri: PackedInt32Array = ob["tri"]
	# meshes
	var pkg = write_mesh_plain(sb, name + "_fitted_target", target, tri)
	check("fitted target mesh .usdz written", typeof(pkg) == TYPE_PACKED_BYTE_ARRAY and pkg.size() > 2 and pkg[0] == 0x50, str(pkg).left(200))
	if typeof(pkg) == TYPE_PACKED_BYTE_ARRAY:
		save_bytes(out_dir.path_join(name + "_fitted_target.usdz"), pkg)
	var pkg2 = write_mesh_plain(sb, name + "_skinned_newbones", results[primary_key]["pos"], tri)
	check("skinned-by-new-bones mesh .usdz written", typeof(pkg2) == TYPE_PACKED_BYTE_ARRAY and pkg2.size() > 2 and pkg2[0] == 0x50, str(pkg2).left(200))
	if typeof(pkg2) == TYPE_PACKED_BYTE_ARRAY:
		save_bytes(out_dir.path_join(name + "_skinned_newbones.usdz"), pkg2)
	# skeleton
	var J := chain.size()
	var jidx := {}
	var names := PackedStringArray()
	var parents := PackedInt32Array()
	for k in J:
		jidx[chain[k]] = k
	var joints_txt := PackedStringArray()
	var rests := PackedFloat32Array()
	var bind_world := {}
	# bind world of weighted bones: Unity bind frame R_j = C * Arm * bind_j^-1 * C ; others: prefab world
	var Cf := Transform3D(Basis.from_scale(Vector3(sgn, 1, 1)), Vector3.ZERO)
	var arm := scene_xf(mi)
	var rcount := 0
	for j in nb:
		bind_world[bind_tid[j]] = Cf * arm * (binds[j] as Transform3D).affine_inverse() * Cf
	for k in J:
		var tid: int = chain[k]
		var fa: int = P["tf"][tid]["father"]
		var pk: int = jidx[fa] if jidx.has(fa) else -1
		names.append(tf_name(P, tid))
		parents.append(pk)
		joints_txt.append("%s\t%d" % [names[k], pk])
		var t: Dictionary = P["tf"][tid]
		var lt := Transform3D(Basis(t["rot"]) * Basis.from_scale(t["scl"]), t["pos"])
		rests.append_array(floats12(lt))
		rests.append_array(floats12(bind_world.get(tid, world_of(P, tid))))
	# clips: frame 0 = rest (original locals), frame 1 = fitted
	for mode in clip_modes:
		var rz: Dictionary = results[mode]["rz"]
		var al := PackedFloat32Array()
		var aj := PackedInt32Array()
		for k in J:
			aj.append(k)
		for k in J:
			var t: Dictionary = P["tf"][chain[k]]
			al.append_array(floats12(Transform3D(Basis(t["rot"]) * Basis.from_scale(t["scl"]), t["pos"])))
		for k in J:
			al.append_array(floats12(local_t(rz["local"][chain[k]])))
		# frames x joints x 12: the loop above wrote frame 0 then frame 1 in joint order, which is the layout
		var txt = sb.vmcall("usd_write_skel_clip", "\n".join(names), parents, rests, aj, al, 1.0, "source=bone_fit.gd\nnote=frame 0 rest, frame 1 fitted (" + mode + "); Unity local space, left handed")
		var ok := typeof(txt) == TYPE_STRING and String(txt).begins_with("#usda")
		check("clip %s written (%d joints, 2 frames)" % [mode, J], ok, str(txt).left(200))
		if ok:
			var f := FileAccess.open(out_dir.path_join("%s_bones_clip_%s.usda" % [name, mode]), FileAccess.WRITE)
			f.store_string(txt)
			f.close()
			var op := str(sb.vmcall("usd_open", String(txt).to_utf8_buffer()))
			var got: PackedFloat32Array = sb.vmcall("usd_skel_anim_transforms", 0)
			var cd := 0.0
			var cw := 0
			for ii in mini(got.size(), al.size()):
				if absf(got[ii] - al[ii]) > cd:
					cd = absf(got[ii] - al[ii])
					cw = ii
			check("clip %s reads back: 2 frames x %d joints x 12 floats, max difference %.9f at frame %d joint %s component %d" % [mode, J, cd, cw / (J * 12), names[(cw / 12) % J], cw % 12], op.begins_with("ok ") and got.size() == al.size() and cd < 1e-3, op.left(100))
			clip_back[mode] = got
	# bind-pose skinned mesh, then OpenUSD's own LBS with original and with fitted locals
	var bind_pts := PackedVector3Array()
	bind_pts.resize(nv)
	for u in nv:
		var g := gv[gmap[u]]
		var wp: Vector3 = arm * g
		bind_pts[u] = Vector3(wp.x * sgn, wp.y, wp.z)
	var w4 := PackedFloat32Array()
	var i4 := PackedInt32Array()
	for u in nv:
		var g := gmap[u] * stride
		for s in stride:
			if true:
				w4.append(g_w[g + s])
				i4.append(jidx[bind_tid[g_bones[g + s]]] if g_w[g + s] > 0.0 else 0)
			else:
				w4.append(0.0)
				i4.append(0)
	var counts := PackedInt32Array()
	counts.resize(tri.size() / 3)
	counts.fill(3)
	var floats := pts_floats(bind_pts)
	floats.append_array(rests)
	floats.append_array(w4)
	var ints := PackedInt32Array(counts)
	ints.append_array(tri)
	ints.append_array(i4)
	var header := "%s_bind_skinned\nnp=%d\nnuv=0\nnrest=%d\nnf=%d\nni=%d\nsource=bone_fit.gd" % [name, nv, rests.size(), counts.size(), tri.size()]
	var sk = sb.vmcall("usd_write_mesh", floats, ints, header, "\n".join(joints_txt))
	var sk_ok: bool = typeof(sk) == TYPE_PACKED_BYTE_ARRAY and sk.size() > 2 and sk[0] == 0x50
	check("bind-pose skinned mesh .usdz written", sk_ok, str(sk).left(200))
	if not sk_ok:
		return
	save_bytes(out_dir.path_join(name + "_bind_skinned.usdz"), sk)
	var sb2 = new_sb(FileAccess.get_file_as_bytes(elf_path))
	var chunk := 4 * 1024 * 1024
	var at := 0
	while at < sk.size():
		sb2.vmcall("usd_push", sk.slice(at, mini(at + chunk, sk.size())))
		at += chunk
	var op2 := str(sb2.vmcall("usd_open_staged"))
	check("bind-pose skinned mesh opens in OpenUSD", op2.begins_with("ok "), op2.left(200))
	var rest_local := PackedFloat32Array()
	var fit_local := PackedFloat32Array()
	for k in J:
		var t: Dictionary = P["tf"][chain[k]]
		rest_local.append_array(floats12(Transform3D(Basis(t["rot"]) * Basis.from_scale(t["scl"]), t["pos"])))
		fit_local.append_array(floats12(local_t(results[primary_key]["rz"]["local"][chain[k]])))
	var usd_orig = sb2.vmcall("usd_mesh_skin_pose", 0, rest_local, false)
	var usd_fit = sb2.vmcall("usd_mesh_skin_pose", 0, fit_local, false)
	if typeof(usd_orig) != TYPE_PACKED_FLOAT32_ARRAY or typeof(usd_fit) != TYPE_PACKED_FLOAT32_ARRAY:
		check("OpenUSD LBS answered", false, str(usd_orig).left(150) + " | " + str(usd_fit).left(150))
		return
	var po := PackedVector3Array()
	var pf := PackedVector3Array()
	po.resize(nv)
	pf.resize(nv)
	for u in nv:
		po[u] = Vector3(usd_orig[u * 3], usd_orig[u * 3 + 1], usd_orig[u * 3 + 2])
		pf[u] = Vector3(usd_fit[u * 3], usd_fit[u * 3 + 1], usd_fit[u * 3 + 2])
	var e_o := err_stats(po, orig)
	var e_f := err_stats(pf, target)
	var e_f2 := err_stats(pf, results[primary_key]["pos"])
	say("OpenUSD's own skinning (usd_mesh_skin_pose), original locals vs the baked bin: %s" % fmt(e_o))
	say("OpenUSD's own skinning, fitted rigid locals vs target: %s" % fmt(e_f))
	say("OpenUSD's own skinning vs this script's skinning of the same TRS bones: %s" % fmt(e_f2))
	check("OpenUSD skinning with the fitted locals agrees with this script's (<1e-4 m)", e_f2["max"] < 1e-4, fmt(e_f2))
	if clip_back.has(primary_key):
		var back: PackedFloat32Array = clip_back[primary_key].slice(J * 12, 2 * J * 12)
		var uc = sb2.vmcall("usd_mesh_skin_pose", 0, back, false)
		var pc := PackedVector3Array()
		pc.resize(nv)
		for u in nv:
			pc[u] = Vector3(uc[u * 3], uc[u * 3 + 1], uc[u * 3 + 2])
		var e_c := err_stats(pc, target)
		say("END TO END: the delivered rigid clip read back from the .usda (frame 1), skinned by OpenUSD on the bind-pose mesh, vs target: %s  explained %.4f%%" % [fmt(e_c), 100.0 * (1.0 - sumsq(pc, target) / sumsq(target, orig))])
		check("end to end error within 1 mm of the script's own rigid result (mean)", absf(e_c["mean"] - results[primary_key]["e"]["mean"]) < 1e-3)

# summed |weight difference| per bone between two Godot vertices' influence sets
func wdiff(i: int, j: int) -> float:
	var d := {}
	for s in stride:
		if g_w[i * stride + s] > 0.0:
			d[g_bones[i * stride + s]] = d.get(g_bones[i * stride + s], 0.0) + g_w[i * stride + s]
		if g_w[j * stride + s] > 0.0:
			d[g_bones[j * stride + s]] = d.get(g_bones[j * stride + s], 0.0) - g_w[j * stride + s]
	var t := 0.0
	for k in d:
		t += absf(d[k])
	return t

var analysis_only := false
var shared_names := {}
var miroir_names := {}
var base_w := {}   # stage-1 analysis only: transform id -> re-seated world (empty = the prefab pose)

# Modular Avatar MergeArmature settings found in the outfit prefab (text scan)
func merge_info(prefab_path: String) -> PackedStringArray:
	var out := PackedStringArray()
	var ls := FileAccess.get_file_as_string(prefab_path).split("\n")
	for i in ls.size():
		if ls[i].strip_edges() == "mergeTarget:":
			var s := ""
			for k in range(i + 1, mini(i + 12, ls.size())):
				var t := ls[k].strip_edges()
				if t.begins_with("referencePath:") or t.begins_with("prefix:") or t.begins_with("suffix:") or t.begins_with("LockMode:") or t.begins_with("mangleNames:"):
					s += t + "; "
			out.append(s)
	return out

func obj_verts(p: String) -> PackedVector3Array:
	var o := PackedVector3Array()
	for l in FileAccess.get_file_as_string(p).split("\n"):
		if l.begins_with("v "):
			var q := l.split(" ", false)
			o.append(Vector3(float(q[1]), float(q[2]), float(q[3])))
	return o

# Movement of another piece's original vertices when the clothing bones go from world p0 to p1 (ids -> Transform3D).
func piece_movement(nd: MeshInstance3D, sk: Skeleton3D, uname: Dictionary, p0: Dictionary, p1: Dictionary, sgn: float, maxinf: int) -> Dictionary:
	var skin: Skin = nd.skin
	var tids := PackedInt64Array()
	for j in skin.get_bind_count():
		var bi := skin.get_bind_bone(j)
		tids.append(uname.get(sk.get_bone_name(bi), 0) if bi >= 0 else 0)
	var n := 0
	var sum := 0.0
	var mx := 0.0
	var vs := Vector3.ZERO
	var m: ArrayMesh = nd.mesh
	for s in m.get_surface_count():
		var ar := m.surface_get_arrays(s)
		var v: PackedVector3Array = ar[Mesh.ARRAY_VERTEX]
		var bb: PackedInt32Array = ar[Mesh.ARRAY_BONES]
		var ww: PackedFloat32Array = ar[Mesh.ARRAY_WEIGHTS]
		var st := bb.size() / v.size()
		for i in v.size():
			var order: Array = range(st)
			if maxinf > 0 and maxinf < st:
				order.sort_custom(func(x, y): return ww[i * st + x] > ww[i * st + y])
				order = order.slice(0, maxinf)
			var keep := 0.0
			for r in order:
				keep += ww[i * st + r]
			var d := Vector3.ZERO
			for r in order:
				var w := ww[i * st + r]
				var tid: int = tids[bb[i * st + r]]
				if w <= 0.0 or tid == 0:
					continue
				var q: Vector3 = skin.get_bind_pose(bb[i * st + r]) * v[i]
				q = Vector3(q.x * sgn, q.y, q.z)
				d += ((p1[tid] as Transform3D) * q - (p0[tid] as Transform3D) * q) * (w / keep)
			n += 1
			sum += d.length()
			mx = maxf(mx, d.length())
			vs += d
	return {"n": n, "mean": sum / maxi(n, 1), "max": mx, "vec": vs / maxi(n, 1)}

var ma_merged := {}   # outfit transform id -> Miroir transform id, for the bones MA's MergeArmature turns into Miroir bones
var piece_args: Array = []
var seg_g0 := PackedInt32Array()
var seg_u0 := PackedInt32Array()
var seg_binds: Array = []
var seg_names := PackedStringArray()
var seg_synth: Array = []
var g_own := PackedInt32Array()
var reseat_dir := ""
var u_weight := PackedFloat64Array()

func _find_child(Pf: Dictionary, parent: int, nm: String) -> int:
	for c in Pf["tf"][parent]["children"]:
		if Pf["tf"].has(c) and tf_name(Pf, c) == nm:
			return c
	return 0

# Simulation of nadena.dev.modular-avatar MergeArmature (Packages/nadena.dev.modular-avatar):
#  Editor/MergeArmatureHook.cs RecursiveMerge (lines 377-529): the outfit armature root is zip-merged into the merge target;
#   for every child of a zip-merged bone (lines 479-527), prefix/suffix stripped (491-495), the child is zip-merged only if
#   newParent.transform.Find(name) finds a DIRECT child of the matched Miroir bone (497, 519-523). Otherwise it is recursed
#   with zipMerge=false (526), and a non-zip bone does not recurse into its children's matching (475: only `if (zipMerge)`),
#   so a whole subtree below an unmatched bone is never merged, even if a name equals a Miroir bone's name.
#  A zip-merged bone stops being a separate bone: BoneDatabase.AddMergedBone (466) retargets skinning to the Miroir bone.
#  An unmatched bone is reparented (438: SetParent(..., true), world pose kept) under the matched Miroir bone's
#   intermediate object (399) and keeps its own transform.
#  Runtime/ModularAvatarMergeArmature.cs GetBonesMapping (197-228) + FindCorrespondingBone (104-113) use the same rule for the
#   edit-time lock (LockMode); ScanHierarchy recurses only into matched children (222-225), so only merged bones are locked.
func ma_simulate(P: Dictionary, M: Dictionary) -> Dictionary:
	var res := {"merged": {}, "found": false, "pb": PackedStringArray(), "extra": PackedStringArray()}
	var comp: Dictionary = {}
	for id in P["mono"]:
		if P["mono"][id]["guid"] == "2df373bf91cf30b4bbd495e11cb1a2ec":
			comp = P["mono"][id]
	if comp.is_empty():
		return res
	var src_tid := 0
	for tid in P["tf"]:
		if P["tf"][tid]["go"] == comp["go"]:
			src_tid = tid
	var cur := 0
	for tid in M["tf"]:
		if M["tf"][tid]["father"] == 0 or not M["tf"].has(M["tf"][tid]["father"]):
			cur = tid
	for part in String(comp["ref"]).split("/", false):
		cur = _find_child(M, cur, part)
	if src_tid == 0 or cur == 0:
		return res
	res["found"] = true
	_ma_rec(P, M, src_tid, cur, comp, res)
	for tid in res["merged"]:
		var go: int = P["tf"][tid]["go"]
		for cid in P["gocomp"].get(go, []):
			if P["tf"].has(cid):
				continue
			if P["mono"].has(cid) and P["mono"][cid]["guid"] == "2df373bf91cf30b4bbd495e11cb1a2ec":
				continue
			if P["mono"].has(cid) and P["mono"][cid]["fid"] == 1661641543:
				res["pb"].append(tf_name(P, tid))
			else:
				res["extra"].append(tf_name(P, tid))
	return res

func _ma_rec(P: Dictionary, M: Dictionary, src: int, newp: int, comp: Dictionary, res: Dictionary) -> void:
	res["merged"][src] = newp
	var pre: String = comp["prefix"]
	var suf: String = comp["suffix"]
	for child in P["tf"][src]["children"]:
		if not P["tf"].has(child):
			continue
		var has_ma := false
		for cid in P["gocomp"].get(P["tf"][child]["go"], []):
			if P["mono"].has(cid) and P["mono"][cid]["guid"] == "2df373bf91cf30b4bbd495e11cb1a2ec":
				has_ma = true
		if has_ma:
			continue
		var nm := tf_name(P, child)
		if nm.begins_with(pre) and nm.ends_with(suf) and nm.length() > pre.length() + suf.length():
			var f := _find_child(M, newp, nm.substr(pre.length(), nm.length() - pre.length() - suf.length()))
			if f != 0:
				_ma_rec(P, M, child, f, comp, res)

# Stage-1 style re-seat worlds (translation of the paired joints, carried to descendants): used for the synthetic piece target
func reseat_worlds(P: Dictionary, chain: Array, ed: String) -> Dictionary:
	var jn := FileAccess.get_file_as_string(ed.path_join("joints.txt")).split("\n", false)
	var sv := obj_verts(ed.path_join("source_skeleton.obj"))
	var tv := obj_verts(ed.path_join("target_skeleton.obj"))
	var off := {}
	for i in jn.size():
		off[jn[i].strip_edges()] = tv[i] - sv[i]
	var out := {}
	for tid in chain:
		var c: int = tid
		var o := Vector3.ZERO
		while c != 0 and P["tf"].has(c):
			if off.has(tf_name(P, c)):
				o = off[tf_name(P, c)]
				break
			c = P["tf"][c]["father"]
		var w0 := world_of(P, tid)
		out[tid] = Transform3D(w0.basis, w0.origin + o)
	return out

func seg_report(label: String, pos: PackedVector3Array, target: PackedVector3Array, orig: PackedVector3Array) -> void:
	if seg_names.size() < 2:
		return
	for k in seg_names.size():
		var a := seg_u0[k]
		var b := seg_u0[k + 1]
		var ps := pos.slice(a, b)
		var ts := target.slice(a, b)
		var os := orig.slice(a, b)
		var den := sumsq(ts, os)
		var expl := ("%.4f%%" % (100.0 * (1.0 - sumsq(ps, ts) / den))) if den > 1e-9 else "n/a (target equals original)"
		say("   [%s] piece %s: error vs its target %s  explained %s (unfitted movement mean %.6f)" % [label, seg_names[k], fmt(err_stats(ps, ts)), expl, err_stats(os, ts)["mean"]])

var primary_key := "rt"            # which result is checked, delivered as the main clip and used for the effect on the other pieces
var clip_modes: Array = ["trs", "rt"]
