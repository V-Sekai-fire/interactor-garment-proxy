extends SceneTree
# Random per-vertex offset (uniform in a cube of half-width amp) with a fixed seed, to break the exact coplanarity of
# voxel-lattice vertices that IPC's floating-point edge/triangle test (LU solve on coplanar input) reports as intersections.
# usage: -- <in.obj> <out.obj> <amp> <seed>


func _init() -> void:
	var a := OS.get_cmdline_user_args()
	var rng := RandomNumberGenerator.new()
	rng.seed = int(a[3])
	var amp := float(a[2])
	var o := FileAccess.open(a[1], FileAccess.WRITE)
	for l in FileAccess.get_file_as_string(a[0]).split("\n"):
		if l.begins_with("v "):
			var t := l.split(" ", false)
			o.store_line("v %.9f %.9f %.9f" % [float(t[1]) + rng.randf_range(-amp, amp), float(t[2]) + rng.randf_range(-amp, amp), float(t[3]) + rng.randf_range(-amp, amp)])
		elif l.strip_edges() != "":
			o.store_line(l)
	quit(0)
