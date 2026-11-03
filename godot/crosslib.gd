extends RefCounted
# Exact-sign segment/triangle crossing test (no epsilon): the segment p-q crosses triangle a-b-c when p and q lie strictly
# on opposite sides of the triangle plane and the three edge orientations agree. Touching (a zero orientation) is not a crossing.


static func orient(a: Vector3, b: Vector3, c: Vector3, d: Vector3) -> float:
	return (b - a).cross(c - a).dot(d - a)


static func seg_tri(p: Vector3, q: Vector3, a: Vector3, b: Vector3, c: Vector3) -> bool:
	var s1 := orient(a, b, c, p)
	var s2 := orient(a, b, c, q)
	if not ((s1 > 0.0 and s2 < 0.0) or (s1 < 0.0 and s2 > 0.0)):
		return false
	var o1 := orient(p, q, a, b)
	var o2 := orient(p, q, b, c)
	var o3 := orient(p, q, c, a)
	return (o1 > 0.0 and o2 > 0.0 and o3 > 0.0) or (o1 < 0.0 and o2 < 0.0 and o3 < 0.0)
