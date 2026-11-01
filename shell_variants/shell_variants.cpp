// shell_variants -- garment fixtures for the cloth-fit start-check experiments.
//
//   shell_variants <garment.obj> <level> <gap_in_dhat> <dhat> <out.obj> [defect]
//   shell_variants --selftest <garment.obj>
//
// Levels, all built from one garment G so the avatar frame stays valid:
//   L0  G alone                                      (the solver's known-good control)
//   L1  G + a copy shifted along G's longest axis by gap * dhat
//   L2  G + a copy offset inward along vertex normals by gap * dhat (a nested shell)
//   L3  L2 + a strip joining every open edge of G to its copy (a closed double layer)
//   L4  G + a copy offset inward on one half and outward on the other (a crossing)
// defect, when given, plants the failure for the level: L1 gap 0 (coincident), L2 gap 0,
// L3 one strip removed, L4 none (it is already the defect).
// gap is a multiple of dhat; dhat is passed in from the solver's setup.json, never typed here.
// SPDX-License-Identifier: MIT
#include <algorithm>
#include <array>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <map>
#include <set>
#include <sstream>
#include <string>
#include <vector>

using V3 = std::array<double, 3>;
using Tri = std::array<int, 3>;
static V3 sub(V3 a, V3 b) { return { a[0] - b[0], a[1] - b[1], a[2] - b[2] }; }
static V3 add(V3 a, V3 b) { return { a[0] + b[0], a[1] + b[1], a[2] + b[2] }; }
static V3 mul(V3 a, double s) { return { a[0] * s, a[1] * s, a[2] * s }; }
static double dot(V3 a, V3 b) { return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]; }
static V3 cross(V3 a, V3 b) { return { a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0] }; }

struct Mesh { std::vector<V3> v; std::vector<Tri> f; };

static bool read_obj(const std::string &p, Mesh &m) {
	std::ifstream in(p);
	if (!in) return false;
	std::string line;
	while (std::getline(in, line)) {
		std::istringstream s(line);
		std::string t;
		s >> t;
		if (t == "v") { V3 q; s >> q[0] >> q[1] >> q[2]; m.v.push_back(q); }
		else if (t == "f") {
			std::vector<int> idx;
			std::string tok;
			while (s >> tok) idx.push_back(std::atoi(tok.c_str()) - 1);
			for (size_t i = 2; i < idx.size(); i++) m.f.push_back({ idx[0], idx[i - 1], idx[i] });
		}
	}
	return !m.v.empty();
}

static bool write_obj(const std::string &p, const Mesh &m) {
	std::ofstream o(p);
	if (!o) return false;
	o.precision(17);
	for (auto &q : m.v) o << "v " << q[0] << " " << q[1] << " " << q[2] << "\n";
	for (auto &t : m.f) o << "f " << t[0] + 1 << " " << t[1] + 1 << " " << t[2] + 1 << "\n";
	return true;
}

static std::vector<V3> vertex_normals(const Mesh &m) {
	std::vector<V3> n(m.v.size(), V3{ 0, 0, 0 });
	for (auto &t : m.f) {
		V3 fn = cross(sub(m.v[t[1]], m.v[t[0]]), sub(m.v[t[2]], m.v[t[0]])); // area weighted
		for (int k = 0; k < 3; k++) n[t[k]] = add(n[t[k]], fn);
	}
	for (auto &q : n) { double l = std::sqrt(dot(q, q)); if (l > 0) q = mul(q, 1.0 / l); }
	return n;
}

// +1 when the vertex normals mostly point away from the mesh centroid, else -1.
static double outward_sign(const Mesh &m, const std::vector<V3> &n) {
	V3 c{ 0, 0, 0 };
	for (auto &q : m.v) c = add(c, q);
	c = mul(c, 1.0 / m.v.size());
	double s = 0;
	for (size_t i = 0; i < m.v.size(); i++) s += dot(n[i], sub(m.v[i], c));
	return s > 0 ? 1.0 : -1.0;
}

static int longest_axis(const Mesh &m) {
	V3 lo = m.v[0], hi = m.v[0];
	for (auto &q : m.v) for (int k = 0; k < 3; k++) { lo[k] = std::min(lo[k], q[k]); hi[k] = std::max(hi[k], q[k]); }
	int a = 0;
	for (int k = 1; k < 3; k++) if (hi[k] - lo[k] > hi[a] - lo[a]) a = k;
	return a;
}

static std::vector<std::pair<int, int>> open_edges(const Mesh &m) {
	std::map<std::pair<int, int>, int> cnt;
	for (auto &t : m.f) for (int k = 0; k < 3; k++) {
		int a = t[k], b = t[(k + 1) % 3];
		cnt[{ std::min(a, b), std::max(a, b) }]++;
	}
	std::vector<std::pair<int, int>> out;
	for (auto &[e, c] : cnt) if (c == 1) out.push_back(e);
	return out;
}

// Exact-enough crossing count: edge against every triangle sharing no vertex (Moller-Trumbore on the edge segment).
static long crossings(const Mesh &m) {
	std::set<std::pair<int, int>> es;
	for (auto &t : m.f) for (int k = 0; k < 3; k++) {
		int a = t[k], b = t[(k + 1) % 3];
		es.insert({ std::min(a, b), std::max(a, b) });
	}
	long n = 0;
	for (auto &[a, b] : es) {
		V3 p = m.v[a], d = sub(m.v[b], p);
		for (auto &t : m.f) {
			if (t[0] == a || t[1] == a || t[2] == a || t[0] == b || t[1] == b || t[2] == b) continue;
			V3 e1 = sub(m.v[t[1]], m.v[t[0]]), e2 = sub(m.v[t[2]], m.v[t[0]]);
			V3 h = cross(d, e2);
			double det = dot(e1, h);
			if (det == 0) continue;
			V3 s = sub(p, m.v[t[0]]);
			double u = dot(s, h) / det;
			if (u < 0 || u > 1) continue;
			V3 q = cross(s, e1);
			double v = dot(d, q) / det;
			if (v < 0 || u + v > 1) continue;
			double w = dot(e2, q) / det;
			if (w >= 0 && w <= 1) n++;
		}
	}
	return n;
}

static Mesh build(const Mesh &g, const std::string &level, double gap, double dhat, bool defect) {
	Mesh m = g;
	if (level == "L0") return m;
	auto n = vertex_normals(g);
	double sg = outward_sign(g, n);
	double d = (defect ? 0.0 : gap) * dhat;
	Mesh c = g;
	if (level == "L1") {
		int ax = longest_axis(g);
		V3 lo = g.v[0], hi = g.v[0];
		for (auto &q : g.v) for (int k = 0; k < 3; k++) { lo[k] = std::min(lo[k], q[k]); hi[k] = std::max(hi[k], q[k]); }
		double shift = (hi[ax] - lo[ax]) + d; // clear of G by d above its extent
		if (defect) shift = 0.5 * (hi[ax] - lo[ax]); // overlapping copy: they cross
		for (auto &q : c.v) q[ax] += shift;
	} else if (level == "L2" || level == "L3") {
		for (size_t i = 0; i < c.v.size(); i++) c.v[i] = add(g.v[i], mul(n[i], -sg * d));
	} else if (level == "L4") {
		V3 mid{ 0, 0, 0 };
		for (auto &q : g.v) mid = add(mid, q);
		mid = mul(mid, 1.0 / g.v.size());
		for (size_t i = 0; i < c.v.size(); i++) {
			double side = g.v[i][0] > mid[0] ? 1.0 : -1.0;
			c.v[i] = add(g.v[i], mul(n[i], -sg * side * gap * dhat));
		}
	}
	int off = (int)m.v.size();
	for (auto &q : c.v) m.v.push_back(q);
	for (auto t : g.f) m.f.push_back({ t[0] + off, t[2] + off, t[1] + off }); // flipped: normals face into the pocket
	if (level == "L3") {
		auto oe = open_edges(g);
		for (size_t i = 0; i < oe.size(); i++) {
			if (defect && i == 0) continue; // one strip missing: the pocket stays open there
			auto [a, b] = oe[i];
			m.f.push_back({ a, b, b + off });
			m.f.push_back({ a, b + off, a + off });
		}
	}
	return m;
}

int main(int argc, char **argv) {
	if (argc >= 3 && std::string(argv[1]) == "--selftest") {
		Mesh g;
		if (!read_obj(argv[2], g)) { std::fprintf(stderr, "cannot read %s\n", argv[2]); return 2; }
		long base = crossings(g);
		long l2 = crossings(build(g, "L2", 4.0, 0.002, false));
		long l4 = crossings(build(g, "L4", 4.0, 0.002, false));
		std::printf("crossings: G %ld, L2(gap 4 dhat) %ld, L4(planted) %ld\n", base, l2, l4);
		bool ok = l2 == base && l4 > base; // a nested copy adds none; the planted crossing must add some
		std::printf("selftest %s\n", ok ? "PASS" : "FAIL");
		return ok ? 0 : 1;
	}
	if (argc >= 3 && std::string(argv[1]) == "--count") {
		Mesh g;
		if (!read_obj(argv[2], g)) return 2;
		std::printf("%ld\n", crossings(g));
		return 0;
	}
	if (argc < 6) { std::fprintf(stderr, "usage: shell_variants garment.obj level gap_in_dhat dhat out.obj [defect]\n"); return 2; }
	Mesh g;
	if (!read_obj(argv[1], g)) { std::fprintf(stderr, "cannot read %s\n", argv[1]); return 2; }
	Mesh m = build(g, argv[2], std::atof(argv[3]), std::atof(argv[4]), argc > 6);
	if (!write_obj(argv[5], m)) { std::fprintf(stderr, "cannot write %s\n", argv[5]); return 2; }
	return 0;
}
