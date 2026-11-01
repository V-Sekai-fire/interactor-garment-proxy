// measure_clip -- how far a body pokes through a garment, in metres.
//
//   measure_clip <body.obj> <garment.obj> [scale [weights.txt rows]]
//
// weights.txt is cloth-fit's skin weights: one row per joint, one column per body vertex. rows is a
// comma-separated list of 1-based joint rows. With both given, only body vertices whose dominant joint is
// in that list are measured, so arms and hands beside a skirt are not read as clipping.
//
// For every body vertex it finds the closest point on the garment (brute force, exact). A vertex counts as
// covered when that closest feature is interior to the cloth: inside a triangle, on an edge two triangles
// share, or on a vertex none of whose edges is a border. Skin below a hem or at a neck opening is therefore
// not counted. For a covered vertex the signed distance s = (p - c) . n uses the normal of the triangle that
// holds the closest point. Which side is "outside" depends on the winding the exporter wrote, so the tool
// does not assume it: the side holding fewer covered vertices is the one the body pokes through (a body
// sits under its dress, so most covered vertices are on the inner side). It prints both counts so a reader
// can see when that does not hold.
//
// The optional scale multiplies the garment about its bounding-box centre in x and z. It is the planted
// control: scale < 1 pulls the cloth into the body and the clipping count must rise, scale > 1 and it must
// fall. There is no distance threshold anywhere: a vertex is clipping when its signed distance is on the
// outside, however small.
//
// Output, one TSV row: name covered inner outer clipping max_mm mean_mm
// SPDX-License-Identifier: MIT
#include <algorithm>
#include <array>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <set>
#include <map>
#include <sstream>
#include <string>
#include <vector>

using V3 = std::array<double, 3>;
static V3 sub(V3 a, V3 b) { return { a[0] - b[0], a[1] - b[1], a[2] - b[2] }; }
static V3 add(V3 a, V3 b) { return { a[0] + b[0], a[1] + b[1], a[2] + b[2] }; }
static V3 mul(V3 a, double s) { return { a[0] * s, a[1] * s, a[2] * s }; }
static double dot(V3 a, V3 b) { return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]; }
static V3 cross(V3 a, V3 b) { return { a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0] }; }

struct Obj { std::vector<V3> v; std::vector<std::array<int, 3>> f; };

static bool read_obj(const char *path, Obj &o) {
	std::ifstream in(path);
	if (!in) return false;
	std::string line;
	while (std::getline(in, line)) {
		std::istringstream s(line);
		std::string tag;
		s >> tag;
		if (tag == "v") { V3 p; s >> p[0] >> p[1] >> p[2]; o.v.push_back(p); }
		else if (tag == "f") {
			std::vector<int> idx;
			std::string tok;
			while (s >> tok) idx.push_back(std::atoi(tok.c_str()) - 1);
			for (size_t i = 2; i < idx.size(); i++) o.f.push_back({ idx[0], idx[i - 1], idx[i] });
		}
	}
	return true;
}

// Closest point on triangle abc to p (Ericson, Real-Time Collision Detection 5.1.5). region: 0 face,
// 1 edge ab, 2 edge bc, 3 edge ca, 4 vertex a, 5 vertex b, 6 vertex c.
static V3 closest(V3 p, V3 a, V3 b, V3 c, int &region) {
	V3 ab = sub(b, a), ac = sub(c, a), ap = sub(p, a);
	double d1 = dot(ab, ap), d2 = dot(ac, ap);
	if (d1 <= 0 && d2 <= 0) { region = 4; return a; }
	V3 bp = sub(p, b);
	double d3 = dot(ab, bp), d4 = dot(ac, bp);
	if (d3 >= 0 && d4 <= d3) { region = 5; return b; }
	double vc = d1 * d4 - d3 * d2;
	if (vc <= 0 && d1 >= 0 && d3 <= 0) { region = 1; return add(a, mul(ab, d1 / (d1 - d3))); }
	V3 cp = sub(p, c);
	double d5 = dot(ab, cp), d6 = dot(ac, cp);
	if (d6 >= 0 && d5 <= d6) { region = 6; return c; }
	double vb = d5 * d2 - d1 * d6;
	if (vb <= 0 && d2 >= 0 && d6 <= 0) { region = 3; return add(a, mul(ac, d2 / (d2 - d6))); }
	double va = d3 * d6 - d5 * d4;
	if (va <= 0 && (d4 - d3) >= 0 && (d5 - d6) >= 0) {
		region = 2;
		return add(b, mul(sub(c, b), (d4 - d3) / ((d4 - d3) + (d5 - d6))));
	}
	double denom = 1.0 / (va + vb + vc);
	region = 0;
	return add(a, add(mul(ab, vb * denom), mul(ac, vc * denom)));
}

int main(int argc, char **argv) {
	if (argc < 3) { std::fprintf(stderr, "usage: measure_clip body.obj garment.obj [scale]\n"); return 2; }
	Obj body, gar;
	if (!read_obj(argv[1], body) || !read_obj(argv[2], gar)) { std::fprintf(stderr, "cannot read input\n"); return 2; }
	double scale = argc > 3 ? std::atof(argv[3]) : 1.0;
	if (scale != 1.0) {
		V3 lo = gar.v[0], hi = gar.v[0];
		for (auto &p : gar.v) for (int k = 0; k < 3; k++) { lo[k] = std::min(lo[k], p[k]); hi[k] = std::max(hi[k], p[k]); }
		double cx = 0.5 * (lo[0] + hi[0]), cz = 0.5 * (lo[2] + hi[2]);
		for (auto &p : gar.v) { p[0] = cx + (p[0] - cx) * scale; p[2] = cz + (p[2] - cz) * scale; }
	}
	// Weld garment vertices by exact position, so a UV seam does not read as a border.
	std::map<V3, int> weld;
	std::vector<int> wid(gar.v.size());
	for (size_t i = 0; i < gar.v.size(); i++) wid[i] = weld.emplace(gar.v[i], (int)weld.size()).first->second;
	std::map<std::pair<int, int>, int> edge_count;
	auto ekey = [&](int a, int b) { return std::make_pair(std::min(wid[a], wid[b]), std::max(wid[a], wid[b])); };
	for (auto &t : gar.f) for (int k = 0; k < 3; k++) edge_count[ekey(t[k], t[(k + 1) % 3])]++;
	std::vector<char> border_vertex(weld.size(), 0);
	for (auto &t : gar.f) for (int k = 0; k < 3; k++)
		if (edge_count[ekey(t[k], t[(k + 1) % 3])] == 1) { border_vertex[wid[t[k]]] = 1; border_vertex[wid[t[(k + 1) % 3]]] = 1; }

	std::vector<V3> nrm(gar.f.size());
	for (size_t i = 0; i < gar.f.size(); i++) {
		auto &t = gar.f[i];
		V3 n = cross(sub(gar.v[t[1]], gar.v[t[0]]), sub(gar.v[t[2]], gar.v[t[0]]));
		double l = std::sqrt(dot(n, n));
		nrm[i] = l > 0 ? mul(n, 1.0 / l) : V3{ 0, 0, 0 };
	}

	V3 glo = gar.v[0], ghi = gar.v[0];
	for (auto &p : gar.v) for (int k = 0; k < 3; k++) { glo[k] = std::min(glo[k], p[k]); ghi[k] = std::max(ghi[k], p[k]); }
	double axx = 0.5 * (glo[0] + ghi[0]), axz = 0.5 * (glo[2] + ghi[2]), away = 0.0;
	for (size_t i = 0; i < gar.f.size(); i++) {
		auto &t = gar.f[i];
		V3 c = mul(add(gar.v[t[0]], add(gar.v[t[1]], gar.v[t[2]])), 1.0 / 3.0);
		away += dot(nrm[i], { c[0] - axx, 0.0, c[2] - axz });
	}
	bool normals_out = away > 0;
	std::vector<char> keep(body.v.size(), 1);
	if (argc > 5) {
		std::ifstream win(argv[4]);
		std::set<int> rows;
		std::stringstream rs(argv[5]);
		std::string r;
		while (std::getline(rs, r, ',')) rows.insert(std::atoi(r.c_str()) - 1);
		std::vector<double> bestw(body.v.size(), -1.0);
		std::vector<int> bestj(body.v.size(), -1);
		std::string line;
		for (int j = 0; std::getline(win, line); j++) {
			std::istringstream ls(line);
			double w;
			for (size_t i = 0; i < body.v.size() && (ls >> w); i++)
				if (w > bestw[i]) { bestw[i] = w; bestj[i] = j; }
		}
		for (size_t i = 0; i < body.v.size(); i++) keep[i] = rows.count(bestj[i]) ? 1 : 0;
	}
	std::vector<double> pos, neg; // signed distances of covered vertices
	for (size_t vi = 0; vi < body.v.size(); vi++) {
		if (!keep[vi]) continue;
		auto &p = body.v[vi];
		double best = 1e300; int bt = -1, breg = 0; V3 bc{};
		for (size_t i = 0; i < gar.f.size(); i++) {
			auto &t = gar.f[i];
			int reg;
			V3 c = closest(p, gar.v[t[0]], gar.v[t[1]], gar.v[t[2]], reg);
			V3 d = sub(p, c);
			double d2 = dot(d, d);
			if (d2 < best) { best = d2; bt = (int)i; breg = reg; bc = c; }
		}
		if (bt < 0) continue;
		auto &t = gar.f[bt];
		bool border;
		if (breg == 0) border = false;
		else if (breg <= 3) { int k = breg - 1; border = edge_count[ekey(t[k], t[(k + 1) % 3])] == 1; }
		else border = border_vertex[wid[t[breg - 4]]] != 0;
		if (border) continue;
		double s = dot(sub(p, bc), nrm[bt]);
		(s > 0 ? pos : neg).push_back(std::fabs(s));
	}
	bool outer_is_pos = normals_out; // a positive signed distance is outside when the normals point away from the axis
	auto &out = outer_is_pos ? pos : neg;
	double mx = 0, sum = 0;
	for (double d : out) { mx = std::max(mx, d); sum += d; }
	std::fprintf(stderr, "normals %s the garment axis\n", normals_out ? "point away from" : "point toward");
	std::printf("%s\t%zu\t%zu\t%zu\t%zu\t%.3f\t%.3f\n", argv[2], pos.size() + neg.size(),
			(outer_is_pos ? neg : pos).size(), out.size(), out.size(), mx * 1000.0,
			out.empty() ? 0.0 : sum / out.size() * 1000.0);
	return 0;
}
