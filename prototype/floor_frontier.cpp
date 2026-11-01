// Frontier over the curvenet extractor's edge floor (CassieCurvenetExtractor pass 2).
//
// The extractor drops an edge whose score (dihedral, radians; curvature_weight 0) is under a floor. The
// floor was 15 degrees, a fixed number. Candidates here state it as a quantity instead, or drop it:
//   old15      pi * 15/180                              (what shipped)
//   none       0                                        (every edge with a non-zero score is kept)
//   rdp_1      atan2(rdp_error, edge_length)            (the tilt rdp_error gives one endpoint)
//   rdp_2      atan2(2 rdp_error, edge_length)          (both endpoints, opposite signs)
//   rdp_4      atan2(4 rdp_error, edge_length)          (the four corners of the two triangles that
//                                                         share the edge, each moved by rdp_error)
//   all        pi                                       (planted: keeps nothing; must fail on any fold)
// A level is a surface family with fixed attributes. A problem is one instance of it (edge length h,
// ripple amplitude e = rdp_error, seed). Ripple moves each vertex along the surface normal by a uniform
// draw in [-e, e], i.e. within the stated error. A problem succeeds when the kept interior edges are
// exactly the ground-truth ridge edges (none for the ripple-only levels; the grid edges on the fold line
// for the fold levels). Regret per level is 1[s >= 1] - s / n.
//
// Folds are only drawn where the answer is not a matter of taste: a fold is a ridge when its deviation
// from the flat plane across one edge, h tan(theta), exceeds 2 e (both endpoints at worst), and a
// non-ridge when it is under e / 2. Draws in between are not problems and are skipped, and the count
// skipped is printed.
//
//   clang++ -std=c++17 -O2 floor_frontier.cpp -o floor_frontier && ./floor_frontier
#include <algorithm>
#include <array>
#include <cmath>
#include <cstdio>
#include <map>
#include <random>
#include <set>
#include <string>
#include <vector>

typedef std::array<double, 3> V3;
static V3 sub(V3 a, V3 b) { return { a[0] - b[0], a[1] - b[1], a[2] - b[2] }; }
static V3 cross(V3 a, V3 b) { return { a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0] }; }
static double dot(V3 a, V3 b) { return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]; }
static double len(V3 a) { return std::sqrt(dot(a, a)); }
typedef std::array<int, 3> Tri;
struct Mesh {
	std::vector<V3> p;
	std::vector<Tri> t;
	std::set<std::pair<int, int>> truth; // ridge edges (empty for the ripple-only levels)
};
static std::pair<int, int> key(int a, int b) { return { std::min(a, b), std::max(a, b) }; }

// A (2n x 2n) grid of cells, spacing h, centred on x = 0 so that x = 0 is a grid line. The surface is
// z = 0 for x <= 0 and z = x tan(theta) for x > 0 (a fold of angle theta along x = 0), then ripple
// along each vertex's own surface normal. Grid cells are cut along one diagonal.
static Mesh fold_grid(int n, double h, double theta, double e, unsigned seed) {
	Mesh m;
	std::mt19937 rng(seed);
	std::uniform_real_distribution<double> u(-1.0, 1.0);
	const int W = 2 * n + 1;
	for (int j = 0; j < W; ++j)
		for (int i = 0; i < W; ++i) {
			double x = (i - n) * h, y = (j - n) * h;
			V3 nrm = { 0, 0, 1 };
			double z = 0;
			if (x > 0) { z = x * std::tan(theta); nrm = { -std::sin(theta), 0, std::cos(theta) }; }
			double r = e * u(rng);
			double xx = x, zz = z;
			xx += r * nrm[0]; zz += r * nrm[2];
			m.p.push_back({ xx, y, zz });
		}
	for (int j = 0; j < W - 1; ++j)
		for (int i = 0; i < W - 1; ++i) {
			int a = j * W + i, b = j * W + i + 1, c = (j + 1) * W + i, d = (j + 1) * W + i + 1;
			m.t.push_back({ a, b, d });
			m.t.push_back({ a, d, c });
		}
	if (theta != 0)
		for (int j = 0; j < W - 1; ++j) m.truth.insert(key(j * W + n, (j + 1) * W + n));
	return m;
}
// An icosphere of radius R (1 subdivision level chosen to give edge length near h), ripple along the
// radial normal. No ridge anywhere.
static Mesh sphere(double R, double h, double e, unsigned seed) {
	Mesh m;
	double g = (1 + std::sqrt(5.0)) / 2;
	double c[12][3] = { { -1, g, 0 }, { 1, g, 0 }, { -1, -g, 0 }, { 1, -g, 0 }, { 0, -1, g }, { 0, 1, g }, { 0, -1, -g }, { 0, 1, -g }, { g, 0, -1 }, { g, 0, 1 }, { -g, 0, -1 }, { -g, 0, 1 } };
	for (auto &v : c) { V3 q = { v[0], v[1], v[2] }; double l = len(q); m.p.push_back({ q[0] / l, q[1] / l, q[2] / l }); }
	int f[20][3] = { { 0, 11, 5 }, { 0, 5, 1 }, { 0, 1, 7 }, { 0, 7, 10 }, { 0, 10, 11 }, { 1, 5, 9 }, { 5, 11, 4 }, { 11, 10, 2 }, { 10, 7, 6 }, { 7, 1, 8 }, { 3, 9, 4 }, { 3, 4, 2 }, { 3, 2, 6 }, { 3, 6, 8 }, { 3, 8, 9 }, { 4, 9, 5 }, { 2, 4, 11 }, { 6, 2, 10 }, { 8, 6, 7 }, { 9, 8, 1 } };
	for (auto &x : f) m.t.push_back({ x[0], x[1], x[2] });
	double edge = R * 1.0515; // icosahedron edge for unit circumradius is 1.0515
	while (edge > h) {
		std::map<std::pair<int, int>, int> mid;
		auto mp = [&](int a, int b) {
			auto k = key(a, b);
			auto it = mid.find(k);
			if (it != mid.end()) return it->second;
			V3 q = { (m.p[a][0] + m.p[b][0]) / 2, (m.p[a][1] + m.p[b][1]) / 2, (m.p[a][2] + m.p[b][2]) / 2 };
			double l = len(q);
			m.p.push_back({ q[0] / l, q[1] / l, q[2] / l });
			return mid[k] = int(m.p.size()) - 1;
		};
		std::vector<Tri> nt;
		for (auto &tr : m.t) {
			int a = tr[0], b = tr[1], c2 = tr[2], ab = mp(a, b), bc = mp(b, c2), ca = mp(c2, a);
			nt.push_back({ a, ab, ca }); nt.push_back({ b, bc, ab }); nt.push_back({ c2, ca, bc }); nt.push_back({ ab, bc, ca });
		}
		m.t = nt;
		edge /= 2;
	}
	std::mt19937 rng(seed);
	std::uniform_real_distribution<double> u(-1.0, 1.0);
	for (auto &q : m.p) { double r = R + e * u(rng); for (auto &x : q) x *= r; }
	return m;
}

struct Floors { const char *name; double (*floor_)(double e, double edge); };
static double f_old15(double, double) { return M_PI * 15.0 / 180.0; }
static double f_none(double, double) { return 0.0; }
static double f_rdp1(double e, double l) { return std::atan2(e, l); }
static double f_rdp2(double e, double l) { return std::atan2(2 * e, l); }
static double f_rdp4(double e, double l) { return std::atan2(4 * e, l); }
static double f_all(double, double) { return M_PI; }
static const Floors kFloors[] = { { "old15", f_old15 }, { "none", f_none }, { "rdp_1", f_rdp1 }, { "rdp_2", f_rdp2 }, { "rdp_4", f_rdp4 }, { "all(planted)", f_all } };

// Kept interior edges for a floor, as the extractor scores them (dihedral only).
static std::set<std::pair<int, int>> kept(const Mesh &m, double e, double (*fl)(double, double)) {
	std::map<std::pair<int, int>, std::vector<int>> et;
	for (size_t i = 0; i < m.t.size(); ++i)
		for (int k = 0; k < 3; ++k) et[key(m.t[i][k], m.t[i][(k + 1) % 3])].push_back(int(i));
	auto fn = [&](int i) { V3 a = m.p[m.t[i][0]], b = m.p[m.t[i][1]], c = m.p[m.t[i][2]]; V3 n = cross(sub(b, a), sub(c, a)); double l = len(n); return V3{ n[0] / l, n[1] / l, n[2] / l }; };
	std::set<std::pair<int, int>> out;
	for (auto &kv : et) {
		if (kv.second.size() != 2) continue; // boundary: always kept by the extractor, identical for every floor
		double score = std::acos(std::max(-1.0, std::min(1.0, dot(fn(kv.second[0]), fn(kv.second[1])))));
		double l = len(sub(m.p[kv.first.first], m.p[kv.first.second]));
		if (score < fl(e, l)) continue;
		out.insert(kv.first);
	}
	return out;
}

// Does the score alone rank every ridge edge above every other edge? Pass 3 of the extractor keeps the top K
// by score, so when this holds the right K recovers the ridges and no floor is needed.
static bool ranks(const Mesh &m) {
	if (m.truth.empty()) return true;
	std::map<std::pair<int, int>, std::vector<int>> et;
	for (size_t i = 0; i < m.t.size(); ++i)
		for (int k = 0; k < 3; ++k) et[key(m.t[i][k], m.t[i][(k + 1) % 3])].push_back(int(i));
	auto fn = [&](int i) { V3 a = m.p[m.t[i][0]], b = m.p[m.t[i][1]], c = m.p[m.t[i][2]]; V3 n = cross(sub(b, a), sub(c, a)); double l = len(n); return V3{ n[0] / l, n[1] / l, n[2] / l }; };
	double min_ridge = 1e9, max_other = -1;
	for (auto &kv : et) {
		if (kv.second.size() != 2) continue;
		double sc = std::acos(std::max(-1.0, std::min(1.0, dot(fn(kv.second[0]), fn(kv.second[1])))));
		if (m.truth.count(kv.first)) min_ridge = std::min(min_ridge, sc); else max_other = std::max(max_other, sc);
	}
	return min_ridge > max_other;
}

struct Level { std::string name; std::vector<Mesh> problems; std::vector<double> es; };

// With the argument "body": rdp_error is the body mesh's median edge length (3.46 mm, avatar.obj) and the
// edge lengths are the median edges of (A)Hair_Acc, (A)Skirt and (B)Spats (1.25, 4.0, 12.3 mm).
int main(int argc, char **argv) {
	const bool body = argc > 1 && std::string(argv[1]) == "body";
	const double H[] = { body ? 0.00125 : 0.002, body ? 0.004 : 0.005, body ? 0.0123 : 0.02 };
	const double E[] = { body ? 0.00346 : 0.0005, body ? 0.00346 : 0.001, body ? 0.00346 : 0.002 };
	std::vector<Level> levels;
	int skipped = 0;
	auto add_fold = [&](const char *name, double theta_deg, int want /* 1 ridge, 0 non-ridge */) {
		Level L; L.name = name;
		double theta = theta_deg * M_PI / 180;
		for (double h : H) for (double e : E) {
			double dev = h * std::tan(theta);
			bool ridge = dev > 2 * e, flat = dev < e / 2;
			if ((want == 1 && !ridge) || (want == 0 && !flat)) { ++skipped; continue; }
			Mesh m = fold_grid(10, h, want ? theta : 0.0, e, 1 + unsigned(L.problems.size()));
			if (!want) { // sub-error bend: the surface really bends by theta, but the truth is "no ridge"
				m = fold_grid(10, h, theta, e, 1 + unsigned(L.problems.size()));
				m.truth.clear();
			}
			L.problems.push_back(m); L.es.push_back(e);
		}
		levels.push_back(L);
	};
	{ Level L; L.name = "flat, ripple within e";
	  for (double h : H) for (double e : E) { L.problems.push_back(fold_grid(10, h, 0.0, e, 1 + unsigned(L.problems.size()))); L.es.push_back(e); }
	  levels.push_back(L); }
	{ Level L; L.name = "sphere, ripple within e";
	  for (double h : H) for (double e : E) { double R = 0.15; if (h * h / (8 * R) > e / 2) { ++skipped; continue; } // facet sag over e/2 is a ridge-sized bend, not smooth
		  L.problems.push_back(sphere(R, h, e, 1 + unsigned(L.problems.size()))); L.es.push_back(e); }
	  levels.push_back(L); }
	add_fold("fold 90 deg (ridge)", 90.0, 1);
	add_fold("fold 30 deg (ridge)", 30.0, 1);
	add_fold("fold 10 deg (ridge)", 10.0, 1);
	add_fold("fold 3 deg (inside e)", 3.0, 0);
	add_fold("fold 1 deg (inside e)", 1.0, 0);

	std::printf("problems skipped (neither clearly a ridge nor clearly inside e): %d\n\n", skipped);
	std::printf("%-26s", "level (problems)");
	for (auto &f : kFloors) std::printf(" %12s", f.name);
	std::printf("\n");
	std::vector<double> tot_s(6, 0), tot_n(6, 0), missed(6, 0), ridges(6, 0), falsek(6, 0), nonridge(6, 0);
	for (auto &L : levels) {
		std::printf("%-26s", (L.name + " (" + std::to_string(L.problems.size()) + ")").c_str());
		for (size_t v = 0; v < 6; ++v) {
			int s = 0; int n = int(L.problems.size());
			for (size_t i = 0; i < L.problems.size(); ++i) {
				auto k = kept(L.problems[i], L.es[i], kFloors[v].floor_);
				if (k == L.problems[i].truth) ++s;
				const Mesh &mm = L.problems[i];
				std::set<std::pair<int, int>> interior;
				{
					std::map<std::pair<int, int>, int> cnt;
					for (auto &t3 : mm.t) for (int q = 0; q < 3; ++q) ++cnt[key(t3[q], t3[(q + 1) % 3])];
					for (auto &kv : cnt) if (kv.second == 2) interior.insert(kv.first);
				}
				for (auto &r : mm.truth) { ridges[v] += 1; if (!k.count(r)) missed[v] += 1; }
				for (auto &ed : interior) if (!mm.truth.count(ed)) { nonridge[v] += 1; if (k.count(ed)) falsek[v] += 1; }
			}
			double regret = (s >= 1 ? 1.0 : 0.0) - double(s) / n;
			std::printf("  %3d/%-2d r%+.2f", s, n, regret);
			tot_s[v] += s; tot_n[v] += n;
		}
		std::printf("\n");
	}
	{
		int ok = 0, n = 0;
		for (auto &L : levels) if (!L.problems.empty() && !L.problems[0].truth.empty()) for (auto &mm : L.problems) { ++n; ok += ranks(mm); }
		std::printf("ridge levels where the score alone ranks every ridge edge above every other edge: %d of %d\n\n", ok, n);
	}
	std::printf("%-26s", "frontier success");
	for (size_t v = 0; v < 6; ++v) std::printf("  %3.0f/%-3.0f    ", tot_s[v], tot_n[v]);
	std::printf("\n%-26s", "ridge edges missed");
	for (size_t v = 0; v < 6; ++v) std::printf("  %5.0f/%-6.0f ", missed[v], ridges[v]);
	std::printf("\n%-26s", "non-ridge edges kept");
	for (size_t v = 0; v < 6; ++v) std::printf("  %5.0f/%-6.0f ", falsek[v], nonridge[v]);
	std::printf("\n");
	return 0;
}
