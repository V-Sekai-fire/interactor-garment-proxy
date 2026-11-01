// Per-garment edge statistics for the garments exported from the Unity project (fit-work/export/garment_*.obj).
// Vertices are welded at 1e-6 m as to_bin.py does. Prints welded vertices, triangles, shells, open edges and
// the edge length minimum, median, mean and maximum in metres. Usage: edge_stats a.obj b.obj ...
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <map>
#include <numeric>
#include <sstream>
#include <string>
#include <vector>
int main(int argc, char **argv) {
	std::printf("%-22s %8s %8s %6s %6s %9s %9s %9s %9s\n", "garment", "verts", "tris", "shells", "open", "min_m", "median_m", "mean_m", "max_m");
	for (int a = 1; a < argc; ++a) {
		std::ifstream f(argv[a]);
		std::vector<double> P;
		std::vector<long long> W;
		std::map<std::array<long long, 3>, int> weld;
		std::vector<int> remap;
		std::vector<std::array<int, 3>> T;
		std::string line;
		while (std::getline(f, line)) {
			if (line.rfind("v ", 0) == 0) {
				double x, y, z;
				std::sscanf(line.c_str() + 2, "%lf %lf %lf", &x, &y, &z);
				std::array<long long, 3> q = { std::llround(x / 1e-6), std::llround(y / 1e-6), std::llround(z / 1e-6) };
				auto it = weld.find(q);
				if (it == weld.end()) { it = weld.emplace(q, int(P.size() / 3)).first; P.insert(P.end(), { x, y, z }); }
				remap.push_back(it->second);
			} else if (line.rfind("f ", 0) == 0) {
				std::istringstream s(line.substr(2));
				std::vector<int> id;
				std::string tok;
				while (s >> tok) id.push_back(remap[std::atoi(tok.c_str()) - 1]);
				for (size_t k = 1; k + 1 < id.size(); ++k) {
					if (id[0] != id[k] && id[k] != id[k + 1] && id[0] != id[k + 1]) T.push_back({ id[0], id[k], id[k + 1] });
				}
			}
		}
		size_t nv = P.size() / 3;
		std::vector<int> uf(nv);
		std::iota(uf.begin(), uf.end(), 0);
		auto find = [&](int x) { while (uf[x] != x) x = uf[x] = uf[uf[x]]; return x; };
		std::map<std::pair<int, int>, int> ec;
		for (auto &t : T) for (int k = 0; k < 3; ++k) {
			int u = t[k], v = t[(k + 1) % 3];
			ec[{ std::min(u, v), std::max(u, v) }]++;
			uf[find(u)] = find(v);
		}
		std::vector<int> used(nv, 0);
		for (auto &t : T) for (int k = 0; k < 3; ++k) used[t[k]] = 1;
		std::vector<int> roots;
		for (size_t v = 0; v < nv; ++v) if (used[v]) roots.push_back(find(int(v)));
		std::sort(roots.begin(), roots.end());
		long shells = std::unique(roots.begin(), roots.end()) - roots.begin();
		long open = 0;
		std::vector<double> L;
		for (auto &kv : ec) {
			if (kv.second == 1) ++open;
			double dx = P[3 * kv.first.first] - P[3 * kv.first.second], dy = P[3 * kv.first.first + 1] - P[3 * kv.first.second + 1], dz = P[3 * kv.first.first + 2] - P[3 * kv.first.second + 2];
			L.push_back(std::sqrt(dx * dx + dy * dy + dz * dz));
		}
		std::sort(L.begin(), L.end());
		double mean = std::accumulate(L.begin(), L.end(), 0.0) / L.size();
		std::string name = argv[a];
		name = name.substr(name.find_last_of('/') + 1);
		std::printf("%-22s %8zu %8zu %6ld %6ld %9.5f %9.5f %9.5f %9.5f\n", name.c_str(), size_t(std::count(used.begin(), used.end(), 1)), T.size(), shells, open, L.front(), L[L.size() / 2], mean, L.back());
	}
	return 0;
}
