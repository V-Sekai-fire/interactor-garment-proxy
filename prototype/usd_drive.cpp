// usd_drive -- carry the Unity fit export through OpenUSD and read it back with usd.elf on the sandbox host.
//
// OpenUSD is only the transport. `write` turns an exported garment (OBJ positions, bone-major weights, joint
// names) into a .usda layer: a mesh with its faces and points, and a skeleton binding with dense skin weights.
// `check` loads usd.elf into the sandbox host, pushes the layer in chunks (usd_push, usd_open_staged), reads the
// points, indices and skin back, and compares each array with the OBJ and weights file bit for bit. It then
// plants a one-ulp change in a point and in a weight and requires the comparison to fail on each.
//
//   usd_drive write <garment.obj> <weights.txt> <joints.txt> <out.usda>
//   usd_drive check <garment.obj> <weights.txt> <joints.txt> <sandbox_host lib> <usd.elf> <layer.usda>
#include <sandbox_host.h>

#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <dlfcn.h>
#include <fstream>
#include <iterator>
#include <sstream>
#include <string>
#include <vector>

struct Garment {
	std::vector<float> points;                // 3 per vertex
	std::vector<int32_t> faces, counts;       // polygon corners and corner counts
	std::vector<std::vector<float>> weights;  // bone-major: weights[joint][vertex]
	std::vector<std::string> joints;
};

static Garment read_garment(const char *obj, const char *wts, const char *jts) {
	Garment g;
	std::ifstream f(obj);
	std::string line;
	while (std::getline(f, line)) {
		if (line.rfind("v ", 0) == 0) {
			std::istringstream s(line.substr(2));
			std::string a, b, c;
			s >> a >> b >> c;
			g.points.push_back(std::strtof(a.c_str(), nullptr));
			g.points.push_back(std::strtof(b.c_str(), nullptr));
			g.points.push_back(std::strtof(c.c_str(), nullptr));
		} else if (line.rfind("f ", 0) == 0) {
			std::istringstream s(line.substr(2));
			std::string tok;
			int n = 0;
			while (s >> tok) { g.faces.push_back(std::atoi(tok.c_str()) - 1); ++n; }
			g.counts.push_back(n);
		}
	}
	std::ifstream w(wts);
	while (std::getline(w, line)) {
		if (line.empty()) continue;
		std::istringstream s(line);
		std::string t;
		std::vector<float> row;
		while (s >> t) row.push_back(std::strtof(t.c_str(), nullptr));
		g.weights.push_back(row);
	}
	std::ifstream j(jts);
	while (std::getline(j, line))
		if (!line.empty()) g.joints.push_back(line);
	return g;
}

static std::string write_usda(const Garment &g) {
	const size_t nv = g.points.size() / 3, nj = g.joints.size();
	std::string o = "#usda 1.0\n(\n    defaultPrim = \"Garment\"\n    metersPerUnit = 1\n    upAxis = \"Y\"\n)\n\n";
	char b[64];
	o += "def SkelRoot \"Garment\"\n{\n    def Skeleton \"Skel\"\n    {\n        uniform token[] joints = [";
	for (size_t j = 0; j < nj; ++j) o += (j ? ", \"" : "\"") + g.joints[j] + "\"";
	o += "]\n        uniform matrix4d[] bindTransforms = [";
	for (size_t j = 0; j < nj; ++j) o += j ? ", ((1, 0, 0, 0), (0, 1, 0, 0), (0, 0, 1, 0), (0, 0, 0, 1))" : "((1, 0, 0, 0), (0, 1, 0, 0), (0, 0, 1, 0), (0, 0, 0, 1))";
	o += "]\n        uniform matrix4d[] restTransforms = [";
	for (size_t j = 0; j < nj; ++j) o += j ? ", ((1, 0, 0, 0), (0, 1, 0, 0), (0, 0, 1, 0), (0, 0, 0, 1))" : "((1, 0, 0, 0), (0, 1, 0, 0), (0, 0, 1, 0), (0, 0, 0, 1))";
	o += "]\n    }\n\n    def Mesh \"Mesh\" (\n        prepend apiSchemas = [\"SkelBindingAPI\"]\n    )\n    {\n";
	o += "        int[] faceVertexCounts = [";
	for (size_t i = 0; i < g.counts.size(); ++i) { std::snprintf(b, sizeof b, i ? ", %d" : "%d", g.counts[i]); o += b; }
	o += "]\n        int[] faceVertexIndices = [";
	for (size_t i = 0; i < g.faces.size(); ++i) { std::snprintf(b, sizeof b, i ? ", %d" : "%d", g.faces[i]); o += b; }
	o += "]\n        point3f[] points = [";
	for (size_t v = 0; v < nv; ++v) {
		std::snprintf(b, sizeof b, "%s(%.9g, %.9g, %.9g)", v ? ", " : "", g.points[3 * v], g.points[3 * v + 1], g.points[3 * v + 2]);
		o += b;
	}
	o += "]\n        uniform token[] skel:joints = [";
	for (size_t j = 0; j < nj; ++j) o += (j ? ", \"" : "\"") + g.joints[j] + "\"";
	o += "]\n        rel skel:skeleton = </Garment/Skel>\n";
	std::snprintf(b, sizeof b, "%zu", nj);
	o += std::string("        int[] primvars:skel:jointIndices = [");
	for (size_t v = 0; v < nv; ++v)
		for (size_t j = 0; j < nj; ++j) { std::snprintf(b, sizeof b, "%s%zu", (v || j) ? ", " : "", j); o += b; }
	o += std::string("] (\n            elementSize = ") + std::to_string(nj) + "\n            interpolation = \"vertex\"\n        )\n";
	o += "        float[] primvars:skel:jointWeights = [";
	for (size_t v = 0; v < nv; ++v)
		for (size_t j = 0; j < nj; ++j) { std::snprintf(b, sizeof b, "%s%.9g", (v || j) ? ", " : "", g.weights[j][v]); o += b; }
	o += std::string("] (\n            elementSize = ") + std::to_string(nj) + "\n            interpolation = \"vertex\"\n        )\n    }\n}\n";
	return o;
}

// ---- the sandbox host, by dlopen as Unity's DllImport does -----------------------------------------------
#define FN(name) static decltype(&name) p_##name;
FN(sbhost_machine_create) FN(sbhost_destroy_all) FN(sbhost_vmcall) FN(sbhost_resume) FN(sbhost_last_error) FN(sbhost_take_output)
#undef FN
enum { T_INT = 2, T_STRING = 4, T_PBYTE = 29, T_PI32 = 30, T_PF32 = 32 };
template <typename T> static void put(std::vector<uint8_t> &o, T v) { const size_t a = o.size(); o.resize(a + sizeof v); std::memcpy(o.data() + a, &v, sizeof v); }
struct Args {
	std::vector<uint8_t> w{ 0, 0, 0, 0 };
	uint32_t n = 0;
	Args &i(int64_t v) { put<uint32_t>(w, T_INT); put(w, v); return bump(); }
	Args &bytes(const uint8_t *p, size_t len) { put<uint32_t>(w, T_PBYTE); put<uint32_t>(w, uint32_t(len)); w.insert(w.end(), p, p + len); return bump(); }
	Args &bump() { ++n; std::memcpy(w.data(), &n, 4); return *this; }
};
struct Value {
	uint32_t type = 0;
	std::string text;
	std::vector<uint8_t> raw;
	int64_t i = 0;
};
static std::string last_error(int64_t m) { char b[4096]; const int n = p_sbhost_last_error(m, b, sizeof b); return std::string(b, n < 0 ? 0 : (n > int(sizeof b) ? int(sizeof b) : n)); }
static bool call(int64_t m, const char *fn, const Args &a, Value &out) {
	const uint8_t *r = nullptr;
	uint32_t rl = 0;
	int32_t rc = p_sbhost_vmcall(m, fn, uint32_t(std::strlen(fn)), a.w.data(), uint32_t(a.w.size()), &r, &rl);
	while (rc == SBHOST_SUSPENDED) rc = p_sbhost_resume(m, &r, &rl);
	if (rc != SBHOST_OK) { std::fprintf(stderr, "%s: rc %d %s\n", fn, rc, last_error(m).c_str()); return false; }
	std::memcpy(&out.type, r, 4);
	const uint8_t *p = r + 4;
	if (out.type == T_STRING) { uint32_t n; std::memcpy(&n, p, 4); out.text.assign((const char *)p + 4, n); }
	else if (out.type == T_INT) std::memcpy(&out.i, p, 8);
	else if (out.type >= 29 && out.type <= 38) out.raw.assign(p + 4, r + rl);
	return true;
}
template <typename T> static std::vector<T> as(const Value &v) { std::vector<T> o(v.raw.size() / sizeof(T)); std::memcpy(o.data(), v.raw.data(), o.size() * sizeof(T)); return o; }

static float ulp_up(float x) { return std::nextafter(x, INFINITY); }
// First difference between the expected and the read-back arrays, or "".
static std::string differ(const Garment &g, const std::vector<float> &pts, const std::vector<int32_t> &tris, const std::vector<int32_t> &sidx, const std::vector<float> &sw) {
	if (pts.size() != g.points.size()) return "point count";
	for (size_t i = 0; i < pts.size(); ++i)
		if (std::memcmp(&pts[i], &g.points[i], 4)) return "point float " + std::to_string(i);
	std::vector<int32_t> want;
	size_t at = 0;
	for (int c : g.counts) { for (int k = 1; k + 1 < c; ++k) { want.push_back(g.faces[at]); want.push_back(g.faces[at + k]); want.push_back(g.faces[at + k + 1]); } at += size_t(c); }
	if (tris != want) return "indices";
	const size_t nj = g.joints.size(), nv = g.points.size() / 3;
	if (sidx.size() != nv * nj || sw.size() != nv * nj) return "skin size";
	for (size_t v = 0; v < nv; ++v)
		for (size_t j = 0; j < nj; ++j) {
			if (sidx[v * nj + j] != int32_t(j)) return "skin joint index vertex " + std::to_string(v);
			if (std::memcmp(&sw[v * nj + j], &g.weights[j][v], 4)) return "skin weight joint " + std::to_string(j) + " vertex " + std::to_string(v);
		}
	return "";
}

int main(int argc, char **argv) {
	if (argc < 6) return 2;
	const std::string mode = argv[1];
	Garment g = read_garment(argv[2], argv[3], argv[4]);
	if (mode == "write") {
		std::ofstream(argv[5]) << write_usda(g);
		std::printf("wrote %s: vertices %zu faces %zu joints %zu\n", argv[5], g.points.size() / 3, g.counts.size(), g.joints.size());
		return 0;
	}
	if (argc < 8) return 2;
	void *h = dlopen(argv[5], RTLD_NOW);
	if (!h) { std::fprintf(stderr, "cannot load %s\n", argv[5]); return 2; }
#define L(name) p_##name = (decltype(p_##name))dlsym(h, #name);
	L(sbhost_machine_create) L(sbhost_destroy_all) L(sbhost_vmcall) L(sbhost_resume) L(sbhost_last_error) L(sbhost_take_output)
#undef L
	std::ifstream ef(argv[6], std::ios::binary);
	std::vector<uint8_t> elf((std::istreambuf_iterator<char>(ef)), std::istreambuf_iterator<char>());
	std::ifstream lf(argv[7], std::ios::binary);
	std::vector<uint8_t> layer((std::istreambuf_iterator<char>(lf)), std::istreambuf_iterator<char>());
	const int64_t m = p_sbhost_machine_create(elf.data(), elf.size(), 1024, 4096, 1000000, 8000);
	if (!m) { std::fprintf(stderr, "create failed: %s\n", last_error(0).c_str()); return 1; }
	Value v;
	if (!call(m, "usd_init", Args(), v)) return 1;
	const size_t chunk = size_t(512) << 10;
	for (size_t at = 0; at < layer.size(); at += chunk)
		if (!call(m, "usd_push", Args().bytes(layer.data() + at, std::min(chunk, layer.size() - at)), v)) return 1;
	if (!call(m, "usd_open_staged", Args(), v)) return 1;
	std::printf("usd_open_staged: %s\n", v.text.c_str());
	if (!call(m, "usd_mesh_count", Args(), v)) return 1;
	std::printf("meshes: %lld\n", (long long)v.i);
	Value pts, idx, sidx, sw;
	if (!call(m, "usd_mesh_points", Args().i(0), pts) || !call(m, "usd_mesh_indices", Args().i(0), idx) ||
			!call(m, "usd_mesh_skin_indices", Args().i(0), sidx) || !call(m, "usd_mesh_skin_weights", Args().i(0), sw))
		return 1;
	if (pts.type != T_PF32 || idx.type != T_PI32 || sidx.type != T_PI32 || sw.type != T_PF32) {
		std::printf("unexpected types points %u indices %u skin_indices %u skin_weights %u (a string here is the guest's FAIL text: %s | %s)\n",
				pts.type, idx.type, sidx.type, sw.type, pts.text.c_str(), sidx.text.c_str());
		return 1;
	}
	auto P = as<float>(pts); auto I = as<int32_t>(idx); auto SI = as<int32_t>(sidx); auto SW = as<float>(sw);
	const std::string d = differ(g, P, I, SI, SW);
	std::printf("round trip through usd.elf: %s\n", d.empty() ? "identical" : ("DIFFERS at " + d).c_str());
	Garment gp = g; gp.points[21] = ulp_up(gp.points[21]);
	Garment gw = g; gw.weights[3][11] = ulp_up(gw.weights[3][11]);
	const std::string d1 = differ(gp, P, I, SI, SW), d2 = differ(gw, P, I, SI, SW);
	std::printf("planted one-ulp point change -> %s\nplanted one-ulp weight change -> %s\n", d1.empty() ? "NOT CAUGHT" : d1.c_str(), d2.empty() ? "NOT CAUGHT" : d2.c_str());
	const bool ok = d.empty() && !d1.empty() && !d2.empty();
	std::printf("%s\n", ok ? "PASS" : "FAIL");
	p_sbhost_destroy_all();
	return ok ? 0 : 1;
}
