// usd_drive -- the skirt through usd.elf's own writer and reader on the sandbox host: usd_write_mesh packs the
// Unity fit export (positions, faces, uvs, 21-joint skin) as a .usdz, usd_open_staged reads that package back, and
// every array is compared bit for bit with what went in. No text is written here and no Python is involved.
//
//   usd_drive <garment.obj> <weights.txt> <joints.txt> <sandbox_host lib> <usd.elf> [out.usdz]
//
// The Unity export has no UVs, so the test gives each point a deterministic synthetic uv (its x and y
// shifted into 0..1 by the mesh bounds); it is there to prove the uv path, not to stand in for real UVs.
// The package must also be a zip with its member data on a 64-byte boundary (the rule usd_open relies on).
#include <sandbox_host.h>

#include <algorithm>
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
	std::vector<float> points, uvs;
	std::vector<int32_t> faces, counts;
	std::vector<std::vector<float>> weights; // bone-major: weights[joint][vertex]
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
	float lo[2] = { 1e30f, 1e30f }, hi[2] = { -1e30f, -1e30f };
	for (size_t v = 0; v < g.points.size() / 3; ++v)
		for (int k = 0; k < 2; ++k) { lo[k] = std::min(lo[k], g.points[3 * v + k]); hi[k] = std::max(hi[k], g.points[3 * v + k]); }
	for (size_t v = 0; v < g.points.size() / 3; ++v)
		for (int k = 0; k < 2; ++k) g.uvs.push_back((g.points[3 * v + k] - lo[k]) / (hi[k] - lo[k]));
	return g;
}

// ---- the sandbox host, by dlopen as Unity's DllImport does -----------------------------------------------
#define FN(name) static decltype(&name) p_##name;
FN(sbhost_machine_create) FN(sbhost_destroy_all) FN(sbhost_vmcall) FN(sbhost_resume) FN(sbhost_last_error)
#undef FN
enum { T_INT = 2, T_STRING = 4, T_PBYTE = 29, T_PI32 = 30, T_PF32 = 32 };
template <typename T> static void put(std::vector<uint8_t> &o, T v) { const size_t a = o.size(); o.resize(a + sizeof v); std::memcpy(o.data() + a, &v, sizeof v); }
struct Args {
	std::vector<uint8_t> w{ 0, 0, 0, 0 };
	uint32_t n = 0;
	Args &i(int64_t v) { put<uint32_t>(w, T_INT); put(w, v); return bump(); }
	Args &s(const std::string &v) { put<uint32_t>(w, T_STRING); put<uint32_t>(w, uint32_t(v.size())); w.insert(w.end(), v.begin(), v.end()); return bump(); }
	Args &bytes(const uint8_t *p, size_t len) { put<uint32_t>(w, T_PBYTE); put<uint32_t>(w, uint32_t(len)); w.insert(w.end(), p, p + len); return bump(); }
	template <typename T> Args &packed(uint32_t type, const std::vector<T> &v) {
		put<uint32_t>(w, type); put<uint32_t>(w, uint32_t(v.size()));
		const uint8_t *p = (const uint8_t *)v.data();
		w.insert(w.end(), p, p + v.size() * sizeof(T));
		return bump();
	}
	Args &f32(const std::vector<float> &v) { return packed(T_PF32, v); }
	Args &i32(const std::vector<int32_t> &v) { return packed(T_PI32, v); }
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
	out = Value();
	std::memcpy(&out.type, r, 4);
	const uint8_t *p = r + 4;
	if (out.type == T_STRING) { uint32_t n; std::memcpy(&n, p, 4); out.text.assign((const char *)p + 4, n); }
	else if (out.type == T_INT) std::memcpy(&out.i, p, 8);
	else if (out.type >= 29 && out.type <= 38) out.raw.assign(p + 4, r + rl);
	return true;
}
template <typename T> static std::vector<T> as(const Value &v) { std::vector<T> o(v.raw.size() / sizeof(T)); std::memcpy(o.data(), v.raw.data(), o.size() * sizeof(T)); return o; }

static float ulp_up(float x) { return std::nextafter(x, INFINITY); }

struct Back { std::vector<float> pts, uvs, sw; std::vector<int32_t> tris, sidx; };
// First difference between what went in and what came back, or "".
static std::string differ(const Garment &g, const Back &b) {
	if (b.pts.size() != g.points.size()) return "point count";
	for (size_t i = 0; i < b.pts.size(); ++i)
		if (std::memcmp(&b.pts[i], &g.points[i], 4)) return "point float " + std::to_string(i);
	if (b.uvs.size() != g.uvs.size()) return "uv count";
	for (size_t i = 0; i < b.uvs.size(); ++i)
		if (std::memcmp(&b.uvs[i], &g.uvs[i], 4)) return "uv float " + std::to_string(i);
	std::vector<int32_t> want;
	size_t at = 0;
	for (int c : g.counts) { for (int k = 1; k + 1 < c; ++k) { want.push_back(g.faces[at]); want.push_back(g.faces[at + k]); want.push_back(g.faces[at + k + 1]); } at += size_t(c); }
	if (b.tris != want) return "indices";
	const size_t nj = g.joints.size(), nv = g.points.size() / 3;
	if (b.sidx.size() != nv * nj || b.sw.size() != nv * nj) return "skin size";
	for (size_t v = 0; v < nv; ++v)
		for (size_t j = 0; j < nj; ++j) {
			if (b.sidx[v * nj + j] != int32_t(j)) return "skin joint index vertex " + std::to_string(v);
			if (std::memcmp(&b.sw[v * nj + j], &g.weights[j][v], 4)) return "skin weight joint " + std::to_string(j) + " vertex " + std::to_string(v);
		}
	return "";
}

static bool read_back(int64_t m, const std::vector<uint8_t> &pkg, Back &b) {
	Value v;
	const size_t chunk = size_t(512) << 10;
	for (size_t at = 0; at < pkg.size(); at += chunk)
		if (!call(m, "usd_push", Args().bytes(pkg.data() + at, std::min(chunk, pkg.size() - at)), v)) return false;
	if (!call(m, "usd_open_staged", Args(), v)) return false;
	std::printf("usd_open_staged: %s\n", v.text.c_str());
	Value pts, uvs, idx, sidx, sw;
	if (!call(m, "usd_mesh_points", Args().i(0), pts) || !call(m, "usd_mesh_uvs", Args().i(0), uvs) || !call(m, "usd_mesh_indices", Args().i(0), idx) ||
			!call(m, "usd_mesh_skin_indices", Args().i(0), sidx) || !call(m, "usd_mesh_skin_weights", Args().i(0), sw))
		return false;
	if (pts.type != T_PF32 || uvs.type != T_PF32 || idx.type != T_PI32 || sidx.type != T_PI32 || sw.type != T_PF32) {
		std::printf("unexpected types (a string is the guest's FAIL text): %s | %s | %s\n", pts.text.c_str(), uvs.text.c_str(), sidx.text.c_str());
		return false;
	}
	b.pts = as<float>(pts); b.uvs = as<float>(uvs); b.tris = as<int32_t>(idx); b.sidx = as<int32_t>(sidx); b.sw = as<float>(sw);
	return true;
}

int main(int argc, char **argv) {
	if (argc < 6) return 2;
	Garment g = read_garment(argv[1], argv[2], argv[3]);
	void *h = dlopen(argv[4], RTLD_NOW);
	if (!h) { std::fprintf(stderr, "cannot load %s\n", argv[4]); return 2; }
#define L(name) p_##name = (decltype(p_##name))dlsym(h, #name);
	L(sbhost_machine_create) L(sbhost_destroy_all) L(sbhost_vmcall) L(sbhost_resume) L(sbhost_last_error)
#undef L
	std::ifstream ef(argv[5], std::ios::binary);
	std::vector<uint8_t> elf((std::istreambuf_iterator<char>(ef)), std::istreambuf_iterator<char>());
	const int64_t m = p_sbhost_machine_create(elf.data(), elf.size(), 1024, 4096, 1000000, 8000);
	if (!m) { std::fprintf(stderr, "create failed: %s\n", last_error(0).c_str()); return 1; }
	Value v;
	if (!call(m, "usd_init", Args(), v)) return 1;

	// The guest writes the package.
	const size_t nv = g.points.size() / 3, nj = g.joints.size();
	std::string jnames;
	for (const std::string &j : g.joints) jnames += j + "\t-1\n";
	std::vector<int32_t> sidx(nv * nj);
	std::vector<float> sw(nv * nj), ident;
	for (size_t j = 0; j < nj; ++j) ident.insert(ident.end(), { 1, 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0 });
	for (size_t vv = 0; vv < nv; ++vv)
		for (size_t j = 0; j < nj; ++j) { sidx[vv * nj + j] = int32_t(j); sw[vv * nj + j] = g.weights[j][vv]; }
	if (const char *sub = std::getenv("SUB")) { const size_t n = size_t(std::atoi(sub)); g.counts.resize(n); g.faces.resize(n * 3); }
	std::vector<float> floats = g.points;
	floats.insert(floats.end(), g.uvs.begin(), g.uvs.end());
	floats.insert(floats.end(), ident.begin(), ident.end());
	floats.insert(floats.end(), sw.begin(), sw.end());
	std::vector<int32_t> ints = g.counts;
	ints.insert(ints.end(), g.faces.begin(), g.faces.end());
	ints.insert(ints.end(), sidx.begin(), sidx.end());
	const std::string header = "ASkirt\nnp=" + std::to_string(nv) + "\nnuv=" + std::to_string(g.uvs.size()) + "\nnrest=" + std::to_string(ident.size()) +
			"\nnf=" + std::to_string(g.counts.size()) + "\nni=" + std::to_string(g.faces.size()) + "\nsource=Unity fit export";
	if (std::getenv("TINY")) {
		Value tz;
		std::vector<float> tf = { 0, 0, 0, 1, 0, 0, 0, 1, 0 };
		std::vector<int32_t> ti = { 3, 0, 1, 2 };
		bool ok = call(m, "usd_write_mesh", Args().f32(tf).i32(ti).s("T\nnp=3\nnuv=0\nnrest=0\nnf=1\nni=3").s(""), tz);
		Value cz;
		std::vector<int32_t> cc = { 3 }, bb;
		bool cok = call(m, "usd_write_curves", Args().f32(tf).i32(cc).i32(bb).s("a").s("k=v"), cz);
		std::printf("curves control (arrays then strings): ok=%d text=%.60s\n", cok, cz.text.c_str());
		std::printf("tiny: ok=%d type=%u bytes=%zu text=%s\n", ok, tz.type, tz.raw.size(), tz.text.c_str());
	}
	Value z;
	if (!call(m, "usd_write_mesh", Args().f32(floats).i32(ints).s(header).s(jnames), z))
		return 1;
	if (z.type != T_PBYTE) { std::printf("usd_write_mesh answered a string: %s\n", z.text.c_str()); return 1; }
	std::printf("usd_write_mesh: %zu bytes\n", z.raw.size());
	if (argc > 6) std::ofstream(argv[6], std::ios::binary).write((const char *)z.raw.data(), std::streamsize(z.raw.size()));

	// It is a zip with the member data on a 64-byte boundary.
	uint32_t sig = 0; std::memcpy(&sig, z.raw.data(), 4);
	uint16_t nlen = 0, elen = 0; std::memcpy(&nlen, z.raw.data() + 26, 2); std::memcpy(&elen, z.raw.data() + 28, 2);
	const size_t data_at = 30 + nlen + elen;
	std::printf("package: signature %s, member '%.*s', data at byte %zu (%s)\n", sig == 0x04034b50 ? "PK 03 04" : "WRONG", int(nlen), (const char *)z.raw.data() + 30, data_at, data_at % 64 == 0 ? "64-byte aligned" : "NOT aligned");
	const bool packed = sig == 0x04034b50 && data_at % 64 == 0;

	// And usd.elf reads that package back.
	Back b;
	if (!read_back(m, z.raw, b)) return 1;
	const std::string d = differ(g, b);
	std::printf("round trip through usd_write_mesh and usd_open_staged: %s\n", d.empty() ? "identical" : ("DIFFERS at " + d).c_str());
	Garment gp = g; gp.points[21] = ulp_up(gp.points[21]);
	Garment gu = g; gu.uvs[40] = ulp_up(gu.uvs[40]);
	Garment gw = g; gw.weights[3][11] = ulp_up(gw.weights[3][11]);
	const std::string d1 = differ(gp, b), d2 = differ(gu, b), d3 = differ(gw, b);
	std::printf("planted one-ulp point change -> %s\nplanted one-ulp uv change -> %s\nplanted one-ulp weight change -> %s\n",
			d1.empty() ? "NOT CAUGHT" : d1.c_str(), d2.empty() ? "NOT CAUGHT" : d2.c_str(), d3.empty() ? "NOT CAUGHT" : d3.c_str());
	const bool ok = packed && d.empty() && !d1.empty() && !d2.empty() && !d3.empty();
	std::printf("%s\n", ok ? "PASS" : "FAIL");
	p_sbhost_destroy_all();
	return ok ? 0 : 1;
}
