// Flatten a garment panel with geogram (ABF++ or LSCM) and report the distortion against the 3D surface.
// A panel that is an annulus (a skirt: waist ring and hem ring, uncut) is cut open first along the shortest path between
// its two boundary loops, so geogram sees a disk.
// usage: geo_flatten in.obj <abf|lscm> flat.txt
// flat.txt: one line per triangle corner "tri corner u v", scaled so the flat area equals the 3D area (units of the input).
// Triangle order is the input order. Distortion per triangle: s1 >= s2 are the singular values of the map flat -> 3D;
// isotropy = s2/s1 (1 = scale only); area scale = s1*s2 relative to the median (1 = the same scale everywhere).
#include <geogram/basic/common.h>
#include <geogram/basic/attributes.h>
#include <geogram/mesh/mesh.h>
#include <geogram/parameterization/mesh_ABF.h>
#include <geogram/parameterization/mesh_LSCM.h>
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <map>
#include <queue>
#include <set>
#include <string>
#include <vector>

typedef std::array<double, 3> P3;
static double dist3(const P3& a, const P3& b) { return std::sqrt((a[0]-b[0])*(a[0]-b[0]) + (a[1]-b[1])*(a[1]-b[1]) + (a[2]-b[2])*(a[2]-b[2])); }
static int find(std::vector<int>& p, int x) { while (p[x] != x) x = p[x] = p[p[x]]; return x; }

int main(int argc, char** argv) {
    if (argc < 4) { std::fprintf(stderr, "usage: geo_flatten in.obj <abf|lscm> flat.txt\n"); return 2; }
    std::vector<P3> V; std::vector<std::array<int, 3>> T;
    {
        FILE* fp = std::fopen(argv[1], "r"); if (!fp) { std::fprintf(stderr, "cannot open %s\n", argv[1]); return 1; }
        char l[512];
        while (std::fgets(l, 512, fp)) {
            if (l[0] == 'v' && l[1] == ' ') { P3 p; std::sscanf(l + 2, "%lf %lf %lf", &p[0], &p[1], &p[2]); V.push_back(p); }
            else if (l[0] == 'f') { int i[3]; char* s = l + 1; for (int k = 0; k < 3; ++k) { i[k] = (int)std::strtol(s, &s, 10) - 1; while (*s && *s != ' ' && *s != '\n') ++s; } T.push_back({i[0], i[1], i[2]}); }
        }
        std::fclose(fp);
    }
    const int nv0 = (int)V.size(), nf = (int)T.size();
    // edges and boundary
    std::map<std::pair<int,int>, int> ecount;
    for (auto& t : T) for (int k = 0; k < 3; ++k) { int a = t[k], b = t[(k+1)%3]; ecount[{std::min(a,b), std::max(a,b)}]++; }
    auto euler = [&](int nv) { return (long)nv - (long)ecount.size() + (long)nf; };
    std::vector<char> isb(nv0, 0);
    for (auto& e : ecount) if (e.second == 1) { isb[e.first.first] = 1; isb[e.first.second] = 1; }
    std::vector<int> par(nv0); for (int i = 0; i < nv0; ++i) par[i] = i;
    for (auto& e : ecount) if (e.second == 1) par[find(par, e.first.first)] = find(par, e.first.second);
    std::map<int, std::vector<int>> loops;
    for (int i = 0; i < nv0; ++i) if (isb[i]) loops[find(par, i)].push_back(i);
    std::printf("input: %d vertices, %d triangles, Euler %ld, %zu boundary loop(s)\n", nv0, nf, euler(nv0), loops.size());

    if (loops.size() == 2) {
        // cut along the shortest interior path from loop A to loop B
        std::vector<int> la = loops.begin()->second, lb = std::next(loops.begin())->second;
        std::vector<char> inB(nv0, 0); for (int v : lb) inB[v] = 1;
        std::vector<std::vector<int>> adj(nv0);
        for (auto& e : ecount) { int a = e.first.first, b = e.first.second; if (e.second == 1 || (isb[a] && isb[b])) continue; adj[a].push_back(b); adj[b].push_back(a); }
        std::vector<double> d(nv0, 1e30); std::vector<int> prev(nv0, -1);
        typedef std::pair<double,int> QI; std::priority_queue<QI, std::vector<QI>, std::greater<QI>> q;
        for (int v : la) { d[v] = 0; q.push({0, v}); }
        int hit = -1;
        while (!q.empty()) { auto [dd, u] = q.top(); q.pop(); if (dd > d[u]) continue; if (inB[u]) { hit = u; break; }
            for (int w : adj[u]) { double nd = dd + dist3(V[u], V[w]); if (nd < d[w]) { d[w] = nd; prev[w] = u; q.push({nd, w}); } } }
        if (hit < 0) { std::fprintf(stderr, "no path between the loops\n"); return 1; }
        std::vector<int> path; for (int v = hit; v != -1; v = prev[v]) path.push_back(v);
        std::reverse(path.begin(), path.end());
        std::printf("seam: %zu vertices, length %.4f\n", path.size(), d[hit]);
        std::set<std::pair<int,int>> cut; for (size_t i = 0; i + 1 < path.size(); ++i) cut.insert({std::min(path[i], path[i+1]), std::max(path[i], path[i+1])});
        for (int w : path) {
            std::vector<int> inc; for (int t = 0; t < nf; ++t) for (int k = 0; k < 3; ++k) if (T[t][k] == w) inc.push_back(t);
            std::vector<int> comp(inc.size(), -1); int nc = 0;
            for (size_t s = 0; s < inc.size(); ++s) { if (comp[s] != -1) continue; comp[s] = nc; std::vector<size_t> st{s};
                while (!st.empty()) { size_t a = st.back(); st.pop_back();
                    for (size_t b = 0; b < inc.size(); ++b) { if (comp[b] != -1) continue;
                        // triangles a, b share an edge (w, x) that is not cut?
                        bool share = false;
                        for (int ka = 0; ka < 3 && !share; ++ka) { int x = T[inc[a]][ka]; if (x == w) continue;
                            for (int kb = 0; kb < 3; ++kb) if (T[inc[b]][kb] == x) { if (!cut.count({std::min(w,x), std::max(w,x)})) share = true; } }
                        if (share) { comp[b] = nc; st.push_back(b); } } }
                ++nc; }
            for (int c = 1; c < nc; ++c) { int nid = (int)V.size(); V.push_back(V[w]);
                for (size_t s = 0; s < inc.size(); ++s) if (comp[s] == c) for (int k = 0; k < 3; ++k) if (T[inc[s]][k] == w) T[inc[s]][k] = nid; }
        }
        ecount.clear(); for (auto& t : T) for (int k = 0; k < 3; ++k) { int a = t[k], b = t[(k+1)%3]; ecount[{std::min(a,b), std::max(a,b)}]++; }
        std::printf("after cut: %zu vertices, Euler %ld\n", V.size(), euler((int)V.size()));
    }
    unsigned nonman = 0; for (auto& e : ecount) if (e.second > 2) ++nonman;
    if (euler((int)V.size()) != 1 || nonman) std::printf("WARNING: not a disk (Euler %ld, %u non-manifold edges)\n", euler((int)V.size()), nonman);

    GEO::initialize();
    GEO::Mesh M;
    M.vertices.create_vertices((GEO::index_t)V.size());
    for (size_t i = 0; i < V.size(); ++i) M.vertices.point((GEO::index_t)i) = GEO::vec3(V[i][0], V[i][1], V[i][2]);
    M.facets.create_triangles((GEO::index_t)nf);
    for (int f = 0; f < nf; ++f) for (int k = 0; k < 3; ++k) M.facets.set_vertex((GEO::index_t)f, (GEO::index_t)k, (GEO::index_t)T[f][k]);
    M.facets.connect();
    GEO::Attribute<double> tex;
    tex.create_vector_attribute(M.vertices.attributes(), "tex_coord", 2);
    const std::string method = argv[2];
    if (method == "abf") GEO::mesh_compute_ABF_plus_plus(M, "tex_coord", false);
    else if (method == "lscm") GEO::mesh_compute_LSCM(M, "tex_coord");
    else { std::fprintf(stderr, "method must be abf or lscm\n"); return 2; }

    std::vector<double> iso, asc; double A3 = 0, AUV = 0; int flipped = 0, degenerate = 0;
    for (int f = 0; f < nf; ++f) {
        GEO::vec3 p[3]; double u[3], v[3];
        for (int k = 0; k < 3; ++k) { unsigned c = M.facets.vertex((GEO::index_t)f, (GEO::index_t)k); p[k] = M.vertices.point(c); u[k] = tex[c*2]; v[k] = tex[c*2+1]; }
        GEO::vec3 e1 = p[1]-p[0], e2 = p[2]-p[0];
        double e1u = u[1]-u[0], e1v = v[1]-v[0], e2u = u[2]-u[0], e2v = v[2]-v[0], det = e1u*e2v - e1v*e2u;
        A3 += 0.5 * GEO::length(GEO::cross(e1, e2)); AUV += 0.5 * std::fabs(det);
        if (std::fabs(det) < 1e-300) { ++degenerate; continue; }
        if (det < 0) ++flipped;
        double i00 = e2v/det, i01 = -e2u/det, i10 = -e1v/det, i11 = e1u/det;
        GEO::vec3 ju = e1*i00 + e2*i10, jv = e1*i01 + e2*i11;
        double aa = GEO::dot(ju,ju), bb = GEO::dot(ju,jv), cc = GEO::dot(jv,jv), tr = aa+cc, disc = std::sqrt(std::max(tr*tr*0.25 - (aa*cc - bb*bb), 0.0));
        double l1 = tr*0.5 + disc, l2 = std::max(tr*0.5 - disc, 0.0);
        iso.push_back(std::sqrt(l2 / std::max(l1, 1e-300))); asc.push_back(std::sqrt(l1*l2));
    }
    std::sort(iso.begin(), iso.end()); std::sort(asc.begin(), asc.end());
    const size_t n = iso.size(); const double med = asc[n/2];
    size_t w10 = 0, i90 = 0; for (size_t i = 0; i < n; ++i) { if (std::fabs(asc[i]/med - 1.0) <= 0.10) ++w10; if (iso[i] >= 0.9) ++i90; }
    std::printf("%s: isotropy min %.3f p5 %.3f median %.3f | area scale / median: p5 %.3f p95 %.3f max %.3f\n", argv[2], iso[0], iso[n*5/100], iso[n/2], asc[n*5/100]/med, asc[n*95/100]/med, asc[n-1]/med);
    std::printf("%s: area scale within 10%% of the median %.1f%%, isotropy >= 0.9 %.1f%%, flipped triangles %d, degenerate %d; 3D area %.5f, flat area %.5f (before rescale)\n",
        argv[2], 100.0*w10/n, 100.0*i90/n, flipped, degenerate, A3, AUV);
    const double s = std::sqrt(A3 / AUV);
    FILE* o = std::fopen(argv[3], "w");
    for (int f = 0; f < nf; ++f) for (int k = 0; k < 3; ++k) { unsigned c = M.facets.vertex((GEO::index_t)f, (GEO::index_t)k); std::fprintf(o, "%d %d %.9f %.9f\n", f, k, tex[c*2]*s, tex[c*2+1]*s); }
    std::fclose(o);
    return 0;
}
