// Wrap a flattened skirt panel (annular sector from geo_flatten) into a cone around the body, seam welded.
// usage: wrap_panel island.obj flat.txt avatar.obj ease out.obj [gap=0.006]
//   island.obj : the 3D panel (one vertex per UV key, faces), flat.txt: "tri corner u v" from geo_flatten,
//   avatar.obj : the body (same frame as the panel times SCALE), ease: waist ease as a fraction of the body ring radius.
// Output is in the avatar's frame (panel * SCALE). Waist = the boundary loop with the higher mean z.
#include <algorithm>
#include <array>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <map>
#include <set>
#include <vector>
using namespace std;
typedef array<double,3> P3;
static const double SCALE = 5.0;
static int fnd(vector<int>& p, int x) { while (p[x] != x) x = p[x] = p[p[x]]; return x; }
int main(int argc, char** argv) {
    if (argc < 6) { fprintf(stderr, "usage: wrap_panel island.obj flat.txt avatar.obj ease out.obj\n"); return 2; }
    vector<P3> V; vector<array<int,3>> T;
    { FILE* f = fopen(argv[1], "r"); char l[512];
      while (fgets(l, 512, f)) { if (l[0]=='v' && l[1]==' ') { P3 p; sscanf(l+2, "%lf %lf %lf", &p[0], &p[1], &p[2]); V.push_back(p); }
        else if (l[0]=='f') { int i[3]; char* s = l+1; for (int k=0;k<3;k++){ i[k]=(int)strtol(s,&s,10)-1; while(*s&&*s!=' '&&*s!='\n')++s; } T.push_back({i[0],i[1],i[2]}); } }
      fclose(f); }
    const int nv = (int)V.size(), nt = (int)T.size();
    // flat coordinates per corner -> per original vertex (seam vertices have two; keep both, key by (vertex, uv))
    vector<array<array<double,2>,3>> UV(nt);
    { FILE* f = fopen(argv[2], "r"); int t, k; double u, v; while (fscanf(f, "%d %d %lf %lf", &t, &k, &u, &v) == 4) UV[t][k] = {u, v}; fclose(f); }
    // boundary loops (3D topology of the island)
    map<pair<int,int>, int> ec; for (auto& t : T) for (int k=0;k<3;k++){ int a=t[k], b=t[(k+1)%3]; ec[{min(a,b),max(a,b)}]++; }
    vector<char> isb(nv,0); vector<int> par(nv); for (int i=0;i<nv;i++) par[i]=i;
    for (auto& e : ec) if (e.second==1) { isb[e.first.first]=isb[e.first.second]=1; par[fnd(par,e.first.first)] = fnd(par,e.first.second); }
    map<int, vector<int>> loops; for (int i=0;i<nv;i++) if (isb[i]) loops[fnd(par,i)].push_back(i);
    if (loops.size() != 2) { fprintf(stderr, "need two boundary loops, found %zu\n", loops.size()); return 1; }
    auto meanz = [&](const vector<int>& L){ double s=0; for (int v : L) s += V[v][2]; return s/L.size(); };
    vector<int> waist = loops.begin()->second, hem = next(loops.begin())->second;
    if (meanz(waist) < meanz(hem)) swap(waist, hem);
    // axis: centroid of the waist loop in xy
    double cx=0, cy=0; for (int v : waist) { cx += V[v][0]; cy += V[v][1]; } cx/=waist.size(); cy/=waist.size();
    auto radial = [&](const vector<int>& L){ double s=0; for (int v : L) s += hypot(V[v][0]-cx, V[v][1]-cy); return s/L.size(); };
    const double Rw0 = radial(waist), Rh = radial(hem), zw = meanz(waist), zh = meanz(hem);
    // body ring at the waist height (avatar frame = panel * SCALE)
    vector<P3> A; { FILE* f = fopen(argv[3], "r"); char l[512]; while (fgets(l,512,f)) if (l[0]=='v'&&l[1]==' ') { P3 p; sscanf(l+2,"%lf %lf %lf",&p[0],&p[1],&p[2]); A.push_back(p);} fclose(f); }
    vector<double> ring; for (auto& p : A) { double z = p[2]/SCALE; if (fabs(z - zw) < 0.012) { double r = hypot(p[0]/SCALE - cx, p[1]/SCALE - cy); if (r < 0.30) ring.push_back(r); } }
    sort(ring.begin(), ring.end());
    if (ring.size() < 20) { fprintf(stderr, "body ring has only %zu points\n", ring.size()); return 1; }
    const double bodyR = ring[ring.size()*9/10]; // outer extent of the torso in the band (90th percentile of radial distance)
    const double ease = atof(argv[4]);
    const double gap = argc > 6 ? atof(argv[6]) : 0.006;
    // outer envelope of the body (torso and legs, arms excluded) per 5 mm height bin, kept at its running maximum going down
    const double zbin = 0.005; const double zmin_env = zh - 0.05, zmax_env = zw + 0.05; const int nb = (int)((zmax_env - zmin_env)/zbin) + 1;
    vector<double> env(nb, 0.0);
    for (auto& p : A) { double z = p[2]/SCALE; if (z < zmin_env || z > zmax_env) continue; double r = hypot(p[0]/SCALE - cx, p[1]/SCALE - cy); if (r < 0.30) { int b = (int)((z - zmin_env)/zbin); env[b] = max(env[b], r); } }
    for (int b = nb - 2; b >= 0; --b) env[b] = max(env[b], 0.0); // bins without body stay 0
    auto envelope = [&](double z) { int b = (int)((z - zmin_env)/zbin); b = max(0, min(nb - 1, b)); double m = 0; for (int k = max(0, b - 2); k <= min(nb - 1, b + 2); ++k) m = max(m, env[k]); return m; };
    const double Rw = bodyR * (1.0 + ease);
    printf("panel: waist loop %zu verts radius %.4f at z %.4f, hem loop %zu verts radius %.4f at z %.4f (panel units = m)\n", waist.size(), Rw0, zw, hem.size(), Rh, zh);
    printf("body ring at waist height: %zu points, p50 %.4f p90 %.4f -> waist radius %.4f (ease %.3f)\n", ring.size(), ring[ring.size()/2], bodyR, Rw, ease);
    // flat apex: Kasa circle fit to the flat points of the waist loop
    set<int> wset(waist.begin(), waist.end()), hset(hem.begin(), hem.end());
    vector<array<double,2>> wp, hp;
    for (int t=0;t<nt;t++) for (int k=0;k<3;k++) { if (wset.count(T[t][k])) wp.push_back(UV[t][k]); if (hset.count(T[t][k])) hp.push_back(UV[t][k]); }
    double sx=0, sy=0; for (auto& q : wp) { sx+=q[0]; sy+=q[1]; } sx/=wp.size(); sy/=wp.size();
    double suu=0, suv=0, svv=0, suuu=0, svvv=0, suvv=0, svuu=0;
    for (auto& q : wp) { double u=q[0]-sx, v=q[1]-sy; suu+=u*u; suv+=u*v; svv+=v*v; suuu+=u*u*u; svvv+=v*v*v; suvv+=u*v*v; svuu+=v*u*u; }
    double det = suu*svv - suv*suv; double uc = 0.5*((suuu+suvv)*svv - (svvv+svuu)*suv)/det, vc = 0.5*((svvv+svuu)*suu - (suuu+suvv)*suv)/det;
    const double ax = uc + sx, ay = vc + sy;
    auto polar = [&](const array<double,2>& q, double& r, double& th){ r = hypot(q[0]-ax, q[1]-ay); th = atan2(q[1]-ay, q[0]-ax); };
    // unwrap angles around the mean direction
    double mx=0, my=0; for (auto& q : wp) { mx += q[0]-ax; my += q[1]-ay; } const double th0 = atan2(my, mx);
    double rw=0, rh=0, thmin=1e9, thmax=-1e9;
    for (auto& q : wp) { double r, th; polar(q, r, th); rw += r; }
    for (auto& q : hp) { double r, th; polar(q, r, th); rh += r; }
    rw /= wp.size(); rh /= hp.size();
    vector<array<double,3>> corner(nt*3); // r, theta-unwrapped per corner
    for (int t=0;t<nt;t++) for (int k=0;k<3;k++) { double r, th; polar(UV[t][k], r, th); double d = th - th0; while (d > M_PI) d -= 2*M_PI; while (d < -M_PI) d += 2*M_PI; corner[t*3+k] = {r, d, 0}; thmin = min(thmin, d); thmax = max(thmax, d); }
    printf("flat apex (%.4f, %.4f), waist r %.4f, hem r %.4f, sector angle %.1f deg, waist arc %.4f m vs body circumference %.4f m\n", ax, ay, rw, rh, (thmax-thmin)*180/M_PI, rw*(thmax-thmin), 2*M_PI*Rw);
    // new 3D position per original vertex (the seam's two sides give the same point because phi is periodic)
    vector<P3> W(nv, {0,0,0}); vector<int> seen(nv, 0);
    for (int t=0;t<nt;t++) for (int k=0;k<3;k++) { int v = T[t][k]; if (seen[v]) continue; seen[v]=1;
        double r = corner[t*3+k][0], d = corner[t*3+k][1]; double tt = (r - rw)/(rh - rw); double phi = 2*M_PI*(d - thmin)/(thmax - thmin);
        double rho = Rw + tt*(Rh - Rw), z = zw + tt*(zh - zw); rho = max(rho, envelope(z) + gap);
        W[v] = {(cx + rho*cos(phi))*SCALE, (cy + rho*sin(phi))*SCALE, z*SCALE}; }
    FILE* o = fopen(argv[5], "w"); for (auto& p : W) fprintf(o, "v %.9f %.9f %.9f\n", p[0], p[1], p[2]);
    for (auto& t : T) fprintf(o, "f %d %d %d\n", t[0]+1, t[1]+1, t[2]+1); fclose(o);
    printf("wrote %s (%d vertices, %d triangles)\n", argv[5], nv, nt);
    return 0;
}
