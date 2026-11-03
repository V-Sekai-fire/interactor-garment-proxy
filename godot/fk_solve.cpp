// fk_solve.cpp: nonlinear forward-kinematics fit of local bone transforms to target vertex positions (Levenberg-Marquardt).
//
// Called by bone_fit.gd (--solver fk) through OS.execute with an input file and an output file; no Python, no dependencies.
//   clang++ -O3 -std=c++17 fk_solve.cpp -o fk_solve
//   fk_solve <input.bin> <output.bin>
//
// Model (Unity): world[j] = world[parent[j]] * local[j], local = T(pos) * R * diag(scale), composed as 3x3 + translation, so a
// non-uniform scale in a parent shears its children's world matrices exactly as Unity does.
// Skinned position of a vertex = sum_i w_i * world[joint_i] * q_i, q_i = the vertex in that bone's bind frame (the same skinning
// bone_fit.gd already verifies against the Unity bake).
// Variables per variable joint (9, a mask says which are free; the others keep their initial value):
//   dt (3): local translation added to the prefab local translation
//   dr (3): axis-angle in the bone's own local axes, local rotation = prefab local rotation * exp(dr)
//   ls (3): log scale, local scale = prefab local scale * exp(ls)
// Cost = sum_v uw_v |skinned_v - target_v|^2 + reg_t |dt|^2 + reg_r |dr|^2 + reg_s |ls|^2  (free parameters).
// The Jacobian is by the chain rule through FK: a parameter of joint m moves the world matrix of every joint j in the subtree of m by
//   dW_j = E * W_j,  E = W_parent(m) * (dL_m * L_m^-1) * W_parent(m)^-1   (dL_m: the local matrix derivative of that parameter),
// so d(skinned)/dp = sum over influences below m of w * E * (W_j q). dL_m is by a central difference of the 12-number local matrix
// only (exact to ~1e-10); everything else is analytic. A --check mode compares the whole gradient with finite differences of the cost.
//
// Input (little endian): see read_input(). Output: see write_output().
#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <random>
#include <vector>

using std::vector;

struct Aff {
	double r[9];
	double t[3];
};
static Aff ident() {
	Aff a;
	std::memset(&a, 0, sizeof(a));
	a.r[0] = a.r[4] = a.r[8] = 1.0;
	return a;
}
static void mul3(const double *a, const double *b, double *o) {
	double c[9];
	for (int i = 0; i < 3; i++)
		for (int j = 0; j < 3; j++)
			c[i * 3 + j] = a[i * 3] * b[j] + a[i * 3 + 1] * b[3 + j] + a[i * 3 + 2] * b[6 + j];
	std::memcpy(o, c, sizeof(c));
}
static Aff mul(const Aff &a, const Aff &b) {
	Aff o;
	mul3(a.r, b.r, o.r);
	for (int i = 0; i < 3; i++)
		o.t[i] = a.r[i * 3] * b.t[0] + a.r[i * 3 + 1] * b.t[1] + a.r[i * 3 + 2] * b.t[2] + a.t[i];
	return o;
}
static void ap(const Aff &a, const double *x, double *o) {
	for (int i = 0; i < 3; i++)
		o[i] = a.r[i * 3] * x[0] + a.r[i * 3 + 1] * x[1] + a.r[i * 3 + 2] * x[2] + a.t[i];
}
static bool inv3(const double *m, double *o) {
	double det = m[0] * (m[4] * m[8] - m[5] * m[7]) - m[1] * (m[3] * m[8] - m[5] * m[6]) + m[2] * (m[3] * m[7] - m[4] * m[6]);
	if (std::fabs(det) < 1e-300)
		return false;
	double d = 1.0 / det;
	o[0] = (m[4] * m[8] - m[5] * m[7]) * d;
	o[1] = (m[2] * m[7] - m[1] * m[8]) * d;
	o[2] = (m[1] * m[5] - m[2] * m[4]) * d;
	o[3] = (m[5] * m[6] - m[3] * m[8]) * d;
	o[4] = (m[0] * m[8] - m[2] * m[6]) * d;
	o[5] = (m[2] * m[3] - m[0] * m[5]) * d;
	o[6] = (m[3] * m[7] - m[4] * m[6]) * d;
	o[7] = (m[1] * m[6] - m[0] * m[7]) * d;
	o[8] = (m[0] * m[4] - m[1] * m[3]) * d;
	return true;
}
static Aff inverse(const Aff &a) {
	Aff o;
	inv3(a.r, o.r);
	for (int i = 0; i < 3; i++)
		o.t[i] = -(o.r[i * 3] * a.t[0] + o.r[i * 3 + 1] * a.t[1] + o.r[i * 3 + 2] * a.t[2]);
	return o;
}
static void expRot(const double *w, double *R) {
	double th = std::sqrt(w[0] * w[0] + w[1] * w[1] + w[2] * w[2]);
	double K[9] = { 0, -w[2], w[1], w[2], 0, -w[0], -w[1], w[0], 0 };
	double A, B;
	if (th < 1e-8) {
		A = 1.0 - th * th / 6.0;
		B = 0.5 - th * th / 24.0;
	} else {
		A = std::sin(th) / th;
		B = (1.0 - std::cos(th)) / (th * th);
	}
	double K2[9];
	mul3(K, K, K2);
	for (int i = 0; i < 9; i++)
		R[i] = (i % 4 == 0 ? 1.0 : 0.0) + A * K[i] + B * K2[i];
}
static void quatToMat(const double *q, double *R) { // x y z w
	double x = q[0], y = q[1], z = q[2], w = q[3];
	double n = std::sqrt(x * x + y * y + z * z + w * w);
	x /= n; y /= n; z /= n; w /= n;
	R[0] = 1 - 2 * (y * y + z * z); R[1] = 2 * (x * y - z * w); R[2] = 2 * (x * z + y * w);
	R[3] = 2 * (x * y + z * w); R[4] = 1 - 2 * (x * x + z * z); R[5] = 2 * (y * z - x * w);
	R[6] = 2 * (x * z - y * w); R[7] = 2 * (y * z + x * w); R[8] = 1 - 2 * (x * x + y * y);
}
static void matToQuat(const double *m, double *q) {
	double tr = m[0] + m[4] + m[8];
	double x, y, z, w;
	if (tr > 0) {
		double s = std::sqrt(tr + 1.0) * 2;
		w = 0.25 * s; x = (m[7] - m[5]) / s; y = (m[2] - m[6]) / s; z = (m[3] - m[1]) / s;
	} else if (m[0] > m[4] && m[0] > m[8]) {
		double s = std::sqrt(1.0 + m[0] - m[4] - m[8]) * 2;
		w = (m[7] - m[5]) / s; x = 0.25 * s; y = (m[1] + m[3]) / s; z = (m[2] + m[6]) / s;
	} else if (m[4] > m[8]) {
		double s = std::sqrt(1.0 + m[4] - m[0] - m[8]) * 2;
		w = (m[2] - m[6]) / s; x = (m[1] + m[3]) / s; y = 0.25 * s; z = (m[5] + m[7]) / s;
	} else {
		double s = std::sqrt(1.0 + m[8] - m[0] - m[4]) * 2;
		w = (m[3] - m[1]) / s; x = (m[2] + m[6]) / s; y = (m[5] + m[7]) / s; z = 0.25 * s;
	}
	double n = std::sqrt(x * x + y * y + z * z + w * w);
	q[0] = x / n; q[1] = y / n; q[2] = z / n; q[3] = w / n;
}

struct Problem {
	int J = 0, NV = 0, nvar = 0, check = 0, maxit = 100;
	double reg_t = 0, reg_r = 0, reg_s = 0, tol = 1e-6, lam0 = 1e-3;
	vector<int32_t> parent, vidx, mask;
	vector<double> pos0, q0, s0, init;
	vector<int32_t> inj;
	vector<double> inw, inq, target, uw;
	vector<int> var_joint;			// slot -> joint
	vector<int> fslot, fk;			// free index -> slot, param (0..8)
	vector<int> fof;				// slot*9+k -> free index or -1
	vector<double> R0;				// per joint prefab rotation matrix
	int nfree = 0;
};

template <class T>
static bool rd(FILE *f, vector<T> &v, size_t n) {
	v.resize(n);
	return n == 0 || fread(v.data(), sizeof(T), n, f) == n;
}

static bool read_input(const char *path, Problem &P) {
	FILE *f = fopen(path, "rb");
	if (!f)
		return false;
	int32_t hdr[8];
	if (fread(hdr, 4, 8, f) != 8)
		return false;
	P.J = hdr[0]; P.NV = hdr[1]; P.nvar = hdr[2]; P.check = hdr[3]; P.maxit = hdr[4];
	double dh[5];
	if (fread(dh, 8, 5, f) != 5)
		return false;
	P.reg_t = dh[0]; P.reg_r = dh[1]; P.reg_s = dh[2]; P.tol = dh[3]; P.lam0 = dh[4];
	bool ok = rd(f, P.parent, P.J) && rd(f, P.vidx, P.J) && rd(f, P.mask, (size_t)P.nvar * 9) && rd(f, P.pos0, (size_t)P.J * 3) && rd(f, P.q0, (size_t)P.J * 4) &&
			rd(f, P.s0, (size_t)P.J * 3) && rd(f, P.init, (size_t)P.nvar * 9) && rd(f, P.inj, (size_t)P.NV * 4) && rd(f, P.inw, (size_t)P.NV * 4) &&
			rd(f, P.inq, (size_t)P.NV * 12) && rd(f, P.target, (size_t)P.NV * 3) && rd(f, P.uw, P.NV);
	fclose(f);
	if (!ok)
		return false;
	P.var_joint.assign(P.nvar, -1);
	for (int j = 0; j < P.J; j++)
		if (P.vidx[j] >= 0)
			P.var_joint[P.vidx[j]] = j;
	P.fof.assign((size_t)P.nvar * 9, -1);
	for (int v = 0; v < P.nvar; v++)
		for (int k = 0; k < 9; k++)
			if (P.mask[v * 9 + k]) {
				P.fof[v * 9 + k] = (int)P.fslot.size();
				P.fslot.push_back(v);
				P.fk.push_back(k);
			}
	P.nfree = (int)P.fslot.size();
	P.R0.resize((size_t)P.J * 9);
	for (int j = 0; j < P.J; j++)
		quatToMat(&P.q0[j * 4], &P.R0[j * 9]);
	return true;
}

// local matrix of joint j for the 9 parameters p (null: the prefab local)
static Aff local_of(const Problem &P, int j, const double *p) {
	Aff L;
	double R[9];
	double s[3], t[3];
	if (!p) {
		std::memcpy(R, &P.R0[j * 9], sizeof(R));
		for (int i = 0; i < 3; i++) { s[i] = P.s0[j * 3 + i]; t[i] = P.pos0[j * 3 + i]; }
	} else {
		double E[9];
		expRot(p + 3, E);
		mul3(&P.R0[j * 9], E, R);
		for (int i = 0; i < 3; i++) { s[i] = P.s0[j * 3 + i] * std::exp(p[6 + i]); t[i] = P.pos0[j * 3 + i] + p[i]; }
	}
	for (int i = 0; i < 3; i++)
		for (int c = 0; c < 3; c++)
			L.r[i * 3 + c] = R[i * 3 + c] * s[c];
	for (int i = 0; i < 3; i++)
		L.t[i] = t[i];
	return L;
}

static void forward(const Problem &P, const vector<double> &par, vector<Aff> &L, vector<Aff> &W) {
	L.resize(P.J);
	W.resize(P.J);
	for (int j = 0; j < P.J; j++) {
		L[j] = local_of(P, j, P.vidx[j] >= 0 ? &par[(size_t)P.vidx[j] * 9] : nullptr);
		W[j] = P.parent[j] < 0 ? L[j] : mul(W[P.parent[j]], L[j]);
	}
}

static double reg_cost(const Problem &P, const vector<double> &par) {
	double c = 0;
	for (int f = 0; f < P.nfree; f++) {
		int v = P.fslot[f], k = P.fk[f];
		double x = par[v * 9 + k];
		c += (k < 3 ? P.reg_t : (k < 6 ? P.reg_r : P.reg_s)) * x * x;
	}
	return c;
}

static double cost_of(const Problem &P, const vector<double> &par, vector<Aff> &L, vector<Aff> &W) {
	forward(P, par, L, W);
	double c = 0;
	for (int u = 0; u < P.NV; u++) {
		double x[3] = { 0, 0, 0 };
		for (int i = 0; i < 4; i++) {
			double w = P.inw[u * 4 + i];
			if (w == 0.0)
				continue;
			double y[3];
			ap(W[P.inj[u * 4 + i]], &P.inq[(u * 4 + i) * 3], y);
			x[0] += w * y[0]; x[1] += w * y[1]; x[2] += w * y[2];
		}
		double dx = x[0] - P.target[u * 3], dy = x[1] - P.target[u * 3 + 1], dz = x[2] - P.target[u * 3 + 2];
		c += P.uw[u] * (dx * dx + dy * dy + dz * dz);
	}
	return c + reg_cost(P, par);
}

struct Lin {
	vector<double> H; // nfree x nfree (2 J^T W J + 2 reg), full symmetric
	vector<double> g; // gradient of the cost
	double cost = 0;
};

// Gauss-Newton pieces at par: cost, gradient, Hessian approximation
static void linearize(const Problem &P, const vector<double> &par, Lin &out, vector<Aff> &L, vector<Aff> &W) {
	forward(P, par, L, W);
	int nf = P.nfree;
	out.H.assign((size_t)nf * nf, 0.0);
	out.g.assign(nf, 0.0);
	// E for every free parameter
	vector<double> EA((size_t)nf * 9), Eb((size_t)nf * 3);
	for (int f = 0; f < nf; f++) {
		int v = P.fslot[f], k = P.fk[f], j = P.var_joint[v];
		double hh = 1e-6;
		double pp[9];
		std::memcpy(pp, &par[v * 9], sizeof(pp));
		pp[k] += hh;
		Aff Lp = local_of(P, j, pp);
		pp[k] -= 2 * hh;
		Aff Lm = local_of(P, j, pp);
		double dr[9], dt[3];
		for (int i = 0; i < 9; i++)
			dr[i] = (Lp.r[i] - Lm.r[i]) / (2 * hh);
		for (int i = 0; i < 3; i++)
			dt[i] = (Lp.t[i] - Lm.t[i]) / (2 * hh);
		Aff Li = inverse(L[j]);
		// Dm = dL * L^-1 : A = dr * Li.r, b = dr * Li.t + dt
		double A[9], b[3];
		mul3(dr, Li.r, A);
		for (int i = 0; i < 3; i++)
			b[i] = dr[i * 3] * Li.t[0] + dr[i * 3 + 1] * Li.t[1] + dr[i * 3 + 2] * Li.t[2] + dt[i];
		Aff Wp = P.parent[j] < 0 ? ident() : W[P.parent[j]];
		Aff Wpi = inverse(Wp);
		// E(y) = Wp.r * (A * (Wpi y) + b)  =>  A_E = Wp.r A Wpi.r, b_E = Wp.r (A Wpi.t + b)
		double t1[9], AE[9];
		mul3(Wp.r, A, t1);
		mul3(t1, Wpi.r, AE);
		double At[3];
		for (int i = 0; i < 3; i++)
			At[i] = A[i * 3] * Wpi.t[0] + A[i * 3 + 1] * Wpi.t[1] + A[i * 3 + 2] * Wpi.t[2] + b[i];
		for (int i = 0; i < 9; i++)
			EA[(size_t)f * 9 + i] = AE[i];
		for (int i = 0; i < 3; i++)
			Eb[(size_t)f * 3 + i] = Wp.r[i * 3] * At[0] + Wp.r[i * 3 + 1] * At[1] + Wp.r[i * 3 + 2] * At[2];
	}
	vector<double> gr((size_t)nf * 3, 0.0);
	vector<int> stamp(nf, -1), touched;
	double cost = 0;
	for (int u = 0; u < P.NV; u++) {
		double x[3] = { 0, 0, 0 };
		double ys[4][3];
		touched.clear();
		for (int i = 0; i < 4; i++) {
			double w = P.inw[u * 4 + i];
			if (w == 0.0)
				continue;
			ap(W[P.inj[u * 4 + i]], &P.inq[(u * 4 + i) * 3], ys[i]);
			x[0] += w * ys[i][0]; x[1] += w * ys[i][1]; x[2] += w * ys[i][2];
		}
		double r[3] = { x[0] - P.target[u * 3], x[1] - P.target[u * 3 + 1], x[2] - P.target[u * 3 + 2] };
		double uwv = P.uw[u];
		cost += uwv * (r[0] * r[0] + r[1] * r[1] + r[2] * r[2]);
		if (uwv == 0.0)
			continue;
		for (int i = 0; i < 4; i++) {
			double w = P.inw[u * 4 + i];
			if (w == 0.0)
				continue;
			for (int m = P.inj[u * 4 + i]; m >= 0; m = P.parent[m]) {
				int v = P.vidx[m];
				if (v < 0)
					continue;
				for (int k = 0; k < 9; k++) {
					int f = P.fof[v * 9 + k];
					if (f < 0)
						continue;
					if (stamp[f] != u) {
						stamp[f] = u;
						touched.push_back(f);
						gr[(size_t)f * 3] = gr[(size_t)f * 3 + 1] = gr[(size_t)f * 3 + 2] = 0.0;
					}
					const double *A = &EA[(size_t)f * 9];
					const double *b = &Eb[(size_t)f * 3];
					const double *y = ys[i];
					gr[(size_t)f * 3] += w * (A[0] * y[0] + A[1] * y[1] + A[2] * y[2] + b[0]);
					gr[(size_t)f * 3 + 1] += w * (A[3] * y[0] + A[4] * y[1] + A[5] * y[2] + b[1]);
					gr[(size_t)f * 3 + 2] += w * (A[6] * y[0] + A[7] * y[1] + A[8] * y[2] + b[2]);
				}
			}
		}
		for (size_t a = 0; a < touched.size(); a++) {
			int fa = touched[a];
			const double *ga = &gr[(size_t)fa * 3];
			out.g[fa] += 2.0 * uwv * (ga[0] * r[0] + ga[1] * r[1] + ga[2] * r[2]);
			for (size_t b2 = a; b2 < touched.size(); b2++) {
				int fb = touched[b2];
				const double *gb = &gr[(size_t)fb * 3];
				double d = 2.0 * uwv * (ga[0] * gb[0] + ga[1] * gb[1] + ga[2] * gb[2]);
				out.H[(size_t)fa * nf + fb] += d;
				if (fa != fb)
					out.H[(size_t)fb * nf + fa] += d;
			}
		}
	}
	for (int f = 0; f < nf; f++) {
		int v = P.fslot[f], k = P.fk[f];
		double rg = k < 3 ? P.reg_t : (k < 6 ? P.reg_r : P.reg_s);
		out.g[f] += 2.0 * rg * par[v * 9 + k];
		out.H[(size_t)f * nf + f] += 2.0 * rg;
	}
	out.cost = cost + reg_cost(P, par);
}

static bool cholesky_solve(vector<double> A, vector<double> b, int n, vector<double> &x) {
	for (int j = 0; j < n; j++) {
		double s = A[(size_t)j * n + j];
		for (int k = 0; k < j; k++)
			s -= A[(size_t)j * n + k] * A[(size_t)j * n + k];
		if (s <= 1e-300)
			return false;
		double l = std::sqrt(s);
		A[(size_t)j * n + j] = l;
		for (int i = j + 1; i < n; i++) {
			double t = A[(size_t)i * n + j];
			for (int k = 0; k < j; k++)
				t -= A[(size_t)i * n + k] * A[(size_t)j * n + k];
			A[(size_t)i * n + j] = t / l;
		}
	}
	for (int i = 0; i < n; i++) {
		double t = b[i];
		for (int k = 0; k < i; k++)
			t -= A[(size_t)i * n + k] * b[k];
		b[i] = t / A[(size_t)i * n + i];
	}
	for (int i = n - 1; i >= 0; i--) {
		double t = b[i];
		for (int k = i + 1; k < n; k++)
			t -= A[(size_t)k * n + i] * b[k];
		b[i] = t / A[(size_t)i * n + i];
	}
	x = b;
	return true;
}

static void write_output(const char *path, const Problem &P, const vector<double> &par, int iters, int evals, double c0, double c1) {
	vector<Aff> L, W;
	forward(P, par, L, W);
	FILE *f = fopen(path, "wb");
	int32_t h[4] = { iters, evals, P.J, P.nvar };
	fwrite(h, 4, 4, f);
	double c[2] = { c0, c1 };
	fwrite(c, 8, 2, f);
	fwrite(par.data(), 8, par.size(), f);
	for (int j = 0; j < P.J; j++) {
		double rec[3 + 4 + 3 + 12];
		// local decomposition: rotation = exp(dr) R0 (a proper rotation), scale = s0 exp(ls); prefab values for fixed joints
		double R[9], s[3], t[3];
		if (P.vidx[j] >= 0) {
			const double *p = &par[(size_t)P.vidx[j] * 9];
			double E[9];
			expRot(p + 3, E);
			mul3(&P.R0[j * 9], E, R);
			for (int i = 0; i < 3; i++) { s[i] = P.s0[j * 3 + i] * std::exp(p[6 + i]); t[i] = P.pos0[j * 3 + i] + p[i]; }
		} else {
			std::memcpy(R, &P.R0[j * 9], sizeof(R));
			for (int i = 0; i < 3; i++) { s[i] = P.s0[j * 3 + i]; t[i] = P.pos0[j * 3 + i]; }
		}
		for (int i = 0; i < 3; i++) rec[i] = t[i];
		matToQuat(R, rec + 3);
		for (int i = 0; i < 3; i++) rec[7 + i] = s[i];
		for (int i = 0; i < 9; i++) rec[10 + i] = W[j].r[i];
		for (int i = 0; i < 3; i++) rec[19 + i] = W[j].t[i];
		fwrite(rec, 8, 22, f);
	}
	fclose(f);
}

int main(int argc, char **argv) {
	if (argc < 3) {
		fprintf(stderr, "usage: fk_solve input.bin output.bin\n");
		return 2;
	}
	Problem P;
	if (!read_input(argv[1], P)) {
		fprintf(stderr, "cannot read input\n");
		return 2;
	}
	auto t0 = std::chrono::steady_clock::now();
	vector<double> par = P.init;
	vector<Aff> L, W;
	if (P.check) {
		std::mt19937 rng(12345);
		std::uniform_real_distribution<double> U(-0.05, 0.05);
		for (int f = 0; f < P.nfree; f++)
			par[P.fslot[f] * 9 + P.fk[f]] += U(rng);
		Lin lin;
		linearize(P, par, lin, L, W);
		double num = 0, den = 0, maxrel = 0;
		int nf = P.nfree;
		vector<double> gf(nf);
		for (int f = 0; f < nf; f++) {
			int idx = P.fslot[f] * 9 + P.fk[f];
			double h = 1e-6, keep = par[idx];
			par[idx] = keep + h;
			double cp = cost_of(P, par, L, W);
			par[idx] = keep - h;
			double cm = cost_of(P, par, L, W);
			par[idx] = keep;
			gf[f] = (cp - cm) / (2 * h);
		}
		double gscale = 0;
		for (int f = 0; f < nf; f++)
			gscale = std::max(gscale, std::fabs(gf[f]));
		for (int f = 0; f < nf; f++) {
			num += (lin.g[f] - gf[f]) * (lin.g[f] - gf[f]);
			den += gf[f] * gf[f];
			maxrel = std::max(maxrel, std::fabs(lin.g[f] - gf[f]) / std::max(std::fabs(gf[f]), 1e-6 * gscale));
		}
		printf("CHECK params=%d gradient relative error (norm) %.3e, worst coordinate relative error %.3e (floor 1e-6 of the largest component), largest |g| %.3e\n", nf, std::sqrt(num / std::max(den, 1e-300)), maxrel, gscale);
		write_output(argv[2], P, par, 0, 0, lin.cost, lin.cost);
		return 0;
	}
	Lin lin;
	linearize(P, par, lin, L, W);
	double c0 = lin.cost;
	double lam = P.lam0;
	int iters = 0, evals = 1, small = 0;
	printf("start cost %.9g, %d free parameters in %d joints\n", c0, P.nfree, P.nvar);
	for (int it = 0; it < P.maxit; it++) {
		bool accepted = false;
		for (int tries = 0; tries < 30; tries++) {
			vector<double> A = lin.H;
			for (int i = 0; i < P.nfree; i++)
				A[(size_t)i * P.nfree + i] += lam * std::max(lin.H[(size_t)i * P.nfree + i], 1e-12);
			vector<double> neg(P.nfree), d;
			for (int i = 0; i < P.nfree; i++)
				neg[i] = -lin.g[i];
			if (!cholesky_solve(A, neg, P.nfree, d)) {
				lam *= 10;
				continue;
			}
			vector<double> q = par;
			for (int f = 0; f < P.nfree; f++)
				q[P.fslot[f] * 9 + P.fk[f]] += d[f];
			double c2 = cost_of(P, q, L, W);
			evals++;
			if (c2 < lin.cost) {
				double rel = (lin.cost - c2) / lin.cost;
				par = q;
				lam = std::max(lam * 0.3, 1e-12);
				accepted = true;
				iters++;
				printf("it %d cost %.9g rel %.3e lambda %.2e\n", iters, c2, rel, lam);
				small = rel < P.tol ? small + 1 : 0;
				linearize(P, par, lin, L, W);
				break;
			}
			lam *= 4;
		}
		if (!accepted || small >= 2)
			break;
	}
	double secs = std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
	printf("done: %d iterations, %d cost evaluations, cost %.9g -> %.9g, %.2f s\n", iters, evals, c0, lin.cost, secs);
	write_output(argv[2], P, par, iters, evals, c0, lin.cost);
	return 0;
}
