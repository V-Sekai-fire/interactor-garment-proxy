// Command-line xatlas for experiments: OBJ in, OBJ with UVs out, every ChartOptions / PackOptions field settable.
//   xatlas_cli <in.obj> <out.obj> [scale=1] [materials=<file>] [key=value ...]
//   scale      multiplies positions (e.g. 0.2 to turn 0.2 m units into metres); output positions are in the scaled units
//   materials  one integer per face; only faces with the same value share a chart (xatlas MeshDecl::faceMaterialData),
//              so crease-bounded panels get their seams exactly where the panel borders are
//   ChartOptions keys: maxChartArea maxBoundaryLength normalDeviationWeight roundnessWeight straightnessWeight
//              normalSeamWeight textureSeamWeight maxCost maxIterations
//   PackOptions keys:  texelsPerUnit resolution padding bruteForce(0/1) blockAlign(0/1) bilinear(0/1)
// Writes out.obj (v, vt normalised to the board, f) and out.obj.json: [{tile, atlas_w, atlas_h, board_w, board_h, ...}]
// plus out.obj.charts.json: chart index per input face. board = atlas size rounded up to whole 45 px squares.
#include "xatlas.h"
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <map>
#include <string>
#include <vector>
int main(int argc, char **argv) {
	if (argc < 3) { fprintf(stderr, "usage: xatlas_cli in.obj out.obj [scale=..] [materials=file] [key=value ...]\n"); return 2; }
	std::map<std::string, double> opt; std::string matFile;
	for (int i = 3; i < argc; i++) {
		const char *eq = strchr(argv[i], '='); if (!eq) continue;
		std::string k(argv[i], eq - argv[i]); if (k == "materials") matFile = eq + 1; else opt[k] = atof(eq + 1);
	}
	double scale = opt.count("scale") ? opt["scale"] : 1.0;
	std::vector<float> pos; std::vector<uint32_t> idx; char line[4096];
	FILE *f = fopen(argv[1], "r"); if (!f) { perror("open"); return 1; }
	while (fgets(line, sizeof line, f)) {
		if (line[0] == 'v' && line[1] == ' ') { double x, y, z; sscanf(line + 2, "%lf %lf %lf", &x, &y, &z); pos.push_back((float)(x * scale)); pos.push_back((float)(y * scale)); pos.push_back((float)(z * scale)); }
		else if (line[0] == 'f' && line[1] == ' ') { char *p = line + 2; int v[3]; for (int k = 0; k < 3; k++) { v[k] = (int)strtol(p, &p, 10); while (*p && *p != ' ' && *p != '\n') p++; } idx.push_back(v[0] - 1); idx.push_back(v[1] - 1); idx.push_back(v[2] - 1); }
	}
	fclose(f);
	uint32_t nv = (uint32_t)(pos.size() / 3), nf = (uint32_t)(idx.size() / 3);
	std::vector<uint32_t> mat;
	if (!matFile.empty()) { FILE *m = fopen(matFile.c_str(), "r"); if (!m) { perror("materials"); return 1; } int v; while (fscanf(m, "%d", &v) == 1) mat.push_back((uint32_t)v); fclose(m); if (mat.size() != nf) { fprintf(stderr, "materials has %zu entries, mesh has %u faces\n", mat.size(), nf); return 1; } }
	xatlas::Atlas *atlas = xatlas::Create();
	xatlas::MeshDecl d; d.vertexPositionData = pos.data(); d.vertexPositionStride = sizeof(float) * 3; d.vertexCount = nv;
	d.indexData = idx.data(); d.indexCount = (uint32_t)idx.size(); d.indexFormat = xatlas::IndexFormat::UInt32;
	if (!mat.empty()) d.faceMaterialData = mat.data();
	if (xatlas::AddMesh(atlas, d) != xatlas::AddMeshError::Success) { fprintf(stderr, "AddMesh failed\n"); return 1; }
	xatlas::ChartOptions co;
#define CO(k) if (opt.count(#k)) co.k = (float)opt[#k];
	CO(maxChartArea) CO(maxBoundaryLength) CO(normalDeviationWeight) CO(roundnessWeight) CO(straightnessWeight) CO(normalSeamWeight) CO(textureSeamWeight) CO(maxCost)
	if (opt.count("maxIterations")) co.maxIterations = (uint32_t)opt["maxIterations"];
	xatlas::ComputeCharts(atlas, co);
	xatlas::PackOptions po; po.texelsPerUnit = opt.count("texelsPerUnit") ? (float)opt["texelsPerUnit"] : 1500.0f; po.createImage = false;
	if (opt.count("resolution")) po.resolution = (uint32_t)opt["resolution"];
	if (opt.count("padding")) po.padding = (uint32_t)opt["padding"];
	if (opt.count("bruteForce")) po.bruteForce = opt["bruteForce"] != 0; if (opt.count("blockAlign")) po.blockAlign = opt["blockAlign"] != 0; if (opt.count("bilinear")) po.bilinear = opt["bilinear"] != 0;
	xatlas::PackCharts(atlas, po);
	const xatlas::Mesh &m = atlas->meshes[0];
	// With resolution set, xatlas fills as many square sheets of that size as it needs; sheets are laid side by side (sheet n at x = n * resolution),
	// so with resolution = the printed board's width each sheet is one instance of the board (ids repeat once per extra sheet).
	const bool sheets = atlas->atlasCount > 0; const uint32_t res = sheets ? (uint32_t)opt["resolution"] : 0;
	int bw = sheets ? (int)(res * atlas->atlasCount) : (int)std::ceil(atlas->width / 45.0) * 45, bh = sheets ? (int)res : (int)std::ceil(atlas->height / 45.0) * 45;
	FILE *o = fopen(argv[2], "w");
	for (uint32_t i = 0; i < m.vertexCount; i++) { const float *p = &pos[m.vertexArray[i].xref * 3]; fprintf(o, "v %.6f %.6f %.6f\n", p[0], p[1], p[2]); }
	for (uint32_t i = 0; i < m.vertexCount; i++) { float ux = m.vertexArray[i].uv[0] + (sheets && m.vertexArray[i].atlasIndex > 0 ? (float)m.vertexArray[i].atlasIndex * res : 0.0f); fprintf(o, "vt %.6f %.6f\n", ux / bw, m.vertexArray[i].uv[1] / bh); }
	for (uint32_t t = 0; t < m.indexCount / 3; t++) fprintf(o, "f %u/%u %u/%u %u/%u\n", m.indexArray[t * 3] + 1, m.indexArray[t * 3] + 1, m.indexArray[t * 3 + 1] + 1, m.indexArray[t * 3 + 1] + 1, m.indexArray[t * 3 + 2] + 1, m.indexArray[t * 3 + 2] + 1);
	fclose(o);
	std::string js = std::string(argv[2]) + ".json"; FILE *j = fopen(js.c_str(), "w");
	fprintf(j, "[{\"tile\": 0, \"atlas_w\": %u, \"atlas_h\": %u, \"board_w\": %d, \"board_h\": %d, \"charts\": %u, \"texelsPerUnit\": %.3f}]\n", atlas->width, atlas->height, bw, bh, atlas->chartCount, atlas->texelsPerUnit); fclose(j);
	// chart index per output triangle (a triangle's chart is its first vertex's chart)
	std::string cj = std::string(argv[2]) + ".charts.json"; FILE *c = fopen(cj.c_str(), "w"); fprintf(c, "[");
	for (uint32_t t = 0; t < m.indexCount / 3; t++) fprintf(c, "%s%d", t ? "," : "", m.vertexArray[m.indexArray[t * 3]].chartIndex); fprintf(c, "]\n"); fclose(c);
	printf("sheets %u, charts %u, atlas %u x %u, texelsPerUnit %.1f, vertices %u -> %u\n", atlas->atlasCount, atlas->chartCount, atlas->width, atlas->height, atlas->texelsPerUnit, nv, m.vertexCount);
	xatlas::Destroy(atlas); return 0;
}
