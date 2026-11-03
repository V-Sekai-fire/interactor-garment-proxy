// Geogram repair + exact surface intersection on an OBJ (open meshes are fine).
// Welds duplicate vertices, then MeshSurfaceIntersection::intersect() splits every crossing exactly.
// It does NOT move vertices, and it does not call classify / remove_*_shell (those use a flood fill; blocklisted).
// usage: geo_intersect in.obj out.obj [colocate_epsilon=0]
#include <geogram/basic/common.h>
#include <geogram/basic/command_line.h>
#include <geogram/mesh/mesh.h>
#include <geogram/mesh/mesh_io.h>
#include <geogram/mesh/mesh_repair.h>
#include <geogram/mesh/mesh_surface_intersection.h>
#include <cstdio>
#include <cstdlib>

int main(int argc, char** argv) {
    if (argc < 3) { std::fprintf(stderr, "usage: geo_intersect in.obj out.obj [eps]\n"); return 2; }
    GEO::initialize();
    GEO::Mesh M;
    if (!GEO::mesh_load(argv[1], M)) { std::fprintf(stderr, "cannot load %s\n", argv[1]); return 1; }
    double eps = argc > 3 ? std::atof(argv[3]) : 0.0;
    std::printf("in:  %u vertices, %u facets\n", (unsigned)M.vertices.nb(), (unsigned)M.facets.nb());
    GEO::mesh_repair(M, (GEO::MeshRepairMode)(GEO::MESH_REPAIR_COLOCATE | GEO::MESH_REPAIR_DUP_F), eps);
    std::printf("after repair: %u vertices, %u facets\n", (unsigned)M.vertices.nb(), (unsigned)M.facets.nb());
    GEO::MeshSurfaceIntersection I(M);
    I.set_verbose(false);
    I.intersect();
    std::printf("after intersect: %u vertices, %u facets\n", (unsigned)M.vertices.nb(), (unsigned)M.facets.nb());
    GEO::MeshIOFlags flags;
    if (!GEO::mesh_save(M, argv[2], flags)) { std::fprintf(stderr, "cannot save %s\n", argv[2]); return 1; }
    return 0;
}
