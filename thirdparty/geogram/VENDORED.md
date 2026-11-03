# Vendored geogram

- Upstream: https://github.com/BrunoLevy/geogram
- Release: v1.9.9
- Commit: c40b4653e34519fffaeb82ca640bdd9c9778e324 (the same commit the engine's thirdparty/geogram was vendored from)
- Snapshot taken with `git archive` on 2026-11-03. Unpatched: the engine's copy has the no-exceptions patches, and those break
  files the engine never compiles, so this tool builds the unpatched sources.
- Licence: BSD-3 (see LICENSE).
- Used by: `geo_intersect.cpp` (repair + exact surface intersection of garment meshes).
- To update: re-run `git archive <new commit>` into this folder, change the commit above, and rebuild the tool.

## Submodules (empty in `git archive`, fetched at the commits upstream pins; all under src/lib/geogram/third_party/)

| Path | Upstream | Commit |
|---|---|---|
| OpenNL | https://github.com/BrunoLevy/OpenNL.git | ae782db2db40cc40fcd7659629fc652e5eab1f9e |
| amgcl | https://github.com/ddemidov/amgcl.git | 93827a00fc926d951c75f08fdb1d491912ff7065 |
| libMeshb | https://github.com/LoicMarechal/libMeshb.git | 88095d5ed04bfaf6bee27b777e7190bebdc72230 |
| rply | https://github.com/diegonehab/rply.git | 4296cc91b5c8c26d4e7d7aac0cee2b194ffc5800 |

The graphics submodules (glfw, imgui, imoguizmo) are not vendored: the tool builds the library only.
