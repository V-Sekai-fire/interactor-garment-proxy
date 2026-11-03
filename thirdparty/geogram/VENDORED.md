# Vendored geogram

- Upstream: https://github.com/BrunoLevy/geogram
- Release: v1.9.9
- Commit: c40b4653e34519fffaeb82ca640bdd9c9778e324 (the same commit the engine's thirdparty/geogram was vendored from)
- Snapshot taken with `git archive` on 2026-11-03. Unpatched: the engine's copy has the no-exceptions patches, and those break
  files the engine never compiles, so this tool builds the unpatched sources.
- Licence: BSD-3 (see LICENSE).
- Used by: `geo_intersect.cpp` (repair + exact surface intersection of garment meshes).
- To update: re-run `git archive <new commit>` into this folder, change the commit above, and rebuild the tool.
