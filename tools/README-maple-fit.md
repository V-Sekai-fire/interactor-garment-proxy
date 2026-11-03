# maple_fit.exs: outfit piece to cloth-fit to fitted bones

One Elixir driver, `tools/maple_fit.exs`, runs the pipeline that fits an outfit piece (the Maple Skirt, the Maple Kimono)
onto the Miroir body and then fits the outfit's own bones so the existing skinning reproduces the result. The driver only
orchestrates. Each stage is a small program it starts in the background with a log and polls for its exit status
(GDScript run headless in the double engine, PolyFEM, `remesh.elf` through the Sandbox host, `bone_fit.gd`). No Python anywhere.
Outputs are OpenUSD (`.usdz`, `.usda`); the OBJ and `.bin` files in the work directory are scratch interchange for PolyFEM and the exporter format.

## Run

    elixir tools/maple_fit.exs --piece BKimono [--work DIR] [--config FILE] [--from STAGE] [--only STAGE] [--force] [--adopt RUN_DIR] [--list]

- `--piece` a key of `pieces` in the config (`BKimono`, `ASkirt`).
- `--work` work directory (default `work/` in the repository); results land in `<work>/<piece>/<stage>/`.
- `--from STAGE` runs that stage and every later one; `--only STAGE` runs one; `--force` ignores the cache.
- `--adopt RUN_DIR` use a finished PolyFEM run instead of running one, but only if its assets and `setup.json` are identical to the ones stage 3 made (otherwise the cloth-fit gate stops the run).
- The driver stops at the first failed gate with `STOP <stage>: <reason>` and the log path, and exits 1. It prints a per-stage table at the end (status, seconds, `run` or `cached`).
- A foreground command is killed after about 9 minutes by the tool harness, the PolyFEM solve takes longer. The solve is started detached (`nohup` style, pid and exit status files next to its log). If the driver is killed while waiting, run it again: stage 4 sees the live pid and resumes waiting.

## Stages

| # | Stage | Program | Gates (the run stops if any fails) |
|---|---|---|---|
| 0 | `inputs` | driver | exporter files, FBX, prefab and tools exist |
| 1 | `csg-clean` | `godot/csg_clean_usd.gd` | `.usdz` read back bit for bit; planted one-ulp change caught; Manifold did not refuse the union |
| 2 | `proxies` | `godot/union_proxy.gd` | CSG union of shells thickened inward (garment from the cleaned `.usdz`, avatar from `avatar.obj`); union non-empty; surface components not visible from outside (air bubbles) dropped |
| 3 | `reduce` | `run_remesh.gd` (remesh.elf), `to_solver.gd`, `fix_smooth.gd`, `jitter.gd`, `shell_pick.gd`, `nofit.gd`, `mesh_check.gd` | per proxy: 0 open edges, positive signed volume, 0 crossing pairs (exact predicates, no welding); avatar vs garment: 0 crossings; a planted shift of the garment must be detected; drop list matches the shell count |
| 4 | `cloth-fit` | `PolyFEM_bin` | exit 0 and `Garment retargeting succeeded!` in the log. For a piece with `external_cloth_fit` the earlier run is used instead |
| 5 | `map-back` | `godot/map_back.gd` | solver frame fit residual below 1e-9; planted control (proxy vertices as queries) returns their own displacement to 1e-6 m. Prints the left/right mirror symmetry of the displacement and the query-to-proxy distance percentiles |
| 6 | `bone-fit` | `godot/bone_fit.gd` | the tool's own checks (bake reproduction, planted control, OpenUSD read-back, "no Miroir bone moves", "no bone another piece weights moves"): any `FAIL` stops the run; optional `expect` reproduction gate |
| 7 | `report` | driver | writes `<work>/<piece>/report/REPORT-auto.md` from the manifests |

Every stage writes `manifest.term` and `manifest.txt` (input hashes, parameters, tool hashes, seconds, numbers, outputs) and is
skipped when the manifest matches the current inputs and parameters and its outputs exist.

## Config

`tools/config/maple.exs` is one map. Everything the stages use is there: tool paths, export directory, FBX and prefab, solver scale (5), CSG thickness, remesh resolutions
and triangle count, jitter, smoothing iterations, the PolyFEM `setup.json` (identical to the Maple runs), map-back gates, bone-fit modes, and per piece the proxy settings.
Paths starting with `./` are relative to the repository root.

Bone-fit modes: `:exclusive` (default, the deliverable: only clothing bones that no other outfit piece weights may move, never a bone with a Miroir bone's name),
`:clothing_free` and `:analysis` (analysis only; `bone_fit.gd` refuses to write a clip for the analysis mode). The multi-piece form is not wired in (not present in `bone_fit.gd` at the time of writing).

## Tools

Binaries are not committed (`tools/bin/` is git-ignored). Copy or rebuild:

- `tools/bin/PolyFEM_bin`: built from `adapters/third-party/solve/core/cloth-fit` (`build_polyfem.sh` in the session scratchpad; see the `unity-garment-cloth-fit` skill, step 4).
- `tools/bin/remesh.elf`: `interactor-remesh` guest, double precision (`build_remesh.sh`, skill step 3b). Check the `.sandbox_variant` section is `28 00 00 00`.
- `tools/bin/usd.elf`: `usd.post-blendshape.elf` (sha256 prefix 1729bed3), the OpenUSD guest.
- the engine: `godot.macos.editor.double.arm64` from `engine-r4-party` (Sandbox class, CSG vertex attributes), path in the config.

## Add a piece

1. Export from the open Unity editor (manual menu action, see below). The exporter writes `garment_<piece>.obj` and `garment_own_<piece>.bin` into the export directory.
2. Add an entry under `pieces` in the config: `mesh` (the FBX mesh name), `same_piece` (variants of the same garment that must not count as other pieces), thicknesses,
   `garment_res`/`avatar_res`/`target_triangles`, `nofit` (`{:below_y, metres}` or `{:height_fraction, f}`), `drop_shell_ranks` (start with `[]`).
3. Run `--piece <name>`. If PolyFEM answers `initial solution has intersections`, find the offending proxy shells (run the solver one shell at a time with `shell_pick.gd`, as was done for the kimono) and list their ranks in `drop_shell_ranks`.

## What is NOT automated

- The Unity export is a manual menu action (Chibifire > Characters > Export fit meshes, or the cua-driver `invoke_menu` call in the skill).
- Applying the result in Unity is not done: the delivered clip (`*_bones_clip_rt.usda`, rest frame 0 and fitted frame 1) and the report tell the operator what to apply; the Miroir is never touched.
- Which proxy shells the solver can start with (`drop_shell_ranks`) is found by hand.
- The raw welded CSG union has open and non-manifold edges where thin shells touch; the closed, crossing-free gate is applied after reduction, not on the raw union.
- The Miroir body blend shapes of the scene (`Body` 72 = 100, `BaseBody` 30 = 20) are not in the export; the fit targets the prefab body.
