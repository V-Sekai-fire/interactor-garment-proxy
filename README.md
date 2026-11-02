# interactor-garment-proxy

Prototypes and tools for repairing a garment as a proxy: the level surface of the unsigned
distance to the garment, so open edges close and layers closer than the distance merge. The
decision and its reasons are in RFD66606-2435. The cloth-fit solve fits the proxy, and the
fitted displacement returns to the original garment, which keeps its UVs, skin weights and
materials.

## Status

Everything in `prototype/` is Python with numpy. It exists to measure and is retired when the
steps move to GDScript or SafeGDScript, the language of the finished pipeline. Nothing here is
a result on its own: read `MEASUREMENTS.md` for what was measured and what failed.

## Layout

- `prototype/dfp.py`: the distance field and two surface extractors (marching tetrahedra and
  surface nets), with a variant switch.
- `prototype/df_proxy.py`: the first end-to-end proxy command, the one the measurements used.
- `prototype/measure_proxy.py`, `prototype/shells_cross.py`: shell count, edge use and
  edge-against-triangle crossings, each with a planted control (`planted.py`).
- `prototype/frontier.py`: the frontier-learning run that scores extractor variants on six
  input families (RFD66606-2309 describes the method).
- `prototype/sheet_data.py`, `prototype/to_bin.py`, `prototype/coincident_proxy.py`: inputs
  for the contact sheets and a count of vertices that share a position.
- `godot/contact_sheet.gd`, `godot/contact_sheet2.gd`: orbit-view contact sheets, cameras from
  `sphere_hammersley_sequence`, rendered by SubViewports (RFD66606-2370). A control that points
  a camera away must fail.
- `godot/csg_union.gd`: the union of thickened shells that did not work.
- `godot/fit_sheet.gd`: contact sheet of a cloth-fit result, input against fitted, with each mesh
  textured by its own ArUco board on real UVs; the window shows every render as it is made.
- `godot/explode_host.gd`: runs model_explode's `explode.gd` on OBJ files and renders the layout.
- `godot/csg_clip.gd`: body INTERSECT garment with Godot's Manifold CSG; empty means no clipping, and
  a planted `--shift` must make the overlap grow.
- `prototype/make_suit_board.py`: the real SOMA suit board (`CharucoBoard((49, 65), 0.03, 0.022)`, `DICT_4X4_1000`, ids
  wrapped mod 1000, 1592 markers), with gates (every marker reads back, blank sheet reads none). A marker is told apart
  by its board position (`idx`); `id = idx % 1000`, `instance = idx // 1000`.
- `prototype/decals.py`: puts that board on a mesh. `--uvmap` maps it onto xatlas UVs (repeating the sheet across a
  larger atlas, flipping mirrored charts, reporting seams, shear, size and square markers); the default mode builds
  one geodesic decal per marker. `YUP=1` for Y-up meshes. `crease_split.py`, `panels_material.py` and
  `godot/unwrap_tiles.gd`, `godot/unwrap_cyl.gd` are the unwrap experiments; `tools/xatlas_cli.cpp` runs upstream xatlas
  with every chart and pack option.
- `prototype/cameras.py`, `godot/fit_decals.gd`: a studio-style camera plan (the fewest cameras that look at every
  marker head-on, 2-4 views each) and a resumable renderer with a UV pass that gives exact ground truth (written next
  to the RGB, never part of a training set).
- `prototype/verify_ids.py`, `prototype/recall_ids.py`: readback with ground truth (a marker is right only if it sits
  where its id belongs, with a planted neighbour-id control) and the paper's measure: the share of all markers read in
  2 or more cameras (so they can be triangulated). `diagnose_unread.py` and `detector_battery.py` explain misses.
- `prototype/texture_markers.py`: paints the markers into the texture of UVs you already have so that no marker is clipped by a seam (per island, the grid offset that fits the most whole markers); gates first (every marker square in one island, no overlap, unique ids, the clean atlas reads back 100%). `prototype/tagfamily.py` loads any OpenCV dictionary or a custom family made by `lean/MarkerDetect` (`tagfamily_gen`, a pass-fail tag family generator).
- `prototype/rfdetr_dataset.py`: exact keypoint labels (4 corners per marker, visibility flags, polygons) for RF-DETR.
- `tools/anny_export.py`: exports the baked ANNY-soma-x body from `solve/core/soma_rig.h` (18,056 vertices, 78 joints)
  with gates and a planted control.

Meshes are not stored here: the garments are exported from the character project, and the
proxy is 4.8 million triangles.

## Licence

MIT. See [LICENSE](LICENSE).
