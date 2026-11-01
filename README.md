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

Meshes are not stored here: the garments are exported from the character project, and the
proxy is 4.8 million triangles.

## Licence

MIT. See [LICENSE](LICENSE).
