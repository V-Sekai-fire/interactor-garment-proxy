# Measurements

Pure-Python scripts with planted controls, run on one outfit piece (the pants) and on the
cloth-fit solver's own example. Welded at one hundred-thousandth of the bounding box unless a
row says otherwise. A crossing is an edge that pierces a triangle sharing no vertex with it.

## What the solver takes, and what it gets

| mesh | welded vertices | triangles | shells | non-manifold edges | open edges | crossing pairs |
| --- | --- | --- | --- | --- | --- | --- |
| solver example garment | 2682 | 5220 | 1 | 0 | 144 | 0 |
| solver example avatar | 5095 | 10171 | 32 | 75 | 4 | 1623 (229 inside a shell, 1394 across) |
| pants as exported | 4014 | 7543 | 53 | 0 | 399 | 1316 (16 inside a shell, 1300 across) |

## Union of thickened shells (stock CSG), did not work

Each shell becomes a solid: an inner copy offset along the vertex normals, joined by a strip on
every open edge. At 4 mm, 24 of 53 solids came out inside out; at 2 mm, 14; at 1 mm and 0.5 mm,
none. The union at 1 mm has 25904 triangles, 29 shells, 295 non-manifold edges and 30969
crossing pairs. At 0.5 mm it has 25076 triangles, 46 shells, 120 non-manifold edges and 30222
crossing pairs.

## Level surface of the unsigned distance, pants, 2 mm on a 1 mm grid

- Grid 59.8 million cells, distance field in 7.1 s.
- Marching tetrahedra: 2416090 vertices, 4832384 triangles, 0 open edges, 0 non-manifold
  edges, 71 shells (the largest holds 4812960 triangles).
- 2469 vertices share a position (to 1e-6 m). 5822 triangles have zero area, 3528 of them with
  two corners at one position.
- Crossings in four sampled boxes of 4 cm: 20 pairs. After welding and dropping those
  triangles: 0 pairs in the same boxes, but 4628 open edges and 1 non-manifold edge.
- Setting field values of exactly zero to a small number changed nothing: 5824 zero-area
  triangles, 20 sampled crossing pairs.

## Frontier run, extractor variants (RFD66606-2309 method)

Six input families, three problems each, scored on closed surface, distinct vertices,
non-zero areas, shell count and no crossings.

- `base` (marching tetrahedra): 0 of 18 in 552 s. Distinct vertices and non-zero areas fail on
  all 18, crossings on 1 (the open tube). Closed surface and shell count pass on all 18.
- `planted` (one triangle deleted), `nets` (surface nets) and `nets_centre` (the same faces
  with each vertex at its cell centre): not yet read.
