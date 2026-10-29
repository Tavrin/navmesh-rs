# Corpus

Inputs for the differential tests. `MANIFEST.toml` lists every mesh with source, licence,
SHA-256 and feature tags; `generated.sha256` holds the hashes of the procedural meshes.
No mesh is committed; the repository holds the manifest, the hash list and the script.
`fetch.sh` downloads the upstream and third-party meshes at pinned commits into
`$NAV_CORPUS_DIR` (a directory outside the repository), generates the procedural ones with the
oracle's `gen` command, and checks every hash. Case files name meshes relative to the same
directory.

```sh
NAV_CORPUS_DIR=<dir> reference/corpus/fetch.sh path/to/nav_oracle
```

## Sources and licences

- **recastnavigation demo meshes** (`nav_test`, `dungeon`, `undulating`). They live in the
  zlib-licensed upstream repository without a separate notice, so we treat them as covered by its
  `License.txt` (copyright Mikko Mononen). Provenance from upstream history: `nav_test` and
  `dungeon` were added by Mikko Mononen in 2009, `undulating` by Jakob Botsch Nielsen in 2019 ("a
  mesh for testing crowds at a large distance of origin"). If this reading is ever questioned, the
  differential suite still runs without them on the generated and third-party meshes.
- **Godot demo projects** navigation level: MIT, attribution in the manifest.
- **Generated meshes:** ours. The generator uses only integer hashing and exact constants, so the
  OBJ text is the same on every platform; a generator change shows up as a hash mismatch.

Rejected: Crytek Sponza (CRYENGINE licence agreement, not permissive) and GPL game maps.
Candidate: Amazon Lumberyard Bistro (CC BY 4.0) as a large real level once a pinned conversion
step exists.

## Feature coverage

| Feature | Meshes |
|---|---|
| Stairs and the climb threshold | `gen_stairs`, `nav_test` |
| Slopes around the 45° limit | `gen_slopes`, `nav_test` |
| Multiple levels, bridges, overhangs | `gen_multilevel`, `nav_test`, `godot_nav_level` |
| Narrow corridors versus agent radius | `gen_corridors`, `dungeon` |
| Islands and off-mesh links | `gen_islands` (two links in its case file) |
| Tile seams | `gen_seams` (features on exact 32-cell boundaries), every `*_tiled` case |
| Large open world | `gen_terrain_large` (1024 m, 2.1M triangles) |
| Far from the origin | `undulating` (offset by 5 km) |
| Crowds | `nav_test`, `dungeon`, `undulating`, `gen_terrain_small`, `gen_multilevel`, `godot_nav_level`, `gen_seams` (tile cache) |
| Convex area volumes, filtered queries | `nav_test` (`nav_test_areas` case), `dungeon` (`dungeon_tilecache` case) |
| Tile cache and obstacle scripts | `gen_seams`, `dungeon` (cylinder, box and oriented-box obstacles, removals) |
| Degenerate triangles | `gen_degenerate` (zero-area, duplicated, slivers, sub-cell, downward-facing, zero-thickness wall) |

Not covered: NaN/Inf vertices, huge coordinates and empty meshes. Upstream behaviour on them is
undefined (float-to-int conversion of NaN grid sizes, allocation failures), so they belong to the
port's fuzz corpus, where the expected verdict is a typed error, not to the oracle records.
