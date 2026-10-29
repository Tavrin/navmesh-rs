# Reference harness and records

The oracle is a small C++ driver (`harness/nav_oracle.cpp`) compiled together with the pinned
upstream sources. For each case it bakes a navmesh (solo, tiled, or through the tile cache), runs
a fixed query set under two filters, several crowd simulations, a malformed-tile check and, for
tile-cache cases, an obstacle script. It prints one JSON line per record (the record name, counts,
the byte length and an FNV-1a 64-bit digest of a canonical byte stream) and, with `--records
FILE`, writes every stream into one record file. `--dump DIR` also writes each stream to
`DIR/<record>.bin`. The Rust port writes the same streams through the same encoder rules; the
comparator in `parity/` (`nav-parity`) reads two record files and reports, per record type,
which records are equal, and for each mismatch the first differing field.

FNV-1a is an equality check over our own outputs, not a security property. If a stronger digest is
wanted later, swap it in both encoders; the streams do not change.

## Build and run

```sh
export NAV_UPSTREAM_DIR=<recastnavigation checkout at the pin>
export NAV_CORPUS_DIR=<directory for the corpus, outside the repository>
cmake -S reference/harness -B build/oracle -G Ninja
ninja -C build/oracle
reference/corpus/fetch.sh build/oracle/nav_oracle
NAV_ORACLE=build/oracle/nav_oracle reference/run-corpus.sh out/run1
NAV_ORACLE=build/oracle/nav_oracle reference/run-corpus.sh out/run2
cargo run -p navmesh-parity -- diff out/run1 out/run2
# the Rust port over one case, compared on the Recast stages only
cargo run --release -p navmesh-parity --bin nav-port -- bake reference/cases/dungeon_solo.case --records out/port.navrec
cargo run -p navmesh-parity -- diff --recast out/port.navrec out/run1/dungeon_solo.navrec
```

The build produces two oracles from the same sources: `nav_oracle` with upstream's default 32-bit
poly refs and `nav_oracle64` with `DT_POLYREF64`. They differ only in the width of the refs in
query, crowd and malformed-tile records and of the links inside tile bytes.

CMake refuses a checkout that is not at the pinned commit or has local modifications
(`-DNAV_ALLOW_UNPINNED=ON` overrides it; such records are a different oracle). The commit, the
compiler version, the flags and the libc version go into every record file's metadata.

Commands: `bake <case> [--records FILE] [--dump DIR]` writes records; `bench <case> --repeat N
[--threads T] [--query-repeat Q]` bakes N times without records and prints min/median wall time;
`gen <kind> <out.obj>` writes a procedural corpus mesh.

Oracle build contract: x86-64 baseline (no `-march`), `-O2 -ffp-contract=off -fno-fast-math`,
glibc ≥ 2.39 (stable merge-sort `qsort`; see `UPSTREAM.md`, determinism item 3). Records produced
under another compiler, flag set or libc are a different oracle and are not mixed with these; the
comparator rejects two oracle files whose compiler, flags or libc differ.

## Case files

`key = value` lines, `#` comments, unknown keys rejected. Keys: `mesh` (relative to
`$NAV_CORPUS_DIR` unless absolute), `mode` (`solo` | `tiled` | `tilecache`), `partition`
(`watershed` | `monotone` | `layers`; not used by `tilecache`), the demo build settings
(`cell_size`, `cell_height`, `agent_height`, `agent_radius`, `agent_max_climb`,
`agent_max_slope`, `region_min_size`, `region_merge_size`, `edge_max_len`, `edge_max_error`,
`verts_per_poly`, `detail_sample_dist`, `detail_sample_max_error`, `tile_size`), the three filter
switches, `queries`, `seed`, `crowd_agents`, `crowd_ticks`, and three repeatable keys:

- `off_mesh = x0 y0 z0 x1 y1 z1 radius bidirectional`
- `area_volume = area hmin hmax x0 z0 x1 z1 x2 z2 ...`: a convex volume marked with
  `rcMarkConvexPolyArea` after erosion (area 1..62), as the demo does.
- `tc_obstacle = cylinder x y z radius height | box x0 y0 z0 x1 y1 z1 | obox cx cy cz hx hy hz
  yaw | remove N | update | update_once`: the tile-cache obstacle script, run in order. `remove N` removes the
  N-th obstacle added (from 0); `update` calls `dtTileCache::update` until it reports up to date; `update_once`
  calls it exactly once, recording the intermediate state and up-to-date flag.

Defaults are the upstream demo defaults. Settings are converted to cell units exactly as the demo
does (`ceilf` height and radius, `floorf` climb, truncated edge length, squared region sizes).
Polygon areas: walkable becomes area 0; flags are 1 for area 0 and `1 | (1 << area)` for areas
1..15.

Tiled and tile-cache modes build every tile of the grid, row-major, with border
`walkableRadius + 3` cells, from the triangles whose xz bounds overlap the bordered tile bounds,
in input order. (The demo's chunky partition reorders triangles and is not used.) Tiles are added
to the navmesh row-major after all are built, so poly refs do not depend on build order or thread
count. Tile-cache mode follows the demo's temporary-obstacle sample (up to 32 layers per tile,
`maxObstacles` 128, the same navmesh parameter derivation) with two differences: layers are
stored uncompressed (a raw compressor, so the bytes do not depend on FastLZ), and the layer
header is zeroed before it is filled, because `dtBuildTileCacheLayer` copies the header struct
including its two padding bytes.

## Canonical encoding

Little-endian, no padding: `u8`, `u16`, `u32`, `i32`, and `f32` as its raw IEEE-754 bits. Arrays
are written as a count followed by elements, except where the count is a header field written
earlier in the same record (grid cells, compact spans, mesh vertices and polygons). Bit-field
members (`rcSpan::smin/smax/area`, `rcCompactCell::index/count`, `rcCompactSpan::con/h`) are
widened to the next listed integer type. Upstream navmesh tile bytes and tile-cache layer bytes
are written verbatim (they are themselves the format under test).

## Record file

One file per case (`<case>.navrec`), all integers little-endian:

| Part | Layout |
|---|---|
| magic | 8 bytes: `NAVREC`, `0x00`, format version `0x01` |
| metadata | `u32` length, then UTF-8 `key=value` lines: `producer`, `upstream` (commit), `compiler`, `flags`, `libc`, `case`, `polyref_bits` (32 or 64; poly and tile refs in records are written as `u32` or `u64`; obstacle refs stay `u32`). A file without `polyref_bits` holds no refs (the port's Recast-only files) or is 32-bit |
| each record | `u32` name length (> 0), name, `u64` payload length, payload, `u64` FNV-1a 64 of the payload |
| end | `u32` 0, then `u32` record count |

Records appear in the order the driver closes them, which is fixed by the driver code; the
comparator matches records by name, so order is not part of the contract. Names are unique per
file.

## Records

Per-tile records in tiled and tile-cache modes are prefixed `tile_X_Y.`; per-layer tile-cache
records are prefixed `layer_X_Y_L.`; the second query set is prefixed `filtered.`. A record's kind
is the part of its name after the last `.`. Query records start with a `u32` entry count (pairs,
or points for the first two) and hold one entry per pair; `status` is the raw `dtStatus`.

| Record | Contents |
|---|---|
| `geometry` | vertex count, vertices, triangle count, indices (after OBJ loading) |
| `config` | every `rcConfig` field used by the build |
| `tri_areas` | count, area id per input triangle after `rcMarkWalkableTriangles` |
| `heightfield_initial`, `heightfield_filtered` | grid header; per column: span count, then `smin`, `smax`, `area` per span, bottom to top |
| `compact_initial`, `compact_eroded`, `compact_marked` | grid header, cells (`index`, `count`), spans (`y`, `con`, `h`, area); `compact_marked` only when the case has area volumes |
| `distance_field` | as above plus `dist` per span and `maxDistance` (watershed only) |
| `regions` | as above plus `reg` per span and `maxRegions` |
| `contours` | per contour: region, area, simplified and raw vertices (4 ints each) |
| `poly_mesh` | vertices (u16×3), per polygon: vertex and neighbour slots, region, flags, area |
| `detail_mesh` | sub-meshes (u32×4), vertices (f32×3), triangles (u8×4) |
| `heightfield_layers` | tile cache: `rcBuildHeightfieldLayers` output, per layer bounds, extents, height range and the height, area and connection grids |
| `layer_data` | tile cache: the layer bytes added to the cache (`dtBuildTileCacheLayer` output) |
| `layer_grid` | tile cache: the decompressed layer (header fields, heights, areas, connections) |
| `layer_regions`, `layer_contours`, `layer_poly_mesh` | tile cache: `dtBuildTileCacheRegions`, `dtBuildTileCacheContours` and `dtBuildTileCachePolyMesh` output; the poly mesh after the area and flag mapping |
| `navmesh_tiles` | tile count; per tile in add order: tile x, y, layer, byte length, `dtCreateNavMeshData` bytes |
| `q_random_point` | `findRandomPoint` status, ref, point, for `queries + 1` points |
| `q_nearest_poly` | `findNearestPoly` from each point offset by (0.37, 1, -0.21), extents (2, 4, 2): status, ref, point, over-poly flag |
| `q_query_polygons` | `queryPolygons` over the same box: status, refs (max 64) |
| `q_find_path` | `findPath` between consecutive points: status, count, refs (max 256) |
| `q_sliced_path` | the same requests through the sliced API, 8 iterations per update; counts carry the mismatches against `q_find_path` |
| `q_partial_sliced_path` | one sliced update of 4 iterations, then `finalizeSlicedFindPathPartial` with the `findPath` corridor as the existing path: the three statuses, iterations done, refs |
| `q_straight_path`, `q_straight_path_all_crossings`, `q_straight_path_area_crossings` | `findStraightPath` with options 0, `DT_STRAIGHTPATH_ALL_CROSSINGS`, `DT_STRAIGHTPATH_AREA_CROSSINGS`: corners (point, flags, ref) |
| `q_raycast` | `raycast` with `DT_RAYCAST_USE_COSTS`: t, normal, edge, visited refs, cost |
| `q_raycast_plain` | the first `raycast` form: t, normal, visited refs |
| `q_distance_to_wall` | radius 5: distance, hit point, normal |
| `q_move_along_surface` | result point and visited refs (max 16) |
| `q_polys_around_circle` | radius 4 around the start point: ref, parent, cost per polygon (max 64) |
| `q_polys_around_shape` | a 4 m square around the start point: ref, parent, cost per polygon (max 64) |
| `q_local_neighbourhood` | radius 3: ref and parent per polygon (max 32) |
| `q_poly_wall_segments` | the start polygon's wall and portal segments with neighbour refs (max 64) |
| `q_closest_point_on_poly`, `q_closest_point_on_poly_boundary` | the pair's end point projected on the start polygon |
| `q_poly_height` | `getPolyHeight` on the start polygon at the start and the end point |
| `q_random_point_around_circle` | radius 3 around the start point |
| `crowd_trajectories` | tick count; per tick (dt = 1/30): tick, active count, per active agent: slot, state, target state, `npos`, `vel`, `nvel`, first corridor poly, corner count, corridor count and every corridor poly. Avoidance type 3 with `dtCrowd`'s default parameters |
| `crowd_quality_0` .. `crowd_quality_3` | the same with the demo's low, medium, good and high avoidance presets |
| `crowd_move_velocity` | odd agents switch to `requestMoveVelocity` toward their target at half speed; at T/2 every third agent gets `resetMoveTarget` |
| `crowd_remove_readd` | agents 1, 5, 9, ... removed at T/3; as many new agents added at 2T/3 |
| `crowd_tile_swap` | at T/2 the tile under agent 0 is removed and the same bytes added back (on a copy of the navmesh) |
| `malformed_tiles` | upstream verdicts on a fresh navmesh with the baked parameters: valid add, same location again, wrong magic, wrong version, more polygons than ref bits, remove of a null ref, remove twice, re-add into the freed slot with the stale ref |
| `tilecache_obstacles` | per script step: the op and its arguments; for adds and removes the status, obstacle ref and number of tiles touched; for `update`/`update_once` the iteration count, status, up-to-date flag, every non-empty obstacle slot (index, ref, state, type, salt) and, for every tile touched since the previous update, the navmesh-resident tile bytes (links included) |

After every tile-cache script `update` or `update_once`, the harness also records every query
family under both filters, prefixed `obstacle_update_N.` (N starts at 1).
The ordinary query layouts and exact comparison rules apply unchanged.

`reference/run-tilecache-parity.sh` accepts separately built 32/64-bit oracle and
Rust drivers, optional debug drivers, and compares complete records plus independent
release repeats on every tile-cache case. It never supplies oracle tile bytes to
the Rust tile-cache build.

Random points come from `findRandomPoint` driven by a fixed xorshift32 (`seed` in the case
file), so the query set is a function of the navmesh, the filter and the seed only; a navmesh
mismatch is reported at `navmesh_tiles` before it can surface as query noise. The `filtered.`
set uses a filter that excludes flag `0x10` (area 4) and gives areas 1..7 costs of
`1 + 1.5 × area`; on cases without area volumes it equals the plain set.

Malformed tile bytes whose upstream behaviour is undefined (truncated buffers, out-of-range
indices, oversized link counts) are not recorded: upstream reads out of bounds on them. They
belong to the port's fuzz targets, where the verdict is a typed error.

## What must be exact, and what gets a tolerance

| Surface | Contract | Why |
|---|---|---|
| Geometry, config, triangle areas | bit-exact | pure input handling; `cosf(slope)` is one value, recorded via the areas it produces |
| Heightfield through detail mesh, tile-cache layers | bit-exact | integer grid algorithms plus f32 arithmetic in a fixed order; with no FMA contraction and stable sorting the Rust port can reproduce every bit |
| Navmesh tile bytes | byte-exact | the format is the compatibility promise for existing upstream data |
| Detour queries (all listed), malformed-tile verdicts | bit-exact | f32 arithmetic and `sqrtf` only, fixed iteration and heap rules |
| Crowd trajectories | bit-exact on the oracle platform (x86-64, glibc libm); under the cross-platform `libm` contract, a stated tolerance | obstacle avoidance uses libm transcendentals whose last bit differs between libms; once one sample direction differs, trajectories diverge chaotically. The port uses the platform libm by default, which on the oracle platform is the same glibc, so exact parity is expected there. Under the `libm`-crate feature the rule is: the first 30 ticks bit-exact, then the same agents active each tick, identical agent and target states, and each agent's position within one cell size (taken from the first `config` record) of the reference |
| Tile cache after obstacle scripts | byte-exact navmesh tiles | integer layer algorithms; `cosf`/`sinf` only for oriented boxes, computed once per obstacle (same libm rule as the crowd) |
| Timing, memory | not compared | measured separately (`nav_oracle bench`, `nav-port bench`) |

Crowd exactness applies to valid inputs in the domain documented under
"Deliberate differences from C++" in `docs/VERIFICATION.md`. In particular, adaptive
sampling with even division counts reads uninitialized storage upstream and has
no C++ parity claim; all recorded presets use odd divisions.

A tolerance is never introduced to make a failing record pass. If a bit-exact record differs,
the port is wrong until a decoded dump shows the cause is one of the listed libm sites.

## Comparator

`nav-parity diff A B` compares two record files, or two directories of `*.navrec` files paired by
name. Every record must exist in both. Equal payloads pass; for a mismatch the decoder for the
record's kind walks both payloads and names the field that holds the first differing byte (for
example `tiles[3].data.polys[17].neis[2]: 5 vs 6`). Every payload is decoded even when both sides
agree (`--no-validate` skips that), so a payload that does not match its kind's layout fails.
`--crowd-tolerance` applies the crowd rule above to `crowd_*` records. `--recast` compares only
the Recast record kinds (`geometry` through `detail_mesh`, and `heightfield_layers`), so a port
file holding those stages can be compared with a full oracle file. `--crowd` selects only the eight crowd trajectory kinds, including their full corridors.
`--detour` adds
`navmesh_tiles`, `malformed_tiles` and every `q_*` kind (the Recast and Detour phases, without
crowd, tile-cache layer and obstacle records). Two files that both state
`polyref_bits` may state different widths: each file is decoded at its own width (absent: 32) and
records that carry poly refs are compared by decoded value, while every other record compares
byte for byte. Metadata must agree on the
upstream commit and the case; two oracle files must also agree on compiler, flags and libc. Exit
status: 0 when everything matches, 1 otherwise, 2 on usage or I/O errors. `nav-parity check`
decodes every record of one or more files; `nav-parity list` prints a file's records.

## Status

The driver emits every record kind above for the 19 cases in `cases/`; the Rust decoders cover
all of them. Current results of the port against the oracle are in `docs/VERIFICATION.md`.

The port's driver writes Detour records with `nav-port bake <case> --records FILE`. For
tile-cache cases it has no tile builder yet (the tile cache is not ported), so it writes no `navmesh_tiles`
record; with `--tiles <oracle record file>` it loads the oracle's tiles and records the query
and malformed-tile kinds on them. Built with `--no-default-features` (in `parity/`) it uses
32-bit poly refs, for comparison with `nav_oracle`. `nav_oracle bench` with
`NAV_BENCH_NO_CROWD=1` times the query set without the crowd run (`nav-port bench-queries` is
the port's counterpart).


## Crowd regression run

`run-crowd-parity.sh <output-directory>` runs every case with `crowd_agents` enabled
for 300 ticks (using temporary case copies in the output directory),
compares the eight variants bit for bit in both reference widths, and checks that
repeated Rust runs produce identical files. Set `NAV_ORACLE32`, `NAV_ORACLE64`,
`NAV_PORT32`, `NAV_PORT64`, `NAV_PARITY` and `NAV_CORPUS_DIR` to the separately built
executables and pinned corpus. Build `nav-port` with default features for 64-bit
refs, and with `--no-default-features` for 32-bit refs. The crowd payload includes
the entire corridor after every tick; old payloads containing only `first_poly`
need to be regenerated with the current harness.
