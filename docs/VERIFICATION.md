# Verification

How the port is checked against recastnavigation, on what inputs, with what results, and what is
not covered yet.

## Reference

- **Upstream pin:** recastnavigation `main` at `9f4ce64458dfae86e1239c525ddc219c4e9e06f1`
  (2026-02-27), not the `v1.6.0` release, which misses 35 later library commits with crash and
  out-of-bounds fixes. [`reference/UPSTREAM.md`](../reference/UPSTREAM.md) has the details.
- **Reference driver:** [`reference/harness/nav_oracle.cpp`](../reference/harness/nav_oracle.cpp)
  is compiled together with the pinned upstream sources (CMake refuses any other commit or a
  modified checkout). For each case file it builds the navmesh, runs a fixed query set under two
  filters, the crowd variants, a malformed-tile check and, for tile-cache cases, an obstacle
  script, and writes every result as a canonical little-endian record. The compiler, flags, libc
  and upstream commit go into each record file's metadata.
- **Both ref widths:** the build produces `nav_oracle` (upstream's default 32-bit poly refs) and
  `nav_oracle64` (`DT_POLYREF64`). The port is compared with default features against the 64-bit
  driver and with `polyref64` off against the 32-bit one.
- **Port driver and comparator:** `nav-port` (in `parity/`) drives the port over the same case
  files and writes the same records. `nav-parity diff` compares two record files record by record
  and reports the first differing field of any mismatch; `--recast` and `--detour` restrict the
  comparison to the record kinds the port implements.

## Records

The record kinds and their exact contents are listed in
[`reference/RECORDS.md`](../reference/RECORDS.md). In short:

- **Recast stages:** input geometry, configuration, triangle areas, the heightfield before and
  after the filters, the compact heightfield after building, erosion and area marking, the
  distance field, regions, contours, the polygon mesh, the detail mesh and heightfield layers.
- **Detour:** `navmesh_tiles` (the tile bytes, compared byte for byte), 21 query families under
  a default and a custom filter (nearest polygon, polygon search, path finding plain, sliced and
  partial, three straight-path modes, two raycast forms, distance to wall, move along surface,
  searches around a circle and a shape, local neighbourhood, wall segments, closest points,
  heights and random points), and `malformed_tiles`: upstream's verdicts on a set of corrupted and
  conflicting tiles.
- **Crowd:** eight trajectory variants with position, actual and chosen velocity, agent and
  target states, corner count and every corridor polygon after each tick.
- **Not yet compared:** tile-cache records (layer data and obstacle scripts).

Generation, tile bytes and queries must be bit-exact. Timing and memory are not part of the
records.

## Corpus

19 case files in [`reference/cases/`](../reference/cases/) over 13 meshes. No mesh is committed:
[`reference/corpus/fetch.sh`](../reference/corpus/fetch.sh) downloads the third-party meshes at
pinned commits, generates the others, and checks every SHA-256 against
[`MANIFEST.toml`](../reference/corpus/MANIFEST.toml).

- recastnavigation's demo meshes `nav_test`, `dungeon` and `undulating` (zlib, from the upstream
  repository).
- The Godot navigation demo level from godot-demo-projects (MIT).
- Nine generated meshes (our own, produced by the driver's `gen` command from integer hashes and
  exact constants): stairs, slopes around the walkable limit, multiple levels with a bridge,
  narrow corridors, islands with off-mesh links, tile seams, degenerate triangles, and terrains of
  128 m and 1024 m (2.1M triangles).

The cases cover solo and tiled builds, the three partitions, convex area volumes, filtered
queries, off-mesh links, crowds, tile-cache builds and coordinates 5 km from the origin.
[`reference/corpus/README.md`](../reference/corpus/README.md) maps features to meshes.

## Current results

Measured on x86-64 Linux with glibc, against both drivers (64-bit and 32-bit refs), on all 19
cases. Both widths give the same table.

| Records | Equal |
|---|---|
| Recast stage records | 35,347 / 35,347 |
| 21 Detour query families, each under two filters | equal on all 19 cases (38 / 38 records per family) |
| `malformed_tiles` | 19 / 19 |
| `navmesh_tiles` | 17 / 19 (the two tile-cache cases, see below) |
| Crowd trajectories, eight variants with full corridors | 72 / 72 in both debug and release, per ref width |

On the two tile-cache cases (`dungeon_tilecache`, `seams_tilecache`) upstream builds the tiles
with DetourTileCache, which is not ported, so the port writes no `navmesh_tiles` record there.
The port instead loads upstream's tile bytes through its own validating reader and navmesh, and
every query family and `malformed_tiles` is equal on them too.

The driver's own records are reproducible: two runs of the whole corpus give byte-identical
record files. Under `wasm32-wasip1` (Wasmtime), the port's record files are byte-identical to
the native ones on the two cases checked (`godot_level_solo`, `dungeon_tiled`).

## Crowd results

The nine crowd cases below are compared bit for bit, without the tolerance option,
in debug and release with both 32-bit and 64-bit polygon references. Each case runs
all eight variants: default avoidance, the four demo quality levels, velocity
requests with target resets, agent removal and re-addition, and a tile swap.

| Case | Agents | Configured ticks per variant |
|---|---:|---:|
| `dungeon_tiled` | 16 | 300 |
| `godot_level_solo` | 8 | 200 |
| `islands_offmesh` | 8 | 300 |
| `multilevel_solo` | 8 | 200 |
| `nav_test_areas` | 8 | 200 |
| `nav_test_solo` | 16 | 300 |
| `seams_tilecache` | 8 | 150 |
| `terrain_small_tiled` | 32 | 300 |
| `undulating_crowd` | 32 | 300 |

The configured runs compare 18,000 simulation ticks and 294,498 active-agent ticks
per build and reference width: 72,000 simulation ticks and 1,177,992 active-agent
comparisons altogether, with zero mismatches. Entire Rust record files from two
independent runs are identical on all nine crowd cases, in both ref widths. The
longest recorded corridor has 54 polygons. The island case exercises off-mesh
animation in all eight variants (512 active-agent ticks per build and width).

The design gate also runs all nine cases for 300 ticks per variant, using case
copies so the configured inputs stay unchanged. All 72 records are bit-exact per
build and width: 21,600 simulation ticks and 323,000 active-agent ticks per build
and width, or 86,400 simulation-tick comparisons and 1,292,000 active-agent
comparisons across all four combinations. Mismatches: **0**. Independent repeat
files are identical. `reference/run-crowd-parity.sh` reproduces the 300-tick release
comparison and repeat check from separately built drivers.

Final gates pass: workspace tests (40 tests and 3 doctests), 32-bit tests, clippy in both ref widths,
debug/release builds in both widths, Rust 1.88, native `no_std`, wasm32 build, documentation,
formatting, public text, and compilation of the crowd fuzz target. An initial lint
failure in the record driver was fixed and the complete lint gate passed again.

The two tile-cache cases still use oracle tile bytes; the crowd comparison on
`seams_tilecache` runs those bytes through the port's validating reader and navmesh.
No tile-cache implementation is implied.

Unit tests cover invalid vectors, parameters and handles, exhausted memory/work
limits, independent per-call helper budgets, capacity, slot reuse, deterministic
trajectories, tile replacement, both
avoidance samplers, grid coordinate limits, corridor movement and queue expiry.
The `crowd_update` fuzz target is compiled but has not been run. Native `no_std` +
`alloc` with `libm`, and `wasm32-unknown-unknown`, build successfully. Crowd runtime
trajectories on wasm and the cross-platform `libm` tolerance are not verified here.

Upstream's adaptive pattern reads uninitialized storage for even angular division
counts; the Rust sampler uses zero-initialized storage there. The recorded presets
use five or seven divisions. Grid coordinates beyond the upstream signed 16-bit
cell range return a typed error.

## Beyond the records

- Tile bytes are untrusted input. Besides the `malformed_tiles` records, unit tests mutate tiles
  and check for typed errors, and the fuzz targets in [`fuzz/`](../fuzz/) feed arbitrary bytes
  to the tile reader, `add_tile`, tile removal and restore and a set of queries, and arbitrary
  meshes and settings to the whole Recast pipeline. A release requires each target to run from
  corpus seeds until coverage has been flat for 10 minutes (at least 10 and at most 120
  CPU-minutes per target) with no crash.
- Limits: unit tests check that exhausted `Limits` give errors, not panics.

## Not covered yet

- DetourTileCache is not ported, so tile-cache records and the `navmesh_tiles` of the
  two tile-cache cases are not compared.
- Only x86-64 Linux with glibc has been compared with upstream. Other platforms can differ in the
  last bit where upstream calls `cos`/`sin`; the `libm` feature has not been compared with the
  driver.
- Queries on invalid refs where upstream copies uninitialised data (the hit normal of the plain
  `raycast`) are not in the query set.
- Tile state save and restore (`storeTileState` / `restoreTileState`) and big-endian tile data
  are not ported.
- The records were produced on one machine; a second machine and a CI job that rebuilds the
  driver are not set up.
