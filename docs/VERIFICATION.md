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
- **Tile cache:** raw layer data, decoded grids, regions, contours, polygon meshes,
  obstacle scripts (including resident links) and queries after each update.

Generation, tile bytes and queries must be bit-exact. Timing and memory are not part of the
records.

## Corpus

20 case files in [`reference/cases/`](../reference/cases/) over 13 meshes. No mesh is committed:
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

## Recast and Detour baseline

Measured on x86-64 Linux with glibc, against both drivers (64-bit and 32-bit refs), on all 19
cases. Both widths give the same table.

| Records | Equal |
|---|---|
| Recast stage records | 35,347 / 35,347 |
| 21 Detour query families, each under two filters | equal on all 19 cases (38 / 38 records per family) |
| `malformed_tiles` | 19 / 19 |
| `navmesh_tiles` | 17 / 19 (the two tile-cache cases, see below) |
| Crowd trajectories, eight variants with full corridors | 72 / 72 in both debug and release, per ref width |

The earlier Detour-only campaign loaded oracle tiles for the two tile-cache
cases. The tile-cache implementation now builds those tiles itself; its separate
campaign below includes `multilevel_tilecache` and queries after each update.

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

Final gates pass: workspace tests (48 tests and 7 doctests), 32-bit tests (33 tests), clippy in both ref widths,
debug/release builds in both widths, Rust 1.88, native `no_std`, wasm32 build, documentation,
formatting, public text, and compilation of the crowd fuzz target. An initial lint
failure in the record driver was fixed and the complete lint gate passed again.
After the crowd review fixes, both complete campaigns above were rerun against
fresh reference records: zero mismatches, identical repeat files, and all count
assertions passed. All 16 crowd regression tests pass in both ref widths.

The earlier crowd campaign used oracle tile bytes for `seams_tilecache`.
The tile-cache campaign below compares that case with Rust-built tiles, including
its crowd records.

Unit tests cover invalid vectors, parameters and handles, exhausted memory/work
limits, independent per-call helper budgets, capacity, slot reuse, deterministic
trajectories, tile replacement, both
avoidance samplers, grid coordinate limits, corridor movement and queue expiry.
The `crowd_update` fuzz target has run under the release rule with no crash
([`FUZZ-RECEIPTS.md`](FUZZ-RECEIPTS.md)). Native `no_std` +
`alloc` with `libm`, and `wasm32-unknown-unknown`, build successfully. Crowd runtime
trajectories on wasm and the cross-platform `libm` tolerance are not verified here.

### Deliberate differences from C++

The pin above is the source of truth for defined, valid-use behavior. These safety
and resource differences do not alter any recorded crowd trajectory:

- **Slot lifetime:** `dtCrowd::removeAgent` only clears `active`, and `addAgent`
  leaves animation, target, corner and queue metadata behind. Rust resets the entire
  slot on removal and addition, preserving allocated buffers. A replacement cannot
  inherit an off-mesh animation or a request. Already submitted queue searches
  finish or expire independently; their handles are no longer retained by the slot.
- **Work limits:** C++ walks proximity-grid hash chains without a budget. Rust
  charges every queried cell and every visited chain entry, even nonmatching hash
  collisions, before processing it. Exhaustion returns `Error::LimitExceeded`.
- **References and capacity:** C++ accepts any nonzero crowd target ref. Rust's
  `request_move_target` borrows the current mesh and rejects zero, fabricated and
  stale refs with `Error::InvalidRef`, without changing the request. `PathQueue`
  also validates both refs when enqueuing. C++ silently drops excess grid/avoidance
  entries and returns an invalid handle for a full queue. Rust returns
  `Error::BufferTooSmall` for full agent, grid, avoidance-circle/segment and path
  queue pools. Grid insertions check all required cells before changing the grid.
  Inside `Crowd`, queue saturation leaves the agent in the publicly readable
  `TargetState::WaitingForQueue`; subsequent updates retry in upstream order.
  Grid query output retains upstream's bounded, deduplicated result order and
  returns the filled count when the caller's id slice is full.
- **Speed:** C++ accepts `maxSpeed == 0`, then divides off-mesh duration by it,
  potentially keeping the animation active forever. Rust requires finite,
  strictly positive `AgentParams::max_speed` on both addition and parameter
  updates. Zero returns `Error::InvalidParameter` before changing the slot.
- **Adaptive sampling:** `sampleVelocityAdaptive` declares an uninitialized
  `pat` float array. For each even angular division count, its final branch writes
  sample `npat + 1` (float indices `npat*2 + 2` and `npat*2 + 3`), then increments
  `npat`, leaving the preceding sample unwritten and subsequently reading it.
  With 32 divisions and four rings, that last write also exceeds C++'s 129-sample
  array. Rust retains zero-initialized storage, room for the extra write, and the
  pinned indexing; even counts have defined, repeatable output, with **no C++ parity
  claim**. Within the accepted parameter ranges, the adaptive pattern is fully
  initialized upstream exactly for odd `adaptive_divs` in `1, 3, ..., 31`, with
  `adaptive_rings` in `1..=4` and `adaptive_depth` in `1..=32`. Those combinations
  retain C++ arithmetic and sampling order on the oracle platform, provided all
  other inputs are valid, capacities and budgets suffice, and no other deliberate
  difference above is triggered. The recorded presets exercise divisions 5/7,
  rings 2/3 and depths 1/2/3/5; the corpus does not exhaust the whole odd-division
  parameter space. Grid sampling does not read the adaptive pattern, so its
  parity domain does not depend on division parity.
- **Cell domain:** Rust rejects cells outside the upstream signed-16-bit range
  with `Error::FormatLimit`, instead of truncating coordinates.

Regression tests cover removal during an actual off-mesh crossing and reuse at a
new location, colliding grid chains with insufficient and exact budgets, invalid
nonzero refs, atomic grid insertion and full helper pools, zero speed at a link
entrance, default/preset parameter customization, and finite repeatable adaptive
sampling for every even division count and ring count at depths 1/3/32. Four
compile-fail doctests enforce non-exhaustive parameter structs and state enums.

## Tile-cache verification

`reference/run-tilecache-parity.sh` compares all records on `dungeon_tilecache`,
`seams_tilecache` and `multilevel_tilecache`, with both polygon-reference widths.
The Rust driver builds every layer and tile itself. The oracle and port both
record all 21 query families under two filters after every obstacle-script update
as `obstacle_update_N.q_*` and `obstacle_update_N.filtered.q_*`.

The cases exercise all three obstacle forms, cross-tile and stacked-layer bounds,
batched additions/removals, salted slot reuse, and add/remove in the same batch.
Initial layer grids, regions, contours, polygon meshes, raw bytes, initial navmesh
tile bytes, and rebuilt resident bytes (including links) are compared exactly.
Independent release repeats and debug builds use the same comparator with no
tolerance. On x86-64 Linux (GCC 13.3.0, glibc 2.39, the pinned oracle flags), all
**6,744 record comparisons pass with zero mismatches** across both ref widths and
Rust debug/release. Independent release record files are byte-identical.

| Case | Layers | Script ops | Updates | Resident tile snapshots | Records per width/build |
|---|---:|---:|---:|---:|---:|
| `dungeon_tilecache` | 30 | 11 | 3 | 4 | 658 |
| `multilevel_tilecache` | 45 | 13 | 5 | 38 | 656 |
| `seams_tilecache` | 16 | 9 | 3 | 21 | 372 |
| Total | 91 | 33 | 11 | 63 | 1,686 |

Each width/build compares 455 layer-stage records, 588 query records (including
462 after obstacle updates), three obstacle scripts and three initial tile sets,
plus the remaining Recast, malformed-tile and crowd records. All resident bytes
include live first-link fields, the entire link pool, and snapped vertices.
`MeshTile.data` retains the original input; the record driver serializes the live
runtime sections for this comparison.

The 16-obstacle/tile scaling fixture uses 3,525,800 work units; 128 use 28,206,400:
**8.0000×** for 8× input, below the 11× regression ceiling. These are charged work
units including scratch-grid initialization, not wall-clock timings or an
all-input worst-case complexity claim. Colliding hash chains are separately
budget-tested. All 14 tile-cache regression tests pass in both ref widths.
Workspace tests (62 tests plus 10 doctests), both-width Clippy and debug/release
builds, Rust 1.88, native no-std/libm, wasm32 build, documentation, fuzz-source
compilation, formatting and public-text checks pass. The fuzz target was not run.
Initial lint failures and a recorder error (original bytes instead of live link
state) were corrected; the entire final verification run then passed.

Unit regressions cover truncated/oversized layers, invalid headers, area ids and
connections, invalid codec output lengths, all shapes and marking, full carving
and restoration, stale handles, obstacle/tile slot reuse, zero-touch lifecycle,
request/obstacle/touched-tile capacities, incomplete region-stage rejection,
budget failure and retry, and charged
hash collisions. The work-scaling fixture grows independent obstacles and tiles
from 16 to 128 and includes insertion, request handling and every rebuild.

### Tile-cache safety and resource differences

- Header sizes, finite ordered bounds, dimensions, grid lengths, area/height ids
  and connection indices are checked before use. Rust rejects malformed layers
  instead of reproducing unchecked C++ reads. Codec output must contain exactly
  three grids. Headers use little-endian fields and zeroed padding.
- Invalid shapes/handles and exhausted capacities return errors. More than eight
  touched layers is rejected atomically rather than silently truncated. The
  request queue holds 64 operations; the rebuild queue can hold every cache slot,
  avoiding upstream's 64-entry update-list truncation. Each coordinate has at
  most 32 layers. Adding/removing compressed tiles is rejected while obstacle
  requests or rebuilds are pending; a tile with obstacle associations cannot be
  removed. These rules keep tile membership stable for queued requests.
- Zero-touch processing and removal complete even with no queued tile. C++ can
  leave these slots stuck in processing/removing. Every reused slot and reverse
  association is reset; stale handles cannot refer to a replacement occupant.
- Bounds lookups charge cells and every hash-chain visit, including collisions.
  Reverse tile-to-obstacle indices avoid a whole-pool scan on each rebuild.
  Layer algorithms retain upstream order; their bounded region merges, ear
  clipping and greedy polygon merges charge their work before execution.
  Vertex and portal coordinate indices replace unrelated vertex/edge scans.
  Allocations are fallible and byte-charged before allocation; tile creation has
  a conservative scratch/output charge. Navmesh mutation also uses the navmesh's
  own limits. Custom compressor/process implementations own their internal costs.
- Failed rebuilds stay queued for retry; previously completed rebuilds remain
  applied. A navmesh insertion error can leave the old tile removed, as upstream
  does. There is no transaction/rollback guarantee across the update.
- The pinned vertex-removal feasibility test allocates 48 scalar entries but
  indexes triples beyond them. Rust reserves 48 full triples, checks capacities,
  and returns errors for invalid/nonmanifold meshes. No parity claim covers
  upstream undefined behavior, malformed input, exhausted resources, or the
  zero-touch/truncation cases above. Oriented-box exactness is measured on the
  oracle platform with its platform math functions.

## Beyond the records

- Tile bytes are untrusted input. Besides the `malformed_tiles` records, unit tests mutate tiles
  and check for typed errors, and the fuzz targets in [`fuzz/`](../fuzz/) feed arbitrary bytes
  to the tile reader, `add_tile`, tile removal and restore and a set of queries, and arbitrary
  meshes and settings to the whole Recast pipeline. A release requires each target to run from
  corpus seeds until coverage has been flat for 10 minutes (at least 10 and at most 120
  CPU-minutes per target) with no crash.
- Limits: unit tests check that exhausted `Limits` give errors, not panics.

## Not covered yet

- Compressed codecs beyond raw storage are supplied by applications and are not
  compared. The tile-cache fuzz source is compile-checked, not a completed campaign.
- Only x86-64 Linux with glibc has been compared with upstream. Other platforms can differ in the
  last bit where upstream calls `cos`/`sin`; the `libm` feature has not been compared with the
  driver.
- Queries on invalid refs where upstream copies uninitialised data (the hit normal of the plain
  `raycast`) are not in the query set.
- Tile state save and restore (`storeTileState` / `restoreTileState`) and big-endian tile data
  are not ported.
- The records were produced on one machine; a second machine and a CI job that rebuilds the
  driver are not set up.
