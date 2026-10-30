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

21 case files in [`reference/cases/`](../reference/cases/) over 13 meshes. No mesh is committed:
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
`seams_tilecache`, `multilevel_tilecache` and `queued_tilecache`, with both
polygon-reference widths. The queued case adds an obstacle after one cross-tile
rebuild, then records another single rebuild before draining: the newly added
obstacle becomes processed while its add request is still queued, as in the pin.
The Rust driver builds every layer and tile itself. The oracle and port both
record all 21 query families under two filters after every obstacle-script update
as `obstacle_update_N.q_*` and `obstacle_update_N.filtered.q_*`.

The cases exercise all three obstacle forms, cross-tile and stacked-layer bounds,
batched additions/removals, salted slot reuse, and add/remove in the same batch.
Initial layer grids, regions, contours, polygon meshes, raw bytes, initial navmesh
tile bytes, and rebuilt resident bytes (including links) are compared exactly.
Independent release repeats and debug builds use the same comparator with no
tolerance. On x86-64 Linux (GCC 13.3.0, glibc 2.39, the pinned oracle flags), all
**9,200 record comparisons pass with zero mismatches** across both ref widths and
Rust debug/release. Independent release record files are byte-identical.

| Case | Layers | Script ops | Updates | Resident tile snapshots | Records per width/build |
|---|---:|---:|---:|---:|---:|
| `dungeon_tilecache` | 30 | 11 | 3 | 4 | 658 |
| `multilevel_tilecache` | 45 | 13 | 5 | 38 | 656 |
| `queued_tilecache` | 45 | 8 | 4 | 17 | 614 |
| `seams_tilecache` | 16 | 9 | 3 | 21 | 372 |
| Total | 136 | 41 | 15 | 80 | 2,300 |

Each width/build compares 680 layer-stage records, 798 query records (including
630 after obstacle updates), four obstacle scripts and four initial tile sets,
plus the remaining Recast, malformed-tile and crowd records. All resident bytes
include live first-link fields, the entire link pool, and snapped vertices.
`MeshTile.data` retains the original input; the record driver serializes the live
runtime sections for this comparison.

The 16-obstacle/tile scaling fixture uses 3,577,496 work units; 128 use 29,365,440:
**8.2084×** for 8× input, below the 11× regression ceiling. These are charged work
units including scratch-grid initialization, not wall-clock timings or an
all-input worst-case complexity claim. Colliding hash chains are separately
budget-tested. All 18 tile-cache integration tests and the merge-pass unit
regression pass in both ref widths. Three new compile-fail doctests enforce non-exhaustive contour
and mesh types; constructors/defaults allow public-field customization.
Workspace tests (67 tests plus 13 doctests), both-width Clippy and debug/release
builds, Rust 1.88, native no-std/libm, wasm32 build, documentation, fuzz-source
compilation, formatting and public-text checks pass. The `tilecache_update` fuzz target has
since run under the release rule with no crash ([`FUZZ-RECEIPTS.md`](FUZZ-RECEIPTS.md)).

Unit regressions cover truncated/oversized layers, invalid headers, area ids and
connections, invalid codec output lengths, all shapes and marking, full carving
and restoration, stale handles, obstacle/tile slot reuse, zero-touch lifecycle,
request/obstacle/touched-tile capacities, incomplete region-stage rejection,
budget failure and retry, and charged
hash collisions. Added regressions reject a 1,020-vertex contour before allocation
with a small work limit, precharge every greedy merge pass, reject 100,000 empty
contours before traversal, count all three contour-list passes, and complete a
queued add during an existing rebuild. The feature matrix also tests
`--no-default-features --features std,tilecache` and builds
`--no-default-features --features libm,tilecache`, without crowd.
The work-scaling fixture grows independent obstacles and tiles
from 16 to 128 and includes insertion, request handling and every rebuild.

The review-fix rerun also compares the complete 21-case corpus in release:
38,009 records per reference width, 76,018 comparisons altogether, with zero
mismatches, including all Recast/Detour records and 21 initial tile sets per width.
Both crowd campaigns above were rerun in debug/release and both reference widths:
158,400 simulation-tick and 2,469,992 active-agent comparisons, zero mismatches.
Independent release repeats are identical on every crowd and tile-cache case.

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
  leave these slots stuck in processing/removing when no rebuild runs. During an
  existing rebuild queue, Rust follows the pinned whole-pool state scan: every
  rebuild completes processing/removing obstacles with zero pending tiles, even
  if their add requests are still deferred. Every reused slot and reverse
  association is reset; stale handles cannot refer to a replacement occupant.
- Bounds lookups charge cells and every hash-chain visit, including collisions.
  Reverse tile-to-obstacle indices select obstacles for carving. Lifecycle
  completion scans the whole obstacle pool after every rebuild in upstream order;
  its bounded per-slot work is charged before the rebuild.
  Layer algorithms retain upstream order. Ear clipping and greedy polygon merging
  have O(n³) worst-case work for n contour vertices; their algorithms are preserved
  for bit-exact output. Default limits are unlimited, so callers handling untrusted
  or large contours must choose a finite `Limits::max_work` to bound work. Cubic
  triangulation work is charged during contour validation, before mesh/scratch
  allocation. Every greedy merge pass charges all pair/edge comparisons before
  running, including its final pass; portal-hole triangulation is also precharged.
  Every contour-list traversal charges one unit per entry before traversing,
  including empty contours, and validation charges vertex bytes separately.
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

### Recast and bake deliberate differences from C++

- Grids with exactly one zero dimension return a typed error instead of an empty
  mesh: `Heightfield::new` returns `Error::InvalidParameter`, and stages reject
  inconsistent public heightfield dimensions with `Error::InvalidInput`.
  This includes sliver meshes whose rounded grid is zero by a saturated row or
  column count. A zero-by-zero grid remains valid. Compact-grid validation and
  stage row traversal charge work even when rows contain no spans; finite work
  budgets can therefore fail earlier with `Error::LimitExceeded`.

- C++ uses allocator hooks and has no byte/work budgets. Rust charges all detail
  queues/triangulation buffers, contour winding/hole/simplification/sort storage,
  region traversal/merge storage, and mesh removal scratch before fallible
  reservation. Exhaustion returns `Error::LimitExceeded` or
  `Error::AllocationFailed`, propagated with the ordered tile coordinate.
  Stable sorting uses fallibly reserved scratch and retains glibc tie order.
- C++'s adjacency builder stores edge-chain indices in `unsigned short` without
  checking their range. Index 65,535 collides with its null sentinel, and 65,536
  wraps to zero. Rust permits indices 0..=65,534 and returns
  `Error::LimitExceeded` before storing an out-of-range index. This deliberately
  rejects the C++ defect; no parity is claimed for meshes that trigger it.
  Adjacency work is charged before its arrays are allocated.
- Convex marking charges validation, polygon coordinates, rectangle cells and
  every visited span times polygon vertex count before visiting that cell's spans.
  C++ does not budget that work. A later cell exhausting work may leave earlier
  cells marked; a failed bake never returns partial tiles.
- The bake extension spatially buckets volumes once in input order and only
  applies volumes overlapping the tile's bordered xz column. Heights retain
  Recast's quantized span checks: a zero-height geometry bounding box must not
  exclude volumes that mark the rasterized floor just above it. Setup charges
  each candidate tile overlap before visiting it, avoiding all-volumes-per-tile
  scans for spatially independent inputs.
- Exact Detour output size and polygon/vertex/link/detail counts are computed
  before builder allocations and checked against tile limits. The public Detour
  builder retains unlimited defaults; the bake builder applies `tile_limits`.
  C++ has no equivalent caller-specified limits.

## Parallel tiled baking

The `bake` extension is **not in recastnavigation**. `tiles` and `tiles_parallel`
share the tile pipeline; only the latter uses rayon with `parallel`. Both APIs
remain available without that feature. Validation and spatial triangle bucketing
are serial, tile pipelines are independent, and insertion into Detour is serial
in row-major order. Setup and each tile have separate budgets; per-tile byte/work
charges accumulate across stage resets. `BakeConfig::max_total_bytes` additionally
admits setup storage, retained output and concurrent worker storage before any
tile starts. It defaults to unlimited. A finite aggregate budget requires finite
`tile_limits.max_bytes`; otherwise it returns `Error::LimitExceeded` without a
tile location. The conservative reservation is:

`setup_bytes + tile_count * min(max_tile_bytes, max_bytes) + min(tile_count, workers) * max_bytes`

Here the two tile limits come from `tile_limits`; setup includes both result-slot
vectors. Serial uses one worker; parallel uses the current rayon pool size (one
without the feature). Arithmetic overflow rejects admission. Setup allocations
also respect the total ceiling. Empty tiles reserve output space too; a worker's
output may be counted in both terms. These are charged-storage bounds, excluding
allocator metadata/transient reallocations, thread stacks and rayon's pool and
scheduler storage, rather than a process RSS limit. Per-tile limits remain in
force, and `into_navmesh` uses its own separate limits.

On x86-64 Linux, `nav-port verify-parallel` compares the independent original
serial driver against the public serial API, then against the public parallel API
at **1, 2, 4, 8 and 16 threads**, with both polygon-reference widths. Comparisons
include the complete DNAV tile bytes and all 21 query families under both filters,
using the public ordered navmesh assembler for the parallel results.

| Case | Nonempty tiles per bake | Query records per bake |
|---|---:|---:|
| dungeon_tiled | 41 | 42 |
| nav_test_tiled | 66 | 42 |
| seams_tiled | 16 | 42 |
| terrain_small_tiled | 49 | 42 |
| terrain_large_tiled | 2,681 | 42 |

All **50 parallel case/width/thread combinations** pass: **28,530 tile-byte** and
**2,100 query-record comparisons**, zero mismatches. The ten additional public
serial/API baseline checks also pass (5,706 tiles and 420 query records).
Every input mesh SHA-256 matches the repository manifests. This is serial/parallel
identity evidence; it does not rerun the earlier C++ oracle campaigns above.

### Wall time

Measured 2026-10-30 on an AMD Ryzen 9 7945HX, release profile, Rust 1.98.1,
with the processes restricted to CPUs 0,2,...,30 (one logical CPU per physical
core, the same affinity mask for every configuration). Values are **median ms of
five interleaved samples** per configuration, after one warmup each. Round order
rotates and alternates direction. These are shared-host observations, with boost
enabled; affinity does not reserve exclusive cores or fix CPU frequency.

The timed interval includes input validation, triangle bucketing, all tile stages,
ordered output and public navmesh assembly. It excludes OBJ loading, pool startup,
queries and destruction of the assembled navmesh. Serial bucketing and insertion
remain in every interval; no algorithm changes or single-tile acceleration are
claimed.

| Case | Ref bits | Serial | Rayon 1 | Rayon 2 | Rayon 4 | Rayon 8 | Rayon 16 |
|---|---:|---:|---:|---:|---:|---:|---:|
| terrain_small_tiled | 32 | 80.048 | 82.457 | 42.545 | 23.165 | 13.522 | 11.882 |
| terrain_small_tiled | 64 | 81.945 | 79.418 | 41.480 | 22.841 | 13.116 | 11.124 |
| terrain_large_tiled | 32 | 2912.815 | 2873.817 | 1578.098 | 854.902 | 483.144 | 325.919 |
| terrain_large_tiled | 64 | 2851.153 | 2856.590 | 1505.990 | 831.592 | 476.162 | 342.498 |

Reproduce with `reference/run-parallel.py <fresh-output-directory>`, setting
`NAV_PORT32`, `NAV_PORT64` to release `nav-port` binaries built with `parallel`
(and with/without `polyref64`), `NAV_CORPUS_DIR` to the hash-checked corpus, and
`NAV_BENCH_CPUS` to the affinity mask. The runner retains `identity.json`, all
120 individual timing samples in `samples.jsonl`, and medians in `summary.json`.

### Gates and boundaries

Workspace tests pass with defaults and with `parallel` in both ref widths; Clippy
passes with warnings denied for all three configurations. Formatting, public-text,
Rust 1.88, rustdoc with warnings denied, native no_std/libm, and the wasm32 build
also pass. Wasm is built **without `parallel`**; threaded rayon on wasm is not
qualified. The new `parallel_bake` fuzz target compiles with its `parallel` feature;
it and `recast_input` later ran under the release rule with no crash (see
[`FUZZ-RECEIPTS.md`](FUZZ-RECEIPTS.md), "Parallel bake release run"). The complete
gate rerun passes after correcting an initial iterator lint.

Regressions cover all three partitions, area volumes, triangle areas, off-mesh
links, empty tiles, one-tile layouts, malformed inputs, setup budgets/retry,
cumulative stage budgets, format limits, first-tile failures at every requested
thread count, and a later tile error finishing before the first ordered error.

### Parallel bake safety review rerun (2026-10-30)

After the allocation, aggregate-budget, volume-work, adjacency-range and
preallocation-limit fixes, fresh pinned C++ oracles and fresh Rust drivers pass
the complete 21-case corpus in both ref widths: **76,018 records, zero
mismatches**. The separate debug/release tile-cache campaign passes **9,200**
comparisons; repeat files are identical. Both crowd campaigns pass in
both widths and debug/release: **158,400 simulation-tick and 2,469,992 active-agent
comparisons**, zero mismatches. Parallel identity again passes all **50**
case/width/thread combinations at 1/2/4/8/16 threads: **28,530 tile-byte and 2,100
query-record comparisons**, plus ten public serial/API baselines (5,706 tiles,
420 queries).

The 8-to-64 independent tile/volume fixture charges setup work 442 -> 3,634
(**8.22x**) and total tile work 671,969 -> 5,481,198 (**8.16x**), below its 9x
regression ceiling. These are work counters, not wall-clock scaling claims.
Thirteen added regressions cover the five findings, allocation siblings, exact
adjacency range, border/height volume selection and finite aggregate admission.
Workspace tests (88 with parallel, plus 14 doctests), both-width serial/parallel
Clippy, parallel-off tests, tilecache-only and bake-only feature tests, native
no_std/libm, wasm without parallel, Rust 1.88, rustdoc, formatting, public text
and parallel fuzz-source compilation all pass. The complete final gate set was
rerun after development formatting fixes. The new fuzz target was not yet run at that
point; its release-rule run is recorded in [`FUZZ-RECEIPTS.md`](FUZZ-RECEIPTS.md).

### Degenerate-grid regression rerun (2026-10-31)

After rejecting grids with exactly one zero dimension and charging empty-row
traversal, fresh pinned C++ and Rust drivers pass all 21 corpus cases in both
reference widths: **76,018 records, zero mismatches**. Separate tile-cache checks
pass **9,200** debug/release comparisons with identical repeats. The original and
300-tick crowd campaigns pass **158,400 tick and 2,469,992 active-agent
comparisons** across both widths and debug/release. Parallel identity passes all
**50** case/width/thread combinations: **28,530 tile-byte and 2,100 query-record
comparisons**, plus ten serial/API baselines (5,706 tiles and 420 queries).

The three new regressions cover both sliver orientations, mutated public grids,
and finite work limits on rows without spans. Workspace tests (91 with parallel,
plus 14 doctests), both-width serial/parallel Clippy, feature-specific tests,
no_std/libm, wasm without parallel, Rust 1.88, rustdoc, fuzz-source compilation,
formatting and public-text checks all pass. The full gate set was rerun after
fixing a constant-chunk iterator lint in the new regression.

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
  compared.
- Only x86-64 Linux with glibc has been compared with upstream. Other platforms can differ in the
  last bit where upstream calls `cos`/`sin`; the `libm` feature has not been compared with the
  driver.
- Queries on invalid refs where upstream copies uninitialised data (the hit normal of the plain
  `raycast`) are not in the query set.
- Tile state save and restore (`storeTileState` / `restoreTileState`) and big-endian tile data
  are not ported.
- The records were produced on one machine; a second machine and a CI job that rebuilds the
  driver are not set up.
