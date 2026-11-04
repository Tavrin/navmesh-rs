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

## Upstream corrections (`UpstreamFixes`)

Each correction is off by default, so every record above is unchanged. With a flag on, the
tests in `tests/fixes.rs` show the upstream behaviour (flag off) and the corrected one (flag
on) on a constructed case: sliced against plain corridors on a tiled floor
(`SLICED_CROSS_SIDE`), the six nearest neighbours of an agent among more than 32 candidates
(`NEAREST_NEIGHBOURS`), the velocity chosen from a two-division pattern (`ADAPTIVE_PATTERN`),
and the detail mesh of a zero sample distance, directly and through `bake` (`ZERO_SAMPLE_DIST`).
`nav-port --fixes all` writes corrected records and `reference/determinism.sh` compares their
changed digests with `reference/determinism-fixes.golden`. The ledge-filter report (`#729`)
is analysed in `reference/UPSTREAM.md` and has no flag.

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

## Cross-platform determinism

Contract. With the `libm` feature, every recorded output (Recast stage records, Detour tile
bytes, query results, crowd trajectories, tile-cache layers and rebuilt tiles) of the cases below
is byte-identical on x86-64 Linux, AArch64 Linux and wasm32. The default `std` build is identical
to the C++ oracle on x86-64 glibc (above) and can differ in the last bit on other platforms where
`cos`/`sin` are called. The `libm` build is a separate contract: it is not compared with the C++
library and need not equal the default build, although on this corpus on x86-64 Linux the two
produce the same digests (no `cos`/`sin` result differs in the last bit).

Sources of platform variance found and handled:

- `cos`/`sin` (walkable slope, oriented-box obstacles, avoidance sampling): platform libm with
  `std`, the `libm` crate with the feature. `sqrt`, `floor` and `ceil` are exact IEEE operations
  and the same either way. No other transcendental, `powf`, `round` or `mul_add` is used.
- FMA contraction: Rust never contracts `a * b + c`; the `libm` crate does not call `fma` in
  `cosf`/`sinf`.
- Sorting: stable sorts with upstream comparators; the unstable sorts are on totally ordered
  integer keys, where equal keys are indistinguishable.
- Hash iteration: none. The hashes (vertex, proximity grid, tile cache, node pool) are integer
  and masked, so `usize` width does not change a bucket.
- Pointer width: only the byte accounting (`Usage::bytes`, `Limits::max_bytes`) depends on it;
  outputs do not. Work accounting is in `u64`.
- NaN bit patterns: records compare float bit patterns; none differed on wasm32.

Check. `reference/determinism.sh` bakes eight cases that use only downloaded meshes
(`reference/corpus/fetch.sh --public-only`: solo, tiled and tile-cache builds, queries, crowds,
obstacle scripts), writes one line per record (`case record length digest`) and one per case for
the whole stream, and compares them with the committed `reference/determinism.golden` (3,035
lines). It needs no C++ oracle. CI runs it on x86-64 Linux, AArch64 Linux (`ubuntu-24.04-arm`) and
wasm32 (`wasm32-wasip1` under wasmtime).

Results at the commit that added it:

| Target | Run | Result |
|---|---|---|
| x86-64 Linux, `libm` | local | 3,035 digests equal the golden file |
| wasm32-wasip1 (wasmtime 50.0.1), `libm` | local | 3,035 digests equal the golden file |
| AArch64 Linux, `libm` | CI only | not run locally |
| x86-64 Linux, default `std` | local | digests equal those of the sources before the budget and link changes (identity of the default build) |

## Work budgets

Upstream budgets only the sliced A*. Every Detour query that can walk an unbounded part of the
navmesh now has a budgeted form taking a `Budget`, and `Crowd::update_budgeted` bounds a crowd
tick. One unit is one expansion: a polygon popped from a search, a polygon a raycast enters, a
portal the string-pulling funnel examines, a tile or candidate polygon of a box query. A charge
that does not fit is refused before the work is done, so a call never spends more than its
budget. Unbudgeted calls are unchanged and run the same per-polygon code.

`nav-port budget <case>` checks every budgeted form over the case's query set: with a sufficient
budget (and for the resumable searches, stepped eight units at a time) the result equals the
unbudgeted call, and no call or step spends more than its budget. CI runs it on `nav_test_solo`
and `dungeon_tiled` (63 random query pairs each, all equal, none over budget). Worst work per call
(units) and worst wall time (microseconds, release build, one x86-64 machine, noisy at this
scale), `nav_test_solo`:

| query | 1 | 16 | 64 | 256 | 1024 | unlimited |
|---|---|---|---|---|---|---|
| find_path | 1 / 0.4 | 16 / 1.9 | 64 / 10.0 | 77 / 7.0 | 77 / 6.9 | 77 / 5.2 |
| find_straight_path | 1 / 0.3 | 16 / 0.8 | 33 / 1.1 | 33 / 0.8 | 33 / 0.8 | 33 / 0.8 |
| query_polygons | 1 / 1.4 | 16 / 0.8 | 64 / 1.4 | 156 / 1.4 | 156 / 2.5 | 156 / 1.3 |
| find_random_point | 1 / 0.1 | 16 / 0.3 | 64 / 0.9 | 156 / 2.2 | 156 / 1.5 | 156 / 1.5 |
| polys_around_circle | 1 / 4.0 | 16 / 4.1 | 64 / 33.9 | 77 / 8.3 | 77 / 6.6 | 77 / 5.9 |
| distance_to_wall | 1 / 2.1 | 5 / 5.0 | 5 / 1.6 | 5 / 1.6 | 5 / 1.5 | 5 / 1.5 |
| local_neighbourhood | 1 / 2.4 | 16 / 12.7 | 64 / 76.5 | 64 / 66.5 | 64 / 58.3 | 64 / 73.2 |
| move_along_surface | 1 / 1.4 | 16 / 8.5 | 61 / 7.1 | 61 / 7.3 | 61 / 5.4 | 61 / 3.4 |
| raycast | 1 / 0.5 | 8 / 0.8 | 8 / 0.4 | 8 / 0.3 | 8 / 0.3 | 8 / 0.3 |

On `dungeon_tiled` the unlimited worst cases are 172 units for `find_path`, 183 for
`query_polygons`, 89 for `polys_around_circle` and 11 for `raycast`. The searches are bounded by
the query's node pool (`max_nodes`) and the local ones by their 64-node pool, so the budget
matters most for large pools and long raycasts. The crowd tick is not resumable (its phases
depend on each other): `update_budgeted` leaves the crowd untouched when the budget cannot cover
the up-front charge, and otherwise reports `Exhausted` with the caveat of `LimitExceeded` from
`update` (earlier phases applied; discard the tick).

## Global off-mesh links

`NavMesh::set_global_off_mesh_links(true)` lets a connection resolve its far endpoint in any
loaded tile. `tests/detour.rs` covers the minimal repro (tile size 1, quads in tiles (0,0) and
(10,0), a bidirectional connection between their centres): no path without the option, a path
with it, in either order of loading, and after removing and re-adding either tile; enabling and
disabling after loading relinks and unlinks. The default is unchanged because the extension only
adds links for pairs of tiles that upstream does not resolve.

## Not covered yet

- Compressed codecs beyond raw storage are supplied by applications and are not
  compared.
- Only x86-64 Linux with glibc has been compared with upstream, and only with the default
  `std` feature. Other platforms can differ in the last bit where upstream calls `cos`/`sin`;
  the `libm` feature is held to the cross-platform contract above, not to the driver.
- The determinism check covers the cases whose meshes are downloaded; the procedural meshes need
  the oracle's generator and are not in the golden file. AArch64 is verified by CI only.
- Queries on invalid refs where upstream copies uninitialised data (the hit normal of the plain
  `raycast`) are not in the query set.
- Tile state save and restore (`storeTileState` / `restoreTileState`) and big-endian tile data
  are not ported.
- The records were produced on one machine; a second machine and a CI job that rebuilds the
  driver are not set up.

## Query and large-terrain optimization qualification

The 200-pair `bench-paths` command in both drivers measures one nearest-polygon
query, one path search, and one ordinary straight-path query per pair. The drivers
use the same seeded random endpoints, node capacity (2,048), corridor capacity
(256), filter and nearest-query extents. Query/filter initialization and endpoint
generation precede the timed loop. Rust reuses its corridor and corner vectors.
Each instruction sample subtracts a one-batch process from a 101-batch process,
then divides by 20,000 pairs. This removes geometry loading, baking, assembly and
endpoint generation from the instruction estimate. Results describe this query
corpus, including partial paths when the 256-polygon capacity is insufficient;
they do not predict the cost of arbitrary authored queries.

The comparison uses release Rust and the pinned 64-bit C++ oracle, three
interleaved rounds on one fixed CPU, with medians reported. Stage measurements
use temporary instrumented source copies, per-thread Linux hardware instruction
counters excluding kernel/hypervisor work, and wall clocks at matching pipeline
boundaries. Stages include setup/marking, rasterization, filters, compact-field
construction, erosion/marking, distance field, regions, contours, polygon mesh,
detail mesh and tile creation. Per-stage counters exclude OBJ parsing, triangle
bucketing and final navmesh assembly. Summed per-tile wall times describe worker
work, not parallel elapsed time. End-to-end `bench-tiles --threads 16` includes
validation, bucketing, all tile work and ordered insertion, excluding OBJ parsing
and pool startup. Wall timings were collected on a shared machine and are not an
isolated throughput guarantee.

The first sparse node-reset candidate was discarded after inconsistent query
results; the bulk reset remains. Polygon traversal streams candidates without a
temporary Vec. Concrete filters specialize the path and nearest-polygon search;
trait-object filters remain supported. Rasterization swaps distinct buffer
references in the same order as upstream's pointer swaps. The default heuristic,
filter arithmetic, tie breaks and clipping operations remain unchanged.

Float import preserves the supplied base-vertex bits, including signed zero.
Only optional BV bounds are quantized; there is no base-vertex grid round trip.
The wide-extent regression retains a 0.0001-unit polygon beside a polygon at
10,000 units. Dijkstra regression coverage includes a positive-cost direct route
and a cheaper geometric detour through zero-cost off-mesh links. Triangulation
regressions distinguish upstream-compatible partial output from strict typed
failure; reports count incomplete contours and missing expected triangles.

On the final source, all 21 cases compare exactly against the pinned C++ oracle
in both reference widths: **76,018 records, zero mismatches**. All five tiled
cases also retain serial-driver/serial-API/parallel-API identity at 1, 2, 4, 8 and
16 threads in both widths. The unchanged golden file matches all **3,035** record
digests. Default tests (99 unit/integration tests and 15 doctests), both-width
and parallel feature checks, CI Clippy configurations with warnings denied,
native no_std variants, wasm32, documentation, formatting, the public-text guard,
and Rust 1.88 checks pass. Opt-in extension semantics are covered by Rust
regressions; they have no C++ equivalent.

An initial profiling attempt reused cached artifacts across temporary source
copies and was rejected. Accepted stage copies were rebuilt and their binary
identities checked. The final ordinary query binary is rebuilt after cleaning
only the two packages' release artifacts in the dedicated measurement target.
The public crate retains its unsafe-code prohibition; hardware-counter FFI exists
only in temporary instrumented copies. Earlier failed validation and profiling
attempts remain separate from the accepted measurements.

### Measurements

Measured on an AMD Ryzen 9 7945HX, Rust 1.98.1 and GCC 13.3.0. Each query
row is the median of three instruction-difference samples. The equally weighted
21-case mean fell from **39,107 to 36,998 instructions per
route pipeline (5.4% fewer)**. C++ used **29,293**;
the port remains **1.263x C++**, so the at-or-below-C++ query target
was **not reached**. These are route-pipeline counts (nearest + path + funnel),
not instructions for each individual helper in isolation.

For 200 terrain routes, solo median timed-loop wall cost was 0.145 ms before,
0.151 ms after, and 0.136 ms for C++; tiled cost was 0.112, 0.108 and 0.099 ms.
The core solo bake used 32.21 billion instructions before and 31.56 billion after
(C++: 26.21 billion); tiled used 45.48 and 44.56 billion (C++: 36.66 billion).
Rasterization instruction counts fell 4.1% solo and 3.9% tiled; total core-bake
counts fell about 2%. Validation remains a material part of Rust's stage cost.

Uninstrumented end-to-end 16-thread tiled bake samples were 890.041, 1040.917,
811.025 ms before and 999.106, 861.894, 934.004 ms after: medians **890.041 and
934.004 ms**. A separately timed C++ 16-thread run had a **383.741 ms** median.
These shared-machine runs establish an absolute measured tiled bake below one
second at the median, but **do not establish a wall-time improvement** or an
isolated Rust/C++ throughput ratio. The parallel stage table below is a separate
single instrumented run; its summed worker milliseconds are not elapsed time.
No consumer scene replay or production timeout acceptance is inferred.

| 200-pair case | Before instructions/pair | After | C++ | After/C++ |
|---|---:|---:|---:|---:|
| corridors_solo | 9,404 | 9,016 | 7,462 | 1.208 |
| degenerate_solo | 13,257 | 12,665 | 10,309 | 1.229 |
| dungeon_solo | 46,573 | 44,503 | 36,106 | 1.233 |
| dungeon_tilecache | 53,645 | 51,032 | 40,804 | 1.251 |
| dungeon_tiled | 63,672 | 60,276 | 48,185 | 1.251 |
| godot_level_solo | 27,869 | 26,073 | 20,702 | 1.259 |
| islands_offmesh | 4,990 | 4,674 | 3,965 | 1.179 |
| multilevel_solo | 18,190 | 17,310 | 14,162 | 1.222 |
| multilevel_tilecache | 51,455 | 47,987 | 38,023 | 1.262 |
| nav_test_areas | 41,504 | 38,881 | 31,008 | 1.254 |
| nav_test_solo | 30,964 | 29,204 | 23,263 | 1.255 |
| nav_test_tiled | 81,898 | 75,792 | 59,937 | 1.265 |
| queued_tilecache | 51,455 | 47,987 | 38,023 | 1.262 |
| seams_tilecache | 13,273 | 12,847 | 10,206 | 1.259 |
| seams_tiled | 14,387 | 13,923 | 10,877 | 1.280 |
| slopes_solo | 17,920 | 17,147 | 14,031 | 1.222 |
| stairs_monotone | 21,476 | 20,468 | 16,838 | 1.216 |
| terrain_large_solo | 12,008 | 11,391 | 9,364 | 1.217 |
| terrain_large_tiled | 9,312 | 8,811 | 6,861 | 1.284 |
| terrain_small_tiled | 185,757 | 175,894 | 138,076 | 1.274 |
| undulating_crowd | 52,232 | 51,086 | 36,956 | 1.382 |

terrain_large_solo: instruction millions / wall milliseconds per complete bake, summed across tiles.

| Stage | Before | After | C++ |
|---|---:|---:|---:|
| setup | 372.28 / 62.20 | 372.28 / 58.48 | 141.26 / 71.36 |
| raster | 15815.60 / 903.73 | 15166.73 / 946.91 | 14208.54 / 1018.17 |
| filters | 2573.07 / 145.03 | 2573.07 / 176.59 | 2310.65 / 201.43 |
| compact | 2114.95 / 204.43 | 2114.95 / 248.04 | 2370.82 / 224.64 |
| erode | 2823.11 / 221.56 | 2823.11 / 261.02 | 1912.52 / 195.94 |
| distance | 3456.49 / 299.16 | 3456.49 / 378.23 | 2427.25 / 227.67 |
| regions | 2750.77 / 562.60 | 2750.77 / 566.92 | 2211.37 / 552.89 |
| contours | 1426.66 / 121.45 | 1426.66 / 124.95 | 559.02 / 70.60 |
| mesh | 32.07 / 2.23 | 32.10 / 2.31 | 26.59 / 2.65 |
| detail | 837.65 / 63.28 | 837.65 / 66.14 | 31.33 / 4.06 |
| tile | 6.95 / 15.04 | 7.02 / 15.64 | 6.95 / 14.69 |

terrain_large_tiled: instruction millions / wall milliseconds per complete bake, summed across tiles.

| Stage | Before | After | C++ |
|---|---:|---:|---:|
| setup | 550.93 / 58.14 | 550.93 / 57.67 | 209.93 / 43.31 |
| raster | 23784.66 / 1134.86 | 22860.98 / 1086.42 | 21316.05 / 1261.49 |
| filters | 3493.83 / 163.77 | 3493.83 / 162.31 | 3156.67 / 195.15 |
| compact | 2839.91 / 231.21 | 2839.91 / 234.71 | 3168.35 / 198.44 |
| erode | 3816.21 / 311.49 | 3816.21 / 310.24 | 2571.44 / 240.55 |
| distance | 4668.50 / 353.93 | 4668.50 / 353.09 | 3257.76 / 294.35 |
| regions | 2876.48 / 273.54 | 2876.48 / 273.60 | 1825.79 / 232.83 |
| contours | 2050.49 / 150.83 | 2050.49 / 152.78 | 887.25 / 108.11 |
| mesh | 111.86 / 14.91 | 112.23 / 14.92 | 80.64 / 14.66 |
| detail | 1245.69 / 69.44 | 1245.69 / 69.17 | 164.80 / 22.65 |
| tile | 39.73 / 32.53 | 40.36 / 32.38 | 23.44 / 6.64 |

16-thread tiled stage profile: instruction millions / summed worker milliseconds (one instrumented run).

| Stage | Before | After | C++ |
|---|---:|---:|---:|
| setup | 642.78 / 95.52 | 642.75 / 92.16 | 210.33 / 70.10 |
| raster | 23785.98 / 1802.12 | 22862.26 / 1578.22 | 21316.20 / 2126.98 |
| filters | 3493.03 / 246.41 | 3493.03 / 228.23 | 3156.81 / 323.79 |
| compact | 2837.79 / 342.67 | 2837.79 / 334.95 | 3168.71 / 332.20 |
| erode | 3813.54 / 423.48 | 3813.53 / 390.63 | 2571.70 / 377.65 |
| distance | 4666.02 / 489.36 | 4666.02 / 447.85 | 3258.12 / 459.39 |
| regions | 2882.01 / 381.69 | 2882.32 / 364.56 | 1829.84 / 355.60 |
| contours | 2049.20 / 214.94 | 2049.16 / 205.41 | 889.14 / 159.32 |
| mesh | 110.71 / 22.70 | 110.99 / 20.02 | 81.11 / 22.20 |
| detail | 1244.40 / 118.69 | 1244.37 / 107.74 | 165.46 / 34.32 |
| tile | 41.00 / 67.53 | 41.58 / 65.69 | 24.49 / 9.99 |


## Follow-up speed measurements (2026-10-31)

The follow-up reduces dense triangle validation gathers and detail-triangle
corner lookup overhead. Compact-field validation has a verified dense
single-layer specialization; sparse and multilayer fields retain the generic
checks, as do finite work budgets. No validation is cached across caller edits.

All **76,018 parity records** (38,009 per reference width) and **3,035 determinism
digests** remain unchanged. Formatting, all CI clippy feature sets, tests,
doctests, parallel identity checks, no_std, wasm32, documentation and MSRV
checks returned zero. The public-text guard initially rejected a bit-field
comment; the corrected text passed the final check.

The equally weighted query mean fell from **36,998 to 35,829 instructions**
(**3.2%**), versus **29,293** for C++ (**1.223x**). The goal of at most 1.1x
C++ for every stage and queries was **not reached**. Solo setup improved 27.9%;
the large detail/contour gaps remain. Full compact-field validation still visits
the whole grid for each public stage call; the dense specialization did not
materially reduce that cost on the large terrain. Region expansion,
clipping, erosion and distance propagation remain substantial costs in the
retained Rust and C++ instruction profiles and annotations.

The same release toolchains and corpus as above were used. Query entries are
medians of three paired differences, `(instructions(repeat=101) -
instructions(repeat=1)) / 20,000`, pinning each process to CPU 6. Stages use
per-thread hardware instruction counters, summed across tiles, with the same
stage boundaries as above. Before and C++ stages were interleaved; final after
stages were collected separately after rejecting an earlier validation rewrite
that increased instructions. Initial scratch builds that reused stale artifacts
were also rejected; accepted builds forced recompilation of each source copy.
These stage wall times are shared-machine observations, not a causal speedup.
The counter FFI exists only in temporary profiling copies.

Uninstrumented 16-thread tiled bake samples (milliseconds) were
380.270, 385.445, 378.523 before and 411.163, 441.524, 387.951 after;
medians **380.270 and 411.163**. C++ also ran with **16 threads**, with a
**398.977 ms** median. These samples do **not establish a wall-time improvement**.

| 200-pair case | Before instructions/pair | After | C++ | After/C++ |
|---|---:|---:|---:|---:|
| corridors_solo | 9,016 | 8,578 | 7,462 | 1.150 |
| degenerate_solo | 12,665 | 12,041 | 10,309 | 1.168 |
| dungeon_solo | 44,503 | 43,684 | 36,106 | 1.210 |
| dungeon_tilecache | 51,031 | 50,406 | 40,804 | 1.235 |
| dungeon_tiled | 60,276 | 59,370 | 48,185 | 1.232 |
| godot_level_solo | 26,073 | 23,293 | 20,701 | 1.125 |
| islands_offmesh | 4,674 | 4,531 | 3,965 | 1.143 |
| multilevel_solo | 17,310 | 16,512 | 14,162 | 1.166 |
| multilevel_tilecache | 47,987 | 47,495 | 38,023 | 1.249 |
| nav_test_areas | 38,881 | 37,691 | 31,008 | 1.216 |
| nav_test_solo | 29,204 | 28,079 | 23,263 | 1.207 |
| nav_test_tiled | 75,792 | 74,802 | 59,937 | 1.248 |
| queued_tilecache | 47,987 | 47,495 | 38,023 | 1.249 |
| seams_tilecache | 12,847 | 12,507 | 10,206 | 1.225 |
| seams_tiled | 13,923 | 13,351 | 10,877 | 1.227 |
| slopes_solo | 17,147 | 16,605 | 14,031 | 1.183 |
| stairs_monotone | 20,468 | 19,956 | 16,838 | 1.185 |
| terrain_large_solo | 11,391 | 10,584 | 9,364 | 1.130 |
| terrain_large_tiled | 8,811 | 8,328 | 6,861 | 1.214 |
| terrain_small_tiled | 175,894 | 172,131 | 138,076 | 1.247 |
| undulating_crowd | 51,086 | 44,975 | 36,956 | 1.217 |

terrain_large_solo: instruction millions / wall milliseconds per complete bake, summed across tiles.

| Stage | Before | After | C++ |
|---|---:|---:|---:|
| setup | 372.28 / 58.63 | 268.52 / 65.70 | 141.26 / 67.03 |
| raster | 15166.73 / 829.68 | 15065.06 / 1192.82 | 14208.54 / 900.47 |
| filters | 2573.07 / 165.09 | 2573.07 / 250.95 | 2310.65 / 188.59 |
| compact | 2114.95 / 219.53 | 2114.95 / 297.75 | 2370.82 / 212.12 |
| erode | 2823.11 / 248.08 | 2811.47 / 283.95 | 1912.52 / 164.47 |
| distance | 3456.49 / 311.41 | 3444.85 / 396.42 | 2427.25 / 281.32 |
| regions | 2750.77 / 541.32 | 2739.12 / 779.32 | 2211.37 / 556.69 |
| contours | 1426.66 / 122.75 | 1415.01 / 134.00 | 559.02 / 75.09 |
| mesh | 32.10 / 2.12 | 32.10 / 3.57 | 26.59 / 2.50 |
| detail | 837.65 / 62.59 | 826.00 / 71.40 | 31.33 / 4.23 |
| tile | 7.02 / 13.75 | 7.02 / 20.56 | 6.95 / 13.66 |

terrain_large_tiled: instruction millions / wall milliseconds per complete bake, summed across tiles.

| Stage | Before | After | C++ |
|---|---:|---:|---:|
| setup | 550.93 / 64.91 | 541.77 / 75.49 | 209.93 / 46.23 |
| raster | 22860.98 / 1193.45 | 22864.04 / 1632.28 | 21316.05 / 1373.75 |
| filters | 3493.83 / 178.69 | 3493.83 / 243.15 | 3156.67 / 211.98 |
| compact | 2839.91 / 258.63 | 2839.91 / 332.27 | 3168.35 / 214.52 |
| erode | 3816.21 / 338.85 | 3800.48 / 401.26 | 2571.44 / 258.70 |
| distance | 4668.50 / 386.63 | 4652.77 / 462.70 | 3257.76 / 316.63 |
| regions | 2876.48 / 300.73 | 2860.75 / 359.43 | 1825.79 / 248.83 |
| contours | 2050.49 / 168.63 | 2034.76 / 205.86 | 887.25 / 115.79 |
| mesh | 112.23 / 16.73 | 112.23 / 20.08 | 80.64 / 15.07 |
| detail | 1245.69 / 78.92 | 1229.96 / 97.69 | 164.80 / 24.17 |
| tile | 40.36 / 38.68 | 40.36 / 48.62 | 23.44 / 7.12 |

## Structural terrain-stage validation diagnosis

Instruction sampling (`perf record -e instructions:u -c 50000 -g`) and
`perf annotate --stdio` on release Rust and the pinned C++ oracle locate the
large detail-mesh gap in `area::check_chf`, called once at entry to each public
stage. The solo compact field contains **11,648,569 cells and 5,594,809 spans**.
The existing dense specialization requires one span in every cell, so this
sparse field takes the general four-direction connection-validation loop.
C++ performs no equivalent full-field validation in `rcBuildPolyMeshDetail` or
`rcBuildContours`.

Separate diagnostic probes measured **801.64 million instructions** validating
the solo field at each stage entry. Actual detail construction used **24.36
million**, compared with **31.33 million** for C++; contour construction used
**613.38 million**, compared with **559.02 million** for C++. Thus validation
accounts for about **97% of the detail stage** and **57% of contours** before
this change. This is additional whole-grid work, rather than extra per-polygon
sampling or triangulation. The stage totals below exclude these nested probes;
their syscall/printing overhead is material when accumulated across small tiles.

Independent counting builds produced exactly the same helper counts in Rust
and C++, including the same total polygon-height-patch area:

| Helper or loop bound (each implementation) | Solo | Tiled sum |
|---|---:|---:|
| Detail polygons / height patches / hull triangulations | 1,570 | 11,297 |
| Height samples (`getHeight`) | 14,034 | 90,114 |
| Total height-patch cells | 98,961 | 425,355 |
| Interior distance-to-polygon / distance-to-triangle-mesh calls | 0 | 0 |
| Delaunay hull / edge lookup / facet completion calls | 0 | 0 |
| Contour walks / simplifications | 349 | 7,740 |
| Contour corner-height calls | 23,964 | 156,530 |
| Contour point-to-segment distance calls | 78,838 | 371,978 |
| Contour hole merges | 0 | 2 |

The fix is confined to detail and contour entry validation. A shared private
validator checks single-layer cell ranges and connections in one row pass,
using adjacent row slices and packed connection masks. Empty cells need a range
check; they do not each need four neighbouring-span lookups. Multilayer or
noncanonical connections fall back to the general validator. Finite work
budgets use the original path, including its exact error and charging order.
There is no validation cache, new public API, or unchecked-input bypass.
Differential regressions compare errors and work accounting with the original
validator across every direction/layer, cell edits, unused/aliased spans,
empty blocks and rows, and finite budgets.

Rejected experiments are retained separately: a block-based shortcut still
scanned neighbours of almost every empty cell (363,189 of 364,018 solo blocks
mixed occupied and empty cells); a contour row-iterator rewrite increased
instructions and was removed. An initial counter build had a C++ destructor
symbol collision; corrected per-module destructors produce the matching counts
above. An initial regression incorrectly compared unlimited dense-error
accounting with the finite-budget path; the corrected test compares matching
paths. A scratch-artifact cache collision selected a diagnostic binary during
one parity attempt; accepted builds force recompilation before copying each
binary. These attempts are excluded from accepted performance and gate results.

### Retained measurements

Baseline source is `b79a953e8d2c486529f284bed05986d2c6db909e`. The final
comparison uses default 64-bit poly refs, Rust 1.98.1 and GCC 13.3.0 on the
Ryzen 9 7945HX, CPU 6,
with three interleaved before/C++/after rounds per case. Hardware instructions
exclude kernel/hypervisor work. Only the matching outer stage boundaries are
instrumented; tiled entries sum all tiles. Values are medians of each round's
stage totals. Wall times come from a shared machine and are observations,
not an isolated throughput guarantee.

| Case | Stage | Before M instructions / ms | After M instructions / ms | C++ M instructions / ms | Before → after / C++ instructions |
|---|---|---:|---:|---:|---:|
| terrain_large_solo | detail | 826.00 / 54.45 | 303.71 / 26.08 | 31.33 / 4.23 | 26.363 → 9.693 |
| terrain_large_solo | contours | 1415.01 / 107.89 | 892.73 / 81.69 | 559.02 / 63.11 | 2.531 → 1.597 |
| terrain_large_tiled | detail | 1229.96 / 77.40 | 528.98 / 46.79 | 164.80 / 21.75 | 7.463 → 3.210 |
| terrain_large_tiled | contours | 2034.76 / 169.33 | 1333.78 / 134.27 | 887.25 / 105.35 | 2.293 → 1.503 |

Detail instructions fall **63.2% solo / 57.0% tiled**; contours fall **36.9% /
34.5%**. The **at-most-1.1x C++ target is not reached for either stage**.
Full-field input validation remains material, and C++ has no corresponding
pass. The retained change preserves these checks and the existing contour and
detail algorithms.

Retained receipts include the three-round `final-<case>-<implementation>-<round>.tsv`
stage logs, `profile-binaries.sha256`, instruction-sampling reports and assembly
annotations (`report-final.txt`, `annotate-final.txt`), and independent
`counts-<case>-<implementation>.tsv` helper counters. Hardware-counter FFI and
counter instrumentation exist only in temporary profiling copies; the library
keeps its unsafe-code prohibition. Binary/source hashes and the numerical
summary are retained with the run receipts. Earlier experiments and failed
checks remain separate from this final source and measurement set.

Final validation returned **0** for formatting, all CI Clippy feature sets with
warnings denied, tests and doctests, parallel and 32-bit configurations, both
native no_std configurations, wasm32, documentation, Rust 1.88, and the
public-text guard. All **76,018 parity records** (38,009 in each reference width)
match C++, and all **3,035 determinism digests** match the unchanged golden file.
All five tiled cases retain serial/parallel identity at 1, 2, 4, 8 and 16 threads
in both reference widths. These results qualify output identity and the measured
instruction reductions; they do not establish the unmet 1.1x performance target.

## 0.1.5 speed: validate once per pipeline, not once per stage

Same machine, toolchains, pinned oracle and measurement method as above (hardware
`instructions:u`, CPU 6, repeat-3 minus repeat-1 for bakes, repeat-101 minus repeat-1
over 20,000 pairs for queries). Function-level attribution used `perf record -e
ibs_op/cnt_ctl=1/ -c 500000` on both binaries, grouped by stage; it counts dispatched
ops, not instructions (a compare fused with its branch is one op), so it ranks
functions and is not a substitute for the instruction totals.

Causes found and fixed, all with byte-identical output:

- **Per-stage whole-grid validation.** The public stage functions each re-scan the whole
  compact field (11.6 million cells on the solo terrain). Attribution put `check_chf` and
  the surface validator at 2,382 million ops of 33,490 (7.1%). The crate's own bake
  pipeline now marks the workspace as holding a compact field it built and never exposed
  (`Workspace::compact_trusted`, crate-private, set only around the stages after
  `CompactHeightfield::build`), so those stages skip the scans while charging exactly the
  work the scans charged. Public stage calls on caller-supplied fields still validate in
  full, and a caller who edits public fields between calls is still caught. The public
  validator also takes the packed single-layer row pass for sparse fields when no work
  budget is set (it previously did so only for the detail and contour stages).
- **Library `floorf`/`ceilf` calls in span insertion and detail sampling.** The library
  routine linked into the binary (the baseline x86-64 target has no rounding instruction)
  took 2,027 samples, 3.0% of ops. `floor_i32`/`ceil_i32` truncate and correct, with the same saturation
  and NaN behaviour as `floorf(x) as i32`.
- **Open-list slots read the node array.** Each heap comparison indexed the node pool
  (a bounds check and a dependent load). Slots now cache the node's total, which only
  `modify` changes and `modify` rewrites the slot first, so the order, tie-breaks and
  results are unchanged.

| Bake (billion instructions per bake) | 0.1.4 | 0.1.5 | C++ | 0.1.5 / C++ |
|---|---:|---:|---:|---:|
| terrain_large_solo, public stage API | 30.47 | 28.50 | 26.21 | 1.087 |
| terrain_large_tiled, public stage API | 44.31 | 41.54 | 38.33 | 1.084 |
| terrain_large_tiled, `bake::tiles` pipeline (incl. bucketing, assembly) | 46.25 | 39.86 | 38.33 | 1.040 |

Stage attribution, solo terrain, million ops per bake (from `ibs_op`), 0.1.4 / 0.1.5 / C++:
setup 228 / 212 / 142; rasterization 15,134 / 14,190 / 14,558; filters 2,528 / 2,544 /
1,962; compact field 2,022 / 2,031 / 2,133; erosion 1,704 / 1,650 / 1,774; validation
2,382 / 1,324 / 0; distance field 2,383 / 2,388 / 2,221; regions 1,424 / 1,425 / 1,928;
contours 648 / 632 / 557 (the remainder of the contour and detail gap in the earlier
tables was validation). Through the public API the validation passes (1,324) are the
whole remaining excess in contours and detail; through the bake pipeline they are zero.
Filters (about 1.3x) and setup (1.5x of a small stage) were attributed but not reduced.

| Query, instructions per pair | 0.1.4 | 0.1.5 | C++ | 0.1.5 / C++ |
|---|---:|---:|---:|---:|
| equally weighted 21-case mean of the ratio | 1.187 | 1.154 | 1.000 | 1.154 |
| sum over the 21 cases | 746,399 | 717,929 | 615,252 | 1.167 |

The best cases are 1.09 (`godot_level_solo`) and 1.10 (`terrain_large_solo`); the worst
are 1.19 (`multilevel_tilecache`, `dungeon_tilecache`). Query profiles are flat: the
largest function is `find_path_impl` at about 32% of ops, and no single line holds more
than 2%. The remaining gap is spread over bounds checks and `Option` handling in
polygon lookup, portal points and the node pool, which the unsafe-free crate cannot
remove without per-function rewrites.

**The goal of at most 1.1x C++ for every stage and for queries was not reached.** Bake
totals are within 1.09x through the public API and 1.04x through the pipeline. Queries
remain at 1.15x. Output is unchanged: all 76,018 parity records (38,009 per width) match
C++ and all 3,035 determinism digests match the unchanged golden file.
