# Fuzz receipts

Each receipt records one run of a fuzz target under the release rule in
[`fuzz/README.md`](../fuzz/README.md): start from seeds, run in 600-second chunks reusing the
corpus, stop when a whole chunk adds no coverage, for at least 10 and at most 120 CPU-minutes,
with no crash.

## `crowd_update`, 2026-10-29

Commit under test: `fb14438` (`feat/crowd`). Toolchain: nightly, `cargo-fuzz` 0.13.2, one libFuzzer
worker, `-max_len=1024`, 600-second chunks (`-max_total_time=600`) reusing one corpus directory.

Seeds: 11 hand-built inputs (the `seeds` tool has no crowd mode): five scripted scenarios (add and
update, move target, move velocity, eight agents sent to one target, remove and reset) and six
pseudo-random 40-record inputs. Each record is the 16-byte layout the target reads (three
little-endian `f32`, agent id byte, operation byte, two unused bytes).

| Chunk | CPU seconds | Executions | Coverage start | Coverage end | Crashes |
|---|---|---|---|---|---|
| 1 | 600 | 197,146 | cov 2028 / ft 5267 (11 seeds) | cov 2153 / ft 11349 | 0 |
| 2 | 600 | 184,925 | cov 2153 / ft 11349 | cov 2153 / ft 11370 | 0 |
| Total | 1,200 | 382,071 | cov 2028 | cov 2153 | 0 |

`cov` is libFuzzer's edge count. It did not change during chunk 2 (the last edge was found at
execution 195,187 of chunk 1, about 2 seconds before its end), so it was flat for the full 600
seconds of chunk 2. The feature count (`ft`) rose by 21 in chunk 2; the rule is applied to edge
coverage. 1,200 CPU-seconds is above the 10-minute floor and far below the 2-hour cap.
Final corpus: 384 inputs, 148 KB. Peak RSS 429 MB. No artifact files were written.

## Tile cache and crowd rerun, 2026-10-29

Commit under test: `df27614` (`feat/tilecache`) plus the fuzz-only changes of the commit that adds
this receipt. Toolchain: nightly, `cargo-fuzz` 0.13.2, one libFuzzer worker per target, 600-second
chunks (`-max_total_time=600`) reusing one corpus directory per target, `-print_final_stats=1`.
Rule: stop when a whole chunk adds no edge coverage, after at least 600 CPU-seconds and at most
7,200; no crash. CPU seconds are user plus system time of the fuzzer process
(`/usr/bin/time`), so time spent waiting in the machine's job queue is not counted.

### `tilecache_update`

The target now reads `[mode][layer length u16][layer bytes][16-byte operations]` (layout in the
target source). The layer is either a raw layer (header and grids mutated directly) or an 8 x 8
grid specification turned into a valid layer with `build_layer`. It goes through
`decompress_layer`, `build_regions`, `build_contours` and `build_poly_mesh`, and through the cache's
own `add_tile`, so the cache rebuilds it on `update`. Operations add cylinder, box and oriented-box
obstacles, remove one, update once and drain the update queue; the cache is drained after the
last operation. Before this change the layer bytes and the operation stream were the same bytes,
and the cache only ever held the fixed flat layer.

Seeds: 17 files written by `seeds tilecache` (four raw layers with and without holes, each with
and without operations; one obstacle script on the fixed layer; eight grid specifications with
derived and raw connections). `-max_len=4096`.

| Chunk | CPU s | Executions | Coverage start (cov / ft) | Coverage end | Crashes |
|---|---|---|---|---|---|
| 1 | 595 | 43,222 | 2663 / 9678 (17 seeds) | 2965 / 16757 | 0 |
| 2 | 593 | 39,077 | 2965 / 16757 | 2974 / 17443 | 0 |
| 3 | 594 | 36,257 | 2974 / 17443 | 2994 / 17675 | 0 |
| 4 | 578 | 28,322 | 2994 / 17675 | 3175 / 18144 | 0 |
| 5 | 576 | 31,298 | 3175 / 18144 | 3191 / 18304 | 0 |
| 6 | 588 | 30,033 | 3191 / 18304 | 3210 / 18604 | 0 |
| 7 | 588 | 32,229 | 3210 / 18604 | 3214 / 18774 | 0 |
| 8 | 593 | 32,803 | 3214 / 18774 | 3229 / 19337 | 0 |
| 9 | 595 | 35,026 | 3229 / 19337 | 3240 / 19546 | 0 |
| 10 | 594 | 33,199 | 3240 / 19546 | 3242 / 19790 | 0 |
| 11 | 589 | 27,439 | 3242 / 19790 | 3242 / 20001 | 0 |
| Total | 6,483 | 368,905 | cov 2663 | cov 3242 | 0 |

Chunk 11 added no edge, so the run stopped there (6,483 CPU-seconds, under the 7,200 cap).
Final corpus: 2,219 inputs, 9.0 MB. Peak RSS 371 MB. No artifact files were written.

What the final corpus reaches (`cargo fuzz coverage`, region coverage of the instance used by the
target): `add_obstacle` 26/26 regions, `remove_obstacle` 14/15, `process_requests` 46/47,
`update` 42/44, `rebuild` 15/17, `add_tile` 45/51, `decompress_layer` 6/6, the layer decoder 51/60,
`build_regions`, `build_contours` and `build_poly_mesh` 100%. Over `cache.rs`, `layer.rs` and
`builder.rs` together: 85.5% of regions, 72.0% of lines.

### `crowd_update`

Seeds: 55 files written by the new `seeds crowd` mode, in the 16-byte record layout (three
little-endian `f32`, agent byte, operation byte, two tail bytes): 1 to 48 records in three
operation mixes (random, eight agents then targets and updates, add/remove churn), and for
every mix the 62, 63 and 64-record sizes (992, 1,008 and 1,024 bytes) with four tail styles (zero,
`0xff`, counter, random), plus a 63-record input with a 15-byte partial trailing record
(1,023 bytes). The 11 hand-made seeds of the first run were not kept. `-max_len=1024`.

| Chunk | CPU s | Executions | Coverage start (cov / ft) | Coverage end | Crashes |
|---|---|---|---|---|---|
| 1 | 588 | 126,577 | 2026 / 8086 (55 seeds) | 2154 / 11272 | 0 |
| 2 | 595 | 145,776 | 2154 / 11272 | 2154 / 11281 | 0 |
| Total | 1,183 | 272,353 | cov 2026 | cov 2154 | 0 |

Chunk 2 added no edge. Final corpus: 641 inputs, 2.6 MB. Peak RSS 436 MB. No artifact files were
written. The end coverage is within one edge of the first run's 2153, from different seeds.

## Parallel bake release run, 2026-10-30

Commit under test: `2200e4f` (`feat/parallel-bake`, PR 4) plus the fuzz-only changes of the commit
that adds this receipt (no library source changed). Toolchain: `rustc 1.101.0-nightly
(282215592 2026-10-04)`, `cargo-fuzz` 0.13.2, AddressSanitizer build, one libFuzzer worker,
`-timeout=60 -rss_limit_mb=4096`, 600-second chunks (`-max_total_time=600`) reusing one corpus
directory per target, `-seed=4200+chunk`. Rule: stop when a whole chunk adds no edge coverage after
at least 600 CPU-seconds, cap 7,200; no crash. CPU seconds are user plus system time of the
fuzzer process (`/usr/bin/time`), queue waits not counted; the cap is checked between chunks, so
the last chunk may end above it. The fuzz binaries were copied to unique names
(`navmesh012-parallel_bake`, sha256 `17a4495e9b8f…`; `navmesh012-recast_input`, sha256
`d2e53cb54236…`) and run from there, with coverage built in a separate target directory.

Targets: `parallel_bake` (new) and `recast_input`. `tilecache_update`, `crowd_update` and
`tile_add_query` do not reach the changed Recast code: the tile cache has its own contour and mesh
builders, and the other two read and query finished tiles. A seed-corpus coverage run of
`tilecache_update` executed two of the 123 added charge lines (both in
`detour::create_nav_mesh_data`) and none in `recast/`.

### Reachability first

`parallel_bake` as committed could not reach most of the change. It built no area volumes, triangle
areas, aggregate budget or tight limits, and its seeds did not exist. The target now takes the
mesh, tile size, partition, radius, polygon size, cell size, filters, triangle areas, up to three
area volumes, per-tile limits and the aggregate budget from the input (layout in the target
source). Under an aggregate budget the serial bake reserves for one worker and the parallel bake
for the rayon pool, so an aggregate rejection of only the parallel bake is not counted as a
mismatch; every other difference, including the error and its tile, still fails. Seeds:
`seeds parallel` (110 files: six meshes, three partitions, six feature sets, two raw-float inputs),
`seeds recast` with new `-border` inputs (72 files in all) and `seeds recast-grid` (two meshes).

The first seed-only coverage run found three gaps, each closed in the seeds or the target:

- Area volumes were never applied. The target built one area byte per triangle index instead of
  per triangle, so every input with areas failed validation. Fixed in the target.
- Hole merging in `ContourSet::build` and border-vertex removal in the polygon mesh
  (`can_remove_vertex`, `remove_vertex`) were not reached. Added planes with interior holes
  (`seeds parallel`) and `-border` meshes without erosion (`seeds recast`).
- The 16-bit edge guard of `build_mesh_adjacency` needs more than 32,767 polygons in one mesh. Added
  `seeds recast-grid`: 5,041 plus-shaped islands, one cell wide, five cells apart, each
  triangulated into ten polygons with `nvp = 3`. Both seeds end in `LimitExceeded` with unlimited
  limits as well, so it is the guard and not a fuzz limit.

Region coverage of the final corpora (`cargo fuzz coverage` build, corpus replayed once;
regions covered / total, the instance with the most regions per function):

| Function | `parallel_bake` | `recast_input` |
|---|---|---|
| `bake::tiles`, `bake::tiles_parallel` | 6/6, 6/6 | not called |
| `bake::bake`, `bake::prepare` | 39/44, 242/275 | not called |
| `bake::fill_results` (serial and rayon paths) | 13/13 | not called |
| `bake::build_tile_in_workspace` | 84/93 | not called |
| `bake::check_total_bytes` (aggregate budget) | 15/16 | not called |
| `area::mark_convex_poly_area` (volume marking) | 65/71 | not called |
| `contour::merge_contours`, `merge_region_holes` | 22/24, 119/138 | 22/24, 120/138 |
| `contour::simplify_contour` | 209/219 | 213/219 |
| `detail::delaunay_hull`, `triangulate_hull` | 95/125, 76/78 | 109/125, 76/78 |
| `detail::seed_array_with_poly_center`, `get_height_data` | 147/152, 112/115 | 147/152, 112/115 |
| `mesh::can_remove_vertex`, `remove_vertex` | 55/57, 213/231 | 55/57, 213/231 |
| `mesh::merge_polys`, `merge_poly_verts` | 58/59, 32/37 | 58/59, 32/37 |
| `mesh::build_mesh_adjacency` (u16 edge guard) | 90/96 | 91/96 |
| `builder::create_bv_tree` | 45/85 | 45/85 |
| `builder::create_nav_mesh_data` | uses `create_with_limits` | 4/4 |

Added charge and guard lines (every added line in `git diff e557bf5..HEAD -- src` that calls
`ws.reserve`, `push`, `vec`, `vec_with_capacity`, `work`, `charge_bytes` or returns
`LimitExceeded`/`SizeOverflow`, outside test modules; 123 lines), executed at least once by the
two final corpora: 110 of 123. By file: `recast/area.rs` 6/6, `recast/detail.rs` 8/8,
`recast/region.rs` 11/11, `recast/contour.rs` 10/11, `recast/mesh.rs` 16/17, `detour/builder.rs`
6/7, `bake.rs` 53/63. The u16 edge guard (`mesh.rs:882`) ran twice, both on the grid seeds. The
aggregate budget rejected 79 bakes (`bake.rs:835`); the new volume bucketing ran 11,534 times and
its work charge in `mark_convex_poly_area` 251,580 times.

Not reached (13 lines): nine `SizeOverflow` branches that cannot fire for finite validated inputs
on a 64-bit target (`bake.rs:315, 337, 356, 389, 424, 567, 679, 713`; `mesh.rs:568`, an
`nvp > 64` allocation the public builder refuses first); `bake.rs:827` (a finite aggregate budget
with unlimited tile bytes, which the target never builds); `bake.rs:721` and `builder.rs:467`
(`max_tile_bytes` and the tile format limits, which the target leaves unset); `contour.rs:725`
(growth of the diagonal scratch buffer when a later hole has more candidates than the first).
The unit tests of the PR cover the budget lines directly.

### `parallel_bake`

Seeds: 110 files from `seeds parallel`, `-max_len=4096`.

| Chunk | CPU s | Executions | Coverage start (cov / ft) | Coverage end | Crashes |
|---|---|---|---|---|---|
| 1 | 1,694 | 5,954 | 4958 / 20481 (110 seeds) | 5766 / 35781 | 0 |
| 2 | 1,707 | 4,545 | 5755 / 35425 | 5788 / 37090 | 0 |
| 3 | 1,800 | 3,203 | 5791 / 37018 | 5804 / 37258 | 0 |
| 4 | 1,753 | 2,735 | 5802 / 37204 | 5803 / 37361 | 0 |
| 5 | 1,703 | 4,320 | 5781 / 37298 | 5790 / 37719 | 0 |
| Total | 8,657 | 20,757 | cov 4958 | cov 5790 (peak 5804) | 0 |

The run stopped at the cap, not on a flat chunk: chunk 4 added one edge, and chunk 5 ended below
chunk 4 (edge counts of this target are not reproducible to a few edges: chunk 4 ended at 5803,
and chunk 5 loaded the same corpus at 5781). Four chunks were 6,954 CPU-seconds; chunk 5 started below the cap
and ended at 8,657. Chunks use 2.8 CPUs on average (four rayon workers plus the serial bake). Final
corpus: 2,557 inputs, 1.26 MB. Peak RSS 668 MB. No crash; no libFuzzer timeout.

Two slow-unit notices were written: 613-byte raw-float inputs that differ in one byte. Five
vertices span about 97 units at cell size 0.1 with tile size 49 and radius 4, so the grid is about
20 x 20 tiles of 63 x 63 cells (derived from the vertex bounds), and 128 large triangles overlap
most tiles. Alone, in the fuzz binary: 22.8 s wall (59.7 CPU-s) and 26.4 s wall (75.0 CPU-s)
against the 60-second limit; both passed in the runs, which bake the grid twice (serial and
parallel). That is 44% of the limit at worst, not "well under" it, so a heavily loaded machine
could turn one into a timeout; the cost is many large triangles per tile, not one slow stage.

### `recast_input`

Seeds: 72 files from `seeds recast` (12 corpus meshes, six settings each; the 1024 m terrain is
too large for a seed), `-max_len=1048576`. The two `seeds recast-grid` files were added to the corpus
after chunk 7, which had been flat; the rule was then applied again to the chunks that followed
(flatness is measured against the previous chunk's end).

| Chunk | CPU s | Executions | Coverage start (cov / ft) | Coverage end | Crashes |
|---|---|---|---|---|---|
| 1 | 602 | 11,620 | 6244 / 23450 (72 seeds) | 6684 / 33626 | 0 |
| 2 | 600 | 5,225 | 6684 / 33626 | 6705 / 34276 | 0 |
| 3 | 597 | 3,392 | 6705 / 34276 | 6720 / 34530 | 0 |
| 4 | 593 | 2,407 | 6720 / 34530 | 6734 / 34693 | 0 |
| 5 | 597 | 3,293 | 6734 / 34693 | 6736 / 34903 | 0 |
| 6 | 571 | 2,048 | 6736 / 34903 | 6737 / 34982 | 0 |
| 7 | 670 | 1,091 | 6737 / 34982 | 6737 / 34982 | 0 |
| 8 | 596 | 3,387 | 6754 / 35002 (+2 grid seeds) | 6756 / 35209 | 0 |
| 9 | 589 | 3,395 | 6756 / 35209 | 6763 / 35290 | 0 |
| 10 | 592 | 2,796 | 6763 / 35290 | 6763 / 35390 | 0 |
| Total | 6,007 | 38,654 | cov 6244 | cov 6763 | 0 |

Chunk 10 added no edge, so the run stopped there (6,007 CPU-seconds). Final corpus: 1,198 inputs,
158 MB (the large meshes are seeds). Peak RSS 527 MB. No crash; no libFuzzer timeout.

Timings alone, in the fuzz binary: the one slow-unit notice is a 396,318-byte seed (a corpus
terrain mesh), 1.4 s.

One finding that is not a crash. Replaying the final corpus with the coverage-instrumented build
timed out on a 1,098-byte input (`artifacts/recast_input-cov-timeout/`, saved with the
corpora). Alone, the fuzz binary runs it in 0.04 s; the coverage build takes more than 100 s. The
mesh is a sliver: `calc_grid_size` returns a grid of 0 x 2,147,483,647 cells. `Heightfield::new`
accepts it because zero cells need no allocation, and `check_chf`, `ContourSet::build` and the
other stages then loop over 2^31 rows with an empty inner loop, which no limit charges. Optimised
builds hoist the loop; instrumented and debug builds do not. This code predates the PR. It is not
fixed here (the brief covers crashes and this is not one). The fix is to reject a grid with exactly
one zero dimension in `Heightfield::new` and `check_chf`, with a regression test; it changes the
result for such inputs from an empty mesh to a typed error.

### Corpora

Kept in local scratch storage and copied to network storage (`corpus/`, `seeds/`,
`artifacts/`; no symlinks, so no archive). Verified by file count, total bytes, and the SHA-256 of
the sorted `path size` list (and of all file contents), source against copy: `corpus/parallel_bake`
2,557 files, 1,257,428 bytes, list hash `002bb59214eb…`; `corpus/recast_input` 1,198 files,
158,035,746 bytes, list hash `a2c2b6085b91…`; seeds and artifacts match likewise.

## Budgeted queries, resumable searches, global off-mesh links, 2026-10-31

Commit under test: `ff8debd` (`feat/determinism-budgets`) plus the fuzz-only changes that add this
receipt. Toolchain: nightly, `cargo-fuzz` 0.13.2, one libFuzzer worker per target, 600-second
chunks (`-max_total_time=600`) reusing one corpus directory per target, `-print_final_stats=1`.
Rule as above: stop when a whole chunk adds no edge coverage, after at least 600 CPU-seconds and
at most 7,200; no crash. CPU seconds are user plus system time of the whole `cargo fuzz run`
command, so the first chunk of each target includes its incremental build.

### `budgeted_query` (new)

One flag byte, then up to 48 operations of 4 bytes (layout in the target source) on a row of six
single-polygon tiles: add a tile with an optional off-mesh connection to another tile, remove a
tile, toggle global off-mesh links, run every budgeted query under a budget of 0 to 199 (200 and
above is unlimited in effect), or run each resumable search to the end in steps of the given
budget after a zero-budget step. Properties asserted: a `Complete` outcome equals the unbudgeted
call (result and output buffers); a large budget never exhausts; a budget is never overspent; a
zero-budget step spends nothing; an `Exhausted` step has recorded a refusal; a resumed search,
run to `Done`, equals the unbudgeted query (circle, shape, wall distance, local neighbourhood,
surface move, raycast; results and status).

Seeds: 21 files written by `seeds budget` (scripted tile loads with queries at budgets 0, 1, 2, 3,
5, 9, 20 and 255, single-shot and resumed, with and without global links; one add, remove and
toggle sequence; 16 random). `-max_len=200`. The seeds alone ran clean (22 executions).

Reach before the long run, from `cargo fuzz coverage` over the 21 seeds, `llvm-cov` line coverage of
the new code: `resumable.rs` 83 percent of lines; every `*_budgeted` method in `query.rs`,
`PolysAroundSearch`, `DistanceToWallSearch`, `LocalNeighbourhoodSearch`, `SurfaceMoveSearch` and
`RaycastSearch` `resume`/`begin_*` (except `begin_polys_around_shape`, which the seeds missed and
which the target now also drives), and `set_global_off_mesh_links`, `connect_global_off_mesh_links`
and `disconnect_global_pairs` in `navmesh.rs` all executed. On the final corpus (1,310 inputs):
`resumable.rs` 89 percent of lines, `begin_polys_around_shape` reached, `src/budget.rs` 58 percent
(the rest is `Budget` accessors and `Limits`-based constructors the target does not call).

| Chunk | CPU s | Executions | Coverage start (cov / ft) | Coverage end | Crashes |
|---|---|---|---|---|---|
| 1 | 681 | 734,648 | 2514 / 10368 (21 seeds) | 2557 / 15803 | 0 |
| 2 | 680 | 831,378 | 2557 / 15803 | 2557 / 15884 | 0 |
| Total | 1,361 | 1,566,026 | cov 2514 | cov 2557 | 0 |

Chunk 2 added no edge, so the run stopped there. Final corpus: 1,310 inputs, 5.3 MB. Peak RSS 461
MB. No artifact files; slowest unit under 1 s.

### `crowd_update` (extended)

Operation 6 is now `Crowd::update_budgeted` with a budget from the two tail bytes of the record
(0 to 3 as is, larger values times 256, so 0 and 1 refuse the up-front charge and values around
128 to 1,000 reach later phases). It asserts the budget is not overspent and that `Exhausted`
recorded a refusal. Seeds: 73 files from `seeds crowd`, which now also writes a mix with budgeted
ticks. `-max_len=1024`.

| Chunk | CPU s | Executions | Coverage start (cov / ft) | Coverage end | Crashes |
|---|---|---|---|---|---|
| 1 | 680 | 151,177 | 2083 / 8289 (73 seeds) | 2228 / 11672 | 0 |
| 2 | 677 | 185,485 | 2228 / 11672 | 2228 / 11706 | 0 |
| Total | 1,357 | 336,662 | cov 2083 | cov 2228 | 0 |

Chunk 2 added no edge. Final corpus: 748 inputs, 3.0 MB. Peak RSS 437 MB. No crash, no artifact
files.
