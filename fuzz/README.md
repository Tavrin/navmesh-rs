# Fuzz targets

[cargo-fuzz](https://github.com/rust-fuzz/cargo-fuzz) targets (nightly toolchain, libFuzzer).
A panic, an out-of-memory or a timeout in any of them is a bug.

| Target | Input |
|---|---|
| `tile_add_query` | arbitrary bytes as a navmesh tile: the tile reader, a single-tile navmesh, `add_tile` on a tiled navmesh, remove and restore (fresh and stale), then queries |
| `recast_input` | a mesh and settings (layout in `src/lib.rs`, `RecastInput`) through every Recast stage, the tile builder, a navmesh and queries; per-call `Limits` bound memory and work |

Seeds come from the test corpus (see `reference/corpus/`):

```sh
# tiles: from record files written by the port or the reference driver
cargo run --release --bin seeds -- tiles corpus/tile_add_query <records-dir>/*.navrec
# meshes: the corpus OBJ files, with upstream's demo settings
cargo run --release --bin seeds -- recast corpus/recast_input "$NAV_CORPUS_DIR"/*.obj
cargo +nightly fuzz run tile_add_query corpus/tile_add_query -- -max_len=4194304
cargo +nightly fuzz run recast_input corpus/recast_input -- -max_len=1048576
```

Release rule: each target starts from the corpus seeds and runs until its coverage has been
flat for 10 minutes, for at least 10 and at most 120 CPU-minutes, with no crash.

`crowd_update` creates a bounded crowd on a small mesh and applies arbitrary
agent positions, targets, velocities, removals, resets and update intervals
(up to 64 records of 16 bytes).

`tilecache_update` takes `[mode][layer length][layer bytes][16-byte operations]`. The layer
is a raw layer (mutated header and grids) or a grid specification turned into a valid
layer. It is decoded, run through region/contour/poly building under limits, and added to
the cache beside a fixed flat layer; the operations add cylinder, box and oriented-box
obstacles, remove them, and update or drain the rebuild queue.

Operation 6 of `crowd_update` is `Crowd::update_budgeted` with a budget read from the two tail
bytes (0 to 3 as is, larger values times 256); it asserts the budget is never overspent.

`budgeted_query` takes one flag byte (bit 0: global off-mesh links on from the start) and up to 48
operations of 4 bytes `[op, a, b, c]` on a row of six single-polygon tiles: add a tile (optionally
with an off-mesh connection to another tile), remove a tile, toggle global off-mesh links, run
every budgeted query under a budget of 0 to 199 (200 and above is effectively unlimited), or run
the resumable searches in steps of `c` (after a zero-budget step) to the end. Properties: a
`Complete` outcome equals the unbudgeted call, a large budget always completes, a resumed search
equals the unbudgeted query, and a budget is never overspent. Seed writer and run:

```sh
cargo run --release --bin seeds -- budget corpus/budgeted_query
cargo +nightly fuzz run budgeted_query corpus/budgeted_query -- -max_len=200
```

`f32_import` feeds `create_nav_mesh_data_f32` arbitrary float vertices (NaN, infinities, huge
values, out-of-bounds points), polygons, bounds, cell sizes, off-mesh connections and detail
data, then loads an accepted tile and queries it. `heuristic_path` runs
`find_path_with_heuristic_scale` with arbitrary scale bit patterns and area costs (including
zero) on linked tiles; invalid scales must be errors and reachability must not depend on the
scale. `strict_triangulation` builds polygon meshes from arbitrary contours in default and
strict mode and compares the two. `compact_dense` runs a small mesh through the Recast
pipeline under unlimited work (one-pass compact-field validation) and under a finite work
limit (generic validation) and requires identical meshes. Seed writers and runs:

```sh
cargo run --release --bin seeds -- f32 corpus/f32_import
cargo run --release --bin seeds -- heuristic corpus/heuristic_path
cargo run --release --bin seeds -- strict corpus/strict_triangulation
cargo run --release --bin seeds -- dense corpus/compact_dense
cargo +nightly fuzz run f32_import corpus/f32_import -- -max_len=512 -rss_limit_mb=512
cargo +nightly fuzz run heuristic_path corpus/heuristic_path -- -max_len=256 -rss_limit_mb=512
cargo +nightly fuzz run strict_triangulation corpus/strict_triangulation -- -max_len=512 -rss_limit_mb=512
cargo +nightly fuzz run compact_dense corpus/compact_dense -- -max_len=4096 -rss_limit_mb=512
```

`crowd_update` and `tilecache_update` have seed writers:

```sh
cargo run --release --bin seeds -- crowd corpus/crowd_update
cargo run --release --bin seeds -- tilecache corpus/tilecache_update
cargo +nightly fuzz run crowd_update corpus/crowd_update -- -max_len=1024
cargo +nightly fuzz run tilecache_update corpus/tilecache_update -- -max_len=4096
```

Receipts of the runs are in [`../docs/FUZZ-RECEIPTS.md`](../docs/FUZZ-RECEIPTS.md).

`parallel_bake` (enable `--features parallel`) compares bounded serial and rayon
bakes, including the first error and tile coordinates. It builds a mesh, tile
settings, triangle areas, up to three area volumes, tight per-tile limits and an
aggregate byte budget from the input (layout in the target source). Under an
aggregate budget the serial and parallel bakes reserve for different worker
counts, so an aggregate rejection of only the parallel bake is not a mismatch.
Seed writer and run:

```sh
cargo run --release --bin seeds --features parallel -- parallel corpus/parallel_bake
cargo +nightly fuzz run parallel_bake --features parallel corpus/parallel_bake -- -max_len=4096
```

`seeds recast` also writes `-border` inputs (no erosion, a border), which reach
the polygon mesh's border-vertex removal. `seeds recast-grid <dir>` writes two
meshes of 5,041 plus-shaped islands that the polygon mesh turns into about 50,000
polygons, past the 16-bit edge numbering of the adjacency builder; they are the
seeds that reach its `LimitExceeded` guard.

`incremental_bake` builds a small editable world (triangles, area volumes, off-mesh links)
and a list of edits from the input (layout in `src/bake_world.rs`), bakes it with a
`BakeJob`, rebakes only the tiles `dirty_from_boxes` and `skip_unchanged` select, and
requires the merged tiles and manifest to equal a full bake of the edited world.
`bake_job_steps` (enable `--features parallel`) drives a job over the same kind of world
with data-chosen step budgets, a cancel at a data-chosen step and the parallel driver at 1,
4 and 16 threads, and compares every result with `bake::tiles`: same bytes and order, no
tile after a cancel, delivered tiles equal the reference prefix. Seeds and runs:

```sh
cargo run --release --bin seeds -- bake-job corpus/bake_job
cargo +nightly fuzz run incremental_bake corpus/bake_job -- -max_len=1024
cargo +nightly fuzz run bake_job_steps --features parallel corpus/bake_job -- -max_len=1024
```
