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

Both have seed writers:

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
