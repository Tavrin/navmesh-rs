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
agent positions, targets, velocities, removals, resets and update intervals.
