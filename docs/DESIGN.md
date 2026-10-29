# Design: a pure-Rust port of recastnavigation

This is the design the port follows. Recast and Detour are ported; DetourCrowd, DetourTileCache,
the bake API and the extensions below are not yet. Where this document describes something not
built yet (bake API, `'MSET'` sets, big-endian tile reading, `parallel`, `serde`), it is the plan,
not the current API; the README and the crate documentation describe what exists, and
`docs/VERIFICATION.md` what is checked.

## 1. Goal and scope

Port all of [recastnavigation](https://github.com/recastnavigation/recastnavigation) to safe Rust:
Recast (navmesh generation), Detour (runtime navmesh and queries), DetourCrowd and
DetourTileCache. The C++ library is the reference. Differential tests on a public corpus show the
port produces the same bytes and the same query results. After parity, opt-in extensions go beyond
upstream while default behaviour stays identical to it. Licence: MIT OR Apache-2.0 for our code,
with the upstream zlib notice kept (section 10).

Not in scope: the RecastDemo application, DebugUtils drawing, a C API, engine bindings. Those can
come later as separate crates.

## 2. Reference

Details in [`reference/UPSTREAM.md`](../reference/UPSTREAM.md). Summary:

- **Pin:** `main` at `9f4ce64458dfae86e1239c525ddc219c4e9e06f1` (2026-02-27). The latest release,
  `v1.6.0` (`6dc1667f`, 2023-05-21), is 35 library commits older; those commits include crash and
  out-of-bounds fixes and behaviour changes (detail-mesh edge detection, layer-region areas,
  invalid-span handling, ledge filter). Porting the release would mean porting known bugs.
- **Size:** about 27,400 lines of C++ in the four libraries (Recast 10.5k, Detour 9.2k, Crowd 4.2k,
  TileCache 3.5k, headers included).
- **Formats:** navmesh tile `'DNAV'` v7, tile state `'DNMS'` v1, tile-cache layer `'DTLR'` v1, all
  native-endian raw structs with 4-byte section alignment. Multi-tile set files (`'MSET'`,
  `'TSET'`) are demo conventions.
- **Determinism properties** found while building the oracle: upstream output depends on the libc
  `qsort` (three comparators tie on distinct items), on FMA contraction, and on libm at four
  `cosf`/`sinf` sites; sliced and unsliced A* return different corridors on tiled meshes.
  Everything else is index-ordered and reproducible.
- **Threading:** none upstream. Tiles can be built concurrently; `dtNavMesh` mutation, queries,
  crowd and tile cache are single-threaded objects.
- **Known issues:** upstream trusts tile bytes (eleven recent reports of out-of-bounds reads and
  writes in `addTile` and linking); proximity-grid overflow far from the origin; an infinite loop
  with `detailSampleDist == 0`; crowd degradation past 32 neighbours.

## 3. Landscape

Data gathered 2026-10-28 from crates.io, GitHub and the repositories (cloned where noted).

| Project | What it covers | State | Licence | What we take from it |
|---|---|---|---|---|
| [rerecast](https://github.com/janhohenheim/rerecast) | Recast generation only (watershed partition, contours, poly mesh, detail mesh); Bevy integration and an editor in sibling crates; no Detour | active, 0.5.0 (2026-10-26), 173 stars, ~59k downloads | MIT OR Apache-2.0 | its stage-by-stage comparison against C++ JSON dumps (`tests/cpp_comparison.rs`) is the right test shape; within 2-9% of C++ instructions on our solo bakes, but its counts diverge from C++ on the large terrain (see measurements); `no_std` via `libm` feature |
| [recast-rs](https://github.com/wowemulation-dev/recast-rs) (wowemulation-dev) | Recast, Detour, Crowd, TileCache, "dynamic" navmesh, CLI; 61.7k lines; WASM | active since 2026-01, 10 stars, not on crates.io under that name | MIT OR Apache-2.0 | proof that a full port is a few-month effort; its parity evidence is counts only (polys, vertices, detail tris against a C++ CLI), which is the gap our byte-level records close |
| [oxidized_navigation](https://github.com/TheGrimsey/oxidized_navigation) | Bevy plugin: tiled runtime generation from colliders, async rebuild, A* + string pulling | archived; README marks it deprecated in favour of rerecast | MIT OR Apache-2.0 | dirty-tile async rebuild driven by collider changes is the integration shape engines want |
| [landmass](https://github.com/andriyDev/landmass) | Agent navigation system over given navmeshes: islands with transforms, path finding, steering, local avoidance; `landmass_rerecast` bridge | active, 0.9.3 (2026-10-11) | MIT OR Apache-2.0 | engine-agnostic agent API and multi-island worlds; a likely consumer of a Detour-compatible crate |
| [polyanya](https://github.com/vleue/polyanya) / [vleue_navigator](https://github.com/vleue/vleue_navigator) | Any-angle optimal path finding (Polyanya, IJCAI 2017) on 2D/layered meshes; mesh building from obstacle outlines (CDT); Bevy plugin | active, 0.18.0 / 0.17.0 (2026-10-27), ~525/500 stars | MIT OR Apache-2.0 | optimal any-angle search is a candidate extension query (section 8) over Detour polygons |
| [navmesh](https://github.com/PsichiX/navmesh) | Simple navmesh/grid A* | unmaintained since 2021 | MIT/Apache metadata | nothing beyond a reminder that small scope stays small |
| [recastnavigation-sys](https://github.com/andriyDev/recastnavigation-rs-sys), [recast-rs bindings](https://github.com/andriyDev/recast-rs) | Raw FFI to the C++ library | last release 2023-2024 | MIT (repo) | none for the port; FFI is what we are replacing |
| [recastnavigation-rs](https://github.com/SlimeYummy/recastnavigation-rs) | Binding over a deterministic fork of the C++ library (Recast, Detour, Crowd) | active 2026-01 | MPL-2.0 | confirms demand for cross-platform determinism (lockstep games) |
| [DotRecast](https://github.com/ikpil/DotRecast) (C#), [recast4j](https://github.com/recast4j/recast4j) (Java) | Full ports with additions (dynamic voxel navmesh, jump-link extraction) | active | Zlib | how long-lived ports diverge; ideas for extensions; not used as references |

Takeaways: no Rust project covers all four libraries with reference-level evidence. rerecast is the
strongest generation-only port; recast-rs is the broadest but checks counts, not bytes. Our
differentiator is the oracle: byte-exact tiles, exact queries and exact crowd trajectories on a
public corpus, then extensions measured against that baseline.

## 4. Corpus

Details in [`reference/corpus/`](../reference/corpus/README.md). 12 meshes are active: the three
upstream demo meshes (zlib by inclusion), the Godot navigation demo level (MIT), and eight
generated meshes (stairs, slopes around the limit, multi-level with a bridge, narrow corridors,
islands with off-mesh links, tile seams, a 128 m and a 1024 m terrain with 2.1M triangles). Each
has source, licence, SHA-256 and feature tags in `MANIFEST.toml`; `fetch.sh` downloads at pinned
commits, generates, and checks hashes; nothing is committed. Rejected: Crytek Sponza (CRYENGINE
licence agreement) and GPL game maps. Candidate: Amazon Lumberyard Bistro (CC BY 4.0). Since
added: tile-cache obstacle scripts, convex area volumes, degenerate inputs (19 cases in all).

## 5. Reference harness

Details in [`reference/RECORDS.md`](../reference/RECORDS.md). `reference/harness/nav_oracle.cpp`
compiles the pinned upstream sources and, per case file, writes one record per stage (geometry,
config, triangle areas, heightfield before and after filters, compact heightfield, erosion,
distance field, regions, contours, poly mesh, detail mesh, navmesh tile bytes), per query family
(random points, nearest poly, find path, sliced path, straight path in two modes, raycast with
costs, distance to wall, move along surface), and the crowd trajectory over N ticks. Each record
is a canonical little-endian byte stream with counts and an FNV-1a digest; `--dump` writes the
streams for field-level diffs.

The skeleton runs 15 cases (solo watershed, monotone and layer partitions, tiled builds, off-mesh
links, crowds); twelve were run twice and gave identical records. Bit-exactness is required for all
generation stages, tile bytes and queries; the crowd is exact on the oracle platform and gets a
stated tolerance only under the cross-platform `libm` feature, because libm `cosf`/`sinf` feed
the obstacle-avoidance sampler.

## 6. Measurements

Measured on the C++ library with `nav_oracle bench` (per-bake cost is the difference between runs
with 11 and 1 bakes). On a Ryzen 9 7945HX, one thread, demo settings: `nav_test` solo bakes in 279 M instructions (22.5 ms); the tiled dungeon in
289 M (20.8 ms); the 2.1M-triangle terrain in 26.2 G instructions solo (2.24 s, 458 MB) or 38.3 G
tiled (2.67 s, 92 MB). Tile baking on 16 threads is 7.3× faster on the large terrain (369 ms);
bucketing and tile linking stay serial. A query set (nine queries per pair) costs 4.5-46 µs per
pair; the crowd costs 1.8-3.1 µs per agent per tick. rerecast on the same solo bakes uses 2-9%
more instructions and 3-27% more time; its stage counts match C++ on the three upstream meshes
but diverge on the large terrain from the walkable triangles onward (likely the slope threshold
computed in radians).

## 7. Architecture

### Crate layout

One published crate with feature-gated modules, plus unpublished workspace members for tooling:

```
navmesh-rs/               (published crate)
  src/common/             vector math, f32 helpers, sort, status types
  src/recast/             heightfield, filters, compact, area, region (3 partitions),
                          contour, mesh, detail, layers
  src/detour/             tile data (build, validate, bytes), navmesh, node pool, query, filter
  src/crowd/              crowd, corridor, local boundary, obstacle avoidance, proximity grid,
                          path queue
  src/tilecache/          layer build/compress, tile cache, obstacles, layer mesh builder
  src/bake/               high-level solo/tiled bake on top of recast + detour
reference/                oracle harness (C++), cases, corpus scripts (exists now)
parity/                   Rust-side record encoder and differ, used by tests/ (unpublished)
fuzz/                     cargo-fuzz targets (tile bytes, generation inputs, queries)
```

Features: `recast`, `detour`, `crowd` (implies `detour`), `tilecache` (implies `recast` and
`detour`), all on by default; `std` (default); `libm` (cross-platform transcendentals, required
without `std`); `parallel` (rayon tile baking); `polyref64`; `serde` (optional derives on config
types only). One crate rather than four: the modules share math and types, a runtime-only user turns
off `recast` and `tilecache`, and one version number is easier to keep honest. This matches the
meshoptimizer-rs layout we already maintain.

### API shape

Two layers, like upstream:

- **Stage API**, one function per upstream stage, named after it (`Heightfield::new`,
  `rasterize_triangles`, `filter_ledge_spans`, `CompactHeightfield::build`, `erode_walkable_area`,
  `build_regions` / `build_regions_monotone` / `build_layer_regions`, `ContourSet::build`,
  `PolyMesh::build`, `PolyMeshDetail::build`, `TileData::build`). Each takes inputs by reference, a
  `&mut Workspace`, and returns `Result<_, Error>`. This is the surface the differential tests
  exercise stage by stage.
- **Bake API** for most users: `bake::solo(&Input, &BakeConfig, &mut Workspace) -> Result<TileData>`
  and `bake::tiles(&Input, &BakeConfig, tiles, ..) -> Result<Vec<TileData>>`, where `Input` holds
  vertices, triangles, per-triangle areas, convex area volumes and off-mesh links, and
  `BakeConfig` holds world-unit agent settings converted to cells exactly as the demo does.

Detour: `NavMesh` owns tiles (`add_tile(TileData) -> Result<TileRef>`, `remove_tile`), `PolyRef`
is a newtype over `u32` (or `u64` with `polyref64`) with the upstream bit layout. `NavMeshQuery`
borrows a `&NavMesh` and owns its node pools (it is the query workspace). Filters are a trait
(`QueryFilter`) with `DefaultFilter` matching `dtQueryFilter` (flags and 64 area costs).
Upstream's `dtStatus` success-with-details maps to `Ok(T)` where `T` carries the detail
(`partial: bool`, `out_of_nodes: bool`, `truncated: bool`); failures map to `Error`. Output arrays
become caller-provided `&mut Vec` or slices with `_into` forms, like meshoptimizer-rs.

Crowd and tile cache keep upstream's object shape (`Crowd`, `TileCache`) with explicit
`update(dt)`, typed agent and obstacle handles, and borrowed navmesh access (`&mut NavMesh` passed
to `TileCache::update`, so ownership is visible).

### Limits, Workspace, Error

Same contract as meshoptimizer-rs:

- `Limits { max_bytes, max_work }` per call. Generation counts work as span and cell visits;
  queries count node expansions and polygon visits. Detour's `maxNodes` (≤ 65,535 upstream) becomes
  a `NavMeshQuery` construction parameter checked against `max_bytes`.
- `Workspace` retains fallibly allocated scratch between calls (span pools, distance buffers,
  region stacks, contour buffers), reports `Usage { bytes, work }`, and releases under limit
  pressure. One workspace per thread in parallel baking.
- `Error` (non-exhaustive): `InvalidParameter`, `InvalidInput` (NaN/Inf vertices, out-of-range
  indices), `InvalidTileData` (with a reason code), `InvalidRef`, `TileSlotOccupied`,
  `BufferTooSmall`, `SizeOverflow`, `LimitExceeded`, `AllocationFailed`, `FormatLimit` (an upstream
  format limit such as more than 65,535 vertices in a tile or more than 255 layer cells).

Where upstream asserts, crashes or has undefined behaviour, the port returns an error; the record
for that input says so. Panics are bugs; fuzz targets enforce it.

### no_std, alloc, wasm

All modules build with `no_std` + `alloc`. Floating-point needs are `sqrt`, `floor`, `ceil`
(exact IEEE operations everywhere) and `cos`, `sin` at four call sites. With `std` they come
from the platform libm (on Linux glibc, matching the oracle); without `std`, or with the `libm`
feature, from the `libm` crate (a musl port). On `wasm32-unknown-unknown` Rust's own math is already
the musl implementation. No `unsafe` in generation, Detour or crowd; if a SIMD fast path is ever
added it follows the meshoptimizer-rs rule (scalar canonical bytes, documented safety arguments).

### Determinism contract

1. Same input, same crate version, same features: identical output, run to run and at any
   thread count (tiles are assembled in a fixed order).
2. Default features on x86-64 Linux: bit-identical to the C++ oracle built as `RECORDS.md`
   specifies, for every record.
3. Across platforms (x86-64, AArch64, wasm32): generation, tile bytes and queries are identical
   except where the walkable-slope `cos` or an oriented-box obstacle `cos/sin` lands on a different
   last bit; the crowd may diverge after the first avoidance `sin/cos` difference. With the
   `libm` feature, all records are identical across platforms (the C++ oracle comparison then
   carries the tolerance in `RECORDS.md` for crowd records). The slope threshold can also be given
   directly as a cosine to remove that site.
4. Sorting uses stable sorts with upstream comparators, which reproduces glibc's merge-sort
   `qsort`. Where upstream ties make output libc-dependent we follow glibc and document it.
5. No FMA contraction (Rust guarantees it) and no `mul_add`.

### Parallel tile baking

`bake::tiles` with the `parallel` feature: (1) bucket triangles by tile in one pass, in parallel
over triangle chunks, then merged in input order (the per-tile triangle order is part of the
contract because rasterization order affects span merging); (2) build tiles with rayon, each
worker with its own `Workspace` and per-tile `Limits`; (3) collect `TileData` in row-major order;
(4) add tiles to the navmesh serially or with `NavMesh::add_tiles` that connects internal links in
parallel and external links in a second pass over tile pairs. Output bytes equal the serial path
at any thread count; the differential suite runs the tiled cases at 1 and N threads. Incremental
rebake: the same path for a set of dirty tiles, with source hashes per tile so unchanged tiles are
skipped.

### Serialization

- **Tile bytes** are upstream's `'DNAV'` v7 layout, little-endian, byte-for-byte equal to
  `dtCreateNavMeshData` output (this is checked by the `navmesh_tiles` record). Reading validates
  everything upstream trusts: magic, version, section sizes against the buffer, every vertex,
  polygon, link, detail and BV index, off-mesh endpoints, and that counts fit the declared ref
  width. Big-endian upstream data is accepted through an explicit swap function.
- The link section size depends on the poly-ref width (`dtLink` holds a ref), so 32-bit and 64-bit
  tiles are distinct formats; the header carries no flag for it, so the reader takes the width from
  the build feature and rejects size mismatches.
- **Sets:** readers and writers for the demo `'MSET'` and `'TSET'` files for interoperability with
  RecastDemo and other ports, and no new container format of our own until an extension needs
  one (streaming may).
- **Tile cache layers:** `'DTLR'` v1 with a `LayerCompressor` trait; provided implementations: none
  (raw) and FastLZ level 1 (MIT, ported) for demo compatibility.

## 8. Beyond upstream (opt-in, ranked by value for cost)

| # | Extension | Value | Cost | How it is measured |
|---|---|---|---|---|
| 1 | Parallel and incremental tile baking (section 7) | high: bake time is the main wait in authoring tools; 7.3× on the large terrain with the naive driver | low-medium, 3-5 days | wall time at 1/4/8/16 threads on `terrain_large_tiled`; identical bytes at every count; incremental rebake of k dirty tiles costs ~k/N of a full bake |
| 2 | Budgeted queries everywhere | high: frame-time safety for games; upstream budgets only the sliced A* | medium, 4-6 days | every query takes `Limits.max_work`; resumable forms for Dijkstra searches and long raycasts; measured as worst-case work and wall time per call on the query corpus, with results equal to the unbudgeted call when the budget suffices |
| 3 | Cross-platform determinism | high for lockstep and replays; the C++ library cannot offer it (qsort, FMA, libm) | low, 2-3 days (mostly CI) | all records identical on x86-64 Linux, AArch64 and wasm32 with `libm`; one CI job per target |
| 4 | Hierarchical path finding for large worlds | medium-high: long paths on 1 km+ worlds expand many nodes | high, 2-3 weeks | nodes expanded, wall time and path-length ratio versus flat A* on long pairs over `terrain_large`; target ≤ 5% longer paths |
| 5 | ORCA (RVO2-style) local avoidance as an alternative to the sampled velocity planner | medium: smoother crowds, fewer oscillations | medium, 1-2 weeks | overlaps per agent-second, time to goal, deadlocks on standard scenarios (corridor swap, circle crossing, doorway), cost per agent-tick against the 1.8-3.1 µs baseline |
| 6 | Navmesh streaming | medium: Detour already supports add/remove; the gap is an API that loads, validates and links off the main thread and keeps refs stable | medium, 1 week | hitch per tile add/remove, memory per resident tile |
| 7 | Async-friendly incremental baking (resumable bake jobs with cancellation) | medium | low-medium, 3-4 days | longest uninterrupted slice under a work budget |
| 8 | Upstream defect fixes behind a flag (`#650` zero sample distance, `#641` neighbour limit, others found by the oracle) | medium | small each | a record pair: upstream behaviour by default, fixed behaviour under the flag |
| 9 | Any-angle optimal paths (Polyanya) over Detour polygons | low-medium | medium | path length versus straight-path corridors on the query corpus |
| 10 | Origin-relative tiles for very large coordinates (`#421`) | low-medium | high, breaks format compatibility | position error far from the origin on `undulating` shifted further |

The top three bets are 1, 2 and 3: they are cheap relative to their value and none of them changes
default behaviour.

## 9. Order of work

Each step ends with a proof gate before the next one starts.

| Step | Work | Proof gate | State |
|---|---|---|---|
| Oracle | the harness: generation, tile, query, crowd, tile-cache and obstacle records, malformed-tile verdicts; Rust record encoder and differ in `parity/`; corpus | records reproducible run to run; every record kind decoded by the Rust differ; corpus hashes pinned | done (second machine and CI job not yet) |
| Recast | all generation stages, three partitions, layers; stage API; `Workspace`/`Limits` | every Recast stage record bit-exact on every case; instructions within 1.25× of the oracle | done (bake API not yet) |
| Detour | tile data builder and validating reader, navmesh, node pool, all queries, filters | `navmesh_tiles` byte-exact; all query records exact; malformed tile corpus rejected with typed errors; fuzzing without panics | done (`'MSET'` sets not yet) |
| Crowd | crowd, corridor, boundary, avoidance, proximity grid, path queue | crowd records exact for 300 ticks on all crowd cases and all avoidance presets on the oracle platform | not started |
| TileCache | layers, compression trait plus FastLZ, obstacles, incremental rebuild | obstacle-script records produce byte-exact tiles after each update | not started |
| Extensions | in the order of section 8 | each extension's own measurement, and every default-feature record unchanged | not started |

## 10. Licence and attribution

Our code: MIT OR Apache-2.0. The port is a translation of zlib-licensed code, so each source
distribution keeps the upstream zlib notice unaltered (the `NOTICE` file), the README and crate
documentation say plainly that this is an altered, translated version of recastnavigation at the
pinned commit, and credit Mikko Mononen and the recastnavigation contributors. FastLZ (MIT) needs
its notice if ported. Corpus files are not committed; their licences are in the manifest.

## 11. Decisions recorded here

- Oracle pin on `main@9f4ce64`, not `v1.6.0`. Reversal: cheap (one variable, regenerate records).
- One crate with features, not a workspace of four published crates. Reversal: cheap before the
  first release.
- Stable-sort semantics as the definition of upstream's `qsort` ties (glibc). Reversal: cheap,
  but any other choice is platform-specific.
- Tile triangle selection by bordered xz-overlap in input order, not the demo's chunky mesh.
  Reversal: cheap (both sides of the harness).
- Default behaviour matches upstream including its defined-but-questionable results; fixes are
  opt-in. Reversal: per fix.

- 64-bit poly refs by default (`polyref64` on): large worlds need more than 22 tile+poly bits.
  Upstream defaults to 32-bit; turning the feature off gives its format. Reversal: breaking
  after the first release.
- The first release covers Recast and Detour; Crowd and TileCache follow.

## Sources

Upstream repository and issue tracker (cloned at the pin); crates.io API and GitHub API for the
landscape table; the cloned rerecast (`1d5abc74`) and recast-rs (`b90f3730`) sources; glibc's
January 2024 commit restoring merge-sort `qsort` (glibc commit `709fbd3e`, discussed on the
libvirt list); the Khronos glTF-Sample-Assets Sponza metadata (CRYENGINE licence); the ASWF "Links
to Open Assets" page (Bistro, CC BY 4.0); godot-demo-projects `LICENSE.md` (MIT); Polyanya: Cui,
Harabor, Grastien, IJCAI 2017.
