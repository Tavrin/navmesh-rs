# Changelog

All notable changes to this project are documented in this file. The format follows
[Keep a Changelog](https://keepachangelog.com/en/1.1.0/), and the project uses
[Semantic Versioning](https://semver.org/).

## [Unreleased]

### Changed

- The bake pipeline validates a compact field once, where it builds it, instead of
  scanning the whole grid again at every stage; the work budget is charged as before.
  Public stage functions still validate compact fields they are given.
- Validate sparse single-layer compact fields with the packed row pass in every stage
  entry point (unlimited budgets only).
- Replace library `floorf`/`ceilf` calls in span insertion and detail sampling with
  integer floor and ceiling of identical value.
- Cache each node's total in the A* open list so heap operations do not index the node
  pool. Results are unchanged.

## [0.1.4] - 2026-11-01

### Added

- World-space float tile import with `create_nav_mesh_data_f32`; vertex floats are
  preserved without a uniform-grid round trip.
- `find_path_with_heuristic_scale`, including zero for Dijkstra searches with
  zero-cost links or costs below one.
- `PolyMesh::build_with_report`, typed partial-triangulation errors in strict
  mode, and per-tile triangulation diagnostics in the bake results.
- Matching 200-pair path/funnel/nearest-query benchmarks in both record drivers.

### Changed

- Specialize path and nearest-polygon queries for concrete filters while retaining
  support for trait-object filters.
- Stream polygon candidates to query callbacks without a temporary allocation.
- Swap rasterization buffer references directly, preserving clipping order.
- Validate dense triangle meshes with contiguous coordinate and index scans;
  preserve referenced-only validation for sparse tile subsets.
- Validate dense single-layer compact fields with bit-field scans, retaining
  generic validation and finite work-budget traversal.
- Resolve detail-triangle corners directly in nearest-polygon height queries.

## [0.1.3] - 2026-10-31

### Added

- Opt-in global off-mesh links (`NavMesh::set_global_off_mesh_links`, not in recastnavigation):
  a connection resolves its far endpoint in any loaded tile, relinked on tile removal and
  re-addition in either order. Default behaviour is unchanged.

- Work budgets on every query (not in recastnavigation): `Budget`, `Budgeted` and `Progress`;
  `find_path_budgeted`, `find_straight_path_budgeted`, `query_polygons_budgeted`,
  `find_nearest_poly_budgeted`, `find_random_point_budgeted`,
  `find_random_point_around_circle_budgeted` and `Crowd::update_budgeted`; resumable
  `begin_polys_around_circle`/`_shape`, `begin_distance_to_wall`, `begin_local_neighbourhood`,
  `begin_move_along_surface` and `begin_raycast`. Unbudgeted calls are unchanged and share the
  per-polygon code with the budgeted ones.
- Cross-platform determinism contract with the `libm` feature: every recorded output is identical
  on x86-64 Linux, AArch64 Linux and wasm32. `reference/determinism.golden` and
  `reference/determinism.sh` check it; CI runs the check on all three targets.
- `nav-port digest` and `nav-port budget` in the parity crate; `fetch.sh --public-only`.

## [0.1.2] - 2026-10-31

- Reject grids with exactly one zero dimension with a typed error instead of
  returning an empty mesh, including sliver meshes with a saturated dimension.
  Validate mutated public grid dimensions and charge empty-row traversal work.

- Add the `bake` extension (not in recastnavigation): serial tiled generation,
  opt-in rayon tile workers through `parallel`, deterministic tile/error ordering,
  independent per-tile limits, and serial navmesh assembly. The API remains
  available without rayon, including `no_std`/wasm builds.
- Fuzzing: `parallel_bake` also varies triangle areas, area volumes, per-tile limits and
  the aggregate budget; new `seeds parallel` and `seeds recast-grid` modes and `-border`
  Recast seeds reach the new allocation charges, the volume marking and the edge-count guard.

## [0.1.1] - 2026-10-30

### Added

- Opt-in global off-mesh links (`NavMesh::set_global_off_mesh_links`, not in recastnavigation):
  a connection resolves its far endpoint in any loaded tile, relinked on tile removal and
  re-addition in either order. Default behaviour is unchanged.

- Crowd simulation: corridor maintenance, steering, separation, local avoidance,
  path queues and off-mesh animation.
- Tile cache with cylinder, box and oriented-box dynamic obstacles and incremental
  navmesh rebuilding.
- A pluggable layer compressor trait with raw storage provided by default.
- Crowd and tile-cache seed modes for the fuzz corpus generator.
- Non-exhaustive crowd parameters and state enums, with defaults and avoidance
  quality presets for construction.

### Fixed

- Clear animation, movement targets and cached state when reusing crowd slots.
- Use `as_chunks` for complete 16-byte records in the crowd fuzz target, fixing
  Clippy on current toolchains.
- Gate integration tests and the build example on their required features.

### Deliberate differences from C++

- Validate inputs and references, report exhausted capacities and work budgets,
  reset reused slots, require positive agent speeds and reject out-of-range cells.
- Define adaptive avoidance for even division counts, where upstream reads
  uninitialized samples; no C++ parity is claimed for those counts.
- Validate tile-cache layers and codec output, keep tile membership stable during
  updates, reject touched-layer overflow and retain all queued rebuilds.
- Complete zero-touch obstacle lifecycles, charge allocation and algorithm work,
  and retain failed rebuilds for retry without rolling back completed work.
- Reserve full vertex triples in tile-cache removal instead of reproducing an
  upstream buffer overrun. See [verification](docs/VERIFICATION.md) for the full
  differences and parity boundaries.

## [0.1.0]

### Added

- Opt-in global off-mesh links (`NavMesh::set_global_off_mesh_links`, not in recastnavigation):
  a connection resolves its far endpoint in any loaded tile, relinked on tile removal and
  re-addition in either order. Default behaviour is unchanged.

- Recast navmesh generation, including all three partitions, area volumes and
  heightfield layers.
- Detour tile creation, tiled navmeshes, off-mesh connections and navmesh queries
  with 32-bit or 64-bit polygon references.
- Per-call memory and work limits with typed errors.
- `no_std` with `alloc` and `wasm32` support.

[Unreleased]: https://github.com/Tavrin/navmesh-rs/compare/v0.1.4...HEAD
[0.1.4]: https://github.com/Tavrin/navmesh-rs/compare/v0.1.3...v0.1.4
[0.1.3]: https://github.com/Tavrin/navmesh-rs/compare/v0.1.2...v0.1.3
[0.1.2]: https://github.com/Tavrin/navmesh-rs/compare/v0.1.1...v0.1.2
[0.1.1]: https://github.com/Tavrin/navmesh-rs/compare/v0.1.0...v0.1.1
[0.1.0]: https://github.com/Tavrin/navmesh-rs/releases/tag/v0.1.0
