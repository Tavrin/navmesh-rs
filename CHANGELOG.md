# Changelog

All notable changes to this project are documented in this file. The format follows
[Keep a Changelog](https://keepachangelog.com/en/1.1.0/), and the project uses
[Semantic Versioning](https://semver.org/).

## [Unreleased]

### Added

- `UpstreamFixes`, a bit set of opt-in corrections for wrong but defined upstream behaviour.
  `UpstreamFixes::NONE` is the default everywhere and reproduces upstream exactly. It is
  carried by `BakeConfig::fixes`, `NavMeshQuery::set_fixes`, `PathQueue::set_fixes`,
  `Crowd::set_fixes` and `ObstacleAvoidanceQuery::set_fixes`. Flags:
  - `SLICED_CROSS_SIDE`: sliced A* keys nodes by (polygon, cross side) like the plain
    search, so both return the same corridor on tiled meshes.
  - `NEAREST_NEIGHBOURS`: the crowd keeps the nearest neighbours of an agent instead of the
    nearest of the first 32 candidates in grid order.
  - `ADAPTIVE_PATTERN`: adaptive velocity sampling with an even division count writes its
    last ring sample in the right slot.
  - `ZERO_SAMPLE_DIST`: a detail sample distance of zero builds the detail mesh from the
    polygon hull instead of leaving polygons without detail triangles (upstream loops
    forever there). Use `PolyMeshDetail::build_with_fixes`.
- `CrowdAgent::neighbours`, the neighbours kept by the last update.
- `nav-port --fixes` writes corrected records; `reference/determinism-fixes.golden` holds
  the digests that change with every correction on.
- Upstream issue 729 (ledge-filter rewrite) was analysed and is not a defect; the evidence
  is in `reference/UPSTREAM.md`.
- `PolyMesh::merge`, `PolyMesh::copy` and `PolyMeshDetail::merge`
  (`rcMergePolyMeshes`, `rcCopyPolyMesh`, `rcMergePolyMeshDetails`), compared with the
  C++ driver on the corpus: a copy of the first mesh, and merges of all tile meshes and of
  the tiles in the 2 x 2 block at the grid origin. Inputs upstream would read out of bounds
  (a vertex index past the mesh, differing `nvp`, a result over 65,535 vertices or polygons)
  are typed errors.
- `NavMesh::tile_state_size`, `store_tile_state` and `restore_tile_state`
  (`getTileStateSize`, `storeTileState`, `restoreTileState`): polygon flags and areas per
  tile, byte-identical blobs, untrusted blobs checked completely before the tile changes
  (`Error::InvalidTileState(TileStateError)`: too short, wrong magic, wrong version, other
  tile).
- Big-endian tile data: `detour::nav_mesh_header_swap_endian` and
  `nav_mesh_data_swap_endian` (`dtNavMeshHeaderSwapEndian`, `dtNavMeshDataSwapEndian`) and
  `tilecache::tile_cache_header_swap_endian` (`dtTileCacheHeaderSwapEndian`). Output is
  byte-identical to upstream's on every corpus tile and layer; the counts in a header are
  checked against the buffer before anything is swapped. `nav_mesh_swap_off_mesh_user_ids`
  (not in recastnavigation) converts the off-mesh `user_id` that upstream's body swap skips.
- `Heightfield::rasterize_triangles_u16` and `rasterize_triangle_soup` (the other two
  overloads of `rcRasterizeTriangles`), `detour::detail_tri_edge_flags`
  (`dtGetDetailTriEdgeFlags`), and `Crowd::active_agents`, `query_half_extents`, `grid` and
  `path_queue` (`getActiveAgents`, `getQueryHalfExtents`, `getGrid`, `getPathQueue`).
- Record kinds `poly_mesh_copy`, `merged_poly_mesh[_block]`, `merged_detail_mesh[_block]`,
  `tile_state`, `tile_endian` and `layer_endian` in the oracle driver, the port driver and
  the comparator; fuzz targets `tile_state` and `tile_endian`.
- `docs/VERIFICATION.md` lists the upstream public API that is intentionally not ported.
- Tile streaming with stable polygon refs (not in recastnavigation):
  - `SlotPolicy` and `NavMesh::with_policy`: `Sequential` (upstream's free list, the
    default) or `Keyed`, where tile (x, y, layer) always takes the same slot of a grid, so
    refs do not depend on load order or session. Keyed slot pages are allocated as tiles
    reach them.
  - `prepare_tile` parses, validates, hashes and builds internal links on any thread (`PreparedTile`);
    `NavMesh::commit`, `commit_budgeted` (all or nothing against `commit_cost`, returns
    `Committed`) and `commit_with_state` link it on the owner thread.
  - `NavMesh::unload_tile` evicts a tile but keeps its slot and salt and returns
    `UnloadedTile` (bytes, tile state, `content_hash`). Committing the same content makes
    every ref into it valid again; other content increments the salt. `forget_unloaded`
    drops the reservation.
    The unload state carries a versioned hash footer, so `commit_with_state` rejects
    other content even in a fresh mesh with a matching initial ref. The ordinary
    `store_tile_state` format is unchanged.
  - `NavMesh::ref_state` returns `RefState::{Valid, Unloaded(TileKey), Stale, Invalid}`;
    a stale ref never becomes valid again (except through `restore_tile_allow_stale`).
  - `NavMesh::mark_known`, `is_column_unloaded`, `unloaded_column_at`, `has_unloaded`;
    `NavMeshQuery::find_path_report` returns the unloaded columns the search reached, and
    `find_path` sets the new `Status::UNLOADED_BORDER` with a partial result there.
    `RaycastHit::edge` (`EdgeKind::{None, Wall, UnloadedBorder, WorldEdge}`).
  - `NavMesh::epoch` and `changed_since`, a structural change counter with a log of the
    last `REMOVAL_LOG_CAPACITY` (256) removals. Resumable searches and sliced A* now fail
    with `Error::MeshChanged` (`Status::MESH_CHANGED`) when a tile they touched was
    unloaded or removed since the last step, instead of reading stale refs; additions are
    allowed.
  - `NavMesh::residency` / `restore_residency` and `ResidencyRecord::{to_bytes,
    from_bytes}` (validated untrusted blob): refs saved with a record stay valid in a later
    session when the same tiles are committed.
  - Crowd: `TargetState::WaitingForTile` and `CrowdAgent::waiting_tile`. An agent whose
    target or intermediate corridor tile is unloaded follows the resident part of its corridor to the
    border and replans when the tile returns; `request_move_target` accepts an unloaded
    target.
  - `TileCache::unload_tile` (allowed with obstacle associations) and `load_tile`
    (re-associates live obstacles over the layer and queues its rebuild).
  - `nav-port` with `NAV_PORT_STREAMED=1` unloads and recommits every tile before the
    queries; fuzz targets `stream_ops`, `prepared_tile`, `residency_record`.

### Fixed

- `NavMeshQuery::portal_points` (and so `find_path` and the other searches that take a
  portal) indexed a polygon's six vertices with the edge of a link read from tile bytes. A
  tile that the reader accepts but whose BV tree reaches a polygon typed as an off-mesh
  connection gets a landing link with edge 255 on it, and the lookup panicked with an
  out-of-bounds index. Found by the new `tile_endian` fuzz target; the same tile bytes
  reached it in 0.1.0 to 0.1.5. The lookup now ends with a typed error. Results for every
  valid tile are unchanged.

### Changed

- `Error` has a new variant, `InvalidTileState` (the enum is `#[non_exhaustive]`).
- Breaking: `RaycastHit` is `#[non_exhaustive]` and has a new field `edge`; build values
  with `RaycastHit::default()`.
- `Error` has new variants `TileOutsideGrid` and `MeshChanged`; `TargetState` has
  `WaitingForTile`; `Status` has detail bits `UNLOADED_BORDER` and `MESH_CHANGED`.
- Resumable searches and sliced A* check the navmesh between steps (see above). Before,
  resuming after a tile removal was documented as unsupported and read stale refs.
- `NavMesh` keeps each tile behind a `Box` and its slots in pages; `NavMesh::with_limits`
  is `with_policy(.., SlotPolicy::Sequential)`. Refs, links and every parity record are
  unchanged.

## [0.1.5] - 2026-11-03

### Fixed

- Rasterizing a finite triangle that is enormous next to the grid (for example an x extent
  of 1e23) no longer panics with an out-of-bounds index in the polygon clipper. In `f32`
  the clipped polygon stops being convex and outgrows upstream's seven-vertex buffers
  (upstream writes out of bounds there); the clipper now checks every write and
  `rasterize_triangles` / `rasterize_triangle` return `Error::InvalidInput`. Output for
  every input that fits the buffers is unchanged. Affects 0.1.0 to 0.1.4.

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

[Unreleased]: https://github.com/Tavrin/navmesh-rs/compare/v0.1.5...HEAD
[0.1.5]: https://github.com/Tavrin/navmesh-rs/compare/v0.1.4...v0.1.5
[0.1.4]: https://github.com/Tavrin/navmesh-rs/compare/v0.1.3...v0.1.4
[0.1.3]: https://github.com/Tavrin/navmesh-rs/compare/v0.1.2...v0.1.3
[0.1.2]: https://github.com/Tavrin/navmesh-rs/compare/v0.1.1...v0.1.2
[0.1.1]: https://github.com/Tavrin/navmesh-rs/compare/v0.1.0...v0.1.1
[0.1.0]: https://github.com/Tavrin/navmesh-rs/releases/tag/v0.1.0
