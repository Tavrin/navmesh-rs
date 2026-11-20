# Changelog

All notable changes to this project are documented in this file. The format follows
[Keep a Changelog](https://keepachangelog.com/en/1.1.0/), and the project uses
[Semantic Versioning](https://semver.org/).

## [Unreleased]

### Added

- `Selector::DetailPrism` selects ground polygons by intersection with their
  Detour detail triangles, including sloped-face contact within roundoff.
- Modifier-managed polygons accept caller flag and area edits while preserving
  base values for fields owned by enabled effects. New selections retain
  existing caller edits. Base overrides can be inspected and removed and are
  pruned when their tile content is retired.
- DNDY version 1 snapshots still load. Selected polygons in a restored layer now
  accept caller flag and area writes subject to modifier ownership. Saving a layer
  with a caller base override or detail selector emits version 2.
  See [Dynamic modifiers](docs/DYNAMIC-MODIFIERS.md).

## [0.2.3] - 2026-11-20

### Added

- Agent-size bake profiles: each agent size is baked as its own profile, with checked tile
  halos and versioned catalogs. Streaming, queries, connectivity and crowds are bound to a
  profile. See [Agent-size bake profiles](docs/BAKE-PROFILES.md).
- Mixed-size crowds: agents on different bake profiles share one crowd world, with ORCA and
  sampled avoidance between them. Routing, replanning, traversal and moving-platform landings
  are per profile. Mixed snapshots keep each agent's and queue's profile binding.
- Moving platform frames with yaw-only poses, runtime docks, routes across frames, and stable
  riders for both crowds. Frame and framed-crowd snapshots restore dock availability and riders
  that are in transit. See [moving platforms](docs/MOVING-PLATFORMS.md).
- Tile-cache zones: `bake::tile_cache_layers` bakes selected columns as tile-cache layers.
  `TileCache::update_budgeted` rebuilds the affected layers in bounded slices, keeps refs when
  the new content is equal, and applies registered modifiers when it replaces a tile.
  See [Tile-cache zones](docs/TILE-CACHE-ZONES.md).
- Managed runtime off-mesh connections: stable ids, optional reserved ground link slots,
  typed landing reports, availability controlled by the game, and versioned snapshots. Tile
  replacement and streaming re-resolve the registered links.
- `OffMeshMode::External` in both crowds: arrival reports stay pending until the game
  completes or aborts the traversal. See [runtime links](docs/RUNTIME-LINKS.md).
- Opt-in `dynamic` feature: gates and cost zones over baked polygons. The navmesh owns the
  modifier layer and derives runtime flags and areas when modifiers are installed or reloaded.
  Toggling a modifier by its stable id does not rebuild tiles or advance their salts. The
  layer is saved in the versioned `DNDY` format. See [Dynamic modifiers](docs/DYNAMIC-MODIFIERS.md).
- A managed link is usable only if both landings pass the query filter, so closing a gate at
  either landing blocks paths through the link.
- Tile change stamps cover residency, replacement, modifier edits and runtime link changes.
  `CrowdWorld` replans the affected corridors in the next step, with an optional per-step cap.
  `Crowd` can opt in with `set_stamp_revalidation`. See [crowd replanning](docs/CROWD-REPLANNING.md).
- `crowd::world::CrowdWorld` (feature `crowd-world`): a crowd store for up to 2^24 agents.
  - Agents use generational `AgentHandle`s. Hot per-agent data is in flat arrays; corridors,
    walls, corners and neighbours are in a side pool.
  - Neighbours come from a sorted-cell grid rebuilt each step (radix-sorted, with no per-cell
    or 16-bit id cap).
  - With the `parallel` feature, the per-agent phases run on the current rayon pool, and
    digests are identical for any thread count.
  - `step` charges `STEP_WORK_PER_AGENT` per active agent, plus the path queue's node
    expansions, to a `Budget` before simulating. If the work does not fit, it refuses the step
    and changes nothing.
  - On an unchanged navmesh, each agent matches `Crowd` with `UpstreamFixes::NEAREST_NEIGHBOURS`
    bit for bit.
  - `'DCRW'` snapshots (`to_bytes`/`from_bytes`) restore a world that steps identically on the
    same navmesh, including sliced searches in progress. Malformed bytes give
    `Error::InvalidSnapshot`.
- `PathQueue::pending`.
- Shared goal fields (`crowd::field`).
  - `GoalField` is a budgeted, resumable Dijkstra flood from a goal over the polygons of a
    region (a radius or a tile set). It uses Detour's portal-midpoint cost model and a
    `(cost, tile, polygon)` tie rule that does not depend on tile slots. Costs are stored per
    tile, with no node-pool cap. One-way off-mesh connections are followed through a reverse
    link index.
  - Every structural change starts a full budgeted rebuild. Until then, previous values may be
    served as `Stale`. `Fresh` values are exact only when the field is `Ready` and the mesh
    epoch matches. Lookups check current residency and salt, and steering reports freshness.
  - Radius regions grow on refresh; cache keys include mesh identity. A cache step with a zero
    budget refreshes every entry for the mesh. Admission counts column storage and key copies;
    when the cache counters are exhausted it returns a typed error.
  - `GoalField::steer` steers through a funnel over the next three polygons.
  - `GoalFieldCache` shares fields by goal, filter and region, with reference counts,
    least-recently-used eviction and a byte cap. It refuses regions that are too large for the cap.
- `NavMesh::replace_tile`, `replace_tile_budgeted` and `replace_cost`, returning `Replaced`:
  swap the tile at a prepared tile's column and layer, all or nothing. Equal content (same
  SHA-256) changes nothing: no salt advance, no epoch change, and runtime flags and areas are
  kept. Other content takes the same slot with one salt advance. A refused replacement leaves
  the resident tile with its salt, refs and runtime state.
- Opt-in ref layouts for 64-bit refs: `RefLayout`, `NavMesh::with_layout`,
  `NavMesh::ref_layout` and `prepare_tile_with_layout`. `RefLayout::WIDE_SALT` (22 salt,
  28 tile, 14 polygon bits) allows 4,194,302 salt advances per slot instead of 65,534, with at
  most 16,384 polygons per tile. The default layout and its refs are unchanged.
- `NavMesh::salt_headroom`: the number of salt advances left for a tile's slot.
- `replace_ops` fuzz target.
- Long-route planner (`route` feature):
  - Sector (`DNSS`) and super-sector (`DNSR`) records move to version 3; regenerate them from
    the source tiles. Older versions are rejected with `InvalidSnapshot(WrongVersion)`.
  - Stitched exits and super-sector sector lists carry a 64-bit summary content id. Records
    from different bakes leave a super-sector stale, including records whose exits land in a
    re-baked super-sector, or make `RouteIndex::build_exact` fail instead of combining them.
    Decoders reject exits that name two content ids for one sector.
  - Inserting every summary of a partial re-bake leaves fresh records in any order. Summary
    versions are computed once instead of on every check.
  - Inserting and decoding records charge their work against `Limits::max_work`. Decoders also
    reject duplicate or skipped super-region members, portals off their side, self-loop exits
    and costs the region arcs do not connect.
  - `SectorSummary::build` charges its transition cost table before allocating it.
  - `Navigator` charges node-pool setup, finalization and string pulling. It retries a
    single-sector window with the whole pool before failing, rejects pools smaller than the
    largest sector, and binds the atlas to the whole filter. It clears every request when
    `update` sees an invalidating edit, and fails only the affected requests when `sync` finds
    an unloaded tile. Label costs stay finite at huge coordinates. Finished coarse searches'
    label arrays are recycled instead of freed inside an update slice, where the allocator
    could return heap memory to the system at several times the slice's cost.
  - Unstitched indexes are documented as hints.
  - New fuzz targets `sector_summary` and `super_record`.
- Arrival policies: shared point goals can use mesh-checked goal slots and optional park or
  remove policies in both crowd managers, so agents do not crowd one destination. Existing
  arrival behavior remains the default.

### Changed

- `CrowdWorld` writes `DCRW` version 4 or later when arrival policies or external traversal
  are used. It still reads the version 2 and 3 layouts. `to_bytes` records the navmesh change
  epoch of the last step, so a restored world replans exactly where the original would.
- Tile bytes: trailing bytes that are not an exact 12-byte reference-layout trailer are
  ignored again, as before layouts existed. A trailer with an unknown version gives
  `TileDataError::WrongVersion`. A trailer that names the default layout no longer changes the
  tile's content hash.
- `Crowd::update`'s per-agent phase arithmetic moved into functions shared with `CrowdWorld`.
  The order of operations is unchanged, so trajectories are unchanged.
- Residency records version 4 and unload-state footers version 3 name an explicit ref layout.
  A navmesh with the default layout still writes versions 3 and 2, byte for byte as before.
- `TileInstaller` installs through `prepare_tile` and `replace_tile`.
  - Pushing the bytes of the resident tile again keeps its refs and runtime flags.
  - A refused tile no longer comes back under a new salt with its flags reset.
  - The layer comes from the tile header instead of being assumed to be layer 0. Removals still
    remove layer 0.
  - Bytes whose header names a different column than the entry's key are refused.
  - Installation now also computes the tile's SHA-256 identity.
- `TileCache` rebuilds replace the resident tile in place, so a refused rebuild keeps the
  resident tile instead of losing it. As in upstream, identical bytes still advance the salt by
  default, so tile-cache parity is unchanged. `TileCache::set_keep_identical_tiles(true)` makes
  identical bytes keep the tile's refs, salt and runtime flags.

### Fixed

- Persisted nondefault ref layouts now tag bare tile state (DNMS version 2) and exported tile
  bytes (NRFL version 1 trailer). Every restore path checks the tag before changing anything,
  including colliding numeric refs and bare restore of unload state. Untagged legacy files mean
  the default layout; default output is unchanged. Old default files load; new readers reject
  unknown layouts.
- Tile-cache processors can no longer retarget a rebuild to another tile or layer.
- Layer-header decoding and cache rebuild use checked or infallible conversions instead of
  unwraps.
- Document prepared content identity, including nondefault layout provenance, and make the
  replacement property model retain the installed bytes.

## [0.2.2] - 2026-11-06

### Added

- Opt-in ORCA crowd avoidance with deterministic nearest-neighbour selection,
  two-sided local boundary constraints, speed-bounded incremental linear programs
  and charged work through `update_budgeted`. The sampled planner remains default.
  Compared with the RVO2 library on the same scenes (`docs/ORCA-RESULTS.md`): maximum
  penetration 0.127 m against 0.135 m on a 512-agent circle, arrival within 5%, about 1.3x the
  cost of the sampled planner. In a dense 64-agent doorway neither finishes within 600 simulated
  seconds; this mode leaves 3 agents unarrived and RVO2 leaves 9.
- Public planar `orca_velocity`, `OrcaAgent`, `OrcaParams` and reusable
  `OrcaScratch`; standalone oracle, trajectory and cost qualification drivers.
- ORCA input fuzz target and ORCA coverage in the crowd-update fuzz target.

### Fixed

- ORCA directional ties select the consistent tangent endpoint; output and
  acceleration interpolation retain the exact speed bound after float rounding.
- ORCA linear programs normalize distance equations and use double precision for
  disc intersections and projected constraints, reducing near-parallel cancellation.
- ORCA finite static segments project the current velocity onto visible endpoint
  arcs, cutoff segments and tangent rays, preserving two-sided endpoint behavior.
- ORCA nearest-neighbour queries use a smaller search only after proving that it
  contains the full nearest set; otherwise they repeat the full-range query.
  The sampled planner and its golden files are unchanged.
- Experimental additive uniform-cost xz interval queries: `AnyAnglePoint`, `AnyAnglePath`,
  `AnyAngleSearch::{begin,resume}`, and `NavMeshQuery::find_any_angle_path`.
  Filters opt in through `QueryFilter::uniform_cost`; weighted costs return
  `Unsupported`. Passable off-mesh connections are traversed in their linked
  direction at their xz length. Search progress is budgeted, retained states
  are capped, and mesh changes require a restart.
- Experimental opt-in boundary traversal extraction through `BakeConfig::links`, with drop,
  jump and climb metadata, solid-span clearance checks, generated-link inspection
  on `Tile`/`TileOutput`, and extraction reach included in incremental halos and
  configuration hashes. Disabled extraction preserves the existing pipeline.

### Changed

- Any-angle qualification validates the funnel baseline before applying the
  never-longer rule and accepts raycast hits at `t >= 1 - 1e-6` as endpoint
  containment. Invalid baselines and tolerance use are counted separately.
- Traversal extraction checks a continuous swept square enclosing the agent's
  radius, including the vertical portions of drops, climbs and raised jumps.
- Internal clipped portal roots use closest detail-surface height lookup after
  rounding; caller endpoints retain strict validation.
- Any-angle search no longer returns `Unsupported` on passable off-mesh
  neighbours; it routes through them.
- Any-angle search turns only at mesh boundary vertices (corner pruning),
  reducing expanded states without changing returned lengths.
- Any-angle dominance comparisons inline their hot predicates and reject
  disjoint bounds before evaluating geometric tests.
- Any-angle qualification checks segment containment with an independent
  closed-coverage test over portal-connected polygons. Detour's raycast stops
  on segments that run along a shared edge or through a shared vertex; those
  stops are still counted and reported.
- Extraction has an independent link-set reference test (exhaustive candidate
  enumeration with plane-partition clearance), on a synthetic scene and, when
  `NAV_CORPUS_DIR` is set, on three corpus meshes.

## [0.2.1] - 2026-11-05

### Added

- Opt-in per-job and per-tile bake reports (`JobOptions::reports`, `BakeJob::report`):
  coverage, span-filter transitions, discarded small regions, area histograms,
  triangulation, local islands and retained tile failures.
- `connectivity`: epoch-scoped local/world weak-component labels with incremental
  affected-component relabeling, directed seed coverage and budgeted typed path
  diagnoses, including filter cuts, unloaded frontiers and resource exhaustion.
- Optional `serde` serialization for reports; plain report structs remain available
  with `no_std` and `alloc`. The diagnostic field contract starts at schema version 1.
- `connectivity_ops` model-checking fuzz target.

## [0.2.0] - 2026-11-04

### Added

- Incremental, resumable and cancellable tile baking from a geometry provider (`bake`
  module, extension not in recastnavigation):
  - `GeometrySource`: a deterministic provider of triangles, area volumes and off-mesh
    connections for one tile column at a time, so a large world never has to be in memory.
    `Input` implements it.
  - `GridLayout`, `TileKey`, `BakeManifest` (with `to_bytes` / `from_bytes` for untrusted
    bytes), `ManifestEntry`: the tile grid, and per-tile input and output digests the caller
    persists next to the tiles. `BakeManifest::dirty_from_boxes(cfg, grid, boxes)` turns
    edited boxes, a grown grid or a changed configuration or layout into the tiles to rebake
    (`DirtyTiles`: keys, removed tiles, `full_rebake`) and `BakeJob::skip_unchanged` drops
    those whose input did not change (and reports a manifest it cannot compare with).
    `BakeManifest::retarget` moves a manifest to a new grid; `apply` takes the job's
    `ManifestDelta` and refuses one baked under another configuration. `from_bytes` checks
    the grid against the configuration hash and bounds the grid size by `Limits::max_work`.
  - `BakeJob`: the same pipeline as `bake::tiles`, driven by `step` (work `Budget`, whole
    stages), `run_parallel` / `run_parallel_chunk` (rayon with `parallel`) or both, with
    finished tiles handed out in row-major order by `next_ready` and the matching manifest
    entries by `manifest_delta`. Output is byte-identical for any step budget, thread count
    and for an incremental rebake against a full bake.
  - `CancelToken` and `Error::Cancelled`: a relaxed atomic load at every work charge stops
    a job within one charge interval; no partial tile is ever returned.
  - `Workspace::set_cancel_token`.
  - `TileInstaller`: queues finished tiles and installs them into a `NavMesh` in slices of a
    work `Budget` (one unit per KiB of tile data), resuming on the next call; and
    `GridLayout::nav_mesh_params(max_tiles, max_polys)`, so the residency cap is the
    caller's parameter. No tile-count limit in the job, the manifest or the installer. A
    rebaked tile replaces the resident one, the newest queued version of a tile wins, a tile
    rebaked to nothing removes it, and `clear` drops what is queued.
  - `Input::indexed` / `IndexedInput`: the same source as `Input` with a spatial index
    built once, so a fetch costs what the tile holds. A job's byte limits count as `tiles`
    counts them, and `BakeConfig::max_total_bytes` is enforced by the job.
- Fuzz targets `incremental_bake` and `bake_job_steps`; seed mode `bake-job`.

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

- Cache eviction preflights association and hash-chain work before mutation, so a
  refused unload preserves live obstacle associations and queued rebuild behavior.
- Streaming content identity now uses full SHA-256 digests, including trailing
  bytes accepted by the tile parser. Saved state is bound to the digest.
  `content_hash` returns `[u8; 32]`; streaming state footer version 2 and residency
  version 3 reject older records, which must be regenerated from original bytes.
- Tile slots refuse salt reuse with `Error::SaltExhausted` before mutation. With
  64-bit references, a slot supports 65,534 advances; identical reloads still work.
- Global off-mesh pairs are relinked in canonical tile-key order on commit and
  removal, fixing load-order-dependent snapped endpoints and link order.
- `NavMeshQuery::portal_points` (and so `find_path` and the other searches that take a
  portal) indexed a polygon's six vertices with the edge of a link read from tile bytes. A
  tile that the reader accepts but whose BV tree reaches a polygon typed as an off-mesh
  connection gets a landing link with edge 255 on it, and the lookup panicked with an
  out-of-bounds index. Found by the new `tile_endian` fuzz target; the same tile bytes
  reached it in 0.1.0 to 0.1.5. The lookup now ends with a typed error. Results for every
  valid tile are unchanged.

### Changed

- `bake::tiles` and the job share one stage machine for the per-tile pipeline (no change
  to output, budgets or errors).
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

[Unreleased]: https://github.com/Tavrin/navmesh-rs/compare/v0.2.3...HEAD
[0.2.3]: https://github.com/Tavrin/navmesh-rs/compare/v0.2.2...v0.2.3
[0.2.2]: https://github.com/Tavrin/navmesh-rs/compare/v0.2.1...v0.2.2
[0.2.1]: https://github.com/Tavrin/navmesh-rs/compare/v0.2.0...v0.2.1
[0.2.0]: https://github.com/Tavrin/navmesh-rs/compare/v0.1.6...v0.2.0
[0.1.5]: https://github.com/Tavrin/navmesh-rs/compare/v0.1.4...v0.1.5
[0.1.4]: https://github.com/Tavrin/navmesh-rs/compare/v0.1.3...v0.1.4
[0.1.3]: https://github.com/Tavrin/navmesh-rs/compare/v0.1.2...v0.1.3
[0.1.2]: https://github.com/Tavrin/navmesh-rs/compare/v0.1.1...v0.1.2
[0.1.1]: https://github.com/Tavrin/navmesh-rs/compare/v0.1.0...v0.1.1
[0.1.0]: https://github.com/Tavrin/navmesh-rs/releases/tag/v0.1.0
