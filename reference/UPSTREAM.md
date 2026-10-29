# Upstream reference: recastnavigation

The C++ library [recastnavigation](https://github.com/recastnavigation/recastnavigation) is the
reference implementation. This file records what we port, at which revision, and the upstream
properties the port has to reproduce or work around.

## Pin

| | Value |
|---|---|
| Latest release | tag `v1.6.0`, commit `6dc1667f580357e8a2154c28b7867bea7e8ad3a7` (2023-05-21) |
| Oracle pin | `main` at commit `9f4ce64458dfae86e1239c525ddc219c4e9e06f1` (2026-02-27) |
| Library commits between them | 35 commits touching `Recast/`, `Detour/`, `DetourCrowd/`, `DetourTileCache/` (29 files, +1289/-1035) |

We pin `main` at `9f4ce64`, not the `v1.6.0` tag. The library commits since the tag include
behaviour-changing fixes that a new port should not reproduce as bugs and then fix again:

- `#796` crash (assert) on large-scale navmesh generation;
- `762ecb5` invalid spans (zero or negative size) are now logged and ignored;
- `#657` more robust detail-mesh edge detection;
- `#470` `rcBuildLayerRegions` was missing the area type;
- `#635` out-of-bounds access in `dtMergeCorridorStartMoved`;
- `#753` `raycast()` exits on `DT_BUFFER_TOO_SMALL`;
- `#763` poly-ref bit overflow guard;
- `#672` ledge-span filter rewrite. Upstream issue `#729` reports that this "refactor" changed
  behaviour; the oracle follows the current code, and the difference is listed as a known
  upstream question below.

If a release tag appears later, moving the pin is one variable in the harness and the corpus
fetch script; every record is regenerated and the port's differential tests show the delta.

## Modules and size

Line counts at the pin (headers and sources, excluding the demo, debug drawing and tests):

| Module | Purpose | Lines |
|---|---|---|
| Recast | Voxel generation: rasterize, filter, compact heightfield, erode/mark areas, partition (watershed, monotone, layers), contours, poly mesh, detail mesh, heightfield layers | ~10,400 |
| Detour | Runtime navmesh (tiles, links, BV tree, off-mesh connections), tile data builder, queries (A*, sliced A*, straight path, raycast, Dijkstra searches, random points, wall queries), node pool | ~9,200 |
| DetourCrowd | Crowd manager, path corridor, local boundary, obstacle avoidance (sampled velocity), proximity grid, path queue | ~4,200 |
| DetourTileCache | Compressed heightfield layers, obstacles (cylinder, AABB, oriented box), per-layer regions/contours/poly mesh, incremental rebuild | ~3,500 |

`DebugUtils/` (debug drawing, dump helpers) and `RecastDemo/` are not part of the port. The demo is
read for how the library is meant to be driven (`Sample_SoloMesh.cpp`, `Sample_TileMesh.cpp`,
`Sample_TempObstacles.cpp`).

## Public API surface

Recast (free functions over `rcHeightfield`, `rcCompactHeightfield`, `rcContourSet`,
`rcPolyMesh`, `rcPolyMeshDetail`, `rcHeightfieldLayerSet`, with an `rcContext` for logs and timers):
`rcCalcBounds`, `rcCalcGridSize`, `rcCreateHeightfield`, `rcMarkWalkableTriangles`,
`rcClearUnwalkableTriangles`, `rcAddSpan`, `rcRasterizeTriangle(s)` (three index forms),
`rcFilterLowHangingWalkableObstacles`, `rcFilterLedgeSpans`, `rcFilterWalkableLowHeightSpans`,
`rcGetHeightFieldSpanCount`, `rcBuildCompactHeightfield`, `rcErodeWalkableArea`,
`rcMedianFilterWalkableArea`, `rcMarkBoxArea`, `rcMarkConvexPolyArea`, `rcOffsetPoly`,
`rcMarkCylinderArea`, `rcBuildDistanceField`, `rcBuildRegions`, `rcBuildRegionsMonotone`,
`rcBuildLayerRegions`, `rcBuildHeightfieldLayers`, `rcBuildContours`, `rcBuildPolyMesh`,
`rcMergePolyMeshes`, `rcCopyPolyMesh`, `rcBuildPolyMeshDetail`, `rcMergePolyMeshDetails`, plus
alloc/free pairs and a pluggable allocator (`rcAllocSetCustom`).

Detour: `dtCreateNavMeshData` and endian swap helpers; `dtNavMesh` (init single-tile or
multi-tile, `addTile`/`removeTile`, tile lookup by location or ref, poly ref encode/decode, poly
flags and area setters, off-mesh endpoints, `storeTileState`/`restoreTileState`);
`dtNavMeshQuery` (`findPath`, sliced `init/update/finalize(Partial)SlicedFindPath`,
`findStraightPath`, `findPolysAroundCircle`, `findPolysAroundShape`, `getPathFromDijkstraSearch`,
`findNearestPoly` (two forms), `queryPolygons` (array and callback forms),
`findLocalNeighbourhood`, `moveAlongSurface`, `raycast` (two forms, the second with costs and a
`dtRaycastHit`), `findDistanceToWall`, `getPolyWallSegments`, `findRandomPoint`,
`findRandomPointAroundCircle`, `closestPointOnPoly`, `closestPointOnPolyBoundary`,
`getPolyHeight`, `isValidPolyRef`, `isInClosedList`); `dtQueryFilter` (include/exclude flags,
64 area costs; virtual `passFilter`/`getCost` when `DT_VIRTUAL_QUERYFILTER` is defined).

DetourCrowd: `dtCrowd` (`init`, `addAgent`/`removeAgent`, `requestMoveTarget`,
`requestMoveVelocity`, `resetMoveTarget`, `update(dt, debug)`, up to 8 obstacle-avoidance
parameter sets, 16 query filters), plus the building blocks `dtPathCorridor`, `dtLocalBoundary`,
`dtObstacleAvoidanceQuery`, `dtProximityGrid`, `dtPathQueue`.

DetourTileCache: `dtBuildTileCacheLayer`, `dtDecompressTileCacheLayer`,
`dtBuildTileCacheRegions`, `dtBuildTileCacheContours`, `dtBuildTileCachePolyMesh`,
`dtMarkCylinderArea`/`dtMarkBoxArea` (AABB and oriented); `dtTileCache` (`init`, `addTile`,
`removeTile`, `addObstacle`, `addBoxObstacle` (two forms), `removeObstacle`, `queryTiles`,
`update(dt, navmesh, upToDate)`, `buildNavMeshTile(s)At`); user-supplied
`dtTileCacheCompressor` and `dtTileCacheMeshProcess`.

## Data formats

All formats are raw native-endian structs; endian swap helpers exist for cross-endian loading.

- **Navmesh tile** (`dtCreateNavMeshData` output): magic `'DNAV'`, version 7. Layout: `dtMeshHeader`,
  then vertices (f32×3), `dtPoly` (first link, 6 vertex indices, 6 neighbour codes, flags,
  vertex count, area and type packed in one byte), `dtLink` slots (reserved, filled at
  `addTile`), detail sub-meshes, detail vertices (f32×3), detail triangles (4 bytes each), BV tree
  nodes (quantized u16 bounds, index), off-mesh connections. Each section is padded to 4 bytes.
  Neighbour codes: internal neighbour index + 1, or `DT_EXT_LINK | side` for tile borders.
- **Poly refs:** 32-bit by default (salt 10 bits, tile and poly bits from `maxTiles`/`maxPolys`,
  together 22 bits); 64-bit with `DT_POLYREF64` (salt 16, tile 28, poly 20). Issue `#669`
  questions the link layout in 64-bit mode.
- **Tile state** (`storeTileState`): magic `'DNMS'`, version 1, per-poly flags and area.
- **Tile cache layer:** `dtTileCacheLayerHeader` magic `'DTLR'`, version 1, followed by the
  compressed grid (heights, areas, connections), u8 dimensions (so a layer is at most 255×255
  cells). The compressor is user-supplied; the demo uses FastLZ (MIT).
- **Multi-tile set files** (`'MSET'` navmesh sets and `'TSET'` tile-cache sets, version 1) are
  RecastDemo conventions, not library formats.
- **Limits baked into the format:** 6 vertices per polygon (`DT_VERTS_PER_POLYGON`), 64 area ids,
  u16 polymesh vertex coordinates, 6-bit span area, 13-bit span heights
  (`RC_SPAN_HEIGHT_BITS`), compact cells with a 24-bit index and 8-bit span count, 255 regions per
  tile-cache layer.

## Determinism properties

Measured or read at the pin. Each item names what the port has to do.

1. **Float model.** Everything is `float` (f32) arithmetic; integer grids carry most of Recast.
   Transcendentals appear at four places: `cosf` of the walkable slope (`Recast.cpp:344` and
   `:369`, once per call), `cosf`/`sinf` in the obstacle-avoidance sample pattern
   (`DetourObstacleAvoidance.cpp:499` and `:535`) and in oriented box obstacles
   (`DetourTileCache.cpp:451`); `dtMathAtan2f` is defined but unused. `sqrtf`, `floorf` and
   `ceilf` (used throughout) are exact IEEE operations; `cosf`/`sinf` come from the platform libm,
   so C++ results at those sites can differ across libms.
2. **Contraction.** C++ compilers may fuse `a*b+c` into FMA (GCC contracts by default where the
   target has FMA, e.g. AArch64 or `-march=native`). Rust never contracts. The oracle is built with
   `-ffp-contract=off` for x86-64 baseline; an oracle built otherwise is a different oracle.
3. **Unstable `qsort` with ties.** `qsort` sorts contour holes (`compareHoles`), merge diagonals
   (`compareDiagDist`) and BV-tree items (`compareItemX/Y/Z`), and all three comparators return 0
   for distinct items (equal distances, equal quantized bounds are common on grid data). Output
   therefore depends on the C library's `qsort`. glibc's `qsort` is a merge sort, stable when
   its scratch allocation succeeds (glibc restored it in January 2024 after a short-lived switch
   to introsort); MSVC and the BSD/macOS libcs use different algorithms. Upstream C++ is
   therefore not bit-reproducible across platforms. Our contract: **stable sort semantics** (the
   Rust `sort_by` result), which equals glibc's merge sort; the oracle runs on glibc ≥ 2.39.
4. **Iteration orders** are index-driven (span order, region ids by flood order, contour order by
   region, poly order by contour), with no hash-map iteration. The Detour node pool is a hash
   table keyed by poly ref, but searches walk the open list (binary heap with a fixed sift rule),
   so results do not depend on hash order. Heap tie-breaks (equal `total`) follow the sift-up and
   sift-down code exactly and must be ported as written.
5. **Sliced and unsliced A\* are different functions.** `findPath` keys nodes by
   `(ref, crossSide)` so a polygon reached across different tile sides gets separate states;
   `updateSlicedFindPath` uses `getNode(ref, 0)`. On tiled meshes they return different corridors
   for the same request (measured: 30 of 64 pairs on the tiled dungeon, 17/64 on tiled nav_test,
   8/64 on the tiled small terrain, 2/32 on the seams case; 0 on all seven solo cases).
   Parity is per function, never sliced-against-unsliced.
6. **Random numbers** are caller-supplied (`float (*frand)()` in `findRandomPoint*`); the library
   holds no RNG state. The crowd has no randomness.
7. **Allocation** goes through `rcAlloc`/`dtAlloc` hooks; results do not depend on addresses.
8. **Run-to-run.** Twelve corpus cases baked twice (separate processes) produced identical
   records (all stages, queries, crowd).

## Threading assumptions

Upstream has no threading support (its roadmap: "Currently Recast provides no specific threading
support"). In practice: Recast functions only touch their arguments, so separate tiles can be
built on separate threads with separate `rcContext`s. `dtNavMesh` is not synchronised: tile
add/remove must be serialised against queries. `dtNavMeshQuery` owns node pools and must not be
shared across threads; several queries over one navmesh are fine while it is not mutated.
`dtCrowd` and `dtTileCache` are single-threaded objects. The global allocator hooks are process
wide.

## Known upstream issues relevant to the port

From the upstream tracker (109 open issues at the time of reading; 37 labelled `type: bug`):

- **Unvalidated tile data.** A cluster of recent reports (`#829`–`#839`) shows `addTile` and the
  link/off-mesh code trusting header counts and indices: out-of-bounds reads and writes,
  null dereferences, signed overflow on `maxLinkCount`. The port validates tile bytes completely
  before use (typed error, never UB).
- `#826` signed integer overflow in `dtProximityGrid::hashPos2` far from the origin.
- `#650` infinite loop in `triangulateHull` when `detailSampleDist == 0`.
- `#687` `dividePoly` crash on an 8th point; `#508` crash in `canRemoveVertex` (tile cache).
- `#641` crowd behaviour degrades past `MAX_NEIS` (32) neighbours; `#496` sample-pattern index
  out of bounds.
- `#501` obstacle stuck in `DT_OBSTACLE_REMOVING`; `#779` large tile size in the tile cache builds
  an incorrect navmesh (u8 layer dimensions).
- `#445` `findStraightPath` overflowing caller arrays; `#438` sliced path finder bug report.
- `#729` behaviour change inside the `#672` ledge-filter refactor (see Pin).
- `#421` request for double precision.

Policy: where upstream has undefined behaviour or a crash, the port returns a typed error and
the record notes the divergence; where upstream returns a wrong but defined result, the port
matches it by default and an opt-in fix flag may correct it (see `docs/DESIGN.md`).

## Licence and attribution

recastnavigation is under the zlib licence (`License.txt`, copyright 2009 Mikko Mononen). The
port is a derivative work (a translation of the source), so the zlib terms apply to it:

> 1. The origin of this software must not be misrepresented; you must not claim that you wrote
>    the original software. If you use this software in a product, an acknowledgment in the
>    product documentation would be appreciated but is not required.
> 2. Altered source versions must be plainly marked as such, and must not be misrepresented as
>    being the original software.
> 3. This notice may not be removed or altered from any source distribution.

Obligations for this repository, distributed under MIT OR Apache-2.0:

- Keep a copy of the upstream zlib notice (`LICENSE-ZLIB` or a section of `NOTICE`) in every
  source distribution, unaltered.
- State plainly in the README, crate docs and `UPSTREAM.md` that this is an altered,
  translated version of recastnavigation, with the pinned commit.
- Do not present the port as the original. Credit Mikko Mononen and the recastnavigation
  contributors.
- MIT/Apache-2.0 applies to our own contributions; zlib is permissive and compatible with both,
  and imposes no obligation on downstream binaries beyond the optional acknowledgment.

Third-party pieces we may touch: FastLZ (MIT, Ariya Hidayat) if we port the demo's tile-cache
compressor; the demo meshes are discussed in `reference/corpus/README.md`.
