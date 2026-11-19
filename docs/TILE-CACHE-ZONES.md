# Tile-cache zones

A zone is an application-chosen set of whole tile columns that support runtime
obstacle carving. Use `bake::tile_cache_layers` for each selected column and
give the other keys to `BakeJob`. Persist the selected keys and codec choice
alongside the normal bake manifest. Install the polygon tiles with
`TileInstaller` and the zone layers with `TileCache::load_tile` followed by
`update_budgeted`. Each column belongs to one pipeline. Keep the zone layout
fixed while its tiles are resident.

## Bake and install

- Share the same `GridLayout` and bake settings across the boundary. Both
  pipelines rasterize with the same halo, filter, erode and apply area
  volumes. Detour connects their boundary portals when they overlap within
  the walkable climb.
- The zone raster grid must fit `tile_size + 2 * grid.halo_cells <= 255`.
  With a five-cell halo, a 240-cell tile fits at 250 cells including border.
  The runtime layer grids contain the interior cells and the seam portal
  information derived from the halo.
- Set `TileCacheParams::orig` to the grid origin. Give the cache interior
  dimensions of `tile_size` by `tile_size`, the bake's `cs` and `ch`,
  walkable height, radius and climb in world units, and its contour
  simplification error in cells. Size the cache and navmesh for the number
  of resident layers. Each column can have up to 32 cache layers.
- For a raw layer of interior width `w` and depth `h`, stored bytes are
  `56 + 3 * w * h` for the header, heights, areas and connections. A codec
  replaces the `3 * w * h` payload with its compressed output. Cache pools,
  decoded rebuild scratch and the resident navmesh are additional.
- Area markings survive the shared bake prefix. `DefaultMeshProcess`
  maps walkable area 63 to ground area 0 and assigns flag 1.
  Use a custom `MeshProcess` if the zone needs the same area-derived flags as
  the polygon bake or authored off-mesh connections. Supply connections
  through `MeshProcess::configure` on each rebuild.

## Runtime updates

Enqueue obstacle shapes with `add_obstacle`. A mechanism can move its carve by
enqueuing removal of its old handle and adding the new box or oriented box
before the next update. Each command can touch up to eight cached layers.
Overlaps use a bounds test, so an overlapping queued layer can rebuild to
identical bytes. Size the obstacle pool for both handles while the move is
queued, and retain the new handle for the next move.

Call `update_budgeted` before path queries or crowd updates. One budget unit
starts one whole layer, including preparation and installation. A batch that
touches no cached layers consumes one unit. Use a fresh allowance each tick
until the call returns `Progress::Done`. Choose the allowance from measured
layer costs on the target hardware. A single large layer can take longer than a
frame's time allowance. With a one-layer budget, the old and new locations of
a moving obstacle can reflect the change on different ticks.

Replacement keeps equal-content refs and applies the navmesh's registered
dynamic modifiers as it attaches each changed tile. A fully carved layer is
removed. Failed rebuilds stay queued for retry. On eviction, unload the navmesh
tile as well as the cached layer. Reload with `load_tile` so it re-associates
live obstacles before queuing the rebuild.

## Surface height at seams

Cached layers build a coarse polygon surface. Detour generates a triangle fan
from those polygon vertices. The normal bake samples geometry for detail
heights. Paths can cross mixed seams even when heights sampled on opposite
sides differ. Keep boundaries on flat or gently graded terrain and validate
height transitions on the authored geometry. The mixed-seam test checks that
the height step is below 0.00001 world units on its flat floor. Building sampled
detail on each carve would require additional source data and rebuild work.
It can be added later without changing the zone selection or budget API.
