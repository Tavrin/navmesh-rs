# Extended bake links

Set `BakeConfig::extended_links` to enable ladders, stair steps or level gap
jumps independently. The serial, parallel and incremental bakes use the same
extraction stages. A `ProfileBakeSpec` uses the settings and agent dimensions
in its own `BakeConfig`. These settings participate in the manifest and profile
catalog identities and increase the bake halo to cover landing search and
clearance near tile borders. Within a tile, ladder links follow volume and
floor order, then step and gap links follow polygon, edge and sample order.
Inspect the bake's generated connections in `Tile::extended_links` or
`TileOutput::extended_links`.

## Authoring and clearance

- Set `ladders` with `LadderLinks::new(reach, area, flags)`. Author each shaft
  with `AreaVolume::ladder(verts, hmin, hmax)` in `Input::volumes`, or call
  `VolumeSink::push_ladder` from a custom geometry source. Use a convex
  footprint covering the shaft and its landing approaches. The mean of the
  footprint vertices defines the vertical shaft axis. The bake selects the
  nearest accessible walkable cell center at each voxel floor height within
  the footprint and reach. Any walkable surface within the footprint can
  supply a floor, including incidental platforms. A missing landing can split
  the ladder. The bidirectional segments join consecutive landings and check
  the vertical shaft and both approaches for clearance. The lower-floor
  approach can extend up to `reach` and is checked for clearance only. Author
  the height interval to include voxelized floor heights and keep the shaft
  clear of solid geometry.
- Set `steps` with `StepLinks::new(distance, height, area, flags)`.
  Wall-edge samples look for a different surface beyond ordinary walkable
  climb, within the vertical and horizontal limits. The voxel centerline
  must cross solid support and a discontinuity above walkable climb. The
  agent must have clearance to rise and then move across at the upper
  landing height. The extractor samples each side independently. A legacy
  link for the same directed polygon pair takes precedence over a step. The
  bake removes duplicate same-tile steps. Across a seam, the navmesh suppresses
  the step's connection once it can resolve the destination polygon.
- Set `level_gaps` with `GapLinks::new(distance, clearance, area, flags)`.
  Both landings must have the same voxel floor height, with a break in
  support along a centerline free of walls or curbs above the climb threshold.
  Clearance checks both vertical legs and the
  raised segment joining them. Distance is measured horizontally between
  the sampled points on the eroded landings.

All distances are in world units. The clearance test sweeps a square enclosing
the agent radius and extending up by the agent height through the solid
heightfield. Extraction uses voxel landings and fixed wall-edge samples, so
small features and approaches that require a curved trajectory can be missed.
These settings apply to ground tiles produced by the Recast tile bake.
With extended extraction enabled, zero detail sample distance still triangulates
the ground polygons so traversal endpoints can snap to the correct floor.

## Costs and traversal

**Use `OffMeshMode::External` for ladders.** The default `OffMeshMode::Animate`
moves in a straight line between the landings and can pass through a slab edge.
External mode lets the caller follow the shaft and landing approaches.

Set distinct Detour polygon areas for independently configurable kinds and set
those entries in `DefaultFilter::area_cost` on the query or the crowd's filter.
Use the link flags to control which filters may traverse it. Use
`NavMesh::off_mesh_connection_by_ref` and `OffMeshConnection::kind` to identify
the traversal in a path. In `OffMeshMode::External`, both crowds expose the
same kind in `OffMeshReached::kind`. Use this mode to implement the authored
ladder, step or jump movement, then complete or abort the hand-off as described
in [runtime links](RUNTIME-LINKS.md).

With ladders enabled, triangle rasterization and landing selection use the
shared world voxel grid, including at non-binary cell sizes. Each segment belongs to the
tile containing its lower landing. That tile emits it once, and the upper
landing tile reserves a reverse link slot for a cross-tile segment.

The originating ground tile owns the connection. The crate attaches the
landing in the same bake layer at the endpoint's tile coordinate, including
across empty columns and when either tile is streamed in later. The connection
record uses the existing DNAV flags byte, with explicit values for the three
new kinds. The crowd derives the kind from the tile on snapshot restore.

## Authoring decision

The bake geometry API is in-memory. A dedicated ladder volume kind shares
the existing volume queries and tile buckets. It never paints ground, and
profiles without ladder extraction ignore it. All Recast area IDs remain
available for area paint in every profile. The alternative of reserving an
area ID would require tracking which IDs each source uses and coordinating
them across profiles. Reversing the choice would require migrating
geometry-source adapters and rebaking affected tiles.
