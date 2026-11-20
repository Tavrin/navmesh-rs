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

- Set `ladders` with `LadderLinks::new(marker_area, reach, area, flags)`.
  Supply each shaft as a convex `AreaVolume` with that marker area, the
  intended floor heights in `hmin..=hmax`, and a footprint covering the
  shaft and its landing approaches. The mean of the footprint vertices
  defines the vertical shaft axis. The bake selects the nearest accessible
  walkable cell center at each voxel floor height within the footprint
  and reach. It joins consecutive landings with bidirectional links after
  checking both horizontal approaches and the vertical shaft. Author the
  height interval to include the voxelized floor heights. Keep the shaft
  clear of solid rasterized geometry.
- Set `steps` with `StepLinks::new(distance, height, area, flags)`.
  Wall-edge samples look for a different surface beyond ordinary walkable
  climb, within the vertical and horizontal limits. The voxel centerline
  must cross solid support and a discontinuity above walkable climb. The
  agent must have clearance to rise and then move across at the upper
  landing height. The extractor samples each side independently.
- Set `level_gaps` with `GapLinks::new(distance, clearance, area, flags)`.
  Both landings must have the same voxel floor height, with a break in
  support along the centerline. Clearance checks both vertical legs and the
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

Set distinct Detour polygon areas for independently configurable kinds and set
those entries in `DefaultFilter::area_cost` on the query or the crowd's filter.
Use the link flags to control which filters may traverse it. Use
`NavMesh::off_mesh_connection_by_ref` and `OffMeshConnection::kind` to identify
the traversal in a path. In `OffMeshMode::External`, both crowds expose the
same kind in `OffMeshReached::kind`. Use this mode to implement the authored
ladder, step or jump movement, then complete or abort the hand-off as described
in [runtime links](RUNTIME-LINKS.md).

The originating ground tile owns the connection. The crate attaches the
landing in the same bake layer at the endpoint's tile coordinate, including
across empty columns and when either tile is streamed in later. The connection
record uses the existing DNAV flags byte, with explicit values for the three
new kinds. The crowd derives the kind from the tile on snapshot restore.

## Authoring decision

Tagged area volumes reuse the existing borrowed input, geometry-source
volume queries, tile buckets and incremental invalidation. In a ladder-enabled
profile the selected marker area is reserved for shaft volumes and those
volumes are consumed before ground area painting. A separate ladder input
stream would allow independent area marking with the same id. Moving to one
would require updating geometry-source adapters and rebaking affected profiles.
