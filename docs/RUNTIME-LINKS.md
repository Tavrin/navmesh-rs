# Runtime off-mesh links

Reserve a layer with `NavMesh::set_link_layer(Some(layer))`. Each occupied
column uses one extra navmesh tile slot. A keyed grid must include the layer in
`SlotPolicy::Keyed::max_layers`. The column containing the authored start owns
the link-only tile. The two endpoints are resolved against resident ground tiles,
including in other columns. In an overlap the greatest eligible `(x, y, layer)`
wins. Within that tile the closest ground point wins, using the three-dimensional
radius.

Use `BakeConfig::spare_links_per_tile` for a Recast bake,
`create_nav_mesh_data_with_spare_links` for a direct build, or
`MeshProcess::spare_links_per_tile` for a tile-cache processor. Each link
reserves one slot in its start tile, plus one in its end tile when bidirectional.
The pool is separate from ordinary links and costs one additional encoded and
one decoded link record per slot. Zero preserves the default tile bytes and bake
manifest identity.

`add_runtime_link` takes a `LinkSpec` and an initial enabled bit.
`set_runtime_link_enabled` controls whether paths may traverse it. A disabled
connection keeps its reservations and polygon ref, so enabling it again does
not consume a salt. Adding or removing a connection can change the generated
tile's refs. Use the stable, caller-assigned `LinkId` for application state.

Each link has a `LinkReport`. A missing landing keeps the definition pending.
If a new connection would exhaust a resident tile's reserved slots, the add
returns `Error::NoLinkCapacity` without changing the mesh. After a ground
install, a newly resolvable conflict is reported as `Landing::NoLinkCapacity`.
Allocation is deterministic in ascending link id, with both endpoints reserved
before the connection can enter the graph. Rebuilding, evicting and reloading
ground tiles causes the registered links to resolve again. Managed link-only
tiles remain resident until the last definition in their column is removed.
Use the runtime link API for those tiles.

## Caller-controlled traversal and docks

Set `AgentParams::off_mesh` to `OffMeshMode::External` in either crowd. Poll
`off_mesh_arrivals()` after each update. It lists waiting agents in handle order,
along with their oriented snapped endpoints, ground and connection refs, and
connection identity. The same arrival remains visible while the agent waits.
Drive the traversal in the caller, then call `complete_off_mesh` with a position
near the captured destination. The crowd validates the radius and the landing
polygon with the agent's filter, snaps to the surface and resumes planning.
`abort_off_mesh` resumes from the departure side. Disable a stop before an
abort if it is no longer usable. An agent already handed off can finish while the
link is disabled, provided its captured landing polygon is valid.

For a lift, give each directed stop transfer its own `LinkId` and enable that
transfer when its departure is usable. Apply availability before the crowd
step. A path follower outside the crowds can identify a runtime connection
with `runtime_link_id` and get oriented endpoints with
`off_mesh_connection_poly_end_points`.

[Moving platform docks](MOVING-PLATFORMS.md) use the same stable id,
endpoint radius, availability and direction contract. Their legs and landing
refs are scoped to separate meshes. The frame owner validates the seam and
places the agent in the destination simulation after the local crowd step.

## Saved state

Save `runtime_links_to_bytes()` alongside ground content and, if refs are
persisted, `NavMesh::residency()`. Restore residency first, call
`restore_runtime_links()`, then recommit any remaining ground tiles. Restore
the crowd snapshot after the mesh. A residency reservation for a generated
tile must match the rebuilt tile's content hash. Crowd snapshots with external
traversal state use `DCRW` version 2 and restore pending arrivals.

`DNLK` version 1 is a little-endian record. The 20-byte header contains the four
magic bytes, a `u32` version, an `i32` layer (`-1` for no layer) and a `u64`
definition count.
Each 36-byte definition contains a `u32` id, three start coordinates, three end
coordinates and a radius (seven IEEE 754 `f32` values), `u16` flags, `u8` area
and a `u8` state byte. Bit 0 is bidirectionality and bit 1 is availability.
Definitions appear in strictly ascending id order and are followed by a SHA-256
digest of the preceding bytes. Tile bytes and landing refs are derived at install. Unknown versions return
`Error::InvalidSnapshot(SnapshotError::WrongVersion)`.

For ground tiles with reservations, the first 12 bytes of the serialized
DNAV link pool contain `NRSPARE1` followed by the little-endian `u32` reserved
slot count. The reserved slots are at the end of the pool. The runtime rebuilds
the link entries on installation. A tool that rewrites the serialized pool
must retain the marker to preserve the reservation.
