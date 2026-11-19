# Moving platforms

Use `frames::FrameSet` to store the fixed world mesh and each platform's
local-space mesh. Frame zero is the world. Polygon refs, corridors and
velocities belong to one frame. A rider stays in deck coordinates while
the caller updates the absolute pose. `to_world` converts the final rider
position for rendering or game logic.

## Poses, docks, and queries

1. Register each platform with its local mesh and an absolute pose. Supply a
   yaw pair and translation before each simulation tick. Keep scale, pitch,
   and roll on the rendering side.
2. Register a `DockSpec` with a stable `LinkId`, two frame-local endpoints, a
   landing radius and a direction. Give a lift separate ids for boarding and
   alighting at each stop. Enable a dock while the surfaces meet at that stop
   and disable it before departure.
3. Call `find_nearest_world` to pick ground in a named frame using a world-space
   point. Call `plan` with local points to find a route. Its ordered legs carry
   local refs and the dock id to cross after each approach. Zero complete
   routes is represented by `Complete(None)`. An exhausted search has no
   usable route.

Docks follow the stable id, endpoint radius, direction and availability
contract of [runtime links](RUNTIME-LINKS.md). They reuse the ground-landing
test and apply the query filter to both surfaces. Each surface keeps its own
tile graph, refs, and spare in-mesh link slots. The planner runs Dijkstra on
oriented dock arrivals, so two exits in the same frame can be disconnected.
It prices each corridor by its area-weighted portal-midpoint segments. Ties
use destination frame and dock id. The caller may offer a disabled dock to
planning at an estimated wait cost. A scheduled route still waits for the
dock to open and for its snapped endpoints to align.

Dock transfer is a walk across a seam between surfaces. Place the markers
where the two walkable surfaces meet. Their world positions must agree
within 1 mm even if the search radius is larger. At transfer, the current
source polygon must admit the agent's position and the reprojected position
must snap onto the destination within 1 mm of the source and arrival
marker. Use ordinary external traversal for animated crossings or gaps.
No pose, dock toggle, or cross-frame transfer rebuilds a tile.

## Crowd simulation

Attach one configured empty native simulation per frame to
`crowd::platforms::FramedCrowd<Crowd>` or `FramedCrowd<CrowdWorld>`. Use the same
filter definitions for frames that share agents and the same fixed time step
for all attached `CrowdWorld` simulations. Give each rider a stable
`PlatformAgentId` and set a goal with the destination frame. Use
`set_wait_cost` for a lift stop that will open later. Each tick applies
local movement targets, steps occupied frames in frame id order, and then
attempts crossings in stable agent id order. If the destination crowd is
full the agent remains at the departure side and retries. The destination
corridor starts at the validated local landing. The final goal persists
across the hand-over. Native slot ids, velocities and corridor state are
frame-local. Ordinary external off-mesh requests can still be completed or
aborted in the current frame.

`CrowdWorld` models agents across the two sides of an aligned, enabled dock as
moving discs for avoidance only. Both agents must be within the authored
dock radius, and the recipient's collision range and height overlap still
apply. The nearest six discs are considered independently of same-frame
neighbours. Position and velocity are transformed into the recipient's
frame using the docked poses. Same-frame separation and collision correction
run in the native simulations. The single-threaded `Crowd` keeps agent
avoidance within each frame. In both crowds, the departure-side wall
segment at an open seam is skipped in avoidance for the approaching rider.
Local surface movement still constrains its position until hand-over.

## Snapshot and restore

Save each mesh's tile data, residency, and ordinary runtime links using the
normal APIs. Save the frame snapshot with `FrameSet::to_bytes`. Restore the
meshes with the same frame ids, then apply the frame snapshot. Its binding
checks the mesh's grid, residency and runtime link records. A framed
`CrowdWorld` snapshot restores the native simulations and stable rider
handles, goals and wait costs. Restore it after the matching frame snapshot.

All integers and float bit patterns are little-endian. Both formats end in a
SHA-256 checksum of the preceding bytes. Unknown versions return
`Error::InvalidSnapshot(SnapshotError::WrongVersion)`.

- `DNFR` version 1 stores magic, a `u32` version and `u64` frame and dock
  counts. Each frame is a `u32` id, five pose floats, and a SHA-256 binding of
  its residency record, the five grid floats (origin, width, height) and
  the runtime link record. Each dock is a `u32` id,
  two pairs of a `u32` frame id and three position floats, a radius float,
  and a byte with bit 0 indicating enabled and bit 1 bidirectional.
- `DCFR` version 1 stores magic, a `u32` version, a SHA-256 binding of the
  frame snapshot, and `u64` crowd, rider and wait counts. Each crowd entry is
  a `u32` frame id and a `u64` byte length followed by its `DCRW` snapshot.
  A rider entry is a `u64` stable id, three `u32` values for the frame,
  native slot and generation, and a goal byte. If the byte is 1, a `u32`
  goal frame id and three position floats follow. A wait entry is a
  `u32` dock id and a cost float. Entries in each section are sorted by id.

## Decisions

- Each native simulation owns one frame's queues and refs. Stable rider ids
  live above those simulations. Sharing a native slot across frames would
  require changes to both phase schedulers and their queue snapshots.
  Reversing this choice after publication would require preserving the
  stable-id API and migrating the framed crowd snapshot.
- Docks join coincident surfaces. A gap crossing needs a game-supplied
  traversal. Adding one later would extend the hand-over API and the
  persisted traversal state.
- Cross-frame avoidance is dock-local in `CrowdWorld`. Global cross-frame
  neighbour search would require indexing moving meshes together. Extending
  the cap or the single-threaded crowd's avoidance would change the framed
  simulation's trajectories.
