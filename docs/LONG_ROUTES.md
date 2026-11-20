# Long-route measurement

The ignored `route_world` integration test measures the production sliced search
through its eight-slot path queue. It adds no runtime feature or library behavior.
Enable `parallel` and run the ignored test in release mode. The module documentation
lists the environment controls and JSON report destination.

Both worlds use seeded terrain, a spanning wall, an elevated slab and directed
off-mesh links. The dense variant adds seeded boxes with null-area roofs: 6 metre
blocks, 176 occupied blocks per 256, independent edge lengths of 1.2 to 3.6 metres,
and a 14 metre top. Density must average 300 to 1000 polygons per nonempty tile.
Every report includes the parameters, tile counts and density distribution.
Fixture reuse checks the configuration, generator source and every tile digest.

Each cohort defaults to 1000 reproducible endpoint pairs. Reference and production
execution alternate order per pair. Calibration uses thread CPU per actual sliced
expansion. The 1 millisecond frame contains four units targeting 0.25 milliseconds
each; the effective iteration count is printed and can be pinned across worlds.
Frames are simulated iteration budgets, while measured unit and frame CPU report
all overruns. Calibration does not guarantee a hard time limit. Queue calls include
initialization and finalization, whose work is not bounded by expansion count.

The baseline publishes its first corridor when the queue returns a result, including
a usable partial corridor after pool exhaustion. Pool exhaustion and completion are
reported separately. The concurrent cohort submits 64 requests at frame one, holds
eight in the queue and retains the remainder in a caller backlog. It collects every
finished result after each unit, before expiration. This measures existing FIFO
service, including caller waiting; it does not establish a fair scheduling policy.

A valid query can return a partial corridor for a disconnected goal. This is
distinct from an API error. Reachability is resolved by reaching a connected goal
or exhausting the search without exhausting the pool for an unreachable goal.
Pool exhaustion remains unresolved and fails the reachability agreement bar.

The independent quality reference is unbounded A* over directed incoming portals.
It uses portal midpoint distance and area costs, with separate labels for different
entries to a polygon. It is a quality oracle, not the production comparator. Its
CPU and expansion counts are not interchangeable with a reference that keeps only
one label per polygon.

Direction agreement charges the delivered first 50 metres, then an exact completion
from that point. A shorter corridor counts only if it reaches the goal. Missing
prefixes cannot earn agreement credit. A separate greedy baseline finalizes the best
node after one frame of sliced search. Both report coverage and agreement counts.
Acceptance bars are reported even when they fail; successful test execution means
the measurement completed, not that the planner met those bars.

Use `NAV_ROUTE_QUALITY_ONLY=1` with the original `NAV_ROUTE_FRAME_UNITS` to replay
correctness checks across worker threads without repeating a CPU baseline.
Each pair owns its query state; indexed collection preserves digest order.
This mode reports quality and deterministic concurrent frame counts, omitting
CPU statistics. A regression compares complete reports from one and four workers.
Reuse matching saved timing cohorts when only these correctness metrics change.

Sector compression, streaming, hierarchy memory and edit fan-out require a sector
planner and are outside this flat baseline. Existing corpus parity and determinism
checks remain independent compatibility gates.

## Directed routes

Enable the optional `route` feature and build a `SectorGraph::from_mesh` after
installing the complete world, applying modifiers and linking its connections.
The builder contracts reciprocal polygon links into sector regions and records
the remaining directed arcs. It resolves both ends of every crossing against
the installed polygon links, including runtime links and distant landings.

Sector and super-sector records use version 3. Regenerate both from the source
tiles when upgrading; older versions are rejected with
`InvalidSnapshot(WrongVersion)`. A sector record includes sorted directed arcs,
run-length polygon membership and, when stitched, outgoing region addresses. A
stitched super-sector records directed local arcs and exits addressed by
destination sector and region. Every stitched exit also carries the content id
of the destination sector's summary (a fingerprint of its source tiles), and a
super-sector record lists the content id of each of its sectors. Records from
different bakes are detected where they are combined: a graph marks a
super-sector stale when its record and resident summaries disagree on a summary
version or a crossing names another content id, and `RouteIndex::build_exact`
rejects exits whose destination record lists another content id. It also
requires stitched records with resolved destinations.

Individually collected tile batches produce isolated (unstitched) sector
summaries. Their connectivity is estimated, not exact: facing portals join where
their runs overlap in both directions, and an off-mesh link joins the first
region whose box holds its far end within a millimetre (only across the 3 x 3
neighbouring super-sectors). An index built from them is a hint that can report
connections that do not exist and miss ones that do. Only records stitched by
`SectorGraph::from_mesh` against a complete installed world are exact; bind
isolated summaries that way before certifying a route.

Inserting and decoding records charge their work against the graph's
`Limits::max_work` (one unit per examined region, portal pair, off-mesh landing
test, exit and cost-table cell), so a crafted record ends in `LimitExceeded`
with the graph unchanged instead of an unbounded rebuild.

Prepare a `RouteAtlas` with the same mesh and filter, then create a `Navigator`.
Submit two `WindowPoint` endpoints with `request`. Call `update` repeatedly
with one shared frame budget and the maximum units per scheduler turn. An
unanswered request gets priority over a continuation. The directed region
search checks reachability. A short live search in the reciprocal start region
can publish a prefix toward a coarse crossing. Keep calling `update` while
the agent walks. The prefix and its endpoint stay available until acknowledged,
and the continuation starts at that endpoint.

Level-two search considers several concrete crossing positions on a wide
border. Live refinement spans multiple sectors and a one-sector margin. It
prices each step between incoming and outgoing portal midpoints using the
current polygon's area cost. If the node limit is reached, the scheduler
shortens the window and can fall back to single-sector refinement. A
single-sector window starts with a small node pool and, when that runs out,
retries with the whole pool before the request fails. `Navigator::new`
rejects a pool smaller than the passable polygon count of the atlas's largest
sector, so that retry always fits.

On `RouteState::Segment`, `segment` gives the polygon corridor, `corners` gives
its string-pulled steering points and `toward` gives the endpoint to acknowledge
with `advance`. The last acknowledgment sets `Complete`. Call `cancel` to release the request.

If a window or its margin needs a missing sector, the navigator reports
`Waiting` for that sector after the caller acknowledges the available segment.
It uses the same window after reload. Call `sync` after residency changes. It compares slot identities
outside the frame budget, restarts active windows and wakes waiting requests.
A request whose published segment crosses an unloaded tile fails alone with
`MeshChanged`. A changed tile generation, modifier revision or runtime-link
edit returns `MeshChanged` from `sync` or `update` and fails every request, so
no getter returns a corridor through the edit. Rebuild the graph and atlas and
resubmit from the current position. The current navigator retains the level-two payloads while level-one
tiles stream. Its mesh binding, scheduler handles and live polygon corridors
are process-local.

Live refinement allocates node and hash-table pages as it examines links. A
window reuses these pages when the next search starts. A work unit can include
corridor finalization. Work done outside the expansion loop (a window's node
pool, finalization, string pulling of a published segment, tile scans after
streaming) is charged too: what the current unit cannot pay becomes a debt the
request pays before its next step. The work
budget alone gives no wall-clock guarantee. The ignored `route_world` harness
selects the candidate with `NAV_ROUTE_CANDIDATE=1` and the `parallel,route`
features. Quality replay and pinned timing use the same seeded pair order.
