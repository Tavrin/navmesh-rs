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

## Experimental query components

The default-off `route` feature provides the sector summary records and two query
components. `RouteIndex::build` compiles one profile's super-region graph into
arrays outside the frame loop. `RouteIndex::request` starts a `CoarseSearch`;
`resume` charges heap pops, individual edge relaxations and path reconstruction
to a shared `Budget`. The completed corridor is exposed in reverse order, so
completion does not hide an unbounded reversal. Stable region keys break ties.
Dijkstra is used because geometric distance is not admissible for the region
crossing lower bounds. A graph mutation invalidates existing searches.

`CorridorWindow` runs sliced search over the live mesh with a sector-set filter.
Its detached state allows tile streaming between updates. A changed mesh epoch
returns `MeshChanged`; this conservative policy also invalidates windows for
unrelated edits. Admission allocates the node pool, and explicit `finish` copies
the polygon corridor; these operations are not expansion-budgeted. Callers must
inspect partial-result and node-exhaustion status bits. Window endpoints must be
resident, and callers supply the sector margin and resolve portal positions.

These are building blocks, not a qualified end-to-end long-route service. There
is no automatic level-2 portal search, window advancement, first-move scheduling,
or waiting/resumption service yet. No latency, fairness, cost, streaming, or
full-world connectivity acceptance result is claimed for these components.

The current summary representation joins one-way off-mesh connections into
undirected regions, and locates some far endpoints using bounding boxes. Therefore
coarse connectivity is only an optimistic hint. A unit regression documents two
regions connected in one direction collapsing to one super-region. Fixing exact
reachability requires directed region transitions and unambiguous endpoint
membership; callers must not use the current graph to certify reachability.
