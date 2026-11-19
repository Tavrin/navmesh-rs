# Arrival at shared goals

The default arrival policy is `ArrivalPolicy::Keep`: reaching a target does not
change the crowd. Existing trajectories, digests and default snapshots keep their
previous behavior.

Set `AgentParams::arrival_policy` before requesting a point goal:

- `Remove` stops and removes the agent at the end of the update that reaches its
  target. A world handle becomes stale through the normal removal path.
- `Park` stops steering when it reaches the target, stays in the neighbour grid,
  and yields under collision correction. Moving ORCA agents take full avoidance
  responsibility around a parked agent; the parked agent takes none. Sampled
  avoidance keeps its existing circle handling.
- `Keep` continues steering towards the target as before.

Arrival requires a complete corridor to the target polygon, planar distance at
most `arrival_distance` (default 0.5 m), and vertical distance at most the agent
height. Partial paths, unloaded targets and velocity requests do not arrive.
Parked agents expose `CrowdAgent::arrived()`; any new movement request clears it.

Use one `GoalSlots` allocator per destination region to spread shared goals.
`GoalSlots::around(goal)` uses 3 m spacing; `new(goal, spacing)` accepts at least
1.6 m. Allocation returns a reservation index, polygon and position. Pass that
polygon and position to `Crowd::request_move_target` or
`CrowdWorld::set_goal` with `Goal::Point`. Allocation uses a deterministic square
spiral, rejects positions away from the mesh and checks spacing after projection.
It returns an error without changing reservations when its candidate bound is
exhausted. Use the same query filter as the agent.

Keep reservations while agents occupy slots, and release them on removal or
retargeting. Nearby shared goal points should use a single allocator to prevent
overlapping reservations. Spacing greater than four times the radius allows an
agent to pass between two neighbours at their slot centres; increase spacing
further when parking with a large arrival distance. Slots in disconnected regions
still require the caller's ordinary reachability checks.

Reservations are caller-owned; save them separately from world snapshots.
World snapshots preserve the arrival policy and parked state. Readers accept the
original version and the extended arrival version; default worlds still write the
original format.

The doorway diagnosis found that all 64 agents passed the opening within 30 s,
while the late arrivals crowded around two shared goal points. Arrival policies
and slots address that congestion. Portal admission was rejected because the
doorway itself was not the bottleneck; adding it later does not require changing
this API. The caller-owned allocator avoids changing movement request signatures;
moving reservation ownership into a manager later would require an additive API.

The qualification command is `nav-orca doorway-slots <reference-executable>`.
It uses the existing doorway geometry, starts, 0.05 s step and 0.5 m arrival
criterion, checks both crowd managers bit for bit, requires all 64 arrivals with
each arrival policy, and compares the default policy's p100 to the reference at
the same allocated goals with a maximum ratio of 1.05.

## Doorway qualification

With default 3 m slots, every agent arrived in both crowd managers. Arrival p100
was 65.80 s with Keep, 65.70 s with Park and 65.60 s with Remove. The reference at
the identical allocated targets gave 65.95 s, so Keep's ratio was 0.998, below
the pre-registered 1.05 limit. Positions and velocities matched between the two
managers at every step, including removals.

The initial 2 m layout finished with Keep (62.90 s versus 76.60 s reference), but
Park did not finish by 600 s: the 0.5 m stopping tolerance could close passages.
The default therefore uses 3 m. Custom spacing remains available down to 1.6 m;
callers must account for agent radius and stopping tolerance.

With arrival behavior disabled, the six-scene world-equivalence check completed
14,400 steps and 6,643,200 agent-tick comparisons with zero mismatches. The ORCA
goldens and all 3,116 sampled plus 27 corrected record digests were unchanged.

All nine crowd corpus cases also passed their existing reference comparisons in
both polygon-reference widths, using the archived oracle records.
