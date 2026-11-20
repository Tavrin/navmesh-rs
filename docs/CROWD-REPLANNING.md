# Crowd replanning after mesh changes

Apply navmesh edits before calling `CrowdWorld::step` or `Crowd::update`.
`CrowdWorld` checks for changed tiles by default. Call
`Crowd::set_stamp_revalidation(true)` to enable the same checks in `Crowd`.

`NavMesh::change_epoch` and `tile_change_stamp` cover effective flag and area
changes, tile installs and removals, and runtime link availability and landings.
Tile-cache rebuilds publish changes through tile replacement. Equal-content
replacement preserves the stamp when it preserves the tile. The counters are
local to a navmesh instance. Tile refs, structural epochs and modifier state
epochs keep their existing meanings.

On an edit, a point-directed agent checks the tiles on its corridor and at its
goal. A changed tile triggers a fresh filtered path search before steering. The
crowd remembers the first obstruction on the old corridor so a reopened gate
can restore a route after a detour. Agents with a failed or partial route retry
when the mesh changes, since their current corridors can omit newly passable
tiles. A resident goal that becomes blocked is kept for retry. A search that
proves the goal unreachable reports `TargetState::Failed` and holds the agent
in place. Larger searches can continue through the existing sliced queue. A
known unloaded tile can still cause `TargetState::WaitingForTile`.

Set `set_replan_limit` on either manager to cap the number of stamp-triggered
replans per tick. The default limit is 1,024. The cap is a count
of admitted replans, independent of the step's work budget and the sliced
queue's expansion budget. Agents that exceed the cap stop and remain pending
for later ticks. Admission resumes after the last admitted index and wraps
at the end of the active agents. An agent already traversing a
link finishes or aborts the traversal before replanning. Inspect
`CrowdAgent::replan_pending` for deferrals and `StepReport::replans` for the
number admitted by a world step.

Take a world snapshot at a completed step boundary and restore the navmesh to
the same state before restoring the crowd. The crowd snapshot uses version 4
when a custom cap, pending replan or watched obstruction must be saved. Older
snapshot versions remain readable.
