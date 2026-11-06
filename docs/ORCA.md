# Reciprocal local avoidance

This implementation is an **unqualified candidate**. See the
[recorded acceptance failures](ORCA-RESULTS.md) before considering integration.

`AgentParams::avoidance = AvoidanceMode::Orca(OrcaParams::default())` selects
ORCA for an agent. `OBSTACLE_AVOIDANCE` still gates velocity planning. The default
is `AvoidanceMode::Sampled`; `obstacle_avoidance_type` still selects its existing
preset. Agents in different modes may share a crowd.

```rust
use navmesh_rs::crowd::{AgentParams, AvoidanceMode, OrcaParams};
let mut params = AgentParams::default();
params.avoidance = AvoidanceMode::Orca(OrcaParams::default());
```

The defaults are a two-second agent horizon, a one-second obstacle horizon and
sixteen neighbours. The crowd retains the nearest agents by `(distance squared,
slot index)`, using the same full candidate query as `NEAREST_NEIGHBOURS`, even
when that correction flag is off. Full queries use a stamp table for constant-time
id deduplication; all cells in the configured range are still visited and charged. Sampled agents keep their existing six-neighbour
behaviour unless that correction is explicitly enabled. Wall constraints precede
agent constraints; agents are processed in ascending slot order from the same
velocity snapshot. ORCA does not spawn worker threads.

`orca_velocity` exposes the planar solver independently of a navmesh. It takes a
workspace, an `OrcaAgent`, neighbours, two-sided `[x0, z0, x1, z1]` segments,
parameters, a positive integration step and reusable `OrcaScratch`. Its result is
fallible. Input positions, velocities, radii and speed limits must be finite and
no greater than `1e6` in magnitude; radii and speed limits must be positive.
Horizons and positive steps must be within `1e-5..=1e5` seconds. The crowd also
accepts a zero-time tick, which skips ORCA planning; the existing penetration
correction and corridor phases still run.

## Geometry and solver

The implementation derives reciprocal disc half-planes from equations (5)-(8) of
[van den Berg et al., *Reciprocal n-Body Collision Avoidance* (2011)](https://doi.org/10.1007/978-3-642-19457-3_1).
Each agent takes half the responsibility. The incremental 2D program intersects
the speed disc with those planes and selects the closest velocity to the desired
one. An infeasible program invokes section 5.3's projected 3D epigraph program,
minimizing the largest signed violation of the agent planes. Static constraints
retain priority when feasible. Constraint order is fixed rather than randomized.

Static constraints use section 5.4's zero optimization velocity: project the agent
centre onto the segment, expand by its radius, and construct the tangent at the
nearest velocity-obstacle boundary. Endpoint reversal has no effect. The crowd
uses its existing `LocalBoundary` cache, which retains at most eight segments from
sixteen local polygons. Polygon convexity and connected-obstacle flags are not
available. This wall policy can differ from reference-library polygon handling.

The solver uses arithmetic and square roots only. It does not use trigonometric
functions or require `libm` for the standard-library build. Pure solver storage is
reused through `OrcaScratch`. Crowd solver storage is currently reused within a
tick and released after it. Candidate visits, constraint additions, constraint
checks and LP pivots are charged before work. Capacity growth is fallible and
charged to the workspace. `update_budgeted` keeps its existing partial-tick
refusal contract; no velocity is returned after a refused solver charge.

## Limits

ORCA supplies a local velocity, not a global routing or fairness guarantee.
Acceleration limiting after planning can violate the computed half-planes. Finite
neighbour and boundary ranges can omit relevant collisions; choose a query range
that covers the horizon and relative speeds. Different agent horizons, sampled
neighbours and nonreciprocal controllers also weaken the reciprocal assumption.
Infeasible configurations minimize violation instead of guaranteeing separation.
Coincident stationary centres use a fixed recovery axis, and initial overlaps
cannot be undone instantaneously. The crowd retains its existing penetration
correction and corridor projection after acceleration limiting.

## Reproduction

`reference/harness/orca-oracle.cc` is a public-API adapter for
[RVO2](https://github.com/snape/RVO2), built separately outside this repository.
RVO2 is an Apache-2.0 test oracle; none of its implementation is copied or
translated into the crate. Build the adapter against a pinned public source tree
and pass its executable and `nav-orca` to `reference/orca_compare.py`:

```sh
: "${RVO2_SOURCE:?}" "${RVO2_ORACLE:?}" "${CARGO_TARGET_DIR:?}"
c++ -O2 -fno-fast-math -std=c++11 -I"$RVO2_SOURCE/src" \
    reference/harness/orca-oracle.cc "$RVO2_SOURCE"/src/*.cc -o "$RVO2_ORACLE"
cargo build --locked --release -p navmesh-parity --bin nav-orca
python3 reference/orca_compare.py --port "$CARGO_TARGET_DIR/release/nav-orca" \
    --oracle "$RVO2_ORACLE" --output oracle-results.json
```

The comparison uses a fixed seed, 100,000 frozen random configurations and 400-step standard
scenarios with identical oracle-generated inputs on every step; its declared
velocity tolerance is `1e-4 m/s`. It reports all deviations and exits nonzero when
the tolerance is exceeded.

`cargo run --release -p navmesh-parity --bin nav-orca -- digest` emits crowd
trajectory digests over 60 seconds, including the collision phases. `invariants` reports overlaps deeper than 1 mm, deepest
penetration, unreached agents, stationary deadlocks (under 5 cm displacement
over the final 20 seconds with no arrival), and completion time on the registered scenes.
`measure` performs ten warmup ticks and nine interleaved, alternating-order pairs
of ten ticks at 100, 1,000 and 5,000 agents, reporting median microseconds per
agent-tick against sampled quality 3. Pin the process to one CPU and keep the
command/environment with the results. These measurements include crowd update and
preferred-velocity preparation.

See [qualification decisions and retained failures](ORCA-RESULTS.md) for the current acceptance record.
