# ORCA qualification record

Status: **not qualified for integration**. Implementation checks and unchanged
default-mode evidence do not establish the requested oracle, nonpenetration and
performance acceptance criteria. The opt-in implementation is a review candidate.
The sampled planner remains the default.

Earlier sections retain historical measurements. The final section describes
the current candidate, comparison policy and stop decision.

## Decisions

- Preserve `obstacle_avoidance_type` as the sampled preset selector, adding
  `AvoidanceMode::Sampled` / `Orca(OrcaParams)` instead of adding a second preset
  field to the enum. Existing sampled configuration keeps its meaning.
- Derive the solver from the paper. Use RVO2 only through a separate public-API
  executable; do not copy or translate its implementation.
- Use the paper's zero optimization velocity for two-sided wall segments.
  Polygon-oracle endpoint handling is a measured compatibility limit.
- Keep the hard speed bound even when the oracle exceeds it. Do not loosen the
  `1e-4 m/s` comparison threshold to conceal those cases.
- Keep full candidate visits and charges in the shared corrected-neighbour path.
  Use a stamp table and sorted insertion to reduce overhead. Range pruning was
  rejected because the specified collection policy visits the full query range.
- Keep default and corrected sampled goldens unchanged; store new default-feature
  ORCA trajectory digests separately.

## Sources and method

Base revision: `0cb318613e078a21f1d0f3aad75efd99434f5fca` (0.2.1).
The implementation uses equations (5)-(8), the projected epigraph LP in section
5.3, and the static-segment construction in section 5.4 of
[van den Berg et al. (2011)](https://doi.org/10.1007/978-3-642-19457-3_1).
The Apache-2.0 oracle is
[RVO2 at `6b4eb00a93a9f3abf723235ff006d6aad3e2f212`](https://github.com/snape/RVO2/tree/6b4eb00a93a9f3abf723235ff006d6aad3e2f212).
Its downloaded archive SHA-256 is
`d92c7d73be7548b955856d54367e634d58562eaf4db51cd79b5b444910ea4396`.
Before the later numerical diagnosis, only its licence, build configuration and
public headers had been inspected.

The oracle comparison uses 100,000 fixed-seed random configurations, 548,574
agent velocities, and 400-step circle, crossing, corridor and static-segment
scenarios. Both solvers receive identical frozen oracle-generated states at every
step. Inputs include overlapping and initially over-speed agents. Failures are
retained individually; those adversarial states are not silently filtered away.

Native qualification uses Rust 1.99.1 on x86-64 Linux, an AMD Ryzen 9 7945HX.
Timing is pinned to one allowed CPU, with ten warmup ticks followed by nine
alternating-order pairs of ten ticks. Values are median microseconds per agent
per tick and include preferred-velocity preparation and crowd update. The shared
host was not an isolated benchmark machine.

## Initial oracle evidence

| Inputs | Velocities | Maximum error, m/s | Above `1e-4 m/s` |
|---|---:|---:|---:|
| Random configurations | 548,574 | 2.38349138 | 202 |
| Circle, 64 agents | 25,600 | 0.0000337435 | 0 |
| Crossing groups | 12,800 | 0.00000396146 | 0 |
| Corridor | 12,800 | 1.041500733 | 8 |
| Final staggered static segments | 12,800 | 1.43884319 | 821 |

Of the 202 failing random configurations, 185 have initial overlaps, 175 have
initial velocities exceeding their speed limit, and 130 produce an oracle output
above the requested speed limit. Both failures with initially disjoint,
speed-compliant agents also produce over-speed oracle outputs. These categories
overlap; they do not excuse the remaining deviations.

The worst oracle output is `(2.80224609, -3.08642578)` at a requested speed limit
of 2 m/s. The candidate returns `(1.76511538, -0.940408289)` within that limit.
Matching this oracle output within the declared tolerance would contradict the
speed-bound invariant. No reference-library implementation was read to diagnose
this discrepancy; it is a public-API counterexample.

The initial static fixture duplicated the corridor. It was replaced by two
staggered finite vertical segments before the final comparison; that initial
fixture is not claimed as independent static-obstacle evidence.

## Initial long-run crowd evidence

Initial overlap events count pairs with penetration deeper than 1 mm, once per
tick. The initial collector reused the crowd's pre-integration spatial grid. Its
search radius included motion margin, but large penetration corrections can
invalidate that assumption. Final measurement rebuilds a separate grid from final
positions before collecting pairs; initial counts are retained rather than
silently substituted.
Unreached counts mean no arrival within 0.5 m of the goal by the stated timeout;
they are not measurements of deadlock. Acceleration is 100 m/s², maximum speed
2 m/s, radius 0.4 m, step 0.05 s, ORCA horizons 2/1 s, cap 16 and range 8 m.
The comparison uses sampled quality 3.

| Scene | Sampled overlap events | ORCA overlap events | ORCA deepest penetration, m | Sampled / ORCA unreached |
|---|---:|---:|---:|---:|
| Circle 64, 70 s maximum | 16,061 | 1,478 | 0.03402561 | 0 / 0 |
| Circle 512, 120 s | 1,681,531 | 107,058 | 0.19442379 | 0 / 355 |
| Crossing 128, 120 s maximum | 6,511 | 199 | 0.01453370 | 48 / 0 |
| Corridor 64, 120 s maximum | 3,411 | 1,762 | 0.04027182 | 8 / 0 |
| Doorway 64, 70 s | 196,191 | 1,770 | 0.04762995 | 53 / 35 |
| Dense 2,000, 70 s maximum | 616,172 | 0 | 0 | 0 / 0 |

| Scene | Sampled / ORCA measured duration, s | Sampled / ORCA overlap events per agent-second |
|---|---:|---:|
| circle 64 | 30.90 / 53.00 | 8.121460 / 0.435731 |
| circle 512 | 95.15 / 120.00 | 34.516450 / 1.742480 |
| crossing 128 | 120.00 / 104.00 | 0.423893 / 0.014949 |
| corridor 64 | 120.00 / 116.55 | 0.444141 / 0.236218 |
| doorway 64 | 70.00 / 70.00 | 43.792634 / 0.395089 |
| dense 2000 | 56.55 / 50.40 | 5.448028 / 0.000000 |

ORCA reduces overlap events but does not meet the absolute 1 mm nonpenetration
criterion. Circle-512 progress also remains incomplete at the selected timeout.
The isolated two-agent 2,000-step test passes separation and goal progress, and
10,000 adversarial pure-solver configurations pass finite-output and speed bounds.
A fourth pure-solver test pins the worst overlapping oracle input by its exact
f32 bits and verifies that the infeasible LP still respects the speed disc.
These focused tests do not replace the failed multi-agent acceptance evidence.

## Initial cost evidence

| Agents | Original sampled | Original ORCA | Stamp-table sampled | Stamp-table ORCA |
|---:|---:|---:|---:|---:|
| 100 | 2.438730 | 2.248761 | 4.188822 | 2.806796 |
| 1,000 | 2.616826 | 10.546971 | 4.041952 | 7.825665 |
| 5,000 | 2.325769 | 12.367541 | 3.740471 | 7.969567 |

The absolute timings changed with host conditions. Compare interleaved ratios
within each run. Both recorded runs fail the no-higher-cost criterion at 1,000
and 5,000 agents. Original measurements are retained alongside subsequent runs.

## Final cost evidence

The final sorted-insertion optimization retains full-range collection and the
same nearest sets. Its focused Clippy, crowd/fixes/ORCA tests, build and trajectory
digest comparison all exit 0. The full 100,000-configuration and standard-scenario
oracle comparison still exits 1; static obstacles now use the distinct fixture.

| Agents | Sampled, µs/agent-tick | ORCA, µs/agent-tick | ORCA / sampled |
|---:|---:|---:|---:|
| 100 | 2.486224 | 2.067104 | 0.831423 |
| 1,000 | 3.250257 | 6.316956 | 1.943525 |
| 5,000 | 3.495388 | 7.552991 | 2.160845 |

The acceptance criterion still fails at 1,000 and 5,000 agents. Candidate
collection is intentionally exhaustive; no work charge, sample preset or
comparison threshold was weakened to improve this result.

## Implementation and default-mode evidence

The first complete implementation matrix exited 0 for formatting, all requested
Clippy configurations, default/parallel/32-bit tests, no_std builds, tile-cache
variants, rustdoc, WebAssembly library build, MSRV 1.88, both fuzz-target builds,
pure ORCA tests and the unmodified public-text guard. No fuzz campaign was run.

Full default parity compares 76,544 records in each reference width across 22
cases, with zero failures. Existing determinism files remain unchanged: 3,116
normal and 27 corrected sampled digests. Native and WASI checks pass. The initial six new ORCA trajectory digests matched natively and under WASI,
using default features, but covered only six seconds. That missed the interaction
phase of the larger scenes. Final digests cover sixty seconds; the initial hashes
are retained separately.
Initial serial ORCA results also match with thread-count environment settings 1/4/16.
AArch64 execution was not performed locally; its CI job now checks the ORCA file.

Initial corpus attempts are retained: one used an outdated adapter that rejected
`update_once`; another paired a 32-bit oracle with the 64-bit port. Neither is
counted as passing evidence. Rebuilding the current adapter against the pinned
upstream source and matching reference widths produced the passing comparisons.

## Reviewer attack points

1. Normal/offset conditioning, near-parallel constraints and the projected 3D
   fallback in `src/crowd/orca.rs`, especially where the oracle exceeds its speed
   bound or infeasible programs have multiple equivalent minimizers.
2. Two-sided wall endpoints, conflicting static constraints, finite local-boundary
   capacity and the zero optimization velocity used for walls.
3. Full-query stamp indexing and nearest ordering in `src/crowd/grid.rs` and
   `update.rs`, followed by acceleration, penetration correction and corridor
   projection, which can invalidate velocity-space separation assumptions.

A local read-only adversarial pass recorded the three attack areas above and the
oracle speed-bound counterexample. Independent review and integration remain for
the maintainer. The acceptance status is not promoted by passing implementation
gates, deterministic replay or improved cost measurements.

## Final verification receipt

The overlap collector now rebuilds a separate spatial grid from final positions
after every tick. The stationary-deadlock detector counts agents with no recorded
arrival and less than 5 cm displacement over the final 20 simulated seconds.
It is an operational detector, not a proof of permanent deadlock.

| Scene | Sampled / ORCA overlap events | ORCA deepest penetration, m | Sampled / ORCA unreached | Sampled / ORCA stationary deadlocks |
|---|---:|---:|---:|---:|
| circle 64 | 16,061 / 1,478 | 0.03402561 | 0 / 0 | 0 / 0 |
| circle 512 | 1,681,531 / 107,058 | 0.19442379 | 0 / 355 | 0 / 0 |
| crossing 128 | 6,511 / 199 | 0.01453370 | 48 / 0 | 5 / 0 |
| corridor 64 | 3,411 / 1,762 | 0.04027182 | 8 / 0 | 0 / 0 |
| doorway 64 | 196,191 / 1,770 | 0.04762995 | 53 / 35 | 0 / 0 |
| dense 2000 | 616,172 / 0 | 0.00000000 | 0 / 0 | 0 / 0 |

| Scene | Sampled / ORCA duration, s | Sampled / ORCA overlap events per agent-second |
|---|---:|---:|
| circle 64 | 30.90 / 53.00 | 8.121460 / 0.435731 |
| circle 512 | 95.15 / 120.00 | 34.516450 / 1.742480 |
| crossing 128 | 120.00 / 104.00 | 0.423893 / 0.014949 |
| corridor 64 | 120.00 / 116.55 | 0.444141 / 0.236218 |
| doorway 64 | 70.00 / 70.00 | 43.792634 / 0.395089 |
| dense 2000 | 56.55 / 50.40 | 5.448028 / 0.000000 |

Final command exit codes (metric collection and acceptance are separate):

| Check | Exit code |
|---|---:|
| `format` | 0 |
| `fmt` | 0 |
| `clippy-parallel` | 0 |
| `clippy-32` | 0 |
| `clippy-all` | 0 |
| `test` | 0 |
| `test-parallel` | 0 |
| `test-parallel-32` | 0 |
| `test-32` | 0 |
| `nostd` | 0 |
| `tilecache-test` | 0 |
| `tilecache-nostd` | 0 |
| `rustdoc` | 0 |
| `wasm-build` | 0 |
| `msrv` | 0 |
| `fuzz` | 0 |
| `driver` | 0 |
| `orca-unit` | 0 |
| `public` | 0 |
| `sampled-goldens` | 0 |
| `orca-new-goldens` | 0 |
| `orca-native` | 0 |
| `threads-1` | 0 |
| `threads-1-diff` | 0 |
| `threads-4` | 0 |
| `threads-4-diff` | 0 |
| `threads-16` | 0 |
| `threads-16-diff` | 0 |
| `orca-wasi` | 0 |
| `sampled-wasi` | 0 |
| `invariants` | 0 |
| `fuzz-format` | 0 |
| `fuzz-fmt-check` | 0 |
| `parity-build64` | 0 |
| `parity-port64` | 0 |
| `parity64` | 0 |
| `parity-build32` | 0 |
| `parity-port32` | 0 |
| `parity32` | 0 |
| `acceptance` | 1 |
| `public-final` | 0 |

The unchanged sampled files contain 3,116 normal and 27 corrected digests.
The new ORCA file covers 1,200 ticks (60 seconds) per scene, including interaction
phases. Its native/WASI and thread-setting comparisons are listed above.

```text
orca circle 64 3187dd7068a943e7
orca circle 512 daa2977f8d3e13d4
orca crossing 64 e3c5fbb35567b4f4
orca corridor 64 2192d4720ff05908
orca doorway 64 15fd7836066d29d1
orca dense 2000 d6e51a0ec2509332
```

The final cost receipt is unchanged by the later metric-collector correction;
that correction affects only `invariants`, not the measured crowd-update path.
The measured executable and implementation checksums are retained separately.

Decision: **not qualified for integration**. Raw oracle errors remain above
`1e-4 m/s`, crowd penetration exceeds 1 mm, progress is incomplete in the large
circle, and costs at 1,000/5,000 agents exceed sampled quality 3. Passing
implementation, parity and replay checks does not satisfy those acceptance bars.


## Fix-round decisions

The earlier measurements above are retained as the pre-fix record. The current
candidate changes the following mechanisms; qualification results follow below.

- Normalize agent-plane normals and offsets together, then retain distance
  equations, disc intersections and projected epigraph constraints in double
  precision. Intersect the actual normal with the speed disc rather than assuming
  that a rounded normal has exactly unit length. Keep the final speed clamp.
- For static segments, use the current velocity as the projection objective.
  This deliberately replaces section 5.4's zero-velocity choice for compatibility
  with the public-API oracle. Independently construct the truncated velocity
  obstacle from its cutoff segment, visible endpoint arcs and supporting tangent
  rays. Reject hidden endpoint features. Static constraints remain before agent
  constraints in the LP. Outside an obstacle, zero velocity remains feasible.
- Restrict ORCA collection to a smaller square only when at least the neighbour
  cap of eligible centres lie inside its inscribed circle. All centres closer
  than those are included, so sorting by distance and index returns exactly the
  full-range nearest set. When that proof fails, clear the first query's stamps
  and repeat the full-range query. Charge both queries and their candidate scans.
  Corrected and default sampled collection retain their existing code paths.
- Keep the integration order: every avoidance velocity is computed before any
  current velocity or position changes. Acceleration clamping, overlap correction
  and corridor projection still follow the frozen-state solve. Those operations
  can invalidate a velocity-only separation guarantee; changing their sampled
  behavior is outside this change.

The focused regressions check a rounded-normal boundary against both its plane
and speed disc, and a finite endpoint tangent against geometric contact and
endpoint reversal. Existing crowd tests compare retained neighbours with brute
force and exercise budget exhaustion without committing a partial update.

## Fix-round measurements and stop decision

Status: **not qualified for integration**. The stop condition is reached for
strict oracle matching while retaining the speed disc. No remaining difference
is granted a floating-point exemption. Penetration also fails its requested bar. The final circle deadline comparison
is recorded below; passing cost and implementation checks cannot promote this
candidate.

### Frozen-state comparison

The same 100,000 configurations and frozen 400-step scenarios were rerun.

| Inputs | Velocities | Maximum error, m/s | Above `1e-4 m/s` |
|---|---:|---:|---:|
| Random configurations | 548,574 | 2.383491385 | 202 |
| Circle, 64 agents | 25,600 | 0.0000337434 | 0 |
| Crossing groups | 12,800 | 0.00000311885 | 0 |
| Corridor | 12,800 | 1.041500673 | 8 |
| Staggered static segments | 12,800 | 1.079127356 | 4 |

The endpoint construction removes 817 of the 821 static-obstacle differences.
The conditioning change does not remove the 202 random or eight corridor
failures. Those results reject the hypothesis that our LP precision alone caused
all of the original oracle gaps.

For 108 failing random velocities, the oracle speed exceeds the speed limit by
more than the comparison tolerance. The worst output is still
`(2.80224609, -3.08642578)`, whose norm is `4.168765674 m/s` at a `2 m/s` limit.
For every speed-bounded candidate `v`, the reverse triangle inequality gives
`|v - oracle| >= |oracle| - 2 = 2.168765674 m/s`. This is an analytical lower
bound derived from a measured public-API output, far above `1e-4 m/s`.
Retaining the speed invariant makes strict matching of this input impossible.
The exact input remains pinned by the overlapping-snapshot regression.

An unchanged oracle source build using extended intermediate precision
(`-mfpmath=387`, without fast-math) was replayed on every retained failure.
43 of the 202 random discrepancies then fall below tolerance; the largest change
between oracle builds is `3.096233174 m/s`. However, the remaining random error
can still reach `2.320489693 m/s`. None of the 12 corridor/static discrepancies
is resolved; their oracle precision changes are at most `0.000000477 m/s`.
This proves sensitivity for some cases, not an oracle-side float-order
classification for all residuals. The remaining 94 random velocities without the
strict speed contradiction and all 12 scenario velocities remain unresolved.
Both precision outputs and every failing scenario input are retained externally.

### Crowd separation and progress

An initial public-API oracle run measures the circle arrival deadline rather
than treating the old 120-second timeout as the progress criterion. It uses the
same quantized initial positions, goal-velocity policy, radii, timestep, range and
cap as the crowd fixture. Circle-64 finishes at `92.55 s`; circle-512 finishes at
`193.60 s`, so its acceptance deadline is `212.96 s`. The oracle itself reaches
penetration depths of `0.0685141 m` and `0.148379 m`, respectively; it is not a
reference proof of the absolute nonpenetration bar.

The candidate collector rebuilds its independent grid after integration,
correction and corridor projection. Circle-512 now allows up to 600 simulated
seconds to measure actual arrival; the other timeouts are retained.

| Scene | ORCA deepest penetration, m | Unreached | Arrival or timeout, s |
|---|---:|---:|---:|
| Circle 64 | 0.03428417 | 0 | 61.40 |
| Circle 512 | 0.12550938 | 0 | 220.90 |
| Crossing 128 | 0.01076978 | 0 | 100.45 |
| Corridor 64 | 0.04027081 | 0 | 113.85 |
| Doorway 64 | 0.03034782 | 28 | 70.00, timeout |
| Dense 2,000 | 0 | 0 | 50.40 |

Five scenes fail the `0.01 m` penetration bar. Circle-512 eventually reaches all
its goals, but takes about 14.1% longer than the oracle and misses the 10% bar.
These initial deadline figures are superseded by the operation-order check
below. Its former unreached count at 120 seconds was an inadequate progress comparison.
No stationary deadlocks are reported for the candidate by the retained detector.
Frozen-state solving is already ordered before integration; changing that phase
order is not supported by this evidence. Acceleration, capped neighbour sets,
position correction and corridor movement still limit velocity-space guarantees.

### Pinned interleaved cost

The unchanged benchmark uses ten warmup ticks and nine alternating-order pairs
of ten ticks on one pinned CPU. These are medians from the same executable and
run; absolute values should not be compared with the historical host timings.

| Agents | Sampled, µs/agent-tick | ORCA, µs/agent-tick | ORCA / sampled | At most 1.5× |
|---:|---:|---:|---:|---|
| 100 | 2.467225 | 2.520654 | 1.021656 | Yes |
| 1,000 | 2.393968 | 3.127606 | 1.306452 | Yes |
| 5,000 | 2.471737 | 3.194594 | 1.292449 | Yes |

The smaller proved-complete queries pass both requested cost bars without
changing retained neighbours or the sampled planner. The first IBS attempt
failed before execution because passwordless privilege elevation was unavailable;
that failure is retained separately from the unprofiled cost run.

### Implementation and replay checks

The native matrix exits 0 for formatting; default, parallel, 32-bit and
all-feature Clippy with warnings denied; default, parallel and 32-bit tests;
no_std; tilecache-only variants; rustdoc with warnings denied; WebAssembly;
MSRV; all fuzz-target builds; both budget cases; and the unchanged public-text
guard. Fuzz targets were built, not campaigned. The rounded-normal, finite-wall
and brute-force nearest-neighbour regressions all pass.

The sampled golden files remain byte-identical: 3,116 default and 27 corrected
digests match natively and under WASI. The old ORCA golden comparison exits 1,
retained as evidence of the intentional trajectory change. The replacement six
60-second ORCA digests match natively and under WASI; they establish replay
consistency, not physical acceptance. Remote AArch64 execution was not performed.

An initial runner edit overlapped its execution, causing a shell exit-127 error
and duplicated Clippy checks. That runner log is retained. Its individually
completed checks pass, but an immutable clean runner is used for the final
verification receipt rather than interpreting the wrapper status as acceptance.

### IBS attribution

Unprivileged per-process `ibs_op` sampling succeeds on the same pinned CPU and
benchmark executable: 41,560 samples, zero lost samples and six out-of-order
events. The combined alternating sampled/ORCA run attributes 10.53% of samples
to ORCA's complete grid collector, 11.49% to sampled grid collection, 4.77% to
neighbour sorted insertion, 3.79% to ORCA velocity selection, 2.27% to agent-plane
construction and 0.59% to the ORCA LP solver. The shared crowd update accounts for
24.92%; sampled penalty evaluation and adaptive sampling account for 9.73% and
6.13%, respectively. These are sampled exclusive symbol percentages across both
modes, not per-mode elapsed-time fractions. They support targeting collection and
ordering rather than weakening the LP to satisfy the cost bar.

The raw IBS recording, resolved user-symbol report and failed privileged attempt
are retained. Kernel symbols were restricted and frame-pointer call chains are
incomplete; no kernel or complete inline-stack attribution is claimed. Profiled
timings include sampling overhead and are not substituted for the unprofiled
interleaved cost table above.

### Final circle deadline with matching operation order

The first oracle run multiplied each goal delta by a precomputed speed/distance
ratio. The crowd fixture divides each component by distance before multiplying
by speed. A second oracle run matches that precise operation order, retaining
the first experiment separately. This changes only the test harness, not the
candidate or the frozen-state comparisons.

| Agents | Oracle unreached | Oracle finish, s | Oracle deepest penetration, m |
|---:|---:|---:|---:|
| 64 | 0 | 60.50 | 0.06852560 |
| 512 | 0 | 189.60 | 0.13489600 |

The final circle-512 deadline is **208.56 s**. The candidate reaches all
goals at **220.90 s**, 16.51% beyond the oracle time: **fails**
the plus-10% progress bar. Floating-point operation order materially affects this
symmetric crowd, so the deadline is bound to this exact fixture and executable.

### Final clean gate receipt

All checks in the immutable clean runner exit 0; its wrapper also exits 0.

| Check | Exit code |
|---|---:|
| `ibs-user` | 0 |
| `ibs-user-report` | 0 |
| `fmt` | 0 |
| `clippy-default` | 0 |
| `clippy-parallel` | 0 |
| `clippy-32` | 0 |
| `clippy-all` | 0 |
| `test` | 0 |
| `test-parallel` | 0 |
| `test-parallel-32` | 0 |
| `test-32` | 0 |
| `nostd` | 0 |
| `tilecache-test` | 0 |
| `tilecache-nostd` | 0 |
| `rustdoc` | 0 |
| `wasm-build` | 0 |
| `msrv` | 0 |
| `fuzz` | 0 |
| `public` | 0 |
| `sampled-goldens` | 0 |
| `budget-solo` | 0 |
| `budget-tiled` | 0 |
| `orca-new-goldens` | 0 |
| `orca-native` | 0 |
| `orca-wasi` | 0 |
| `sampled-wasi` | 0 |
| `public-final` | 0 |

Retained nonpassing diagnostics: frozen-state oracle acceptance `1`, retired
ORCA golden comparison `1`, first privileged IBS attempt `1`, its unavailable
report `255`, and the initial runner shell error `127`. None is relabelled as a
passing result. No fuzz campaign or Git integration was performed.

The final ORCA replay digests, separately from the unchanged sampled goldens, are:

```text
orca circle 64 dbc8b97a613414bd
orca circle 512 12e27f0d7c818b60
orca crossing 64 001a712a65dec884
orca corridor 64 8c2c756465196406
orca doorway 64 c6bb2e3edf2c9475
orca dense 2000 b0dabb5563add10f
```

Decision: stop with the best measured candidate, **not qualified for integration**.
Strict oracle matching contradicts the speed disc on the retained counterexamples;
the alternative numerical exemption is not established for all residuals.
The separation bar still fails. Cost, implementation gates and deterministic
replay pass, and do not substitute for physical acceptance.


## Relative-reference qualification

Status: **not qualified for integration**. The relative test is measured, not
substituted for the historical failures above. The sampled planner remains the
default. No absolute acceptance test was weakened. The new relative evaluator
fails closed on missing arrival tails and unexplained bounded fallback changes.

### Changes and mismatch classification

An exact directional tie in the projected LP selected the wrong interval endpoint
for the chosen tangent orientation. Reversing a tangent requires reversing its
zero-dot-product tie decision too. Selecting the upper tangent endpoint removes
all eight corridor and four static-segment discrepancies. The focused regression
pins this endpoint choice independently of the crowd trajectories.

The final f32 velocity can round just outside the speed disc even after an
otherwise correct clamp. Measure the rounded vector in double precision and,
only if outside, leave one f32 epsilon of radial margin. Apply the same check
after acceleration interpolation, which can round outside again. A `(0.6, 0.8)`
regression and the strengthened 10,000-state speed test pin this invariant.
The sampled integration path is unchanged. For ORCA, reducing `max_speed`
below the current speed enforces the new speed bound immediately; in that
recovery case the hard speed invariant takes priority over gradual deceleration.
The fixed qualification fixtures do not change their speed limits.

The final frozen-state comparison retains the original 100,000 configurations,
548,574 random velocities and four 400-step scenarios, with the original
`1e-4 m/s` threshold:

| Inputs | Remaining mismatches | Maximum error, m/s |
|---|---:|---:|
| Random | 202 | 2.383491595 |
| Circle | 0 | 0.0000339074 |
| Crossing | 0 | 0.00000311885 |
| Corridor | 0 | 0.0000100155 |
| Static segments | 0 | 0.0000881563 |

Every retained random velocity has an individual classification and a certificate:

| Class, in priority order | Count | Evidence and limit |
|---|---:|---|
| Reference speed contradiction | 108 | Reference speed exceeds the disc by more than `1e-4 m/s`; the worst analytical velocity-error lower bound remains `2.168765674 m/s`. |
| Demonstrated precision-sensitive fallback | 33 | An unchanged-source extended-intermediate-precision reference build reduces each error below `1e-4 m/s`; maximum residual `0.0000920714 m/s`, maximum original error `0.279261726 m/s`. |
| Bounded infeasible-fallback behavioral difference | 61 | Candidate output is certified near-optimal for the reference's observable normalized planes, but differs by up to `1.550569510 m/s`; extended precision still leaves errors up to `1.608031592 m/s`. |

All 202 programs are infeasible. The independent certificate uses only public API
half-planes, not reference implementation code. For any nonnegative weights
summing to one, the minimum worst violation inside a speed disc is at least
`sum(w*c) - speed*norm(sum(w*n))`. Single-plane, two-plane and three-plane dual
supports supply a lower bound; evaluating the candidate supplies an upper bound.
Across all cases the largest upper-minus-lower gap is `0.000000731 m/s`.
For the 61 behavioral residuals, every infeasibility lower bound exceeds
`0.290774 m/s`; the reference's own worst-violation gap can reach
`0.601191 m/s`. This is evidence against replacing the candidate's valid minimax
solution with an arbitrary reference output. It does not establish the precise
arithmetic cause of all 61 differences, nor a small velocity-error bound.

The extended-precision build resolves 43 cases overall; ten already belong to the
higher-priority speed-contradiction class. No case is double-counted. No numerical
exemption is granted. The 61 residuals remain an explicit compatibility failure;
no candidate minimax defect was found that would justify changing its fallback
to reproduce them. `reference/orca_classify.py` reproduces the classification and
stdlib dual bounds from retained inputs and a public-plane diagnostic executable.
Its exit 1 intentionally preserves that behavioral failure.

### Common metric definitions and native reference measurements

`nav-orca relative-metrics` collects all six existing crowd fixtures. Supplying
the separately built `reference/harness/orca-trajectory.cc` executable uses the
same Rust metric collector on reference trajectories. Initial positions and goals
are exported from the actual crowd fixture as round-tripping f32 decimals.
Radius, timestep, speed, horizons, neighbour range/cap and goal-speed arithmetic
match the fixture. Reference motion uses its own public simulator API, including
its own integration and obstacle processing. No reference code enters the crate.

The independent final-position grid enumerates unordered pair-ticks without the
avoidance cap. Penetration includes **all positive** depths; p99 below is the
nearest-rank percentile conditional on positive penetration, not a percentile
padded with noninteracting pairs. Zero means no positive penetration. Arrival is
the first tick within 0.5 m of the goal; agents continue participating until all
have arrived or the original scene timeout. Arrival percentiles rank all agents;
missing tail ranks remain unmeasured rather than being replaced by the timeout.
Stationary deadlocks mean unarrived agents moving less than 5 cm over the final
20 seconds, using a rolling 401-position history. This is an operational detector.

The standalone reference has no navigation funnel. Its doorway uses an explicit
opening waypoint; the candidate uses its existing corridor steering. Doorway
standalone navigation therefore cannot qualify an equal-policy arrival ratio.
An equal-navigation control would be needed to qualify that distinction; its
unmeasured status is recorded below.

| Scene | Reference / candidate maximum penetration, m | Reference / candidate positive-depth p99, m | Reference / candidate arrival p100, s | Reference / candidate unreached |
|---|---:|---:|---:|---:|
| Circle 64 | 0.068525612 / 0.034274042 | 0.025568247 / 0.016960919 | 60.50 / 49.35 | 0 / 0 |
| Circle 512 | 0.134895742 / 0.171704173 | 0.036157608 / 0.036863089 | 189.60 / 173.25 | 0 / 0 |
| Crossing 128 | 0.035444796 / 0.014600396 | 0.008958638 / 0.004372895 | 101.90 / 102.90 | 0 / 0 |
| Corridor 64 | 0.060882211 / 0.040270925 | 0.048327625 / 0.027207255 | unmeasured / unmeasured | 1 / 1 |
| Doorway 64 | 0.031862676 / 0.021192491 | 0.011198580 / 0.005482495 | unmeasured / unmeasured | 24 / 28 |
| Dense 2,000 | 0 / 0 | 0 / 0 | 50.40 / 50.40 | 0 / 0 |

| Scene | Reference arrival p50 / p95 / p99, s | Candidate arrival p50 / p95 / p99, s | Reference speed violations above 1e-6 m/s |
|---|---|---|---:|
| Circle 64 | 46.05 / 59.80 / 60.50 | 42.55 / 47.00 / 49.35 | 2 |
| Circle 512 | 152.85 / 177.90 / 185.35 | 140.70 / 158.00 / 163.50 | 59 |
| Crossing 128 | 91.45 / 96.70 / 100.80 | 91.75 / 97.20 / 102.10 | 12 |
| Corridor 64 | 97.00 / 107.35 / unmeasured | 97.90 / 108.95 / unmeasured | 2 |
| Doorway 64 | 61.50 / unmeasured / unmeasured | 58.25 / unmeasured / unmeasured | 1 |
| Dense 2,000 | 50.40 / 50.40 / 50.40 | 50.40 / 50.40 / 50.40 | 0 |

Both implementations report zero stationary deadlocks in all six fixtures.
The candidate has **zero strictly positive speed excesses** in every collected
state, without a tolerance. The standalone reference has 76 excesses above
`1e-6 m/s`; its maximum excess is `0.010719477 m/s` in circle-512. The tolerated
reference counter is reported separately from the candidate's strict counter.

Circle-512 now passes the arrival bar (`173.25 <= 189.60 * 1.05 = 199.08 s`),
but fails relative maximum penetration
(`0.171704173 > 0.134895742 + 0.01 = 0.144895742 m`). Its positive-depth p99
passes. Missing corridor and doorway arrival tails are not passing evidence.

### Final cost

The unchanged pinned benchmark uses ten warmup ticks and nine alternating-order
pairs of ten ticks; medians include preferred-velocity preparation and crowd
update. Shared-host timings remain diagnostic, with the within-run ratios used
for the requested cost comparison.

| Agents | Sampled, microseconds/agent-tick | Candidate, microseconds/agent-tick | Ratio |
|---:|---:|---:|---:|
| 100 | 2.599019 | 2.445856 | 0.941069 |
| 1,000 | 2.522081 | 3.074417 | 1.219000 |
| 5,000 | 2.585293 | 3.279537 | 1.268536 |

All three measured ratios pass 1.5. Neither the neighbor set nor sampled quality
was reduced to obtain those ratios. The new speed checks are included.



### Equal-navigation control: unmeasured

A copied-source diagnostic backend was prepared to substitute a separate
public-API reference process for velocity selection while retaining the crowd's
navigation and original integration. The first run reused a cached candidate
executable after switching copied sources in a shared build directory. Its
exit-0 output is **invalid reference evidence**, as confirmed by the absence of
the reference protocol marker in the binary. That output is retained separately.
A distinct build fingerprint exposed a missing qualified string import in the
scratch backend; the failed build exits 101 and is retained.

The corrected diagnostic was never admitted. After more than 36 minutes under
the required reservation, its owned queued wrapper was terminated and exited
143. No admitted job or other operator's job was canceled. No equal-navigation
control result is claimed. The native circle-512 comparison already fails the
relative maximum-penetration bar without a navigation-policy difference, so this
additional diagnostic cannot promote the current candidate. The standalone
doorway comparison remains explicitly limited by its different steering policy.


### Gate receipt and decision

The immutable final implementation-gate runner exits 0. Every implementation
and replay row below passes; acceptance and unadmitted-diagnostic rows retain
their nonzero codes separately. The measured and gated implementation checksums
match the final worktree. The 3,116 default and
27 corrected sampled records remain byte-identical to the base revision.

| Final checks | Exit code |
|---|---:|
| Formatting; default, parallel, 32-bit/tilecache and all-feature Clippy with warnings denied | 0 |
| Default, workspace parallel, parallel 32-bit and crowd 32-bit tests | 0 |
| no_std crowd; tilecache-only std tests and no_std build | 0 |
| Rustdoc with warnings denied; WebAssembly build; MSRV check | 0 |
| All fuzz-target builds, without campaigns | 0 |
| Unchanged public-text guard | 0 |
| Sampled native and WASI determinism; both corpus budget cases | 0 |
| Replacement ORCA native and WASI determinism | 0 |
| Frozen-state comparison | 1 |
| Mismatch classification acceptance | 1 |
| Relative standalone acceptance | 1 |
| Additional equal-navigation control, canceled before admission | 143 |

The replacement ORCA-only 60-second digests are:

```text
orca circle 64 4ba62029a50a6966
orca circle 512 3159a1e880a76949
orca crossing 64 3665ccd25aea1c32
orca corridor 64 70cb542664397110
orca doorway 64 d1b31fde9b478eac
orca dense 2000 2bb7a7fd69ca5e5c
```

Retained nonpassing attempts are distinct from the final implementation gates:
three initial Clippy checks exit 101 for collector formatting/lint mistakes;
initial native/WASI ORCA and the explicit retired-golden comparison exit 1;
the first numerical-analysis environment failed to import its installed numeric
packages. The final dual certificate uses only the Python standard library.
A diagnostic copied-source build initially reused a cached candidate executable;
its exit-0 metrics are invalidated and retained, not counted as reference evidence.
The subsequent diagnostic rebuild exposed a missing qualified string import
and exited 101; that scratch-only error is corrected separately from production.

Decision: **not qualified for integration**. Keep the deterministic tie and strict
speed fixes, the sampled default and all failure evidence. The measured speed
correction changes symmetric trajectories enough to improve arrival and worsen
maximum penetration; small local velocity changes cannot be extrapolated into
physical acceptance. Removing the clamp would violate the invariant. Replacing
a certified minimax result with an unexplained bounded reference output would
trade correctness for an unsupported compatibility rule. Broader position
correction or neighbor-policy changes have no qualifying evidence in this round.
Stop without promoting the relative penetration failure or unmeasured arrival
tails. No commits, branch/index changes, integration or fuzz campaigns were made.


## Further qualification and equal-navigation control

Status: **not qualified for integration across all required fixtures**. The
circle-512 maximum-penetration gap is fixed in the measured candidate, and all
202 frozen-state mismatches now have retained causal or analytical evidence.
Corridor and doorway arrival p100 remain unmeasured at the frozen timeouts.
Those missing tails cannot satisfy the arrival acceptance test. The sampled
planner remains the default.

### Cause and retained fix

The equal-navigation control ran first. It replaced only local velocity
selection with a separate public-API reference process while retaining crowd
steering, frozen-state ordering, acceleration interpolation, strict-speed
clamping, overlap correction and corridor projection. A separate build profile
and an embedded protocol marker establish the executable's identity. All six
scenes completed collection successfully. Its pipe overhead is excluded from
planner-cost evidence; affinity and scheduling changes are retained with that
diagnostic's execution receipt.

Circle-512 maximum penetration is `0.134895742 m` for the standalone reference,
`0.155890822 m` for the reference under shared crowd integration, and
`0.171704173 m` for the pre-fix candidate. Shared integration contributes to the
gap, but does not explain the additional `0.015813351 m` candidate depth. The
control is diagnostic and does not replace the frozen standalone limit of
`0.144895742 m`. The flat circle uses requested velocities, so doorway funnel
policy is not an explanation for its discrepancy.

A hard-bound defect remained in the final velocity clamp: exact products of f32
coordinates fit in f64, but their sum can round to the squared speed limit even
when the exact vector lies outside. For example, `(2, 2^-26)` is outside radius 2,
although the rounded f64 sum is 4. The former comparison then skipped its radial
margin. The retained fix recovers the addition residual and clamps a positive
residual at equality as well as an above-limit sum. Subnormal speed limits use
an absolute representable step of radial margin. Focused regressions cover the
hidden orthogonal component, exact axial boundary and subnormal limits.

This changes only the ORCA speed clamp. No acceleration, corridor, neighbour or
position-correction policy change is retained. Under the same fixture and source
build, circle-512 now reaches maximum penetration `0.127451062 m` and arrival
p100 `190.80 s`, passing `0.144895742 m` and `199.08 s` respectively. Small
boundary changes alter the interacting trajectory; this is measured fixture
acceptance, not a general multi-agent separation guarantee. The equal-navigation
control used the earlier clamp and recorded two strictly positive speed counts
whose excess rounds to zero at twelve decimal places. These are retained as
failures of that diagnostic clamp, not attributed to the final candidate.

The relative evaluator also now requires candidate zero deadlocks even when the
reference deadlocks. Three synthetic cases verify zero/zero acceptance and
rejection of candidate-only and shared deadlocks. This strengthens the existing
zero-deadlock bar.

### Identical navigation and integration control

All rows below use reference velocity selection inside the crowd pipeline.
The raw collector's generic mode label is not an executable-identity claim.

| Scene | Maximum penetration, m | Positive-depth p99, m | Arrival p100, s | Unreached | Stationary deadlocks |
|---|---:|---:|---:|---:|---:|
| circle 64 | 0.034270525 | 0.016439557 | 52.90 | 0 | 0 |
| circle 512 | 0.155890822 | 0.034852326 | 199.05 | 0 | 0 |
| crossing 128 | 0.011043131 | 0.004558265 | 103.80 | 0 | 0 |
| corridor 64 | 0.040272295 | 0.026745856 | unmeasured | 1 | 0 |
| doorway 64 | 0.015406430 | 0.005212128 | unmeasured | 32 | 0 |
| dense 2000 | 0.000000000 | 0.000000000 | 50.40 | 0 | 0 |

### Final native reference and candidate

The original inputs, horizons, caps, speed, step, metric definitions and timeouts
are unchanged. All six candidate scenes have zero strict speed excess and zero
stationary deadlocks. Standalone doorway reference steering still differs; the
control above separately measures the identical crowd steering policy.

| Scene | Reference / candidate maximum penetration, m | Reference / candidate positive-depth p99, m | Reference / candidate arrival p100, s | Reference / candidate unreached |
|---|---:|---:|---:|---:|
| circle 64 | 0.068525612 / 0.034274042 | 0.025568247 / 0.016960919 | 60.50 / 49.35 | 0 / 0 |
| circle 512 | 0.134895742 / 0.127451062 | 0.036157608 / 0.040486455 | 189.60 / 190.80 | 0 / 0 |
| crossing 128 | 0.035444796 / 0.014600396 | 0.008958638 / 0.004372895 | 101.90 / 102.90 | 0 / 0 |
| corridor 64 | 0.060882211 / 0.040270925 | 0.048327625 / 0.027207255 | unmeasured / unmeasured | 1 / 1 |
| doorway 64 | 0.031862676 / 0.021192491 | 0.011198580 / 0.005482495 | unmeasured / unmeasured | 24 / 28 |
| dense 2000 | 0.000000000 / 0.000000000 | 0.000000000 / 0.000000000 | 50.40 / 50.40 | 0 / 0 |

### All 61 fallback residuals explained

The original frozen comparison still reports 202 random velocity mismatches at
`1e-4 m/s`, and zero in the four frozen scenarios. The priority classes remain
108 reference speed contradictions, 33 demonstrated extended-precision-sensitive
fallbacks, and 61 bounded fallback residuals. No case is double-counted.

For this diagnosis the reference implementation source was inspected as test
oracle data. No reference implementation was copied or translated into the
crate. An instrumented reference reproduces every original residual output
exactly. Its projected-line construction stores a distant line intersection in
f32 and later evaluates its offset with a rounded projected direction. That
representation amplifies cancellation. Measured projected-offset errors across
the 61 cases range from approximately `6.04e-8` to `1.50e-4 m/s`; small offset
errors can yield much larger velocity differences in these conditioned programs.

An isolated oracle-only intervention retains the original projected direction,
parallel cutoff, state and constraints, but represents the same projected line
near the origin using `(n_j-n_i).v = c_j-c_i`. All 61 outputs then agree with the
candidate within `0.000004540184 m/s`. Tightening the parallel cutoff as a separate
probe resolves no additional cases. Eight exact coordinate symmetries resolve
none and change none beyond tolerance; no coordinate-order exemption is inferred.
Each case retains its input hash, original and intervened outputs, projection
trace, and source/executable hashes. This establishes projection-representation
sensitivity for all 61, beyond the earlier optimality-only classification.

The candidate already uses stable distance equations and remains independently
certified within `0.000000731 m/s` of the minimax objective. Reproducing the
original reference's inaccurate fallback would discard that evidence. The
unmodified reference still differs by up to `1.550569510 m/s` in this class;
the diagnostic intervention is an explanation, not a replacement qualification
oracle or a relaxed velocity threshold. The original frozen and classification
acceptance commands therefore retain exit 1.

### Rejected integration probes

Both probes use the final clamp and retain sampled behavior in their scratch
sources. Neither is adopted.

| Probe | Circle-512 maximum penetration, m | Circle-512 arrival p100, s | Additional failure |
|---|---:|---:|---|
| Directly assign reachable ORCA velocity | 0.152106047 | 234.00 | One stationary crossing deadlock; circle penetration and arrival fail. |
| Up to twelve ORCA position-correction iterations, with early convergence | 0.127242744 | 223.30 | Circle arrival fails; circle-64 arrival also fails its relative deadline. |

### Final cost and gates

The unchanged timing method uses ten warmup ticks and nine alternating pairs of
ten ticks on one allowed CPU. Ratios are from the same shared-host run and include
preferred-velocity preparation and crowd update.

| Agents | Sampled, microseconds/agent-tick | Candidate, microseconds/agent-tick | Ratio |
|---:|---:|---:|---:|
| 100 | 2.518288 | 2.490980 | 0.989156 |
| 1000 | 2.500716 | 3.035481 | 1.213845 |
| 5000 | 2.248246 | 2.987633 | 1.328873 |

All 24 repeated final-source implementation and replay checks exit 0: formatting;
four Clippy configurations with warnings denied; default, parallel and 32-bit
tests; no_std and tile-cache variants; rustdoc; WebAssembly; MSRV; all fuzz-target
builds; the unchanged public-text guard; sampled and ORCA native/WASI determinism;
and both corpus budget cases. No fuzz campaign ran.

The 3,116 sampled and 27 corrected sampled records remain byte-identical. Only the
ORCA circle-512 digest changes, from `3159a1e880a76949` to `5a610dca209a2597`.
The explicit comparison with the prior ORCA golden exits 1; replacement native
and WASI comparisons exit 0. Native source compilation precedes measurement,
and retained measured and final replay executable identities are checked.

| Receipt | Exit code |
|---|---:|
| Six-scene identical-navigation control collection | 0 |
| Numerical intervention and deadlock-gate regression runner | 0 |
| All 202 mismatch explanations validated against fresh inputs and outputs | 0 |
| Final candidate/reference metric collection and cost collection | 0 |
| All 24 final-source implementation/replay checks | 0 |
| Original frozen-reference matching | 1 |
| Original mismatch-classification acceptance | 1 |
| Full relative acceptance, including missing arrival tails | 1 |
| Explicit prior ORCA-golden comparison | 1 |

Decision: retain the minimal hard-bound fix, the stricter deadlock evaluator and
only the changed ORCA digest. The circle-512 physical gap and the 61 explanatory
gaps are closed. **Full integration qualification remains rejected** because
corridor and doorway arrival p100 are still unmeasured at the frozen timeouts;
the original strict reference-compatibility commands also remain nonpassing.
Do not promote those missing tails or diagnostic oracle interventions to passes.
Stop with the bounded result and all counterexamples retained. No Git integration,
index/branch changes, commits or fuzz campaigns were performed.
