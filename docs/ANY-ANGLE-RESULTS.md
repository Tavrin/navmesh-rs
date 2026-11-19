# Any-angle query and traversal extraction results

Status: **PASS for the tested uniform-cost workload** (2026-11-08).

All 21 corpus cases pass natively and on WASI: 624 validated pairs per platform,
zero rejected or skipped tested pairs, and matching path digests in every case.
The maximum length ratio against a valid complete funnel is 1.00000000.
Experimental queries and extraction remain opt-in; extraction defaults to `None`.
This result does not qualify arbitrary meshes or the original incomplete
long-range pairs described below.

## Containment cause and correction

The earlier interior raycast failures were not all geometric escapes. In the
retained dungeon witness, the segment runs from
`[1.9847851, 10.197294, -9.941269]` to
`[1.9847851, 10.197294, -31.241272]`. The raycast stops at `t = 0.92957747`
where a wall touches the segment at the shared vertex
`[1.9847851, -29.741272]` in xz. Portal-connected polygons cover the continuation.
The same ambiguity occurs along shared edges and at a segment departure vertex.
The former endpoint-only raycast rule could not distinguish these from escapes.

Qualification now independently clips the segment against closed convex polygon
half-planes, follows only portals touched by that segment, and checks the union
of reachable parameter intervals for gaps. A disconnected projected polygon
cannot supply coverage. Regression fixtures accept shared-edge and shared-vertex
continuations and reject missing connectivity, an outside start, and a real gap.
The ordinary raycast is preserved and its early stops remain reported.

Tile border copies can differ by floating-point rounding: the retained tiled
witness has border coordinates `38.31068420410156` and `38.31068801879883`.
The checker explicitly uses spatial slack
`eps = 8 * f32::EPSILON * max(1, abs(endpoint xz coordinates))`:
it widens polygon half-planes by `eps`, permits portal endpoint rounding at
that scale, and accepts uncovered parameter gaps of at most `eps / segment_length`.
This is a declared spatial reconciliation rule, distinct from the historical
`1e-6` endpoint-parameter tolerance. Sub-scale gaps can pass; exact zero-slack
containment is not claimed. The search likewise reconciles almost-collinear
border copies and rounds newly clipped intervals inward into their constraints.
Caller endpoints retain strict validation.

The final sweep reports 32 raycast stops before the historical endpoint window
and 103 segments needing spatial reconciliation. It also reports zero-slack
coverage as `max_exact_gap`. Its maximum, 88.9115743508, is an *unreachable suffix*
after exact traversal loses a portal at a rounded seam, not the physical width
of a hole accepted by the spatial rule. The reconciled maximum gap is zero in
every case. The zero-slack dungeon witness has no gap.

## Endpoint workload and corpus results

There are 409 original complete-baseline pairs and 215 deterministic local
replacement pairs. If an original funnel is partial or exhausts its node pool,
the driver tries one replacement using a search radius of 48 from the original
start. Both implementations receive the same replacement endpoint. An incomplete
replacement is skipped and never counted as passing. No node cap was increased.
Both large-terrain cases use 32 replacements: their original long-range pairs
remain unqualified. Six geometrically invalid complete funnel baselines are
excluded from length comparison; their candidates still pass independent
containment and repeatability checks. Off-mesh steps are checked against their
actual linked endpoints and permitted direction; 12 such segments were tested.

Native and WASI counts and digests agree in every row. `Local` counts replacements,
`Invalid` counts excluded funnel baselines, `Early` counts interior raycast stops,
and `Spatial` counts candidate segments requiring spatial reconciliation.

| Case | Passed per platform | Local | Invalid | Early | Spatial |
| --- | ---: | ---: | ---: | ---: | ---: |
| `corridors_solo` | 32 | 0 | 0 | 0 | 0 |
| `degenerate_solo` | 16 | 0 | 0 | 1 | 0 |
| `dungeon_solo` | 32 | 8 | 0 | 11 | 0 |
| `dungeon_tilecache` | 32 | 12 | 2 | 6 | 19 |
| `dungeon_tiled` | 32 | 3 | 0 | 8 | 50 |
| `public_solo_fixture` | 32 | 20 | 0 | 1 | 0 |
| `islands_offmesh` | 32 | 23 | 0 | 0 | 0 |
| `multilevel_solo` | 32 | 0 | 0 | 0 | 1 |
| `multilevel_tilecache` | 24 | 4 | 0 | 0 | 0 |
| `nav_test_areas` | 32 | 29 | 0 | 0 | 0 |
| `nav_test_solo` | 32 | 24 | 0 | 0 | 1 |
| `nav_test_tiled` | 32 | 22 | 2 | 2 | 17 |
| `queued_tilecache` | 24 | 4 | 0 | 0 | 0 |
| `seams_tilecache` | 32 | 0 | 0 | 0 | 0 |
| `seams_tiled` | 32 | 0 | 0 | 0 | 0 |
| `slopes_solo` | 16 | 0 | 0 | 0 | 0 |
| `stairs_monotone` | 32 | 2 | 0 | 0 | 2 |
| `terrain_large_solo` | 32 | 32 | 0 | 1 | 0 |
| `terrain_large_tiled` | 32 | 32 | 0 | 1 | 0 |
| `terrain_small_tiled` | 32 | 0 | 2 | 1 | 13 |
| `undulating_crowd` | 32 | 0 | 0 | 0 | 0 |

Searches are warmed and repeated on identical endpoints; complete results must
compare equal, including points, polygon references and search accounting.
The quality sweeps use two unpinned workers. Their wall times are not used as
controlled performance evidence.

## Length references, traversal and extraction

The independent f64 visibility graph passes 192 routes on 64 deterministic
strictly convex quadrilateral meshes, plus eleven rectilinear routes. The
quantized-portal analytic optimum remains `28.681541692`; the interval result
is `28.681541443`. The shorter funnel `28.676330566` is excluded because its
corner lies outside the middle rectangle. Independent length references retain
an absolute `1e-4` tolerance; the valid-funnel never-longer comparison is strict.

The independent external oracle passes dungeon, corridors and slopes. Identical
mesh and query inputs reuse the retained corridors and slopes receipts; dungeon
has replacement endpoints and is checked afresh. The external adapter still
refuses overlapping projected vertices in `nav_test_solo`; full external-oracle
coverage of overlapping floors is not claimed.

Ground endpoints can now route through passable off-mesh connections at their
xz length. The departure carries the connection polygon, and the landing carries
the ground polygon. One-way and bidirectional tests verify direction, filtering,
exact expected length, one-state resume budgets and retained-state cap failures.
Non-uniform costs and off-mesh caller endpoints remain typed `Unsupported`.

Extraction retains continuous swept-square clearance enclosing the agent radius,
including vertical legs. The independent plane-partition test passes 3,072
trajectories at three radii. A new complete link-set reference exhaustively
enumerates candidates, independently checks clearance, and applies the documented
selection and run grouping. It matches the synthetic obstructed-slab scene and
three corpus meshes: stairs 29 links, multilevel 9 links, islands 0 links.
The zero-link scene is a negative case under the minimum-height rule.
Serial/job output, dirty/full rebakes, and 1/4/16-thread fixtures pass; native and
WASI extraction digests both equal `13dcb3885bf0a7d6`.

The two-slab diagnostic over 16 repeats records default/extraction totals of
7,176,151 / 11,909,987 ns, tile sizes 828 / 1,140 bytes, and two links.
Equal-height gap jumps remain excluded. Halo landings without local target
polygon IDs remain separate; the complete-set reference covers single-tile
pipelines rather than claiming a complete cross-halo installation oracle.

## Profile, costs and rejected alternative

Per-process IBS sampling succeeds. The initial small-terrain receipt attributes
51.00% to interval containment, 34.30% to insertion/dominance scans and 11.38% to
root comparisons. The final change inlines the hot predicates and rejects
non-overlapping bounds before evaluating cross products. Its comparison semantics,
state ordering, node caps and storage layout are unchanged.

A reverse-dominance index-unlinking attempt passed its exhaustive decision test
but increased the approximate IBS event count from 165,660,371,920 to
266,770,851,398 on the four-repeat small-terrain workload. It was removed.
The final count is 103,049,355,068, about 38% below the retained initial receipt,
with zero lost samples. The inlined comparisons are now attributed to insertion,
which still accounts for 96.76%: the underlying scan remains a performance limit.
System-wide IBS was denied; the successful receipts are per-process samples.

Dedicated cost runs use one logical CPU and four alternating baseline/candidate
repeats after warmup. The pre-change receipt is reused rather than recomputed.
These are shared-host snapshots, not a controlled statistical performance gate:
control timings vary materially between snapshots.

| Workload | Pairs | Funnel us/query | Interval us/query | Expanded | Peak search bytes |
| --- | ---: | ---: | ---: | ---: | ---: |
| Dungeon, including replacements | 32 | 10.435 | 597.446 | 16127 | 663552 |
| Dungeon, original complete pairs only | 24 | 10.055 | 689.352 | 13798 | 663552 |
| Small terrain | 32 | 27.086 | 166923.453 | 78869 | 6569984 |

The retained pre-change small-terrain snapshot is 278497.075 us/query versus
166923.453 now (about 40% lower); its funnel control was 36.844 us/query versus
27.086 now. Returned digests, expansion counts and peak storage agree with that
snapshot. No general speedup over the funnel or production performance acceptance
is claimed.

## Validation commands and exits

The final commands below exit zero. The initial lint failure for a test module
preceding another item was corrected by moving the module to the file end;
no lint was suppressed. An earlier WASI sweep used unavailable relative guest
paths and exited one; the corrected sweep uses mapped absolute case paths.
Those failed receipts and the rejected index experiment remain retained externally.

```sh
tools/check_public_text.sh
cargo fmt --all --check
cargo clippy --locked --all-targets -- -D warnings
cargo clippy --locked -p navmesh-rs --all-targets --no-default-features --features std,recast,detour,crowd -- -D warnings
cargo clippy --locked --workspace --all-targets --features parallel -- -D warnings
cargo clippy --locked --all-features --all-targets -- -D warnings
cargo test --locked
cargo test --locked --workspace --features parallel
cargo test --locked -p navmesh-rs --no-default-features --features std,recast,detour,parallel
cargo test --locked -p navmesh-rs --no-default-features --features std,recast,detour,crowd --tests
cargo test --locked -p navmesh-rs --lib corpus_link_sets_match_independent_reference -- --nocapture
cargo build --locked -p navmesh-rs --lib --no-default-features --features libm,recast,detour,crowd,polyref64
cargo test --locked -p navmesh-rs --no-default-features --features std,tilecache
cargo build --locked -p navmesh-rs --lib --no-default-features --features libm,tilecache
RUSTDOCFLAGS="-D warnings" cargo doc --locked -p navmesh-rs --no-deps
cargo build --locked -p navmesh-rs --lib --target wasm32-unknown-unknown
cargo +1.88 check --locked -p navmesh-rs --lib
reference/determinism.sh
NAV_TARGET=wasm32-wasip1 reference/determinism.sh
```

The determinism commands use `NAV_CORPUS_DIR` and the WASI command also uses a
mapped runner. Both match 3,116 default and 27 corrected golden digests.
Release builds in both reference widths, both corpus budget commands, all
21 native and all 21 WASI `nav-port verify-any-angle` commands, focused native/WASI
query/extraction tests, feature digest comparison, three external-oracle checks,
extraction costs, and per-process IBS recording/reporting also exit zero.
Fresh `nav-parity diff` runs match the retained pinned references in both widths:
76,544 records across 42 files, zero differences. Golden files are unchanged.
The final public-text and `git diff --check` receipts follow this report write.

## Decisions and remaining scope

| Decision | Alternative | Reversal cost |
| --- | --- | --- |
| Independent closed coverage with explicit numeric reconciliation | Change the legacy raycast | Remove the additive checker; no default-query or tile-format migration |
| Traverse passable off-mesh connections | Keep conservative neighbour rejection | Remove the experimental routing and associated contract/tests; tile data stays compatible |
| Inline comparisons and check bounds first | Reverse-dominance index cleanup | Two attributes and predicate ordering; no storage or format changes |
| Local complete-baseline supplements | Increase node caps for incomplete original pairs | Change only the driver workload; original pairs remain separately unqualified |

AArch64 runtime execution was not performed. Fuzz campaigns remain separately
coordinated and were not run; fuzz sources are unchanged. Overlapping-floor
external-oracle coverage, original incomplete long-range pairs, equal-height
extraction jumps, and full cross-halo installation proofs retain the limits
stated above. Build artifacts are removed after verification; source, binary
pins, commands, case logs and failed attempts are retained externally. No Git
state was changed.
