# Any-angle query and traversal extraction results

Status: **NOT QUALIFIED — corrected containment bar still fails** (2026-11-06).

The corrected quantized-portal test passes, but the complete corpus sweep exposes
interior segment failures that the previous stop-at-first-error driver never
reached. 12 of 21 cases exit zero. Failing cases and cases with no
complete baseline pairs remain unqualified. Experimental queries and traversal
extraction remain opt-in; extraction still defaults to `None`.

## Corrected contract and harness

Never-longer compares only against a complete funnel path whose segment starts
belong to their assigned polygons and whose every segment passes raycast.
Checking the start is necessary: raycast assumes that precondition and can
otherwise return completion for a quantized corner outside its assigned polygon.
Invalid baselines are counted and excluded from the length comparison; the
candidate is still checked for containment and deterministic repetition.

A raycast segment is contained when `t >= 1 - 1e-6`. This is an endpoint tolerance,
not a spatial tolerance for interior hits. The sweep records 15 candidate queries
using it across 17 segments, and 25 baseline segments using it. Counts include
otherwise failing queries or baselines; they are not qualification counts.
There are 25 invalid baselines. All independent length comparisons retain
an absolute `1e-4` tolerance.

The driver checks at most 32 deterministic consecutive random endpoint pairs per
case (fewer when configured). Incomplete or disconnected baseline pairs are
reported as skipped, never passing paths. It now continues after counterexamples.
Off-mesh rejections remain typed `Unsupported` and visible as rejected queries;
they are not silently counted as ground-path passes.

## Corpus results

The native run uses `libm`; WASI uses the same feature and case inputs. `Q/F/S`
means qualified pairs, rejected pairs (including typed restrictions) and skipped
pairs. `T` counts candidate queries
using the endpoint tolerance. A matching digest from a failed case proves only
repeatability of its accepted subset, not complete qualification.

| Case | Native Q/F/S | Invalid baseline | T | Native exit | WASI Q/F/S | WASI exit |
| --- | --- | ---: | ---: | ---: | --- | ---: |
| `corridors_solo` | 32/0/0 | 0 | 0 | 0 | 32/0/0 | 0 |
| `degenerate_solo` | 15/1/0 | 2 | 0 | 1 | 15/1/0 | 1 |
| `dungeon_solo` | 14/10/8 | 4 | 1 | 1 | 14/10/8 | 1 |
| `dungeon_tilecache` | 8/12/12 | 5 | 0 | 1 | 8/12/12 | 1 |
| `dungeon_tiled` | 7/22/3 | 9 | 0 | 1 | 7/22/3 | 1 |
| `public_solo_fixture` | 12/0/20 | 0 | 2 | 0 | 12/0/20 | 0 |
| `islands_offmesh` | 3/6/23 | 0 | 0 | 1 | 3/6/23 | 1 |
| `multilevel_solo` | 32/0/0 | 0 | 0 | 0 | 32/0/0 | 0 |
| `multilevel_tilecache` | 20/0/4 | 0 | 1 | 0 | 20/0/4 | 0 |
| `nav_test_areas` | 3/0/29 | 0 | 0 | 0 | 3/0/29 | 0 |
| `nav_test_solo` | 8/0/24 | 0 | 3 | 0 | 8/0/24 | 0 |
| `nav_test_tiled` | 5/5/22 | 3 | 0 | 1 | 5/5/22 | 1 |
| `queued_tilecache` | 20/0/4 | 0 | 1 | 0 | 20/0/4 | 0 |
| `seams_tilecache` | 32/0/0 | 0 | 2 | 0 | 32/0/0 | 0 |
| `seams_tiled` | 32/0/0 | 0 | 1 | 0 | 32/0/0 | 0 |
| `slopes_solo` | 16/0/0 | 0 | 0 | 0 | 16/0/0 | 0 |
| `stairs_monotone` | 30/0/2 | 0 | 2 | 0 | 30/0/2 | 0 |
| `terrain_large_solo` | 0/0/32 | 0 | 0 | 1 | 0/0/32 | 1 |
| `terrain_large_tiled` | 0/0/32 | 0 | 0 | 1 | 0/0/32 | 1 |
| `terrain_small_tiled` | 19/13/0 | 2 | 0 | 1 | 19/13/0 | 1 |
| `undulating_crowd` | 32/0/0 | 0 | 2 | 0 | 32/0/0 | 0 |

## Decisive containment witness and rejected repair

In `dungeon_solo`, query 4 segment 2 runs from
`[1.9847851, 10.197294, -9.941269]` to
`[1.9847851, 10.197294, -31.241272]`. Its raycast stops at
`t = 0.92957747`, well before the allowed endpoint window. The alternative
polygon containing the departure produces the same interior stop. Other dungeon
queries reproduce this leg in both directions. Query 26 separately starts its
last leg from a polygon that yields `t = 0`; alternative departure polygons can
raycast that leg successfully. These are distinct from the old query 2 endpoint
hit at `0.99999994`, which the corrected tolerance accepts.

A result-finalization attempt that split legs at ray hits and searched adjacent
containing polygons improved only one of the ten dungeon failures. Nine remained;
that repair was removed. Collapsing root height out of the xz state identity did
not reduce the recorded expansion count in this probe and was also removed.
The final implementation retains neither attempted change. Further numeric and
corridor reconstruction work is required; the measured interior failures cannot
be admitted by the corrected endpoint rule. Qualification is rejected at this
bar, and unfinished proofs below remain unfinished.

## Exact references and typed traversal restriction

The ordinary test suite now includes the quantized-portal qualification. Its
independent analytic optimum is `28.681541692`; the interval result is
`28.681541443`. The shorter funnel, `28.676330566`, has a corner outside the middle
rectangle and is an invalid baseline. Default funnel output is preserved.

The independent f64 visibility graph passes 192 routes on 64 deterministic
jittered meshes of strictly convex, non-rectilinear quadrilaterals, plus the
existing eleven rectilinear queries. Convex segment visibility uses independent
half-plane clipping and union coverage, without runtime adjacency or raycasts.
The analytic three-tile test also remains within `1e-4`.

The independent external length oracle reports:

- `nav_test_solo`: export rejected overlapping projected vertices; no valid external length comparison.
- `dungeon_solo`: `oracle=polyanya-0.18.0 queries=24 max_absolute_error=0.0000076293945`.
- `corridors_solo`: `oracle=polyanya-0.18.0 queries=32 max_absolute_error=0`.
- `slopes_solo`: `oracle=polyanya-0.18.0 queries=16 max_absolute_error=0`.

These external comparisons include candidates that fail the separate containment
bar. Length agreement does not certify their geometry. Complete solo-corpus
external-oracle coverage remains unqualified.

Passable off-mesh neighbours encountered during expansion return
`Error::Unsupported`, even if a ground-only route exists. Encounter order is the
deterministic search order. Callers requiring ground-only queries must filter
out off-mesh polygons; this version does not compare their costs with ground
routes. Both query entry points document this conservative typed restriction.
Internal clipped portal endpoints obtain height from the nearest detail boundary;
caller endpoints retain strict validation.

## Extraction clearance and remaining extraction proof

Clearance now uses a continuous swept square of half-width `walkable_radius`,
including departure, landing and the vertical legs of raised jumps, drops and
climbs. The square encloses a cylindrical agent and can conservatively reject
corner contacts. Solid spans are expanded by the body dimensions, then intersected
against the continuous trajectory in f64; there are no half-cell temporal gaps.

An independent exhaustive plane-partition calculation agrees on 3,072 trajectories
at three radii. A stationary side-obstacle witness passes the former centre-line
check but fails the radius check. Existing slab tests verify installed metadata,
job/serial equality and dirty/full rebakes; parallel runs exercise 1, 4 and 16
threads. Native and WASI extraction digests are compared in the receipts.

**Not completed:** independent brute-force comparison of complete generated link
sets on a corpus subset. The clearance oracle is not that proof. Halo landings
still lack local target polygon IDs and remain separate. The minimum-height rule
still excludes equal-height gap jumps. No fuzz sources were edited or executed.

## Diagnostic query costs

Shared-host monotonic wall time with logical CPU 2 affinity. Each pair is warmed,
then measured four times in alternating baseline/candidate order. Means below
include every timed pair, including paths subsequently rejected for containment;
failed untimed warmups are excluded. Timing includes each query's allocations,
not qualification raycasts. Peak storage and expansion totals cover returned
paths; storage on typed failures is unmeasured. This is cost disclosure, not performance acceptance.

| Case | Timed pairs | Funnel us/query | Interval us/query | Expanded states | Peak search bytes |
| --- | ---: | ---: | ---: | ---: | ---: |
| `corridors_solo` | 32 | 2.551 | 19.795 | 397 | 313344 |
| `degenerate_solo` | 16 | 3.450 | 98.528 | 1477 | 362496 |
| `dungeon_solo` | 24 | 7.384 | 727.836 | 14813 | 663552 |
| `dungeon_tilecache` | 8 | 4.712 | 107.452 | 898 | 315392 |
| `dungeon_tiled` | 7 | 2.502 | 23.831 | 201 | 288768 |
| `public_solo_fixture` | 12 | 1.707 | 10.265 | 87 | 275456 |
| `islands_offmesh` | 3 | 1.286 | 11.181 | 3 | 262592 |
| `multilevel_solo` | 32 | 2.365 | 34.413 | 1614 | 315392 |
| `multilevel_tilecache` | 20 | 3.502 | 203.194 | 4347 | 466944 |
| `nav_test_areas` | 3 | 2.078 | 10.686 | 21 | 269312 |
| `nav_test_solo` | 8 | 2.597 | 19.257 | 195 | 288768 |
| `nav_test_tiled` | 6 | 3.463 | 179.284 | 1280 | 475136 |
| `queued_tilecache` | 20 | 3.488 | 200.715 | 4347 | 466944 |
| `seams_tilecache` | 32 | 2.161 | 12.022 | 365 | 290816 |
| `seams_tiled` | 32 | 2.171 | 13.357 | 345 | 288768 |
| `slopes_solo` | 16 | 2.245 | 12.764 | 204 | 288768 |
| `stairs_monotone` | 30 | 2.869 | 96.380 | 4768 | 364544 |
| `terrain_large_solo` | 0 | unmeasured | unmeasured | 0 | 0 |
| `terrain_large_tiled` | 0 | unmeasured | unmeasured | 0 | 0 |
| `terrain_small_tiled` | 20 | 17.928 | 770.982 | 12960 | 1064960 |
| `undulating_crowd` | 32 | 4.267 | 56.611 | 1660 | 376832 |

IBS outcome (full receipt retained externally):

The initial IBS attempt used an unsupported per-thread mode. The system-wide
retry was denied by performance-monitoring permissions. No symbol report was
produced: function-level cost attribution remains unmeasured, and wall-time
totals do not establish where it goes.

Extraction cost receipt:

```text
test extraction_costs ... fixture=two_slabs repeats=16 default_ns=2976595 extraction_ns=5276417 default_dnav_bytes=828 extraction_dnav_bytes=1140 links=2
```

## Exit codes and preservation

| Check | Exit |
| --- | ---: |
| `fmt` | 0 |
| `clippy-default` | 0 |
| `clippy-parallel` | 0 |
| `clippy-32` | 0 |
| `clippy-all` | 0 |
| `tests-default` | 0 |
| `tests-parallel` | 0 |
| `tests-parallel-32` | 0 |
| `tests-32` | 0 |
| `no-std` | 0 |
| `tilecache-std` | 0 |
| `tilecache-no-std` | 0 |
| `rustdoc` | 0 |
| `wasm` | 0 |
| `msrv` | 0 |
| `public-text` | 0 |
| `determinism` | 0 |
| `budgets-solo` | 0 |
| `budgets-tiled` | 0 |
| `determinism-wasi` | 0 |
| `release` | 0 |
| `export-nav_test_solo` | 1 |
| `external-nav_test_solo` | 1 |
| `export-dungeon_solo` | 1 |
| `external-dungeon_solo` | 0 |
| `export-corridors_solo` | 0 |
| `external-corridors_solo` | 0 |
| `export-slopes_solo` | 0 |
| `external-slopes_solo` | 0 |
| `extraction-costs` | 0 |
| `wasi-focused` | 0 |
| `new-feature-digests` | 1 |
| `ibs` | 255 |
| `parity-release` | 0 |
| `records-corridors_solo` | 0 |
| `records-degenerate_solo` | 0 |
| `records-dungeon_solo` | 0 |
| `records-dungeon_tilecache` | 0 |
| `records-dungeon_tiled` | 0 |
| `records-public_solo_fixture` | 0 |
| `records-islands_offmesh` | 0 |
| `records-multilevel_solo` | 0 |
| `records-multilevel_tilecache` | 0 |
| `records-nav_test_areas` | 0 |
| `records-nav_test_solo` | 0 |
| `records-nav_test_tiled` | 0 |
| `records-queued_tilecache` | 0 |
| `records-seams_tilecache` | 0 |
| `records-seams_tiled` | 0 |
| `records-slopes_solo` | 0 |
| `records-stairs_monotone` | 0 |
| `records-terrain_large_solo` | 0 |
| `records-terrain_large_tiled` | 0 |
| `records-terrain_small_tiled` | 0 |
| `records-undulating_crowd` | 0 |
| `parity` | 0 |
| `parity32-release` | 0 |
| `records32-corridors_solo` | 0 |
| `records32-degenerate_solo` | 0 |
| `records32-dungeon_solo` | 0 |
| `records32-dungeon_tilecache` | 0 |
| `records32-dungeon_tiled` | 0 |
| `records32-public_solo_fixture` | 0 |
| `records32-islands_offmesh` | 0 |
| `records32-multilevel_solo` | 0 |
| `records32-multilevel_tilecache` | 0 |
| `records32-nav_test_areas` | 0 |
| `records32-nav_test_solo` | 0 |
| `records32-nav_test_tiled` | 0 |
| `records32-queued_tilecache` | 0 |
| `records32-seams_tilecache` | 0 |
| `records32-seams_tiled` | 0 |
| `records32-slopes_solo` | 0 |
| `records32-stairs_monotone` | 0 |
| `records32-terrain_large_solo` | 0 |
| `records32-terrain_large_tiled` | 0 |
| `records32-terrain_small_tiled` | 0 |
| `records32-undulating_crowd` | 0 |
| `parity32` | 0 |
| `public-text-final` | 0 |
| `fmt-final` | 0 |
| `diff-check` | 0 |
| `fuzz-unchanged` | 0 |
| `unchanged-contracts` | 0 |
| `corpus-unchanged` | 0 |
| `ibs-system` | 255 |
| `public-text-delivery` | 0 |

Default golden files and existing query behavior were not changed. The native
and WASI determinism receipts test the 3,116 default and 27 corrected records.
Fresh native parity matches the retained pinned references in both widths:
76,544 records across 42 files, zero differences. AArch64 runtime execution was
not performed. A passing regression or
matching partial digest does not override the failed new-feature containment bar.

The full earlier report, failed attempts, exact commands, source and binary pins,
case logs and this report's receipts are retained externally. No Git state was
mutated. The temporary build target is removed after verification.
