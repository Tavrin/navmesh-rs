# Any-angle query and traversal extraction results

Status: **NOT QUALIFIED — strict acceptance rejected** (2026-11-06).

The implementation is additive. Existing queries retain their behavior, and
`BakeConfig::links` defaults to `None`. No existing golden file has been edited.
Default-output checks passed: native and WASI determinism retained all 3,116
default and 27 corrected golden digests. Fresh parity against the clean, pinned
C++ reference matched 38,272 records per reference width: 76,544 records across
42 files for 32-bit and 64-bit references, with zero differences.

The host regression, lint, no_std, documentation, minimum-toolchain, WASI and
fuzz-build gates passed. These results qualify default preservation on the
exercised configurations, not the new features. AArch64 runtime CI was not run.
The measured counterexample below prevents satisfying all strict acceptance
requirements simultaneously while retaining existing default outputs. The
implementation remains experimental; no acceptance threshold was relaxed.

## Implemented API

- `QueryFilter::uniform_cost()` defaults to `None`; `DefaultFilter` opts in when
  every configured area has the same finite positive multiplier.
- `AnyAnglePoint`, `AnyAnglePath`, `AnyAngleSearch::{begin,resume}`, and
  `NavMeshQuery::find_any_angle_path`: root/interval search in xz, detail-surface
  heights, deterministic queue ordering, expansion budgets, bounded retained
  states, mesh ownership checks and touched-tile removal checks. Search errors
  are terminal. Allocated search storage is reported separately from results.
- `BakeConfig::links: Option<LinkExtraction>`, `LinkLimits`, `LinkKind`, and
  `ExtractedLink`: boundary sampling, drop/jump/climb clearance checks, caller
  flags/areas, generated connections installed in DNAV and exposed on `Tile` and
  `TileOutput`. Extraction runs after detail generation as a separate job stage.
  Its reach widens the source/dependency halo and participates in configuration
  hashing; the disabled configuration retains its previous hash sequence.

## Recorded verification

The initial workspace check exited 101 on six new test-call argument-order
errors, subsequently corrected. The table records the later suite; exit codes
are observations, not a substitute for the remaining qualification proofs.

| Check | Exit |
| --- | ---: |
| `format` | 0 |
| `format-fuzz` | 0 |
| `source-pin` | 0 |
| `focused` | 0 |
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
| `release` | 0 |
| `binary-pin` | 0 |
| `any-angle-nav_test_solo` | 1 |
| `any-angle-dungeon_solo` | 1 |
| `any-angle-corridors_solo` | 0 |
| `any-angle-multilevel_solo` | 0 |
| `any-angle-slopes_solo` | 0 |
| `any-angle-seams_tiled` | 1 |
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
| `parity` | 1 |
| `budget-solo` | 0 |
| `budget-tiled` | 0 |
| `fuzz-build` | 0 |
| `fuzz-links-build` | 0 |
| `determinism-wasi` | 0 |
| `wasi-tests` | 0 |
| `external-oracle-build` | 0 |
| `export-nav_test_solo` | 1 |
| `external-nav_test_solo` | 1 |
| `export-dungeon_solo` | 1 |
| `external-dungeon_solo` | 0 |
| `export-corridors_solo` | 0 |
| `external-corridors_solo` | 0 |
| `export-slopes_solo` | 0 |
| `external-slopes_solo` | 0 |
| `any-angle-wasi-nav_test_solo` | 1 |
| `any-angle-wasi-dungeon_solo` | 1 |
| `any-angle-wasi-corridors_solo` | 1 |
| `any-angle-wasi-multilevel_solo` | 1 |
| `any-angle-wasi-slopes_solo` | 1 |
| `any-angle-wasi-seams_tiled` | 1 |
| `extraction-costs` | 0 |
| `new-feature-digests` | 1 |
| `fuzz-incremental-build` | 0 |
| `partial-portal-qualification` | 101 |
| `record-results` | 0 |
| `public-text-final` | 0 |
| `public-text-verified` | 0 |
| `any-angle-wasi-nav_test_solo-corrected` | 1 |
| `any-angle-wasi-dungeon_solo-corrected` | 1 |
| `any-angle-wasi-corridors_solo-corrected` | 0 |
| `any-angle-wasi-multilevel_solo-corrected` | 0 |
| `any-angle-wasi-slopes_solo-corrected` | 0 |
| `any-angle-wasi-seams_tiled-corrected` | 1 |
| `cpp-config` | 0 |
| `cpp-build` | 0 |
| `cpp-binary-pin` | 0 |
| `cpp-records` | 0 |
| `parity-cpp` | 0 |
| `supported-digests` | 0 |
| `record-results-corrected` | 0 |
| `public-text-corrected` | 0 |
| `cpp32-build` | 0 |
| `release32` | 0 |
| `binary32-pin` | 0 |
| `cpp32-records` | 0 |
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
| `parity-cpp32` | 0 |
| `retained-binary-pins` | 0 |

Timing below is diagnostic shared-host wall time with logical CPU 2 affinity.
Each query pair is warmed and then measured four times in alternating order;
the totals include all attempted pairs, including failures. No speed threshold
or dedicated-host performance qualification is asserted.

| Native case | Validated | Failures | Maximum length ratio | Expanded | Peak search bytes | Baseline total ms | Interval total ms |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| `corridors_solo` | 32 | 0 | 1.00000000 | 397 | 313344 | 0.287 | 2.529 |
| `dungeon_solo` | 2 | 1 | 1.00000000 | 426 | 368640 | 0.064 | 2.280 |
| `multilevel_solo` | 32 | 0 | 1.00000000 | 1614 | 315392 | 0.393 | 8.468 |
| `nav_test_solo` | 0 | 1 | 0.96222180 | 111 | 288768 | 0.027 | 0.553 |
| `seams_tiled` | 13 | 1 | 1.00000000 | 167 | 276480 | 0.173 | 1.466 |
| `slopes_solo` | 16 | 0 | 1.00000000 | 204 | 288768 | 0.199 | 1.395 |

| Extraction fixture | Repeats | Default total ms | Extraction total ms | Default DNAV bytes | Extraction DNAV bytes | Links |
| --- | ---: | ---: | ---: | ---: | ---: | ---: |
| Two slabs | 16 | 5.083 | 8.944 | 828 | 1140 | 2 |

Missing metrics are unmeasured. A nonzero check remains a failure even when
its log contains some passing queries or a digest. The external receipts retain
full logs and earlier failures. Fuzz campaigns were not run.

## Qualification harness and remaining limits

Eleven independent visibility-graph queries and the analytic three-tile check
meet the exact-length tolerance. The separate external oracle agrees on 51
returned lengths (32 corridor, 16 slope and three dungeon samples), with maximum
reported absolute error zero. One dungeon candidate separately fails strict
containment; these 51 comparisons establish length agreement only.

Three complete corpus cases pass all checks for 80 query pairs. Their native
and WASI digests match, as does the extraction fixture digest. The remaining
three attempted cases fail strict raycast containment near segment endpoints
(`t = 0.99999994`, and one `0.9999998`). Identical baseline segments reproduce
the endpoint failures in two cases; that does not turn either failure into a
pass. Full corpus acceptance and new-feature determinism remain rejected.

The earlier `parity` exit 1 compared against a directory without record files;
the fresh `parity-cpp` and `parity-cpp32` checks supersede that harness failure.
The first WASI corpus invocations used unresolved relative case paths; the
`-corrected` checks use absolute inputs and reproduce the native outcomes.
Earlier receipts remain preserved. The `supported-digests` exit 0 covers only
the three passing cases plus extraction; `new-feature-digests` is not a pass.

The new correctness tests include an independent visibility-graph oracle on
small rectilinear obstacle meshes, comparison with A* plus funnel, segment
containment, partial portals, stacked disconnected floors, portal-boundary
endpoints, one-expansion resumes and mesh changes. Extraction tests cover
metadata, installed connections, serial/job equivalence, dirty/full rebakes and
thread counts 1, 4 and 16. Their execution outcomes are reported above; these focused fixtures do not establish the remaining corpus proofs.

The corpus driver adds `nav-port verify-any-angle <case>` with alternating
baseline/candidate timings on identical endpoints, segment raycasts, path-length
checks, expansion/storage counts and deterministic digests. The separate external
oracle adapter rejects overlapping projected vertices and tile portals rather
than merging their topology. Its execution outcome is reported above. This is not yet the
complete small-mesh/random-convex oracle proof required for qualification.

Known restrictions requiring review:

1. Passable off-mesh traversal encountered before settling a ground path is
   rejected conservatively; this is stricter than rejecting only necessary
   off-mesh corridors. Non-observable successors retain extra endpoint states
   instead of implementing the paper's full tautness/corner pruning.
2. Extraction checks sampled solid-span clearance and vertical endpoint travel;
   it has not established a swept-radius clearance proof or the independent
   brute-force corpus comparison. Halo landings lack local target polygon IDs
   and are kept separate instead of merging across an unknown target polygon.
   The specified minimum-height rule applies to every kind, so equal-height gap
   jumps are excluded.
3. The independent exact tests currently use rectilinear meshes. The full solo
   corpus, random convex meshes, complete external-oracle agreement, full
   new-feature native/WASI digests and corpus-wide never-longer invariant are
   unqualified. The independent brute-force extraction corpus proof is also
   deferred. Focused passing samples do not replace these missing proofs.

Fuzz target sources `any_angle_query` and `link_extract` were added. The existing
incremental-bake target can enable extraction. Build outcomes are listed above; no campaigns have been run.

## Measured blocking counterexample

`quantized_portal_never_longer_qualification` constructs three flat tiles. The
middle rectangle covers x = 10..20, z = 3..7; the outer rectangles cover z =
0..10. Endpoints are (1, 0.5) and (29, 0.5) in xz.

| Reference | xz length |
| --- | ---: |
| Independent analytic optimum over the closed walkable rectangles | 28.681541692 |
| Interval search, with portals intersected with receiving polygons | 28.681541443 |
| Detour path plus funnel | 28.676330566 |

The interval path passes the `1e-4` exact-reference check and every strict
segment raycast. The funnel uses (10, 2.980392), outside the receiving polygon,
then (20, 3): the segment between them lies below the middle rectangle. Link
quantization gives the first portal endpoints z = 6.980392 and 2.980392.

The explicit strict never-longer qualification exits **101**. An on-mesh shortest
path cannot also be no longer than this shorter off-mesh baseline. The search
keeps the receiving-polygon intersection; default Detour output is preserved.
This is a failed acceptance bar, not an accepted approximation. The fixture is
ignored by ordinary tests and explicitly executed as a separate mandatory
qualification check, so green regression tests do not imply qualification.

Reproduce this failed bar explicitly with
`cargo test --test any_angle quantized_portal_never_longer_qualification -- --ignored --exact --nocapture`.

Corpus cases stop at their first counterexample. A failure during untimed warmup
has no timed comparison. External length-oracle inputs may include candidates
that failed a separate containment check; agreement there proves only length
agreement on those samples.

## Sources

The interval model and heuristic derive from the
[IJCAI 2017 paper](https://doi.org/10.24963/ijcai.2017/70).
The separate comparison oracle uses the MIT/Apache-2.0
[`polyanya` library](https://github.com/vleue/polyanya); its source was not
translated into the implementation.
