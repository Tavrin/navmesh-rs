# Fuzz receipts

Each receipt records one run of a fuzz target under the release rule in
[`fuzz/README.md`](../fuzz/README.md): start from seeds, run in 600-second chunks reusing the
corpus, stop when a whole chunk adds no coverage, for at least 10 and at most 120 CPU-minutes,
with no crash.

## `crowd_update`, 2026-10-29

Commit under test: `fb14438` (`feat/crowd`). Toolchain: nightly, `cargo-fuzz` 0.13.2, one libFuzzer
worker, `-max_len=1024`, 600-second chunks (`-max_total_time=600`) reusing one corpus directory.

Seeds: 11 hand-built inputs (the `seeds` tool has no crowd mode): five scripted scenarios (add and
update, move target, move velocity, eight agents sent to one target, remove and reset) and six
pseudo-random 40-record inputs. Each record is the 16-byte layout the target reads (three
little-endian `f32`, agent id byte, operation byte, two unused bytes).

| Chunk | CPU seconds | Executions | Coverage start | Coverage end | Crashes |
|---|---|---|---|---|---|
| 1 | 600 | 197,146 | cov 2028 / ft 5267 (11 seeds) | cov 2153 / ft 11349 | 0 |
| 2 | 600 | 184,925 | cov 2153 / ft 11349 | cov 2153 / ft 11370 | 0 |
| Total | 1,200 | 382,071 | cov 2028 | cov 2153 | 0 |

`cov` is libFuzzer's edge count. It did not change during chunk 2 (the last edge was found at
execution 195,187 of chunk 1, about 2 seconds before its end), so it was flat for the full 600
seconds of chunk 2. The feature count (`ft`) rose by 21 in chunk 2; the rule is applied to edge
coverage. 1,200 CPU-seconds is above the 10-minute floor and far below the 2-hour cap.
Final corpus: 384 inputs, 148 KB. Peak RSS 429 MB. No artifact files were written.

## Tile cache and crowd rerun, 2026-10-29

Commit under test: `df27614` (`feat/tilecache`) plus the fuzz-only changes of the commit that adds
this receipt. Toolchain: nightly, `cargo-fuzz` 0.13.2, one libFuzzer worker per target, 600-second
chunks (`-max_total_time=600`) reusing one corpus directory per target, `-print_final_stats=1`.
Rule: stop when a whole chunk adds no edge coverage, after at least 600 CPU-seconds and at most
7,200; no crash. CPU seconds are user plus system time of the fuzzer process
(`/usr/bin/time`), so time spent waiting in the machine's job queue is not counted.

### `tilecache_update`

The target now reads `[mode][layer length u16][layer bytes][16-byte operations]` (layout in the
target source). The layer is either a raw layer (header and grids mutated directly) or an 8 x 8
grid specification turned into a valid layer with `build_layer`. It goes through
`decompress_layer`, `build_regions`, `build_contours` and `build_poly_mesh`, and through the cache's
own `add_tile`, so the cache rebuilds it on `update`. Operations add cylinder, box and oriented-box
obstacles, remove one, update once and drain the update queue; the cache is drained after the
last operation. Before this change the layer bytes and the operation stream were the same bytes,
and the cache only ever held the fixed flat layer.

Seeds: 17 files written by `seeds tilecache` (four raw layers with and without holes, each with
and without operations; one obstacle script on the fixed layer; eight grid specifications with
derived and raw connections). `-max_len=4096`.

| Chunk | CPU s | Executions | Coverage start (cov / ft) | Coverage end | Crashes |
|---|---|---|---|---|---|
| 1 | 595 | 43,222 | 2663 / 9678 (17 seeds) | 2965 / 16757 | 0 |
| 2 | 593 | 39,077 | 2965 / 16757 | 2974 / 17443 | 0 |
| 3 | 594 | 36,257 | 2974 / 17443 | 2994 / 17675 | 0 |
| 4 | 578 | 28,322 | 2994 / 17675 | 3175 / 18144 | 0 |
| 5 | 576 | 31,298 | 3175 / 18144 | 3191 / 18304 | 0 |
| 6 | 588 | 30,033 | 3191 / 18304 | 3210 / 18604 | 0 |
| 7 | 588 | 32,229 | 3210 / 18604 | 3214 / 18774 | 0 |
| 8 | 593 | 32,803 | 3214 / 18774 | 3229 / 19337 | 0 |
| 9 | 595 | 35,026 | 3229 / 19337 | 3240 / 19546 | 0 |
| 10 | 594 | 33,199 | 3240 / 19546 | 3242 / 19790 | 0 |
| 11 | 589 | 27,439 | 3242 / 19790 | 3242 / 20001 | 0 |
| Total | 6,483 | 368,905 | cov 2663 | cov 3242 | 0 |

Chunk 11 added no edge, so the run stopped there (6,483 CPU-seconds, under the 7,200 cap).
Final corpus: 2,219 inputs, 9.0 MB. Peak RSS 371 MB. No artifact files were written.

What the final corpus reaches (`cargo fuzz coverage`, region coverage of the instance used by the
target): `add_obstacle` 26/26 regions, `remove_obstacle` 14/15, `process_requests` 46/47,
`update` 42/44, `rebuild` 15/17, `add_tile` 45/51, `decompress_layer` 6/6, the layer decoder 51/60,
`build_regions`, `build_contours` and `build_poly_mesh` 100%. Over `cache.rs`, `layer.rs` and
`builder.rs` together: 85.5% of regions, 72.0% of lines.

### `crowd_update`

Seeds: 55 files written by the new `seeds crowd` mode, in the 16-byte record layout (three
little-endian `f32`, agent byte, operation byte, two tail bytes): 1 to 48 records in three
operation mixes (random, eight agents then targets and updates, add/remove churn), and for
every mix the 62, 63 and 64-record sizes (992, 1,008 and 1,024 bytes) with four tail styles (zero,
`0xff`, counter, random), plus a 63-record input with a 15-byte partial trailing record
(1,023 bytes). The 11 hand-made seeds of the first run were not kept. `-max_len=1024`.

| Chunk | CPU s | Executions | Coverage start (cov / ft) | Coverage end | Crashes |
|---|---|---|---|---|---|
| 1 | 588 | 126,577 | 2026 / 8086 (55 seeds) | 2154 / 11272 | 0 |
| 2 | 595 | 145,776 | 2154 / 11272 | 2154 / 11281 | 0 |
| Total | 1,183 | 272,353 | cov 2026 | cov 2154 | 0 |

Chunk 2 added no edge. Final corpus: 641 inputs, 2.6 MB. Peak RSS 436 MB. No artifact files were
written. The end coverage is within one edge of the first run's 2153, from different seeds.
