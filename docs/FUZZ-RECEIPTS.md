# Fuzz receipts

Each receipt records one run of a fuzz target under the release rule in
[`fuzz/README.md`](../fuzz/README.md): start from seeds, run until coverage has been flat for 10
minutes, for at least 10 and at most 120 CPU-minutes, with no crash.

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
