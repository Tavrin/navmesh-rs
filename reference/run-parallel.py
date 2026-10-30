#!/usr/bin/env python3
"""Serial-driver/API/rayon identity and pinned interleaved end-to-end bake timing.

Set NAV_PORT32, NAV_PORT64, NAV_CORPUS_DIR, NAV_BENCH_CPUS; pass a fresh receipt
output directory. Run under the calling environment's resource admission policy.
"""
import json
import os
from pathlib import Path
import statistics
import subprocess
import sys

out = Path(sys.argv[1])
out.mkdir(parents=True, exist_ok=False)
root = Path(__file__).resolve().parent
cpus = os.environ["NAV_BENCH_CPUS"]
counts = [0, 1, 2, 4, 8, 16]  # 0 is the public serial API
cases = sorted(p for p in (root / "cases").glob("*.case") if "mode = tiled" in p.read_text())
assert len(cases) == 5
summary = {"cpus": cpus, "identity": [], "timings": []}

def run(port, command, case, *args):
    result = subprocess.run(["taskset", "-c", cpus, port, command, str(case), *map(str, args)],
                            check=True, text=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
    return [json.loads(line) for line in result.stdout.splitlines()]

for width in [32, 64]:
    port = os.environ[f"NAV_PORT{width}"]
    for case in cases:
        rows = run(port, "verify-parallel", case)
        assert [r["threads"] for r in rows] == counts[1:]
        assert all(r["equal"] for r in rows)
        for row in rows:
            row.update(width=width, case=case.stem)
        summary["identity"].extend(rows)
        (out / "identity.json").write_text(json.dumps(summary["identity"], indent=2) + "\n")
        print(f"identity {width} {case.stem}: all five pools exact", flush=True)
    for name in ["terrain_small_tiled", "terrain_large_tiled"]:
        case = root / "cases" / f"{name}.case"
        # One warmup per configuration, then five interleaved rounds. Parsing,
        # pool startup and warmup are outside the reported timed intervals.
        for threads in counts:
            run(port, "bench-tiles", case, "--threads", threads)
        samples = {t: [] for t in counts}
        for round_index in range(5):
            order = counts[round_index:] + counts[:round_index]
            if round_index % 2: order = order[::-1]
            for threads in order:
                rows = run(port, "bench-tiles", case, "--threads", threads)
                assert len(rows) == 1
                ms = rows[0]["ms"]
                samples[threads].append(ms)
                with (out / "samples.jsonl").open("a") as log:
                    log.write(json.dumps(dict(width=width, case=name, round=round_index, threads=threads, ms=ms)) + "\n")
        for threads, values in samples.items():
            summary["timings"].append(dict(width=width, case=name, threads=threads,
                                           samples_ms=values, median_ms=statistics.median(values)))
        (out / "summary.json").write_text(json.dumps(summary, indent=2) + "\n")
        print(f"timing {width} {name}: five interleaved samples per configuration", flush=True)
