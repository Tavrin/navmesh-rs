#!/usr/bin/env python3
"""Fail-closed relative acceptance over retained, matched-fixture measurements."""
import argparse
import json
import math
import re

SCENES = {('circle', 64), ('circle', 512), ('crossing', 128),
          ('corridor', 64), ('doorway', 64), ('dense', 2000)}


def records(path):
    with open(path, encoding='utf8') as source:
        rows = [json.loads(line) for line in source if line.strip()]
    result = {(r['scene'], r['agents']): r for r in rows}
    if len(result) != len(rows) or set(result) != SCENES:
        raise ValueError('missing, duplicate or unexpected scene measurements')
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    for name in ('candidate', 'reference', 'cost', 'classification', 'output'):
        parser.add_argument('--' + name, required=True)
    args = parser.parse_args()
    candidate, reference = records(args.candidate), records(args.reference)
    scenes = []
    for key in sorted(SCENES):
        ours, oracle = candidate[key], reference[key]
        checks = {}
        for metric in ('max_penetration', 'p99_positive_penetration'):
            a, b = ours[metric], oracle[metric]
            checks[metric] = math.isfinite(a) and math.isfinite(b) and a <= b + 0.01
        a, b = ours['arrival_p100'], oracle['arrival_p100']
        checks['arrival_p100'] = (a is not None and b is not None and
                                  math.isfinite(a) and math.isfinite(b) and a <= b * 1.05)
        checks['deadlocks'] = oracle['deadlocks'] != 0 or ours['deadlocks'] == 0
        checks['speed'] = (ours['strict_speed_violations'] == 0 and
                           ours['max_speed_excess'] == 0)
        scenes.append(dict(scene=key[0], agents=key[1], checks=checks))
    with open(args.cost, encoding='utf8') as source:
        costs = {int(n): float(ratio) for n, ratio in re.findall(
            r'cost (\d+) sampled_us [\d.]+ orca_us [\d.]+ ratio ([\d.]+)', source.read())}
    cost_pass = set(costs) == {100, 1000, 5000} and all(math.isfinite(v) and v <= 1.5 for v in costs.values())
    with open(args.classification, encoding='utf8') as source:
        classes = json.load(source)['classes']
    # A different bounded fallback velocity is a behavioural residual until its
    # cause has been fixed or a specific numerical bound has been accepted.
    mismatch_pass = set(classes) <= {'oracle-speed-contradiction'}
    accepted = cost_pass and mismatch_pass and all(all(s['checks'].values()) for s in scenes)
    report = dict(accepted=accepted, scenes=scenes, costs=costs,
                  cost_pass=cost_pass, mismatch_classes=classes, mismatch_pass=mismatch_pass)
    with open(args.output, 'w', encoding='utf8') as output:
        json.dump(report, output, indent=2)
        output.write('\n')
    print(json.dumps(report, indent=2))
    return int(not accepted)


if __name__ == '__main__':
    raise SystemExit(main())
