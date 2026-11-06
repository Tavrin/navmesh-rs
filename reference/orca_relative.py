#!/usr/bin/env python3
"""Fail-closed relative acceptance over retained, matched-fixture measurements."""
import argparse
import json
import math
import re

SCENES = {('circle', 64), ('circle', 512), ('crossing', 128),
          ('corridor', 64), ('doorway', 64), ('dense', 2000)}
TAIL_SCENES = {('corridor', 64), ('doorway', 64)}


def records(path, scenes=SCENES):
    with open(path, encoding='utf8') as source:
        rows = [json.loads(line) for line in source if line.strip()]
    result = {(r['scene'], r['agents']): r for r in rows}
    if len(result) != len(rows) or set(result) != scenes:
        raise ValueError('missing, duplicate or unexpected scene measurements')
    return result


def arrival_checks(ours, oracle):
    a, b = ours['arrival_p100'], oracle['arrival_p100']
    # When the reference itself does not finish by the common ceiling there is no finite p100 to
    # compare, so the candidate must leave no more agents unarrived than the reference.
    if b is None and oracle['unreached'] > 0:
        arrival = ours['unreached'] <= oracle['unreached']
    else:
        arrival = (a is not None and b is not None and
                   math.isfinite(a) and math.isfinite(b) and
                   a >= 0 and b >= 0 and a <= b * 1.05 and
                   ours['unreached'] == 0 and oracle['unreached'] == 0)
    return dict(
        arrival_p100=arrival,
        deadlocks=ours['deadlocks'] == 0,
    )


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    for name in ('candidate', 'reference', 'output'):
        parser.add_argument('--' + name, required=True)
    for name in ('cost', 'classification'):
        parser.add_argument('--' + name)
    parser.add_argument('--tails-only', action='store_true',
                        help='evaluate only the two arrival and deadlock bars')
    args = parser.parse_args()
    if not args.tails_only and (not args.cost or not args.classification):
        parser.error('--cost and --classification are required for full acceptance')
    selected = TAIL_SCENES if args.tails_only else SCENES
    candidate = records(args.candidate, selected)
    reference = records(args.reference, selected)
    scenes = []
    for key in sorted(selected):
        ours, oracle = candidate[key], reference[key]
        checks = arrival_checks(ours, oracle)
        if args.tails_only:
            scenes.append(dict(scene=key[0], agents=key[1], checks=checks))
            continue
        for metric in ('max_penetration', 'p99_positive_penetration'):
            a, b = ours[metric], oracle[metric]
            checks[metric] = math.isfinite(a) and math.isfinite(b) and a <= b + 0.01
        checks['speed'] = (ours['strict_speed_violations'] == 0 and
                           ours['max_speed_excess'] == 0)
        scenes.append(dict(scene=key[0], agents=key[1], checks=checks))
    if args.tails_only:
        report = dict(accepted=all(all(s['checks'].values()) for s in scenes),
                      scope='corridor and doorway arrival tails', scenes=scenes)
        with open(args.output, 'w', encoding='utf8') as output:
            json.dump(report, output, indent=2)
            output.write('\n')
        print(json.dumps(report, indent=2))
        return int(not report['accepted'])
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
