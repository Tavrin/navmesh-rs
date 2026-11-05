#!/usr/bin/env python3
"""Classify retained differential failures using public half-planes and dual bounds.

For weights w >= 0 summing to one, every speed-bounded velocity has worst
violation >= sum(w*c) - speed*norm(sum(w*n)). This is an independent lower
certificate, not a second implementation of the reference linear program.
"""
import argparse
import itertools
import json
import math
import subprocess


def cross(a, b):
    return a[0] * b[1] - a[1] * b[0]


def certificate(planes, speed):
    """Lower bound on minimum nonnegative worst normalized-plane violation."""
    lower = 0.0
    for x, y, c in planes:
        lower = max(lower, c - speed * math.hypot(x, y))
    for a, b in itertools.combinations(planes, 2):
        def dual(t):
            return (t * a[2] + (1-t) * b[2]
                    - speed * math.hypot(t*a[0] + (1-t)*b[0],
                                         t*a[1] + (1-t)*b[1]))
        lo, hi = 0.0, 1.0
        for _ in range(80):
            left, right = (2*lo+hi)/3, (lo+2*hi)/3
            if dual(left) < dual(right):
                lo = left
            else:
                hi = right
        lower = max(lower, dual((lo+hi)/2))
    for a, b, c in itertools.combinations(planes, 3):
        weights = [cross(b, c), cross(c, a), cross(a, b)]
        total = sum(weights)
        if abs(total) < 1e-15:
            continue
        weights = [w / total for w in weights]
        if min(weights) >= 0:
            # Retain the residual normal in the bound even after solving for
            # zero; this makes floating-point residuals conservative.
            nx = sum(w*p[0] for w, p in zip(weights, (a, b, c)))
            ny = sum(w*p[1] for w, p in zip(weights, (a, b, c)))
            lower = max(lower, sum(w*p[2] for w, p in zip(weights, (a, b, c)))
                        - speed * math.hypot(nx, ny))
    return lower


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--prefix', required=True)
    parser.add_argument('--details', required=True)
    parser.add_argument('--precision-reference')
    parser.add_argument('--output', required=True)
    args = parser.parse_args()
    with open(args.prefix + '.failures.json', encoding='utf8') as source:
        cases = json.load(source)
    with open(args.prefix + '.scenario-failures.json', encoding='utf8') as source:
        cases += json.load(source)
    run = subprocess.run([args.details], input=''.join(c['input'] for c in cases),
                         capture_output=True, text=True, check=True)
    lines = iter(run.stdout.splitlines())
    precision = None
    if args.precision_reference:
        rerun = subprocess.run([args.precision_reference],
                               input=''.join(c['input'] for c in cases),
                               capture_output=True, text=True, check=True)
        precision = iter([tuple(map(float, line.split()))
                          for line in rerun.stdout.splitlines()])
    rows = []
    for case in cases:
        source = case['input'].splitlines()
        n = int(source[0].split()[0])
        agents = [list(map(float, s.split())) for s in source[1:n+1]]
        for i in range(n):
            values = list(map(float, next(lines).split()))
            extended = next(precision) if precision is not None else None
            actual, expected = case['actual'][i], case['expected'][i]
            error = math.dist(actual, expected)
            if error <= 1e-4:
                continue
            count, neighbours = map(int, values[2:4])
            data = values[4:]
            if len(data) != 4*count:
                raise ValueError('invalid public plane protocol')
            planes = []
            for j in range(count):
                dx, dy, px, py = data[4*j:4*j+4]
                length = math.hypot(dx, dy)
                nx, ny = -dy/length, dx/length
                planes.append((nx, ny, nx*px + ny*py))
            speed = agents[i][7]
            oracle_speed = math.hypot(*expected)
            row = dict(scene=case.get('scenario', 'random'),
                       case=case.get('configuration', case.get('step')), agent=i,
                       velocity_error=error, oracle_speed=oracle_speed,
                       speed_error_lower_bound=max(0, oracle_speed-speed))
            if count != neighbours:
                row['class'] = 'unresolved-static-priority'
            else:
                lower = certificate(planes, speed)
                slack = lambda v: max(0., *(c-x*v[0]-y*v[1] for x, y, c in planes))
                ours, oracle = slack(actual), slack(expected)
                row.update(dual_lower=lower, candidate_slack=ours,
                           oracle_slack=oracle, candidate_duality_gap=ours-lower,
                           oracle_duality_gap=oracle-lower)
                if extended is not None:
                    row.update(extended_error=math.dist(actual, extended),
                               precision_change=math.dist(expected, extended))
                if oracle_speed-speed > 1e-4:
                    row['class'] = 'oracle-speed-contradiction'
                elif extended is not None and math.dist(actual, extended) <= 1e-4:
                    row['class'] = 'precision-sensitive-fallback'
                elif lower > 1e-5 and ours-lower <= 1e-5:
                    # A certified candidate minimizer differs from a bounded
                    # oracle output. Do not call this a float-order exemption:
                    # establishing the arithmetic cause needs further evidence.
                    row['class'] = 'infeasible-fallback-behaviour'
                else:
                    row['class'] = 'unresolved'
            rows.append(row)
    counts = {c: sum(r['class'] == c for r in rows) for c in sorted({r['class'] for r in rows})}
    report = dict(classes=counts, failures=rows,
                  max_candidate_duality_gap=max((r.get('candidate_duality_gap', 0) for r in rows), default=0),
                  float_order_exemptions=0)
    with open(args.output, 'w', encoding='utf8') as output:
        json.dump(report, output, indent=2)
        output.write('\n')
    print(json.dumps({k: v for k, v in report.items() if k != 'failures'}, indent=2))
    # Behavioural differences are retained failures, even with an optimality
    # certificate. Classification alone never promotes velocity acceptance.
    return int(any(c != 'oracle-speed-contradiction' for c in counts))


if __name__ == '__main__':
    raise SystemExit(main())
