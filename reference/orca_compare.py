#!/usr/bin/env python3
"""Frozen-state ORCA comparisons using independent public-API executables.

The reference executable is built outside the repository from RVO2 plus
harness/orca-oracle.cc. No reference implementation is imported into the crate.
"""
import argparse
import json
import math
import random
import subprocess


def encode(agents, walls=()):
    lines = [f"{len(agents)} {len(walls)} 0.05 2 1 16"]
    lines.extend(" ".join(format(x, ".9g") for x in a) for a in agents)
    lines.extend(" ".join(map(str, wall)) for wall in walls)
    return "\n".join(lines) + "\n"


def velocities(binary, text):
    run = subprocess.run([binary], input=text, text=True, capture_output=True, check=True)
    return [tuple(map(float, line.split())) for line in run.stdout.splitlines()]


def compare(port, oracle, text):
    expected = velocities(oracle, text)
    actual = velocities(port, text)
    assert len(expected) == len(actual)
    errors = [math.dist(a, b) for a, b in zip(actual, expected)]
    return {"velocities": len(errors), "max_error_m_s": max(errors, default=0),
            "over_1e_4": sum(e > 1e-4 for e in errors)}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", required=True)
    parser.add_argument("--oracle", required=True)
    parser.add_argument("--output", required=True)
    parser.add_argument("--configurations", type=int, default=100000)
    args = parser.parse_args()
    rng = random.Random(0x4F524341)
    cases = []
    for _ in range(args.configurations):
        n = rng.randrange(2, 10)
        agents = []
        for _ in range(n):
            agents.append([rng.uniform(-8, 8), rng.uniform(-8, 8),
                           rng.uniform(-2, 2), rng.uniform(-2, 2),
                           rng.uniform(-2, 2), rng.uniform(-2, 2),
                           rng.uniform(.2, .7), 2.])
        cases.append(encode(agents))
    report = {"tolerance_m_s": 1e-4, "random_configurations": args.configurations,
              "random": compare(args.port, args.oracle, "".join(cases))}
    # Retain exact failing inputs, including the first failures and worst error.
    expected = velocities(args.oracle, "".join(cases))
    actual = velocities(args.port, "".join(cases))
    cursor = 0
    failures = []
    for index, case in enumerate(cases):
        count = int(case.split()[0])
        errors = [math.dist(a, b) for a, b in zip(actual[cursor:cursor+count], expected[cursor:cursor+count])]
        if max(errors) > 1e-4:
            failures.append({"configuration": index, "input": case, "max_error": max(errors),
                             "actual": actual[cursor:cursor+count], "expected": expected[cursor:cursor+count]})
        cursor += count
    report["random_failure_classification"] = {
        "configurations": len(failures),
        "oracle_over_speed_configurations": sum(any(math.hypot(*v) > 2.00001 for v in f["expected"]) for f in failures),
    }
    with open(args.output + ".failures.json", "w", encoding="utf8") as output:
        json.dump(sorted(failures, key=lambda f: -f["max_error"]), output, indent=2)
    del cases
    scenario_failures = []
    for scenario in ["circle", "crossing", "corridor", "static"]:
        n = 64 if scenario == "circle" else 32
        agents, goals = [], []
        walls = [[-20, -3, 20, -3], [-20, 3, 20, 3]] if scenario == "corridor" else []
        if scenario == "static":
            walls = [[-2, -3, -2, 1], [2, -1, 2, 3]]
        for i in range(n):
            if scenario == "circle":
                theta = i * 2 * math.pi / n
                x, y = 20 * math.cos(theta), 20 * math.sin(theta)
                goal = [-x, -y]
            elif scenario == "crossing":
                x, y = (-15 if i % 2 else 15), (i // 2 - 8) * 1.2
                if i % 4 < 2:
                    x, y = y, x
                goal = [-x, -y]
            else:
                x, y = (-12 if i % 2 else 12) + (i // 8) * 1.2, -2 + (i // 2 % 4) * 1.2
                goal = [-x, y]
            agents.append([x, y, 0., 0., 0., 0., .4, 2.])
            goals.append(goal)
        frozen, expected = [], []
        # Feed both solvers identical oracle-generated states on every step.
        for _ in range(400):
            for a, goal in zip(agents, goals):
                delta = [goal[0] - a[0], goal[1] - a[1]]
                length = max(math.hypot(*delta), 1.)
                a[4:6] = [2 * d / length for d in delta]
            text = encode(agents, walls)
            frozen.append(text)
            vs = velocities(args.oracle, text)
            expected.extend(vs)
            for a, v in zip(agents, vs):
                a[2:4] = v
                a[0] += .05 * v[0]
                a[1] += .05 * v[1]
        actual = velocities(args.port, "".join(frozen))
        errors = [math.dist(a, b) for a, b in zip(actual, expected)]
        report[scenario] = {"steps": 400, "velocities": len(errors),
                            "max_error_m_s": max(errors), "over_1e_4": sum(e > 1e-4 for e in errors)}
        for step, case in enumerate(frozen):
            start, end = step * n, (step + 1) * n
            failing = [i for i, error in enumerate(errors[start:end]) if error > 1e-4]
            if failing:
                scenario_failures.append({"scenario": scenario, "step": step, "input": case,
                                          "failing_agents": failing,
                                          "actual": actual[start:end], "expected": expected[start:end],
                                          "max_error": max(errors[start:end])})
    with open(args.output + ".scenario-failures.json", "w", encoding="utf8") as output:
        json.dump(scenario_failures, output, indent=2)
        output.write("\n")
    with open(args.output, "w", encoding="utf8") as output:
        json.dump(report, output, indent=2)
        output.write("\n")
    print(json.dumps(report, indent=2))
    return int(any(v.get("over_1e_4", 0) for v in report.values() if isinstance(v, dict)))


if __name__ == "__main__":
    raise SystemExit(main())
