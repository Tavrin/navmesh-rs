"""Regression checks for the frozen arrival-tail acceptance bars."""
import json
from pathlib import Path
import unittest

from orca_relative import arrival_checks


def row(arrival=100., unreached=0, deadlocks=0):
    return dict(arrival_p100=arrival, unreached=unreached, deadlocks=deadlocks)


class ArrivalAcceptance(unittest.TestCase):
    def test_recorded_single_run_verdict(self):
        fixture = json.loads(Path(__file__).with_name('orca-arrival-tails.json').read_text())
        candidate = {r['scene']: r for r in fixture['candidate']}
        oracle = {r['scene']: r for r in fixture['reference']}
        self.assertEqual(set(candidate), {'corridor', 'doorway'})
        self.assertEqual(set(candidate), set(oracle))
        checks = {scene: arrival_checks(candidate[scene], oracle[scene])
                  for scene in candidate}
        self.assertTrue(all(checks['corridor'].values()))
        self.assertTrue(checks['doorway']['arrival_p100'])
        self.assertTrue(checks['doorway']['deadlocks'])
        self.assertEqual(all(all(c.values()) for c in checks.values()), fixture['accepted'])

    def test_frozen_deadline(self):
        self.assertTrue(all(arrival_checks(row(105.), row()).values()))
        self.assertFalse(arrival_checks(row(105.01), row())['arrival_p100'])

    def test_censored_and_invalid_arrivals_fail(self):
        for invalid in (None, float('inf'), float('nan'), -1.):
            for ours, oracle in ((row(invalid), row()), (row(), row(invalid))):
                self.assertFalse(arrival_checks(ours, oracle)['arrival_p100'])
        for ours, oracle in ((row(unreached=1), row()), (row(), row(unreached=1))):
            self.assertFalse(arrival_checks(ours, oracle)['arrival_p100'])

    def test_censored_reference_compares_unarrived_agents(self):
        censored = row(None, unreached=9)
        self.assertTrue(arrival_checks(row(None, unreached=3), censored)['arrival_p100'])
        self.assertTrue(arrival_checks(row(None, unreached=9), censored)['arrival_p100'])
        self.assertFalse(arrival_checks(row(None, unreached=10), censored)['arrival_p100'])

    def test_deadlocks_remain_fail_closed(self):
        self.assertTrue(arrival_checks(row(), row())['deadlocks'])
        for oracle in (row(), row(deadlocks=1)):
            self.assertFalse(arrival_checks(row(deadlocks=1), oracle)['deadlocks'])


if __name__ == '__main__':
    unittest.main()
