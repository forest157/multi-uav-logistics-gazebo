import runpy
import unittest
from pathlib import Path


SCRIPT = Path(__file__).resolve().parents[1] / 'scripts' / 'benchmark_world_resources'
evaluate_budget = runpy.run_path(str(SCRIPT))['evaluate_budget']


class ResourceBudgetTest(unittest.TestCase):
    def setUp(self):
        self.report = {
            'real_time_factor': 0.8,
            'full_stack_cpu_cores': 8.0,
            'full_stack_pss_peak_sampled_mib': 12288.0,
        }
        self.budget = {
            'minimum_real_time_factor': 0.8,
            'maximum_cpu_cores': 8.0,
            'maximum_full_stack_pss_mib': 12288.0,
        }

    def test_boundary_passes(self):
        self.assertTrue(evaluate_budget(self.report, self.budget)['passed'])

    def test_each_exceeded_limit_fails(self):
        changes = {'real_time_factor': 0.799,
                   'full_stack_cpu_cores': 8.001,
                   'full_stack_pss_peak_sampled_mib': 12288.1}
        for key, value in changes.items():
            with self.subTest(metric=key):
                report = dict(self.report, **{key: value})
                self.assertFalse(evaluate_budget(report, self.budget)['passed'])

    def test_server_only_sample_is_rejected(self):
        with self.assertRaisesRegex(ValueError, 'full-stack'):
            evaluate_budget({'real_time_factor': 1.0}, self.budget)


if __name__ == '__main__':
    unittest.main()
