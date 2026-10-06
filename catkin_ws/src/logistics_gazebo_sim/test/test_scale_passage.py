import unittest
from pathlib import Path

from logistics_gazebo_sim.outdoor_world_profile import load_outdoor_profile
from logistics_gazebo_sim.scale_passage import (corridor_capacity, passage_plan,
                                                stress_passage)


WORLD_DIR = Path(__file__).resolve().parents[1] / 'worlds'


class ScalePassageTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.profile = load_outdoor_profile('outdoor_scale_yard', str(WORLD_DIR))

    def test_physical_corridor_restricts_group_to_three(self):
        self.assertEqual(corridor_capacity(self.profile), 3)

    def test_one_three_five_eight_partition_without_overlap(self):
        for count, sizes in ((1, [1]), (3, [3]), (5, [3, 2]),
                             (8, [3, 3, 2])):
            plan = passage_plan(count, self.profile)
            self.assertEqual([len(group['vehicles']) for group in plan['groups']], sizes)
            self.assertEqual([v for g in plan['groups'] for v in g['vehicles']],
                             list(range(count)))
            for first, second in zip(plan['groups'], plan['groups'][1:]):
                self.assertGreaterEqual(second['entry_s'] - first['exit_s'], 5.0)

    def test_invalid_or_blocked_geometry_fails_closed(self):
        with self.assertRaises(ValueError):
            passage_plan(51, self.profile)
        with self.assertRaises(ValueError):
            passage_plan(8, self.profile, guard_s=0)
        with self.assertRaises(ValueError):
            corridor_capacity(self.profile, altitude_m=float('nan'))
        altered = dict(self.profile, obstacles=[dict(kind='box', label='blocked',
                    x=0, y=0, half_x=100, half_y=100, height=30)])
        with self.assertRaises(ValueError):
            passage_plan(5, altered)

    def test_twenty_to_fifty_vehicle_kinematic_pressure(self):
        report = stress_passage(self.profile)
        self.assertTrue(report['passed'])
        self.assertEqual([case['vehicles'] for case in report['cases']], [20, 35, 50])
        self.assertEqual([case['groups'] for case in report['cases']], [7, 12, 17])


if __name__ == '__main__':
    unittest.main()
