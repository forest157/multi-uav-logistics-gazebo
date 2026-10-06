import os
import unittest

from logistics_gazebo_sim.outdoor_world_profile import load_outdoor_profile
from logistics_gazebo_sim.scale_flight import phase_plan


WORLD_DIR = os.path.join(os.path.dirname(__file__), '..', 'worlds')


class ScaleFlightTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.profile = load_outdoor_profile('outdoor_scale_yard', WORLD_DIR)

    def test_supported_fleets_have_full_return(self):
        for count, groups in ((1, [1]), (3, [3]), (5, [3, 2]),
                              (8, [3, 3, 2])):
            with self.subTest(count=count):
                plan = phase_plan(count, self.profile)
                self.assertEqual([len(g) for g in plan['groups']], groups)
                self.assertEqual(len(plan['phases']), 1 + 10 * len(groups))
                self.assertEqual(plan['phases'][-1]['name'], 'return_arrive')
                self.assertGreaterEqual(plan['planned_minimum_separation_m'], 2.7)

    def test_rejects_unverified_capacity(self):
        with self.assertRaises(ValueError):
            phase_plan(9, self.profile)


if __name__ == '__main__':
    unittest.main()
