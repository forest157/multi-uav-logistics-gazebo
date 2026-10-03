import unittest
from pathlib import Path

from logistics_gazebo_sim.outdoor_route_planner import planner_inputs
from logistics_gazebo_sim.outdoor_world_profile import load_outdoor_profile
from logistics_gazebo_sim.worlds import OUTDOOR_LAYOUTS


class OutdoorRoutePlannerTest(unittest.TestCase):
    def test_committed_worlds_use_metric_bounds_and_roof_clear_band(self):
        world_dir = Path(__file__).resolve().parents[1] / 'worlds'
        for name in OUTDOOR_LAYOUTS:
            with self.subTest(world=name):
                profile = load_outdoor_profile(name, str(world_dir))
                settings = planner_inputs(profile)
                x_low, x_high, y_low, y_high = settings['xy_bounds_m']
                self.assertLess(x_low, -46.0)
                self.assertGreater(x_high, 46.0)
                self.assertLess(y_low, -46.0)
                self.assertGreater(y_high, 46.0)
                self.assertEqual(len(settings['obstacle_specs']),
                                 len(profile['obstacles']))
                self.assertGreater(settings['z_bounds_m'][0],
                                   max(box['height'] for box in profile['obstacles']))
                self.assertLessEqual(settings['z_bounds_m'][0],
                                     profile['recommended_cruise_altitude_m'])
                self.assertGreaterEqual(settings['z_bounds_m'][1],
                                        profile['recommended_cruise_altitude_m'])
                for point in (profile['spawn_center_m'], profile['goal_center_m']):
                    self.assertLessEqual(x_low, point[0])
                    self.assertGreaterEqual(x_high, point[0])
                    self.assertLessEqual(y_low, point[1])
                    self.assertGreaterEqual(y_high, point[1])

    def test_invalid_formation_and_reserve_fail_closed(self):
        world_dir = Path(__file__).resolve().parents[1] / 'worlds'
        profile = load_outdoor_profile('outdoor_campus', str(world_dir))
        for kwargs in ({'vehicle_count': 2}, {'spacing_m': float('nan')},
                       {'reserve_m': 0.1}, {'reserve_m': float('inf')}):
            with self.subTest(kwargs=kwargs), self.assertRaises(ValueError):
                planner_inputs(profile, **kwargs)


if __name__ == '__main__':
    unittest.main()
