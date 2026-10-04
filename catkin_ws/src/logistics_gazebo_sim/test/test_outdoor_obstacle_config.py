import unittest
from pathlib import Path

import yaml

from logistics_gazebo_sim.outdoor_world_profile import (check_cruise_route,
                                                        load_outdoor_profile)


class OutdoorObstacleConfigTest(unittest.TestCase):
    def test_single_bird_stays_clear_of_static_buildings(self):
        package = Path(__file__).resolve().parents[1]
        profile = load_outdoor_profile('outdoor_campus',
                                       str(package / 'worlds'))
        with (package / 'config' /
              'dynamic_obstacles_outdoor_campus_single.yaml').open(
                  encoding='utf-8') as stream:
            obstacles = yaml.safe_load(stream)['obstacles']
        self.assertEqual(len(obstacles), 1)
        bird = obstacles[0]
        report = check_cruise_route(profile,
                                    [bird['start'], bird['end']],
                                    bird['radius_m'], bird['height_m'] / 2,
                                    bird['height_m'] / 2)
        self.assertTrue(report['feasible'], report)


if __name__ == '__main__':
    unittest.main()
