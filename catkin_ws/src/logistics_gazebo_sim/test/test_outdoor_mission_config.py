import copy
import unittest
from pathlib import Path

import yaml

from logistics_gazebo_sim.outdoor_mission import verify_outdoor_mission_config
from logistics_gazebo_sim.outdoor_world_profile import load_outdoor_profile


class OutdoorMissionConfigTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        package = Path(__file__).resolve().parents[1]
        cls.world_dir = package / 'worlds'
        cls.world_file = cls.world_dir / 'outdoor_campus.world'
        with (package / 'config' / 'mission_outdoor_campus.yaml').open(
                encoding='utf-8') as stream:
            cls.config = yaml.safe_load(stream)

    def verify(self, config, **overrides):
        values = dict(name='outdoor_campus', world_dir=str(self.world_dir),
                      world_file=str(self.world_file), spawn_x=-45.0,
                      spawn_y=-65.0, spawn_spacing_m=15.0,
                      goal_x=60.0, goal_y=65.0, target_z=19.0)
        values.update(overrides)
        return verify_outdoor_mission_config(config, **values)

    def test_committed_campus_mission_matches_verified_world(self):
        profile = self.verify(self.config)
        self.assertEqual(profile['name'], 'outdoor_campus')

    def test_all_outdoor_missions_match_their_worlds(self):
        package = self.world_dir.parent
        for name in ('outdoor_campus', 'outdoor_residential', 'outdoor_urban'):
            with self.subTest(world=name):
                profile = load_outdoor_profile(name, str(self.world_dir))
                with (package / 'config' / ('mission_' + name + '.yaml')).open(
                        encoding='utf-8') as stream:
                    config = yaml.safe_load(stream)
                verified = verify_outdoor_mission_config(
                    config, name, str(self.world_dir), str(profile['world_path']),
                    profile['spawn_center_m'][0], profile['spawn_center_m'][1],
                    profile['spawn_spacing_m'], profile['goal_center_m'][0],
                    profile['goal_center_m'][1],
                    profile['recommended_cruise_altitude_m'])
                self.assertEqual(verified['world_sha256'], config['world_sha256'])

    def test_wrong_world_hash_or_launch_coordinates_fail(self):
        stale = copy.deepcopy(self.config)
        stale['world_sha256'] = '0' * 64
        with self.assertRaisesRegex(ValueError, 'hash'):
            self.verify(stale)
        with self.assertRaisesRegex(ValueError, 'launch coordinates'):
            self.verify(self.config, spawn_x=-40.0)
        with self.assertRaisesRegex(ValueError, 'world file'):
            self.verify(self.config,
                        world_file=str(self.world_dir / 'outdoor_urban.world'))

    def test_tampered_route_is_rejected(self):
        altered = copy.deepcopy(self.config)
        midpoint = len(altered['center_trajectory']) // 2
        altered['center_trajectory'][midpoint][1] = 1000.0
        with self.assertRaises(ValueError):
            self.verify(altered)


if __name__ == '__main__':
    unittest.main()
