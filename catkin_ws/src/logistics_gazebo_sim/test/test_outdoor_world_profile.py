import json
import tempfile
import unittest
from pathlib import Path

from logistics_gazebo_sim.outdoor_preflight import metadata_for_world
from logistics_gazebo_sim.outdoor_world_profile import (check_cruise_route,
                                                        load_outdoor_profile)
from logistics_gazebo_sim.worlds import OUTDOOR_LAYOUTS, render_outdoor_variant


class OutdoorWorldProfileTest(unittest.TestCase):
    def test_committed_worlds_match_profiles(self):
        world_dir = Path(__file__).resolve().parents[1] / 'worlds'
        for name in OUTDOOR_LAYOUTS:
            profile = load_outdoor_profile(name, str(world_dir))
            self.assertEqual(profile['name'], name)

    def make_world(self, directory, name):
        root = Path(directory)
        world = root / (name + '.world')
        world.write_text(render_outdoor_variant(name), encoding='utf-8')
        metadata = dict(OUTDOOR_LAYOUTS[name], **metadata_for_world(name, world))
        (root / (name + '.json')).write_text(json.dumps(metadata), encoding='utf-8')
        return load_outdoor_profile(name, directory)

    def check(self, profile, path):
        return check_cruise_route(profile, path, 4.5, 0.6, 0.6)

    def test_all_generated_worlds_have_a_clear_high_corridor(self):
        with tempfile.TemporaryDirectory() as directory:
            for name in OUTDOOR_LAYOUTS:
                profile = self.make_world(directory, name)
                z = profile['recommended_cruise_altitude_m']
                start = profile['spawn_center_m']
                goal = profile['goal_center_m']
                report = self.check(profile, [[*start, z], [*goal, z]])
                self.assertTrue(report['feasible'], (name, report))
                self.assertGreater(report['sample_count'], 1)
                self.assertEqual(len(profile['obstacles']),
                                 len(OUTDOOR_LAYOUTS[name]['blocks']))

    def test_world_or_metadata_drift_is_rejected(self):
        with tempfile.TemporaryDirectory() as directory:
            self.make_world(directory, 'outdoor_campus')
            metadata_path = Path(directory) / 'outdoor_campus.json'
            saved = json.loads(metadata_path.read_text(encoding='utf-8'))
            saved['world_sha256'] = '0' * 64
            metadata_path.write_text(json.dumps(saved), encoding='utf-8')
            with self.assertRaises(ValueError):
                load_outdoor_profile('outdoor_campus', directory)
            self.make_world(directory, 'outdoor_campus')
            world_path = Path(directory) / 'outdoor_campus.world'
            world_path.write_text(world_path.read_text(encoding='utf-8').replace(
                '</world>', '<model name="unmodelled"><link name="link">'
                '<collision name="collision"><geometry><box><size>1 1 1</size>'
                '</box></geometry></collision></link></model></world>', 1), encoding='utf-8')
            metadata = dict(OUTDOOR_LAYOUTS['outdoor_campus'],
                            **metadata_for_world('outdoor_campus', world_path))
            metadata_path.write_text(json.dumps(metadata), encoding='utf-8')
            with self.assertRaises(ValueError):
                load_outdoor_profile('outdoor_campus', directory)
            self.make_world(directory, 'outdoor_campus')
            world_path = Path(directory) / 'outdoor_campus.world'
            world_path.write_text(world_path.read_text(encoding='utf-8').replace(
                '<size>24 22 10</size>', '<size>25 22 10</size>', 1), encoding='utf-8')
            with self.assertRaises(ValueError):
                load_outdoor_profile('outdoor_campus', directory)

    def test_swept_route_rejects_buildings_bounds_and_altitude(self):
        with tempfile.TemporaryDirectory() as directory:
            profile = self.make_world(directory, 'outdoor_campus')
            self.assertEqual(self.check(profile, [[-55, -38, 8]])['error_code'],
                             'E_STATIC_CLEARANCE')
            self.assertEqual(self.check(profile, [[89, 0, 22]])['error_code'],
                             'E_BOUNDARY')
            self.assertEqual(self.check(profile, [[0, 0, 2]])['error_code'],
                             'E_VERTICAL_CLEARANCE')
            with self.assertRaises(ValueError):
                self.check(profile, [[0, 0, float('nan')]])

    def test_sampling_reserve_catches_obstacle_between_samples(self):
        profile = {'xy_half_extent_m': [10, 10], 'obstacles': [
            {'kind': 'box', 'label': 'thin', 'x': 0, 'y': 0,
             'half_x': 0.05, 'half_y': 0.05, 'height': 5}]}
        report = check_cruise_route(profile, [[-1, 0, 5], [1, 0, 5]],
                                    0.1, 0.1, 0.1, sample_step_m=2)
        self.assertEqual(report['error_code'], 'E_STATIC_CLEARANCE')


if __name__ == '__main__':
    unittest.main()
