import unittest
from pathlib import Path

from logistics_gazebo_sim.pointcloud_perception import exclude_mapped_static
from logistics_gazebo_sim.static_world_map import static_primitives_for_world
from logistics_gazebo_sim.worlds import OUTDOOR_LAYOUTS


class StaticWorldMapTest(unittest.TestCase):
    def test_legacy_scene_still_resolves(self):
        self.assertTrue(static_primitives_for_world(scene_id=0))
        self.assertEqual(static_primitives_for_world(scene_id='0'),
                         static_primitives_for_world(scene_id=0))

    def test_outdoor_building_returns_are_excluded(self):
        world_dir = Path(__file__).resolve().parents[1] / 'worlds'
        for name in OUTDOOR_LAYOUTS:
            with self.subTest(world=name):
                primitives = static_primitives_for_world(
                    outdoor_world=name, world_dir=str(world_dir))
                self.assertEqual(len(primitives),
                                 len(OUTDOOR_LAYOUTS[name]['blocks']))
                for building in primitives:
                    point = (building['x'], building['y'],
                             min(5.0, building['height']))
                    self.assertEqual(exclude_mapped_static([point], primitives,
                                                           margin=1.5), [])

    def test_missing_ambiguous_or_unknown_map_fails_closed(self):
        world_dir = Path(__file__).resolve().parents[1] / 'worlds'
        for kwargs in ({}, {'scene_id': 999},
                       {'scene_id': 0, 'outdoor_world': 'outdoor_campus',
                        'world_dir': str(world_dir)},
                       {'outdoor_world': 'outdoor_campus'},
                       {'outdoor_world': 'unknown', 'world_dir': str(world_dir)}):
            with self.subTest(kwargs=kwargs), self.assertRaises(ValueError):
                static_primitives_for_world(**kwargs)


if __name__ == '__main__':
    unittest.main()
