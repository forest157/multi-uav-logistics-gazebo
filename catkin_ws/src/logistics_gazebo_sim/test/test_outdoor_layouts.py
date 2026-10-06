import unittest
import tempfile
from pathlib import Path
from xml.etree import ElementTree as ET
from logistics_gazebo_sim.worlds import OUTDOOR_LAYOUTS, render_outdoor_variant, render_outdoor_world
from logistics_gazebo_sim.outdoor_preflight import metadata_for_world


class OutdoorLayoutTest(unittest.TestCase):
    def test_original_markers_use_metric_coordinates(self):
        models = {m.get('name'): m for m in ET.fromstring(render_outdoor_world()).findall('world/model')}
        for name, expected in [('start_zone', [-18, -4]), ('goal_zone', [0, 42])]:
            self.assertEqual(list(map(float, models[name].findtext('pose').split()))[:2], expected)

    def test_variant_landing_clearance_and_collision_visual_agreement(self):
        for name, layout in OUTDOOR_LAYOUTS.items():
            root = ET.fromstring(render_outdoor_variant(name))
            for m in root.findall('world/model'):
                collision = m.find('link/collision/geometry/box/size')
                if collision is not None:
                    self.assertEqual(collision.text, m.findtext('link/visual/geometry/box/size'))
            for x, y in layout['pads'] + [layout['goal']]:
                self.assertLess(abs(x) + 3.5, layout['extent_m'][0] / 2)
                self.assertLess(abs(y) + 3.5, layout['extent_m'][1] / 2)
                for bx, by, w, d, height in layout['blocks']:
                    self.assertTrue(abs(x - bx) > w / 2 + 3.5 or abs(y - by) > d / 2 + 3.5)
            for bx, by, w, d, height in layout['blocks']:
                for rx, ry, rw, rd in layout['roads']:
                    self.assertTrue(abs(bx-rx) >= (w+rw)/2 or abs(by-ry) >= (d+rd)/2)

    def test_metadata_matches_every_generated_world_and_rejects_geometry_drift(self):
        with tempfile.TemporaryDirectory() as directory:
            for name, layout in OUTDOOR_LAYOUTS.items():
                path = Path(directory) / (name + '.world')
                original = render_outdoor_variant(name)
                path.write_text(original, encoding='utf-8')
                metadata = metadata_for_world(name, path)
                self.assertEqual(metadata['maximum_supported_uavs'], len(layout['pads']))
                self.assertEqual(metadata['sdf_model_count'],
                                 len(ET.fromstring(original).findall('world/model')))
                self.assertEqual(len(metadata['no_fly_volumes']), len(layout['blocks']))
                if name != 'outdoor_scale_yard':
                    self.assertGreater(metadata['recommended_cruise_altitude_m'],
                                       metadata['maximum_building_height_m'])
                else:
                    self.assertLess(metadata['recommended_cruise_altitude_m'],
                                    metadata['maximum_building_height_m'])
                self.assertEqual(metadata['dynamic_obstacle_routes'], [])
                changed = original.replace('<size>24 22 10</size>',
                                           '<size>24 22 11</size>', 1)
                if changed == original:
                    changed = original.replace('<size>18 16 6</size>',
                                               '<size>18 16 7</size>', 1)
                if changed == original:
                    changed = original.replace('<size>30 28 14</size>',
                                               '<size>30 28 15</size>', 1)
                if changed == original:
                    changed = original.replace('<size>28 120 22</size>',
                                               '<size>28 120 23</size>', 1)
                path.write_text(changed, encoding='utf-8')
                with self.assertRaises(ValueError):
                    metadata_for_world(name, path)
