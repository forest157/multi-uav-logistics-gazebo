import unittest
from xml.etree import ElementTree as ET
from logistics_gazebo_sim.worlds import OUTDOOR_LAYOUTS, render_outdoor_variant, render_outdoor_world


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
