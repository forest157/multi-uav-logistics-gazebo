import unittest
from xml.etree import ElementTree

from logistics_gazebo_sim.scenes import SCENES, metric_xy
from logistics_gazebo_sim.worlds import render_world


class WorldTest(unittest.TestCase):
    def test_all_scenes_are_valid_classic_sdf(self):
        for scene_id in SCENES:
            root = ElementTree.fromstring(render_world(scene_id))
            self.assertEqual(root.attrib["version"], "1.6")
            self.assertIsNotNone(root.find("world/physics"))

    def test_reference_centre_maps_to_enu_origin(self):
        self.assertEqual(metric_xy((250, 250)), (0.0, 0.0))

    def test_all_obstacles_have_physical_height(self):
        for scene in SCENES.values():
            for obstacle in scene["obstacles"]:
                self.assertGreater(obstacle["height"], 0.0)
                self.assertLessEqual(obstacle["height"], 35.0)

    def test_zone_markers_are_visual_decals_without_collision(self):
        root = ElementTree.fromstring(render_world(0))
        models = {model.attrib["name"]: model for model in root.findall("world/model")}
        for name in ("start_zone", "goal_zone"):
            marker = models[name]
            self.assertLess(float(marker.find("pose").text.split()[2]), 0.0)
            self.assertIsNone(marker.find("link/collision"))
            self.assertIsNotNone(marker.find("link/visual"))


if __name__ == "__main__":
    unittest.main()
