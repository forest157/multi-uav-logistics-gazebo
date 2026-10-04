import unittest
from xml.etree import ElementTree

from logistics_gazebo_sim.scenes import SCENES, metric_xy
from logistics_gazebo_sim.worlds import render_world


class WorldTest(unittest.TestCase):
    def test_roof_visual_is_contained_in_building_collision(self):
        for scene_id in SCENES:
            models={m.get("name"):m for m in ElementTree.fromstring(render_world(scene_id)).findall("world/model")}
            for name,roof in models.items():
                if not name.endswith("_roof"):continue
                body=models[name[:-5]]
                roof_size=list(map(float,roof.find("link/visual/geometry/box/size").text.split()))
                body_size=list(map(float,body.find("link/collision/geometry/box/size").text.split()))
                roof_center=list(map(float,roof.find("pose").text.split()))[:3]
                body_center=list(map(float,body.find("pose").text.split()))[:3]
                for axis in range(3):
                    self.assertLessEqual(abs(roof_center[axis]-body_center[axis])+roof_size[axis]/2.,body_size[axis]/2.+1e-6)

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

    def test_real_scale_context_is_visual_only_and_bounded(self):
        for scene_id in SCENES:
            root=ElementTree.fromstring(render_world(scene_id));models=root.findall("world/model")
            road=next(model for model in models if model.attrib["name"]=="service_road")
            size=[float(value) for value in road.find("link/visual/geometry/box/size").text.split()]
            self.assertEqual(size[1],7.0);self.assertGreater(size[0],50.0)
            self.assertIsNone(road.find("link/collision"));self.assertLess(len(models),100)

    def test_every_safety_obstacle_keeps_collision_and_visual(self):
        root=ElementTree.fromstring(render_world(0))
        obstacles=[model for model in root.findall("world/model") if model.attrib["name"].startswith("obstacle_") and not model.attrib["name"].endswith("_roof")]
        self.assertTrue(obstacles)
        for model in obstacles:
            self.assertIsNotNone(model.find("link/collision"));self.assertIsNotNone(model.find("link/visual"))


if __name__ == "__main__":
    unittest.main()
