import unittest
from logistics_gazebo_sim.pointcloud_perception import exclude_mapped_static
from logistics_gazebo_sim.clearance_analyzer import obstacle_primitives
from logistics_gazebo_sim.scenes import SCENES


class StaticReturnTest(unittest.TestCase):
    def test_observed_wall_fragment_is_removed_but_airborne_target_remains(self):
        wall=(-26.12,-25.9,4.88);bird=(0,12,16.5)
        self.assertEqual(exclude_mapped_static([wall,bird],obstacle_primitives(SCENES[0])),[bird])

    def test_cylinder_height_and_margin_are_bounded(self):
        shape=dict(kind="cylinder",x=0,y=0,radius=1.,height=5.)
        points=[(1.3,0,2),(1.6,0,2),(0,0,6)]
        self.assertEqual(exclude_mapped_static(points,[shape]),points[1:])

    def test_no_map_does_not_filter_unknown_environment(self):
        points=[(1,2,3)]
        self.assertEqual(exclude_mapped_static(points,[]),points)
