import unittest
from logistics_gazebo_sim.pointcloud_perception import DetectionAssociator


class DetectionAssociationTest(unittest.TestCase):
    def test_confirms_and_keeps_stable_id(self):
        associator=DetectionAssociator(maximum_distance=2,confirmation_hits=3,maximum_misses=1)
        self.assertEqual(associator.update([{"position":[0,0,2]}]),[])
        self.assertEqual(associator.update([{"position":[0.5,0,2]}]),[])
        confirmed=associator.update([{"position":[1.0,0,2]}])
        self.assertEqual(confirmed[0]["id"],"lidar_target_0")
        self.assertEqual(associator.update([{"position":[1.5,0,2]}])[0]["id"],"lidar_target_0")

    def test_prunes_missed_track(self):
        associator=DetectionAssociator(confirmation_hits=1,maximum_misses=1)
        self.assertEqual(associator.update([{"position":[0,0,0]}])[0]["id"],"lidar_target_0")
        associator.update([]);associator.update([])
        self.assertFalse(associator.tracks)

if __name__=="__main__":unittest.main()
