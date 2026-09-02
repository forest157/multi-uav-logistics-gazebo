import unittest
from logistics_gazebo_sim.pointcloud_perception import DetectionAssociator,VoxelBackground,calibrated_detection_confidence,cluster_detection,euclidean_clusters,exclude_near_vehicles,target_sized_detections


class PointCloudPerceptionTest(unittest.TestCase):
    def test_confidence_combines_point_support_and_motion(self):
        self.assertEqual(calibrated_detection_confidence({"point_count":30},3,4),1.0)
        self.assertAlmostEqual(calibrated_detection_confidence({"point_count":15},0,4),.325)
    def test_target_size_filter_rejects_building_fragments(self):
        detections=[{"id":"bird","radius":.75,"height":.8},{"id":"wall","radius":2.2,"height":.5}]
        self.assertEqual([item["id"] for item in target_sized_detections(detections)],["bird"])
        with self.assertRaises(ValueError):target_sized_detections(detections,maximum_radius=0)

    def test_vehicle_exclusion(self):
        self.assertEqual(exclude_near_vehicles([(0,0,0),(3,0,0)],[(0,0,0)],1.0),[(3.0,0.0,0.0)])
    def test_static_voxel_becomes_background(self):
        model=VoxelBackground(voxel_size=1,background_hits=2)
        self.assertEqual(len(model.update([(2,2,2)])),1);self.assertEqual(len(model.update([(2,2,2)])),1)
        self.assertEqual(model.update([(2,2,2)]),[])
    def test_cluster_and_detection(self):
        points=[(0.1*i,0,2) for i in range(10)]+[(10+0.1*i,0,2) for i in range(10)]
        clusters=euclidean_clusters(points,tolerance=0.25,minimum_points=5)
        self.assertEqual(len(clusters),2);self.assertEqual(cluster_detection("x",clusters[0])["point_count"],10)

    def test_motion_consistency_rejects_static_jitter(self):
        model=DetectionAssociator(confirmation_hits=3,minimum_speed=.8)
        self.assertEqual(model.update([{"position":[0,0,5]}],0.0),[])
        self.assertEqual(model.update([{"position":[.05,0,5]}],.2),[])
        self.assertEqual(model.update([{"position":[.02,0,5]}],.4),[])

    def test_motion_consistency_confirms_moving_target(self):
        model=DetectionAssociator(confirmation_hits=3,minimum_speed=.8)
        self.assertEqual(model.update([{"position":[0,0,5]}],0.0),[])
        self.assertEqual(model.update([{"position":[.6,0,5]}],.2),[])
        result=model.update([{"position":[1.2,.05,5]}],.4)
        self.assertEqual(len(result),1);self.assertEqual(result[0]["motion_hits"],2)

    def test_motion_consistency_rejects_direction_reversal(self):
        model=DetectionAssociator(confirmation_hits=3,minimum_speed=.8,minimum_direction_cosine=0.0)
        model.update([{"position":[0,0,5]}],0.0);model.update([{"position":[.6,0,5]}],.2)
        self.assertEqual(model.update([{"position":[0,0,5]}],.4),[])

    def test_motion_consistency_rejects_abrupt_acceleration(self):
        model=DetectionAssociator(confirmation_hits=3,minimum_speed=.8,minimum_direction_cosine=0.0,maximum_acceleration=5.0)
        model.update([{"position":[0,0,5]}],0.0);model.update([{"position":[.4,0,5]}],.2)
        self.assertEqual(model.update([{"position":[1.2,0,5]}],.4),[])

    def test_prediction_preserves_id_across_short_occlusion(self):
        model=DetectionAssociator(confirmation_hits=3,minimum_speed=.8,minimum_direction_cosine=.5,maximum_track_age=1.5)
        model.update([{"position":[0,0,5]}],0.0);model.update([{"position":[.6,0,5]}],.2)
        confirmed=model.update([{"position":[1.2,0,5]}],.4);identity=confirmed[0]["id"]
        model.update([],0.8);model.update([],1.2)
        reacquired=model.update([{"position":[4.2,0,5]}],1.4)
        self.assertEqual(reacquired[0]["id"],identity)

    def test_prediction_expires_id_after_occlusion_limit(self):
        model=DetectionAssociator(maximum_track_age=.5)
        model.update([{"position":[0,0,5]}],0.0);model.update([],.6)
        self.assertFalse(model.tracks)

if __name__=="__main__":unittest.main()
