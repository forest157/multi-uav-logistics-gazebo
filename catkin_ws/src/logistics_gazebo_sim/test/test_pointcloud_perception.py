import unittest
from logistics_gazebo_sim.pointcloud_perception import VoxelBackground,cluster_detection,euclidean_clusters,exclude_near_vehicles,target_sized_detections


class PointCloudPerceptionTest(unittest.TestCase):
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

if __name__=="__main__":unittest.main()
