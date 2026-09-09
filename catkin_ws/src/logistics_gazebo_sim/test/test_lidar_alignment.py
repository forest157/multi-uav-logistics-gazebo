import unittest
from unittest.mock import patch, Mock
import importlib.machinery
from pathlib import Path
import rospy
from geometry_msgs.msg import Pose, Point32
from sensor_msgs.msg import PointCloud
from sensor_msgs import point_cloud2
from logistics_gazebo_sim.lidar_alignment import select_pose, valid_return


class LidarAlignmentTest(unittest.TestCase):
    def test_ros_aggregation_filters_misses_and_does_not_republish(self):
        module=importlib.machinery.SourceFileLoader('lidar_aggregation_test',str(Path(__file__).resolve().parents[1]/'scripts/lidar_cloud_aggregator')).load_module()
        node=module.LidarCloudAggregator.__new__(module.LidarCloudAggregator)
        pose=Pose();pose.position.x=1;pose.position.z=8;pose.orientation.w=1
        scan=PointCloud();scan.header.stamp=rospy.Time.from_sec(10)
        scan.points=[Point32(9,0,0),Point32(35,0,0)]
        node.poses=[[(10,pose)]];node.clouds=[scan];node.consumed=[0.]
        node.origins=[(0,0)];node.publisher=Mock()
        with patch.object(rospy.Time,'now',return_value=rospy.Time.from_sec(10.1)):
            node.publish_locked();node.publish_locked()
        self.assertEqual(node.publisher.publish.call_count,1)
        cloud=node.publisher.publish.call_args[0][0]
        points=list(point_cloud2.read_points(cloud,field_names=('x','y','z')))
        self.assertEqual(len(points),1)
        self.assertAlmostEqual(points[0][0],10)
        self.assertAlmostEqual(points[0][2],8.18,places=5)
        self.assertEqual(cloud.header.stamp,scan.header.stamp)
    def test_no_echo_sphere_is_not_an_obstacle(self):
        for p in [(35.,0.,0.),(0.,-35.,0.),(21.,28.,0.),(0.,0.,34.999999)]:
            self.assertFalse(valid_return(p))
        self.assertTrue(valid_return((10.,0.,0.)))
        self.assertFalse(valid_return((float('inf'),0.,0.)))
    def test_static_return_uses_acquisition_pose(self):
        # At scan time vehicle x=1, wall return x=9. Latest pose x=2 is wrong.
        history=[(10.0, 1.0), (10.2, 2.0)]
        self.assertEqual(9+select_pose(history,10.0,10.2,9.8),10)

    def test_duplicate_scan_cannot_look_fresh(self):
        self.assertIsNone(select_pose([(10.,1)],10.,10.1,10.))

    def test_missing_time_aligned_pose_is_rejected(self):
        self.assertIsNone(select_pose([(10.2,1)],10.,10.2,9.))

    def test_stale_future_and_invalid_scans(self):
        for stamp,now in [(10.,11.),(11.,10.),(float('nan'),10.),(0.,0.)]:
            self.assertIsNone(select_pose([(stamp,1)],stamp,now,0.))
