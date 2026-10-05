import unittest
from pathlib import Path
from logistics_gazebo_sim import target_tracking

ROOT=Path(__file__).resolve().parents[1]


class LidarModelTest(unittest.TestCase):
    def test_scan_publisher_uses_completed_measurement_and_exact_sample_angles(self):
        model=(ROOT/"models"/"iris_3d_lidar"/"iris_3d_lidar.sdf.jinja").read_text()
        source=(ROOT/"src"/"logistics_lidar_plugin.cpp").read_text()
        self.assertIn("liblogistics_lidar_plugin.so",model)
        self.assertIn("ConnectUpdated",source)
        self.assertIn("LastMeasurementTime()",source)
        self.assertIn("yaw0+i*yaw_step",source)
        self.assertIn("pitch0+j*pitch_step",source)
        self.assertNotIn("LastUpdateTime()",source)
        self.assertNotIn("WorldPose()",source)
        self.assertNotIn("ros::Time::now()",source)

    def test_single_multilayer_sensor_is_namespaced(self):
        model=(ROOT/"models"/"iris_3d_lidar"/"iris_3d_lidar.sdf.jinja").read_text(encoding="utf-8")
        self.assertEqual(model.count("sensor name='lidar_3d'"),1)
        self.assertIn("<vertical><samples>32</samples>",model)
        self.assertIn("<horizontal><samples>720</samples>",model)
        self.assertIn("<max>35.0</max>",model)
        self.assertIn("uav{{ mavlink_id | int - 1 }}",model)

    def test_three_vehicle_launch_uses_lidar_model(self):
        launch=(ROOT/"launch"/"three_uav_sitl.launch").read_text(encoding="utf-8")
        instance=(ROOT/"launch"/"sitl_instance.launch").read_text(encoding="utf-8")
        self.assertEqual(launch.count("sitl_instance.launch"),3)
        self.assertEqual(instance.count("single_vehicle_lidar_spawn.launch"),1)
        aggregator=(ROOT/"scripts"/"lidar_cloud_aggregator").read_text(encoding="utf-8")
        self.assertIn('"/perception/lidar_points",PointCloud2',aggregator)

    def test_cloud_aggregation_is_independent_from_visualization(self):
        launch=(ROOT/"launch"/"three_uav_mission.launch").read_text(encoding="utf-8")
        self.assertIn('<arg name="lidar_cloud_aggregation" default="true"/>',launch)
        self.assertIn('<node if="$(arg lidar_cloud_aggregation)"',launch)
        self.assertIn('<param name="static_map_margin_m" value="1.5"/>',launch)
        self.assertEqual(launch.count('<param name="vehicle_exclusion_radius_m" value="1.8"/>'),2)

if __name__=="__main__":unittest.main()
