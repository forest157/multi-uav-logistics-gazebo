import unittest
from pathlib import Path
from logistics_gazebo_sim import target_tracking

ROOT=Path(__file__).resolve().parents[1]


class LidarModelTest(unittest.TestCase):
    def test_single_multilayer_sensor_is_namespaced(self):
        model=(ROOT/"models"/"iris_3d_lidar"/"iris_3d_lidar.sdf.jinja").read_text(encoding="utf-8")
        self.assertEqual(model.count("sensor name='lidar_3d'"),1)
        self.assertIn("<vertical><samples>16</samples>",model)
        self.assertIn("<max>35.0</max>",model)
        self.assertIn("uav{{ mavlink_id | int - 1 }}",model)

    def test_three_vehicle_launch_uses_lidar_model(self):
        launch=(ROOT/"launch"/"three_uav_sitl.launch").read_text(encoding="utf-8")
        self.assertEqual(launch.count("single_vehicle_lidar_spawn.launch"),3)
        aggregator=(ROOT/"scripts"/"lidar_cloud_aggregator").read_text(encoding="utf-8")
        self.assertIn('"/perception/lidar_points",PointCloud2',aggregator)

if __name__=="__main__":unittest.main()
