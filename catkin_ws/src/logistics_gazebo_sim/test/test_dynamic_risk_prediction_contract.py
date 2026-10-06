import importlib.machinery
import os
import types
import unittest
from unittest.mock import patch

import rospy

from logistics_gazebo_sim.operator_risk_view import validate_risk_report


SCRIPT = os.path.join(os.path.dirname(__file__), '..', 'scripts', 'dynamic_risk_monitor')
monitor = importlib.machinery.SourceFileLoader('risk_prediction_contract_monitor', SCRIPT).load_module()


class DynamicRiskPredictionContractTest(unittest.TestCase):
    def instance(self):
        item = monitor.DynamicRiskMonitor.__new__(monitor.DynamicRiskMonitor)
        item.last_obstacle_stamp = rospy.Time(10)
        item.obstacle_source_stamp = 10.0
        item.obstacles = [{'id': 'crossing_1', 'position': [4.0, 0.0, 8.0],
                           'velocity': [0.0, 0.0, 0.0], 'radius': 0.5,
                           'height': 1.0, 'observed': True}]
        item.orca_recovery_active = False
        item.poses = [types.SimpleNamespace(x=0.0, y=0.0, z=8.0)]
        item.targets = [types.SimpleNamespace(x=8.0, y=0.0, z=8.0)]
        item.count = 1
        item.horizon = 8.0
        item.warning = 2.0
        item.minimum_separation = 3.0
        item.local_avoidance_algorithm = 'collective_offset'
        item.level_hysteresis = types.SimpleNamespace(update=lambda level, now: level)
        item.paths_for_report = lambda: [[[0.0, 0.0, 0.0, 8.0], [8.0, 8.0, 0.0, 8.0]]]
        item.critical_ttc_threshold = lambda: 4.0
        item.request_plan = lambda paths: {'viable': False}
        item.execution_context = lambda: None
        return item

    def test_new_risk_report_contains_valid_bounded_track(self):
        item = self.instance()
        with patch.object(monitor.rospy.Time, 'now', return_value=rospy.Time(10)), \
                patch.object(monitor.rospy, 'get_time', return_value=10.0):
            result = item.report()
        view = validate_risk_report(result)
        self.assertEqual(result['prediction_summary']['frame'], 'world')
        self.assertEqual(view['prediction_tracks'][0]['id'], 'crossing_1')
        self.assertEqual(len(view['prediction_tracks'][0]['samples']), 3)

    def test_old_source_stamp_cannot_publish_old_track(self):
        item = self.instance()
        item.obstacle_source_stamp = 8.0
        with patch.object(monitor.rospy.Time, 'now', return_value=rospy.Time(10)):
            result = item.report()
        self.assertEqual(result['level'], 'STALE')
        self.assertEqual(result['obstacle_count'], 0)
        self.assertNotIn('prediction_summary', result)


if __name__ == '__main__':
    unittest.main()
