"""Exercise the actual Qt signal path, not only direct callback invocation."""

import json
import os
import tempfile
import unittest
from unittest.mock import patch

os.environ.setdefault('QT_QPA_PLATFORM', 'offscreen')

from python_qt_binding.QtCore import QObject, QTimer
from python_qt_binding.QtWidgets import QApplication
from std_msgs.msg import String

import logistics_gazebo_sim.operator_plugin as module


class _Sink:
    def publish(self, *args):
        pass

    def unregister(self):
        pass


class _Context(QObject):
    def add_widget(self, widget):
        self.widget = widget


class OperatorQtBridgeTest(unittest.TestCase):
    def test_queued_ros_snapshot_and_parameter_signal_reach_widgets(self):
        app = QApplication.instance() or QApplication([])
        with patch.object(module.rospy, 'Publisher', side_effect=lambda *a, **k: _Sink()), \
                patch.object(module.rospy, 'Subscriber', side_effect=lambda *a, **k: _Sink()), \
                patch.object(module.rospy.Time, 'now', return_value=module.rospy.Time(0)), \
                patch.object(module.OperatorPlugin, 'active_runtime_processes', return_value=[]):
            plugin = module.OperatorPlugin(_Context())
            plugin.analysis_timer.stop()
            plugin.start_x.setValue(-39.0)
            self.assertTrue(plugin.analysis_timer.isActive())
            plugin.analysis_timer.stop()
            snapshot = {'schema': 1, 'vehicle_count': 1, 'vehicles': [{
                'vehicle_id': 'uav0', 'status': 'READY', 'connected': True,
                'armed': False, 'mode': 'AUTO.LOITER', 'position_local_m': [0, 0, 0],
                'battery_fraction': 1.0, 'battery_source': 'MAVROS',
                'remaining_wh': None, 'reserve_safe': None, 'state_age_s': 0.1}]}
            plugin.ros_ui_bridge.telemetry_received.emit(String(data=json.dumps(snapshot)))
            QTimer.singleShot(100, app.quit)
            app.exec_()
            self.assertEqual(plugin.vehicle_table.rowCount(), 1)
            self.assertEqual(plugin.vehicle_table.item(0, 0).text(), 'uav0')
            running = plugin.requested_runtime_configuration()
            with patch.object(module.rospy, 'get_master') as master, \
                    patch.object(module.rospy, 'get_param',
                                 side_effect=lambda name: running[name.rsplit('/', 1)[-1]]):
                master.return_value.lookupNode.return_value = (1, 'online', 'http://127.0.0.1:1234')
                plugin.check_runtime_button.click()
            self.assertIn('参数一致', plugin.runtime_config_status.text())
            plugin.orca_timeout.setValue(0.5)
            self.assertIn('请重新核对', plugin.runtime_config_status.text())
            plugin.orca_timeout.setValue(0.6)
            with patch.object(module.rospy, 'get_master') as master, \
                    patch.object(module.rospy, 'get_param',
                                 side_effect=lambda name: dict(running,
                                     local_avoidance_algorithm='orca3d')[name.rsplit('/', 1)[-1]]):
                master.return_value.lookupNode.return_value = (1, 'online', 'http://127.0.0.1:1234')
                plugin.check_runtime_button.click()
            self.assertIn('不一致', plugin.runtime_config_status.text())
            with patch.object(module.rospy, 'get_master') as master:
                master.return_value.lookupNode.return_value = (-1, 'offline', '')
                plugin.check_runtime_button.click()
            self.assertIn('无法核对', plugin.runtime_config_status.text())
            with tempfile.TemporaryDirectory() as directory:
                preset = os.path.join(directory, 'task.json')
                report_path = os.path.join(directory, 'report.json')
                with patch.object(module.QFileDialog, 'getSaveFileName', return_value=(preset, '')):
                    plugin.save_task_button.click()
                plugin.start_x.setValue(-38.0)
                with patch.object(module.QFileDialog, 'getOpenFileName', return_value=(preset, '')):
                    plugin.load_task_button.click()
                self.assertEqual(plugin.start_x.value(), -39.0)
                self.assertFalse(plugin.start_sim.isEnabled())
                plugin.event_journal.planning_failure('FEASIBILITY', '规划失败',
                                                     '检查净空', 1728604800.5)
                with patch.object(module.QFileDialog, 'getSaveFileName',
                                  return_value=(report_path, '')):
                    plugin.export_report_button.click()
                with open(report_path, encoding='utf-8') as stream:
                    report = json.load(stream)
                self.assertEqual(report['kind'], 'operator_ui_snapshot')
                self.assertFalse(report['planning']['approved'])
                self.assertEqual(report['events'][0]['time_utc'],
                                 '2024-10-11T00:00:00.500000+00:00')
            plugin.analysis_timer.stop()
            plugin.analysis_timeout.stop()
            plugin.telemetry_timer.stop()
            plugin.energy_timer.stop()
            plugin.risk_timer.stop()
            for subscriber in plugin.ros_subscribers:
                subscriber.unregister()


if __name__ == '__main__':
    unittest.main()
