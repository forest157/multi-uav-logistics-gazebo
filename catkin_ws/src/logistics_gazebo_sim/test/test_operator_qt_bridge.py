"""Exercise the actual Qt signal path, not only direct callback invocation."""

import json
import os
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
                patch.object(module.rospy.Time, 'now', return_value=module.rospy.Time(0)):
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
            plugin.analysis_timer.stop()
            plugin.analysis_timeout.stop()
            plugin.telemetry_timer.stop()
            plugin.energy_timer.stop()
            plugin.risk_timer.stop()
            for subscriber in plugin.ros_subscribers:
                subscriber.unregister()


if __name__ == '__main__':
    unittest.main()
