import copy
import math
import unittest

from logistics_gazebo_sim.fleet_operator_telemetry import (
    FleetOperatorTelemetry, validate_snapshot)


class FleetOperatorTelemetryTest(unittest.TestCase):
    def test_one_three_eight_rows_are_bounded_and_ordered(self):
        for count in (1, 3, 8):
            with self.subTest(count=count):
                model = FleetOperatorTelemetry(count)
                for index in range(count):
                    model.update_state(index, True, index == 0, 'OFFBOARD', 10.0)
                    model.update_pose(index, [index, 0, 18], 10.0)
                report = model.snapshot(10.2)
                self.assertEqual([v['vehicle_id'] for v in validate_snapshot(report)],
                                 ['uav{}'.format(i) for i in range(count)])
                self.assertEqual(report['vehicles'][0]['status'], 'ARMED')
                if count > 1:
                    self.assertEqual(report['vehicles'][1]['status'], 'READY')

    def test_stale_and_disconnected_hide_old_position(self):
        model = FleetOperatorTelemetry(1)
        model.update_state(0, False, False, '', 1.0)
        model.update_pose(0, [2, 3, 4], 1.0)
        self.assertEqual(model.snapshot(1.1)['vehicles'][0]['status'], 'DISCONNECTED')
        old = model.snapshot(4.0)['vehicles'][0]
        self.assertEqual(old['status'], 'STALE')
        self.assertIsNone(old['position_local_m'])
        self.assertIsNone(old['mode'])

    def test_battery_prefers_mavros_and_falls_back_to_labelled_model(self):
        model = FleetOperatorTelemetry(1)
        model.update_energy({'vehicles': [{'vehicle_id': 'uav0',
                                           'remaining_wh': 91.0,
                                           'remaining_fraction': 0.5,
                                           'reserve_safe': True}]}, 10.0)
        model.update_battery(0, 0.8, 10.0)
        current = model.snapshot(10.1)['vehicles'][0]
        self.assertEqual((current['battery_fraction'], current['battery_source']),
                         (0.8, 'MAVROS'))
        model.update_energy({'vehicles': [{'vehicle_id': 'uav0',
                                           'remaining_wh': 88.0,
                                           'remaining_fraction': 0.4}]}, 15.0)
        fallback = model.snapshot(15.1)['vehicles'][0]
        self.assertEqual((fallback['battery_fraction'], fallback['battery_source']),
                         (0.4, 'MODEL'))
        self.assertIsNone(model.snapshot(19.0)['vehicles'][0]['battery_fraction'])

    def test_malformed_values_do_not_enter_snapshot_or_qt(self):
        model = FleetOperatorTelemetry(3)
        self.assertFalse(model.update_pose(0, [1, math.nan, 2], 1.0))
        self.assertFalse(model.update_battery(0, 1.2, 1.0))
        self.assertFalse(model.update_energy({'vehicles': [
            {'vehicle_id': 'uav0'}, {'vehicle_id': 'uav0'}]}, 1.0))
        with self.assertRaises(ValueError):
            model.update_state(8, True, True, 'OFFBOARD', 1.0)
        report = model.snapshot(1.0)
        for mutation in (
                lambda data: data.update(vehicle_count=9),
                lambda data: data['vehicles'][0].update(vehicle_id='uav8'),
                lambda data: data['vehicles'][0].update(battery_fraction='0.5'),
                lambda data: data['vehicles'][0].update(position_local_m=[0, math.nan, 0]),
                lambda data: data['vehicles'][0].update(remaining_wh=float('inf'))):
            bad = copy.deepcopy(report)
            mutation(bad)
            with self.assertRaises(ValueError):
                validate_snapshot(bad)


if __name__ == '__main__':
    unittest.main()
