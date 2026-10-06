import copy
import math
import unittest

from logistics_gazebo_sim.operator_risk_view import validate_risk_report


def report():
    return {
        'level': 'WARNING', 'obstacle_count': 1,
        'vehicle_reports': [
            {'vehicle_id': 'uav0', 'level': 'WARNING', 'obstacle_id': 'crossing_1',
             'minimum_clearance_m': 0.7, 'time_to_conflict_s': None,
             'critical_time_s': 2.3, 'critical_position': [4.0, 5.0, 8.0]},
            {'vehicle_id': 'uav1', 'level': 'SAFE', 'obstacle_id': 'crossing_1',
             'minimum_clearance_m': 3.2, 'time_to_conflict_s': None,
             'critical_time_s': 3.1, 'critical_position': [7.0, 5.0, 8.0]},
        ],
        'fleet_separation': {'safe': True, 'minimum_separation_m': 3.2,
                             'closest_pair': ['uav0', 'uav1'],
                             'time_to_conflict_s': None},
        'local_avoidance_algorithm': 'orca3d',
        'avoidance': {'viable': True},
    }


class OperatorRiskViewTest(unittest.TestCase):
    def test_conflict_object_point_and_separation_are_exposed(self):
        result = validate_risk_report(report())
        self.assertEqual(result['rows'][0]['obstacle_id'], 'crossing_1')
        self.assertEqual(result['rows'][0]['closest_position_m'], [4.0, 5.0, 8.0])
        self.assertEqual(result['closest_pair'], ['uav0', 'uav1'])
        self.assertEqual((result['algorithm'], result['plan_viable']), ('orca3d', True))

    def test_stale_and_empty_airspace_hide_conflicts(self):
        self.assertEqual(validate_risk_report({
            'level': 'STALE', 'obstacle_count': 0, 'vehicle_reports': []})['rows'], [])
        self.assertEqual(validate_risk_report({
            'level': 'SAFE', 'obstacle_count': 0, 'vehicle_reports': []})['rows'], [])

    def test_rejects_malformed_or_unbounded_prediction(self):
        for mutation in (
                lambda value: value.update(level='UNKNOWN'),
                lambda value: value.update(obstacle_count=-1),
                lambda value: value['vehicle_reports'][0].update(vehicle_id='uav8'),
                lambda value: value['vehicle_reports'][1].update(vehicle_id='uav0'),
                lambda value: value['vehicle_reports'][0].update(obstacle_id='x' * 65),
                lambda value: value['vehicle_reports'][0].update(minimum_clearance_m=math.nan),
                lambda value: value['vehicle_reports'][0].update(time_to_conflict_s=-1),
                lambda value: value['vehicle_reports'][0].update(critical_position=[0, math.inf, 2]),
                lambda value: value['fleet_separation'].update(closest_pair=['uav0', 'uav0']),
                lambda value: value.update(local_avoidance_algorithm='unknown'),
                lambda value: value.update(avoidance={'viable': 'true'}),
                lambda value: value.update(level='STALE')):
            bad = copy.deepcopy(report())
            mutation(bad)
            with self.subTest(mutation=mutation), self.assertRaises(ValueError):
                validate_risk_report(bad)


if __name__ == '__main__':
    unittest.main()
