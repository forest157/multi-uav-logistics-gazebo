import copy
import unittest

from logistics_gazebo_sim.outdoor_acceptance import (audit_no_bird_outdoor,
                                                     audit_outdoor_flight)


class OutdoorAcceptanceTest(unittest.TestCase):
    def setUp(self):
        self.profile = {
            'spawn_center_m': [-45.0, -65.0],
            'spawn_spacing_m': 15.0,
            'obstacles': [{'kind': 'box', 'x': 0.0, 'y': 0.0,
                           'half_x': 2.0, 'half_y': 2.0, 'height': 10.0}],
        }
        self.rows = []
        for index in range(100):
            stamp = 10.0 + index * 0.1
            self.rows.append({
                'stamp': stamp,
                'mission': {'state': 'RUNNING', 'dynamic_action': 'NORMAL'},
                'flight_states': {str(i): {'armed': True} for i in range(3)},
                'perception': {'state': 'TRACKING', 'confirmed_targets': 0},
                'perception_receipt_stamp': stamp,
                'safety': {'logistics_fleet/safety': {'level': 0}},
                'safety_receipt_stamp': stamp,
                'truth_receipt_stamp': stamp,
                'truth_evaluation_only': {
                    'iris{}'.format(i): {'position': [-60.0 + 15.0 * i,
                                                     -65.0, 19.0]}
                    for i in range(3)},
                'poses': {str(i): [stamp, 0.0, 0.0, 19.0]
                          for i in range(3)},
                'targets': {str(i): [stamp, 0.0, 0.0, 19.0]
                            for i in range(3)},
            })
        complete = copy.deepcopy(self.rows[-1])
        complete['stamp'] += 0.1
        complete['mission']['state'] = 'COMPLETE'
        complete['flight_states'] = {str(i): {'armed': False}
                                     for i in range(3)}
        self.rows.append(complete)

    def test_complete_clean_flight_passes(self):
        report = audit_no_bird_outdoor(self.rows, self.profile)
        self.assertTrue(report['passed'], report['failure_reasons'])
        self.assertEqual(report['minimum_truth_separation_m'], 15.0)

    def test_false_lidar_target_fails(self):
        self.rows[50]['perception']['confirmed_targets'] = 1
        report = audit_no_bird_outdoor(self.rows, self.profile)
        self.assertIn('maximum_lidar_confirmed_targets',
                      report['failure_reasons'])

    def test_stale_truth_and_safety_error_fail(self):
        for row in self.rows[:-1]:
            row['truth_receipt_stamp'] = row['stamp'] - 1.0
        self.rows[50]['safety']['logistics_fleet/safety']['level'] = 2
        report = audit_no_bird_outdoor(self.rows, self.profile)
        self.assertIn('insufficient_valid_truth_samples',
                      report['failure_reasons'])
        self.assertIn('safety_error_samples', report['failure_reasons'])

    def test_single_bird_requires_detection_orca_and_clearance(self):
        for row in self.rows[:-1]:
            row['truth_evaluation_only']['bird_crossing_0'] = {
                'position': [-50.0, -65.0, 19.0]}
        self.rows[50]['perception']['confirmed_targets'] = 1
        self.rows[50]['mission']['dynamic_action'] = 'ORCA'
        self.rows[50]['mission']['state'] = 'DYNAMIC_ORCA'
        report = audit_outdoor_flight(self.rows, self.profile,
                                      expect_single_bird=True)
        self.assertTrue(report['passed'], report['failure_reasons'])
        self.assertAlmostEqual(report['minimum_bird_clearance_m'], 3.05)

        self.rows[50]['perception']['confirmed_targets'] = 0
        self.rows[50]['mission']['dynamic_action'] = 'NORMAL'
        self.rows[50]['mission']['state'] = 'RUNNING'
        self.rows[50]['truth_evaluation_only']['bird_crossing_0']['position'] = [
            -60.0, -65.0, 19.0]
        report = audit_outdoor_flight(self.rows, self.profile,
                                      expect_single_bird=True)
        self.assertIn('minimum_bird_clearance_m', report['failure_reasons'])
        self.assertIn('lidar_bird_detection_missing', report['failure_reasons'])
        self.assertIn('orca_intervention_missing', report['failure_reasons'])


if __name__ == '__main__':
    unittest.main()
