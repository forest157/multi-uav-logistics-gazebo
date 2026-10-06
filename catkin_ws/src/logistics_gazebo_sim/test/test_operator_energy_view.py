import copy
import math
import unittest

from logistics_gazebo_sim.operator_energy_view import validate_energy_advisory


def advisory():
    return {
        'mode': 'shadow', 'control_applied': False, 'fleet_level': 'LOW',
        'energy_age_s': 0.2,
        'vehicles': [
            {'vehicle_id': 'uav0', 'level': 'LOW', 'final_margin_wh': 8.0,
             'usable_margin_wh': 16.0, 'required_to_land_wh': 8.0},
            {'vehicle_id': 'uav1', 'level': 'NORMAL', 'final_margin_wh': 20.0,
             'usable_margin_wh': 28.0, 'required_to_land_wh': 8.0},
        ],
        'slot_assignments': {'uav0': 1, 'uav1': 0},
        'landing_order': ['uav0', 'uav1'],
    }


class OperatorEnergyViewTest(unittest.TestCase):
    def test_advisory_exposes_per_vehicle_order_and_slot(self):
        result = validate_energy_advisory(advisory())
        self.assertEqual(result['level'], 'LOW')
        self.assertEqual(result['rows'][0]['slot'], 1)
        self.assertEqual(result['rows'][1]['landing_rank'], 2)

    def test_stale_hides_old_rows(self):
        value = advisory()
        value.update(fleet_level='STALE', vehicles=[], slot_assignments={}, landing_order=[])
        self.assertEqual(validate_energy_advisory(value)['rows'], [])
        value['energy_age_s'] = math.inf  # Current advisor before first sample.
        self.assertIsNone(validate_energy_advisory(value)['age_s'])

    def test_rejects_inconsistent_or_unsafe_payloads(self):
        for mutation in (
                lambda value: value.update(control_applied=True),
                lambda value: value.update(mode='active'),
                lambda value: value['vehicles'][0].update(final_margin_wh=math.nan),
                lambda value: value['vehicles'][0].update(vehicle_id='uav8'),
                lambda value: value['slot_assignments'].update(uav0=8),
                lambda value: value['slot_assignments'].update(uav1=1),
                lambda value: value.update(landing_order=['uav0', 'uav0']),
                lambda value: value.update(fleet_level='STALE')):
            bad = copy.deepcopy(advisory())
            mutation(bad)
            with self.subTest(mutation=mutation), self.assertRaises(ValueError):
                validate_energy_advisory(bad)


if __name__ == '__main__':
    unittest.main()
