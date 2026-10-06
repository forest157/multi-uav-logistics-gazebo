import math
import unittest

from logistics_gazebo_sim.risk_prediction_summary import summarize_predictions


def track(index, observed=True):
    return {'id': 'track_{}'.format(index), 'position': [index, 0.0, 8.0],
            'velocity': [1.0, 0.0, 0.0], 'observed': observed}


class RiskPredictionSummaryTest(unittest.TestCase):
    def test_samples_are_bounded_and_conflict_tracks_come_first(self):
        result = summarize_predictions([track(index) for index in range(12)], 8.0,
                                       priority_ids=['track_11', 'track_10'])
        self.assertEqual(result['total_tracks'], 12)
        self.assertEqual(result['truncated_tracks'], 4)
        self.assertEqual([item['id'] for item in result['tracks'][:2]],
                         ['track_11', 'track_10'])
        self.assertEqual(result['tracks'][0]['samples'],
                         [[0.0, 11.0, 0.0, 8.0], [4.0, 15.0, 0.0, 8.0],
                          [8.0, 19.0, 0.0, 8.0]])

    def test_occluded_state_and_horizon_cap_are_preserved(self):
        result = summarize_predictions([track(0, observed=False)], 60.0)
        self.assertFalse(result['tracks'][0]['observed'])
        self.assertEqual(result['horizon_s'], 30.0)
        self.assertEqual(result['tracks'][0]['samples'][-1][0], 30.0)

    def test_invalid_tracks_fail_closed(self):
        for obstacles, horizon in (
                ([track(0), track(0)], 8.0),
                ([dict(track(0), position=[0, math.nan, 8])], 8.0),
                ([dict(track(0), observed='false')], 8.0),
                ([track(0)], math.inf)):
            with self.subTest(obstacles=obstacles, horizon=horizon), self.assertRaises(ValueError):
                summarize_predictions(obstacles, horizon)


if __name__ == '__main__':
    unittest.main()
