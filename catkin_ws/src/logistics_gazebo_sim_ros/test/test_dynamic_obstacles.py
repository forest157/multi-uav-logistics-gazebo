#!/usr/bin/env python3
import unittest

import numpy as np

from logistics_gazebo_sim_ros.dynamic_obstacles import (
    DynamicObstacleError, assess_timed_path, interpolate_timed_path,
    obstacle_clearance, predict_position)


class DynamicObstacleTest(unittest.TestCase):
    def obstacle(self, position=(5.0, -5.0, 5.0), velocity=(0.0, 1.0, 0.0)):
        return {"id": "crossing_1", "position": position,
                "velocity": velocity, "radius": 0.8, "height": 2.0}

    def test_constant_velocity_prediction(self):
        predicted = predict_position(self.obstacle(), 3.0)
        np.testing.assert_allclose(predicted, [5.0, -2.0, 5.0])

    def test_timed_path_interpolation(self):
        path = [[0.0, 0.0, 5.0, 5.0], [10.0, 10.0, 0.0, 5.0]]
        np.testing.assert_allclose(interpolate_timed_path(path, 2.5), [2.5, 3.75, 5.0])

    def test_crossing_trajectory_is_critical(self):
        path = [[0.0, 0.0, 0.0, 5.0], [10.0, 10.0, 0.0, 5.0]]
        report = assess_timed_path(path, [self.obstacle()], horizon=10.0)
        self.assertEqual(report["level"], "CRITICAL")
        self.assertEqual(report["obstacle_id"], "crossing_1")
        self.assertIsNotNone(report["time_to_conflict_s"])

    def test_vertical_separation_is_safe(self):
        path = [[0.0, 0.0, 12.0, 12.0], [10.0, 10.0, 12.0, 12.0]]
        report = assess_timed_path(path, [self.obstacle()], horizon=10.0)
        self.assertEqual(report["level"], "SAFE")
        self.assertGreater(report["minimum_clearance_m"], 2.0)

    def test_signed_clearance_detects_overlap(self):
        value = obstacle_clearance([5.0, 0.0, 5.0], self.obstacle(), 5.0)
        self.assertLess(value, 0.0)

    def test_invalid_obstacle_reports_input_error(self):
        with self.assertRaises(DynamicObstacleError):
            predict_position({"id": "broken"}, 1.0)


if __name__ == "__main__":
    unittest.main()
