#!/usr/bin/env python3
import importlib.machinery
import os
import unittest

import numpy as np


SCRIPT = os.path.join(os.path.dirname(__file__), "..", "scripts", "fleet_mission_player")
mission_player = importlib.machinery.SourceFileLoader("mission_player", SCRIPT).load_module()


class SoftLandingTest(unittest.TestCase):
    def test_smoothstep_descent_respects_speed_limit(self):
        cruise, touchdown, limit = 12.0, 0.18, 0.25
        duration = mission_player.soft_descent_duration(cruise, touchdown, 12.0, limit)
        times = np.linspace(0.0, duration, 2001)
        ratios = np.array([mission_player.smoothstep(t / duration) for t in times])
        heights = cruise + (touchdown - cruise) * ratios
        speeds = np.abs(np.diff(heights) / np.diff(times))
        self.assertLessEqual(float(speeds.max()), limit + 1e-5)
        self.assertAlmostEqual(float(heights[-1]), touchdown)

    def test_safe_formation_adds_tracking_reserve_without_centroid_drift(self):
        raw = [(-3.0, 0.0, 0.0), (0.0, 0.0, 0.0), (3.0, 0.0, 0.0)]
        safe = mission_player.safe_formation(raw, 3.3)
        self.assertAlmostEqual(sum(value[0] for value in safe), 0.0)
        self.assertAlmostEqual(min(
            np.linalg.norm(np.asarray(safe[i])-np.asarray(safe[j]))
            for i in range(3) for j in range(i+1, 3)), 3.3)

    def test_safe_formation_rejects_coincident_vehicles(self):
        with self.assertRaises(ValueError):
            mission_player.safe_formation([(0.0, 0.0, 0.0)] * 2, 3.3)

    def test_existing_slower_descent_is_preserved(self):
        planned = 90.0
        self.assertEqual(
            mission_player.soft_descent_duration(5.0, 0.18, planned), planned
        )


if __name__ == "__main__":
    unittest.main()
