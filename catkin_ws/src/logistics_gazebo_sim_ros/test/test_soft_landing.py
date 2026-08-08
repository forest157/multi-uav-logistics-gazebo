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

    def test_existing_slower_descent_is_preserved(self):
        planned = 90.0
        self.assertEqual(
            mission_player.soft_descent_duration(5.0, 0.18, planned), planned
        )


if __name__ == "__main__":
    unittest.main()
