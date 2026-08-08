#!/usr/bin/env python3

import unittest

import numpy as np

from logistics_gazebo_sim.formation_scheduler import (
    FormationScheduleError,
    build_formation_schedule,
    safe_blend,
)


class FormationSchedulerTest(unittest.TestCase):
    def test_safe_blend_preserves_minimum_separation(self):
        source = [[0.0, 0.0, 0.0], [4.0, 0.0, 0.0], [-4.0, 0.0, 0.0]]
        target = [[0.0, 0.0, 0.0], [0.0, 0.0, 4.0], [0.0, 0.0, -4.0]]
        for ratio in np.linspace(0.0, 1.0, 21):
            values = safe_blend(source, target, float(ratio), 3.0)
            minimum = min(np.linalg.norm(values[i] - values[j]) for i in range(len(values)) for j in range(i + 1, len(values)))
            self.assertGreaterEqual(minimum, 3.0 - 1e-6)

    def test_open_path_keeps_preferred_formation(self):
        times = np.linspace(0.0, 10.0, 51)
        centers = [[10.0 + t, 10.0, 12.0] for t in times]
        schedule = build_formation_schedule(
            0, times, centers, "triangle", 3, 4.0, 2.0
        )
        self.assertEqual(schedule["switches"], [])
        self.assertEqual(schedule["formation_sample_counts"].get("triangle"), len(times))
        self.assertGreaterEqual(schedule["minimum_separation_m"], 3.0)

    def test_narrow_obstacle_edge_uses_vertical_formation(self):
        times = np.linspace(0.0, 20.0, 201)
        x_values = np.concatenate(
            (
                np.linspace(-15.0, -19.5, 81),
                np.full(39, -19.5),
                np.linspace(-19.5, -15.0, 81),
            )
        )
        centers = [[float(x), -24.0, 8.0] for x in x_values]
        schedule = build_formation_schedule(
            0, times, centers, "triangle", 3, 4.0, 2.0
        )
        self.assertGreater(schedule["formation_sample_counts"].get("vertical", 0), 0)
        self.assertGreaterEqual(len(schedule["switches"]), 2)
        self.assertEqual(schedule["keys"][0][1], schedule["keys"][-1][1])

    def test_center_inside_obstacle_reports_no_feasible_formation(self):
        with self.assertRaises(FormationScheduleError):
            build_formation_schedule(
                0, [0.0, 1.0], [[-24.0, -24.0, 8.0]] * 2,
                "triangle", 3, 4.0, 2.0
            )


if __name__ == "__main__":
    unittest.main()
