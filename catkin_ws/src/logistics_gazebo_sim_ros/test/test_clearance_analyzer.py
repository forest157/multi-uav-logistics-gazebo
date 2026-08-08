import unittest

from logistics_gazebo_sim_ros.clearance_analyzer import (
    analyze_candidates, analyze_path, obstacle_primitives, sample_polyline)


class ClearanceAnalyzerTest(unittest.TestCase):
    def test_samples_segment_at_requested_resolution(self):
        samples = sample_polyline([[0, 0, 8], [2, 0, 8]], step=0.5)
        self.assertEqual(len(samples), 5)
        self.assertEqual(samples[-1].tolist(), [2.0, 0.0, 8.0])

    def test_converts_all_obstacles_to_metric_primitives(self):
        from logistics_gazebo_sim_ros.scenes import SCENES
        for scene in SCENES.values():
            primitives = obstacle_primitives(scene)
            self.assertTrue(primitives)
            self.assertTrue(all(item["height"] > 0 for item in primitives))

    def test_open_high_path_is_feasible(self):
        report = analyze_path(
            0, [[-40, -40, 30], [40, 40, 30]], formation="triangle")
        self.assertTrue(report["feasible"], report)
        self.assertGreater(report["sample_count"], 100)

    def test_low_path_through_building_reports_corridor(self):
        report = analyze_path(
            0, [[-24, -27, 8], [-24, -17, 8]], formation="triangle")
        self.assertFalse(report["feasible"])
        self.assertEqual(report["error_code"], "E_CORRIDOR_TOO_NARROW")
        self.assertTrue(report["obstacle"].startswith("shopping_centre"))

    def test_vertical_formation_rejected_near_ceiling(self):
        report = analyze_path(
            0, [[0, 0, 44], [1, 0, 44]], formation="vertical")
        self.assertFalse(report["feasible"])
        self.assertEqual(report["error_code"], "E_VERTICAL_CLEARANCE")

    def test_candidate_report_preserves_failures(self):
        report = analyze_candidates(
            0, [[-40, -40, 30], [40, 40, 30]],
            ["triangle", "vertical"])
        self.assertTrue(report["feasible"])
        self.assertEqual(set(report["reports"]), {"triangle", "vertical"})


if __name__ == "__main__":
    unittest.main()
