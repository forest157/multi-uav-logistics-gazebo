import unittest

from logistics_gazebo_sim.clearance_analyzer import analyze_ground_capacity
from logistics_gazebo_sim.error_model import classify_exception


class AnalysisErrorTest(unittest.TestCase):
    def test_input_error_is_not_retryable(self):
        report=classify_exception(RuntimeError("E_DISTANCE"))
        self.assertEqual(report["category"],"INPUT")
        self.assertFalse(report["retryable"])

    def test_planner_timeout_is_unknown_and_retryable(self):
        report=classify_exception(RuntimeError("E_OMPL_TIMEOUT"))
        self.assertEqual(report["category"],"PLANNING")
        self.assertTrue(report["retryable"])

    def test_environment_signature_is_classified_separately(self):
        report=classify_exception(TypeError("SafeDumper object is not iterable"))
        self.assertEqual(report["code"],"E_ENVIRONMENT")

    def test_geometry_json_becomes_structured_context(self):
        report=classify_exception(RuntimeError(
            'E_START_CAPACITY:{"message":"空间不足","location":[46,46,0]}'))
        self.assertEqual(report["context"]["location"],[46,46,0])
        self.assertEqual(report["detail"],"空间不足")

    def test_default_takeoff_area_accepts_initial_line(self):
        report=analyze_ground_capacity(0,(-40,-40),"column",3,3.0)
        self.assertTrue(report["feasible"],report)

    def test_boundary_takeoff_area_reports_capacity(self):
        report=analyze_ground_capacity(
            0,(46,46),"column",3,3.0,"E_START_CAPACITY")
        self.assertFalse(report["feasible"])
        self.assertEqual(report["error_code"],"E_START_CAPACITY")


if __name__=="__main__":unittest.main()
