import unittest
from logistics_gazebo_sim.perception_acceptance import run_perception_acceptance_matrix

class PerceptionAcceptanceTest(unittest.TestCase):
    def test_complete_matrix_passes(self):
        report=run_perception_acceptance_matrix();self.assertTrue(report["pass"],report)
        self.assertEqual(report["case_count"],6)

if __name__=="__main__":unittest.main()
