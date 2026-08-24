#!/usr/bin/env python3
import importlib.machinery,os,unittest
ROOT=os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
benchmark=importlib.machinery.SourceFileLoader("benchmark_local_avoidance",os.path.join(ROOT,"scripts","benchmark_local_avoidance")).load_module()
class AlgorithmBenchmarkTest(unittest.TestCase):
    def test_profiles_are_deterministic_and_multi_bird_has_two_obstacles(self):
        first=benchmark.scenario(2,profile="diagonal",speed=1.5,seed=7)
        second=benchmark.scenario(2,profile="diagonal",speed=1.5,seed=7)
        self.assertEqual(first,second)
        _,obstacles=benchmark.scenario(0,profile="multi_bird",speed=1.0,seed=0)
        self.assertEqual(len(obstacles),2)
        self.assertTrue(any(abs(item["velocity"][2])>0.0 for item in obstacles))

    def test_matrix_records_groups_and_metadata(self):
        rows,summary=benchmark.run(1,profiles=("crossing","head_on"),speeds=(0.8,1.5),seeds=(0,1))
        self.assertEqual(len(rows),24)
        self.assertTrue(all("profile" in row and "seed" in row and "obstacle_count" in row for row in rows))
        for value in summary.values():
            self.assertEqual(value["cases"],8)
            self.assertEqual(len(value["groups"]),8)
            self.assertEqual(set(value["profile_groups"]),{"crossing","head_on"})
            self.assertEqual(set(value["speed_groups"]),{"0.8","1.5"})
            self.assertEqual(set(value["seed_groups"]),{"0","1"})
            self.assertIn("rejections",value)

    def test_small_comparison_has_common_metrics(self):
        rows,summary=benchmark.run(2)
        self.assertEqual(set(summary),set(benchmark.ALGORITHMS));self.assertEqual(len(rows),6)
        for algorithm,value in summary.items():
            self.assertEqual(value["cases"],2);self.assertGreaterEqual(value["viable_rate"],0.0);self.assertLessEqual(value["viable_rate"],1.0)
            self.assertGreaterEqual(value["mean_ms"],0.0);self.assertIn("minimum_accepted_separation_m",value)
        self.assertTrue(all("energy_proxy" in row and "smoothness_proxy" in row for row in rows))
if __name__=="__main__":unittest.main()
