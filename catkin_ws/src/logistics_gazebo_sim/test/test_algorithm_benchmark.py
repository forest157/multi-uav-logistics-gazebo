#!/usr/bin/env python3
import importlib.machinery,os,unittest
ROOT=os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
benchmark=importlib.machinery.SourceFileLoader("benchmark_local_avoidance",os.path.join(ROOT,"scripts","benchmark_local_avoidance")).load_module()
class AlgorithmBenchmarkTest(unittest.TestCase):
    def test_small_comparison_has_common_metrics(self):
        rows,summary=benchmark.run(2)
        self.assertEqual(set(summary),set(benchmark.ALGORITHMS));self.assertEqual(len(rows),6)
        for algorithm,value in summary.items():
            self.assertEqual(value["cases"],2);self.assertGreaterEqual(value["viable_rate"],0.0);self.assertLessEqual(value["viable_rate"],1.0)
            self.assertGreaterEqual(value["mean_ms"],0.0);self.assertIn("minimum_accepted_separation_m",value)
        self.assertTrue(all("energy_proxy" in row and "smoothness_proxy" in row for row in rows))
if __name__=="__main__":unittest.main()
