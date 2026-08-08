#!/usr/bin/env python3
import unittest
from logistics_gazebo_sim_ros.fleet_config import line_spawn_offsets,validate_vehicle_count,indexed_palette
class FleetConfigTest(unittest.TestCase):
    def test_vehicle_count_bounds(self):
        self.assertEqual(validate_vehicle_count("8"),8)
        for value in (0,9,"bad"):
            with self.assertRaises(ValueError):validate_vehicle_count(value)
    def test_spawn_offsets_are_centered_for_even_and_odd_fleets(self):
        for count in (1,3,4,8):
            offsets=line_spawn_offsets(count,3.0)
            self.assertEqual(len(offsets),count)
            self.assertAlmostEqual(sum(x for x,_ in offsets),0.0)
            if count>1:self.assertAlmostEqual(offsets[1][0]-offsets[0][0],3.0)
    def test_palette_covers_configured_fleet(self):self.assertEqual(len(indexed_palette(8)),8)
if __name__=="__main__":unittest.main()
