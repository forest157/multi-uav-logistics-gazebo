#!/usr/bin/env python3
import unittest
import numpy as np
from logistics_gazebo_sim_ros.local_avoidance import (
    CollectiveOffsetPlanner, Orca3DPlanner, available_local_planners,
    create_local_planner)
from logistics_gazebo_sim_ros.dynamic_obstacles import DynamicObstacleError

class LocalAvoidanceTest(unittest.TestCase):
    def paths(self,count=3):
        middle=(count-1)*0.5
        return [[[0.0,0.0,(i-middle)*4.0,8.0],[5.0,10.0,(i-middle)*4.0,8.0]]
                for i in range(count)]
    def test_registry_keeps_stable_planner_and_adds_orca(self):
        self.assertEqual(available_local_planners(),("collective_offset","orca3d"))
        self.assertIsInstance(create_local_planner("collective_offset"),CollectiveOffsetPlanner)
        self.assertIsInstance(create_local_planner("orca3d"),Orca3DPlanner)
        with self.assertRaises(DynamicObstacleError):create_local_planner("missing")
    def test_orca_scales_to_eight_vehicles(self):
        result=Orca3DPlanner().plan(self.paths(8),[],max_speed=3.0)
        self.assertTrue(result["viable"]);self.assertEqual(result["vehicle_count"],8)
        self.assertEqual(len(result["commands"]),8)
        self.assertTrue(all(np.linalg.norm(c["velocity"])<=3.0001 for c in result["commands"]))
    def test_orca_changes_velocity_for_head_on_vehicle(self):
        paths=[[[0,-4,0,8],[4,4,0,8]],[[0,4,0,8],[4,-4,0,8]]]
        result=Orca3DPlanner().plan(paths,[],minimum_separation=3.0,max_speed=3.0)
        corrections=[c["correction_norm"] for c in result["commands"]]
        self.assertGreater(max(corrections),0.0)
        self.assertEqual(result["command_type"],"per_vehicle_velocity")
        self.assertTrue(result["constraints_satisfied"])
        self.assertAlmostEqual(result["commands"][0]["velocity"][1],
                               -result["commands"][1]["velocity"][1],places=3)
        self.assertGreaterEqual(result["predicted_minimum_separation_m"],2.95)
        self.assertTrue(result["shadow_mode"])
    def test_orca_respects_buffered_obstacle_clearance(self):
        obstacle={"id":"near_bird","position":[2,2.8,8],"velocity":[0,0,0],"radius":0.8,"height":1.0}
        result=Orca3DPlanner().plan([[[0,0,0,8],[4,8,0,8]]],[obstacle],
            max_speed=3.0,safety_buffer=0.5,required_clearance=0.5)
        self.assertGreater(result["commands"][0]["correction_norm"],0.0)
        self.assertTrue(result["constraints_satisfied"])
        self.assertGreaterEqual(result["predicted_minimum_obstacle_clearance_m"],-0.05)

    def test_orca_responds_to_moving_3d_obstacle(self):
        obstacle={"id":"bird","position":[2,0,8],"velocity":[0,0,0],"radius":1.0,"height":1.0}
        result=Orca3DPlanner().plan([[[0,0,0,8],[4,8,0,8]]],[obstacle],max_speed=3.0)
        self.assertGreater(result["commands"][0]["correction_norm"],0.0)
    def test_collective_adapter_preserves_legacy_contract(self):
        result=CollectiveOffsetPlanner().plan(self.paths(),[],candidate_offsets=[[0,0,0]])
        self.assertTrue(result["viable"]);self.assertEqual(result["algorithm"],"collective_offset")
        self.assertEqual(result["command_type"],"collective_offset")

if __name__=="__main__":unittest.main()
