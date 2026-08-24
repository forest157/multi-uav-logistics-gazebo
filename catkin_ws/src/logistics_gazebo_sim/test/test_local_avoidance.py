#!/usr/bin/env python3
import unittest
import numpy as np
from logistics_gazebo_sim.local_avoidance import (
    CollectiveOffsetPlanner, Orca3DPlanner, OrcaCommandGate, orca_position_targets, available_local_planners,
    create_local_planner)
from logistics_gazebo_sim.dynamic_obstacles import DynamicObstacleError

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
        self.assertFalse(result["shadow_mode"]);self.assertTrue(result["requires_external_safety_gate"])
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
    def test_orca_command_gate_limits_and_smooths_velocity(self):
        gate=OrcaCommandGate(2,max_speed=2.0,max_climb_rate=0.5,
            max_acceleration=1.0,smoothing=0.5,timeout=0.6)
        plan={"viable":True,"algorithm":"orca3d",
            "command_type":"per_vehicle_velocity","contract_version":"orca_velocity_v1","valid_for_s":0.6,"stamp":10.0,
            "constraints_satisfied":True,"static_validation":{"feasible":True},"commands":[{"vehicle_id":"uav0","velocity":[4,0,2],"preferred_velocity":[1,0,0]},
                        {"vehicle_id":"uav1","velocity":[0,-4,-2],"preferred_velocity":[0,-1,0]}]}
        values=gate.condition(plan,10.2,0.2)
        self.assertEqual(len(values),2)
        self.assertLessEqual(np.linalg.norm(np.asarray(values[0])-np.asarray([1.0,0.0,0.0])),0.1001)
        self.assertLessEqual(abs(values[0][2]),0.1001)

    def test_orca_command_gate_rejects_stale_or_incomplete_plan(self):
        gate=OrcaCommandGate(2,timeout=0.5)
        plan={"viable":True,"algorithm":"orca3d",
            "command_type":"per_vehicle_velocity","contract_version":"orca_velocity_v1","valid_for_s":0.6,"stamp":1.0,
            "constraints_satisfied":True,"static_validation":{"feasible":True},"commands":[{"vehicle_id":"uav0","velocity":[0,0,0],"preferred_velocity":[0,0,0]}]}
        with self.assertRaises(DynamicObstacleError):gate.condition(plan,1.1,0.1)
        plan["commands"].append({"vehicle_id":"uav1","velocity":[0,0,0],"preferred_velocity":[0,0,0]})
        with self.assertRaises(DynamicObstacleError):gate.condition(plan,2.0,0.1)

    def test_orca_static_validation_rejects_boundary_escape(self):
        result=Orca3DPlanner().plan([[[0,45,0,8],[4,50,0,8]]],[],scene_id=0,max_speed=3.0)
        self.assertFalse(result["viable"])
        self.assertIn("E_BOUNDARY",result["rejection_summary"])

    def test_orca_velocity_to_position_target(self):
        targets=orca_position_targets([(1,2,3),(0,0,4)],[(2,0,-1),(0,1,0)],0.5)
        self.assertEqual(targets,[(2.0,2.0,2.5),(0.0,0.5,4.0)])
        with self.assertRaises(DynamicObstacleError):orca_position_targets([(0,0,0)],[],0.5)

    def test_collective_adapter_preserves_legacy_contract(self):
        result=CollectiveOffsetPlanner().plan(self.paths(),[],candidate_offsets=[[0,0,0]])
        self.assertTrue(result["viable"]);self.assertEqual(result["algorithm"],"collective_offset")
        self.assertEqual(result["command_type"],"collective_offset")

if __name__=="__main__":unittest.main()
