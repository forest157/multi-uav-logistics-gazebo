#!/usr/bin/env python3
from pathlib import Path
import unittest
import numpy as np
from logistics_gazebo_sim.local_avoidance import (
    CollectiveOffsetPlanner, Orca3DPlanner, DistributedMpcPlanner, OrcaCommandGate, orca_position_targets, available_local_planners,
    create_local_planner, validate_orca_execution)
from logistics_gazebo_sim.dynamic_obstacles import DynamicObstacleError

class LocalAvoidanceTest(unittest.TestCase):
    def test_risk_monitor_forwards_required_clearance_to_planner_and_gate(self):
        source=(Path(__file__).resolve().parents[1]/"scripts"/"dynamic_risk_monitor").read_text()
        self.assertIn('"required_clearance":self.orca_required_clearance',source)
        self.assertIn("required_clearance=self.orca_required_clearance",source)

    def test_complete_acceleration_ramp_is_validated_before_first_slew_step(self):
        gate=OrcaCommandGate(1)
        bird=dict(id="bird",position=[0,12,8],velocity=[0,-3,0],radius=.75,height=1.5)
        context=self.execution_context(position=(-3,0,8),velocity=(1.4,0,0),obstacles=[bird])
        plan=dict(viable=True,algorithm="orca3d",command_type="per_vehicle_velocity",
                  contract_version="orca_velocity_v1",stamp=10.,valid_for_s=.6,
                  constraints_satisfied=True,static_validation=dict(feasible=True),
                  commands=[dict(vehicle_id="uav0",velocity=[2,0,.6],preferred_velocity=[1.4,0,0])])
        result=gate.condition(plan,10.,.2,safety_context=context)
        self.assertEqual(result,[(1.4,0.,0.)])
        self.assertTrue(validate_orca_execution(gate.target_velocities,context,10.))
        self.assertEqual(gate.target_velocities[0][2],0.)
        advanced=gate.current(10.2)
        self.assertAlmostEqual(advanced[0][0],1.6)
        self.assertEqual(advanced[0][2],0.)

    def test_gate_refreshes_cached_target_and_expires_fail_closed(self):
        gate=OrcaCommandGate(1,timeout=.6)
        context=self.execution_context(velocity=(0,0,0))
        plan=dict(viable=True,algorithm="orca3d",command_type="per_vehicle_velocity",
                  contract_version="orca_velocity_v1",stamp=10.,valid_for_s=.6,
                  constraints_satisfied=True,static_validation=dict(feasible=True),
                  commands=[dict(vehicle_id="uav0",velocity=[1,0,0],preferred_velocity=[0,0,0])])
        gate.condition(plan,10.,.2,safety_context=context)
        self.assertAlmostEqual(gate.current(10.2)[0][0],.2)
        context["stamp"]=10.5
        self.assertAlmostEqual(gate.refresh(context,10.5)[0][0],.5)
        with self.assertRaisesRegex(DynamicObstacleError,"expired"):
            gate.current(11.11)

    def test_clear_recovery_allows_validated_vertical_return_to_route(self):
        gate=OrcaCommandGate(1,max_climb_rate=.8)
        context=self.execution_context(velocity=(0,0,0))
        context["nominal_errors"]=[5.]
        plan=dict(viable=False,algorithm="orca3d",command_type="per_vehicle_velocity",
                  contract_version="orca_velocity_v1",stamp=10.,valid_for_s=.6,
                  constraints_satisfied=False,static_validation=dict(feasible=False),
                  commands=[dict(vehicle_id="uav0",velocity=[.2,.1,-2.],preferred_velocity=[.2,.1,-2.])])
        gate.condition(plan,10.,.2,safety_context=context)
        self.assertAlmostEqual(gate.target_velocities[0][2],-.8)

    def execution_context(self,position=(0,0,8),velocity=(0,0,0),obstacles=None):
        return dict(stamp=10.,positions=[position],velocities=[velocity],
                    obstacles=obstacles or [],horizon=8.,control_delay=.6)

    def test_execution_rejects_hold_in_incoming_bird_path(self):
        bird=dict(id="bird",position=[0,12,8],velocity=[0,-3,0],radius=.75,height=1.5)
        context=self.execution_context(obstacles=[bird])
        self.assertFalse(validate_orca_execution([(0,0,0)],context,10.))
        self.assertTrue(validate_orca_execution([(2,0,0)],context,10.))

    def test_execution_accounts_for_control_delay(self):
        bird=dict(id="bird",position=[0,4,8],velocity=[0,-3,0],radius=.75,height=1.5)
        self.assertFalse(validate_orca_execution([(2,0,0)],self.execution_context(obstacles=[bird]),10.))

    def test_execution_uses_configured_dynamic_clearance(self):
        bird=dict(id="bird",position=[2.7,0,8],velocity=[0,0,0],radius=.75,height=.8)
        context=self.execution_context(obstacles=[bird])
        self.assertTrue(validate_orca_execution([(0,0,0)],context,10.))
        context["required_clearance"]=.5
        self.assertFalse(validate_orca_execution([(0,0,0)],context,10.))
        context["required_clearance"]=-.1
        with self.assertRaisesRegex(DynamicObstacleError,"required clearance"):
            validate_orca_execution([(0,0,0)],context,10.)

    def test_execution_rejects_stale_context_and_pair_crossing(self):
        context=self.execution_context()
        with self.assertRaises(DynamicObstacleError):validate_orca_execution([(0,0,0)],context,11.)
        context.update(positions=[[-4,0,8],[4,0,8]],velocities=[[1,0,0],[-1,0,0]])
        self.assertFalse(validate_orca_execution([(1,0,0),(-1,0,0)],context,10.))

    def test_gate_rechecks_filtered_command_without_mutating_on_rejection(self):
        gate=OrcaCommandGate(1)
        bird=dict(id="bird",position=[0,8,8],velocity=[0,-3,0],radius=.75,height=1.5)
        plan=dict(viable=True,algorithm="orca3d",command_type="per_vehicle_velocity",
                  contract_version="orca_velocity_v1",stamp=10.,valid_for_s=.6,
                  constraints_satisfied=True,static_validation=dict(feasible=True),
                  commands=[dict(vehicle_id="uav0",velocity=[2,0,0],preferred_velocity=[0,0,0])])
        with self.assertRaisesRegex(DynamicObstacleError,"conditioned ORCA"):
            gate.condition(plan,10.,.2,safety_context=self.execution_context(obstacles=[bird]))
        self.assertIsNone(gate.current_velocities)
        self.assertIsNone(gate.target_velocities)

    def test_gate_finds_split_horizontal_escape_for_crossing_fleet_threat(self):
        gate=OrcaCommandGate(3)
        context=dict(stamp=10.,
            positions=[[-20.6039,-18.3025,16.8675],
                       [-17.6279,-12.3581,16.8797],
                       [-14.5739,-18.3442,16.8472]],
            velocities=[[-.5516,.036,0.],[-.5846,.0011,0.],[-.4311,.0425,0.]],
            obstacles=[dict(id="bird",position=[-12.2499,-6.6273,16.1953],
                velocity=[-2.1975,-2.0545,.101],radius=.794,height=1.407)],
            horizon=8.,control_delay=.6,minimum_separation=3.,
            required_clearance=.5,scene_id=0)
        desired=[np.zeros(3) for _ in range(3)]
        selected=gate._safe_target(desired,context,10.)
        self.assertTrue(validate_orca_execution(selected,context,10.))
        self.assertTrue(all(value[2]==0. for value in selected))
        self.assertFalse(np.allclose(selected[0],selected[1]))

    def paths(self,count=3):
        middle=(count-1)*0.5
        return [[[0.0,0.0,(i-middle)*4.0,8.0],[5.0,10.0,(i-middle)*4.0,8.0]]
                for i in range(count)]
    def test_registry_keeps_stable_planner_and_adds_orca(self):
        self.assertEqual(available_local_planners(),("collective_offset","distributed_mpc","orca3d"))
        self.assertIsInstance(create_local_planner("collective_offset"),CollectiveOffsetPlanner)
        self.assertIsInstance(create_local_planner("orca3d"),Orca3DPlanner)
        self.assertIsInstance(create_local_planner("distributed_mpc"),DistributedMpcPlanner)
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

    def test_distributed_mpc_generates_safe_shadow_trajectories(self):
        paths=self.paths(3)
        obstacle={"id":"bird","position":[5,0,8],"velocity":[0,0,0],"radius":0.5,"height":0.8}
        result=DistributedMpcPlanner().plan(paths,[obstacle],mpc_steps=6,mpc_dt=0.5,mpc_max_iterations=55)
        self.assertTrue(result["viable"]);self.assertTrue(result["shadow_mode"])
        self.assertEqual(result["command_type"],"per_vehicle_trajectory")
        self.assertEqual(len(result["trajectories"]),3)
        self.assertTrue(result["fleet_separation"]["safe"])
        self.assertLess(result["solve_time_ms"]["total"],1500.0)

    def test_distributed_mpc_warm_start_reuses_safe_solution(self):
        planner=DistributedMpcPlanner();obstacle={"id":"bird","position":[5,0,8],"velocity":[0,0,0],"radius":0.5,"height":0.8}
        first=planner.plan(self.paths(3),[obstacle],mpc_steps=6,mpc_dt=0.5,mpc_max_iterations=55)
        second=planner.plan(self.paths(3),[obstacle],mpc_steps=6,mpc_dt=0.5,mpc_max_iterations=55)
        self.assertTrue(first["viable"]);self.assertTrue(second["viable"])
        self.assertEqual(first["warm_started_vehicle_count"],0);self.assertGreater(second["warm_started_vehicle_count"],0)


    def test_distributed_mpc_consensus_preserves_nominal_offsets(self):
        result=DistributedMpcPlanner().plan(self.paths(3),[],mpc_consensus_strength=1.0)
        self.assertTrue(result["viable"]);self.assertEqual(result["consensus_strength"],1.0)
        trajectories=np.asarray(result["trajectories"],dtype=float)[:,:,1:]
        relative=trajectories[1]-trajectories[0]
        np.testing.assert_allclose(relative,np.tile(relative[0],(len(relative),1)),atol=2e-4)
        self.assertGreaterEqual(result["fleet_separation"]["minimum_separation_m"],3.99)
        with self.assertRaises(DynamicObstacleError):DistributedMpcPlanner().plan(self.paths(3),[],mpc_consensus_strength=1.1)


    def test_distributed_mpc_distinguishes_unsafe_nominal_spacing(self):
        paths=[[[0,0,0,8],[4,8,0,8]],[[0,0,2.5,8],[4,8,2.5,8]]]
        result=DistributedMpcPlanner().plan(paths,[],minimum_separation=3.0,mpc_consensus_strength=1.0)
        self.assertFalse(result["viable"]);self.assertFalse(result["nominal_fleet_separation"]["safe"])
        self.assertIn("NOMINAL_VEHICLE_SEPARATION",result["rejection_summary"])
        self.assertNotIn("VEHICLE_SEPARATION",result["rejection_summary"])


    def test_distributed_mpc_timeout_is_contained(self):
        result=DistributedMpcPlanner().plan(self.paths(3),[],mpc_vehicle_timeout_s=0.0001)
        self.assertFalse(result["viable"]);self.assertTrue(result["solver_isolated"])
        self.assertIn("MPC_SOLVER_TIMEOUT",result["rejection_summary"])


    def test_distributed_mpc_rejects_physically_late_avoidance(self):
        obstacle={"id":"bird","position":[1,0,8],"velocity":[0,0,0],"radius":1.0,"height":1.0}
        result=DistributedMpcPlanner().plan([[[0,0,0,8],[3,6,0,8]]],[obstacle],mpc_steps=4,mpc_dt=0.4)
        self.assertFalse(result["viable"]);self.assertIn("DYNAMIC_CLEARANCE",result["rejection_summary"])

    def test_collective_adapter_preserves_legacy_contract(self):
        result=CollectiveOffsetPlanner().plan(self.paths(),[],candidate_offsets=[[0,0,0]])
        self.assertTrue(result["viable"]);self.assertEqual(result["algorithm"],"collective_offset")
        self.assertEqual(result["command_type"],"collective_offset")

if __name__=="__main__":unittest.main()
