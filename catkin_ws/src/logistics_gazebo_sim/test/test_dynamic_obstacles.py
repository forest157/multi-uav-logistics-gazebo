#!/usr/bin/env python3
import unittest

import numpy as np

from logistics_gazebo_sim.dynamic_obstacles import (
    DynamicObstacleError, DynamicSafetyResponse, AvoidanceExecution, assess_fleet_separation, assess_timed_path, interpolate_timed_path,
    obstacle_clearance, plan_collective_avoidance, minimum_spawn_clearance, prediction_path, predict_position, shifted_path, validate_static_paths)


class DynamicObstacleTest(unittest.TestCase):
    def obstacle(self, position=(5.0, -5.0, 5.0), velocity=(0.0, 1.0, 0.0)):
        return {"id": "crossing_1", "position": position,
                "velocity": velocity, "radius": 0.8, "height": 2.0}

    def test_constant_velocity_prediction(self):
        predicted = predict_position(self.obstacle(), 3.0)
        np.testing.assert_allclose(predicted, [5.0, -2.0, 5.0])

    def test_timed_path_interpolation(self):
        path = [[0.0, 0.0, 5.0, 5.0], [10.0, 10.0, 0.0, 5.0]]
        np.testing.assert_allclose(interpolate_timed_path(path, 2.5), [2.5, 3.75, 5.0])

    def test_timed_path_interpolation_accepts_numpy_scalar_query(self):
        path = [[0.0, 1.0, 2.0, 3.0], [4.0, 5.0, 6.0, 7.0],
                [8.0, 9.0, 10.0, 11.0]]
        np.testing.assert_allclose(
            interpolate_timed_path(path, np.float64(6.0)), [7.0, 8.0, 9.0])


    def test_crossing_trajectory_is_critical(self):
        path = [[0.0, 0.0, 0.0, 5.0], [10.0, 10.0, 0.0, 5.0]]
        report = assess_timed_path(path, [self.obstacle()], horizon=10.0)
        self.assertEqual(report["level"], "CRITICAL")
        self.assertEqual(report["obstacle_id"], "crossing_1")
        self.assertIsNotNone(report["time_to_conflict_s"])

    def test_vertical_separation_is_safe(self):
        path = [[0.0, 0.0, 12.0, 12.0], [10.0, 10.0, 12.0, 12.0]]
        report = assess_timed_path(path, [self.obstacle()], horizon=10.0)
        self.assertEqual(report["level"], "SAFE")
        self.assertGreater(report["minimum_clearance_m"], 2.0)

    def test_signed_clearance_detects_overlap(self):
        value = obstacle_clearance([5.0, 0.0, 5.0], self.obstacle(), 5.0)
        self.assertLess(value, 0.0)

    def test_invalid_obstacle_reports_input_error(self):
        with self.assertRaises(DynamicObstacleError):
            predict_position({"id": "broken"}, 1.0)

    def test_warning_scales_whole_fleet_clock(self):
        response = DynamicSafetyResponse(warning_scale=0.3)
        value = response.update("WARNING", 1.0)
        self.assertEqual(value["action"], "SLOW")
        self.assertAlmostEqual(value["speed_scale"], 0.3)

    def test_critical_hold_requires_stable_safe_release(self):
        response = DynamicSafetyResponse(release_delay=2.0)
        self.assertEqual(response.update("CRITICAL", 1.0)["action"], "HOLD")
        self.assertEqual(response.update("SAFE", 2.0)["action"], "HOLD")
        self.assertEqual(response.update("WARNING", 2.5)["action"], "HOLD")
        self.assertEqual(response.update("SAFE", 3.0)["action"], "HOLD")
        self.assertEqual(response.update("SAFE", 5.0)["action"], "NORMAL")

    def test_stale_does_not_create_new_hold(self):
        response = DynamicSafetyResponse()
        value = response.update("STALE", 1.0)
        self.assertEqual(value["action"], "NORMAL")
        self.assertEqual(value["speed_scale"], 1.0)


    def fleet_paths(self):
        return [[[0.0,0.0,y,5.0],[10.0,10.0,y,5.0]] for y in (-3.0,0.0,3.0)]

    def test_collective_offset_preserves_formation_distances(self):
        paths=self.fleet_paths();shifted=[shifted_path(path,[2.0,-1.0,4.0]) for path in paths]
        before=np.linalg.norm(np.asarray(paths[0])[-1,1:]-np.asarray(paths[1])[-1,1:])
        after=np.linalg.norm(shifted[0][-1,1:]-shifted[1][-1,1:])
        self.assertAlmostEqual(before,after)

    def test_collective_planner_selects_safe_vertical_candidate(self):
        result=plan_collective_avoidance(self.fleet_paths(),[self.obstacle()],candidate_offsets=[[0.0,0.0,0.0],[0.0,0.0,6.0]],horizon=10.0)
        self.assertTrue(result["viable"]);self.assertEqual(result["selected_offset"],[0.0,0.0,6.0])
        self.assertGreaterEqual(result["minimum_clearance_m"],0.5)

    def test_collective_planner_rejects_all_unsafe_candidates(self):
        obstacle=self.obstacle();obstacle["radius"]=30.0;obstacle["height"]=30.0
        result=plan_collective_avoidance(self.fleet_paths(),[obstacle],candidate_offsets=[[0.0,0.0,0.0],[0.0,0.0,3.0]],horizon=10.0)
        self.assertFalse(result["viable"]);self.assertIsNone(result["selected_offset"])
        self.assertIn("no collective offset",result["reason"])

    def test_individual_vehicle_may_use_space_outside_formation_center_limit(self):
        report=validate_static_paths(0,[[[0.0,47.5,45.0,8.0],[2.0,48.0,45.0,8.0]]])
        self.assertTrue(report["feasible"])


    def test_static_boundary_rejects_collective_candidate(self):
        paths=[[[0.0,44.0,0.0,8.0],[10.0,45.0,0.0,8.0]]]
        result=plan_collective_avoidance(paths,[],candidate_offsets=[[6.0,0.0,0.0]],scene_id=0)
        self.assertFalse(result["viable"]);self.assertEqual(result["rejection_summary"].get("E_BOUNDARY"),1)

    def test_static_height_rejects_collective_candidate(self):
        paths=[[[0.0,-40.0,-40.0,4.0],[10.0,-35.0,-40.0,4.0]]]
        result=plan_collective_avoidance(paths,[],candidate_offsets=[[0.0,0.0,-5.0]],scene_id=0)
        self.assertFalse(result["viable"]);self.assertEqual(result["rejection_summary"].get("E_VERTICAL_CLEARANCE"),1)

    def test_static_building_rejects_collective_candidate(self):
        paths=[[[0.0,10.0,-15.0,8.0],[10.0,20.0,-15.0,8.0]]]
        result=plan_collective_avoidance(paths,[],candidate_offsets=[[0.0,0.0,0.0]],scene_id=0)
        self.assertFalse(result["viable"]);self.assertEqual(result["rejection_summary"].get("E_CORRIDOR_TOO_NARROW"),1)

    def test_static_safe_candidate_is_accepted(self):
        paths=[[[0.0,-40.0,-40.0,8.0],[10.0,-35.0,-40.0,8.0]]]
        result=plan_collective_avoidance(paths,[],candidate_offsets=[[0.0,0.0,0.0]],scene_id=0)
        self.assertTrue(result["viable"]);self.assertTrue(result["static_validation"]["feasible"])

    def viable_avoidance(self,offset=(0.0,3.0,0.0)):
        return {"viable":True,"selected_offset":list(offset),"static_validation":{"feasible":True}}

    def test_avoidance_requires_confirmation_and_blends_offset(self):
        execution=AvoidanceExecution(confirmation_s=1.0,apply_s=2.0,recover_s=2.0)
        self.assertEqual(execution.update("WARNING",self.viable_avoidance(),0.0)["state"],"CONFIRMING")
        self.assertEqual(execution.update("WARNING",self.viable_avoidance(),1.0)["state"],"APPLYING")
        middle=execution.command(2.0);self.assertEqual(middle["action"],"AVOID")
        self.assertAlmostEqual(middle["offset"][1],1.5,places=2)
        self.assertEqual(execution.command(3.0)["state"],"ACTIVE")

    def test_avoidance_recovers_smoothly_after_safe(self):
        execution=AvoidanceExecution(confirmation_s=0.0,apply_s=1.0,recover_s=2.0)
        execution.update("WARNING",self.viable_avoidance(),0.0);execution.update("WARNING",self.viable_avoidance(),0.0)
        execution.command(1.0);self.assertEqual(execution.command(1.0)["state"],"ACTIVE")
        self.assertEqual(execution.update("SAFE",{},1.0)["state"],"RECOVERING")
        self.assertAlmostEqual(execution.command(2.0)["offset"][1],1.5,places=2)
        self.assertEqual(execution.command(3.0)["state"],"IDLE")

    def test_invalid_or_changed_candidate_falls_back_to_hold(self):
        execution=AvoidanceExecution(confirmation_s=0.0,apply_s=1.0)
        self.assertEqual(execution.update("CRITICAL",{"viable":False},0.0)["action"],"HOLD")
        execution.reset();execution.update("WARNING",self.viable_avoidance(),0.0)
        execution.update("WARNING",self.viable_avoidance(),0.0)
        changed=execution.update("WARNING",self.viable_avoidance((0.0,-3.0,0.0)),0.2)
        self.assertEqual(changed["action"],"HOLD")

    def test_stale_during_active_avoidance_falls_back_to_hold(self):
        execution=AvoidanceExecution(confirmation_s=0.0,apply_s=0.1)
        execution.update("WARNING",self.viable_avoidance(),0.0);execution.update("WARNING",self.viable_avoidance(),0.0)
        execution.command(0.1);self.assertEqual(execution.command(0.1)["state"],"ACTIVE")
        self.assertEqual(execution.update("STALE",{},0.2)["action"],"HOLD")

    def test_stationary_target_keeps_full_prediction_horizon(self):
        path=prediction_path([1.0,2.0,3.0],[1.0,2.0,3.0],8.0)
        self.assertEqual(path[0][0],0.0);self.assertEqual(path[-1][0],8.0)
        self.assertEqual(path[0][1:],path[-1][1:])

    def test_spawn_clearance_detects_direct_overlap(self):
        self.assertAlmostEqual(minimum_spawn_clearance([0,0,5],[[0,0,5],[10,0,5]]),0.0)
        self.assertGreater(minimum_spawn_clearance([0,0,5],[[10,0,5]]),5.0)

    def test_fleet_separation_detects_between_waypoint_crossing(self):
        paths=[
            [[0.0,-5.0,0.0,5.0],[10.0,5.0,0.0,5.0]],
            [[0.0,5.0,0.0,5.0],[10.0,-5.0,0.0,5.0]],
        ]
        report=assess_fleet_separation(
            paths,horizon=10.0,sample_period=0.1,minimum_separation=3.0)
        self.assertFalse(report["safe"])
        self.assertEqual(report["closest_pair"],["uav0","uav1"])
        self.assertAlmostEqual(report["minimum_separation_m"],0.0)

    def test_collective_planner_rejects_vehicle_crossing(self):
        paths=[
            [[0.0,-5.0,0.0,5.0],[10.0,5.0,0.0,5.0]],
            [[0.0,5.0,0.0,5.0],[10.0,-5.0,0.0,5.0]],
        ]
        result=plan_collective_avoidance(
            paths,[],candidate_offsets=[[0.0,3.0,0.0]],horizon=10.0)
        self.assertFalse(result["viable"])
        self.assertEqual(result["rejection_summary"]["VEHICLE_SEPARATION"],1)

    def test_fleet_separation_allows_small_tracking_tolerance(self):
        paths=[
            [[0.0,0.0,0.0,5.0],[8.0,8.0,0.0,5.0]],
            [[0.0,0.0,2.98,5.0],[8.0,8.0,2.98,5.0]],
        ]
        report=assess_fleet_separation(
            paths,minimum_separation=3.0,tracking_tolerance=0.05)
        self.assertTrue(report["safe"])
        self.assertAlmostEqual(report["minimum_separation_m"],2.98)

    def test_fleet_separation_rejects_beyond_tracking_tolerance(self):
        paths=[
            [[0.0,0.0,0.0,5.0],[8.0,8.0,0.0,5.0]],
            [[0.0,0.0,2.9,5.0],[8.0,8.0,2.9,5.0]],
        ]
        report=assess_fleet_separation(
            paths,minimum_separation=3.0,tracking_tolerance=0.05)
        self.assertFalse(report["safe"])


if __name__ == "__main__":
    unittest.main()
