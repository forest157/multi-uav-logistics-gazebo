import unittest

from logistics_gazebo_sim.safety_interlock import FleetSeparationInterlock,diagnostic_error_present,guarded_collective_targets


class SafetyInterlockTest(unittest.TestCase):
    def test_close_vehicles_latch_hold_until_stable_release(self):
        guard=FleetSeparationInterlock(2.7,3.0,1.0)
        self.assertTrue(guard.update([(0,0,5),(2.6,0,5)],0.0)["hold"])
        self.assertTrue(guard.update([(0,0,5),(3.1,0,5)],.5)["hold"])
        self.assertTrue(guard.update([(0,0,5),(3.1,0,5)],1.4)["hold"])
        self.assertFalse(guard.update([(0,0,5),(3.1,0,5)],1.5)["hold"])

    def test_release_timer_resets_inside_hysteresis_band(self):
        guard=FleetSeparationInterlock(2.7,3.0,1.0)
        guard.update([(0,0,0),(2.0,0,0)],0.0)
        guard.update([(0,0,0),(3.1,0,0)],1.0)
        guard.update([(0,0,0),(2.9,0,0)],1.5)
        self.assertTrue(guard.update([(0,0,0),(3.1,0,0)],2.1)["hold"])

    def test_external_clearance_error_latches_hold(self):
        guard=FleetSeparationInterlock()
        value=guard.update([(0,0,5),(4,0,5)],0.0,external_error=True)
        self.assertTrue(value["hold"]);self.assertIn("diagnostic",value["reason"])

    def test_stale_diagnostic_is_not_a_safety_error(self):
        self.assertFalse(diagnostic_error_present([0,1,3]))
        self.assertTrue(diagnostic_error_present([0,2,3]))

    def test_empty_air_cannot_apply_stale_vertical_offset(self):
        targets=[(0,0,5),(3.3,0,5)]
        command={"state":"ACTIVE","action":"AVOID","offset":[0,0,3]}
        self.assertEqual(guarded_collective_targets(targets,command,0),targets)

    def test_obstacle_offset_and_smooth_recovery_are_preserved(self):
        targets=[(0,0,5)]
        active={"state":"ACTIVE","action":"AVOID","offset":[0,2,3]}
        recovery={"state":"RECOVERING","action":"AVOID","offset":[0,1,.5]}
        self.assertEqual(guarded_collective_targets(targets,active,1),[(0,2,8)])
        self.assertEqual(guarded_collective_targets(targets,recovery,0),[(0,1,5.5)])

    def test_invalid_limits_and_offsets_fail_closed(self):
        with self.assertRaises(ValueError):FleetSeparationInterlock(3.0,2.0)
        with self.assertRaises(ValueError):guarded_collective_targets([(0,0,5)],{"action":"AVOID","state":"ACTIVE","offset":[0,float("nan"),0]},1)


if __name__=="__main__":unittest.main()
