import unittest
from logistics_gazebo_sim.energy_return_policy import EnergyReturnPolicy,assign_return_slots,staggered_descent_progress


def report(stamp,margins,required=5.0):
    return {"stamp":stamp,"vehicles":[{"vehicle_id":"uav{}".format(i),"usable_margin_wh":margin+required,
        "required_to_land_wh":required} for i,margin in enumerate(margins)]}


class EnergyReturnPolicyTest(unittest.TestCase):
    def test_stale_or_missing_input_fails_safe_without_control(self):
        policy=EnergyReturnPolicy(stale_after_s=1.0)
        value=policy.update(report(1,[20]),3)
        self.assertEqual(value["fleet_level"],"STALE");self.assertEqual(value["action"],"HOLD_RECOMMENDED")
        self.assertFalse(value["control_applied"])
    def test_low_and_critical_are_based_on_final_margin(self):
        value=EnergyReturnPolicy().update(report(10,[30,10,-1]),10)
        self.assertEqual([v["level"] for v in value["vehicles"]],["NORMAL","LOW","CRITICAL"])
        self.assertEqual(value["action"],"ALTERNATE_LANDING_RECOMMENDED")
    def test_hysteresis_prevents_threshold_chatter(self):
        policy=EnergyReturnPolicy(low_margin_wh=12,release_hysteresis_wh=3)
        self.assertEqual(policy.update(report(1,[11]),1)["fleet_level"],"LOW")
        self.assertEqual(policy.update(report(2,[14]),2)["fleet_level"],"LOW")
        self.assertEqual(policy.update(report(3,[16]),3)["fleet_level"],"NORMAL")
    def test_escalation_from_low_to_critical_is_immediate(self):
        policy=EnergyReturnPolicy();self.assertEqual(policy.update(report(1,[5]),1)["fleet_level"],"LOW")
        self.assertEqual(policy.update(report(2,[-1]),2)["fleet_level"],"CRITICAL")
    def test_lowest_energy_gets_shortest_weighted_route(self):
        assessments=EnergyReturnPolicy().update(report(1,[2,40,50]),1)["vehicles"]
        slots=assign_return_slots(assessments,[(0,0,8),(0,0,8),(0,0,8)],[(20,0,0),(5,0,0),(10,0,0)],min_separation=0.0)
        self.assertEqual(slots["uav0"],1)
    def test_slots_only_recommended_in_open_return_airspace(self):
        policy=EnergyReturnPolicy();args=(report(1,[5,30]),1,[(0,0,8),(4,0,8)],[(10,0,0),(14,0,0)])
        self.assertEqual(policy.update(*args,phase="DELIVERY_DESCENT")["slot_assignments"],{})
        self.assertTrue(policy.update(*args,phase="RETURN")["slot_assignments"])
    def test_landing_order_prioritizes_low_margin(self):
        value=EnergyReturnPolicy().update(report(1,[20,-1,5]),1)
        self.assertEqual(value["landing_order"],["uav1","uav2","uav0"])
    def test_crossing_slot_exchange_is_rejected(self):
        assessments=EnergyReturnPolicy().update(report(1,[1,20]),1)["vehicles"]
        slots=assign_return_slots(assessments,[(-2,0,0),(2,0,0)],[(2,0,0),(-2,0,0)],min_separation=1.0)
        self.assertEqual(slots,{"uav0":1,"uav1":0})
    def test_staggered_descent_is_monotonic_and_energy_ordered(self):
        first=staggered_descent_progress(15,10,20,0,2);second=staggered_descent_progress(15,10,20,1,2)
        self.assertGreater(first,second);self.assertEqual(staggered_descent_progress(10,10,20,2,2),0.0)
        self.assertEqual(staggered_descent_progress(24,10,20,2,2),1.0)


if __name__=="__main__":unittest.main()
