import importlib.machinery
import json
import os
import unittest
from unittest.mock import Mock, patch
from std_msgs.msg import String
from logistics_gazebo_sim.dynamic_obstacles import DynamicSafetyResponse, DynamicObstacleError

SCRIPT=os.path.join(os.path.dirname(__file__),"..","scripts","fleet_mission_player")
player=importlib.machinery.SourceFileLoader("orca_test_player",SCRIPT).load_module()


class OrcaExecutionTest(unittest.TestCase):
    def test_safe_risk_does_not_jump_back_to_distant_nominal_target(self):
        p=self.instance();p.orca_recovering=True
        self.call(p,"SAFE",context=dict(stamp=10.,nominal_errors=[5.]))
        self.assertEqual(p.dynamic_action,"ORCA");self.assertTrue(p.orca_recovering)
        p.orca_gate.condition.reset_mock()
        self.call(p,"SAFE",context=dict(stamp=10.,nominal_errors=[.2]))
        self.assertEqual(p.dynamic_action,"NORMAL");self.assertFalse(p.orca_recovering)
        p.orca_gate.condition.assert_not_called()

    def instance(self):
        p=player.MissionPlayer.__new__(player.MissionPlayer)
        p.dynamic_safety_enabled=p.dynamic_avoidance_enabled=True
        p.local_avoidance_algorithm="orca3d";p.orca_control_mode="limited"
        p.dynamic_response=DynamicSafetyResponse();p.last_orca_update=None
        p.current_poses=[(0,0,8)];p.dynamic_hold_targets=None
        p.orca_gate=Mock();p.orca_gate.condition.return_value=[(1,0,0)]
        p.orca_gate.refresh.side_effect=DynamicObstacleError("no cached command")
        p.orca_plan_stamp=None;p.orca_failure=None;p.orca_recovering=False
        return p

    def call(self,p,level,plan=None,context=None):
        payload=dict(level=level,obstacle_count=1,avoidance=plan or dict(stamp=10.,viable=True),
                     execution_context=context)
        with patch.object(player.rospy,"get_time",return_value=10.),patch.object(player.rospy,"logwarn"):
            p._dynamic_risk_cb(String(data=json.dumps(payload)))

    def test_critical_can_continue_only_after_execution_validation(self):
        p=self.instance();self.call(p,"CRITICAL",context=dict(stamp=10.))
        self.assertEqual(p.dynamic_action,"ORCA");self.assertIsNone(p.dynamic_hold_targets)
        self.assertEqual(p.orca_gate.condition.call_args.kwargs["safety_context"],dict(stamp=10.))

    def test_critical_rejected_trajectory_holds(self):
        p=self.instance();p.orca_gate.condition.side_effect=DynamicObstacleError("unsafe")
        self.call(p,"CRITICAL",context=dict(stamp=10.))
        self.assertEqual(p.dynamic_action,"HOLD");self.assertEqual(p.dynamic_hold_targets,p.current_poses)

    def test_missing_context_cannot_execute(self):
        p=self.instance();self.call(p,"WARNING")
        self.assertEqual(p.dynamic_action,"HOLD");p.orca_gate.condition.assert_not_called()

    def test_pending_critical_holds_but_warning_slows(self):
        for level,action in (("CRITICAL","HOLD"),("WARNING","SLOW")):
            p=self.instance();self.call(p,level,dict(rejection_summary=dict(PLANNER_PENDING=1)))
            self.assertEqual(p.dynamic_action,action);p.orca_gate.condition.assert_not_called()

    def test_pending_plan_keeps_only_a_revalidated_cached_target(self):
        p=self.instance();p.orca_gate.refresh.side_effect=None
        p.orca_gate.refresh.return_value=[(.8,.2,0.)];p.orca_plan_stamp=9.8
        context=dict(stamp=10.)
        self.call(p,"CRITICAL",dict(rejection_summary=dict(PLANNER_PENDING=1)),context)
        self.assertEqual(p.dynamic_action,"ORCA")
        self.assertEqual(p.orca_velocities,[(.8,.2,0.)])
        p.orca_gate.refresh.assert_called_once_with(context,10.)

    def test_replanning_does_not_publish_a_transient_slow_state(self):
        p=self.instance();p.dynamic_action="ORCA";p.dynamic_scale=.35;p.orca_recovering=True
        def condition(*args,**kwargs):
            self.assertEqual(p.dynamic_action,"ORCA")
            return [(1,0,0)]
        p.orca_gate.condition.side_effect=condition
        self.call(p,"WARNING",context=dict(stamp=10.,nominal_errors=[2.]))
        self.assertEqual(p.dynamic_action,"ORCA")
