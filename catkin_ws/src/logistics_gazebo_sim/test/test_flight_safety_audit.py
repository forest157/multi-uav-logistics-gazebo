import copy
import importlib.util
from pathlib import Path
import unittest

SCRIPT=Path(__file__).resolve().parents[4]/"reference/gazebo_3d_ros_package_experiment/v0511_real_scale_worlds/analyze_flight_safety.py"
spec=importlib.util.spec_from_file_location("flight_audit",str(SCRIPT))
audit=importlib.util.module_from_spec(spec);spec.loader.exec_module(audit)


class FlightAuditTest(unittest.TestCase):
    def row(self):
        fleet={};poses={}
        for i in range(3):
            fleet["iris"+str(i)]=dict(position=[-40+(i-1)*3.3,-40,0.1],velocity=[0,0,0])
            poses[str(i)]=[10.,0.,0.,.1]
        fleet["bird_trial_0"]=dict(position=[0,0,8],velocity=[0,0,0])
        targets={str(i):[10.,0.,0.,.1] for i in range(3)}
        return dict(stamp=10.,truth_receipt_stamp=10.,truth_evaluation_only=fleet,poses=poses,
                    targets=targets,
                    mission=dict(state="COMPLETE",dynamic_action="NORMAL"),
                    flight_states={str(i):dict(armed=False) for i in range(3)})

    def test_complete_clear_flight_passes(self):
        report=audit.analyze([self.row()])
        self.assertTrue(report["passed"])
        self.assertAlmostEqual(report["minimum_fleet_separation_m"],3.3)

    def test_bird_overlap_fails_even_if_flight_estimates_look_safe(self):
        row=self.row()
        row["truth_evaluation_only"]["bird_trial_0"]["position"]=[-40,-40,0.1]
        self.assertIn("minimum_bird_envelope_clearance_m",audit.analyze([row])["failure_reasons"])

    def test_stale_truth_and_incomplete_mission_cannot_pass(self):
        row=self.row();row["truth_receipt_stamp"]=8.;row["mission"]["state"]="RUNNING"
        report=audit.analyze([row])
        self.assertFalse(report["passed"])
        self.assertIn("insufficient_truth_coverage",report["failure_reasons"])
        self.assertIn("mission_not_complete",report["failure_reasons"])

    def test_estimate_divergence_is_reported(self):
        row=self.row();row["poses"]["2"][1]=20.
        self.assertIn("maximum_estimate_error_m",audit.analyze([row])["failure_reasons"])

    def test_declared_no_bird_flight_requires_no_tracks_or_avoidance(self):
        row=self.row();del row["truth_evaluation_only"]["bird_trial_0"]
        row["tracks"]={"obstacles":[]}
        report=audit.analyze([row],require_bird=False)
        self.assertTrue(report["passed"])
        self.assertEqual(report["maximum_perception_track_count"],0)
        self.assertEqual(report["maximum_settled_vertical_tracking_error_m"],0.)
        row["tracks"]={"obstacles":[{"id":"false_track"}]}
        row["mission"]["state"]="RUNNING"
        row["mission"]["dynamic_action"]="ORCA"
        report=audit.analyze([row],require_bird=False)
        self.assertIn("unexpected_perception_tracks",report["failure_reasons"])
        self.assertIn("unexpected_avoidance_action",report["failure_reasons"])

    def test_declared_no_bird_flight_rejects_vertical_tracking_error(self):
        row=self.row();del row["truth_evaluation_only"]["bird_trial_0"]
        row["tracks"]={"obstacles":[]};drift=copy.deepcopy(row)
        drift["stamp"]=11.;drift["truth_receipt_stamp"]=11.
        drift["poses"]["1"]=[11.,0.,0.,2.]
        drift["targets"]["1"]=[11.,0.,0.,.1]
        report=audit.analyze([row,drift],require_bird=False)
        self.assertIn("maximum_settled_vertical_tracking_error_m",report["failure_reasons"])

    def test_bird_mode_rejects_missing_bird_truth(self):
        row=self.row();del row["truth_evaluation_only"]["bird_trial_0"]
        report=audit.analyze([row])
        self.assertIn("bird_truth_missing",report["failure_reasons"])
