import copy
import hashlib
import json
import tempfile
import unittest
from pathlib import Path

from logistics_gazebo_sim.series_acceptance import (
    audit_manifest, evaluate_dropout, evaluate_energy_and_safety, evaluate_flight,
    evaluate_perception_matrix)


def flight(capture, bird=False):
    report = dict(world='outdoor_campus', world_sha256='', capture=capture,
                  flight_samples=1000, valid_truth_samples=1000,
                  fresh_lidar_samples=1000, fresh_safety_samples=1000,
                  minimum_truth_separation_m=3.2,
                  minimum_truth_static_clearance_m=1.0,
                  maximum_estimate_error_m=0.5,
                  maximum_settled_vertical_error_m=0.5,
                  safety_error_samples=0, complete=True, disarmed=True,
                  passed=True, failure_reasons=[],
                  maximum_lidar_confirmed_targets=1 if bird else 0,
                  bird_truth_samples=100 if bird else 0,
                  unexpected_avoidance_samples=0)
    if bird:
        report.update(minimum_bird_clearance_m=0.8, active_orca_samples=20)
    return report


class SeriesAcceptanceTest(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name)
        world = self.root / 'catkin_ws/src/logistics_gazebo_sim/worlds/outdoor_campus.world'
        world.parent.mkdir(parents=True)
        world.write_text('world', encoding='utf-8')
        digest = hashlib.sha256(b'world').hexdigest()
        self.reports = [flight('run1'), flight('run2'), flight('bird', True)]
        for index, report in enumerate(self.reports):
            report['world_sha256'] = digest
            self.write('report{}.json'.format(index), report)
        self.write('limits.json', dict(minimum_real_time_factor=0.8,
                                       maximum_cpu_cores=8.0,
                                       maximum_full_stack_pss_mib=12288.0))
        self.write('resource.json', dict(real_time_factor=0.9,
                                         full_stack_cpu_cores=2.0,
                                         full_stack_pss_peak_sampled_mib=7000.0,
                                         sample_wall_s=30.0,
                                         acceptance=dict(minimum_real_time_factor=0.8,
                                                         maximum_cpu_cores=8.0,
                                                         maximum_full_stack_pss_mib=12288.0,
                                                         passed=True)))
        self.manifest = dict(release='v0.5.12', minimum_runs_per_world=2,
                             worlds={'outdoor_campus': dict(world_sha256=digest,
                                 no_bird_reports=['report0.json', 'report1.json'])},
                             single_bird=dict(world='outdoor_campus',
                                              report='report2.json'),
                             resource_budget=dict(limits='limits.json',
                                                  report='resource.json'))

    def write(self, name, value):
        (self.root / name).write_text(json.dumps(value), encoding='utf-8')

    def test_complete_independent_evidence_passes(self):
        result = audit_manifest(self.root, self.manifest)
        self.assertTrue(result['passed'], result['checks'])

    def test_duplicate_capture_cannot_count_as_repeat(self):
        self.reports[1]['capture'] = 'run1'
        self.write('report1.json', self.reports[1])
        result = audit_manifest(self.root, self.manifest)
        self.assertFalse(result['passed'])
        self.assertIn('duplicate capture', str(result['checks']))
        self.assertIn('only 1 valid independent runs', str(result['checks']))

    def test_missing_and_path_escape_fail_closed(self):
        self.manifest['worlds']['outdoor_campus']['no_bird_reports'][1] = 'missing.json'
        self.assertFalse(audit_manifest(self.root, self.manifest)['passed'])
        self.manifest['worlds']['outdoor_campus']['no_bird_reports'][1] = '../other.json'
        self.assertIn('escapes repository', str(audit_manifest(self.root, self.manifest)))

    def test_saved_pass_flag_does_not_override_bad_metrics(self):
        bad = copy.deepcopy(self.reports[0])
        bad['minimum_truth_separation_m'] = 2.9
        bad['fresh_lidar_samples'] = 0
        self.assertTrue(evaluate_flight(bad, 'no_bird'))
        self.write('report0.json', bad)
        self.assertFalse(audit_manifest(self.root, self.manifest)['passed'])

    def test_pending_check_and_resource_drift_block_release(self):
        self.manifest['pending_checks'] = ['energy']
        self.assertFalse(audit_manifest(self.root, self.manifest)['passed'])
        self.manifest['pending_checks'] = []
        resource = json.loads((self.root / 'resource.json').read_text())
        resource['acceptance']['maximum_cpu_cores'] = 80.0
        self.write('resource.json', resource)
        result = audit_manifest(self.root, self.manifest)
        self.assertFalse(result['passed'])
        self.assertIn('drift', str(result['checks']))

    def test_perception_matrix_requires_its_actual_invariants(self):
        cases = {
            'empty_air': {'pass': True, 'track_samples': 0},
            'single_target_noise': {'pass': True, 'unique_ids': ['a']},
            'short_occlusion': {'pass': True, 'before_id': 'a', 'after_id': 'a'},
            'long_occlusion': {'pass': True, 'before_id': 'a', 'after_id': 'b'},
            'two_target_crossing': {'pass': True, 'track_ids': ['a', 'b']},
            'perception_stale': {'pass': True,
                                 'actions': ['SLOW', 'HOLD', 'HOLD', 'NORMAL']},
        }
        report = {'pass': True, 'case_count': 6, 'cases': cases}
        self.assertEqual(evaluate_perception_matrix(report), [])
        cases['two_target_crossing']['track_ids'] = ['a', 'a']
        self.assertIn('two-target', str(evaluate_perception_matrix(report)))

    def test_dropout_requires_slow_hold_guard_and_recovery(self):
        report = dict(passed=True, complete=True, disarmed=True, samples=1000,
                      valid_truth_samples=1000, minimum_fleet_separation_m=3.2,
                      maximum_estimate_error_m=0.4,
                      maximum_settled_vertical_tracking_error_m=0.4,
                      dropout_slow_samples=10, dropout_hold_samples=10,
                      dropout_release_guard_samples=10,
                      bird_truth_samples=0, frames_with_perception_tracks=0,
                      unexpected_avoidance_samples=0, dropout_recovered=True)
        self.assertEqual(evaluate_dropout(report), [])
        report['dropout_hold_samples'] = 0
        self.assertIn('dropout_hold_samples', str(evaluate_dropout(report)))

    def test_landing_report_checks_real_target_order(self):
        report = dict(pass_=True, control_applied=True,
                      critical_diversion_observed=True,
                      home_descent_reached=True,
                      priority_order_observed=True,
                      target_altitudes=[8.0, 7.9, 0.18])
        report['pass'] = report.pop('pass_')
        self.assertEqual(evaluate_energy_and_safety('landing', report), [])
        report['target_altitudes'] = [8.0, 8.1, 0.18]
        self.assertIn('landing target order',
                      str(evaluate_energy_and_safety('landing', report)))

    def test_version_metadata_must_match_candidate(self):
        (self.root / 'package.xml').write_text(
            '<package><version>0.5.12</version></package>', encoding='utf-8')
        (self.root / 'setup.py').write_text("version='0.5.12'", encoding='utf-8')
        self.manifest['version_metadata'] = dict(package_xml='package.xml',
                                                 setup_py='setup.py')
        self.assertTrue(audit_manifest(self.root, self.manifest)['passed'])
        (self.root / 'setup.py').write_text("version='0.5.1'", encoding='utf-8')
        result = audit_manifest(self.root, self.manifest)
        self.assertFalse(result['passed'])
        self.assertIn('versions do not match', str(result['checks']))

    def test_release_draft_must_disclose_scope(self):
        self.manifest['release_document'] = 'draft.md'
        (self.root / 'draft.md').write_text(
            'v0.5.12 未发布；双鸟暂缓，Baylands 仅导入，净空为抽样。',
            encoding='utf-8')
        self.assertTrue(audit_manifest(self.root, self.manifest)['passed'])
        (self.root / 'draft.md').write_text('v0.5.12', encoding='utf-8')
        result = audit_manifest(self.root, self.manifest)
        self.assertFalse(result['passed'])
        self.assertIn('release draft omits', str(result['checks']))


if __name__ == '__main__':
    unittest.main()
