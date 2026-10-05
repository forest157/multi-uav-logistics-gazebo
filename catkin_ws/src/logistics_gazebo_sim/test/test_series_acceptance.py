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
        digest = hashlib.sha256(b'world').hexdigest()
        self.reports = [flight('run1'), flight('run2'), flight('bird', True)]
        worlds = {}
        for name in ('outdoor_campus', 'outdoor_residential', 'outdoor_urban'):
            world = self.root / ('catkin_ws/src/logistics_gazebo_sim/worlds/' + name + '.world')
            world.parent.mkdir(parents=True, exist_ok=True)
            world.write_text('world', encoding='utf-8')
            if name != 'outdoor_campus':
                for capture in ('run1', 'run2'):
                    report = flight(name + capture)
                    report['world'] = name
                    self.reports.append(report)
            first = 0 if name == 'outdoor_campus' else (3 if name == 'outdoor_residential' else 5)
            worlds[name] = dict(world_sha256=digest,
                                no_bird_reports=['report{}.json'.format(first),
                                                 'report{}.json'.format(first + 1)])
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
        matrix = dict(pass_=True, case_count=6, cases={
            'empty_air': dict(pass_=True, track_samples=0),
            'single_target_noise': dict(pass_=True, unique_ids=['a']),
            'short_occlusion': dict(pass_=True, before_id='a', after_id='a'),
            'long_occlusion': dict(pass_=True, before_id='a', after_id='b'),
            'two_target_crossing': dict(pass_=True, track_ids=['a', 'b']),
            'perception_stale': dict(pass_=True, actions=['SLOW', 'HOLD', 'HOLD', 'NORMAL']),
        })
        matrix['pass'] = matrix.pop('pass_')
        for case in matrix['cases'].values():
            case['pass'] = case.pop('pass_')
        self.write('matrix.json', matrix)
        self.write('dropout.json', dict(passed=True, complete=True, disarmed=True,
                  samples=1000, valid_truth_samples=1000, minimum_fleet_separation_m=3.2,
                  maximum_estimate_error_m=0.4, maximum_settled_vertical_tracking_error_m=0.4,
                  dropout_slow_samples=1, dropout_hold_samples=1,
                  dropout_release_guard_samples=1, bird_truth_samples=0,
                  frames_with_perception_tracks=0, unexpected_avoidance_samples=0,
                  dropout_recovered=True))
        energy_fields = {
            'model': ('complete_requirement_zero', 'forecasts_present',
                      'mavros_battery_present', 'payload_released', 'used_energy_recorded'),
            'return': ('alternate_site_present', 'critical_alternate', 'low_return',
                       'multiple_critical_holds', 'safety_precedence', 'shadow_only',
                       'slots_present', 'stale_fails_safe'),
            'landing': ('control_applied', 'critical_diversion_observed',
                        'home_descent_reached', 'priority_order_observed'),
            'interlock': ('hold_triggered', 'released', 'targets_locked'),
        }
        for name, fields in energy_fields.items():
            evidence = {'pass': True}
            evidence.update({field: True for field in fields})
            if name == 'model':
                evidence.update(vehicle_count=3, capacities_wh=[1, 1, 1])
            elif name == 'landing':
                evidence['target_altitudes'] = [8.0, 7.0, 0.18]
            elif name == 'interlock':
                evidence['close_minimum_separation_m'] = 2.5
            self.write(name + '.json', evidence)
        (self.root / 'package.xml').write_text(
            '<package><version>0.5.12</version></package>', encoding='utf-8')
        (self.root / 'setup.py').write_text("version='0.5.12'", encoding='utf-8')
        (self.root / 'draft.md').write_text(
            'v0.5.12 发布；双鸟暂缓，Baylands 仅导入，物理净空为抽样。', encoding='utf-8')
        self.manifest = dict(release='v0.5.12', baseline_tag='v0.5.11',
                             minimum_runs_per_world=2, worlds=worlds,
                             single_bird=dict(world='outdoor_campus',
                                              report='report2.json'),
                             perception=dict(matrix='matrix.json', dropout='dropout.json'),
                             energy_and_safety={name: name + '.json' for name in energy_fields},
                             resource_budget=dict(limits='limits.json',
                                                  report='resource.json'),
                             version_metadata=dict(package_xml='package.xml',
                                                   setup_py='setup.py'),
                             release_document='draft.md', pending_checks=[], exclusions=[])

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

    def test_required_evidence_groups_cannot_be_omitted(self):
        for key in ('worlds', 'single_bird', 'perception', 'energy_and_safety',
                    'resource_budget', 'version_metadata', 'release_document'):
            altered = copy.deepcopy(self.manifest)
            del altered[key]
            result = audit_manifest(self.root, altered)
            self.assertFalse(result['passed'], key)
            self.assertEqual(result['checks'][0]['id'], 'manifest_schema')

    def test_repeat_threshold_cannot_be_weakened(self):
        self.manifest['minimum_runs_per_world'] = 1
        result = audit_manifest(self.root, self.manifest)
        self.assertFalse(result['passed'])
        self.assertIn('at least 2', str(result['checks']))

    def test_negative_counts_and_zero_clearance_fail(self):
        report = copy.deepcopy(self.reports[0])
        report['safety_error_samples'] = -1
        report['minimum_truth_static_clearance_m'] = 0.0
        self.assertIn('valid count', str(evaluate_flight(report, 'no_bird')))
        self.assertIn('minimum_truth_static_clearance_m',
                      str(evaluate_flight(report, 'no_bird')))

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

    def test_release_notes_must_disclose_scope(self):
        self.manifest['release_document'] = 'draft.md'
        (self.root / 'draft.md').write_text(
            'v0.5.12 发布；双鸟暂缓，Baylands 仅导入，物理净空为抽样。',
            encoding='utf-8')
        self.assertTrue(audit_manifest(self.root, self.manifest)['passed'])
        (self.root / 'draft.md').write_text('v0.5.12', encoding='utf-8')
        result = audit_manifest(self.root, self.manifest)
        self.assertFalse(result['passed'])
        self.assertIn('release notes omit', str(result['checks']))


if __name__ == '__main__':
    unittest.main()
