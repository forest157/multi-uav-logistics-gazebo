import json
import math
import os
import tempfile
import unittest

from logistics_gazebo_sim.operator_task_io import (
    build_operator_report, load_task, validate_task, write_json_atomic)


def task():
    return {'schema': 1, 'scene_id': 0, 'start_m': [-40, -40],
            'goal_m': [45, 45], 'altitude_m': 8, 'formation': 'triangle',
            'dynamic_obstacles': True, 'avoidance_mode': 'collective_offset',
            'perception_source': 'perception'}


class OperatorTaskIOTest(unittest.TestCase):
    def test_task_roundtrip_is_bounded_and_normalized(self):
        with tempfile.TemporaryDirectory() as directory:
            path = os.path.join(directory, 'mission.json')
            write_json_atomic(path, validate_task(task()))
            self.assertEqual(load_task(path), validate_task(task()))
            self.assertFalse(any(name.endswith('.tmp') for name in os.listdir(directory)))

    def test_bad_preset_is_rejected_before_ui_mutation(self):
        for change in (
                {'scene_id': 7}, {'start_m': [math.nan, 0]},
                {'altitude_m': 50}, {'formation': 'unknown'},
                {'dynamic_obstacles': 1}, {'avoidance_mode': 'unknown'},
                {'orca_max_speed_mps': 3.0}, {'orca_command_timeout_s': 0.8}):
            bad = dict(task(), **change)
            with self.subTest(change=change), self.assertRaises(ValueError):
                validate_task(bad)

    def test_large_file_is_rejected(self):
        with tempfile.TemporaryDirectory() as directory:
            path = os.path.join(directory, 'large.json')
            with open(path, 'w', encoding='utf-8') as stream:
                stream.write(' ' * 65537)
            with self.assertRaises(ValueError):
                load_task(path)

    def test_older_presets_keep_conservative_orca_defaults(self):
        normalized = validate_task(task())
        self.assertEqual(normalized['orca_max_speed_mps'], 2.0)
        self.assertEqual(normalized['orca_command_timeout_s'], 0.6)
        shadow = dict(task(), avoidance_mode='distributed_mpc')
        self.assertEqual(validate_task(shadow)['avoidance_mode'], 'distributed_mpc')

    def test_report_is_explicitly_ui_only_and_atomic(self):
        event = {'time': 1728604800.5, 'source': '安全联锁', 'level': 'ERROR',
                 'title': '安全联锁保持', 'guidance': '检查机间距'}
        report = build_operator_report(task(), {'approved': False,
            'status': '等待重新规划', 'detail': '旧规划已失效'}, None, None, None, [event])
        self.assertEqual(report['kind'], 'operator_ui_snapshot')
        self.assertFalse(report['planning']['approved'])
        self.assertIn('not a flight safety audit', report['scope'])
        self.assertEqual(report['events'][0]['time_unix_s'], 1728604800.5)
        self.assertEqual(report['events'][0]['time_utc'], '2024-10-11T00:00:00.500000+00:00')
        with tempfile.TemporaryDirectory() as directory:
            path = os.path.join(directory, 'report.json')
            write_json_atomic(path, report)
            with open(path, encoding='utf-8') as stream:
                self.assertEqual(json.load(stream)['events'][0]['title'], '安全联锁保持')
            with self.assertRaises(ValueError):
                write_json_atomic(path, {'invalid': math.nan})
            with open(path, encoding='utf-8') as stream:
                self.assertEqual(json.load(stream)['kind'], 'operator_ui_snapshot')

    def test_report_rejects_missing_or_nonfinite_event_time(self):
        planning = {'approved': False, 'status': '等待重新规划'}
        event = {'time': 1.0, 'source': '动态风险', 'level': 'WARN',
                 'title': '数据过期', 'guidance': '检查感知节点'}
        for invalid in (None, float('nan'), float('inf'), -1, 253402300800):
            with self.subTest(timestamp=invalid), self.assertRaises(ValueError):
                build_operator_report(task(), planning, None, None, None,
                                      [dict(event, time=invalid)])


if __name__ == '__main__':
    unittest.main()
