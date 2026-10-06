import unittest

from logistics_gazebo_sim.operator_event_journal import OperatorEventJournal


class OperatorEventJournalTest(unittest.TestCase):
    def test_safety_hold_and_recovery_are_single_transitions(self):
        journal = OperatorEventJournal()
        hold = {'state': 'SAFETY_HOLD', 'dynamic_action': 'NORMAL',
                'safety_interlock': {'hold': True, 'reason': 'measured separation 2.5 m'}}
        normal = {'state': 'RUNNING', 'dynamic_action': 'NORMAL',
                  'safety_interlock': {'hold': False, 'reason': None}}
        self.assertTrue(journal.mission(hold, 1.0))
        self.assertFalse(journal.mission(hold, 1.2))
        self.assertEqual(len(journal.rows), 1)
        self.assertTrue(journal.mission(normal, 2.0))
        self.assertEqual(journal.rows[0]['title'], '安全联锁解除')

    def test_risk_and_diagnostics_do_not_log_every_five_hertz_frame(self):
        journal = OperatorEventJournal()
        for step in range(100):
            journal.risk('CRITICAL', 10.0 + step * 0.2)
            journal.diagnostic(2, 'vehicle separation violation', 10.0 + step * 0.2)
        self.assertEqual(len(journal.rows), 2)
        self.assertTrue(journal.risk('SAFE', 31.0))
        self.assertTrue(journal.diagnostic(0, 'safe', 31.0))
        self.assertEqual(len(journal.rows), 4)

    def test_planning_failure_is_classified_bounded_and_deduplicated(self):
        journal = OperatorEventJournal(limit=2)
        self.assertTrue(journal.planning_failure('FEASIBILITY', '任务不可行', '调整高度 ' * 100, 1.0))
        self.assertFalse(journal.planning_failure('FEASIBILITY', '任务不可行', '调整高度 ' * 100, 1.2))
        self.assertLessEqual(len(journal.rows[0]['guidance']), 320)
        self.assertTrue(journal.planning_failure('TIMEOUT', '分析超时', '重新规划', 2.0))
        self.assertTrue(journal.planning_failure('INPUT', '输入无效', '检查坐标', 3.0))
        self.assertEqual(len(journal.rows), 2)
        self.assertEqual(journal.rows[0]['source'], '规划/INPUT')

    def test_initial_normal_does_not_create_recovery_event(self):
        journal = OperatorEventJournal()
        self.assertFalse(journal.risk('SAFE', 1.0))
        self.assertFalse(journal.diagnostic(0, 'safe', 1.0))
        self.assertFalse(journal.mission({'state': 'READY', 'dynamic_action': 'NORMAL',
                                          'safety_interlock': {'hold': False}}, 1.0))
        self.assertEqual(journal.rows, [])


if __name__ == '__main__':
    unittest.main()
