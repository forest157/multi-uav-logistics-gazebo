import hashlib
import os
import tempfile
import unittest

from logistics_gazebo_sim.operator_config_info import (
    algorithm_profile, compare_runtime_configuration,
    requested_runtime_configuration, scene_identity, source_identity)


class OperatorConfigInfoTest(unittest.TestCase):
    def test_mpc_orca_comparison_is_shadow_only(self):
        collective = algorithm_profile('collective_offset')
        orca = algorithm_profile('orca3d')
        comparison = algorithm_profile('distributed_mpc')
        self.assertTrue(collective['execution'])
        self.assertTrue(orca['execution'])
        self.assertEqual(orca['orca_mode'], 'limited')
        self.assertFalse(comparison['execution'])
        self.assertEqual(comparison['orca_mode'], 'shadow')
        with self.assertRaises(ValueError):
            algorithm_profile('hybrid_active')

    def test_scene_hash_and_missing_git_are_explicit(self):
        with tempfile.TemporaryDirectory() as directory:
            worlds = os.path.join(directory, 'worlds')
            os.mkdir(worlds)
            with open(os.path.join(worlds, 'scene_0.world'), 'wb') as stream:
                stream.write(b'<sdf>test</sdf>')
            with open(os.path.join(directory, 'package.xml'), 'w', encoding='utf-8') as stream:
                stream.write('<package><version>0.6.0</version></package>')
            identity = scene_identity(directory, 0)
            self.assertEqual(identity['world_sha256'], hashlib.sha256(b'<sdf>test</sdf>').hexdigest())
            self.assertEqual(source_identity(directory)['git_commit'], 'unavailable')
            self.assertEqual(source_identity(directory)['package_version'], '0.6.0')
            with self.assertRaises(ValueError):
                scene_identity(directory, 7)

    def test_runtime_readback_fails_closed_and_distinguishes_shadow(self):
        collective = requested_runtime_configuration('collective_offset', True, 2.0, 0.6)
        self.assertEqual(compare_runtime_configuration(collective, dict(collective)), [])
        shadow = requested_runtime_configuration('distributed_mpc', True, 2.0, 0.6)
        self.assertFalse(shadow['dynamic_avoidance_execution'])
        self.assertEqual(shadow['orca_control_mode'], 'shadow')
        differences = compare_runtime_configuration(collective, dict(collective,
            local_avoidance_algorithm='orca3d', orca_control_mode='limited'))
        self.assertEqual({item['parameter'] for item in differences},
                         {'local_avoidance_algorithm', 'orca_control_mode'})
        for invalid in (dict(collective, dynamic_avoidance_execution='true'),
                        dict(collective, orca_max_speed_mps=float('nan')),
                        {key: value for key, value in collective.items()
                         if key != 'orca_command_timeout_s'}):
            with self.subTest(invalid=invalid), self.assertRaises(ValueError):
                compare_runtime_configuration(collective, invalid)


if __name__ == '__main__':
    unittest.main()
