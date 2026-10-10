import hashlib
import os
import tempfile
import unittest

from logistics_gazebo_sim.operator_config_info import (
    algorithm_profile, scene_identity, source_identity)


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


if __name__ == '__main__':
    unittest.main()
