"""Offline tests: no downloaded assets or running Gazebo required."""
import importlib.machinery
from pathlib import Path
import tempfile
import unittest
import numpy as np

geometry = importlib.machinery.SourceFileLoader(
    'warehouse_geometry', str(Path(__file__).resolve().parents[1] /
                              'scripts/inspect_warehouse_geometry')).load_module()


class WarehouseGeometryTest(unittest.TestCase):
    def mesh(self, matrix, axis='Z_UP'):
        return '''<COLLADA xmlns="http://www.collada.org/2005/11/COLLADASchema">
          <asset><unit meter="0.01"/><up_axis>{}</up_axis></asset>
          <library_geometries><geometry id="g"><mesh>
          <source id="p"><float_array>100 0 0 0 200 0</float_array>
          <technique_common><accessor stride="3"/></technique_common></source>
          <vertices><input semantic="POSITION" source="#p"/></vertices>
          </mesh></geometry></library_geometries>
          <library_visual_scenes><visual_scene id="s"><node><matrix>{}</matrix>
          <instance_geometry url="#g"/></node></visual_scene></library_visual_scenes>
          <scene><instance_visual_scene url="#s"/></scene></COLLADA>'''.format(axis, matrix)

    def read(self, content):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / 'mesh.dae'
            path.write_text(content)
            return geometry.mesh_points(path)

    def test_centimetres_rotation_and_translation(self):
        points, unit = self.read(self.mesh('0 -1 0 300 1 0 0 400 0 0 1 500 0 0 0 1'))
        np.testing.assert_allclose(points, [[3, 5, 5], [1, 4, 5]])
        self.assertEqual(unit, 0.01)

    def test_rejects_transposed_translation(self):
        with self.assertRaisesRegex(ValueError, 'non-affine'):
            self.read(self.mesh('1 0 0 0 0 1 0 0 0 0 1 0 300 400 500 1'))

    def test_rejects_unknown_axis(self):
        with self.assertRaisesRegex(ValueError, 'axis'):
            self.read(self.mesh('1 0 0 0 0 1 0 0 0 0 1 0 0 0 0 1', 'Y_UP'))

    def test_sdf_pose_rotation_and_translation(self):
        transformed = geometry.pose('1 2 3 0 0 1.5707963267948966') @ [2, 0, 0, 1]
        np.testing.assert_allclose(transformed, [1, 4, 3, 1])
