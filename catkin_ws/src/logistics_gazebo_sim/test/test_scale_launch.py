import unittest
from pathlib import Path
import xml.etree.ElementTree as ET

from logistics_gazebo_sim.scale_launch import (PORT_BASES, preflight_scale,
                                               scale_master_uri, scale_ports)


PACKAGE = Path(__file__).resolve().parents[1]


class ScaleLaunchTest(unittest.TestCase):
    def test_launch_declares_all_eight_instances_and_guard(self):
        root = ET.parse(str(PACKAGE / 'launch/fleet_scale_sitl.launch')).getroot()
        guard = next(arg.get('value') for arg in root.findall('arg')
                     if arg.get('name') == 'vehicle_count_guard')
        self.assertIn('(1, 3, 5, 8)', guard)
        includes = [include for include in root.findall('include')
                    if 'sitl_instance.launch' in include.get('file', '')]
        ids = [next(arg.get('value') for arg in include.findall('arg')
                    if arg.get('name') == 'instance_id') for include in includes]
        self.assertEqual(ids, [str(index) for index in range(8)])
        self.assertTrue(all(next(arg.get('value') for arg in item.findall('arg')
                                 if arg.get('name') == 'sensor_model') == 'iris'
                            for item in includes))

    def test_ports_unique_and_match_px4_instance_formula(self):
        ports = scale_ports(8, 11490)
        self.assertEqual(len(ports), 1 + 8 * (1 + len(PORT_BASES)))
        self.assertEqual(len({(protocol, port) for protocol, _, port in ports}),
                         len(ports))
        self.assertIn(('tcp', 'px4_simulator_7', 4567), ports)
        self.assertIn(('udp', 'mavros_bind_udp_7', 14547), ports)
        self.assertIn(('udp', 'px4_offboard_udp_7', 14587), ports)
        self.assertIn(('udp', 'camera_udp_7', 14537), ports)

    def test_world_capacity_and_kinematic_plan_for_each_scale(self):
        self.assertEqual(scale_master_uri(PACKAGE), 'http://127.0.0.1:11490')
        for count, groups in ((1, [1]), (3, [3]), (5, [3, 2]),
                              (8, [3, 3, 2])):
            report = preflight_scale(count, PACKAGE,
                                     resources=lambda: (30.0, 30000.0),
                                     port_available=lambda *_: True)
            self.assertTrue(report['pass'], report['errors'])
            self.assertEqual(report['world_spawn_capacity'], 8)
            self.assertEqual([len(g['vehicles']) for g in report['group_plan']['groups']],
                             groups)

    def test_invalid_count_resource_or_port_blocks_start(self):
        for count in (0, 2, 9):
            report = preflight_scale(count, PACKAGE,
                                     resources=lambda: (30.0, 30000.0),
                                     port_available=lambda *_: True)
            self.assertFalse(report['pass'])
        report = preflight_scale(8, PACKAGE,
                                 resources=lambda: (2.0, 2000.0),
                                 port_available=lambda *_: False)
        self.assertFalse(report['pass'])
        self.assertIn('CPU below', str(report['errors']))
        self.assertIn('memory below', str(report['errors']))
        self.assertIn('port unavailable', str(report['errors']))


if __name__ == '__main__':
    unittest.main()
