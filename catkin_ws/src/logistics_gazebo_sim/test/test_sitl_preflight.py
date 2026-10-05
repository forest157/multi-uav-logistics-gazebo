import unittest
from pathlib import Path
from unittest.mock import patch
import xml.etree.ElementTree as ET
import runpy
import tempfile

from logistics_gazebo_sim import sitl_preflight as preflight


PACKAGE = Path(__file__).resolve().parent.parent


class SitlPreflightTest(unittest.TestCase):
    def test_current_three_vehicle_configuration_passes(self):
        errors = preflight.evaluate_preflight(
            3, 3, 8.0, 20000.0, preflight.launch_ports(11450),
            port_available=lambda _protocol, _port: True)
        self.assertEqual(errors, [])

    def test_count_capacity_resource_and_port_fail_closed(self):
        ports = [('udp', 'first', 14540), ('udp', 'second', 14540),
                 ('tcp', 'occupied', 11450)]
        errors = preflight.evaluate_preflight(
            5, 3, 2.0, 5000.0, ports,
            port_available=lambda protocol, port: port != 11450)
        self.assertIn('exactly three', str(errors))
        self.assertIn('world spawn capacity', str(errors))
        self.assertIn('CPU below', str(errors))
        self.assertIn('memory below', str(errors))
        self.assertIn('duplicate port', str(errors))
        self.assertIn('port unavailable', str(errors))

    def test_nonfinite_resource_measurements_fail_closed(self):
        errors = preflight.evaluate_preflight(
            3, 3, float('nan'), float('inf'), [],
            port_available=lambda _protocol, _port: True)
        self.assertIn('CPU below', str(errors))
        self.assertIn('memory below', str(errors))

    def test_outdoor_world_capacity_comes_from_verified_profile(self):
        worlds = PACKAGE / 'worlds'
        for name in ('outdoor_campus', 'outdoor_residential', 'outdoor_urban'):
            self.assertEqual(preflight.world_spawn_capacity(name, worlds), 3)
        with self.assertRaises(ValueError):
            preflight.world_spawn_capacity('baylands', worlds)

    def test_launch_never_advertises_more_than_three_px4_instances(self):
        root = ET.parse(str(PACKAGE / 'launch/three_uav_mission.launch')).getroot()
        guards = [arg for arg in root.iter('arg')
                  if arg.get('name') == 'vehicle_count_guard']
        self.assertEqual(len(guards), 1)
        self.assertIn("int(arg('vehicle_count')) == 3", guards[0].get('value'))
        values = [param.get('value') for param in root.iter('param')
                  if param.get('name') == 'vehicle_count']
        self.assertTrue(values)
        self.assertEqual(set(values), {'$(arg vehicle_count)'})
        sitl = ET.parse(str(PACKAGE / 'launch/three_uav_sitl.launch')).getroot()
        instances = [include for include in sitl.findall('include')
                     if 'sitl_instance.launch' in include.get('file', '')]
        self.assertEqual([next(arg.get('value') for arg in include.findall('arg')
                               if arg.get('name') == 'instance_id')
                          for include in instances], ['0', '1', '2'])

    def test_preflight_ports_match_roslaunch_defaults(self):
        mission = ET.parse(str(PACKAGE / 'launch/three_uav_mission.launch')).getroot()
        master = next(arg.get('default') for arg in mission.iter('arg')
                      if arg.get('name') == 'gazebo_master_uri')
        self.assertEqual(preflight.gazebo_master_port(master), 11450)
        instance = ET.parse(str(PACKAGE / 'launch/sitl_instance.launch')).getroot()
        group = instance.find('group')
        self.assertEqual(group.get('ns'), "$(eval 'uav' + str(int(arg('instance_id'))))")
        includes = list(group.findall('include'))
        self.assertEqual(len(includes), 2)
        spawn = {arg.get('name'): arg.get('value') for arg in includes[0].findall('arg')}
        mavros = {arg.get('name'): arg.get('value') for arg in includes[1].findall('arg')}
        self.assertEqual(spawn['mavlink_udp_port'], "$(eval 14560 + int(arg('instance_id')))")
        self.assertEqual(spawn['mavlink_tcp_port'], "$(eval 4560 + int(arg('instance_id')))")
        self.assertEqual(mavros['fcu_url'],
                         "$(eval 'udp://:' + str(14540 + int(arg('instance_id'))) + '@localhost:' + str(14580 + int(arg('instance_id'))))")
        self.assertEqual(mavros['tgt_system'], "$(eval 1 + int(arg('instance_id')))")
        self.assertEqual(preflight.FIXED_PORTS, {
            'mavros_udp': (14540, 14541, 14542),
            'gazebo_mavlink_udp': (14560, 14561, 14562),
            'px4_mavlink_tcp': (4560, 4561, 4562),
            'fcu_remote_udp': (14580, 14581, 14582),
        })

    def test_resource_snapshot_respects_cgroup_caps(self):
        files = {'/sys/fs/cgroup/cpu.max': '200000 100000',
                 '/sys/fs/cgroup/memory.max': str(12 * 1048576 * 1024),
                 '/sys/fs/cgroup/memory.current': str(4 * 1048576 * 1024),
                 '/proc/meminfo': 'MemAvailable: 20000000 kB\n'}
        with patch.object(preflight.os, 'sched_getaffinity', return_value=set(range(8))), \
                patch.object(preflight, '_read', side_effect=files.get):
            cores, free_mib = preflight.resource_snapshot()
        self.assertEqual(cores, 2.0)
        self.assertEqual(free_mib, 8192.0)

    def test_master_uri_must_be_local_and_have_port(self):
        self.assertEqual(preflight.gazebo_master_port('http://127.0.0.1:11450'), 11450)
        for uri in ('http://remote:11450', 'http://127.0.0.1', 'not-a-uri'):
            with self.assertRaises(ValueError):
                preflight.gazebo_master_port(uri)

    def test_selected_launch_default_is_checked(self):
        worlds = PACKAGE / 'worlds'
        self.assertEqual(preflight.default_master_uri('scene_0', worlds),
                         'http://127.0.0.1:11450')
        for name in ('outdoor_campus', 'outdoor_residential', 'outdoor_urban'):
            self.assertEqual(preflight.default_master_uri(name, worlds),
                             'http://127.0.0.1:11470')
        with patch.object(preflight, 'evaluate_preflight', return_value=[]) as check, \
                patch.object(preflight, 'resource_snapshot', return_value=(8.0, 20000.0)):
            result = preflight.preflight('outdoor_campus', worlds)
        self.assertTrue(result['pass'])
        self.assertEqual(result['gazebo_master_uri'], 'http://127.0.0.1:11470')
        self.assertEqual(check.call_args.args[4][0], ('tcp', 'gazebo_master', 11470))

    def test_checked_wrapper_maps_world_without_shell(self):
        wrapper = runpy.run_path(str(PACKAGE / 'scripts/launch_outdoor_checked'))
        command = wrapper['launch_command']('outdoor_urban', 'false',
                                            'http://127.0.0.1:11470')
        self.assertEqual(command, ['roslaunch', 'logistics_gazebo_sim',
                                   'outdoor_urban_lidar_trial.launch', 'gui:=false',
                                   'gazebo_master_uri:=http://127.0.0.1:11470'])
        with self.assertRaises(ValueError):
            wrapper['launch_command']('baylands', 'true', 'http://127.0.0.1:11470')
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            (root / 'Tools/sitl_gazebo').mkdir(parents=True)
            (root / 'package.xml').write_text('', encoding='utf-8')
            (root / 'Tools/sitl_gazebo/package.xml').write_text('', encoding='utf-8')
            environment = wrapper['ros_environment'](root, {'ROS_PACKAGE_PATH': '/ros'})
            self.assertEqual(environment['ROS_PACKAGE_PATH'].split(':'),
                             [str(root / 'Tools/sitl_gazebo'), str(root), '/ros'])


if __name__ == '__main__':
    unittest.main()
