"""Fail-closed preflight for lightweight 1/3/5/8 PX4 scale-yard SITL."""

import math
from pathlib import Path
import xml.etree.ElementTree as ET

from .outdoor_world_profile import load_outdoor_profile
from .scale_passage import passage_plan
from .sitl_preflight import (check_port, gazebo_master_port,
                             resource_snapshot)


SUPPORTED_COUNTS = (1, 3, 5, 8)
PORT_BASES = {
    'mavros_bind_udp': 14540,
    'px4_offboard_udp': 14580,
    'gazebo_mavlink_udp': 14560,
    'px4_gcs_udp': 18570,
    'px4_payload_udp': 14280,
    'px4_gimbal_udp': 13030,
    'video_udp': 5600,
    'camera_udp': 14530,
}


def scale_ports(count, master_port):
    if count not in SUPPORTED_COUNTS:
        raise ValueError('scale SITL requires 1, 3, 5 or 8 vehicles')
    ports = [('tcp', 'gazebo_master', master_port)]
    for index in range(count):
        ports.append(('tcp', 'px4_simulator_{}'.format(index), 4560 + index))
        for name, base in PORT_BASES.items():
            ports.append(('udp', '{}_{}'.format(name, index), base + index))
    if len({(protocol, port) for protocol, _, port in ports}) != len(ports):
        raise ValueError('scale SITL port matrix overlaps')
    return ports


def scale_master_uri(package_dir):
    root = ET.parse(str(Path(package_dir) / 'launch/fleet_scale_sitl.launch')).getroot()
    values = [arg.get('default') for arg in root.findall('arg')
              if arg.get('name') == 'gazebo_master_uri']
    if len(values) != 1 or not values[0]:
        raise ValueError('scale launch has no unique Gazebo master default')
    gazebo_master_port(values[0])
    return values[0]


def preflight_scale(count, package_dir, master_uri=None,
                    resources=resource_snapshot, port_available=check_port):
    errors = []
    if type(count) is not int or count not in SUPPORTED_COUNTS:
        errors.append('supported scale counts are 1, 3, 5 and 8')
    profile = None
    plan = None
    try:
        profile = load_outdoor_profile('outdoor_scale_yard',
                                       str(Path(package_dir) / 'worlds'))
        if count in SUPPORTED_COUNTS:
            if len(profile['spawn_positions_m']) < count:
                errors.append('world has fewer spawn pads than vehicles')
            else:
                plan = passage_plan(count, profile)
    except (OSError, ValueError, KeyError, TypeError) as exc:
        errors.append('scale world or passage plan invalid: {}'.format(exc))
    uri = None
    ports = []
    try:
        uri = scale_master_uri(package_dir) if master_uri is None else master_uri
        ports = scale_ports(count, gazebo_master_port(uri))
    except (OSError, ValueError, ET.ParseError) as exc:
        errors.append('scale launch port configuration invalid: {}'.format(exc))
    cpu = memory = 0.0
    try:
        cpu, memory = resources()
        minimum_cpu = max(4.0, float(count) + 2.0)
        minimum_memory = 4096.0 + 1536.0 * count
        if not math.isfinite(cpu) or cpu < minimum_cpu:
            errors.append('available CPU below {:.0f} cores'.format(minimum_cpu))
        if not math.isfinite(memory) or memory < minimum_memory:
            errors.append('available memory below {:.0f} MiB'.format(minimum_memory))
    except (OSError, ValueError, TypeError, RuntimeError) as exc:
        errors.append('scale resource check failed: {}'.format(exc))
    for protocol, label, port in ports:
        if not port_available(protocol, port):
            errors.append('port unavailable: {} ({}/{})'.format(label, protocol, port))
    return {'pass': not errors, 'vehicle_count': count,
            'world': 'outdoor_scale_yard',
            'world_sha256': profile['world_sha256'] if profile else None,
            'world_spawn_capacity': len(profile['spawn_positions_m']) if profile else 0,
            'gazebo_master_uri': uri, 'available_cpu_cores': cpu,
            'available_memory_mib': memory, 'ports_checked': len(ports),
            'group_plan': plan, 'errors': errors,
            'scope': 'prelaunch and kinematic plan only; no PX4 passage claim'}
