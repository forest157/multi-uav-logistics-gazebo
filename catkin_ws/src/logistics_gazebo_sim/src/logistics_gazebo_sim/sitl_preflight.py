"""Fail-closed preflight for the current three-PX4 launch configuration.

This checks configuration and current host availability only. It does not
reserve ports, guarantee future resources, or validate a flight trajectory.
"""

import os
from pathlib import Path
import socket
from urllib.parse import urlparse
import math
import xml.etree.ElementTree as ET


SUPPORTED_VEHICLE_COUNT = 3
MINIMUM_CPU_CORES = 4.0
MINIMUM_FREE_MEMORY_MIB = 10240.0
FIXED_PORTS = {
    'mavros_udp': (14540, 14541, 14542),
    'gazebo_mavlink_udp': (14560, 14561, 14562),
    'px4_mavlink_tcp': (4560, 4561, 4562),
    'fcu_remote_udp': (14580, 14581, 14582),
}


def _read(path):
    try:
        return Path(path).read_text(encoding='ascii').strip()
    except OSError:
        return None


def resource_snapshot():
    """Use the tighter of host availability and cgroup v2 limits."""
    cores = float(len(os.sched_getaffinity(0)))
    cpu_max = _read('/sys/fs/cgroup/cpu.max')
    if cpu_max:
        quota, period = cpu_max.split()
        if quota != 'max':
            cores = min(cores, float(quota) / float(period))
    meminfo = _read('/proc/meminfo')
    if not meminfo:
        raise RuntimeError('MemAvailable is unavailable')
    available = next((line.split()[1] for line in meminfo.splitlines()
                      if line.startswith('MemAvailable:')), None)
    if available is None:
        raise RuntimeError('MemAvailable is unavailable')
    free_mib = float(available) / 1024.0
    limit = _read('/sys/fs/cgroup/memory.max')
    used = _read('/sys/fs/cgroup/memory.current')
    if limit and limit != 'max' and used:
        free_mib = min(free_mib, max(0.0, (int(limit) - int(used)) / 1048576.0))
    return cores, free_mib


def gazebo_master_port(uri):
    parsed = urlparse(uri)
    if parsed.scheme != 'http' or parsed.hostname not in ('127.0.0.1', 'localhost'):
        raise ValueError('Gazebo master must use local http://127.0.0.1:PORT')
    if parsed.port is None or not 1 <= parsed.port <= 65535:
        raise ValueError('Gazebo master port is invalid')
    return parsed.port


def default_master_uri(world, world_dir):
    """Read the selected mission launch, not an unrelated generic default."""
    if world == 'scene_0':
        filename = 'three_uav_mission.launch'
    elif world in ('outdoor_campus', 'outdoor_residential', 'outdoor_urban'):
        filename = world + '_lidar_trial.launch'
    else:
        raise ValueError('world has no supported launch entry: ' + world)
    path = Path(world_dir).resolve().parent / 'launch' / filename
    root = ET.parse(str(path)).getroot()
    values = [arg.get('default') for arg in root.findall('arg')
              if arg.get('name') == 'gazebo_master_uri']
    if len(values) != 1 or not values[0]:
        raise ValueError('launch has no unique Gazebo master default: ' + filename)
    gazebo_master_port(values[0])
    return values[0]


def launch_ports(master_port):
    """Mirror the current launch file's fixed three-instance port matrix."""
    return [('tcp', 'gazebo_master', master_port)] + [
        (protocol, label + '_' + str(index), port)
        for label, values in FIXED_PORTS.items()
        for index, port in enumerate(values)
        for protocol in (('tcp',) if label == 'px4_mavlink_tcp' else ('udp',))
    ]


def check_port(protocol, port):
    kind = socket.SOCK_STREAM if protocol == 'tcp' else socket.SOCK_DGRAM
    with socket.socket(socket.AF_INET, kind) as probe:
        try:
            probe.bind(('0.0.0.0', port))
        except OSError:
            return False
    return True


def evaluate_preflight(vehicle_count, world_capacity, cpu_cores, free_memory_mib,
                       ports, port_available=check_port):
    errors = []
    if type(vehicle_count) is not int or vehicle_count != SUPPORTED_VEHICLE_COUNT:
        errors.append('current PX4 launch supports exactly three vehicles')
    if (type(world_capacity) is not int or type(vehicle_count) is not int or
            world_capacity < vehicle_count):
        errors.append('world spawn capacity is below requested vehicle count')
    if (not isinstance(cpu_cores, (int, float)) or not math.isfinite(cpu_cores) or
            cpu_cores < MINIMUM_CPU_CORES):
        errors.append('available CPU below {:.0f} cores'.format(MINIMUM_CPU_CORES))
    if (not isinstance(free_memory_mib, (int, float)) or
            not math.isfinite(free_memory_mib) or
            free_memory_mib < MINIMUM_FREE_MEMORY_MIB):
        errors.append('available memory below {:.0f} MiB'.format(MINIMUM_FREE_MEMORY_MIB))
    seen = set()
    for protocol, label, port in ports:
        key = (protocol, port)
        if protocol not in ('tcp', 'udp') or type(port) is not int or not 1 <= port <= 65535:
            errors.append('invalid port: ' + label)
        elif key in seen:
            errors.append('duplicate port: ' + label)
        elif not port_available(protocol, port):
            errors.append('port unavailable: {} ({}/{})'.format(label, protocol, port))
        seen.add(key)
    return errors


def world_spawn_capacity(world, world_dir):
    world_dir = Path(world_dir)
    if world == 'scene_0':
        if not (world_dir / 'scene_0.world').is_file():
            raise ValueError('legacy scene_0 world is missing')
        return 3
    if world not in ('outdoor_campus', 'outdoor_residential', 'outdoor_urban'):
        raise ValueError('world has no validated three-vehicle profile: ' + world)
    from .outdoor_world_profile import load_outdoor_profile
    profile = load_outdoor_profile(world, str(world_dir))
    return len(profile['spawn_positions_m'])


def preflight(world, world_dir, vehicle_count=3, gazebo_master_uri=None):
    """Return a report; never claim that a physical flight has been tested."""
    errors = []
    capacity = 0
    ports = []
    resolved_uri = None
    cpu_cores = free_mib = 0.0
    try:
        capacity = world_spawn_capacity(world, world_dir)
    except (OSError, ValueError, KeyError, TypeError) as exc:
        errors.append('world validation failed: {}'.format(exc))
    try:
        resolved_uri = (default_master_uri(world, world_dir) if gazebo_master_uri is None
                        else gazebo_master_uri)
        ports = launch_ports(gazebo_master_port(resolved_uri))
    except (OSError, ValueError, ET.ParseError) as exc:
        errors.append('launch port validation failed: {}'.format(exc))
    try:
        cpu_cores, free_mib = resource_snapshot()
    except (OSError, ValueError, RuntimeError, ZeroDivisionError) as exc:
        errors.append('resource check failed: {}'.format(exc))
    errors.extend(evaluate_preflight(vehicle_count, capacity, cpu_cores,
                                     free_mib, ports))
    return {'pass': not errors, 'world': world, 'vehicle_count': vehicle_count,
            'gazebo_master_uri': resolved_uri,
            'world_spawn_capacity': capacity, 'available_cpu_cores': cpu_cores,
            'available_memory_mib': free_mib, 'ports_checked': len(ports),
            'errors': errors, 'scope': 'prelaunch-only; ports are not reserved'}
