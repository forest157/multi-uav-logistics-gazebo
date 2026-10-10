"""Explicit operator algorithm scope and reproducible source/world identity."""

import hashlib
import math
import os
import subprocess
import xml.etree.ElementTree as ET


ALGORITHM_PROFILES = {
    'collective_offset': {
        'label': '整队避障接管', 'contract': 'collective_offset_v1',
        'execution': True, 'orca_mode': 'shadow',
        'scope': '已验收的三机整队偏移安全门'},
    'orca3d': {
        'label': 'ORCA 受限接管', 'contract': 'orca_velocity_v1',
        'execution': True, 'orca_mode': 'limited',
        'scope': '仅三机、受限速度与安全门；不代表多鸟/遮挡矩阵通过'},
    'distributed_mpc': {
        'label': 'MPC＋ORCA 候选对照（影子）', 'contract': 'mpc_trajectory_v1',
        'execution': False, 'orca_mode': 'shadow',
        'scope': '只计算 MPC 候选及失败时 ORCA 回退候选；不执行轨迹或回退'},
}

RUNTIME_PARAMETER_LABELS = {
    'dynamic_safety_enabled': '动态障碍',
    'dynamic_avoidance_execution': '避障接管',
    'local_avoidance_algorithm': '局部算法',
    'orca_control_mode': 'ORCA 模式',
    'orca_max_speed_mps': 'ORCA 限速',
    'orca_command_timeout_s': 'ORCA 超时',
}


def algorithm_profile(name):
    try:
        return dict(ALGORITHM_PROFILES[name])
    except (KeyError, TypeError):
        raise ValueError('unsupported operator algorithm')


def requested_runtime_configuration(algorithm, dynamic_enabled, orca_speed, orca_timeout):
    """Expected mission-player parameters for the current next-launch controls."""
    profile = algorithm_profile(algorithm)
    if type(dynamic_enabled) is not bool:
        raise ValueError('invalid dynamic-obstacle selection')
    for value, lower, upper in ((orca_speed, 0.5, 2.0), (orca_timeout, 0.3, 0.6)):
        if type(value) not in (int, float) or not math.isfinite(value) or not lower <= value <= upper:
            raise ValueError('invalid ORCA parameter')
    return {
        'dynamic_safety_enabled': dynamic_enabled,
        'dynamic_avoidance_execution': dynamic_enabled and profile['execution'],
        'local_avoidance_algorithm': algorithm,
        'orca_control_mode': profile['orca_mode'],
        'orca_max_speed_mps': float(orca_speed),
        'orca_command_timeout_s': float(orca_timeout),
    }


def compare_runtime_configuration(requested, running):
    """Compare an actual ROS parameter readback without treating missing data as a match."""
    if not isinstance(requested, dict) or not isinstance(running, dict):
        raise ValueError('runtime configuration must be a mapping')
    differences = []
    for name in RUNTIME_PARAMETER_LABELS:
        expected = requested.get(name)
        actual = running.get(name)
        if name in ('dynamic_safety_enabled', 'dynamic_avoidance_execution'):
            valid = type(actual) is bool
        elif name in ('orca_max_speed_mps', 'orca_command_timeout_s'):
            valid = type(actual) in (int, float) and math.isfinite(actual)
        else:
            valid = isinstance(actual, str) and 0 < len(actual) <= 64
        if not valid:
            raise ValueError('missing or invalid runtime parameter: ' + name)
        if actual != expected:
            differences.append({'parameter': name, 'label': RUNTIME_PARAMETER_LABELS[name],
                                'requested': expected, 'running': actual})
    return differences


def source_identity(package_path):
    """Return bounded provenance; installed packages may have no Git metadata."""
    version = 'unknown'
    try:
        version = ET.parse(os.path.join(package_path, 'package.xml')).findtext('version') or 'unknown'
    except (OSError, ET.ParseError):
        pass
    directory = os.path.abspath(package_path)
    for _ in range(8):
        if os.path.exists(os.path.join(directory, '.git')):
            try:
                commit = subprocess.run(['git', '-C', directory, 'rev-parse', '--short=12', 'HEAD'],
                    check=True, capture_output=True, text=True, timeout=1.0).stdout.strip()
                dirty = bool(subprocess.run(['git', '-C', directory, 'status', '--porcelain',
                    '--untracked-files=no'], check=True, capture_output=True,
                    text=True, timeout=1.0).stdout.strip())
                return {'package_version': version, 'git_commit': commit[:12] or 'unknown',
                        'git_dirty': dirty}
            except (OSError, subprocess.SubprocessError):
                break
        parent = os.path.dirname(directory)
        if parent == directory:
            break
        directory = parent
    return {'package_version': version, 'git_commit': 'unavailable', 'git_dirty': None}


def scene_identity(package_path, scene_id):
    if type(scene_id) is not int or not 0 <= scene_id <= 6:
        raise ValueError('invalid operator scene')
    path = os.path.join(package_path, 'worlds', 'scene_{}.world'.format(scene_id))
    digest = hashlib.sha256()
    try:
        with open(path, 'rb') as stream:
            for block in iter(lambda: stream.read(65536), b''):
                digest.update(block)
    except OSError:
        return {'scene_id': scene_id, 'world_sha256': 'unavailable'}
    return {'scene_id': scene_id, 'world_sha256': digest.hexdigest()}
