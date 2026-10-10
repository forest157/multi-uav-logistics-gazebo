"""Explicit operator algorithm scope and reproducible source/world identity."""

import hashlib
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


def algorithm_profile(name):
    try:
        return dict(ALGORITHM_PROFILES[name])
    except (KeyError, TypeError):
        raise ValueError('unsupported operator algorithm')


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
