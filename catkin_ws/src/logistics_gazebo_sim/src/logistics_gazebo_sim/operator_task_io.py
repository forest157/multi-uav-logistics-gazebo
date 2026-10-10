"""Validated task presets and atomic, read-only operator report export."""

import json
import math
import os
import tempfile
from datetime import datetime, timezone


FORMATIONS = ('triangle', 'inverted', 'row', 'column', 'vertical', 'wedge3d', 'helix')
AVOIDANCE = ('collective_offset', 'orca3d', 'distributed_mpc')
PERCEPTION = ('perception', 'lidar', 'truth')


def _number(value, minimum, maximum):
    if type(value) not in (int, float) or not math.isfinite(value) or not minimum <= value <= maximum:
        raise ValueError('task coordinate or altitude is invalid')
    return float(value)


def validate_task(task):
    if not isinstance(task, dict) or task.get('schema') != 1:
        raise ValueError('unsupported task preset')
    scene = task.get('scene_id')
    if type(scene) is not int or not 0 <= scene <= 6:
        raise ValueError('invalid scene ID')
    points = []
    for key in ('start_m', 'goal_m'):
        value = task.get(key)
        if not isinstance(value, list) or len(value) != 2:
            raise ValueError('task start and goal must be XY pairs')
        points.append([_number(axis, -46.0, 46.0) for axis in value])
    altitude = _number(task.get('altitude_m'), 3.0, 45.0)
    formation = task.get('formation')
    avoidance = task.get('avoidance_mode')
    perception = task.get('perception_source')
    dynamic = task.get('dynamic_obstacles')
    orca_speed = _number(task.get('orca_max_speed_mps', 2.0), 0.5, 2.0)
    orca_timeout = _number(task.get('orca_command_timeout_s', 0.6), 0.3, 0.6)
    if formation not in FORMATIONS or avoidance not in AVOIDANCE or perception not in PERCEPTION or type(dynamic) is not bool:
        raise ValueError('invalid task mode')
    return {'schema': 1, 'scene_id': scene, 'start_m': points[0], 'goal_m': points[1],
            'altitude_m': altitude, 'formation': formation,
            'dynamic_obstacles': dynamic, 'avoidance_mode': avoidance,
            'perception_source': perception,
            'orca_max_speed_mps': orca_speed,
            'orca_command_timeout_s': orca_timeout}


def load_task(path):
    if os.path.getsize(path) > 65536:
        raise ValueError('task preset is too large')
    with open(path, 'r', encoding='utf-8') as stream:
        return validate_task(json.load(stream))


def write_json_atomic(path, payload):
    """Replace only the explicitly chosen file after a complete fsynced write."""
    directory = os.path.dirname(os.path.abspath(path))
    if not os.path.isdir(directory):
        raise ValueError('destination directory does not exist')
    data = json.dumps(payload, ensure_ascii=False, allow_nan=False, indent=2) + '\n'
    temporary = None
    try:
        with tempfile.NamedTemporaryFile('w', encoding='utf-8', dir=directory,
                                         prefix='.operator-', suffix='.tmp',
                                         delete=False) as stream:
            temporary = stream.name
            stream.write(data)
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(temporary, path)
    finally:
        if temporary is not None and os.path.exists(temporary):
            os.unlink(temporary)


def build_operator_report(task, planning, vehicles, risk, energy, events):
    task = validate_task(task)
    if not isinstance(planning, dict) or type(planning.get('approved')) is not bool:
        raise ValueError('invalid planning status')
    if not isinstance(events, list) or len(events) > 50:
        raise ValueError('invalid event history')
    bounded_events = []
    for event in events:
        if not isinstance(event, dict) or any(not isinstance(event.get(key), str)
                                               for key in ('source', 'level', 'title', 'guidance')):
            raise ValueError('invalid event')
        bounded_events.append({key: event[key][:320]
                               for key in ('source', 'level', 'title', 'guidance')})
    return {'schema': 1, 'kind': 'operator_ui_snapshot',
            'generated_at_utc': datetime.now(timezone.utc).isoformat(),
            'scope': 'Read-only operator snapshot; not a flight safety audit or PX4 truth record.',
            'task': task,
            'planning': {'approved': planning['approved'],
                         'status': str(planning.get('status', ''))[:160],
                         'detail': str(planning.get('detail', ''))[:700]},
            'vehicles': vehicles, 'dynamic_risk': risk, 'energy_return': energy,
            'events': bounded_events}
