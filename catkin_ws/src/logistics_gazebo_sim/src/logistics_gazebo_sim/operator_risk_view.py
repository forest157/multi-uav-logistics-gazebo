"""Bounded, read-only view model for dynamic risk and predicted conflicts."""

import math


def _number(value, nonnegative=False):
    if type(value) not in (int, float) or not math.isfinite(value):
        raise ValueError('invalid risk number')
    number = float(value)
    if nonnegative and number < 0:
        raise ValueError('negative risk time or distance')
    return number


def _optional_number(value, nonnegative=False):
    return None if value is None else _number(value, nonnegative)


def _vehicle_id(value):
    return isinstance(value, str) and value in ('uav{}'.format(index) for index in range(8))


def validate_risk_report(report):
    if not isinstance(report, dict):
        raise ValueError('risk report must be an object')
    level = report.get('level')
    if level not in ('SAFE', 'WARNING', 'CRITICAL', 'STALE'):
        raise ValueError('invalid fleet risk level')
    count = report.get('obstacle_count')
    if type(count) is not int or not 0 <= count <= 1000:
        raise ValueError('invalid obstacle count')
    raw_rows = report.get('vehicle_reports')
    if not isinstance(raw_rows, list) or len(raw_rows) > 8:
        raise ValueError('invalid risk vehicle list')
    if level == 'STALE' and raw_rows:
        raise ValueError('stale risk report cannot show old predictions')
    algorithm = report.get('local_avoidance_algorithm')
    if algorithm is not None and algorithm not in ('collective_offset', 'orca3d', 'distributed_mpc'):
        raise ValueError('unknown local avoidance algorithm')
    avoidance = report.get('avoidance')
    if avoidance is not None and not isinstance(avoidance, dict):
        raise ValueError('invalid avoidance summary')
    viable = None if avoidance is None else avoidance.get('viable')
    if viable is not None and type(viable) is not bool:
        raise ValueError('invalid avoidance viability')
    rows = []
    seen = set()
    for raw in raw_rows:
        if not isinstance(raw, dict) or not _vehicle_id(raw.get('vehicle_id')):
            raise ValueError('invalid risk vehicle ID')
        identifier = raw['vehicle_id']
        if identifier in seen or raw.get('level') not in ('SAFE', 'WARNING', 'CRITICAL'):
            raise ValueError('duplicate vehicle or invalid vehicle risk level')
        seen.add(identifier)
        obstacle = raw.get('obstacle_id')
        if obstacle is not None and (not isinstance(obstacle, str) or
                                     not obstacle or len(obstacle) > 64):
            raise ValueError('invalid obstacle ID')
        point = raw.get('critical_position')
        if point is not None:
            if not isinstance(point, list) or len(point) != 3:
                raise ValueError('invalid closest-approach point')
            point = [_number(axis) for axis in point]
        rows.append({
            'vehicle_id': identifier,
            'level': raw['level'],
            'obstacle_id': obstacle,
            'clearance_m': _optional_number(raw.get('minimum_clearance_m')),
            'time_to_conflict_s': _optional_number(raw.get('time_to_conflict_s'), True),
            'closest_time_s': _optional_number(raw.get('critical_time_s'), True),
            'closest_position_m': point,
        })
    separation = report.get('fleet_separation')
    pair = None
    separation_m = None
    separation_ttc_s = None
    if separation is not None:
        if not isinstance(separation, dict):
            raise ValueError('invalid fleet separation')
        pair = separation.get('closest_pair')
        if pair is not None and (not isinstance(pair, list) or len(pair) != 2 or
                                 not all(_vehicle_id(value) for value in pair) or
                                 pair[0] == pair[1]):
            raise ValueError('invalid closest vehicle pair')
        separation_m = _optional_number(separation.get('minimum_separation_m'), True)
        separation_ttc_s = _optional_number(separation.get('time_to_conflict_s'), True)
    return {
        'level': level,
        'obstacle_count': count,
        'rows': rows,
        'closest_pair': pair,
        'minimum_separation_m': separation_m,
        'separation_ttc_s': separation_ttc_s,
        'algorithm': algorithm,
        'plan_viable': viable,
    }
