"""Bounded, read-only presentation model for energy return recommendations."""

import math


def _number(value):
    if type(value) not in (int, float) or not math.isfinite(value):
        raise ValueError('energy advisory contains an invalid number')
    return float(value)


def validate_energy_advisory(report):
    if not isinstance(report, dict) or report.get('mode') != 'shadow' or report.get('control_applied') is not False:
        raise ValueError('unsupported energy advisory mode')
    level = report.get('fleet_level')
    if level not in ('NORMAL', 'LOW', 'CRITICAL', 'STALE'):
        raise ValueError('invalid fleet energy level')
    raw_age = report.get('energy_age_s')
    # The existing advisor emits Infinity before its first energy sample.
    age = None if level == 'STALE' and raw_age == math.inf else _number(raw_age)
    if age is not None and age < 0:
        raise ValueError('negative energy age')
    vehicles = report.get('vehicles')
    if not isinstance(vehicles, list) or len(vehicles) > 8:
        raise ValueError('invalid energy vehicle list')
    if level == 'STALE' and vehicles:
        raise ValueError('stale advisory must not show old vehicles')
    identifiers = []
    for vehicle in vehicles:
        if not isinstance(vehicle, dict):
            raise ValueError('invalid energy vehicle')
        identifier = vehicle.get('vehicle_id')
        if identifier not in ('uav{}'.format(index) for index in range(8)) or identifier in identifiers:
            raise ValueError('invalid energy vehicle ID')
        identifiers.append(identifier)
        if vehicle.get('level') not in ('NORMAL', 'LOW', 'CRITICAL'):
            raise ValueError('invalid vehicle energy level')
        for field in ('final_margin_wh', 'usable_margin_wh', 'required_to_land_wh'):
            _number(vehicle.get(field))
    slots = report.get('slot_assignments')
    order = report.get('landing_order')
    if not isinstance(slots, dict) or not isinstance(order, list):
        raise ValueError('invalid return assignment')
    if (any(key not in identifiers or type(slot) is not int or
            not 0 <= slot < len(vehicles) for key, slot in slots.items()) or
            len(set(slots.values())) != len(slots)):
        raise ValueError('invalid return slot')
    if len(order) != len(identifiers) or set(order) != set(identifiers):
        raise ValueError('invalid landing order')
    if level != 'STALE' and not vehicles:
        raise ValueError('non-stale advisory has no vehicles')
    return {
        'level': level,
        'age_s': age,
        'rows': [{
            'vehicle_id': vehicle['vehicle_id'],
            'level': vehicle['level'],
            'final_margin_wh': float(vehicle['final_margin_wh']),
            'required_to_land_wh': float(vehicle['required_to_land_wh']),
            'slot': slots.get(vehicle['vehicle_id']),
            'landing_rank': order.index(vehicle['vehicle_id']) + 1,
        } for vehicle in vehicles],
    }
