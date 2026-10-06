"""Conservative time-slot planning for a physically narrow outdoor corridor.

This is a kinematic schedule, not proof of PX4 tracking or continuous contact
avoidance. Only one subgroup is allowed in the corridor at a time.
"""

import math

from .outdoor_world_profile import check_cruise_route


def corridor_capacity(profile, entry_y=-55.0, exit_y=55.0,
                      altitude_m=18.0, separation_m=3.6,
                      vehicle_radius_m=1.2, maximum=8):
    """Largest centered horizontal row clear of every modeled static box."""
    if (not all(math.isfinite(value) for value in
                (entry_y, exit_y, altitude_m, separation_m, vehicle_radius_m)) or
            entry_y >= exit_y or separation_m < 3.0 or
            vehicle_radius_m <= 0.0 or not isinstance(maximum, int) or
            maximum < 1):
        raise ValueError('invalid corridor geometry or safety margin')
    start_x = profile['spawn_center_m'][0]
    best = 0
    for count in range(1, maximum + 1):
        offsets = [(index - (count - 1) / 2.0) * separation_m
                   for index in range(count)]
        clear = all(check_cruise_route(
            profile, [[start_x + x, entry_y, altitude_m],
                      [start_x + x, exit_y, altitude_m]],
            vehicle_radius_m, 0.6, 0.6)['feasible'] for x in offsets)
        if not clear:
            break
        best = count
    if best < 1:
        raise ValueError('no safe single-vehicle corridor')
    return best


def passage_plan(vehicle_count, profile, speed_mps=2.0,
                 guard_s=5.0, separation_m=3.6):
    """Partition by measured capacity; stagger non-overlapping corridor slots."""
    if (type(vehicle_count) is not int or not 1 <= vehicle_count <= 50 or
            not all(math.isfinite(value) and value > 0 for value in
                    (speed_mps, guard_s, separation_m))):
        raise ValueError('invalid fleet size, speed or guard')
    capacity = corridor_capacity(profile, separation_m=separation_m,
                                 maximum=min(vehicle_count, 8))
    duration = 110.0 / speed_mps
    groups = []
    for first in range(0, vehicle_count, capacity):
        group_id = len(groups)
        start = group_id * (duration + guard_s)
        groups.append({'group': group_id,
                       'vehicles': list(range(first, min(first + capacity, vehicle_count))),
                       'entry_s': round(start, 3),
                       'exit_s': round(start + duration, 3)})
    for earlier, later in zip(groups, groups[1:]):
        if later['entry_s'] - earlier['exit_s'] < guard_s - 0.002:
            raise ValueError('corridor slots overlap')
    return {'vehicle_count': vehicle_count, 'corridor_capacity': capacity,
            'minimum_separation_m': separation_m, 'speed_mps': speed_mps,
            'guard_s': guard_s, 'groups': groups,
            'last_exit_s': groups[-1]['exit_s'],
            'scope': 'kinematic corridor slots; no PX4 tracking claim'}


def stress_passage(profile, fleet_sizes=(20, 35, 50)):
    """Lightweight scheduling pressure check without spawning PX4 or Gazebo."""
    cases = []
    for count in fleet_sizes:
        plan = passage_plan(count, profile)
        groups = plan['groups']
        assigned = [vehicle for group in groups for vehicle in group['vehicles']]
        passed = (assigned == list(range(count)) and
                  all(len(group['vehicles']) <= plan['corridor_capacity']
                      for group in groups) and
                  all(b['entry_s'] > a['exit_s']
                      for a, b in zip(groups, groups[1:])))
        cases.append({'vehicles': count, 'groups': len(groups),
                      'last_exit_s': plan['last_exit_s'], 'passed': passed})
    return {'passed': all(case['passed'] for case in cases), 'cases': cases,
            'scope': 'kinematic scheduler only; no flight or sensor claim'}
