"""Event-driven, one-group-at-a-time route for the scale yard.

The schedule is a geometric contract; the ROS driver must still verify fresh
telemetry and measured separation before advancing any phase.
"""

import math

from .outdoor_world_profile import check_cruise_route
from .scale_passage import corridor_capacity


CRUISE_Z = 18.0
ROW_SPACING = 3.6
STAGING_Y = 75.0
ARRIVAL_Y = 110.0
MIN_SEPARATION = 2.7


def phase_plan(count, profile):
    if type(count) is not int or count not in (1, 3, 5, 8):
        raise ValueError('flight supports 1, 3, 5 or 8 vehicles')
    starts = profile['spawn_positions_m'][:count]
    goals = profile['arrival_positions_m'][:count]
    if len(starts) != count or len(goals) != count:
        raise ValueError('world has too few pads')
    capacity = corridor_capacity(profile, maximum=count)
    groups = [list(range(first, min(first + capacity, count)))
              for first in range(0, count, capacity)]
    center_x = profile['spawn_center_m'][0]

    def row(group):
        return {vehicle: (center_x + (slot - (len(group) - 1) / 2.0) * ROW_SPACING)
                for slot, vehicle in enumerate(group)}

    phases = []
    def add(label, group, xy):
        phases.append({'name': label, 'vehicles': group[:],
                       'targets': {vehicle: [float(x), float(y), CRUISE_Z]
                                   for vehicle, (x, y) in xy.items()}})

    add('takeoff', list(range(count)),
        {i: tuple(starts[i]) for i in range(count)})
    for group in groups:
        compact = row(group)
        add('outbound_stage', group,
            {i: (starts[i][0], -STAGING_Y) for i in group})
        add('outbound_compact', group,
            {i: (compact[i], -STAGING_Y) for i in group})
        add('outbound_corridor', group,
            {i: (compact[i], STAGING_Y) for i in group})
        add('outbound_expand', group,
            {i: (goals[i][0], STAGING_Y) for i in group})
        add('outbound_arrive', group,
            {i: tuple(goals[i]) for i in group})
    for group in reversed(groups):
        compact = row(group)
        add('return_stage', group,
            {i: (goals[i][0], STAGING_Y) for i in group})
        add('return_compact', group,
            {i: (compact[i], STAGING_Y) for i in group})
        add('return_corridor', group,
            {i: (compact[i], -STAGING_Y) for i in group})
        add('return_expand', group,
            {i: (starts[i][0], -STAGING_Y) for i in group})
        add('return_arrive', group,
            {i: tuple(starts[i]) for i in group})

    positions = {i: [float(starts[i][0]), float(starts[i][1]), CRUISE_Z]
                 for i in range(count)}
    minimum = float('inf')
    for phase in phases:
        next_positions = positions.copy()
        next_positions.update(phase['targets'])
        for vehicle in phase['vehicles']:
            result = check_cruise_route(profile,
                                        [positions[vehicle], next_positions[vehicle]],
                                        1.2, 0.6, 0.6)
            if not result['feasible']:
                raise ValueError('{} vehicle {} fails {}'.format(
                    phase['name'], vehicle, result['error_code']))
        # Synchronized linear motion is a planning check, not a tracking proof.
        for step in range(101):
            fraction = step / 100.0
            sample = {i: [positions[i][axis] + fraction *
                          (next_positions[i][axis] - positions[i][axis])
                          for axis in range(3)] for i in range(count)}
            for left in range(count):
                for right in range(left + 1, count):
                    distance = math.dist(sample[left], sample[right])
                    minimum = min(minimum, distance)
                    if distance < MIN_SEPARATION:
                        raise ValueError('{} vehicles {} and {} too close'.format(
                            phase['name'], left, right))
        positions = next_positions
    return {'count': count, 'groups': groups, 'phases': phases,
            'planned_minimum_separation_m': minimum,
            'scope': 'geometric synchronized trajectory; measured flight required'}
