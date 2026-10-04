"""Generate a fail-closed three-UAV outdoor round-trip mission configuration."""

import math
import os

import numpy as np

from .formation_3d import (assign_slots, envelope, generate, initial_line,
                           minimum_separation)
from .formation_scheduler import safe_blend
from .outdoor_route_planner import plan_outdoor_route
from .outdoor_world_profile import check_cruise_route, load_outdoor_profile
from .trajectory_parameterization import toppra_parameterize_3d


def _checked(profile, points, offsets, description):
    # Check actual vehicle sweeps.  A radial fleet envelope would falsely
    # reject the 15 m-wide east/west pads near a north/south site boundary.
    centers = np.asarray(points, dtype=float)
    for vehicle, offset in enumerate(np.asarray(offsets, dtype=float)):
        result = check_cruise_route(profile, centers + offset,
                                    1.2, 0.6, 0.6)
        if not result['feasible']:
            raise ValueError('{} uav{}: {}'.format(description, vehicle,
                                                   result))


def _transition(profile, center, source, target, altitude, description):
    for ratio in np.linspace(0.0, 1.0, 101):
        offsets = safe_blend(source, target, ratio, 3.6)
        if minimum_separation(offsets) < 3.6 - 1e-6:
            raise ValueError('{}: fleet separation'.format(description))
        _checked(profile, [[center[0], center[1], altitude]], offsets,
                 description)


def verify_outdoor_mission_config(config, name, world_dir, world_file,
                                  spawn_x, spawn_y, spawn_spacing_m,
                                  goal_x, goal_y, target_z):
    """Revalidate saved mission geometry against the selected Gazebo world."""
    profile = load_outdoor_profile(name, world_dir)
    if not os.path.samefile(world_file, profile['world_path']):
        raise ValueError('Gazebo world file differs from outdoor mission world')
    if config.get('world') != name or config.get('world_sha256') != profile['world_sha256']:
        raise ValueError('outdoor mission world hash is stale or mismatched')
    if int(config.get('vehicle_count', 0)) != 3:
        raise ValueError('outdoor mission requires exactly three UAVs')
    expected = (profile['spawn_center_m'][0], profile['spawn_center_m'][1],
                profile['spawn_spacing_m'], profile['goal_center_m'][0],
                profile['goal_center_m'][1], profile['recommended_cruise_altitude_m'])
    actual = (spawn_x, spawn_y, spawn_spacing_m, goal_x, goal_y, target_z)
    if not all(math.isfinite(float(value)) for value in actual):
        raise ValueError('non-finite outdoor launch coordinate')
    if not np.allclose(actual, expected, rtol=0.0, atol=0.01):
        raise ValueError('outdoor launch coordinates differ from verified world')
    for key, value in (('spawn_center_m', expected[:2]),
                       ('spawn_spacing_m', expected[2]),
                       ('goal_center_m', expected[3:5]),
                       ('cruise_altitude', expected[5])):
        if key not in config or not np.allclose(config[key], value,
                                                 rtol=0.0, atol=0.01):
            raise ValueError('outdoor mission {} differs from verified world'.format(key))
    formations = {}
    for key in ('initial_formation', 'cruise_formation', 'delivery_formation'):
        values = np.asarray(config[key], dtype=float)
        if values.shape != (3, 3) or not np.isfinite(values).all():
            raise ValueError('invalid outdoor {}'.format(key))
        # The triangle generator uses 0.866 instead of exact sqrt(3)/2;
        # MissionPlayer.safe_formation expands this sub-millimetre difference.
        if minimum_separation(values) < 3.6 - 0.001:
            raise ValueError('outdoor formation separation is too small')
        formations[key] = values
    if not np.allclose(formations['initial_formation'],
                       initial_line(3, profile['spawn_spacing_m']),
                       rtol=0.0, atol=0.01):
        raise ValueError('outdoor home formation differs from spawn pads')
    start = np.asarray(profile['spawn_center_m'], dtype=float)
    goal = np.asarray(profile['goal_center_m'], dtype=float) - start
    for key in ('center_trajectory', 'return_trajectory'):
        rows = np.asarray(config[key], dtype=float)
        if (rows.ndim != 2 or rows.shape[1] != 4 or len(rows) < 2 or
                not np.isfinite(rows).all() or not np.all(np.diff(rows[:, 0]) > 0)):
            raise ValueError('invalid outdoor {}'.format(key))
        expected_first, expected_last = ((np.zeros(2), goal)
                                         if key == 'center_trajectory'
                                         else (goal, np.zeros(2)))
        if (not np.allclose(rows[0, 1:3], expected_first, rtol=0.0, atol=0.25)
                or not np.allclose(rows[-1, 1:3], expected_last,
                                   rtol=0.0, atol=0.25)):
            raise ValueError('outdoor trajectory endpoints differ from world')
        world_points = rows[:, 1:4] + [start[0], start[1], 0.0]
        _checked(profile, world_points, formations['cruise_formation'], key)
    return profile


def outdoor_mission(name, world_dir, planner_executable, solve_seconds=3.0):
    profile = load_outdoor_profile(name, world_dir)
    if abs(profile['spawn_spacing_m'] - 15.0) > 1e-6:
        raise ValueError('unexpected outdoor spawn spacing')
    start = profile['spawn_center_m']
    goal = profile['goal_center_m']
    altitude = profile['recommended_cruise_altitude_m']
    initial = initial_line(3, profile['spawn_spacing_m'])
    cruise = assign_slots(initial, generate('triangle', 3, 3.6))
    delivery = assign_slots(cruise, generate('row', 3, 3.6))
    for label, center, offsets in (
            ('takeoff', start, initial), ('departure', start, cruise),
            ('goal cruise', goal, cruise), ('delivery', goal, delivery)):
        _checked(profile, [[center[0], center[1], altitude]], offsets, label)
    for label, center, source, target in (
            ('departure transition', start, initial, cruise),
            ('delivery transition', goal, cruise, delivery),
            ('return transition', goal, delivery, cruise),
            ('arrival transition', start, cruise, initial)):
        _transition(profile, center, source, target, altitude, label)

    route = plan_outdoor_route(name, world_dir, planner_executable,
                               solve_seconds)
    path = np.asarray(route['path_xyz_m'], dtype=float)
    _checked(profile, path, cruise, 'OMPL route')

    def blocked(point):
        return not _checked_point(point)

    def _checked_point(point):
        horizontal, below, above = envelope(cruise)
        return check_cruise_route(profile, [point], horizontal, below,
                                  above)['feasible']

    trajectory = toppra_parameterize_3d(path, blocked,
                                         velocity_limit=2.0,
                                         acceleration_limit=1.0,
                                         sample_period=0.1)
    positions = trajectory['positions']
    _checked(profile, positions, cruise, 'TOPPRA route')
    duration = float(trajectory['duration'])
    outbound_times = 18.0 + trajectory['times']
    outbound_end = 18.0 + duration
    delivery_row_end = outbound_end + 5.0
    delivery_lower_end = delivery_row_end + 3.0
    delivery_release_end = delivery_lower_end + 4.0
    delivery_raise_end = delivery_release_end + 3.0
    delivery_end = delivery_raise_end + 5.0
    return_end = delivery_end + duration
    arrival_end = return_end + 10.0
    descent_end = arrival_end + max(20.0, 1.5 * (altitude - 0.18) / 0.25)
    stages = {
        'hold_end': 8.0, 'departure_reconfigure_end': 18.0,
        'outbound_end': round(outbound_end, 3),
        'delivery_row_end': round(delivery_row_end, 3),
        'delivery_lower_end': round(delivery_lower_end, 3),
        'delivery_release_end': round(delivery_release_end, 3),
        'delivery_raise_end': round(delivery_raise_end, 3),
        'delivery_hold_end': round(delivery_end, 3),
        'return_end': round(return_end, 3),
        'arrival_reconfigure_start': round(return_end, 3),
        'arrival_reconfigure_end': round(arrival_end, 3),
        'descent_end': round(descent_end, 3),
    }
    relative = positions.copy()
    relative[:, 0] -= start[0]
    relative[:, 1] -= start[1]

    def rows(times, values):
        return [[round(float(t), 3)] + [round(float(v), 3) for v in point]
                for t, point in zip(times, values)]

    config = {
        'vehicle_count': 3,
        'world': name,
        'world_sha256': profile['world_sha256'],
        'spawn_center_m': start,
        'spawn_spacing_m': profile['spawn_spacing_m'],
        'goal_center_m': goal,
        'cruise_altitude': altitude,
        'landing_altitude': 0.18,
        'ready_altitude': max(2.0, altitude - 1.0),
        'initial_formation': initial.tolist(),
        'cruise_formation': cruise.tolist(),
        'delivery_formation': delivery.tolist(),
        'formation': {'type': 'triangle', 'spacing_m': 3.6,
                      'minimum_separation_m': 3.3,
                      'tracking_reserve_m': 0.3},
        'planner': {'method': 'verified_metric_ompl_informed_rrtstar_3d',
                    'map_representation': 'SDF_verified_inflated_building_boxes',
                    'independent_clearance_check': route['independent_clearance_check']},
        'trajectory_parameterization': {
            'method': 'toppra_3d', 'velocity_limit_mps': 2.0,
            'acceleration_limit_mps2': 1.0,
            'actual_max_speed_mps': round(trajectory['max_speed'], 3),
            'actual_max_acceleration_mps2': round(trajectory['max_acceleration'], 3),
            'constraints_relaxed': bool(trajectory['relaxed']),
            'sample_period_s': 0.1},
        'stages': stages,
        'center_trajectory': rows(outbound_times, relative),
        'return_trajectory': rows(delivery_end + trajectory['times'],
                                  relative[::-1]),
    }
    return config, {'world': name, 'world_sha256': profile['world_sha256'],
                    'waypoints': route['waypoint_count'],
                    'route_length_m': route['route_length_m'],
                    'trajectory_duration_s': round(duration, 3),
                    'mission_descent_end_s': stages['descent_end'],
                    'max_speed_mps': config['trajectory_parameterization']['actual_max_speed_mps'],
                    'max_acceleration_mps2': config['trajectory_parameterization']['actual_max_acceleration_mps2']}
