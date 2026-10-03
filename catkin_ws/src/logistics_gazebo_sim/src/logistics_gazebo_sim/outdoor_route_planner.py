"""Offline OMPL route planning against a verified metric outdoor world.

The result is deliberately not a mission file: runtime safety, lidar static
filtering, PX4 spawn and energy-return nodes must share this geometry first.
"""
import math
import os
import subprocess
import tempfile

import numpy as np

from .formation_3d import envelope, generate
from .outdoor_world_profile import check_cruise_route, load_outdoor_profile


def planner_inputs(profile, formation='triangle', vehicle_count=3,
                   spacing_m=3.0, reserve_m=0.5):
    if vehicle_count != 3 or not math.isfinite(spacing_m) or spacing_m <= 0:
        raise ValueError('current outdoor layouts support exactly three UAVs')
    if not math.isfinite(reserve_m) or reserve_m < 0.25:
        raise ValueError('planning reserve must cover postcheck sampling gap')
    horizontal, below, above = envelope(generate(formation, vehicle_count,
                                                  spacing_m))
    half_x, half_y = profile['xy_half_extent_m']
    bounds = (-half_x + horizontal + reserve_m,
              half_x - horizontal - reserve_m,
              -half_y + horizontal + reserve_m,
              half_y - horizontal - reserve_m)
    if bounds[0] >= bounds[1] or bounds[2] >= bounds[3]:
        raise ValueError('formation does not fit outdoor world')
    obstacles = [('box', box['x'], box['y'],
                  box['half_x'] + horizontal + reserve_m,
                  box['half_y'] + horizontal + reserve_m,
                  box['height'] + below + reserve_m)
                 for box in profile['obstacles']]
    # Keep this first route above the tallest roof.  OMPL may otherwise
    # shortcut through a building between discretely checked states.
    roof_top = max((box['height'] for box in profile['obstacles']), default=0.0)
    z_bounds = (max(3.0, roof_top) + below + reserve_m,
                45.0 - above - reserve_m)
    if z_bounds[0] >= z_bounds[1]:
        raise ValueError('no roof-clear outdoor altitude band')
    return {'xy_bounds_m': bounds,
            'z_bounds_m': z_bounds,
            'horizontal_envelope_m': horizontal,
            'below_envelope_m': below,
            'above_envelope_m': above,
            'obstacle_specs': obstacles}


def plan_outdoor_route(name, world_dir, planner_executable,
                       solve_seconds=3.0):
    if not math.isfinite(float(solve_seconds)) or solve_seconds <= 0:
        raise ValueError('solve time must be positive and finite')
    profile = load_outdoor_profile(name, world_dir)
    settings = planner_inputs(profile)
    start = profile['spawn_center_m']
    goal = profile['goal_center_m']
    altitude = profile['recommended_cruise_altitude_m']
    x_low, x_high, y_low, y_high = settings['xy_bounds_m']
    z_low, z_high = settings['z_bounds_m']
    for point in (start, goal):
        if not (x_low <= point[0] <= x_high and y_low <= point[1] <= y_high):
            raise ValueError('outdoor start or goal is outside fleet bounds')
    if not z_low <= altitude <= z_high:
        raise ValueError('recommended altitude is outside fleet bounds')
    if not os.path.isfile(planner_executable) or not os.access(planner_executable, os.X_OK):
        raise ValueError('OMPL planner executable is unavailable')
    with tempfile.TemporaryDirectory(prefix='outdoor_ompl_') as directory:
        obstacle_file = os.path.join(directory, 'obstacles.csv')
        output_file = os.path.join(directory, 'route.csv')
        with open(obstacle_file, 'w', encoding='utf-8') as stream:
            for row in settings['obstacle_specs']:
                stream.write(','.join(map(str, row)) + '\n')
        arguments = [planner_executable, start[0], start[1], altitude,
                     goal[0], goal[1], altitude, obstacle_file, output_file,
                     z_low, z_high, solve_seconds,
                     x_low, x_high, y_low, y_high]
        result = subprocess.run([str(value) for value in arguments],
                                capture_output=True, text=True,
                                timeout=max(5.0, solve_seconds + 5.0))
        if result.returncode:
            raise RuntimeError('OMPL failed: {}'.format(
                result.stderr.strip() or result.stdout.strip()))
        path = np.loadtxt(output_file, delimiter=',', ndmin=2)
    if path.ndim != 2 or path.shape[1] != 3 or len(path) < 2 or not np.isfinite(path).all():
        raise RuntimeError('OMPL returned an invalid 3D route')
    if (np.linalg.norm(path[0] - [*start, altitude]) > 0.25 or
            np.linalg.norm(path[-1] - [*goal, altitude]) > 0.25):
        raise RuntimeError('OMPL route endpoints differ from outdoor metadata')
    check = check_cruise_route(profile, path,
                               settings['horizontal_envelope_m'],
                               settings['below_envelope_m'],
                               settings['above_envelope_m'])
    if not check['feasible']:
        raise RuntimeError('OMPL route failed independent outdoor check: {}'.format(check))
    return {
        'world': name,
        'world_sha256': profile['world_sha256'],
        'scope': 'offline three-UAV centre route; no PX4 mission approval',
        'start_center_m': start,
        'goal_center_m': goal,
        'cruise_altitude_m': altitude,
        'xy_bounds_m': settings['xy_bounds_m'],
        'obstacle_count': len(settings['obstacle_specs']),
        'waypoint_count': len(path),
        'route_length_m': round(float(np.linalg.norm(np.diff(path, axis=0),
                                                     axis=1).sum()), 3),
        'independent_clearance_check': check,
        'path_xyz_m': path.tolist(),
    }
