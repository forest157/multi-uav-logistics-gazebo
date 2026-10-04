"""Validated outdoor geometry shared by route planning and future flight nodes.

No world is accepted from its JSON alone: the SDF, generated layout and saved
metadata must still agree before its bounds or obstacles can be used.
"""
import json
import math
import os
import numpy as np

from .outdoor_preflight import metadata_for_world
from .worlds import OUTDOOR_LAYOUTS


def load_outdoor_profile(name, world_dir):
    if name not in OUTDOOR_LAYOUTS:
        raise ValueError('unknown outdoor world: {}'.format(name))
    world_path = os.path.join(world_dir, name + '.world')
    metadata_path = os.path.join(world_dir, name + '.json')
    verified = metadata_for_world(name, world_path)
    if (verified['world_collision_count'] != len(verified['no_fly_volumes']) or
            verified['external_include_uris'] !=
            ['model://sun', 'model://ground_plane']):
        raise ValueError('outdoor world contains unmodelled collision geometry')
    expected = json.loads(json.dumps(dict(OUTDOOR_LAYOUTS[name], **verified)))
    with open(metadata_path, encoding='utf-8') as stream:
        saved = json.load(stream)
    if saved != expected:
        raise ValueError('outdoor metadata differs from SDF or generated layout')
    extent_x, extent_y = verified['layout_extent_m']
    return {
        'name': name,
        'world_path': world_path,
        'world_sha256': verified['world_sha256'],
        'xy_half_extent_m': [extent_x / 2.0, extent_y / 2.0],
        'spawn_positions_m': verified['spawn_positions_m'],
        'spawn_center_m': verified['spawn_center_m'],
        'spawn_spacing_m': verified['spawn_spacing_m'],
        'goal_center_m': verified['goal_center_m'],
        'arrival_positions_m': verified['arrival_positions_m'],
        'recommended_cruise_altitude_m': verified['recommended_cruise_altitude_m'],
        'obstacles': [
            {'kind': 'box', 'label': 'building_{}'.format(index),
             'x': volume['center_xy_m'][0], 'y': volume['center_xy_m'][1],
             'half_x': volume['size_xy_m'][0] / 2.0,
             'half_y': volume['size_xy_m'][1] / 2.0,
             'height': volume['ceiling_m']}
            for index, volume in enumerate(verified['no_fly_volumes'])],
    }


def check_cruise_route(profile, path, horizontal_envelope_m,
                       below_envelope_m, above_envelope_m, sample_step_m=0.5,
                       minimum_altitude_m=3.0, maximum_altitude_m=45.0):
    """Reject a centre route unless the swept fleet envelope is clear.

    This is a geometric precheck, not a flight-controller or lidar acceptance.
    It deliberately fails closed for invalid geometry and non-finite numbers.
    """
    from .clearance_analyzer import horizontal_distance, sample_polyline

    margins = (horizontal_envelope_m, below_envelope_m, above_envelope_m,
               sample_step_m, minimum_altitude_m, maximum_altitude_m)
    if not all(math.isfinite(float(value)) for value in margins):
        raise ValueError('route limits must be finite')
    horizontal, below, above, step, z_min, z_max = map(float, margins)
    if horizontal <= 0 or below < 0 or above < 0 or step <= 0 or z_min >= z_max:
        raise ValueError('invalid route limits')
    points = np.asarray(path, dtype=float)
    if points.ndim != 2 or points.shape[1] != 3 or len(points) < 1:
        raise ValueError('route must contain xyz points')
    if not np.isfinite(points).all():
        raise ValueError('route coordinates must be finite')
    samples = sample_polyline(points, step)
    half_x, half_y = profile['xy_half_extent_m']
    # Every point on a sampled segment lies within step/2 of some sample.
    # Inflating every limit by that amount closes the inter-sample gap.
    sampling_reserve = step / 2.0
    for index, (x, y, z) in enumerate(samples):
        if (abs(x) + horizontal + sampling_reserve > half_x or
                abs(y) + horizontal + sampling_reserve > half_y):
            return {'feasible': False, 'error_code': 'E_BOUNDARY',
                    'sample_index': index, 'location': [float(x), float(y), float(z)]}
        if (z - below - sampling_reserve < z_min or
                z + above + sampling_reserve > z_max):
            return {'feasible': False, 'error_code': 'E_VERTICAL_CLEARANCE',
                    'sample_index': index, 'location': [float(x), float(y), float(z)]}
        for obstacle in profile['obstacles']:
            if (z - below - sampling_reserve <= obstacle['height'] and
                    horizontal_distance(obstacle, x, y) < horizontal + sampling_reserve):
                return {'feasible': False, 'error_code': 'E_STATIC_CLEARANCE',
                        'sample_index': index, 'location': [float(x), float(y), float(z)],
                        'obstacle': obstacle['label']}
    return {'feasible': True, 'error_code': None, 'sample_count': len(samples)}
