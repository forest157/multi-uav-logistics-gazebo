"""Bounded, constant-velocity track samples for the read-only operator view."""

import math


def _xyz(value):
    if not isinstance(value, (list, tuple)) or len(value) != 3:
        raise ValueError('track position and velocity must be xyz')
    if any(type(axis) not in (int, float) or not math.isfinite(axis) for axis in value):
        raise ValueError('track position and velocity must be finite')
    return [float(axis) for axis in value]


def summarize_predictions(obstacles, horizon_s, priority_ids=(), limit=8):
    """Return at most ``limit`` tracks, putting conflict objects first.

    Samples are in the same world frame as the risk monitor obstacle feed.
    These are constant-velocity extrapolations, not observed future positions.
    """
    if type(horizon_s) not in (int, float) or not math.isfinite(horizon_s) or horizon_s <= 0:
        raise ValueError('prediction horizon must be positive and finite')
    if type(limit) is not int or not 1 <= limit <= 8:
        raise ValueError('prediction track limit must be 1..8')
    horizon = min(float(horizon_s), 30.0)
    tracks = {}
    for obstacle in obstacles:
        if not isinstance(obstacle, dict):
            raise ValueError('invalid obstacle track')
        identifier = obstacle.get('id')
        if not isinstance(identifier, str) or not identifier or len(identifier) > 64 or identifier in tracks:
            raise ValueError('invalid or duplicate obstacle ID')
        position = _xyz(obstacle.get('position'))
        velocity = _xyz(obstacle.get('velocity'))
        observed = obstacle.get('observed', True)
        if type(observed) is not bool:
            raise ValueError('invalid observed state')
        tracks[identifier] = (position, velocity, observed)
    priorities = {identity: index for index, identity in enumerate(dict.fromkeys(priority_ids))}
    selected = sorted(tracks, key=lambda identity: (0 if identity in priorities else 1,
                                                     priorities.get(identity, 0), identity))[:limit]
    samples = []
    for identity in selected:
        position, velocity, observed = tracks[identity]
        path = []
        for seconds in (0.0, horizon / 2.0, horizon):
            point = [round(position[axis] + velocity[axis] * seconds, 2)
                     for axis in range(3)]
            if not all(math.isfinite(axis) for axis in point):
                raise ValueError('predicted track position is not finite')
            path.append([round(seconds, 2)] + point)
        samples.append({'id': identity, 'observed': observed, 'samples': path})
    return {'frame': 'world', 'model': 'constant_velocity',
            'horizon_s': round(horizon, 2), 'total_tracks': len(tracks),
            'truncated_tracks': len(tracks) - len(samples), 'tracks': samples}
