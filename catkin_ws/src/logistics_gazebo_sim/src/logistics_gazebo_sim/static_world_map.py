"""Static lidar exclusion primitives from a selected, verified world."""

import math

from .clearance_analyzer import obstacle_primitives
from .outdoor_world_profile import load_outdoor_profile
from .scenes import SCENES


def primitive_distance(position, primitive):
    """Metric 3D distance to a ground-based static collision primitive."""
    x, y, z = position
    if primitive['kind'] == 'box':
        dx = max(abs(x - primitive['x']) - primitive['half_x'], 0.0)
        dy = max(abs(y - primitive['y']) - primitive['half_y'], 0.0)
    elif primitive['kind'] == 'cylinder':
        dx = max(math.hypot(x - primitive['x'], y - primitive['y'])
                 - primitive['radius'], 0.0)
        dy = 0.0
    else:
        raise ValueError('unsupported static primitive')
    dz = max(-z, z - primitive['height'], 0.0)
    return math.sqrt(dx * dx + dy * dy + dz * dz)


def static_primitives_for_world(scene_id=None, outdoor_world=None,
                                world_dir=None):
    """Return known static geometry; reject absent or ambiguous map selection.

    This is configuration geometry, not a Gazebo truth feed.  Outdoor SDF,
    generated layout and saved metadata are cross-checked on every load.
    """
    if outdoor_world and scene_id is not None:
        raise ValueError('select either scene_id or outdoor_world, not both')
    if outdoor_world:
        if not world_dir:
            raise ValueError('outdoor world directory is required')
        return load_outdoor_profile(outdoor_world, world_dir)['obstacles']
    if scene_id is None:
        raise ValueError('lidar static world map is required')
    try:
        scene_id = int(scene_id)
    except (TypeError, ValueError) as exc:
        raise ValueError('invalid scene_id') from exc
    if scene_id not in SCENES:
        raise ValueError('invalid scene_id')
    return obstacle_primitives(SCENES[scene_id])
