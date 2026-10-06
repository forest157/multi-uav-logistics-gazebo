"""Metric outdoor layout checks before mission integration."""
import hashlib
import math
from xml.etree import ElementTree
from .worlds import OUTDOOR_LAYOUTS


def preflight(name, clearance=3.5):
    if not math.isfinite(clearance) or clearance <= 0:
        raise ValueError('clearance must be positive and finite')
    layout = OUTDOOR_LAYOUTS[name]
    pads = layout['pads']
    if len(pads) < 2:
        raise ValueError('outdoor layout requires at least two spawn pads')
    center = [sum(p[a] for p in pads) / len(pads) for a in (0, 1)]
    spacing = pads[1][0] - pads[0][0]
    middle = (len(pads) - 1) / 2.0
    if spacing <= 0 or any(abs(p[1]-center[1]) > 1e-6 or
                            abs(p[0]-(center[0]+(i-middle)*spacing)) > 1e-6
                            for i, p in enumerate(pads)):
        raise ValueError('pads must match the fleet spawn row')
    if spacing < 2 * clearance:
        raise ValueError('spawn spacing does not preserve clearance')
    # Arrival positions use the same row geometry as departure.
    destinations = [[layout['goal'][0]+(index-middle)*spacing, layout['goal'][1]]
                    for index in range(len(pads))]
    for x, y in list(pads) + destinations:
        if any(abs(v)+clearance > extent/2 for v, extent in zip((x,y), layout['extent_m'])):
            raise ValueError('fleet landing footprint exceeds layout boundary')
        for bx, by, width, depth, height in layout['blocks']:
            if abs(x-bx) <= width/2+clearance and abs(y-by) <= depth/2+clearance:
                raise ValueError('fleet landing footprint overlaps a building')
    return dict(world=name+'.world', coordinate_system='ENU', unit='metre',
                spawn_center_m=center, spawn_spacing_m=spacing, spawn_positions_m=pads,
                goal_center_m=layout['goal'], arrival_positions_m=destinations,
                layout_extent_m=layout['extent_m'], clearance_m=clearance,
                status='ground_layout_passed',
                limitation='No route, flight-controller or in-flight clearance approval')


def metadata_for_world(name, world_path):
    """Describe the generated SDF only after matching it to its source layout."""
    layout = OUTDOOR_LAYOUTS[name]
    ground = preflight(name)
    with open(world_path, 'rb') as stream:
        world_bytes = stream.read()
    root = ElementTree.fromstring(world_bytes)
    world = root.find('world')
    if root.get('version') != '1.6' or world is None or world.get('name') != name:
        raise ValueError('outdoor world name or SDF version differs from layout')
    models = {model.get('name'): model for model in world.findall('model')}
    if len(models) != len(world.findall('model')):
        raise ValueError('outdoor world has duplicate model names')

    def numbers(model, path):
        value = model.findtext(path)
        if value is None:
            raise ValueError('outdoor world is missing {}'.format(path))
        return [float(axis) for axis in value.split()]

    def equal(actual, expected):
        return len(actual) == len(expected) and all(
            math.isclose(a, b, rel_tol=0.0, abs_tol=0.001)
            for a, b in zip(actual, expected))

    buildings = [key for key in models if key.startswith('building_')]
    if len(buildings) != len(layout['blocks']):
        raise ValueError('outdoor world building count differs from layout')
    for index, (x, y, width, depth, height) in enumerate(layout['blocks']):
        model = models.get('building_{}'.format(index))
        if model is None or model.findtext('static') != 'true':
            raise ValueError('outdoor world building is missing or not static')
        pose = numbers(model, 'pose')
        collision = numbers(model, 'link/collision/geometry/box/size')
        visual = numbers(model, 'link/visual/geometry/box/size')
        if not (equal(pose, [x, y, height / 2.0, 0, 0, 0]) and
                equal(collision, [width, depth, height]) and
                equal(visual, collision)):
            raise ValueError('outdoor world building geometry differs from layout')
    if len([key for key in models if key.startswith('road_')]) != len(layout['roads']):
        raise ValueError('outdoor road count differs from layout')
    for index, (x, y, width, depth) in enumerate(layout['roads']):
        road = models.get('road_{}'.format(index))
        if road is None or road.find('link/collision') is not None:
            raise ValueError('outdoor road is missing or has collision')
        if not (equal(numbers(road, 'pose')[:2], [x, y]) and
                equal(numbers(road, 'link/visual/geometry/box/size')[:2],
                      [width, depth])):
            raise ValueError('outdoor road geometry differs from layout')
    for index, point in enumerate(layout['pads']):
        marker = models.get('start_zone' if index == 0 else 'uav_pad_{}'.format(index))
        if marker is None or not equal(numbers(marker, 'pose')[:2], point):
            raise ValueError('outdoor spawn marker differs from layout')
    goal_marker = models.get('goal_zone')
    if goal_marker is None or not equal(numbers(goal_marker, 'pose')[:2], layout['goal']):
        raise ValueError('outdoor goal marker differs from layout')

    height = max(block[4] for block in layout['blocks'])
    ground.update(
        schema_version=1,
        description=layout['description'],
        world_sha256=hashlib.sha256(world_bytes).hexdigest(),
        world_bytes=len(world_bytes),
        sdf_model_count=len(models),
        external_include_uris=[item.findtext('uri') for item in world.findall('include')],
        world_collision_count=len(world.findall('model/link/collision')),
        asset_origin='project_generated',
        recommended_cruise_altitude_m=layout.get('recommended_cruise_altitude_m', height + 3.0),
        maximum_building_height_m=height,
        maximum_supported_uavs=len(layout['pads']),
        # Buildings define the only restricted air volumes in these layouts.
        no_fly_volumes=[dict(center_xy_m=[x, y], size_xy_m=[width, depth],
                             floor_m=0.0, ceiling_m=height)
                        for x, y, width, depth, height in layout['blocks']],
        dynamic_obstacle_routes=[],
        dynamic_obstacles_enabled=False,
        mission_integration='pending_planner_and_flight_validation',
    )
    return ground
