"""Metric outdoor layout checks before mission integration."""
import math
from .worlds import OUTDOOR_LAYOUTS


def preflight(name, clearance=3.5):
    if not math.isfinite(clearance) or clearance <= 0:
        raise ValueError('clearance must be positive and finite')
    layout = OUTDOOR_LAYOUTS[name]
    pads = layout['pads']
    center = [sum(p[a] for p in pads) / len(pads) for a in (0, 1)]
    spacing = pads[1][0] - pads[0][0]
    if spacing <= 0 or any(abs(p[1]-center[1]) > 1e-6 or
                            abs(p[0]-(center[0]+(i-1)*spacing)) > 1e-6
                            for i, p in enumerate(pads)):
        raise ValueError('pads must match the three-UAV spawn row')
    if spacing < 2 * clearance:
        raise ValueError('spawn spacing does not preserve clearance')
    # Arrival positions use the same row geometry as departure.
    destinations = [[layout['goal'][0]+offset*spacing, layout['goal'][1]]
                    for offset in (-1, 0, 1)]
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
