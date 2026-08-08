"""Deterministic 3D formation-clearance analysis for reference scenarios."""
import math

import numpy as np

from logistics_gazebo_sim_ros.formation_3d import envelope, generate
from logistics_gazebo_sim_ros.scenes import ORIGIN, SCALE, SCENES

CENTER_XY_LIMIT = 46.0
WORLD_XY_LIMIT = 50.0
WORLD_Z_MIN = 3.0
WORLD_Z_MAX = 45.0


def obstacle_primitives(scene):
    """Convert scene obstacles to metric rectangle/cylinder primitives."""
    result = []
    for obstacle in scene["obstacles"]:
        height = float(obstacle["height"])
        if obstacle["kind"] == "cylinder":
            result.append({
                "kind": "cylinder", "label": obstacle["label"],
                "x": (float(obstacle["x"]) - ORIGIN) * SCALE,
                "y": (float(obstacle["y"]) - ORIGIN) * SCALE,
                "radius": float(obstacle["radius"]) * SCALE,
                "height": height,
            })
            continue
        rectangles = ([(obstacle["x"], obstacle["y"], obstacle["w"], obstacle["d"])]
                      if obstacle["kind"] == "box" else obstacle["rects"])
        for index, (x, y, width, depth) in enumerate(rectangles):
            result.append({
                "kind": "box",
                "label": obstacle["label"] if len(rectangles) == 1
                         else "{}_part{}".format(obstacle["label"], index + 1),
                "x": (float(x) + float(width) / 2.0 - ORIGIN) * SCALE,
                "y": (float(y) + float(depth) / 2.0 - ORIGIN) * SCALE,
                "half_x": float(width) * SCALE / 2.0,
                "half_y": float(depth) * SCALE / 2.0,
                "height": height,
            })
    return result


def horizontal_distance(primitive, x, y):
    """Unsigned XY distance to a primitive footprint; zero means inside."""
    if primitive["kind"] == "cylinder":
        return max(0.0, math.hypot(x - primitive["x"], y - primitive["y"])
                   - primitive["radius"])
    dx = max(abs(x - primitive["x"]) - primitive["half_x"], 0.0)
    dy = max(abs(y - primitive["y"]) - primitive["half_y"], 0.0)
    return math.hypot(dx, dy)


def sample_polyline(path, step=0.5):
    """Sample all path segments, retaining both endpoints."""
    if float(step) <= 0.0:
        raise ValueError("sample step must be positive")
    values = np.asarray(path, dtype=float)
    if values.ndim != 2 or values.shape[1] != 3 or len(values) < 1:
        raise ValueError("path must contain one or more xyz points")
    samples = [values[0]]
    for start, goal in zip(values[:-1], values[1:]):
        length = float(np.linalg.norm(goal - start))
        count = max(1, int(math.ceil(length / float(step))))
        samples.extend(start + (goal - start) * (index / float(count))
                       for index in range(1, count + 1))
    return np.asarray(samples)


def _failure(code, message, point, available, required, obstacle=None,
             suggestions=None):
    return {
        "feasible": False,
        "error_code": code,
        "message": message,
        "location": [round(float(value), 3) for value in point],
        "obstacle": obstacle,
        "available": {key: round(float(value), 3)
                      for key, value in available.items()},
        "required": {key: round(float(value), 3)
                     for key, value in required.items()},
        "suggestions": list(suggestions or []),
    }


def analyze_path(scene_id, path, formation="triangle", vehicle_count=3,
                 spacing=3.0, sample_step=0.5, vehicle_radius=1.2,
                 vertical_radius=0.6):
    """Check a centre path against world, obstacle and formation envelopes."""
    if scene_id not in SCENES:
        return _failure("E_SCENE", "场景编号无效", (0, 0, 0), {}, {})
    try:
        offsets = generate(formation, int(vehicle_count), float(spacing))
    except (TypeError, ValueError) as exc:
        return _failure("E_FORMATION", str(exc), (0, 0, 0), {}, {})
    horizontal, below, above = envelope(
        offsets, vehicle_radius=vehicle_radius, vertical_radius=vertical_radius)
    required = {"horizontal_m": horizontal, "below_m": below, "above_m": above}
    primitives = obstacle_primitives(SCENES[scene_id])
    samples = sample_polyline(path, sample_step)
    minimum_horizontal = float("inf")
    minimum_floor = float("inf")
    minimum_ceiling = float("inf")
    critical = None

    for point in samples:
        x, y, z = map(float, point)
        boundary_xy = WORLD_XY_LIMIT - max(abs(x), abs(y))
        floor = z - WORLD_Z_MIN
        ceiling = WORLD_Z_MAX - z
        minimum_floor = min(minimum_floor, floor)
        minimum_ceiling = min(minimum_ceiling, ceiling)
        if max(abs(x),abs(y)) > CENTER_XY_LIMIT:
            return _failure(
                "E_BOUNDARY", "编队中心超出规划安全边界", point,
                {"horizontal_m": boundary_xy, "below_m": floor,
                 "above_m": ceiling}, required,
                suggestions=["move_start_or_goal"])
        if boundary_xy < horizontal:
            return _failure(
                "E_BOUNDARY", "编队超出水平安全边界", point,
                {"horizontal_m": boundary_xy, "below_m": floor,
                 "above_m": ceiling}, required,
                suggestions=["move_start_or_goal", "use_compact_formation"])
        if floor < below or ceiling < above:
            return _failure(
                "E_VERTICAL_CLEARANCE", "编队超出允许飞行高度范围", point,
                {"horizontal_m": boundary_xy, "below_m": floor,
                 "above_m": ceiling}, required,
                suggestions=["adjust_altitude", "use_flat_formation"])

        for primitive in primitives:
            distance = horizontal_distance(primitive, x, y)
            vertical_overlap = z - below <= primitive["height"]
            if vertical_overlap and distance < minimum_horizontal:
                minimum_horizontal = distance
                critical = (point.copy(), primitive["label"])
            if vertical_overlap and distance < horizontal:
                available = {
                    "horizontal_m": distance,
                    "below_m": z - primitive["height"],
                    "above_m": ceiling,
                }
                return _failure(
                    "E_CORRIDOR_TOO_NARROW",
                    "路径在当前高度无法容纳完整编队", point, available,
                    required, obstacle=primitive["label"],
                    suggestions=["increase_altitude", "use_column",
                                 "use_vertical_formation", "replan_path"])

    if math.isinf(minimum_horizontal):
        minimum_horizontal = min(
            WORLD_XY_LIMIT - max(abs(float(p[0])), abs(float(p[1])))
            for p in samples)
        critical = (samples[int(np.argmin([
            WORLD_XY_LIMIT - max(abs(float(p[0])), abs(float(p[1])))
            for p in samples]))], "world_boundary")

    point, label = critical
    return {
        "feasible": True,
        "error_code": None,
        "message": "路径满足当前三维队形包络",
        "formation": formation,
        "vehicle_count": int(vehicle_count),
        "sample_count": int(len(samples)),
        "sample_step_m": float(sample_step),
        "minimum_horizontal_clearance_m": round(float(minimum_horizontal), 3),
        "minimum_floor_clearance_m": round(float(minimum_floor), 3),
        "minimum_ceiling_clearance_m": round(float(minimum_ceiling), 3),
        "critical_location": [round(float(value), 3) for value in point],
        "critical_obstacle": label,
        "required": {key: round(float(value), 3)
                     for key, value in required.items()},
    }


def analyze_candidates(scene_id, path, formations, vehicle_count=3,
                       spacing=3.0, sample_step=0.5):
    """Evaluate candidate formations without hiding individual failures."""
    reports = {
        name: analyze_path(scene_id, path, name, vehicle_count, spacing,
                           sample_step)
        for name in formations
    }
    feasible = [name for name in formations if reports[name]["feasible"]]
    return {
        "feasible": bool(feasible),
        "recommended_formation": feasible[0] if feasible else None,
        "feasible_formations": feasible,
        "reports": reports,
    }


def analyze_ground_capacity(scene_id, xy, formation="column", vehicle_count=3,
                            spacing=3.0, error_code="E_START_CAPACITY"):
    """Check that a complete formation footprint fits at takeoff/landing."""
    if scene_id not in SCENES:
        return _failure("E_SCENE", "场景编号无效", (xy[0],xy[1],0), {}, {})
    offsets=generate(formation,int(vehicle_count),float(spacing))
    horizontal,_,_=envelope(offsets)
    x,y=map(float,xy);boundary=WORLD_XY_LIMIT-max(abs(x),abs(y))
    required={"horizontal_m":horizontal,"below_m":0.0,"above_m":0.0}
    if max(abs(x),abs(y))>CENTER_XY_LIMIT or boundary<horizontal:
        return _failure(error_code,"地面编队超出安全边界",(x,y,0),
                        {"horizontal_m":boundary},required,
                        suggestions=["move_start_or_goal","use_compact_formation"])
    for primitive in obstacle_primitives(SCENES[scene_id]):
        distance=horizontal_distance(primitive,x,y)
        if distance<horizontal:
            return _failure(error_code,"地面区域无法容纳完整编队",(x,y,0),
                            {"horizontal_m":distance},required,
                            obstacle=primitive["label"],
                            suggestions=["move_start_or_goal","use_compact_formation"])
    return {"feasible":True,"error_code":None,
            "message":"地面区域可容纳完整编队","formation":formation,
            "vehicle_count":int(vehicle_count),
            "location":[round(x,3),round(y,3),0.0],
            "required":{"horizontal_m":round(horizontal,3)},
            "available":{"horizontal_m":round(boundary,3)}}
