"""Prediction and time-indexed clearance checks for dynamic obstacles."""
import math

import numpy as np


class DynamicObstacleError(ValueError):
    pass


def validate_obstacle(value):
    required = ("id", "position", "velocity", "radius", "height")
    missing = [key for key in required if key not in value]
    if missing:
        raise DynamicObstacleError("missing fields: {}".format(",".join(missing)))
    position = np.asarray(value["position"], dtype=float)
    velocity = np.asarray(value["velocity"], dtype=float)
    if position.shape != (3,) or velocity.shape != (3,):
        raise DynamicObstacleError("position and velocity must contain xyz")
    radius, height = float(value["radius"]), float(value["height"])
    if not str(value["id"]) or radius <= 0.0 or height <= 0.0:
        raise DynamicObstacleError("id, radius and height must be positive")
    return {
        "id": str(value["id"]),
        "position": position,
        "velocity": velocity,
        "radius": radius,
        "height": height,
    }


def predict_position(obstacle, seconds):
    item = validate_obstacle(obstacle)
    return item["position"] + item["velocity"] * max(0.0, float(seconds))


def interpolate_timed_path(path, query_time):
    """Interpolate rows shaped [time, x, y, z]."""
    values = np.asarray(path, dtype=float)
    if values.ndim != 2 or values.shape[1] != 4 or len(values) < 2:
        raise DynamicObstacleError("timed path must contain at least two [t,x,y,z] rows")
    if np.any(np.diff(values[:, 0]) <= 0.0):
        raise DynamicObstacleError("path timestamps must increase")
    t = float(query_time)
    if t <= values[0, 0]:
        return values[0, 1:].copy()
    if t >= values[-1, 0]:
        return values[-1, 1:].copy()
    upper = int(np.searchsorted(values[:, 0], t))
    a, b = values[upper - 1], values[upper]
    ratio = (t - a[0]) / (b[0] - a[0])
    return a[1:] + ratio * (b[1:] - a[1:])


def obstacle_clearance(point, obstacle, seconds, vehicle_radius=1.2,
                       vehicle_half_height=0.6, safety_buffer=0.5):
    """Return signed 3D clearance; negative values represent overlap."""
    item = validate_obstacle(obstacle)
    center = item["position"] + item["velocity"] * max(0.0, float(seconds))
    point = np.asarray(point, dtype=float)
    horizontal = math.hypot(*(point[:2] - center[:2]))
    horizontal -= item["radius"] + float(vehicle_radius) + float(safety_buffer)
    obstacle_bottom = center[2] - item["height"] * 0.5
    obstacle_top = center[2] + item["height"] * 0.5
    vehicle_bottom = point[2] - float(vehicle_half_height)
    vehicle_top = point[2] + float(vehicle_half_height)
    vertical_gap = max(obstacle_bottom - vehicle_top, vehicle_bottom - obstacle_top)
    if vertical_gap > 0.0:
        return math.hypot(max(0.0, horizontal), vertical_gap)
    return horizontal


def assess_timed_path(path, obstacles, horizon=8.0, sample_period=0.2,
                      vehicle_radius=1.2, vehicle_half_height=0.6,
                      safety_buffer=0.5, warning_clearance=2.0):
    """Assess a vehicle path against constant-velocity obstacle predictions."""
    checked = [validate_obstacle(value) for value in obstacles]
    if not checked:
        return {"level": "SAFE", "minimum_clearance_m": None,
                "time_to_conflict_s": None, "obstacle_id": None,
                "critical_time_s": None, "critical_position": None}
    path_values = np.asarray(path, dtype=float)
    start = float(path_values[0, 0])
    end = min(float(path_values[-1, 0]), start + max(0.0, float(horizon)))
    count = max(2, int(math.ceil((end - start) / float(sample_period))) + 1)
    minimum = float("inf")
    critical = None
    first_conflict = None
    for query_time in np.linspace(start, end, count):
        point = interpolate_timed_path(path_values, query_time)
        seconds = float(query_time - start)
        for item in checked:
            clearance = obstacle_clearance(
                point, item, seconds, vehicle_radius,
                vehicle_half_height, safety_buffer)
            if clearance < minimum:
                minimum = clearance
                critical = (item["id"], seconds, point.tolist())
            if clearance <= 0.0 and first_conflict is None:
                first_conflict = seconds
    if minimum <= 0.0:
        level = "CRITICAL"
    elif minimum <= float(warning_clearance):
        level = "WARNING"
    else:
        level = "SAFE"
    return {
        "level": level,
        "minimum_clearance_m": round(float(minimum), 3),
        "time_to_conflict_s": (None if first_conflict is None
                               else round(float(first_conflict), 3)),
        "obstacle_id": critical[0],
        "critical_time_s": round(float(critical[1]), 3),
        "critical_position": [round(float(v), 3) for v in critical[2]],
    }
