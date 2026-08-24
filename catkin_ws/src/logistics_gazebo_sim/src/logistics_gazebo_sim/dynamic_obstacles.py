"""Prediction and time-indexed clearance checks for dynamic obstacles."""
import math
import bisect

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


def prediction_path(start, target, horizon):
    """Keep prediction coverage at the full horizon, including while holding."""
    start=np.asarray(start,dtype=float);target=np.asarray(target,dtype=float)
    duration=float(horizon)
    if start.shape!=(3,) or target.shape!=(3,):
        raise DynamicObstacleError("prediction endpoints must contain xyz")
    if duration<=0.0:
        raise DynamicObstacleError("prediction horizon must be positive")
    return [[0.0]+start.tolist(),[duration]+target.tolist()]


def minimum_spawn_clearance(proposed, vehicle_positions):
    proposed=np.asarray(proposed,dtype=float)
    positions=np.asarray(vehicle_positions,dtype=float)
    if proposed.shape!=(3,) or positions.ndim!=2 or positions.shape[1]!=3:
        raise DynamicObstacleError("spawn clearance requires xyz positions")
    if len(positions)==0:
        raise DynamicObstacleError("vehicle positions are required")
    return float(np.min(np.linalg.norm(positions-proposed,axis=1)))


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
    # Use Python's deterministic binary search for this short timestamp list.
    # A live delivery trial exposed a scalar-dispatch failure in NumPy's
    # searchsorted while this function was called from concurrent ROS loops.
    upper = bisect.bisect_right(values[:, 0].tolist(), t)
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
                      safety_buffer=0.5, warning_clearance=2.0,
                      critical_time_threshold=None):
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
    if critical_time_threshold is not None and float(critical_time_threshold)<0.0:
        raise DynamicObstacleError("critical time threshold must be non-negative")
    imminent=(first_conflict is not None and (critical_time_threshold is None or first_conflict<=float(critical_time_threshold)))
    if minimum <= 0.0 and imminent:
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
        "imminent_conflict": bool(imminent),
    }


class DynamicSafetyResponse:
    """Convert noisy risk levels into fleet-wide speed/hold commands."""
    def __init__(self, warning_scale=0.35, release_delay=2.0):
        self.warning_scale = float(warning_scale)
        self.release_delay = float(release_delay)
        self.hold_latched = False
        self.clear_since = None

    def reset(self):
        self.hold_latched = False
        self.clear_since = None

    def update(self, level, now):
        level = str(level).upper()
        now = float(now)
        if level == "CRITICAL":
            self.hold_latched = True
            self.clear_since = None
        elif self.hold_latched:
            if level == "SAFE":
                if self.clear_since is None:
                    self.clear_since = now
                if now-self.clear_since >= self.release_delay:
                    self.hold_latched = False
                    self.clear_since = None
            else:
                self.clear_since = None

        if self.hold_latched:
            return {"action": "HOLD", "speed_scale": 0.0,
                    "risk_level": level}
        if level == "WARNING":
            return {"action": "SLOW", "speed_scale": self.warning_scale,
                    "risk_level": level}
        return {"action": "NORMAL", "speed_scale": 1.0,
                "risk_level": level}


def shifted_path(path, offset):
    """Blend a collective xyz offset during the first 40 percent of the path."""
    values = np.asarray(path, dtype=float).copy()
    if values.ndim != 2 or values.shape[1] != 4 or len(values) < 2:
        raise DynamicObstacleError("timed path must contain at least two [t,x,y,z] rows")
    offset = np.asarray(offset, dtype=float)
    if offset.shape != (3,):
        raise DynamicObstacleError("avoidance offset must contain xyz")
    span = values[-1, 0]-values[0, 0]
    if span <= 0.0:
        raise DynamicObstacleError("path timestamps must increase")
    sample_times = np.linspace(values[0,0],values[-1,0],max(11,len(values)))
    sampled = np.asarray([interpolate_timed_path(values,t) for t in sample_times])
    values = np.column_stack((sample_times,sampled))
    ratios = np.minimum(1.0, (values[:, 0]-values[0, 0])/(0.4*span))
    values[:, 1:] += ratios[:, None]*offset[None, :]
    return values


def assess_fleet_separation(paths, horizon=8.0, sample_period=0.1,
                            minimum_separation=3.0, tracking_tolerance=0.05):
    """Sample synchronized paths and enforce pairwise 3D separation."""
    arrays = [np.asarray(path, dtype=float) for path in paths]
    if not arrays:
        raise DynamicObstacleError("at least one vehicle path is required")
    for value in arrays:
        if value.ndim != 2 or value.shape[1] != 4 or len(value) < 2:
            raise DynamicObstacleError(
                "timed path must contain at least two [t,x,y,z] rows")
    start = max(float(value[0, 0]) for value in arrays)
    end = min(min(float(value[-1, 0]) for value in arrays),
              start + max(0.0, float(horizon)))
    if end <= start:
        raise DynamicObstacleError("fleet paths do not share a prediction window")
    count = max(2, int(math.ceil(
        (end - start) / max(0.01, float(sample_period)))) + 1)
    closest = float("inf")
    closest_pair = None
    closest_time = None
    first_conflict = None
    required = float(minimum_separation)
    for query_time in np.linspace(start, end, count):
        positions = [interpolate_timed_path(value, query_time)
                     for value in arrays]
        for first in range(len(positions)):
            for second in range(first + 1, len(positions)):
                distance = float(np.linalg.norm(
                    positions[first] - positions[second]))
                if distance < closest:
                    closest = distance
                    closest_pair = ["uav{}".format(first),
                                    "uav{}".format(second)]
                    closest_time = float(query_time - start)
                if distance < required - float(tracking_tolerance) and first_conflict is None:
                    first_conflict = float(query_time - start)
    return {
        "safe": closest + float(tracking_tolerance) >= required,
        "minimum_separation_m": (None if closest_pair is None
                                  else round(closest, 3)),
        "required_separation_m": required,
        "tracking_tolerance_m": float(tracking_tolerance),
        "closest_pair": closest_pair,
        "closest_time_s": (None if closest_time is None
                           else round(closest_time, 3)),
        "time_to_conflict_s": (None if first_conflict is None
                               else round(first_conflict, 3)),
    }


def collective_avoidance_candidates(paths, lateral=3.5, vertical=3.0):
    """Generate rigid fleet offsets perpendicular to the mean route."""
    arrays = [np.asarray(path, dtype=float) for path in paths]
    if not arrays:
        raise DynamicObstacleError("at least one vehicle path is required")
    directions = np.asarray([value[-1, 1:3]-value[0, 1:3]
                             for value in arrays])
    direction = np.mean(directions, axis=0)
    norm = float(np.linalg.norm(direction))
    if norm < 1e-6:
        perpendicular = np.asarray([0.0, 1.0])
    else:
        perpendicular = np.asarray([-direction[1], direction[0]])/norm
    lateral_vector = float(lateral)*perpendicular
    return [
        [float(lateral_vector[0]), float(lateral_vector[1]), 0.0],
        [-float(lateral_vector[0]), -float(lateral_vector[1]), 0.0],
        [0.0, 0.0, float(vertical)],
        [0.0, 0.0, -float(vertical)],
        [2.0*float(lateral_vector[0]), 2.0*float(lateral_vector[1]), 0.0],
        [-2.0*float(lateral_vector[0]), -2.0*float(lateral_vector[1]), 0.0],
        [0.0, 0.0, 2.0*float(vertical)],
        [0.0, 0.0, -2.0*float(vertical)],
    ]


def validate_static_paths(scene_id, paths):
    """Validate actual vehicle xyz paths against scene geometry and limits."""
    from logistics_gazebo_sim.clearance_analyzer import analyze_path
    reports=[]
    for index,path in enumerate(paths):
        xyz=np.asarray(path,dtype=float)[:,1:]
        report=analyze_path(int(scene_id),xyz,formation="triangle",vehicle_count=1,spacing=3.0,center_xy_limit=50.0)
        report["vehicle_id"]="uav{}".format(index);reports.append(report)
    failed=next((value for value in reports if not value["feasible"]),None)
    return {"feasible":failed is None,"error_code":None if failed is None else failed["error_code"],
            "message":"all vehicle paths satisfy static constraints" if failed is None else failed["message"],
            "vehicle_id":None if failed is None else failed["vehicle_id"],
            "obstacle":None if failed is None else failed.get("obstacle"),"reports":reports}


def plan_collective_avoidance(paths,obstacles,candidate_offsets=None,horizon=8.0,
                              required_clearance=0.5,warning_clearance=2.0,
                              scene_id=None,minimum_separation=3.0, tracking_tolerance=0.05):
    """Select one rigid 3D offset safe for every vehicle, or reject all."""
    if not paths:raise DynamicObstacleError("at least one vehicle path is required")
    offsets=collective_avoidance_candidates(paths) if candidate_offsets is None else candidate_offsets
    viable=[];evaluated=[];rejection_summary={}
    for offset in offsets:
        shifted=[shifted_path(path,offset) for path in paths]
        reports=[assess_timed_path(path,obstacles,horizon=horizon,warning_clearance=warning_clearance) for path in shifted]
        clearances=[report["minimum_clearance_m"] for report in reports if report["minimum_clearance_m"] is not None]
        minimum=min(clearances) if clearances else float("inf")
        static=validate_static_paths(scene_id,shifted) if scene_id is not None else {"feasible":True,"error_code":None,"message":"static validation disabled"}
        separation=assess_fleet_separation(
            shifted,horizon=horizon,minimum_separation=minimum_separation,
            tracking_tolerance=tracking_tolerance)
        if any(report["level"]=="CRITICAL" for report in reports):rejection="DYNAMIC_CONFLICT"
        elif clearances and minimum<float(required_clearance):rejection="DYNAMIC_CLEARANCE"
        elif not separation["safe"]:rejection="VEHICLE_SEPARATION"
        elif not static["feasible"]:rejection=static["error_code"] or "STATIC_CONSTRAINT"
        else:rejection=None
        value={"offset":[round(float(v),3) for v in offset],
               "minimum_clearance_m":None if not clearances else round(float(minimum),3),
               "levels":[report["level"] for report in reports],
               "static_validation":static,"fleet_separation":separation,"rejection_reason":rejection}
        evaluated.append(value)
        if rejection is None:
            norm=float(np.linalg.norm(np.asarray(offset,dtype=float)));viable.append((norm,len(evaluated),value))
        else:rejection_summary[rejection]=rejection_summary.get(rejection,0)+1
    if not viable:return {"viable":False,"selected_offset":None,"minimum_clearance_m":None,
        "candidates":evaluated,"reason":"no collective offset satisfies dynamic, static and fleet separation constraints",
        "rejection_summary":rejection_summary}
    selected=min(viable,key=lambda item:(item[0],item[1]))[2]
    return {"viable":True,"selected_offset":selected["offset"],
            "minimum_clearance_m":selected["minimum_clearance_m"],"candidates":evaluated,
            "reason":"safe collective offset found","static_validation":selected["static_validation"],
            "fleet_separation":selected["fleet_separation"],
            "rejection_summary":rejection_summary}


class AvoidanceExecution:
    """Fail-safe state machine for a collectively validated xyz offset."""
    def __init__(self, confirmation_s=0.8, apply_s=2.0, recover_s=2.0,
                 candidate_tolerance=0.25):
        self.confirmation_s=float(confirmation_s);self.apply_s=float(apply_s)
        self.recover_s=float(recover_s);self.tolerance=float(candidate_tolerance)
        self.reset()

    def reset(self):
        self.state="IDLE";self.candidate=None;self.candidate_since=None
        self.active=np.zeros(3);self.start_offset=np.zeros(3)
        self.target=np.zeros(3);self.transition_started=0.0
        self.failure=None

    def _same(self, first, second):
        return first is not None and second is not None and (
            np.linalg.norm(np.asarray(first)-np.asarray(second))<=self.tolerance)

    def update(self, level, avoidance, now):
        now=float(now);level=str(level).upper();avoidance=avoidance or {}
        viable=bool(avoidance.get("viable"))
        proposed=avoidance.get("selected_offset") if viable else None
        constrained=bool((avoidance.get("static_validation") or {}).get(
            "feasible", viable))
        viable=viable and constrained and proposed is not None

        if level in ("WARNING","CRITICAL"):
            if not viable:
                self.state="HOLD";self.failure=avoidance.get(
                    "reason","avoidance candidate unavailable")
                self.candidate=None;self.candidate_since=None
                return self.command(now)
            if not self._same(self.candidate, proposed):
                if self.state in ("APPLYING","ACTIVE"):
                    self.state="HOLD";self.failure="validated candidate changed during avoidance"
                    return self.command(now)
                self.candidate=np.asarray(proposed,dtype=float)
                self.candidate_since=now
                if self.state not in ("APPLYING","ACTIVE"):
                    self.state="CONFIRMING"
                return self.command(now)
            if self.state=="CONFIRMING" and now-self.candidate_since>=self.confirmation_s:
                self.state="APPLYING";self.start_offset=self.active.copy()
                self.target=self.candidate.copy();self.transition_started=now
            elif self.state=="ACTIVE" and not self._same(self.target,proposed):
                self.state="APPLYING";self.start_offset=self.active.copy()
                self.target=np.asarray(proposed,dtype=float);self.transition_started=now
        elif level=="SAFE":
            self.candidate=None;self.candidate_since=None
            if self.state in ("APPLYING","ACTIVE","CONFIRMING"):
                self.state="RECOVERING";self.start_offset=self.active.copy()
                self.target=np.zeros(3);self.transition_started=now
            elif self.state=="HOLD":
                self.state="IDLE";self.failure=None;self.active=np.zeros(3)
        elif self.state in ("APPLYING","ACTIVE"):
            self.state="HOLD";self.failure="risk state stale during avoidance"

        return self.command(now)

    def command(self, now):
        now=float(now)
        if self.state=="APPLYING":
            ratio=min(1.0,max(0.0,(now-self.transition_started)/max(.01,self.apply_s)))
            ratio=ratio*ratio*(3.0-2.0*ratio)
            self.active=self.start_offset+ratio*(self.target-self.start_offset)
            if ratio>=1.0:self.state="ACTIVE"
        elif self.state=="RECOVERING":
            ratio=min(1.0,max(0.0,(now-self.transition_started)/max(.01,self.recover_s)))
            ratio=ratio*ratio*(3.0-2.0*ratio)
            self.active=self.start_offset*(1.0-ratio)
            if ratio>=1.0:self.state="IDLE";self.active=np.zeros(3)
        action=("HOLD" if self.state=="HOLD" else
                "AVOID" if self.state in ("APPLYING","ACTIVE","RECOVERING")
                else "WAIT")
        return {"state":self.state,"action":action,
                "offset":[round(float(v),3) for v in self.active],
                "failure":self.failure}
