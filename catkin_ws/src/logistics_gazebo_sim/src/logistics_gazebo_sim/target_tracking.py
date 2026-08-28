"""Dynamic-object detections and lightweight alpha-beta tracking."""
import math
import random


class TrackingError(ValueError):
    """Raised when a perception payload is malformed."""


def _vector3(value, name):
    if not isinstance(value, (list, tuple)) or len(value) != 3:
        raise TrackingError("{} must contain xyz".format(name))
    result = [float(axis) for axis in value]
    if not all(math.isfinite(axis) for axis in result):
        raise TrackingError("{} must be finite".format(name))
    return result


def validate_detection_payload(payload):
    if not isinstance(payload, dict):
        raise TrackingError("detection payload must be an object")
    stamp = float(payload.get("stamp", 0.0))
    if not math.isfinite(stamp) or stamp < 0.0:
        raise TrackingError("detection stamp is invalid")
    detections = []
    for raw in payload.get("detections", []):
        detections.append({
            "id": str(raw["id"]),
            "position": _vector3(raw["position"], "position"),
            "radius": max(0.0, float(raw.get("radius", 0.0))),
            "height": max(0.0, float(raw.get("height", 0.0))),
            "confidence": min(1.0, max(0.0, float(raw.get("confidence", 1.0)))),
        })
    return {"stamp": stamp, "frame_id": str(payload.get("frame_id", "map")),
            "detections": detections}


def simulate_detections(obstacles, observer_positions, maximum_range, noise_stddev,
                        dropout_probability, seed):
    """Generate deterministic range-limited detections from simulator truth."""
    rng = random.Random(seed)
    result = []
    for raw in obstacles:
        position = _vector3(raw["position"], "position")
        visible = any(math.sqrt(sum((a-b)**2 for a, b in zip(position, observer)))
                      <= maximum_range for observer in observer_positions)
        if not visible or rng.random() < dropout_probability:
            continue
        measured = [axis + rng.gauss(0.0, noise_stddev) for axis in position]
        result.append({"id": str(raw["id"]), "position": measured,
                       "radius": float(raw.get("radius", 0.0)),
                       "height": float(raw.get("height", 0.0)),
                       "confidence": max(0.05, 1.0-noise_stddev/max(0.1, maximum_range))})
    return result


class AlphaBetaTracker:
    """Per-ID constant-velocity tracker with bounded missed-detection lifetime."""
    def __init__(self, alpha=0.75, beta=0.2, maximum_age=1.0):
        self.alpha = float(alpha); self.beta = float(beta)
        self.maximum_age = float(maximum_age); self.tracks = {}

    def update(self, stamp, detections):
        stamp = float(stamp); seen = set()
        for detection in detections:
            identity = str(detection["id"]); measured = _vector3(detection["position"], "position")
            previous = self.tracks.get(identity)
            if previous is None or stamp <= previous["stamp"]:
                position, velocity = measured, [0.0, 0.0, 0.0]
            else:
                dt = stamp-previous["stamp"]
                predicted = [p+v*dt for p, v in zip(previous["position"], previous["velocity"])]
                residual = [m-p for m, p in zip(measured, predicted)]
                position = [p+self.alpha*r for p, r in zip(predicted, residual)]
                velocity = [v+self.beta*r/dt for v, r in zip(previous["velocity"], residual)]
            self.tracks[identity] = {"id": identity, "position": position, "velocity": velocity,
                "radius": float(detection.get("radius", 0.0)), "height": float(detection.get("height", 0.0)),
                "confidence": float(detection.get("confidence", 1.0)), "stamp": stamp, "last_observed": stamp, "observed": True}
            seen.add(identity)
        for identity, track in list(self.tracks.items()):
            age = stamp-track["last_observed"]
            if identity in seen: continue
            if age > self.maximum_age:
                del self.tracks[identity]; continue
            track["position"] = [p+v*(stamp-track["stamp"]) for p, v in zip(track["position"], track["velocity"])]
            track["stamp"] = stamp; track["confidence"] *= 0.8; track["observed"] = False
        return [dict(track) for _, track in sorted(self.tracks.items())]
