"""Match lidar scans to their measurement-time pose."""
import math


def valid_return(point, minimum=0.4, maximum=35.0):
    # Gazebo block laser publishes max-range misses as finite xyz coordinates.
    # Float32 rounding requires a small tolerance at the range endpoint.
    return all(math.isfinite(v) for v in point) and minimum <= math.sqrt(sum(v*v for v in point)) < maximum-0.001


def select_pose(history, stamp, now, consumed):
    if not all(math.isfinite(v) for v in (stamp, now, consumed)):
        return None
    if stamp <= consumed or stamp <= 0 or not 0 <= now-stamp <= 0.3 or not history:
        return None
    pose_stamp, pose = min(history, key=lambda item: abs(item[0]-stamp))
    return pose if abs(pose_stamp-stamp) <= 0.05 else None
