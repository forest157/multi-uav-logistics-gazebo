"""Non-standard cubic B-spline and TOPPRA parameterization ported from reference/logistics."""
import numpy as np
from scipy.interpolate import BSpline
import toppra as ta
import toppra.algorithm as algo
import toppra.constraint as constraint


class CubicBSplinePlanner:
    """Chord-length clamped spline used by the original logistics planner."""
    def __init__(self, control_points):
        self.cp = np.asarray(control_points, dtype=float)
        self.n = len(self.cp) - 1
        self.p = min(3, self.n)

    def _chord_params(self):
        if self.n <= 0:
            return np.array([0.0, 1.0])
        values = [0.0]
        length = 0.0
        for i in range(self.n):
            length += np.linalg.norm(self.cp[i + 1] - self.cp[i])
            values.append(length)
        return np.asarray(values) / length if length > 1e-6 else np.linspace(0, 1, self.n + 1)

    def knot_vector(self):
        n_val, p_val = self.n, self.p
        m = n_val + p_val + 1
        knots = np.zeros(m + 1)
        knots[m - p_val:m + 1] = 1.0
        if n_val > p_val:
            params = self._chord_params()
            for j in range(p_val + 1, m - p_val):
                knots[j] = sum(params[k] for k in range(j - p_val, j)
                               if k < len(params)) / p_val
        return knots


def _densify(points, max_segment=5.0):
    output = [np.asarray(points[0], dtype=float)]
    for start, end in zip(points, points[1:]):
        start, end = np.asarray(start, dtype=float), np.asarray(end, dtype=float)
        pieces = max(1, int(np.ceil(np.linalg.norm(end - start) / max_segment)))
        for index in range(1, pieces + 1):
            output.append(start + (end - start) * index / pieces)
    while len(output) < 4:
        longest = max(range(len(output) - 1),
                      key=lambda i: np.linalg.norm(output[i + 1] - output[i]))
        output.insert(longest + 1, (output[longest] + output[longest + 1]) / 2.0)
    return output


def _curve(control_points, samples=500):
    planner = CubicBSplinePlanner(control_points)
    parameter = np.linspace(0.0, 1.0, samples)
    knots = planner.knot_vector()
    values = np.column_stack([
        BSpline(knots, planner.cp[:, 0], planner.p)(parameter),
        BSpline(knots, planner.cp[:, 1], planner.p)(parameter),
    ])
    return parameter, values


def _safe(control_points, blocked):
    _, points = _curve(control_points, max(200, len(control_points) * 30))
    return all(not blocked(tuple(point)) for point in points)


def smooth_control_points(path, blocked):
    """Preserve the original iterative insertion and conservative Laplacian smoothing."""
    control = _densify(path)
    for _ in range(20):
        if _safe(control, blocked):
            break
        refined = [control[0]]
        for start, end in zip(control, control[1:]):
            refined.extend(((start + end) / 2.0, end))
        control = refined
        if len(control) > 400:
            raise RuntimeError("B-spline cannot be made collision-free")
    else:
        raise RuntimeError("B-spline smoothing did not converge")
    if not _safe(control, blocked):
        raise RuntimeError("B-spline intersects inflated obstacles")
    for _ in range(30):
        old = np.asarray(control)
        candidate = old.copy()
        candidate[1:-1] += 0.2 * ((old[:-2] + old[2:]) / 2.0 - old[1:-1])
        if not _safe(candidate, blocked):
            break
        control = list(candidate)
    return np.asarray(control)


def toppra_parameterize(path, blocked, velocity_limit=2.0,
                        acceleration_limit=1.0, sample_period=0.1):
    control = smooth_control_points(path, blocked)
    path_parameter, waypoints = _curve(control, 500)
    geometric_path = ta.SplineInterpolator(path_parameter, waypoints)
    velocity_axis = velocity_limit / np.sqrt(2.0)
    acceleration_axis = acceleration_limit / np.sqrt(2.0)

    def solve(v_axis, a_axis):
        velocity = constraint.JointVelocityConstraint(
            np.array([[-v_axis, v_axis], [-v_axis, v_axis]]))
        acceleration = constraint.JointAccelerationConstraint(
            np.array([[-a_axis, a_axis], [-a_axis, a_axis]]))
        return algo.TOPPRA([velocity, acceleration], geometric_path,
                           solver_wrapper="seidel").compute_trajectory(0.0, 0.0)

    trajectory = solve(velocity_axis, acceleration_axis)
    relaxed = False
    if trajectory is None:
        trajectory = solve(velocity_axis * 1.5, acceleration_axis * 1.5)
        relaxed = True
    if trajectory is None:
        raise RuntimeError("TOPPRA constraints are infeasible")
    duration = float(trajectory.duration)
    times = np.arange(0.0, duration, sample_period)
    if not len(times) or duration - times[-1] > 1e-6:
        times = np.append(times, duration)
    positions = np.asarray(trajectory(times, 0))
    velocities = np.asarray(trajectory(times, 1))
    accelerations = np.asarray(trajectory(times, 2))
    return {
        "control_points": control,
        "times": times,
        "positions": positions,
        "velocities": velocities,
        "accelerations": accelerations,
        "duration": duration,
        "relaxed": relaxed,
        "max_speed": float(np.max(np.linalg.norm(velocities, axis=1))),
        "max_acceleration": float(np.max(np.linalg.norm(accelerations, axis=1))),
    }


def toppra_parameterize_3d(waypoints, blocked, velocity_limit=2.0,
                           acceleration_limit=1.0, sample_period=0.1):
    """TOPPRA parameterization of an OMPL-smoothed xyz path."""
    points=np.asarray(waypoints,dtype=float)
    if len(points)<4:
        raise RuntimeError("OMPL path has too few states")
    segment=np.linalg.norm(np.diff(points,axis=0),axis=1)
    chord=np.concatenate(([0.0],np.cumsum(segment)))
    if chord[-1]<1e-6:
        raise RuntimeError("OMPL path has zero length")
    parameter=chord/chord[-1]
    geometric_path=ta.SplineInterpolator(parameter,points)
    velocity_axis=velocity_limit/np.sqrt(3.0)
    acceleration_axis=acceleration_limit/np.sqrt(3.0)

    def solve(v_axis,a_axis):
        velocity=constraint.JointVelocityConstraint(
            np.tile([-v_axis,v_axis],(3,1)))
        acceleration=constraint.JointAccelerationConstraint(
            np.tile([-a_axis,a_axis],(3,1)))
        return algo.TOPPRA([velocity,acceleration],geometric_path,
                           solver_wrapper="seidel").compute_trajectory(0.0,0.0)

    trajectory=solve(velocity_axis,acceleration_axis)
    relaxed=False
    if trajectory is None:
        trajectory=solve(velocity_axis*1.5,acceleration_axis*1.5)
        relaxed=True
    if trajectory is None:
        raise RuntimeError("TOPPRA 3D constraints are infeasible")
    duration=float(trajectory.duration)
    times=np.arange(0.0,duration,sample_period)
    if not len(times) or duration-times[-1]>1e-6:
        times=np.append(times,duration)
    positions=np.asarray(trajectory(times,0))
    if any(blocked(tuple(point)) for point in positions):
        raise RuntimeError("TOPPRA spline leaves OMPL free space")
    velocities=np.asarray(trajectory(times,1))
    accelerations=np.asarray(trajectory(times,2))
    return {
        "times":times,"positions":positions,"velocities":velocities,
        "accelerations":accelerations,"duration":duration,"relaxed":relaxed,
        "max_speed":float(np.max(np.linalg.norm(velocities,axis=1))),
        "max_acceleration":float(np.max(np.linalg.norm(accelerations,axis=1))),
    }
