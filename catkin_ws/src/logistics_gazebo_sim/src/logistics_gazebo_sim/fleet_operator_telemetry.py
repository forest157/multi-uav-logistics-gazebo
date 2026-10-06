"""Small, bounded per-vehicle snapshot model for the rqt operator view.

MAVROS and energy callbacks may run concurrently. This module deliberately
contains no ROS or Qt imports so freshness and malformed data can be tested.
"""

import math
import threading

from .fleet_config import validate_vehicle_count


def _finite(value, lower=None, upper=None):
    if type(value) is bool:
        return None
    try:
        number = float(value)
    except (TypeError, ValueError):
        return None
    if (not math.isfinite(number) or
            (lower is not None and number < lower) or
            (upper is not None and number > upper)):
        return None
    return number


class FleetOperatorTelemetry:
    def __init__(self, count, stale_after_s=2.5):
        self.count = validate_vehicle_count(count)
        self.stale_after_s = _finite(stale_after_s, lower=0.1)
        if self.stale_after_s is None:
            raise ValueError('stale_after_s must be positive and finite')
        self.lock = threading.RLock()
        self.states = [None] * self.count
        self.poses = [None] * self.count
        self.batteries = [None] * self.count
        self.energy = [None] * self.count

    def _index(self, index):
        if type(index) is not int or not 0 <= index < self.count:
            raise ValueError('vehicle index out of range')
        return index

    def update_state(self, index, connected, armed, mode, now):
        index = self._index(index)
        stamp = _finite(now, lower=0.0)
        if stamp is None or type(connected) is not bool or type(armed) is not bool:
            return False
        with self.lock:
            self.states[index] = (stamp, connected, armed, str(mode)[:48])
        return True

    def update_pose(self, index, xyz, now):
        index = self._index(index)
        stamp = _finite(now, lower=0.0)
        if stamp is None or not isinstance(xyz, (tuple, list)) or len(xyz) != 3:
            return False
        position = [_finite(axis) for axis in xyz]
        if any(axis is None for axis in position):
            return False
        with self.lock:
            self.poses[index] = (stamp, position)
        return True

    def update_battery(self, index, fraction, now):
        index = self._index(index)
        stamp = _finite(now, lower=0.0)
        value = _finite(fraction, lower=0.0, upper=1.0)
        if stamp is None or value is None:
            return False
        with self.lock:
            self.batteries[index] = (stamp, value)
        return True

    def update_energy(self, report, now):
        stamp = _finite(now, lower=0.0)
        if stamp is None or not isinstance(report, dict):
            return False
        vehicles = report.get('vehicles')
        if not isinstance(vehicles, list) or len(vehicles) > self.count:
            return False
        updates = {}
        for item in vehicles:
            if not isinstance(item, dict):
                return False
            identifier = item.get('vehicle_id')
            if not isinstance(identifier, str) or not identifier.startswith('uav'):
                return False
            try:
                index = int(identifier[3:])
            except ValueError:
                return False
            if ('uav{}'.format(index) != identifier or not 0 <= index < self.count or
                    index in updates):
                return False
            updates[index] = {
                'remaining_wh': _finite(item.get('remaining_wh'), lower=0.0),
                'remaining_fraction': _finite(item.get('remaining_fraction'),
                                              lower=0.0, upper=1.0),
                'reserve_safe': item.get('reserve_safe') if type(
                    item.get('reserve_safe')) is bool else None,
            }
        with self.lock:
            for index, item in updates.items():
                self.energy[index] = (stamp, item)
        return True

    def snapshot(self, now):
        stamp = _finite(now, lower=0.0)
        if stamp is None:
            raise ValueError('snapshot time invalid')
        vehicles = []
        with self.lock:
            for index in range(self.count):
                state = self.states[index]
                pose = self.poses[index]
                battery = self.batteries[index]
                energy = self.energy[index]
                state_fresh = state is not None and 0 <= stamp - state[0] <= self.stale_after_s
                pose_fresh = pose is not None and 0 <= stamp - pose[0] <= self.stale_after_s
                battery_fresh = battery is not None and 0 <= stamp - battery[0] <= 5.0
                energy_fresh = energy is not None and 0 <= stamp - energy[0] <= 3.0
                if not state_fresh or not pose_fresh:
                    status = 'STALE'
                elif not state[1]:
                    status = 'DISCONNECTED'
                else:
                    status = 'ARMED' if state[2] else 'READY'
                model = energy[1] if energy_fresh else {}
                fraction = battery[1] if battery_fresh else model.get('remaining_fraction')
                vehicles.append({
                    'vehicle_id': 'uav{}'.format(index),
                    'status': status,
                    'connected': state[1] if state_fresh else None,
                    'armed': state[2] if state_fresh else None,
                    'mode': state[3] if state_fresh else None,
                    'position_local_m': [round(axis, 2) for axis in pose[1]]
                    if pose_fresh else None,
                    'battery_fraction': round(fraction, 4) if fraction is not None else None,
                    'battery_source': ('MAVROS' if battery_fresh else
                                       'MODEL' if model.get('remaining_fraction') is not None else None),
                    'remaining_wh': model.get('remaining_wh'),
                    'reserve_safe': model.get('reserve_safe'),
                    'state_age_s': round(max(0.0, stamp - state[0]), 2)
                    if state is not None else None,
                })
        return {'schema': 1, 'vehicle_count': self.count, 'vehicles': vehicles,
                'scope': 'MAVROS local position and state; battery source labelled'}


def validate_snapshot(report):
    """Reject malformed/unbounded ROS JSON before passing it to Qt widgets."""
    if not isinstance(report, dict) or report.get('schema') != 1:
        raise ValueError('unknown snapshot schema')
    if type(report.get('vehicle_count')) is not int:
        raise ValueError('snapshot count must be an integer')
    count = validate_vehicle_count(report.get('vehicle_count'))
    vehicles = report.get('vehicles')
    if not isinstance(vehicles, list) or len(vehicles) != count:
        raise ValueError('snapshot vehicle count mismatch')
    if [item.get('vehicle_id') if isinstance(item, dict) else None
            for item in vehicles] != ['uav{}'.format(i) for i in range(count)]:
        raise ValueError('snapshot vehicle IDs invalid')
    for item in vehicles:
        if item.get('status') not in ('STALE', 'DISCONNECTED', 'ARMED', 'READY'):
            raise ValueError('invalid vehicle status')
        position = item.get('position_local_m')
        if position is not None and (not isinstance(position, list) or len(position) != 3 or
                                     any(type(axis) not in (int, float) or
                                         _finite(axis) is None for axis in position)):
            raise ValueError('invalid position')
        fraction = item.get('battery_fraction')
        if fraction is not None and (type(fraction) not in (int, float) or
                                     _finite(fraction, 0.0, 1.0) is None):
            raise ValueError('invalid battery')
        for name in ('remaining_wh', 'state_age_s'):
            value = item.get(name)
            if value is not None and (type(value) not in (int, float) or
                                      _finite(value, 0.0) is None):
                raise ValueError('invalid {}'.format(name))
        if item.get('mode') is not None and (not isinstance(item['mode'], str) or
                                             len(item['mode']) > 48):
            raise ValueError('invalid mode')
        if item.get('battery_source') not in (None, 'MAVROS', 'MODEL'):
            raise ValueError('invalid battery source')
    return vehicles
