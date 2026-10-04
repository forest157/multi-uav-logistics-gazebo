"""Independent outdoor acceptance from read-only Gazebo truth capture.

Truth data is used only offline; flight, radar, and planning nodes never see it.
"""

import math

from .static_world_map import primitive_distance


def audit_outdoor_flight(rows, profile, expect_single_bird=False,
                         vehicle_radius_m=1.2, bird_radius_m=0.75):
    metrics = dict(samples=0, flight_samples=0, valid_truth_samples=0,
                   fresh_lidar_samples=0, fresh_safety_samples=0,
                   minimum_truth_separation_m=None,
                   minimum_truth_static_clearance_m=None,
                   maximum_estimate_error_m=None,
                   maximum_settled_vertical_error_m=None,
                   settled_vertical_samples=0,
                   maximum_lidar_confirmed_targets=0,
                   unexpected_avoidance_samples=0, safety_error_samples=0,
                   bird_truth_samples=0, minimum_bird_clearance_m=None,
                   active_orca_samples=0, complete=False, disarmed=False)
    vertical = {str(i): dict(target=None, settled=False) for i in range(3)}
    start_x, start_y = profile['spawn_center_m']
    spacing = profile['spawn_spacing_m']

    def minimum(key, value):
        old = metrics[key]
        metrics[key] = value if old is None else min(old, value)

    def maximum(key, value):
        old = metrics[key]
        metrics[key] = value if old is None else max(old, value)

    for row in rows:
        metrics['samples'] += 1
        mission = row.get('mission') or {}
        states = row.get('flight_states') or {}
        metrics['complete'] = mission.get('state') == 'COMPLETE'
        metrics['disarmed'] = (len(states) == 3 and
                               all(not state.get('armed', True)
                                   for state in states.values()))
        if mission.get('state') in (None, 'INITIALIZING', 'READY', 'COMPLETE'):
            continue
        metrics['flight_samples'] += 1
        if mission.get('dynamic_action') == 'ORCA':
            metrics['active_orca_samples'] += 1
        if not expect_single_bird and mission.get('dynamic_action') not in ('NORMAL', None):
            metrics['unexpected_avoidance_samples'] += 1
        stamp = row.get('stamp')
        if stamp is None or not math.isfinite(stamp):
            continue
        perception = row.get('perception') or {}
        received = row.get('perception_receipt_stamp')
        if (received is not None and 0 <= stamp - received <= 1.0 and
                perception.get('state') == 'TRACKING'):
            metrics['fresh_lidar_samples'] += 1
            metrics['maximum_lidar_confirmed_targets'] = max(
                metrics['maximum_lidar_confirmed_targets'],
                int(perception.get('confirmed_targets', 0)))
        safety = (row.get('safety') or {}).get('logistics_fleet/safety')
        safety_stamp = row.get('safety_receipt_stamp')
        if (safety and safety_stamp is not None and
                0 <= stamp - safety_stamp <= 1.0):
            metrics['fresh_safety_samples'] += 1
            if int(safety.get('level', 3)) == 2:
                metrics['safety_error_samples'] += 1
        truth_stamp = row.get('truth_receipt_stamp')
        truth = row.get('truth_evaluation_only') or {}
        if truth_stamp is None or not 0 <= stamp - truth_stamp <= 0.2:
            continue
        names = ['iris{}'.format(i) for i in range(3)]
        if any(name not in truth for name in names):
            continue
        fleet = [truth[name]['position'] for name in names]
        if any(len(point) != 3 or not all(math.isfinite(v) for v in point)
               for point in fleet):
            continue
        metrics['valid_truth_samples'] += 1
        birds = [body['position'] for name, body in truth.items()
                 if 'bird' in name and len(body.get('position', [])) == 3]
        if birds:
            metrics['bird_truth_samples'] += 1
        for i, point in enumerate(fleet):
            for other in fleet[:i]:
                minimum('minimum_truth_separation_m', math.dist(point, other))
            for obstacle in profile['obstacles']:
                minimum('minimum_truth_static_clearance_m',
                        primitive_distance(point, obstacle) - vehicle_radius_m)
            for bird in birds:
                if all(math.isfinite(v) for v in bird):
                    minimum('minimum_bird_clearance_m',
                            math.dist(point, bird) - vehicle_radius_m - bird_radius_m)
            pose = (row.get('poses') or {}).get(str(i))
            if pose and abs(truth_stamp - pose[0]) <= 0.1:
                estimate = (pose[1] + start_x + (i - 1) * spacing,
                            pose[2] + start_y, pose[3])
                maximum('maximum_estimate_error_m',
                        math.dist(point, estimate))
            target = (row.get('targets') or {}).get(str(i))
            if target and abs(truth_stamp - target[0]) <= 0.1:
                state = vertical[str(i)]
                if state['target'] is None or abs(target[3] - state['target']) > 0.05:
                    state['target'] = target[3]
                    state['settled'] = False
                error = abs(point[2] - target[3])
                if not state['settled'] and error <= 0.5:
                    state['settled'] = True
                if state['settled']:
                    maximum('maximum_settled_vertical_error_m', error)
                    metrics['settled_vertical_samples'] += 1

    reasons = []
    flight = metrics['flight_samples']
    if not flight:
        reasons.append('no_flight_samples')
    for key, fraction in (('valid_truth_samples', 0.95),
                          ('fresh_lidar_samples', 0.95),
                          ('fresh_safety_samples', 0.95)):
        if metrics[key] < fraction * flight:
            reasons.append('insufficient_' + key)
    for key, threshold, lower in (
            ('minimum_truth_separation_m', 3.0, True),
            ('minimum_truth_static_clearance_m', 0.0, True),
            ('maximum_estimate_error_m', 1.0, False),
            ('maximum_settled_vertical_error_m', 1.0, False)):
        value = metrics[key]
        if value is None or (value < threshold if lower else value > threshold):
            reasons.append(key)
    for key in ('unexpected_avoidance_samples', 'safety_error_samples'):
        if metrics[key]:
            reasons.append(key)
    if expect_single_bird:
        if not metrics['bird_truth_samples']:
            reasons.append('bird_truth_missing')
        if metrics['minimum_bird_clearance_m'] is None or metrics['minimum_bird_clearance_m'] <= 0:
            reasons.append('minimum_bird_clearance_m')
        if not metrics['maximum_lidar_confirmed_targets']:
            reasons.append('lidar_bird_detection_missing')
        if not metrics['active_orca_samples']:
            reasons.append('orca_intervention_missing')
    else:
        for key in ('maximum_lidar_confirmed_targets', 'bird_truth_samples'):
            if metrics[key]:
                reasons.append(key)
    if not metrics['complete']:
        reasons.append('mission_not_complete')
    if not metrics['disarmed']:
        reasons.append('vehicles_not_disarmed')
    metrics['passed'] = not reasons
    metrics['failure_reasons'] = reasons
    metrics['scope'] = ('Sampled Gazebo truth and conservative sphere envelope; '
                        'not a continuous contact-sensor proof')
    return metrics


def audit_no_bird_outdoor(rows, profile, vehicle_radius_m=1.2):
    return audit_outdoor_flight(rows, profile,
                                vehicle_radius_m=vehicle_radius_m)
