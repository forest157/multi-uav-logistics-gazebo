"""Fail-closed review of v0.5 series evidence, separate from flight control.

This consumes saved offline reports only. Gazebo truth must never be routed to
the planner, radar detector, or flight controller.
"""

import hashlib
import json
import math
from pathlib import Path
import xml.etree.ElementTree as ET


def _number(value):
    return (isinstance(value, (int, float)) and not isinstance(value, bool)
            and math.isfinite(value))


def _at_least(report, key, limit, errors):
    value = report.get(key)
    if not _number(value) or value < limit:
        errors.append('{} < {}'.format(key, limit))


def _at_most(report, key, limit, errors):
    value = report.get(key)
    if not _number(value) or value > limit:
        errors.append('{} > {}'.format(key, limit))


def _count(report, key, errors, maximum=None):
    value = report.get(key)
    if (not isinstance(value, int) or isinstance(value, bool) or value < 0 or
            (maximum is not None and value > maximum)):
        errors.append('{} is not a valid count'.format(key))


def evaluate_flight(report, kind):
    """Re-evaluate numeric evidence; never accept only a saved passed flag."""
    errors = []
    flight = report.get('flight_samples')
    if not isinstance(flight, int) or isinstance(flight, bool) or flight < 1000:
        errors.append('insufficient flight_samples')
    else:
        for key in ('valid_truth_samples', 'fresh_lidar_samples',
                    'fresh_safety_samples'):
            _count(report, key, errors, flight)
            _at_least(report, key, 0.95 * flight, errors)
    _at_least(report, 'minimum_truth_separation_m', 3.0, errors)
    _at_least(report, 'minimum_truth_static_clearance_m', 0.1, errors)
    _at_most(report, 'maximum_estimate_error_m', 1.0, errors)
    _at_most(report, 'maximum_settled_vertical_error_m', 1.0, errors)
    _count(report, 'safety_error_samples', errors)
    _at_most(report, 'safety_error_samples', 0, errors)
    if report.get('complete') is not True or report.get('disarmed') is not True:
        errors.append('mission not complete and disarmed')
    if report.get('passed') is not True or report.get('failure_reasons') != []:
        errors.append('source audit did not pass cleanly')
    if kind == 'no_bird':
        for key in ('maximum_lidar_confirmed_targets', 'bird_truth_samples',
                    'unexpected_avoidance_samples'):
            _count(report, key, errors)
            _at_most(report, key, 0, errors)
    elif kind == 'single_bird':
        for key in ('bird_truth_samples', 'maximum_lidar_confirmed_targets',
                    'active_orca_samples'):
            _count(report, key, errors)
        _at_least(report, 'bird_truth_samples', 1, errors)
        _at_least(report, 'minimum_bird_clearance_m', 0.1, errors)
        _at_least(report, 'maximum_lidar_confirmed_targets', 1, errors)
        _at_least(report, 'active_orca_samples', 1, errors)
    else:
        errors.append('unknown flight kind')
    return errors


def evaluate_perception_matrix(report):
    errors = []
    cases = report.get('cases') or {}
    required = ('empty_air', 'single_target_noise', 'short_occlusion',
                'long_occlusion', 'two_target_crossing', 'perception_stale')
    if report.get('pass') is not True or report.get('case_count') != len(required):
        errors.append('matrix summary incomplete')
    if set(cases) != set(required) or any(cases.get(k, {}).get('pass') is not True for k in required):
        errors.append('required perception cases missing or failed')
        return errors
    if cases['empty_air'].get('track_samples') != 0:
        errors.append('empty air produced tracks')
    if len(cases['single_target_noise'].get('unique_ids') or []) != 1:
        errors.append('single target ID unstable')
    if cases['short_occlusion'].get('before_id') != cases['short_occlusion'].get('after_id'):
        errors.append('short occlusion lost ID')
    if cases['long_occlusion'].get('before_id') == cases['long_occlusion'].get('after_id'):
        errors.append('long occlusion incorrectly preserved ID')
    if len(set(cases['two_target_crossing'].get('track_ids') or [])) != 2:
        errors.append('two-target crossing not distinguished')
    if cases['perception_stale'].get('actions') != ['SLOW', 'HOLD', 'HOLD', 'NORMAL']:
        errors.append('stale recovery sequence incorrect')
    return errors


def evaluate_dropout(report):
    errors = []
    if report.get('passed') is not True or report.get('complete') is not True or report.get('disarmed') is not True:
        errors.append('dropout mission incomplete')
    _at_least(report, 'samples', 1000, errors)
    _at_least(report, 'valid_truth_samples', 0.95 * report.get('samples', 0), errors)
    _at_least(report, 'minimum_fleet_separation_m', 3.0, errors)
    _at_most(report, 'maximum_estimate_error_m', 1.0, errors)
    _at_most(report, 'maximum_settled_vertical_tracking_error_m', 1.0, errors)
    for key in ('dropout_slow_samples', 'dropout_hold_samples',
                'dropout_release_guard_samples'):
        _at_least(report, key, 1, errors)
    for key in ('bird_truth_samples', 'frames_with_perception_tracks',
                'unexpected_avoidance_samples'):
        _at_most(report, key, 0, errors)
    if report.get('dropout_recovered') is not True:
        errors.append('dropout did not recover')
    return errors


def evaluate_energy_and_safety(name, report):
    requirements = {
        'model': ('complete_requirement_zero', 'forecasts_present',
                  'mavros_battery_present', 'payload_released',
                  'used_energy_recorded'),
        'return': ('alternate_site_present', 'critical_alternate',
                   'low_return', 'multiple_critical_holds',
                   'safety_precedence', 'shadow_only', 'slots_present',
                   'stale_fails_safe'),
        'landing': ('control_applied', 'critical_diversion_observed',
                    'home_descent_reached', 'priority_order_observed'),
        'interlock': ('hold_triggered', 'released', 'targets_locked'),
    }
    if name not in requirements:
        return ['unknown energy or safety report']
    errors = []
    if report.get('pass') is not True:
        errors.append('source trial did not pass')
    for key in requirements[name]:
        if report.get(key) is not True:
            errors.append(key + ' not demonstrated')
    if name == 'model':
        if report.get('vehicle_count') != 3 or len(report.get('capacities_wh') or []) != 3:
            errors.append('three vehicle batteries missing')
    elif name == 'landing':
        heights = report.get('target_altitudes')
        if (not isinstance(heights, list) or len(heights) != 3 or
                not all(_number(value) for value in heights) or
                not heights[0] > heights[1] > heights[2]):
            errors.append('landing target order incorrect')
    elif name == 'interlock':
        _at_most(report, 'close_minimum_separation_m', 2.7, errors)
    return errors


def _safe_file(root, relative):
    if not isinstance(relative, str) or not relative or Path(relative).is_absolute():
        raise ValueError('unsafe evidence path')
    path = (root / relative).resolve()
    if root != path and root not in path.parents:
        raise ValueError('evidence path escapes repository')
    return path


def _load(root, relative):
    with _safe_file(root, relative).open(encoding='utf-8') as stream:
        return json.load(stream)


def validate_manifest(manifest):
    """Reject omitted evidence groups and weakened repeat requirements."""
    errors = []
    if not isinstance(manifest, dict):
        return ['manifest must be an object']
    if manifest.get('release') != 'v0.5.12' or manifest.get('baseline_tag') != 'v0.5.11':
        errors.append('release or baseline identity changed')
    minimum = manifest.get('minimum_runs_per_world')
    if not isinstance(minimum, int) or isinstance(minimum, bool) or minimum < 2:
        errors.append('minimum_runs_per_world must be at least 2')
    worlds = manifest.get('worlds')
    required_worlds = {'outdoor_campus', 'outdoor_residential', 'outdoor_urban'}
    if not isinstance(worlds, dict) or set(worlds) != required_worlds:
        errors.append('required outdoor worlds missing or changed')
    else:
        for world, spec in worlds.items():
            if (not isinstance(spec, dict) or
                    not isinstance(spec.get('world_sha256'), str) or
                    len(spec['world_sha256']) != 64 or
                    not isinstance(spec.get('no_bird_reports'), list) or
                    len(spec['no_bird_reports']) < 2 or
                    not all(isinstance(item, str) and item for item in spec['no_bird_reports'])):
                errors.append('invalid world evidence specification: ' + world)
    bird = manifest.get('single_bird')
    if (not isinstance(bird, dict) or bird.get('world') != 'outdoor_campus' or
            not isinstance(bird.get('report'), str) or not bird['report']):
        errors.append('campus single-bird evidence missing')
    perception = manifest.get('perception')
    if (not isinstance(perception, dict) or set(perception) != {'matrix', 'dropout'} or
            not all(isinstance(value, str) and value for value in perception.values())):
        errors.append('perception evidence group incomplete')
    energy = manifest.get('energy_and_safety')
    if (not isinstance(energy, dict) or
            set(energy) != {'model', 'return', 'landing', 'interlock'} or
            not all(isinstance(value, str) and value for value in energy.values())):
        errors.append('energy and safety evidence group incomplete')
    budget = manifest.get('resource_budget')
    if (not isinstance(budget, dict) or set(budget) != {'limits', 'report'} or
            not all(isinstance(value, str) and value for value in budget.values())):
        errors.append('resource budget evidence group incomplete')
    metadata = manifest.get('version_metadata')
    if (not isinstance(metadata, dict) or set(metadata) != {'package_xml', 'setup_py'} or
            not all(isinstance(value, str) and value for value in metadata.values())):
        errors.append('version metadata evidence group incomplete')
    if not isinstance(manifest.get('release_document'), str) or not manifest['release_document']:
        errors.append('release document missing')
    if not isinstance(manifest.get('pending_checks'), list):
        errors.append('pending_checks must be an explicit list')
    if not isinstance(manifest.get('exclusions'), list):
        errors.append('exclusions must be an explicit list')
    return errors


def audit_manifest(root, manifest):
    """Return machine-readable pass/fail with every missing evidence item."""
    root = Path(root).resolve()
    schema_errors = validate_manifest(manifest)
    if schema_errors:
        return dict(release=manifest.get('release') if isinstance(manifest, dict) else None,
                    passed=False, checks=[dict(id='manifest_schema', passed=False,
                                               errors=schema_errors)], exclusions=[])
    checks = []
    minimum_runs = manifest['minimum_runs_per_world']
    for world, spec in sorted(manifest['worlds'].items()):
        world_path = 'catkin_ws/src/logistics_gazebo_sim/worlds/{}.world'.format(world)
        try:
            actual_hash = hashlib.sha256(_safe_file(root, world_path).read_bytes()).hexdigest()
            hash_errors = [] if actual_hash == spec['world_sha256'] else ['world hash mismatch']
        except (OSError, ValueError, KeyError) as exc:
            hash_errors = ['world unavailable: {}'.format(exc)]
        checks.append(dict(id=world + ':world', passed=not hash_errors,
                           errors=hash_errors))
        captures = set()
        valid_runs = 0
        for index, relative in enumerate(spec.get('no_bird_reports', []), 1):
            errors = list(hash_errors)
            try:
                report = _load(root, relative)
                errors.extend(evaluate_flight(report, 'no_bird'))
                if report.get('world') != world or report.get('world_sha256') != spec['world_sha256']:
                    errors.append('report world identity mismatch')
                capture = report.get('capture')
                if not isinstance(capture, str) or not capture or capture in captures:
                    errors.append('missing or duplicate capture identity')
                else:
                    captures.add(capture)
            except (OSError, ValueError, KeyError, TypeError) as exc:
                errors.append('report unavailable: {}'.format(exc))
            valid_runs += not errors
            checks.append(dict(id='{}:no_bird:{}'.format(world, index),
                               passed=not errors, errors=errors))
        repeat_errors = [] if valid_runs >= minimum_runs else [
            'only {} valid independent runs; need {}'.format(valid_runs, minimum_runs)]
        checks.append(dict(id=world + ':repeat', passed=not repeat_errors,
                           errors=repeat_errors))
    bird = manifest['single_bird']
    errors = []
    try:
        report = _load(root, bird['report'])
        errors.extend(evaluate_flight(report, 'single_bird'))
        world = bird['world']
        if report.get('world') != world or report.get('world_sha256') != manifest['worlds'][world]['world_sha256']:
            errors.append('bird report world identity mismatch')
    except (OSError, ValueError, KeyError, TypeError) as exc:
        errors.append('bird report unavailable: {}'.format(exc))
    checks.append(dict(id='single_bird', passed=not errors, errors=errors))

    perception = manifest.get('perception')
    if perception:
        for name, evaluator in (('matrix', evaluate_perception_matrix),
                                ('dropout', evaluate_dropout)):
            errors = []
            try:
                errors.extend(evaluator(_load(root, perception[name])))
            except (OSError, ValueError, KeyError, TypeError) as exc:
                errors.append('perception evidence unavailable: {}'.format(exc))
            checks.append(dict(id='perception:' + name, passed=not errors,
                               errors=errors))
    for name, relative in sorted(manifest.get('energy_and_safety', {}).items()):
        errors = []
        try:
            errors.extend(evaluate_energy_and_safety(name, _load(root, relative)))
        except (OSError, ValueError, KeyError, TypeError) as exc:
            errors.append('energy or safety evidence unavailable: {}'.format(exc))
        checks.append(dict(id='energy_safety:' + name, passed=not errors,
                           errors=errors))

    budget = manifest['resource_budget']
    errors = []
    try:
        limits = _load(root, budget['limits'])
        report = _load(root, budget['report'])
        for measured, limit_key, lower in (
                ('real_time_factor', 'minimum_real_time_factor', True),
                ('full_stack_cpu_cores', 'maximum_cpu_cores', False),
                ('full_stack_pss_peak_sampled_mib', 'maximum_full_stack_pss_mib', False)):
            limit = limits[limit_key]
            if not _number(limit):
                errors.append('invalid resource limit ' + limit_key)
            elif lower:
                _at_least(report, measured, limit, errors)
            else:
                _at_most(report, measured, limit, errors)
            if report.get('acceptance', {}).get(limit_key) != limit:
                errors.append('reported resource limit drift: ' + limit_key)
        _at_least(report, 'sample_wall_s', 30.0, errors)
        if report.get('acceptance', {}).get('passed') is not True:
            errors.append('source resource audit did not pass')
    except (OSError, ValueError, KeyError, TypeError) as exc:
        errors.append('resource evidence unavailable: {}'.format(exc))
    checks.append(dict(id='resource_budget', passed=not errors, errors=errors))
    metadata = manifest.get('version_metadata')
    if metadata:
        errors = []
        release = manifest['release']
        expected = release[1:] if release.startswith('v') else release
        try:
            package = ET.parse(_safe_file(root, metadata['package_xml']))
            version = package.getroot().findtext('version')
            setup = _safe_file(root, metadata['setup_py']).read_text(encoding='utf-8')
            if version != expected or "version='{}'".format(expected) not in setup:
                errors.append('ROS and Python package versions do not match candidate')
        except (OSError, ValueError, KeyError, ET.ParseError) as exc:
            errors.append('version metadata unavailable: {}'.format(exc))
        checks.append(dict(id='version_metadata', passed=not errors,
                           errors=errors))
    document = manifest.get('release_document')
    if document:
        errors = []
        try:
            content = _safe_file(root, document).read_text(encoding='utf-8')
            for phrase in (manifest['release'], '未发布', '双鸟', 'Baylands', '抽样'):
                if phrase not in content:
                    errors.append('release draft omits ' + phrase)
        except (OSError, ValueError, KeyError) as exc:
            errors.append('release draft unavailable: {}'.format(exc))
        checks.append(dict(id='release_document', passed=not errors,
                           errors=errors))
    for item in manifest.get('pending_checks', []):
        checks.append(dict(id='pending:' + item, passed=False,
                           errors=['required evidence not yet integrated']))
    return dict(release=manifest['release'], passed=all(c['passed'] for c in checks),
                checks=checks, exclusions=manifest.get('exclusions', []))
