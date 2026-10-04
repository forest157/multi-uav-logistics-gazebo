#!/usr/bin/env python3
"""Audit a captured outdoor flight with independent Gazebo truth."""

import argparse
import json
import os

import rospkg

from logistics_gazebo_sim.outdoor_acceptance import audit_outdoor_flight
from logistics_gazebo_sim.outdoor_world_profile import load_outdoor_profile
from logistics_gazebo_sim.worlds import OUTDOOR_LAYOUTS


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('capture', help='JSONL from capture_flight_safety.py')
    parser.add_argument('--world', choices=sorted(OUTDOOR_LAYOUTS), required=True)
    parser.add_argument('--single-bird', action='store_true',
                        help='require a detected bird and active ORCA intervention')
    parser.add_argument('--output', help='optional summary JSON')
    args = parser.parse_args()
    package = rospkg.RosPack().get_path('logistics_gazebo_sim')
    profile = load_outdoor_profile(args.world, os.path.join(package, 'worlds'))
    with open(args.capture, encoding='utf-8') as stream:
        report = audit_outdoor_flight((json.loads(line) for line in stream),
                                      profile, expect_single_bird=args.single_bird)
    report['world'] = args.world
    report['world_sha256'] = profile['world_sha256']
    report['capture'] = args.capture
    rendered = json.dumps(report, ensure_ascii=False, indent=2)
    if args.output:
        with open(args.output, 'w', encoding='utf-8') as stream:
            stream.write(rendered + '\n')
    print(rendered)
    return 0 if report['passed'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
