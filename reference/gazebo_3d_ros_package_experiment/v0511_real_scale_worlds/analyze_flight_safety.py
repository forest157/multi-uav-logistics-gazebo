#!/usr/bin/env python3
"""Offline, conservative safety-envelope audit. Never feeds truth to flight nodes."""
import argparse
import collections
import json
import math


def analyze(rows, spawn_x=-40., spawn_y=-40., spacing=3.3, vehicle_radius=1.2, bird_radius=.75):
    metrics=dict(samples=0,valid_truth_samples=0,minimum_bird_envelope_clearance_m=None,
                 minimum_fleet_separation_m=None,maximum_estimate_error_m=None,
                 closest_bird_event=None,complete=False,disarmed=False)
    actions=collections.Counter();previous=None;transitions=0
    for row in rows:
        metrics["samples"]+=1
        mission=row.get("mission",{});action=mission.get("dynamic_action")
        if previous is not None and action!=previous:transitions+=1
        previous=action;actions[action]+=1
        metrics["complete"]=mission.get("state")=="COMPLETE"
        states=row.get("flight_states",{})
        metrics["disarmed"]=len(states)==3 and all(not s["armed"] for s in states.values())
        stamp=row.get("stamp");received=row.get("truth_receipt_stamp")
        if stamp is None or received is None or not 0<=stamp-received<=.2:continue
        truth=row.get("truth_evaluation_only",{})
        if any("iris"+str(i) not in truth for i in range(3)):continue
        fleet=[truth["iris"+str(i)]["position"] for i in range(3)]
        if any(len(p)!=3 or not all(math.isfinite(v) for v in p) for p in fleet):continue
        metrics["valid_truth_samples"]+=1
        for i,p in enumerate(fleet):
            for other in fleet[:i]:
                d=math.dist(p,other);old=metrics["minimum_fleet_separation_m"]
                metrics["minimum_fleet_separation_m"]=d if old is None else min(old,d)
            pose=row.get("poses",{}).get(str(i))
            if pose and abs(received-pose[0])<=.1:
                estimate=[pose[1]+spawn_x+(i-1)*spacing,pose[2]+spawn_y,pose[3]]
                error=math.dist(p,estimate);old=metrics["maximum_estimate_error_m"]
                metrics["maximum_estimate_error_m"]=error if old is None else max(old,error)
            for name,bird in truth.items():
                if "bird" not in name:continue
                d=math.dist(p,bird["position"])-vehicle_radius-bird_radius
                old=metrics["minimum_bird_envelope_clearance_m"]
                if old is None or d<old:
                    metrics["minimum_bird_envelope_clearance_m"]=d
                    metrics["closest_bird_event"]=dict(stamp=stamp,vehicle=i,bird=name,action=action)
    metrics["actions"]=dict(actions);metrics["action_transitions"]=transitions
    metrics["criteria"]=dict(minimum_fleet_separation_m=3.,minimum_bird_envelope_clearance_m=0.,
                             maximum_estimate_error_m=1.,truth_max_receipt_age_s=.2)
    reasons=[]
    for key,threshold,lower in (("minimum_fleet_separation_m",3.,True),
                               ("minimum_bird_envelope_clearance_m",0.,True),
                               ("maximum_estimate_error_m",1.,False)):
        v=metrics[key]
        if v is None or (v<threshold if lower else v>threshold):reasons.append(key)
    if not metrics["complete"]:reasons.append("mission_not_complete")
    if not metrics["disarmed"]:reasons.append("vehicles_not_disarmed")
    if metrics["valid_truth_samples"]<.95*metrics["samples"]:reasons.append("insufficient_truth_coverage")
    metrics["passed"]=not reasons;metrics["failure_reasons"]=reasons
    metrics["scope"]="Conservative sphere envelopes, not a contact-sensor proof; sampled truth uses receipt timestamps."
    return metrics


if __name__=="__main__":
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument("capture")
    args=parser.parse_args()
    with open(args.capture) as stream:report=analyze(json.loads(line) for line in stream)
    print(json.dumps(report,indent=2))
    raise SystemExit(0 if report["passed"] else 1)
