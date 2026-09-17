#!/usr/bin/env python3
"""Offline, conservative safety-envelope audit. Never feeds truth to flight nodes."""
import argparse
import collections
import json
import math


def analyze(rows, spawn_x=-40., spawn_y=-40., spacing=3.3, vehicle_radius=1.2,
            bird_radius=.75, require_bird=True):
    metrics=dict(samples=0,valid_truth_samples=0,minimum_bird_envelope_clearance_m=None,
                 minimum_fleet_separation_m=None,maximum_estimate_error_m=None,
                 maximum_settled_vertical_tracking_error_m=None,
                 settled_vertical_tracking_samples=0,
                 closest_bird_event=None,bird_truth_samples=0,
                 frames_with_perception_tracks=0,maximum_perception_track_count=0,
                 active_avoidance_samples=0,complete=False,disarmed=False)
    actions=collections.Counter();previous=None;transitions=0
    vertical_states={str(i):dict(target=None,settled=False) for i in range(3)}
    for row in rows:
        metrics["samples"]+=1
        mission=row.get("mission",{});action=mission.get("dynamic_action")
        if previous is not None and action!=previous:transitions+=1
        previous=action;actions[action]+=1
        tracks=row.get("tracks",{});obstacles=(tracks.get("obstacles",[])
                                                if isinstance(tracks,dict) else [])
        track_count=len(obstacles) if isinstance(obstacles,list) else 0
        if track_count:metrics["frames_with_perception_tracks"]+=1
        metrics["maximum_perception_track_count"]=max(
            metrics["maximum_perception_track_count"],track_count)
        if mission.get("state") not in (None,"INITIALIZING","READY","COMPLETE") and action in (
                "SLOW","ORCA","HOLD","AVOID"):
            metrics["active_avoidance_samples"]+=1
        metrics["complete"]=mission.get("state")=="COMPLETE"
        states=row.get("flight_states",{})
        metrics["disarmed"]=len(states)==3 and all(not s["armed"] for s in states.values())
        poses=row.get("poses",{});targets=row.get("targets",{})
        for i in range(3):
            key=str(i);pose=poses.get(key);target=targets.get(key)
            if (pose and target and abs(pose[0]-target[0])<=.1
                    and all(math.isfinite(v) for v in (pose[3],target[3]))):
                error=abs(pose[3]-target[3]);state=vertical_states[key]
                if state["target"] is None or abs(target[3]-state["target"])>.05:
                    state["target"]=target[3];state["settled"]=False
                if not state["settled"] and error<=.5:state["settled"]=True
                if state["settled"]:
                    old=metrics["maximum_settled_vertical_tracking_error_m"]
                    metrics["maximum_settled_vertical_tracking_error_m"]=error if old is None else max(old,error)
                    metrics["settled_vertical_tracking_samples"]+=1
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
            pose=poses.get(str(i))
            if pose and abs(received-pose[0])<=.1:
                estimate=[pose[1]+spawn_x+(i-1)*spacing,pose[2]+spawn_y,pose[3]]
                error=math.dist(p,estimate);old=metrics["maximum_estimate_error_m"]
                metrics["maximum_estimate_error_m"]=error if old is None else max(old,error)
            for name,bird in truth.items():
                if "bird" not in name:continue
                metrics["bird_truth_samples"]+=1
                d=math.dist(p,bird["position"])-vehicle_radius-bird_radius
                old=metrics["minimum_bird_envelope_clearance_m"]
                if old is None or d<old:
                    metrics["minimum_bird_envelope_clearance_m"]=d
                    metrics["closest_bird_event"]=dict(stamp=stamp,vehicle=i,bird=name,action=action)
    metrics["actions"]=dict(actions);metrics["action_transitions"]=transitions
    metrics["criteria"]=dict(minimum_fleet_separation_m=3.,minimum_bird_envelope_clearance_m=0.,
                             maximum_estimate_error_m=1.,maximum_settled_vertical_tracking_error_m=1.,
                             truth_max_receipt_age_s=.2)
    reasons=[]
    checks=[("minimum_fleet_separation_m",3.,True),
            ("maximum_estimate_error_m",1.,False)]
    if require_bird:checks.append(("minimum_bird_envelope_clearance_m",0.,True))
    for key,threshold,lower in checks:
        v=metrics[key]
        if v is None or (v<threshold if lower else v>threshold):reasons.append(key)
    if require_bird and metrics["bird_truth_samples"]==0:reasons.append("bird_truth_missing")
    if not require_bird:
        if metrics["bird_truth_samples"]:reasons.append("unexpected_bird_truth")
        if metrics["frames_with_perception_tracks"]:reasons.append("unexpected_perception_tracks")
        if metrics["active_avoidance_samples"]:reasons.append("unexpected_avoidance_action")
        if metrics["maximum_settled_vertical_tracking_error_m"] is None:
            reasons.append("vertical_tracking_missing")
        elif metrics["maximum_settled_vertical_tracking_error_m"]>1.:
            reasons.append("maximum_settled_vertical_tracking_error_m")
    if not metrics["complete"]:reasons.append("mission_not_complete")
    if not metrics["disarmed"]:reasons.append("vehicles_not_disarmed")
    if metrics["valid_truth_samples"]<.95*metrics["samples"]:reasons.append("insufficient_truth_coverage")
    metrics["passed"]=not reasons;metrics["failure_reasons"]=reasons
    metrics["scope"]="Conservative sphere envelopes, not a contact-sensor proof; sampled truth uses receipt timestamps."
    return metrics


if __name__=="__main__":
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument("capture")
    parser.add_argument("--allow-no-birds",action="store_true",
                        help="audit a declared no-bird scenario and reject any tracks or avoidance")
    args=parser.parse_args()
    with open(args.capture) as stream:report=analyze(
        (json.loads(line) for line in stream),require_bird=not args.allow_no_birds)
    print(json.dumps(report,indent=2))
    raise SystemExit(0 if report["passed"] else 1)
