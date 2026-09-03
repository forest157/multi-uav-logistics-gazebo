"""Deterministic perception acceptance scenarios shared by tests and CLI."""
import math
from logistics_gazebo_sim.dynamic_obstacles import DynamicSafetyResponse
from logistics_gazebo_sim.pointcloud_perception import DetectionAssociator
from logistics_gazebo_sim.target_tracking import AlphaBetaTracker


def _pipeline():
    return (DetectionAssociator(maximum_distance=1.8,confirmation_hits=3,maximum_misses=20,
        minimum_speed=.8,maximum_speed=12.0,minimum_direction_cosine=.5,
        maximum_acceleration=5.0,maximum_track_age=1.5),AlphaBetaTracker(maximum_age=1.5))


def _step(pipeline,stamp,positions):
    associator,tracker=pipeline
    raw=[{"position":list(position),"radius":.4,"height":.5,"point_count":30} for position in positions]
    return tracker.update(stamp,associator.update(raw,stamp))


def run_perception_acceptance_matrix():
    cases={}
    pipeline=_pipeline();empty=[_step(pipeline,.2*i,[]) for i in range(10)]
    cases["empty_air"]={"pass":not any(empty),"track_samples":sum(bool(x) for x in empty)}
    pipeline=_pipeline();ids=[]
    for i in range(15):
        tracks=_step(pipeline,.2*i,[(.6*i+.01*math.sin(i*.4),0.,2.)])
        ids.extend(track["id"] for track in tracks if track["observed"])
    cases["single_target_noise"]={"pass":len(ids)>=10 and len(set(ids))==1,"unique_ids":sorted(set(ids))}
    pipeline=_pipeline();before=None;after=None;occluded=0
    for stamp,position in [(0.,(0,0,2)),(.2,(.6,0,2)),(.4,(1.2,0,2)),(.6,(1.8,0,2)),(.8,None),(1.,None),(1.2,None),(1.4,(4.2,0,2))]:
        tracks=_step(pipeline,stamp,[] if position is None else [position]);observed=[x for x in tracks if x["observed"]]
        if stamp==.6 and observed:before=observed[0]["id"]
        if stamp==1.4 and observed:after=observed[0]["id"]
        occluded+=sum(not track["observed"] for track in tracks)
    cases["short_occlusion"]={"pass":bool(before and before==after and occluded==3),"before_id":before,"after_id":after,"occluded_samples":occluded}
    pipeline=_pipeline();old=None;new=None
    sequence=[(0.,(0,0,2)),(.2,(.6,0,2)),(.4,(1.2,0,2)),(.6,(1.8,0,2))]
    sequence += [(stamp,None) for stamp in (.8,1.,1.2,1.4,1.6,1.8,2.,2.2)]
    sequence += [(2.4,(7.2,0,2)),(2.6,(7.8,0,2)),(2.8,(8.4,0,2))]
    for stamp,position in sequence:
        tracks=_step(pipeline,stamp,[] if position is None else [position]);observed=[x for x in tracks if x["observed"]]
        if stamp==.6 and observed:old=observed[0]["id"]
        if stamp==2.8 and observed:new=observed[0]["id"]
    cases["long_occlusion"]={"pass":bool(old and new and old!=new),"before_id":old,"after_id":new}
    pipeline=_pipeline();first={};last={}
    for i in range(12):
        x=-3.+.6*i;tracks=_step(pipeline,.2*i,[(x,-.5,2.),(-x,.5,2.)])
        if i==2:first={track["id"]:track["position"][0] for track in tracks}
        if i==11:last={track["id"]:track["position"][0] for track in tracks}
    stable=set(first)==set(last) and len(first)==2 and all(first[k]*last[k]<0 for k in first)
    cases["two_target_crossing"]={"pass":stable,"track_ids":sorted(last)}
    response=DynamicSafetyResponse(warning_scale=.35,release_delay=1.,stale_hold_delay=1.)
    actions=[response.update("STALE",0.)["action"],response.update("STALE",1.)["action"],response.update("SAFE",1.1)["action"],response.update("SAFE",2.1)["action"]]
    cases["perception_stale"]={"pass":actions==["SLOW","HOLD","HOLD","NORMAL"],"actions":actions}
    return {"pass":all(case["pass"] for case in cases.values()),"case_count":len(cases),"cases":cases}
