"""Automatic 3D formation selection with per-vehicle collision validation."""

import numpy as np

from logistics_gazebo_sim_ros.clearance_analyzer import (
    WORLD_XY_LIMIT, WORLD_Z_MAX, WORLD_Z_MIN,
    horizontal_distance, obstacle_primitives)
from logistics_gazebo_sim_ros.formation_3d import (
    assign_slots, generate, minimum_separation)
from logistics_gazebo_sim_ros.scenes import SCENES


class FormationScheduleError(RuntimeError):
    pass


def safe_blend(source,target,ratio,min_separation):
    values=(1.0-ratio)*np.asarray(source,dtype=float)+ratio*np.asarray(target,dtype=float)
    distance=minimum_separation(values)
    if distance<min_separation and distance>1e-9:
        values*=min_separation/distance
    return values


def offsets_valid(scene_id,center,offsets,vehicle_radius=1.2,
                  vertical_radius=.6):
    """Validate every vehicle, returning a detailed first failure."""
    primitives=obstacle_primitives(SCENES[scene_id])
    for index,offset in enumerate(np.asarray(offsets,dtype=float)):
        point=np.asarray(center,dtype=float)+offset;x,y,z=map(float,point)
        if max(abs(x),abs(y))>WORLD_XY_LIMIT-vehicle_radius:
            return False,{"vehicle":index,"reason":"world_boundary","point":point.tolist()}
        if z-vertical_radius<WORLD_Z_MIN or z+vertical_radius>WORLD_Z_MAX:
            return False,{"vehicle":index,"reason":"vertical_boundary","point":point.tolist()}
        for primitive in primitives:
            if z-vertical_radius<=primitive["height"] and horizontal_distance(primitive,x,y)<vehicle_radius:
                return False,{"vehicle":index,"reason":"obstacle","obstacle":primitive["label"],
                              "point":point.tolist()}
    return True,None


def _templates(preferred,count,spacing,preferred_offsets=None):
    preferred_values=(generate(preferred,count,spacing) if preferred_offsets is None
                      else np.asarray(preferred_offsets,dtype=float))
    names=[]
    for name in (preferred,"column","vertical","row"):
        if name not in names:names.append(name)
    result={preferred:preferred_values}
    for name in names[1:]:
        result[name]=assign_slots(preferred_values,generate(name,count,spacing))
    return names,result


def _transition_valid(scene_id,centers,source,target,min_separation):
    values=[]
    denominator=max(1,len(centers)-1)
    for index,center in enumerate(centers):
        offsets=safe_blend(source,target,index/float(denominator),min_separation)
        valid,detail=offsets_valid(scene_id,center,offsets)
        if not valid:return False,None,detail
        if minimum_separation(offsets)<min_separation-1e-6:
            return False,None,{"reason":"vehicle_separation"}
        values.append(offsets)
    return True,values,None


def build_formation_schedule(scene_id,times,centers,preferred="triangle",
                             vehicle_count=3,spacing=3.0,
                             transition_seconds=3.0,preferred_offsets=None):
    """Select formations and create validated time-varying xyz offsets."""
    times=np.asarray(times,dtype=float);centers=np.asarray(centers,dtype=float)
    if len(times)!=len(centers) or len(times)<2:
        raise FormationScheduleError("E_FORMATION_TRANSITION: invalid timed path")
    names,templates=_templates(preferred,vehicle_count,spacing,preferred_offsets)
    choices=[];failures=[]
    for center in centers:
        chosen=None
        for name in names:
            valid,detail=offsets_valid(scene_id,center,templates[name])
            if valid:chosen=name;break
            failures.append(detail)
        if chosen is None:
            raise FormationScheduleError("E_NO_FEASIBLE_FORMATION:{}".format(failures[-1]))
        choices.append(chosen)
    offsets=[templates[name].copy() for name in choices]
    switches=[];last_end=0
    index=1
    while index<len(choices):
        if choices[index]==choices[index-1]:
            index+=1;continue
        source_name,target_name=choices[index-1],choices[index]
        duration_index=max(2,int(np.searchsorted(times,times[index]+transition_seconds)-index))
        if target_name!=preferred:
            candidate_ends=range(index,max(last_end+1,index-4*duration_index),-1)
            windows=[(max(last_end,end-duration_index),end) for end in candidate_ends]
        else:
            windows=[(start,min(len(times)-1,start+duration_index))
                     for start in range(index,min(len(times)-1,index+4*duration_index))]
        selected=None
        for start,end in windows:
            if end-start<2:continue
            valid,values,detail=_transition_valid(
                scene_id,centers[start:end+1],templates[source_name],
                templates[target_name],spacing)
            if valid:selected=(start,end,values);break
        if selected is None:
            raise FormationScheduleError(
                "E_FORMATION_TRANSITION:{}->{} at {}".format(
                    source_name,target_name,index))
        start,end,values=selected
        for local,value in enumerate(values):offsets[start+local]=value
        for position in range(end+1,len(choices)):
            if choices[position]==source_name:choices[position]=target_name;offsets[position]=templates[target_name].copy()
            else:break
        switches.append({"from":source_name,"to":target_name,
                         "start_time":round(float(times[start]),3),
                         "end_time":round(float(times[end]),3)})
        last_end=end;index=max(index+1,end+1)
    minimum=float("inf")
    for center,value in zip(centers,offsets):
        valid,detail=offsets_valid(scene_id,center,value)
        if not valid:
            raise FormationScheduleError("E_FORMATION_TRANSITION:{}".format(detail))
        minimum=min(minimum,minimum_separation(value))
    keys=[]
    for time,value in zip(times,offsets):
        keys.append([round(float(time),3),
                     [[round(float(axis),4) for axis in row] for row in value]])
    counts={name:choices.count(name) for name in names}
    return {"preferred":preferred,"candidate_order":names,
            "switches":switches,"sample_count":len(keys),
            "minimum_separation_m":float(round(minimum,3)),
            "formation_sample_counts":counts,
            "keys":keys}
