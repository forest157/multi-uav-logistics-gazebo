"""Pluggable local avoidance algorithms for scalable 3D fleets."""
import math
import numpy as np

from logistics_gazebo_sim.dynamic_obstacles import (
    DynamicObstacleError, interpolate_timed_path, plan_collective_avoidance,
    validate_obstacle)


def _vector(value, name):
    result=np.asarray(value,dtype=float)
    if result.shape!=(3,) or not np.all(np.isfinite(result)):
        raise DynamicObstacleError("{} must contain finite xyz".format(name))
    return result


def _clamp(vector, limit):
    norm=float(np.linalg.norm(vector))
    return vector if norm<=limit or norm<1e-9 else vector*(limit/norm)


class OrcaCommandGate:
    """Fail-closed conditioning for ORCA velocity commands before flight use."""
    def __init__(self, vehicle_count, max_speed=2.0, max_climb_rate=0.8,
                 max_acceleration=1.0, smoothing=0.35, timeout=0.6):
        self.vehicle_count=int(vehicle_count);self.max_speed=float(max_speed)
        self.max_climb_rate=float(max_climb_rate);self.max_acceleration=float(max_acceleration)
        self.smoothing=float(smoothing);self.timeout=float(timeout)
        if self.vehicle_count<1 or min(self.max_speed,self.max_climb_rate,self.max_acceleration,self.timeout)<=0.0:
            raise DynamicObstacleError("ORCA command limits must be positive")
        if not 0.0<self.smoothing<=1.0:
            raise DynamicObstacleError("ORCA smoothing must be in (0,1]")
        self.reset()
    def reset(self):
        self.previous=[np.zeros(3,dtype=float) for _ in range(self.vehicle_count)]
    def condition(self, plan, now, dt):
        if not isinstance(plan,dict) or not plan.get("viable"):
            raise DynamicObstacleError("ORCA plan is not viable")
        if plan.get("contract_version")!="orca_velocity_v1" or plan.get("algorithm")!="orca3d" or plan.get("command_type")!="per_vehicle_velocity":
            raise DynamicObstacleError("unexpected ORCA command contract")
        stamp=float(plan.get("stamp",-1.0))
        age=float(now)-stamp
        validity=min(self.timeout,float(plan.get("valid_for_s",self.timeout)))
        if not np.isfinite(stamp) or validity<=0.0 or age<0.0 or age>validity:
            raise DynamicObstacleError("ORCA command is stale")
        commands=plan.get("commands")
        if not isinstance(commands,list) or len(commands)!=self.vehicle_count:
            raise DynamicObstacleError("ORCA command vehicle count mismatch")
        by_id={item.get("vehicle_id"):item for item in commands if isinstance(item,dict)}
        expected={"uav{}".format(index) for index in range(self.vehicle_count)}
        if set(by_id)!=expected:
            raise DynamicObstacleError("ORCA command vehicle ids mismatch")
        step=max(1e-3,float(dt));result=[]
        for index in range(self.vehicle_count):
            desired=_clamp(_vector(by_id["uav{}".format(index)].get("velocity"),"ORCA velocity"),self.max_speed)
            desired[2]=max(-self.max_climb_rate,min(self.max_climb_rate,desired[2]))
            delta=_clamp(desired-self.previous[index],self.max_acceleration*step)
            limited=self.previous[index]+delta
            filtered=self.previous[index]+self.smoothing*(limited-self.previous[index])
            self.previous[index]=filtered
            result.append(tuple(float(value) for value in filtered))
        return result


def _path_state(path, lookahead):
    values=np.asarray(path,dtype=float)
    if values.ndim!=2 or values.shape[1]!=4 or len(values)<2:
        raise DynamicObstacleError("timed path must contain [t,x,y,z] rows")
    position=values[0,1:].copy()
    query=min(float(values[-1,0]),float(values[0,0])+max(0.1,float(lookahead)))
    target=interpolate_timed_path(values,query)
    velocity=(target-position)/max(0.1,query-float(values[0,0]))
    return position,velocity


def _avoidance_correction(relative_position, relative_velocity, radius,
                           time_horizon, time_step):
    """Return the minimum velocity correction and outward half-plane normal."""
    distance_sq=float(np.dot(relative_position,relative_position))
    radius_sq=float(radius*radius)
    speed_sq=float(np.dot(relative_velocity,relative_velocity))
    closest_time=(float(np.dot(relative_position,relative_velocity))/speed_sq
                  if speed_sq>1e-9 else -1.0)
    if 0.0<closest_time<=float(time_horizon):
        closest=relative_position-relative_velocity*closest_time
        closest_distance=float(np.linalg.norm(closest))
        if closest_distance<radius:
            if closest_distance>1e-9:normal=-closest/closest_distance
            else:
                normal=np.cross(relative_position,np.asarray([0.0,0.0,1.0]))
                if float(np.linalg.norm(normal))<1e-9:
                    normal=np.cross(relative_position,np.asarray([0.0,1.0,0.0]))
                normal=normal/max(1e-9,float(np.linalg.norm(normal)))
            correction=(1.25*(radius-closest_distance)/max(float(time_step),closest_time))*normal
            return correction,normal
    if distance_sq>radius_sq:
        inverse_horizon=1.0/max(0.1,float(time_horizon))
        w=relative_velocity-inverse_horizon*relative_position
        w_length=float(np.linalg.norm(w))
        if w_length<1e-9:
            normal=-relative_position/max(1e-9,math.sqrt(distance_sq))
        else:normal=w/w_length
        correction=(radius*inverse_horizon-w_length)*normal
    else:
        inverse_step=1.0/max(0.02,float(time_step))
        w=relative_velocity-inverse_step*relative_position
        w_length=float(np.linalg.norm(w))
        normal=(w/w_length if w_length>1e-9 else
                -relative_position/max(1e-9,math.sqrt(distance_sq)))
        correction=(radius*inverse_step-w_length)*normal
    return correction,normal


class LocalAvoidancePlanner:
    name="base"
    command_type="none"
    def plan(self, paths, obstacles, **options):
        raise NotImplementedError


class CollectiveOffsetPlanner(LocalAvoidancePlanner):
    name="collective_offset";command_type="collective_offset"
    def plan(self, paths, obstacles, **options):
        allowed=("candidate_offsets","horizon","required_clearance",
                 "warning_clearance","scene_id","minimum_separation",
                 "tracking_tolerance")
        result=plan_collective_avoidance(paths,obstacles,**{
            key:value for key,value in options.items() if key in allowed})
        result.update({"algorithm":self.name,"command_type":self.command_type,
                       "vehicle_count":len(paths)})
        return result


class Orca3DPlanner(LocalAvoidancePlanner):
    """Dependency-free spherical 3D ORCA prototype.

    It produces per-vehicle velocity suggestions. The v0.4.1 integration runs
    this in shadow mode; the existing safety layer remains authoritative.
    """
    name="orca3d";command_type="per_vehicle_velocity"
    def plan(self, paths, obstacles, **options):
        if not paths:raise DynamicObstacleError("at least one vehicle path is required")
        count=len(paths)
        max_speed=float(options.get("max_speed",2.0))
        horizon=float(options.get("orca_time_horizon",options.get("horizon",5.0)))
        step=float(options.get("time_step",0.2))
        radius=float(options.get("vehicle_radius",1.2))
        separation=float(options.get("minimum_separation",3.0))
        safety_buffer=float(options.get("safety_buffer",0.5))
        required_clearance=float(options.get("required_clearance",0.5))
        lookahead=float(options.get("preferred_velocity_lookahead",1.0))
        if not 1<=count<=32:raise DynamicObstacleError("ORCA vehicle_count must be 1..32")
        if min(max_speed,horizon,step,radius,separation,lookahead)<=0.0 or min(safety_buffer,required_clearance)<0.0:
            raise DynamicObstacleError("ORCA limits must be positive")
        states=[_path_state(path,lookahead) for path in paths]
        positions=[value[0] for value in states]
        preferred=[_clamp(value[1],max_speed) for value in states]
        velocities=[value.copy() for value in preferred]
        checked=[validate_obstacle(value) for value in obstacles]
        constraint_counts=[]
        # Two passes make intersecting half-plane projections deterministic and
        # substantially reduce residual violations without a heavy LP package.
        for index in range(count):
            planes=[]
            for other in range(count):
                if other==index:continue
                relative_position=positions[other]-positions[index]
                relative_velocity=preferred[index]-preferred[other]
                correction,normal=_avoidance_correction(
                    relative_position,relative_velocity,
                    max(separation,2.0*radius),horizon,step)
                planes.append((preferred[index]+0.5*correction,normal,
                               "uav{}".format(other)))
            for obstacle in checked:
                relative_position=obstacle["position"]-positions[index]
                relative_velocity=preferred[index]-obstacle["velocity"]
                correction,normal=_avoidance_correction(
                    relative_position,relative_velocity,
                    radius+obstacle["radius"]+safety_buffer+required_clearance,horizon,step)
                planes.append((preferred[index]+correction,normal,obstacle["id"]))
            velocity=preferred[index].copy()
            for _ in range(2):
                for point,normal,_source in planes:
                    violation=float(np.dot(velocity-point,normal))
                    if violation<0.0:velocity-=violation*normal
                    velocity=_clamp(velocity,max_speed)
            velocities[index]=velocity;constraint_counts.append(len(planes))
        commands=[]
        for index,(velocity,pref) in enumerate(zip(velocities,preferred)):
            commands.append({"vehicle_id":"uav{}".format(index),
                "velocity":[round(float(v),4) for v in velocity],
                "preferred_velocity":[round(float(v),4) for v in pref],
                "correction_norm":round(float(np.linalg.norm(velocity-pref)),4),
                "constraint_count":constraint_counts[index]})
        sample_times=np.linspace(0.0,horizon,max(3,int(math.ceil(horizon/step))+1))
        minimum_pair=float("inf")
        for seconds in sample_times:
            future=[position+velocity*seconds for position,velocity in zip(positions,velocities)]
            for first in range(count):
                for second in range(first+1,count):
                    minimum_pair=min(minimum_pair,float(np.linalg.norm(future[first]-future[second])))
        required=max(separation,2.0*radius)
        minimum_obstacle=float("inf")
        for seconds in sample_times:
            for index,(position,velocity) in enumerate(zip(positions,velocities)):
                future=position+velocity*seconds
                for obstacle in checked:
                    obstacle_future=obstacle["position"]+obstacle["velocity"]*seconds
                    clearance=float(np.linalg.norm(future-obstacle_future))-(
                        radius+obstacle["radius"]+safety_buffer+required_clearance)
                    minimum_obstacle=min(minimum_obstacle,clearance)
        pair_safe=(count<2 or minimum_pair>=required-0.05)
        obstacle_safe=(not checked or minimum_obstacle>=-0.05)
        constraints_satisfied=pair_safe and obstacle_safe
        rejections={}
        if not pair_safe:rejections["VEHICLE_SEPARATION"]=1
        if not obstacle_safe:rejections["DYNAMIC_CLEARANCE"]=1
        return {"viable":bool(constraints_satisfied),"algorithm":self.name,
            "command_type":self.command_type,"vehicle_count":count,
            "commands":commands,"selected_offset":None,
            "minimum_clearance_m":None,"candidates":[],
            "predicted_minimum_separation_m":(None if count<2 else round(minimum_pair,4)),
            "predicted_minimum_obstacle_clearance_m":(None if not checked else round(minimum_obstacle,4)),
            "constraints_satisfied":bool(constraints_satisfied),
            "reason":("3D ORCA velocity solution generated in shadow mode" if constraints_satisfied
                      else "3D ORCA could not satisfy fleet separation; hold required"),
            "shadow_mode":True,"rejection_summary":rejections}


_PLANNERS={value.name:value for value in (CollectiveOffsetPlanner,Orca3DPlanner)}

def available_local_planners():return tuple(sorted(_PLANNERS))

def create_local_planner(name):
    key=str(name or "collective_offset").strip().lower()
    try:return _PLANNERS[key]()
    except KeyError:raise DynamicObstacleError(
        "unknown local avoidance algorithm {}; available: {}".format(
            key,",".join(available_local_planners())))
