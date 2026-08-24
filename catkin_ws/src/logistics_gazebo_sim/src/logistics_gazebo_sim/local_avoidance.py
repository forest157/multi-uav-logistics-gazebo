"""Pluggable local avoidance algorithms for scalable 3D fleets."""
import math
import multiprocessing as mp
import time
import warnings
import numpy as np
from scipy.optimize import minimize

from logistics_gazebo_sim.dynamic_obstacles import (
    DynamicObstacleError, assess_fleet_separation, assess_timed_path, interpolate_timed_path, plan_collective_avoidance,
    validate_static_paths,
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
        self.previous=[None for _ in range(self.vehicle_count)]
    def condition(self, plan, now, dt):
        if not isinstance(plan,dict) or not plan.get("viable"):
            raise DynamicObstacleError("ORCA plan is not viable")
        if plan.get("contract_version")!="orca_velocity_v1" or plan.get("algorithm")!="orca3d" or plan.get("command_type")!="per_vehicle_velocity":
            raise DynamicObstacleError("unexpected ORCA command contract")
        if not plan.get("constraints_satisfied") or not (plan.get("static_validation") or {}).get("feasible"):
            raise DynamicObstacleError("ORCA command lacks independent safety validation")
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
            item=by_id["uav{}".format(index)]
            desired=_clamp(_vector(item.get("velocity"),"ORCA velocity"),self.max_speed)
            desired[2]=max(-self.max_climb_rate,min(self.max_climb_rate,desired[2]))
            if self.previous[index] is None:
                preferred=_clamp(_vector(item.get("preferred_velocity"),"ORCA preferred velocity"),self.max_speed)
                preferred[2]=max(-self.max_climb_rate,min(self.max_climb_rate,preferred[2]))
                self.previous[index]=preferred
            delta=_clamp(desired-self.previous[index],self.max_acceleration*step)
            limited=self.previous[index]+delta
            filtered=self.previous[index]+self.smoothing*(limited-self.previous[index])
            self.previous[index]=filtered
            result.append(tuple(float(value) for value in filtered))
        return result


def orca_position_targets(poses, velocities, horizon):
    """Convert conditioned ENU velocities into short position setpoints."""
    if len(poses)!=len(velocities) or not poses:
        raise DynamicObstacleError("ORCA pose and velocity counts must match")
    seconds=float(horizon)
    if not np.isfinite(seconds) or seconds<=0.0:
        raise DynamicObstacleError("ORCA position horizon must be positive")
    return [tuple(float(value) for value in (_vector(pose,"ORCA pose")+
            seconds*_vector(velocity,"conditioned ORCA velocity")))
            for pose,velocity in zip(poses,velocities)]


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


class DistributedMpcPlanner(LocalAvoidancePlanner):
    """Per-vehicle finite-horizon optimizer for v0.4.3 shadow evaluation."""
    name="distributed_mpc";command_type="per_vehicle_trajectory"
    def __init__(self):
        self._warm_accelerations=None
        self._warm_starts=None
        self._warm_shape=None
    def plan(self,paths,obstacles,**options):
        if not paths:raise DynamicObstacleError("at least one vehicle path is required")
        count=len(paths);steps=int(options.get("mpc_steps",6));dt=float(options.get("mpc_dt",0.4))
        max_speed=float(options.get("max_speed",2.0));max_acc=float(options.get("max_acceleration",1.0))
        max_vertical_acc=float(options.get("max_vertical_acceleration",0.6));max_climb=float(options.get("max_climb_rate",0.8))
        separation=float(options.get("minimum_separation",3.0));required_clearance=float(options.get("required_clearance",0.5))
        max_iterations=int(options.get("mpc_max_iterations",45));scene_id=options.get("scene_id")
        if not 1<=count<=32 or not 2<=steps<=20 or min(dt,max_speed,max_acc,max_vertical_acc,max_climb,separation)<=0.0:
            raise DynamicObstacleError("MPC dimensions and limits are invalid")
        checked=[validate_obstacle(value) for value in obstacles]
        arrays=[np.asarray(path,dtype=float) for path in paths]
        query=np.arange(steps+1,dtype=float)*dt
        references=[np.asarray([interpolate_timed_path(path,min(float(path[-1,0]),float(path[0,0])+seconds)) for seconds in query]) for path in arrays]
        starts=[];initial_velocities=[]
        for path in arrays:
            position,velocity=_path_state(path,min(dt,float(path[-1,0])-float(path[0,0])))
            starts.append(position);initial_velocities.append(_clamp(velocity,max_speed))
        warm_enabled=bool(options.get("mpc_warm_start",True));warm_limit=float(options.get("mpc_warm_start_max_displacement",max(3.0,3.0*max_speed*dt)))
        warm_valid=(warm_enabled and self._warm_shape==(count,steps) and self._warm_starts is not None and max(float(np.linalg.norm(starts[index]-self._warm_starts[index])) for index in range(count))<=warm_limit)
        trajectories=[];commands=[];solve_times=[];iterations=[];all_success=True;next_warm=[];warm_used=[]
        bounds=[]
        for _ in range(steps):bounds.extend([(-max_acc,max_acc),(-max_acc,max_acc),(-max_vertical_acc,max_vertical_acc)])
        for vehicle in range(count):
            start_clock=time.perf_counter();cold=np.zeros((steps,3),dtype=float)
            previous=self._warm_accelerations[vehicle].copy() if warm_valid else cold.copy()
            avoid_direction=np.cross(initial_velocities[vehicle],np.asarray([0.0,0.0,1.0]))
            if float(np.linalg.norm(avoid_direction))<1e-6:avoid_direction=np.asarray([0.0,1.0,0.0])
            avoid_direction=avoid_direction/float(np.linalg.norm(avoid_direction))
            def rollout(flat):
                accelerations=np.asarray(flat,dtype=float).reshape((steps,3));position=starts[vehicle].copy();velocity=initial_velocities[vehicle].copy();positions=[position.copy()];velocities=[]
                for acceleration in accelerations:
                    velocity=_clamp(velocity+acceleration*dt,max_speed);velocity[2]=max(-max_climb,min(max_climb,velocity[2]));position=position+velocity*dt;positions.append(position.copy());velocities.append(velocity.copy())
                return np.asarray(positions),np.asarray(velocities),accelerations
            def objective(flat):
                positions,velocities,accelerations=rollout(flat);cost=0.0
                cost+=4.0*float(np.sum((positions-references[vehicle])**2))
                cost+=0.20*float(np.sum(accelerations**2))+0.35*float(np.sum(np.diff(accelerations,axis=0)**2))
                cost+=0.08*float(np.sum(velocities**2))
                for step_index in range(1,steps+1):
                    seconds=query[step_index];position=positions[step_index]
                    for obstacle in checked:
                        obstacle_center=obstacle["position"]+obstacle["velocity"]*seconds;safe=1.2+obstacle["radius"]+0.5+required_clearance
                        for source in range(count):
                            formation_delta=references[vehicle][step_index]-references[source][step_index]
                            center=obstacle_center+formation_delta;gap=float(np.linalg.norm(position-center));weight=1200.0 if source==vehicle else 700.0
                            cost+=weight*max(0.0,safe-gap)**2
                            if gap<2.0*safe:
                                lateral=float(np.dot(position-center,avoid_direction));cost+=400.0*max(0.0,safe-lateral)**2
                    for peer in range(count):
                        if peer==vehicle:continue
                        peer_position=references[peer][step_index];gap=float(np.linalg.norm(position-peer_position));cost+=5000.0*max(0.0,separation-gap)**2
                        nominal_relative=references[vehicle][step_index]-peer_position;actual_relative=position-peer_position;cost+=2.0*float(np.sum((actual_relative-nominal_relative)**2))
                return cost
            use_warm=bool(warm_valid and objective(previous.ravel())<=objective(cold.ravel()))
            if not use_warm:previous=cold.copy()
            warm_used.append(use_warm)
            timeout_s=float(options.get("mpc_vehicle_timeout_s",0.35))
            def solve_isolated(connection):
                try:
                    with warnings.catch_warnings():
                        warnings.simplefilter("ignore",RuntimeWarning)
                        solved=minimize(objective,previous.ravel(),method="SLSQP",bounds=bounds,options={"maxiter":max_iterations,"ftol":1e-4})
                    connection.send({"x":solved.x,"success":bool(solved.success or int(solved.status)==8),"status":int(solved.status),"message":str(solved.message),"iterations":int(solved.nit),"fun":float(solved.fun)})
                except BaseException as error:connection.send({"error":"{}: {}".format(type(error).__name__,error)})
                finally:connection.close()
            def launch_solver():
                parent_pipe,child_pipe=mp.Pipe(False);process=mp.get_context("fork").Process(target=solve_isolated,args=(child_pipe,));process.daemon=True;process.start();child_pipe.close();process.join(timeout_s)
                if process.is_alive():process.terminate();process.join(0.1);value={"error":"vehicle solve timeout after {:.3f}s".format(timeout_s)}
                elif parent_pipe.poll():value=parent_pipe.recv()
                else:value={"error":"vehicle solver exited with code {}".format(process.exitcode)}
                parent_pipe.close();return value
            solver=launch_solver();retried=False
            if solver.get("error") and "timeout" not in solver["error"]:solver=launch_solver();retried=True
            solution=np.asarray(solver.get("x",previous.ravel()),dtype=float);positions,velocities,accelerations=rollout(solution);elapsed_ms=1000.0*(time.perf_counter()-start_clock)
            success=bool(not solver.get("error") and solver.get("success") and np.all(np.isfinite(positions)) and np.isfinite(solver.get("fun",float("nan"))));all_success=all_success and success
            trajectory=[[round(float(query[index]),3)]+[round(float(axis),4) for axis in positions[index]] for index in range(steps+1)]
            trajectories.append(trajectory);solve_times.append(elapsed_ms);iterations.append(int(solver.get("iterations",0)));next_warm.append(accelerations.copy() if success else cold.copy())
            commands.append({"vehicle_id":"uav{}".format(vehicle),"velocity":[round(float(axis),4) for axis in velocities[0]],"preferred_velocity":[round(float(axis),4) for axis in initial_velocities[vehicle]],"acceleration":[round(float(axis),4) for axis in accelerations[0]],"solver_success":success,"solver_status":int(solver.get("status",-2)),"solver_message":solver.get("error",solver.get("message","unknown solver result")),"iterations":int(solver.get("iterations",0)),"solve_time_ms":round(elapsed_ms,3),"solver_retried":retried})
        self._warm_accelerations=next_warm;self._warm_starts=[value.copy() for value in starts];self._warm_shape=(count,steps)
        reports=[assess_timed_path(path,checked,horizon=steps*dt,warning_clearance=required_clearance) for path in trajectories]
        dynamic_safe=all(report["minimum_clearance_m"] is None or report["minimum_clearance_m"]>=required_clearance for report in reports)
        fleet=assess_fleet_separation(trajectories,horizon=steps*dt,minimum_separation=separation)
        static=(validate_static_paths(scene_id,trajectories) if scene_id is not None else {"feasible":True,"error_code":None,"message":"static validation disabled"})
        viable=all_success and dynamic_safe and fleet["safe"] and static["feasible"]
        rejections={}
        if not all_success:rejections["MPC_SOLVER_FAILURE"]=sum(1 for command in commands if not command["solver_success"])
        timeout_count=sum(1 for command in commands if "timeout" in command["solver_message"])
        if timeout_count:rejections["MPC_SOLVER_TIMEOUT"]=timeout_count
        if not dynamic_safe:rejections["DYNAMIC_CLEARANCE"]=1
        if not fleet["safe"]:rejections["VEHICLE_SEPARATION"]=1
        if not static["feasible"]:rejections[static.get("error_code") or "STATIC_CONSTRAINT"]=1
        return {"viable":bool(viable),"algorithm":self.name,"command_type":self.command_type,"vehicle_count":count,"commands":commands,"trajectories":trajectories,"constraints_satisfied":bool(dynamic_safe and fleet["safe"] and static["feasible"]),"static_validation":static,"fleet_separation":fleet,"dynamic_reports":reports,"solve_time_ms":{"total":round(sum(solve_times),3),"maximum":round(max(solve_times),3),"mean":round(sum(solve_times)/len(solve_times),3)},"iterations":{"maximum":max(iterations),"mean":round(sum(iterations)/len(iterations),2)},"warm_started_vehicle_count":sum(1 for value in warm_used if value),"reason":"distributed MPC shadow trajectory generated" if viable else "distributed MPC failed solver or independent safety validation","shadow_mode":True,"requires_external_safety_gate":True,"solver_isolated":True,"rejection_summary":rejections}


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
        scene_id=options.get("scene_id")
        predicted_paths=[]
        for position,velocity in zip(positions,velocities):
            predicted_paths.append([[float(seconds)]+list(position+velocity*seconds)
                                    for seconds in sample_times])
        static=(validate_static_paths(scene_id,predicted_paths) if scene_id is not None
                else {"feasible":True,"error_code":None,"message":"static validation disabled"})
        pair_safe=(count<2 or minimum_pair>=required-0.05)
        obstacle_safe=(not checked or minimum_obstacle>=-0.05)
        constraints_satisfied=pair_safe and obstacle_safe and static["feasible"]
        rejections={}
        if not pair_safe:rejections["VEHICLE_SEPARATION"]=1
        if not obstacle_safe:rejections["DYNAMIC_CLEARANCE"]=1
        if not static["feasible"]:rejections[static.get("error_code") or "STATIC_CONSTRAINT"]=1
        return {"viable":bool(constraints_satisfied),"algorithm":self.name,
            "command_type":self.command_type,"vehicle_count":count,
            "commands":commands,"selected_offset":None,
            "minimum_clearance_m":None,"candidates":[],
            "predicted_minimum_separation_m":(None if count<2 else round(minimum_pair,4)),
            "predicted_minimum_obstacle_clearance_m":(None if not checked else round(minimum_obstacle,4)),
            "static_validation":static,
            "constraints_satisfied":bool(constraints_satisfied),
            "reason":("3D ORCA velocity solution generated for external safety gating" if constraints_satisfied
                      else "3D ORCA command failed dynamic, static or fleet constraints; hold required"),
            "shadow_mode":False,"requires_external_safety_gate":True,"rejection_summary":rejections}


_PLANNERS={value.name:value for value in (CollectiveOffsetPlanner,Orca3DPlanner,DistributedMpcPlanner)}

def available_local_planners():return tuple(sorted(_PLANNERS))

def create_local_planner(name):
    key=str(name or "collective_offset").strip().lower()
    try:return _PLANNERS[key]()
    except KeyError:raise DynamicObstacleError(
        "unknown local avoidance algorithm {}; available: {}".format(
            key,",".join(available_local_planners())))
