"""Pluggable local avoidance algorithms for scalable 3D fleets."""
import math
import heapq
import multiprocessing as mp
import threading
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
    """Fail-closed validation and rate limiting for ORCA flight commands.

    The planner velocity is an eventual target. Safety is checked against the
    complete measured-velocity-to-target acceleration ramp, while ``current``
    emits that ramp at control-loop rate. Treating the first small slew step as
    a velocity held for the whole prediction horizon can turn a valid sidestep
    into an unsafe hover in front of an incoming obstacle.
    """
    def __init__(self, vehicle_count, max_speed=2.0, max_climb_rate=0.8,
                 max_acceleration=1.0, smoothing=0.35, timeout=0.6):
        self.vehicle_count=int(vehicle_count);self.max_speed=float(max_speed)
        self.max_climb_rate=float(max_climb_rate);self.max_acceleration=float(max_acceleration)
        self.smoothing=float(smoothing);self.timeout=float(timeout)
        if self.vehicle_count<1 or min(self.max_speed,self.max_climb_rate,self.max_acceleration,self.timeout)<=0.0:
            raise DynamicObstacleError("ORCA command limits must be positive")
        if not 0.0<self.smoothing<=1.0:
            raise DynamicObstacleError("ORCA smoothing must be in (0,1]")
        self._lock=threading.RLock()
        self.reset()
    def reset(self):
        with self._lock:
            self.current_velocities=None
            self.target_velocities=None
            self.last_output_time=None
            self.valid_until=None
    def _bounded_commands(self, commands):
        by_id={item.get("vehicle_id"):item for item in commands if isinstance(item,dict)}
        expected={"uav{}".format(index) for index in range(self.vehicle_count)}
        if set(by_id)!=expected:
            raise DynamicObstacleError("ORCA command vehicle ids mismatch")
        desired=[];preferred=[]
        for index in range(self.vehicle_count):
            item=by_id["uav{}".format(index)]
            target=_clamp(_vector(item.get("velocity"),"ORCA velocity"),self.max_speed)
            target[2]=max(-self.max_climb_rate,min(self.max_climb_rate,target[2]))
            base=_clamp(_vector(item.get("preferred_velocity"),"ORCA preferred velocity"),self.max_speed)
            base[2]=max(-self.max_climb_rate,min(self.max_climb_rate,base[2]))
            desired.append(target);preferred.append(base)
        return desired,preferred
    def _safe_target(self, desired, context, now):
        obstacles=bool(context.get("obstacles"))
        vertical_errors=context.get("nominal_vertical_errors")
        vertical_recovery=(not obstacles and isinstance(vertical_errors,(list,tuple))
                           and len(vertical_errors)==self.vehicle_count
                           and all(np.isfinite(float(value)) for value in vertical_errors)
                           and max(abs(float(value)) for value in vertical_errors)>0.8)
        full_candidates=[]
        if self.target_velocities is not None:
            full_candidates.append([old+self.smoothing*(new-old)
                                    for old,new in zip(self.target_velocities,desired)])
        full_candidates.append(desired)
        # While tracking an obstacle, prefer level-flight solutions whenever
        # independently safe. This prevents lidar centroid height noise from
        # becoming an unexplained climb/descent avoidance command. Once tracks
        # clear, a validated vertical target may return to the nominal route.
        candidates=[];prefer_horizontal=obstacles
        for candidate in full_candidates:
            horizontal=[np.asarray([value[0],value[1],0.],dtype=float)
                        for value in candidate]
            if not obstacles and not vertical_recovery:
                candidates.append(horizontal)
            else:
                candidates.extend((horizontal,candidate) if prefer_horizontal
                                  else (candidate,horizontal))
        for candidate in candidates:
            if validate_orca_execution(candidate,context,now,self.max_acceleration):
                return [np.asarray(value,dtype=float) for value in candidate]
        # Search coherent horizontal maneuvers only after the planner target
        # fails. A shared correction preserves formation and deliberately
        # avoids inventing a vertical response to lidar noise.
        alternatives=[]
        horizontal_desired=[np.asarray([value[0],value[1],0.],dtype=float)
                            for value in desired]
        speeds=np.linspace(0.5,self.max_speed,max(1,int(math.ceil(self.max_speed/0.5))))
        for angle in np.linspace(0.,2.*math.pi,16,endpoint=False):
            direction=np.asarray([math.cos(angle),math.sin(angle),0.])
            for speed in speeds:
                common=direction*speed
                alternatives.append([_clamp(value+common,self.max_speed)
                                     for value in horizontal_desired])
                alternatives.append([common.copy() for _ in desired])
        alternatives.sort(key=lambda values:sum(float(np.sum((value-target)**2))
                                                   for value,target in zip(values,horizontal_desired)))
        selected=next((values for values in alternatives
                       if validate_orca_execution(values,context,now,self.max_acceleration)),None)
        if selected is None:
            selected=self._independent_emergency_target(
                horizontal_desired,context,now)
        if selected is None:
            raise DynamicObstacleError("conditioned ORCA trajectory is unsafe")
        return [np.asarray(value,dtype=float) for value in selected]
    def _independent_emergency_target(self, desired, context, now):
        """Find a validated per-vehicle horizontal escape when a common one cannot.

        A crossing obstacle can threaten successive members of a formation, so
        forcing every aircraft to use the same fallback velocity may have no
        solution even though a safe split maneuver exists. Each candidate is
        first checked for that aircraft, then complete combinations are checked
        for fleet separation and static-map clearance. The bounded best-first
        search keeps this exceptional path deterministic and finite.
        """
        if self.vehicle_count<=1:return None
        positions=context.get("positions",[]);measured=context.get("velocities",[])
        if len(positions)!=self.vehicle_count or len(measured)!=self.vehicle_count:
            raise DynamicObstacleError("ORCA execution context count mismatch")
        directions=[np.asarray([math.cos(angle),math.sin(angle),0.],dtype=float)
                    for angle in np.linspace(0.,2.*math.pi,72,endpoint=False)]
        pools=[]
        for index,target in enumerate(desired):
            subset=dict(context,positions=[positions[index]],
                        velocities=[measured[index]])
            raw=[np.asarray(target,dtype=float)]+[
                direction*self.max_speed for direction in directions]
            unique=[];seen=set()
            for value in raw:
                key=tuple(round(float(axis),6) for axis in value)
                if key in seen:continue
                seen.add(key)
                if validate_orca_execution([value],subset,now,
                                           self.max_acceleration):
                    unique.append(value)
            if not unique:return None
            unique.sort(key=lambda value:float(np.sum((value-target)**2)))
            pools.append(unique)
        start=tuple(0 for _ in pools);queue=[(0.0,start)];visited={start}
        checks=0
        while queue and checks<512:
            _score,indices=heapq.heappop(queue)
            values=[pools[i][index] for i,index in enumerate(indices)]
            checks+=1
            if validate_orca_execution(values,context,now,self.max_acceleration):
                return values
            for axis in range(len(indices)):
                if indices[axis]+1>=len(pools[axis]):continue
                updated=list(indices);updated[axis]+=1;updated=tuple(updated)
                if updated in visited:continue
                visited.add(updated)
                score=sum(float(np.sum((pools[i][index]-desired[i])**2))
                          for i,index in enumerate(updated))
                heapq.heappush(queue,(score,updated))
        return None
    def _emit_locked(self, now):
        now=float(now)
        if self.current_velocities is None or self.target_velocities is None:
            raise DynamicObstacleError("ORCA command is unavailable")
        if self.valid_until is None or now>self.valid_until:
            raise DynamicObstacleError("ORCA command expired during execution")
        if self.last_output_time is None:self.last_output_time=now
        dt=max(0.0,now-self.last_output_time)
        for index,(current,target) in enumerate(zip(self.current_velocities,self.target_velocities)):
            updated=current+_clamp(target-current,self.max_acceleration*dt)
            updated=_clamp(updated,self.max_speed)
            updated[2]=max(-self.max_climb_rate,min(self.max_climb_rate,updated[2]))
            self.current_velocities[index]=updated
        self.last_output_time=now
        return [tuple(float(axis) for axis in value) for value in self.current_velocities]
    def current(self, now):
        """Return the next acceleration-bounded command for the control loop."""
        with self._lock:return self._emit_locked(now)
    def refresh(self, safety_context, now):
        """Revalidate a cached target against the newest sensor estimates."""
        with self._lock:
            if self.target_velocities is None:
                raise DynamicObstacleError("ORCA command is unavailable")
            vertical_errors=safety_context.get("nominal_vertical_errors")
            if (not safety_context.get("obstacles") and
                    isinstance(vertical_errors,(list,tuple)) and
                    len(vertical_errors)==self.vehicle_count and
                    all(np.isfinite(float(value)) and abs(float(value))<=0.8
                        for value in vertical_errors)):
                horizontal=[np.asarray([value[0],value[1],0.],dtype=float)
                            for value in self.target_velocities]
                if validate_orca_execution(horizontal,safety_context,now,
                                           self.max_acceleration):
                    self.target_velocities=horizontal
            if not validate_orca_execution(self.target_velocities,safety_context,now,
                                           self.max_acceleration):
                raise DynamicObstacleError("cached ORCA trajectory is unsafe")
            self.valid_until=float(now)+self.timeout
            return self._emit_locked(now)
    def condition(self, plan, now, dt, safety_context=None):
        if not isinstance(plan,dict) or (not plan.get("viable") and safety_context is None):
            raise DynamicObstacleError("ORCA plan is not viable")
        if plan.get("contract_version")!="orca_velocity_v1" or plan.get("algorithm")!="orca3d" or plan.get("command_type")!="per_vehicle_velocity":
            raise DynamicObstacleError("unexpected ORCA command contract")
        if safety_context is None and (not plan.get("constraints_satisfied") or not (plan.get("static_validation") or {}).get("feasible")):
            raise DynamicObstacleError("ORCA command lacks independent safety validation")
        stamp=float(plan.get("stamp",-1.0))
        age=float(now)-stamp
        validity=min(self.timeout,float(plan.get("valid_for_s",self.timeout)))
        if not np.isfinite(stamp) or validity<=0.0 or age<0.0 or age>validity:
            raise DynamicObstacleError("ORCA command is stale")
        commands=plan.get("commands")
        if not isinstance(commands,list) or len(commands)!=self.vehicle_count:
            raise DynamicObstacleError("ORCA command vehicle count mismatch")
        desired,preferred=self._bounded_commands(commands)
        with self._lock:
            target=(self._safe_target(desired,safety_context,now)
                    if safety_context is not None else desired)
            if self.current_velocities is None:
                initial=(safety_context or {}).get("velocities",preferred)
                if len(initial)!=self.vehicle_count:
                    raise DynamicObstacleError("ORCA execution velocity count mismatch")
                self.current_velocities=[]
                for value in initial:
                    bounded=_clamp(_vector(value,"measured velocity"),self.max_speed)
                    bounded[2]=max(-self.max_climb_rate,min(self.max_climb_rate,bounded[2]))
                    self.current_velocities.append(bounded)
                self.last_output_time=float(now)
            self.target_velocities=[np.asarray(value,dtype=float) for value in target]
            # A fresh context validation can safely bridge planner latency, but
            # never beyond the gate timeout without another sensor recheck.
            self.valid_until=(float(now)+self.timeout if safety_context is not None
                              else stamp+validity)
            return self._emit_locked(now)


def validate_orca_execution(velocities, context, now, max_acceleration=1.0):
    """Recheck bounded commands with measured motion and a delayed acceleration ramp.

    Truth must never enter this context: use timestamped fleet estimates and
    sensor tracks. Segment closest approach catches between-sample crossings.
    """
    required_fields={"stamp","positions","velocities","obstacles"}
    if not isinstance(context,dict) or not required_fields.issubset(context):
        raise DynamicObstacleError("ORCA execution context is incomplete")
    age=float(now)-float(context["stamp"])
    if not math.isfinite(age) or not 0.0<=age<=0.5:
        raise DynamicObstacleError("ORCA execution context is stale")
    positions=np.asarray([_vector(p,"execution position") for p in context["positions"]])
    measured=np.asarray([_vector(v,"measured velocity") for v in context["velocities"]])
    commands=np.asarray([_vector(v,"execution velocity") for v in velocities])
    if positions.shape!=commands.shape or measured.shape!=commands.shape:
        raise DynamicObstacleError("ORCA execution context count mismatch")
    obstacles=[validate_obstacle(o) for o in context["obstacles"]]
    for obstacle in obstacles:
        _vector(obstacle["position"],"obstacle position");_vector(obstacle["velocity"],"obstacle velocity")
        if not math.isfinite(obstacle["radius"]) or not math.isfinite(obstacle["height"]):
            raise DynamicObstacleError("obstacle dimensions must be finite")
    horizon=float(context.get("horizon",8.0));delay=float(context.get("control_delay",0.6))
    if not math.isfinite(horizon) or not 0.0<horizon<=10.0 or not 0.0<=delay<=2.0:
        raise DynamicObstacleError("invalid ORCA execution horizon")
    count=max(2,int(math.ceil(horizon/0.1)));dt=horizon/count
    positions=positions+age*measured
    path=[positions.copy()];motion=measured.copy()
    for k in range(count):
        next_motion=motion.copy()
        if k*dt>=delay:
            for i in range(len(commands)):
                next_motion[i]+=_clamp(commands[i]-motion[i],max_acceleration*dt)
        positions=positions+0.5*(motion+next_motion)*dt
        path.append(positions.copy());motion=next_motion
    path=np.asarray(path);times=np.arange(count+1)*dt
    def closest(relative):
        start=relative[:-1];delta=np.diff(relative,axis=0)
        fraction=np.clip(-np.sum(start*delta,axis=1)/np.maximum(1e-12,np.sum(delta*delta,axis=1)),0.,1.)
        return float(np.min(np.linalg.norm(start+fraction[:,None]*delta,axis=1)))
    required_clearance=float(context.get("required_clearance",.1))
    if not math.isfinite(required_clearance) or required_clearance<0.0:
        raise DynamicObstacleError("invalid ORCA required clearance")
    for i in range(len(commands)):
        for j in range(i):
            if closest(path[:,i]-path[:,j])<float(context.get("minimum_separation",3.0))+0.1:return False
        for obstacle in obstacles:
            future=obstacle["position"]+(times+age)[:,None]*obstacle["velocity"]
            required=1.2+max(obstacle["radius"],0.5*obstacle["height"])+0.5+required_clearance
            if closest(path[:,i]-future)<required:return False
    scene=context.get("scene_id")
    if scene is not None:
        paths=[np.column_stack((times,path[:,i])).tolist() for i in range(len(commands))]
        if not validate_static_paths(scene,paths)["feasible"]:return False
    return True


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
        obstacle_margin=float(options.get("mpc_obstacle_margin",0.0));own_obstacle_weight=float(options.get("mpc_obstacle_weight",2400.0));shared_obstacle_weight=float(options.get("mpc_shared_obstacle_weight",1400.0));direction_weight=float(options.get("mpc_direction_weight",800.0))
        boundary_margin=float(options.get("mpc_boundary_margin",0.3));boundary_weight=float(options.get("mpc_boundary_weight",3000.0));xy_limit=float(options.get("mpc_xy_limit",48.8));z_lower=float(options.get("mpc_z_lower",3.6));z_upper=float(options.get("mpc_z_upper",44.4))
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
        total_budget_s=float(options.get("mpc_total_timeout_s",0.45));plan_deadline=time.perf_counter()+total_budget_s
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
                    xy_excess=max(0.0,max(abs(float(position[0])),abs(float(position[1])))-(xy_limit-boundary_margin));z_excess=max(0.0,z_lower+boundary_margin-float(position[2]))+max(0.0,float(position[2])-(z_upper-boundary_margin));cost+=boundary_weight*(xy_excess*xy_excess+z_excess*z_excess)
                    for obstacle in checked:
                        obstacle_center=obstacle["position"]+obstacle["velocity"]*seconds;safe=1.2+obstacle["radius"]+0.5+required_clearance+obstacle_margin
                        for source in range(count):
                            formation_delta=references[vehicle][step_index]-references[source][step_index]
                            center=obstacle_center+formation_delta;gap=float(np.linalg.norm(position-center));weight=own_obstacle_weight if source==vehicle else shared_obstacle_weight
                            cost+=weight*max(0.0,safe-gap)**2
                            if gap<2.0*safe:
                                lateral=float(np.dot(position-center,avoid_direction));cost+=direction_weight*max(0.0,safe-lateral)**2
                    for peer in range(count):
                        if peer==vehicle:continue
                        peer_position=references[peer][step_index];gap=float(np.linalg.norm(position-peer_position));cost+=5000.0*max(0.0,separation-gap)**2
                        nominal_relative=references[vehicle][step_index]-peer_position;actual_relative=position-peer_position;cost+=2.0*float(np.sum((actual_relative-nominal_relative)**2))
                return cost
            use_warm=bool(warm_valid and objective(previous.ravel())<=objective(cold.ravel()))
            if not use_warm:previous=cold.copy()
            warm_used.append(use_warm)
            vehicle_timeout_s=float(options.get("mpc_vehicle_timeout_s",0.35));timeout_s=max(0.001,min(vehicle_timeout_s,plan_deadline-time.perf_counter()))
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
                elif parent_pipe.poll():
                    try:value=parent_pipe.recv()
                    except EOFError:value={"error":"vehicle solver pipe closed before result"}
                else:value={"error":"vehicle solver exited with code {}".format(process.exitcode)}
                parent_pipe.close();return value
            solver=launch_solver();retried=False
            remaining=plan_deadline-time.perf_counter()
            if solver.get("error") and "timeout" not in solver["error"] and remaining>0.005:
                timeout_s=max(0.001,min(vehicle_timeout_s,remaining));solver=launch_solver();retried=True
            solution=np.asarray(solver.get("x",previous.ravel()),dtype=float);positions,velocities,accelerations=rollout(solution);elapsed_ms=1000.0*(time.perf_counter()-start_clock)
            success=bool(not solver.get("error") and solver.get("success") and np.all(np.isfinite(positions)) and np.isfinite(solver.get("fun",float("nan"))));all_success=all_success and success
            trajectory=[[round(float(query[index]),3)]+[round(float(axis),4) for axis in positions[index]] for index in range(steps+1)]
            trajectories.append(trajectory);solve_times.append(elapsed_ms);iterations.append(int(solver.get("iterations",0)));next_warm.append(accelerations.copy() if success else cold.copy())
            commands.append({"vehicle_id":"uav{}".format(vehicle),"velocity":[round(float(axis),4) for axis in velocities[0]],"preferred_velocity":[round(float(axis),4) for axis in initial_velocities[vehicle]],"acceleration":[round(float(axis),4) for axis in accelerations[0]],"solver_success":success,"solver_status":int(solver.get("status",-2)),"solver_message":solver.get("error",solver.get("message","unknown solver result")),"iterations":int(solver.get("iterations",0)),"solve_time_ms":round(elapsed_ms,3),"solver_retried":retried})
        consensus_strength=float(options.get("mpc_consensus_strength",1.0))
        if not 0.0<=consensus_strength<=1.0:raise DynamicObstacleError("MPC consensus strength must be in [0,1]")
        if count>1 and consensus_strength>0.0:
            planned=np.asarray([[row[1:] for row in path] for path in trajectories],dtype=float);reference_stack=np.asarray(references,dtype=float);displacements=planned-reference_stack;shared=np.mean(displacements,axis=0)
            projected=reference_stack+(1.0-consensus_strength)*displacements+consensus_strength*shared[None,:,:]
            for vehicle in range(count):
                trajectories[vehicle]=[[round(float(query[index]),3)]+[round(float(axis),4) for axis in projected[vehicle,index]] for index in range(steps+1)]
                velocity=(projected[vehicle,1]-projected[vehicle,0])/dt;acceleration=(velocity-initial_velocities[vehicle])/dt
                commands[vehicle]["velocity"]=[round(float(axis),4) for axis in velocity];commands[vehicle]["acceleration"]=[round(float(axis),4) for axis in acceleration]
        self._warm_accelerations=next_warm;self._warm_starts=[value.copy() for value in starts];self._warm_shape=(count,steps)
        nominal_trajectories=[[[round(float(query[index]),3)]+[round(float(axis),4) for axis in references[vehicle][index]] for index in range(steps+1)] for vehicle in range(count)]
        nominal_fleet=assess_fleet_separation(nominal_trajectories,horizon=steps*dt,minimum_separation=separation)
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
        if not fleet["safe"]:rejections["NOMINAL_VEHICLE_SEPARATION" if not nominal_fleet["safe"] else "VEHICLE_SEPARATION"]=1
        if not static["feasible"]:rejections[static.get("error_code") or "STATIC_CONSTRAINT"]=1
        return {"viable":bool(viable),"algorithm":self.name,"command_type":self.command_type,"vehicle_count":count,"commands":commands,"trajectories":trajectories,"constraints_satisfied":bool(dynamic_safe and fleet["safe"] and static["feasible"]),"static_validation":static,"fleet_separation":fleet,"nominal_fleet_separation":nominal_fleet,"dynamic_reports":reports,"solve_time_ms":{"total":round(sum(solve_times),3),"maximum":round(max(solve_times),3),"mean":round(sum(solve_times)/len(solve_times),3)},"iterations":{"maximum":max(iterations),"mean":round(sum(iterations)/len(iterations),2)},"warm_started_vehicle_count":sum(1 for value in warm_used if value),"consensus_strength":consensus_strength,"reason":"distributed MPC shadow trajectory generated" if viable else "distributed MPC failed solver or independent safety validation","shadow_mode":True,"requires_external_safety_gate":True,"solver_isolated":True,"rejection_summary":rejections}


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
