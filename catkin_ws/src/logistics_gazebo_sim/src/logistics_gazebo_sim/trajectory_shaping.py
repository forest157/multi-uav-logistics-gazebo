"""Dynamically feasible trajectory shaping for local avoidance commands."""
import math
import numpy as np
from .dynamic_obstacles import interpolate_timed_path


def _limited_velocity(value,max_speed,max_climb_rate):
    velocity=np.asarray(value,dtype=float).copy();velocity[2]=np.clip(velocity[2],-max_climb_rate,max_climb_rate)
    norm=float(np.linalg.norm(velocity))
    if norm>max_speed:velocity*=max_speed/norm
    return velocity


def shape_timed_path(reference,initial_velocity,horizon=3.0,dt=0.25,max_speed=2.0,
                     max_acceleration=1.0,max_vertical_acceleration=0.6,
                     max_climb_rate=0.8,position_gain=1.0):
    """Track a timed reference with bounded velocity and per-axis acceleration."""
    values=np.asarray(reference,dtype=float)
    if values.ndim!=2 or values.shape[1]!=4 or len(values)<2:raise ValueError("reference must contain at least two [t,x,y,z] rows")
    if min(horizon,dt,max_speed,max_acceleration,max_vertical_acceleration,max_climb_rate)<=0.0:raise ValueError("trajectory shaping limits must be positive")
    start=float(values[0,0]);end=min(float(values[-1,0]),start+float(horizon));count=max(2,int(math.ceil((end-start)/dt))+1)
    times=np.linspace(start,end,count);position=values[0,1:].copy();velocity=_limited_velocity(initial_velocity,max_speed,max_climb_rate);output=[[float(times[0])]+position.tolist()]
    for previous_time,stamp in zip(times[:-1],times[1:]):
        step=float(stamp-previous_time);desired_position=interpolate_timed_path(values,float(stamp));lookahead=min(end,float(stamp)+step)
        desired_velocity=(interpolate_timed_path(values,lookahead)-desired_position)/max(step,lookahead-float(stamp)) if lookahead>float(stamp) else np.zeros(3)
        desired_velocity=_limited_velocity(desired_velocity+position_gain*(desired_position-position),max_speed,max_climb_rate)
        delta=desired_velocity-velocity;delta[:2]=np.clip(delta[:2],-max_acceleration*step,max_acceleration*step);delta[2]=np.clip(delta[2],-max_vertical_acceleration*step,max_vertical_acceleration*step)
        velocity=_limited_velocity(velocity+delta,max_speed,max_climb_rate);position=position+velocity*step;output.append([float(stamp)]+position.tolist())
    return output


def initial_path_velocity(path):
    values=np.asarray(path,dtype=float)
    if values.ndim!=2 or values.shape[1]!=4 or len(values)<2:raise ValueError("path must contain at least two [t,x,y,z] rows")
    dt=float(values[1,0]-values[0,0])
    if dt<=0.0:raise ValueError("path timestamps must increase")
    return (values[1,1:]-values[0,1:])/dt
