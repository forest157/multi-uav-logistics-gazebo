"""Deterministic, calibratable multicopter energy estimation."""
import math


class EnergyModel:
    def __init__(self,hover_power_w=180.0,horizontal_w_per_mps2=18.0,
                 climb_w_per_mps=80.0,descent_w_per_mps=20.0,
                 acceleration_w_per_mps2=12.0,turn_w_per_rad_s=8.0,
                 payload_w_per_kg=25.0):
        self.hover=float(hover_power_w);self.horizontal=float(horizontal_w_per_mps2)
        self.climb=float(climb_w_per_mps);self.descent=float(descent_w_per_mps)
        self.acceleration=float(acceleration_w_per_mps2);self.turn=float(turn_w_per_rad_s)
        self.payload=float(payload_w_per_kg)
        if min(self.hover,self.horizontal,self.climb,self.descent,self.acceleration,self.turn,self.payload)<0.0 or self.hover<=0.0:
            raise ValueError("energy coefficients must be non-negative and hover power positive")

    def power(self,velocity=(0,0,0),acceleration=(0,0,0),turn_rate=0.0,payload_kg=0.0):
        velocity=_vector(velocity,"velocity");acceleration=_vector(acceleration,"acceleration")
        payload=max(0.0,float(payload_kg));vz=velocity[2]
        watts=(self.hover+self.horizontal*(velocity[0]**2+velocity[1]**2)+
               self.climb*max(0.0,vz)+self.descent*max(0.0,-vz)+
               self.acceleration*math.sqrt(sum(axis*axis for axis in acceleration))+
               self.turn*abs(float(turn_rate))+self.payload*payload)
        if not math.isfinite(watts):raise ValueError("energy inputs must be finite")
        return watts

    def energy_for_seconds(self,seconds,**conditions):
        seconds=float(seconds)
        if seconds<0.0 or not math.isfinite(seconds):raise ValueError("duration must be finite and non-negative")
        return self.power(**conditions)*seconds/3600.0


class EnergyEstimator:
    def __init__(self,model=None,capacity_wh=220.0,reserve_fraction=.2):
        self.model=model or EnergyModel();self.capacity=float(capacity_wh);self.reserve=float(reserve_fraction)
        if self.capacity<=0.0 or not 0.0<=self.reserve<1.0:raise ValueError("battery capacity or reserve is invalid")
        self.used_wh=0.0;self.last_stamp=None;self.last_velocity=None;self.last_heading=None

    def update(self,stamp,velocity,payload_kg=0.0):
        stamp=float(stamp);velocity=_vector(velocity,"velocity")
        if self.last_stamp is None or stamp<=self.last_stamp:
            self.last_stamp=stamp;self.last_velocity=velocity;self.last_heading=_heading(velocity);return self.snapshot()
        dt=stamp-self.last_stamp
        acceleration=[(a-b)/dt for a,b in zip(velocity,self.last_velocity)]
        heading=_heading(velocity);turn_rate=0.0
        if heading is not None and self.last_heading is not None:
            turn_rate=abs(_angle_delta(heading,self.last_heading))/dt
        self.used_wh+=self.model.energy_for_seconds(dt,velocity=velocity,acceleration=acceleration,turn_rate=turn_rate,payload_kg=payload_kg)
        self.last_stamp=stamp;self.last_velocity=velocity
        if heading is not None:self.last_heading=heading
        return self.snapshot()

    def snapshot(self):
        remaining=max(0.0,self.capacity-self.used_wh);reserve_wh=self.capacity*self.reserve
        return {"capacity_wh":round(self.capacity,3),"used_wh":round(self.used_wh,4),
            "remaining_wh":round(remaining,4),"remaining_fraction":round(remaining/self.capacity,4),
            "reserve_wh":round(reserve_wh,3),"usable_margin_wh":round(remaining-reserve_wh,4)}


def mission_energy_forecast(model,remaining_s,return_s,wait_s,landing_altitude,
                            cruise_speed=2.0,descent_speed=.25,payload_kg=0.0):
    remaining=model.energy_for_seconds(remaining_s,velocity=(cruise_speed,0,0),payload_kg=payload_kg)
    returning=model.energy_for_seconds(return_s,velocity=(cruise_speed,0,0),payload_kg=payload_kg)
    waiting=model.energy_for_seconds(wait_s,payload_kg=payload_kg)
    landing_s=max(0.0,float(landing_altitude))/max(.01,float(descent_speed))
    landing=model.energy_for_seconds(landing_s,velocity=(0,0,-abs(float(descent_speed))),payload_kg=payload_kg)
    return {"task_remaining_wh":round(remaining,3),"return_wh":round(returning,3),
        "wait_wh":round(waiting,3),"landing_wh":round(landing,3),
        "required_to_land_wh":round(returning+waiting+landing,3)}


def _vector(value,name):
    if not isinstance(value,(list,tuple)) or len(value)!=3:raise ValueError("{} must contain xyz".format(name))
    result=tuple(float(axis) for axis in value)
    if not all(math.isfinite(axis) for axis in result):raise ValueError("{} must be finite".format(name))
    return result

def _heading(velocity):
    return math.atan2(velocity[1],velocity[0]) if math.hypot(velocity[0],velocity[1])>.05 else None

def _angle_delta(first,second):return (first-second+math.pi)%(2.0*math.pi)-math.pi
