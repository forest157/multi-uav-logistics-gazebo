"""Fail-safe, hysteretic energy-aware return recommendations."""
import itertools
import math


LEVELS={"NORMAL":0,"LOW":1,"CRITICAL":2,"STALE":3}


class EnergyReturnPolicy:
    def __init__(self,low_margin_wh=12.0,critical_margin_wh=0.0,
                 release_hysteresis_wh=3.0,stale_after_s=1.5):
        self.low=float(low_margin_wh);self.critical=float(critical_margin_wh)
        self.release=float(release_hysteresis_wh);self.stale_after=float(stale_after_s)
        if self.critical>self.low or min(self.release,self.stale_after)<0.0:
            raise ValueError("invalid energy return thresholds")
        self.levels={}

    def update(self,report,now,positions=None,home_slots=None,phase="",min_separation=3.3):
        stamp=float(report.get("stamp",-math.inf));age=max(0.0,float(now)-stamp)
        vehicles=list(report.get("vehicles") or [])
        if not vehicles or not math.isfinite(stamp) or age>self.stale_after:
            return self._result("STALE",age,[],{},[],"HOLD_RECOMMENDED")
        assessments=[]
        for index,value in enumerate(vehicles):
            vehicle_id=str(value.get("vehicle_id","uav{}".format(index)))
            available=float(value.get("usable_margin_wh",-math.inf))
            required=float(value.get("required_to_land_wh",math.inf))
            margin=available-required
            raw="CRITICAL" if margin<=self.critical else "LOW" if margin<=self.low else "NORMAL"
            previous=self.levels.get(vehicle_id,"NORMAL")
            level=self._latched(previous,raw,margin);self.levels[vehicle_id]=level
            assessments.append({"vehicle_id":vehicle_id,"level":level,"final_margin_wh":round(margin,3),
                "usable_margin_wh":available,"required_to_land_wh":required})
        fleet=max((value["level"] for value in assessments),key=lambda value:LEVELS[value])
        assignments={};open_air=phase in ("CRUISE_REFORMATION","RETURN")
        if open_air and positions and home_slots and len(positions)==len(home_slots)==len(assessments):
            assignments=assign_return_slots(assessments,positions,home_slots,min_separation)
        landing=[value["vehicle_id"] for value in sorted(assessments,key=lambda value:value["final_margin_wh"])]
        action="ALTERNATE_LANDING_RECOMMENDED" if fleet=="CRITICAL" else "RETURN_RECOMMENDED" if fleet=="LOW" else "MONITOR"
        return self._result(fleet,age,assessments,assignments,landing,action)

    def _latched(self,previous,raw,margin):
        if raw=="CRITICAL":return "CRITICAL"
        if previous=="CRITICAL" and margin<=self.critical+self.release:return "CRITICAL"
        if previous in ("LOW","CRITICAL") and margin<=self.low+self.release:return "LOW"
        return raw

    @staticmethod
    def _result(level,age,vehicles,assignments,landing,action):
        return {"mode":"shadow","fleet_level":level,"energy_age_s":round(age,3),"action":action,
            "vehicles":vehicles,"slot_assignments":assignments,"landing_order":landing,
            "control_applied":False}


def assign_return_slots(assessments,positions,home_slots,min_separation=3.3):
    """Assign scarce short routes preferentially to aircraft with less energy."""
    count=len(assessments)
    if count>8:raise ValueError("slot assignment supports at most eight vehicles")
    ranked=sorted(range(count),key=lambda i:assessments[i]["final_margin_wh"])
    weights={vehicle:float(count-rank) for rank,vehicle in enumerate(ranked)}
    best=None
    for permutation in itertools.permutations(range(count)):
        if not _transition_safe(positions,home_slots,permutation,float(min_separation)):continue
        cost=sum(weights[i]*_route_cost(positions[i],home_slots[permutation[i]]) for i in range(count))
        candidate=(round(cost,9),permutation)
        if best is None or candidate<best:best=candidate
    return {} if best is None else {assessments[i]["vehicle_id"]:int(best[1][i]) for i in range(count)}


def _transition_safe(positions,slots,permutation,min_separation):
    for step in range(21):
        ratio=step/20.0
        values=[tuple(float(start[a])+(float(slots[permutation[i]][a])-float(start[a]))*ratio for a in range(3)) for i,start in enumerate(positions)]
        if any(math.dist(values[i],values[j])<min_separation for i in range(len(values)) for j in range(i+1,len(values))):return False
    return True


def _route_cost(position,slot):
    if len(position)!=3 or len(slot)!=3:raise ValueError("positions and slots must be xyz")
    dx=float(position[0])-float(slot[0]);dy=float(position[1])-float(slot[1]);dz=float(position[2])-float(slot[2])
    return math.hypot(dx,dy)+1.5*max(0.0,dz)+2.0*max(0.0,-dz)


def staggered_descent_progress(elapsed,start,end,rank,delay_s):
    """Monotonic per-aircraft descent progress with an energy-priority delay."""
    duration=max(1e-6,float(end)-float(start));raw=(float(elapsed)-float(start)-int(rank)*float(delay_s))/duration
    raw=max(0.0,min(1.0,raw));return raw*raw*(3.0-2.0*raw)


def choose_alternate_landing_site(position,other_positions,primitives,world_limit=46.0,
                                  footprint_clearance=2.0,min_separation=3.3):
    """Choose the closest clear site with a collision-free level approach."""
    start=tuple(float(value) for value in position);directions=[(0.,0.)]
    for radius in (4.,8.,12.):
        directions.extend((radius*math.cos(index*math.pi/4),radius*math.sin(index*math.pi/4)) for index in range(8))
    for dx,dy in directions:
        candidate=(start[0]+dx,start[1]+dy,0.18)
        if max(abs(candidate[0]),abs(candidate[1]))>world_limit:continue
        if any(math.hypot(candidate[0]-other[0],candidate[1]-other[1])<min_separation for other in other_positions):continue
        if any(_footprint_distance(candidate,primitive)<footprint_clearance for primitive in primitives):continue
        if _level_approach_clear(start,candidate,primitives,footprint_clearance):return tuple(round(value,3) for value in candidate)
    return None


def _footprint_distance(point,primitive):
    if primitive["kind"]=="cylinder":return max(0.,math.hypot(point[0]-primitive["x"],point[1]-primitive["y"])-primitive["radius"])
    dx=max(abs(point[0]-primitive["x"])-primitive["half_x"],0.);dy=max(abs(point[1]-primitive["y"])-primitive["half_y"],0.)
    return math.hypot(dx,dy)


def _level_approach_clear(start,landing,primitives,clearance):
    for step in range(21):
        ratio=step/20.;point=(start[0]+(landing[0]-start[0])*ratio,start[1]+(landing[1]-start[1])*ratio,start[2])
        if any(point[2]<=primitive["height"]+.8 and _footprint_distance(point,primitive)<clearance for primitive in primitives):return False
    return True
