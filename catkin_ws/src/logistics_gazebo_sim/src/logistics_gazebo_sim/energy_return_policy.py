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

    def update(self,report,now,positions=None,home_slots=None,phase=""):
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
            assignments=assign_return_slots(assessments,positions,home_slots)
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


def assign_return_slots(assessments,positions,home_slots):
    """Assign scarce short routes preferentially to aircraft with less energy."""
    count=len(assessments)
    if count>8:raise ValueError("slot assignment supports at most eight vehicles")
    ranked=sorted(range(count),key=lambda i:assessments[i]["final_margin_wh"])
    weights={vehicle:float(count-rank) for rank,vehicle in enumerate(ranked)}
    best=None
    for permutation in itertools.permutations(range(count)):
        cost=sum(weights[i]*_route_cost(positions[i],home_slots[permutation[i]]) for i in range(count))
        candidate=(round(cost,9),permutation)
        if best is None or candidate<best:best=candidate
    return {assessments[i]["vehicle_id"]:int(best[1][i]) for i in range(count)}


def _route_cost(position,slot):
    if len(position)!=3 or len(slot)!=3:raise ValueError("positions and slots must be xyz")
    dx=float(position[0])-float(slot[0]);dy=float(position[1])-float(slot[1]);dz=float(position[2])-float(slot[2])
    return math.hypot(dx,dy)+1.5*max(0.0,dz)+2.0*max(0.0,-dz)
