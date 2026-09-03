"""Runtime safety guards for measured fleet separation and avoidance commands."""
import math


def diagnostic_error_present(levels,error_level=2):
    """Treat only an explicit ERROR as unsafe; ROS STALE has numeric value 3."""
    return any(int(level)==int(error_level) for level in levels)


class FleetSeparationInterlock:
    """Latch a synchronized hold before measured vehicles can overlap."""
    def __init__(self,engage_distance=2.7,release_distance=3.0,release_delay=1.0):
        self.engage=float(engage_distance);self.release=float(release_distance)
        self.delay=float(release_delay);self.engaged=False;self.safe_since=None;self.reason=None
        if self.engage<=0.0 or self.release<=self.engage or self.delay<0.0:
            raise ValueError("fleet interlock limits are invalid")

    @staticmethod
    def minimum_distance(positions):
        values=[tuple(float(axis) for axis in value) for value in positions]
        if any(len(value)!=3 or not all(math.isfinite(axis) for axis in value) for value in values):
            raise ValueError("fleet positions must contain finite xyz values")
        if len(values)<2:return float("inf")
        return min(math.dist(values[i],values[j]) for i in range(len(values)) for j in range(i+1,len(values)))

    def update(self,positions,now,external_error=False):
        now=float(now);minimum=self.minimum_distance(positions)
        unsafe=bool(external_error) or minimum<self.engage
        if unsafe:
            self.engaged=True;self.safe_since=None
            self.reason=("fleet safety diagnostic error" if external_error else
                         "measured separation {:.3f} m below {:.3f} m".format(minimum,self.engage))
        elif self.engaged:
            if minimum<self.release:self.safe_since=None
            elif self.safe_since is None:self.safe_since=now
            elif now-self.safe_since>=self.delay:
                self.engaged=False;self.safe_since=None;self.reason=None
        return {"hold":self.engaged,"minimum_separation_m":round(minimum,3),"reason":self.reason}


def guarded_collective_targets(targets,command,obstacle_count):
    """Apply a validated offset only while risk exists, except smooth recovery."""
    values=[tuple(float(axis) for axis in target) for target in targets]
    command=command or {};state=str(command.get("state","IDLE")).upper()
    if command.get("action")!="AVOID":return values
    if int(obstacle_count)<=0 and state!="RECOVERING":return values
    offset=command.get("offset") or [0.0,0.0,0.0]
    if len(offset)!=3 or not all(math.isfinite(float(axis)) for axis in offset):
        raise ValueError("avoidance offset must contain finite xyz values")
    return [tuple(axis+float(delta) for axis,delta in zip(target,offset)) for target in values]
