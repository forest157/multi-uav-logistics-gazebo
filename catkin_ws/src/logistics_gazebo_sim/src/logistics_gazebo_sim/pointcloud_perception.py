"""Dependency-light filters and clustering for world-frame 3D lidar points."""
import math
from collections import defaultdict, deque


def finite_points(points):
    return [tuple(map(float,p)) for p in points if len(p)==3 and all(math.isfinite(v) for v in p)]


def exclude_near_vehicles(points,vehicles,radius=1.0):
    radius2=float(radius)**2
    return [p for p in points if all(sum((a-b)**2 for a,b in zip(p,v))>radius2 for v in vehicles)]


class VoxelBackground:
    def __init__(self,voxel_size=0.45,background_hits=4,forget_after=40):
        self.size=float(voxel_size);self.hits={};self.last={};self.frame=0

        self.background_hits=int(background_hits);self.forget_after=int(forget_after)
    def key(self,point):return tuple(int(math.floor(axis/self.size)) for axis in point)
    def update(self,points):
        self.frame+=1;keys=set(self.key(point) for point in points);dynamic=[]
        for point in points:
            key=self.key(point)
            if self.hits.get(key,0)<self.background_hits:dynamic.append(point)
        for key in keys:self.hits[key]=min(self.background_hits,self.hits.get(key,0)+1);self.last[key]=self.frame
        for key,seen in list(self.last.items()):
            if self.frame-seen>self.forget_after:self.last.pop(key,None);self.hits.pop(key,None)
        return dynamic


def euclidean_clusters(points,tolerance=1.0,minimum_points=6,maximum_points=500):
    """Grid-accelerated connected components with a Euclidean radius gate."""
    points=finite_points(points);cell=max(0.05,float(tolerance));grid=defaultdict(list)
    for index,p in enumerate(points):grid[tuple(int(math.floor(v/cell)) for v in p)].append(index)
    visited=set();clusters=[];limit2=cell*cell
    for seed in range(len(points)):
        if seed in visited:continue
        visited.add(seed);queue=deque([seed]);members=[]
        while queue:
            current=queue.popleft();members.append(current);base=points[current];key=tuple(int(math.floor(v/cell)) for v in base)
            for dx in (-1,0,1):
                for dy in (-1,0,1):
                    for dz in (-1,0,1):
                        for other in grid.get((key[0]+dx,key[1]+dy,key[2]+dz),[]):
                            if other in visited:continue
                            if sum((a-b)**2 for a,b in zip(base,points[other]))<=limit2:visited.add(other);queue.append(other)
        if minimum_points<=len(members)<=maximum_points:clusters.append([points[index] for index in members])
    return clusters


def cluster_detection(identity,cluster):
    count=float(len(cluster));center=[sum(p[axis] for p in cluster)/count for axis in range(3)]
    spans=[max(p[axis] for p in cluster)-min(p[axis] for p in cluster) for axis in range(3)]
    return {"id":identity,"position":center,"radius":max(0.2,0.5*max(spans[0],spans[1])),
            "height":max(0.2,spans[2]),"confidence":min(1.0,len(cluster)/30.0),"point_count":len(cluster)}

def target_sized_detections(detections,maximum_radius=1.4,maximum_height=2.0):
    """Keep compact airborne targets and reject large static-scene fragments."""
    maximum_radius=float(maximum_radius);maximum_height=float(maximum_height)
    if maximum_radius<=0.0 or maximum_height<=0.0:raise ValueError("target size limits must be positive")
    return [item for item in detections if 0.0<float(item.get("radius",0.0))<=maximum_radius and 0.0<float(item.get("height",0.0))<=maximum_height]


def calibrated_detection_confidence(detection,motion_hits,confirmation_hits):
    """Combine point support and motion consistency into an interpretable score."""
    support=min(1.0,max(0.0,float(detection.get("point_count",0)))/30.0)
    required=max(1,int(confirmation_hits)-1)
    motion=min(1.0,max(0.0,float(motion_hits))/required)
    return round(0.65*support+0.35*motion,3)


class DetectionAssociator:
    """Assign stable IDs and confirm only consistently moving observations."""
    def __init__(self,maximum_distance=1.8,confirmation_hits=3,maximum_misses=3,
                 minimum_speed=1.5,maximum_speed=12.0,
                 minimum_direction_cosine=0.7,maximum_acceleration=5.0):
        self.maximum_distance=float(maximum_distance);self.confirmation_hits=int(confirmation_hits)
        self.maximum_misses=int(maximum_misses);self.minimum_speed=float(minimum_speed)
        self.maximum_speed=float(maximum_speed);self.minimum_direction_cosine=float(minimum_direction_cosine)
        self.maximum_acceleration=float(maximum_acceleration)
        if self.minimum_speed<0.0 or self.maximum_speed<=self.minimum_speed:
            raise ValueError("motion speed limits are invalid")
        if not -1.0<=self.minimum_direction_cosine<=1.0:
            raise ValueError("minimum direction cosine must be between -1 and 1")
        if self.maximum_acceleration<=0.0:raise ValueError("maximum acceleration must be positive")
        self.tracks={};self.sequence=0;self.frame=0
    def update(self,detections,stamp=None):
        self.frame+=1;stamp=float(self.frame*0.2 if stamp is None else stamp)
        unmatched=set(self.tracks);assigned=[]
        for detection in detections:
            best=None;distance=self.maximum_distance
            for identity in unmatched:
                value=math.sqrt(sum((a-b)**2 for a,b in zip(detection["position"],self.tracks[identity]["position"])))
                if value<distance:best,distance=identity,value
            if best is None:
                best="lidar_target_{}".format(self.sequence);self.sequence+=1
                self.tracks[best]={"hits":0,"motion_hits":0,"misses":0,
                    "position":detection["position"],"stamp":stamp,"velocity":None}
            else:unmatched.remove(best)
            track=self.tracks[best];previous=track["position"];dt=stamp-track["stamp"]
            velocity=None
            if track["hits"] and dt>0.0:
                velocity=[(float(a)-float(b))/dt for a,b in zip(detection["position"],previous)]
                speed=math.sqrt(sum(axis*axis for axis in velocity));consistent=self.minimum_speed<=speed<=self.maximum_speed
                prior=track.get("velocity")
                if consistent and prior is not None:
                    prior_speed=math.sqrt(sum(axis*axis for axis in prior))
                    cosine=sum(a*b for a,b in zip(velocity,prior))/(speed*prior_speed) if prior_speed>1e-9 else -1.0
                    acceleration=math.sqrt(sum((a-b)**2 for a,b in zip(velocity,prior)))/dt
                    consistent=cosine>=self.minimum_direction_cosine and acceleration<=self.maximum_acceleration
                track["motion_hits"]=track["motion_hits"]+1 if consistent else 0
            track["hits"]+=1;track["misses"]=0;track["position"]=detection["position"];track["stamp"]=stamp
            if velocity is not None:track["velocity"]=velocity
            output=dict(detection);output["id"]=best;output["confirmation_hits"]=track["hits"]
            output["motion_hits"]=track["motion_hits"]
            output["confidence"]=calibrated_detection_confidence(output,track["motion_hits"],self.confirmation_hits)
            if track["hits"]>=self.confirmation_hits and track["motion_hits"]>=self.confirmation_hits-1:assigned.append(output)
        for identity in unmatched:
            self.tracks[identity]["misses"]+=1
            if self.tracks[identity]["misses"]>self.maximum_misses:del self.tracks[identity]
        return assigned
