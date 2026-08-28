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


class DetectionAssociator:
    """Assign stable IDs and require repeated observations before publication."""
    def __init__(self,maximum_distance=1.8,confirmation_hits=3,maximum_misses=3):
        self.maximum_distance=float(maximum_distance);self.confirmation_hits=int(confirmation_hits)
        self.maximum_misses=int(maximum_misses);self.tracks={};self.sequence=0
    def update(self,detections):
        unmatched=set(self.tracks);assigned=[]
        for detection in detections:
            best=None;distance=self.maximum_distance
            for identity in unmatched:
                value=math.sqrt(sum((a-b)**2 for a,b in zip(detection["position"],self.tracks[identity]["position"])))
                if value<distance:best,distance=identity,value
            if best is None:
                best="lidar_target_{}".format(self.sequence);self.sequence+=1
                self.tracks[best]={"hits":0,"misses":0,"position":detection["position"]}
            else:unmatched.remove(best)
            track=self.tracks[best];track["hits"]+=1;track["misses"]=0;track["position"]=detection["position"]
            output=dict(detection);output["id"]=best;output["confirmation_hits"]=track["hits"]
            if track["hits"]>=self.confirmation_hits:assigned.append(output)
        for identity in unmatched:
            self.tracks[identity]["misses"]+=1
            if self.tracks[identity]["misses"]>self.maximum_misses:del self.tracks[identity]
        return assigned
