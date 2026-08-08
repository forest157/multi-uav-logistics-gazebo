"""Scalable xyz formation generation and minimum-crossing slot assignment."""
import math
import numpy as np
from scipy.optimize import linear_sum_assignment

FORMATION_NAMES=("triangle","inverted","row","column","vertical","wedge3d","helix")

def _center(points):
    points=np.asarray(points,dtype=float)
    return points-points.mean(axis=0) if len(points) else points

def initial_line(count,spacing=3.0):
    return _center([[i*spacing,0.0,0.0] for i in range(count)])

def generate(kind,count,spacing=3.0):
    if count<1:raise ValueError("vehicle_count must be positive")
    if kind=="row":
        points=[[0.0,(i-(count-1)/2.0)*spacing,0.0] for i in range(count)]
    elif kind=="column":
        points=[[(i-(count-1)/2.0)*spacing,0.0,0.0] for i in range(count)]
    elif kind=="vertical":
        points=[[0.0,0.0,(i-(count-1)/2.0)*spacing] for i in range(count)]
    elif kind in ("triangle","inverted"):
        points=[];placed=0;row=0
        while placed<count:
            number=min(row+1,count-placed)
            x=-row*spacing*0.866
            for index in range(number):
                y=(index-(number-1)/2.0)*spacing
                points.append([x,y,0.0]);placed+=1
            row+=1
        if kind=="inverted":
            points=[[-x,-y,z] for x,y,z in points]
    elif kind=="wedge3d":
        points=[[0.0,0.0,0.0]]
        for index in range(1,count):
            row=(index+1)//2;side=1 if index%2 else -1
            points.append([-row*spacing*.7,side*row*spacing*.55,
                           side*spacing*.5])
    elif kind=="helix":
        radius=max(spacing,spacing*math.sqrt(count)/2.0)
        points=[[radius*math.cos(2*math.pi*i/count),
                 radius*math.sin(2*math.pi*i/count),
                 (i-(count-1)/2.0)*spacing*.65] for i in range(count)]
    else:raise ValueError("unknown formation: "+kind)
    return _center(points)

def assign_slots(current,target):
    current=np.asarray(current,dtype=float);target=np.asarray(target,dtype=float)
    rows,cols=linear_sum_assignment(
        np.linalg.norm(current[:,None,:]-target[None,:,:],axis=2))
    assigned=np.zeros_like(target)
    assigned[rows]=target[cols]
    return assigned

def envelope(offsets,vehicle_radius=1.2,vertical_radius=.6):
    values=np.asarray(offsets,dtype=float)
    horizontal=max(np.linalg.norm(values[:,:2],axis=1))+vehicle_radius
    below=max(0.0,-float(values[:,2].min()))+vertical_radius
    above=max(0.0,float(values[:,2].max()))+vertical_radius
    return float(horizontal),float(below),float(above)

def minimum_separation(offsets):
    values=np.asarray(offsets,dtype=float)
    if len(values)<2:return float("inf")
    return min(np.linalg.norm(values[i]-values[j])
               for i in range(len(values)) for j in range(i+1,len(values)))
