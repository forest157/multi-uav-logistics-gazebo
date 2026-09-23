"""Static audit for reproducible, real-scale Gazebo Classic worlds."""
import json
import os
from xml.etree import ElementTree

from .outdoor_preflight import metadata_for_world
from .worlds import OUTDOOR_LAYOUTS


def audit_world(path,max_models=100,max_bytes=250000):
    size=os.path.getsize(path);root=ElementTree.parse(path).getroot();world=root.find("world")
    if world is None:return {"path":path,"pass":False,"errors":["missing world"]}
    models=world.findall("model");collisions=world.findall("model/link/collision");visuals=world.findall("model/link/visual")
    names=[model.get("name","") for model in models];errors=[]
    by_name={model.get("name"):model for model in models}
    for name,roof in by_name.items():
        if not name.endswith("_roof"):continue
        try:
            body=by_name[name[:-5]]
            visual_size=list(map(float,roof.find("link/visual/geometry/box/size").text.split()))
            collision_size=list(map(float,body.find("link/collision/geometry/box/size").text.split()))
            visual_pose=list(map(float,roof.find("pose").text.split()))
            collision_pose=list(map(float,body.find("pose").text.split()))
            if any(visual_pose[3:]+collision_pose[3:]):raise ValueError("rotated roof requires oriented collision audit")
            if any(abs(visual_pose[a]-collision_pose[a])+visual_size[a]/2.>collision_size[a]/2.+1e-6 for a in range(3)):
                errors.append(name+": roof visual exceeds collision volume")
        except (KeyError,AttributeError,TypeError,ValueError,IndexError):
            errors.append(name+": invalid or unsupported roof geometry")
    if root.get("version")!="1.6":errors.append("SDF must remain Gazebo Classic 1.6")
    if len(names)!=len(set(names)):errors.append("model names must be unique")
    if len(models)>max_models:errors.append("model budget exceeded")
    if size>max_bytes:errors.append("world file budget exceeded")
    stem=os.path.splitext(os.path.basename(path))[0]
    if stem in OUTDOOR_LAYOUTS:
        try:
            expected=dict(OUTDOOR_LAYOUTS[stem],**metadata_for_world(stem,path))
            expected=json.loads(json.dumps(expected))
            with open(os.path.splitext(path)[0]+'.json',encoding='utf-8') as stream:
                saved=json.load(stream)
            if saved!=expected:errors.append('outdoor metadata differs from world or layout')
        except (OSError,ValueError,KeyError,TypeError,ElementTree.ParseError):
            errors.append('outdoor world geometry or metadata is invalid')
    elif stem=='outdoor_logistics':
        for road_name in ('road_east_west','road_north_south'):
            road=by_name.get(road_name)
            if road is None or road.find('link/visual') is None or road.find('link/collision') is not None:
                errors.append(road_name+': missing visual-only road')
        for name,model in by_name.items():
            if name.startswith(('warehouse_','tree_obstacle_')) and not name.endswith('_roof'):
                if model.find('link/collision') is None or model.find('link/visual') is None:
                    errors.append(name+': missing collision or visual')
    else:
        road=by_name.get('service_road')
        if road is None:errors.append('missing metric service road')
        elif road.find('link/collision') is not None:errors.append('road decal must not alter collision map')
    obstacles=[model for model in models if model.get("name","").startswith("obstacle_") and not model.get("name","").endswith("_roof")]
    if any(model.find("link/collision") is None or model.find("link/visual") is None for model in obstacles):errors.append("safety obstacle missing collision or visual")
    return {"path":path,"pass":not errors,"errors":errors,"bytes":size,"model_count":len(models),"collision_count":len(collisions),"visual_count":len(visuals),"sdf_version":root.get("version")}


def audit_world_directory(directory):
    paths=sorted(os.path.join(directory,name) for name in os.listdir(directory) if name.endswith(".world"));reports=[audit_world(path) for path in paths]
    return {"pass":bool(reports) and all(report["pass"] for report in reports),"world_count":len(reports),"worlds":reports}
