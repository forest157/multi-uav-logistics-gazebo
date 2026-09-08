"""Static audit for reproducible, real-scale Gazebo Classic worlds."""
import os
from xml.etree import ElementTree


def audit_world(path,max_models=100,max_bytes=250000):
    size=os.path.getsize(path);root=ElementTree.parse(path).getroot();world=root.find("world")
    if world is None:return {"path":path,"pass":False,"errors":["missing world"]}
    models=world.findall("model");collisions=world.findall("model/link/collision");visuals=world.findall("model/link/visual")
    names=[model.get("name","") for model in models];errors=[]
    if root.get("version")!="1.6":errors.append("SDF must remain Gazebo Classic 1.6")
    if len(names)!=len(set(names)):errors.append("model names must be unique")
    if len(models)>max_models:errors.append("model budget exceeded")
    if size>max_bytes:errors.append("world file budget exceeded")
    road=next((model for model in models if model.get("name")=="service_road"),None)
    if road is None:errors.append("missing metric service road")
    elif road.find("link/collision") is not None:errors.append("road decal must not alter collision map")
    obstacles=[model for model in models if model.get("name","").startswith("obstacle_") and not model.get("name","").endswith("_roof")]
    if any(model.find("link/collision") is None or model.find("link/visual") is None for model in obstacles):errors.append("safety obstacle missing collision or visual")
    return {"path":path,"pass":not errors,"errors":errors,"bytes":size,"model_count":len(models),"collision_count":len(collisions),"visual_count":len(visuals),"sdf_version":root.get("version")}


def audit_world_directory(directory):
    paths=sorted(os.path.join(directory,name) for name in os.listdir(directory) if name.endswith(".world"));reports=[audit_world(path) for path in paths]
    return {"pass":bool(reports) and all(report["pass"] for report in reports),"world_count":len(reports),"worlds":reports}
