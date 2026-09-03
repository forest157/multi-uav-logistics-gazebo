"""Gazebo Classic 11 SDF world generation."""
import os
import re
import math
from xml.etree import ElementTree

from .scenes import SCALE, SCENES, metric_xy

HEADER = '''<?xml version="1.0"?>
<sdf version="1.6"><world name="{name}">
  <physics name="default_physics" type="ode"><max_step_size>0.004</max_step_size><real_time_factor>1</real_time_factor><real_time_update_rate>250</real_time_update_rate><magnetic_field>6.0e-6 2.3e-5 -4.2e-5</magnetic_field></physics>
  <gravity>0 0 -9.8066</gravity>
  <include><uri>model://sun</uri></include>
  <include><uri>model://ground_plane</uri></include>
  <spherical_coordinates><surface_model>EARTH_WGS84</surface_model><latitude_deg>47.397742</latitude_deg><longitude_deg>8.545594</longitude_deg><elevation>488</elevation><heading_deg>0</heading_deg></spherical_coordinates>
'''
FOOTER = '</world></sdf>\n'


def _safe_name(value):
    return re.sub(r"[^a-zA-Z0-9_]", "_", value)


def _model(name, x, y, z, geometry, colour="0.48 0.52 0.58 1"):
    return '''<model name="{name}"><static>true</static><pose>{x:.3f} {y:.3f} {z:.3f} 0 0 0</pose><link name="link"><collision name="collision"><geometry>{geometry}</geometry></collision><visual name="visual"><geometry>{geometry}</geometry><material><ambient>{colour}</ambient><diffuse>{colour}</diffuse></material></visual></link></model>\n'''.format(
        name=_safe_name(name), x=x, y=y, z=z, geometry=geometry, colour=colour)


def _visual_model(name,x,y,z,roll,pitch,yaw,geometry,colour):
    return '''<model name="{name}"><static>true</static><pose>{x:.3f} {y:.3f} {z:.3f} {roll:.3f} {pitch:.3f} {yaw:.3f}</pose><link name="link"><visual name="visual"><cast_shadows>false</cast_shadows><geometry>{geometry}</geometry><material><ambient>{colour}</ambient><diffuse>{colour}</diffuse></material></visual></link></model>\n'''.format(name=_safe_name(name),x=x,y=y,z=z,roll=roll,pitch=pitch,yaw=yaw,geometry=geometry,colour=colour)


def _box(name, x, y, w, d, height):
    cx, cy = metric_xy((x + w / 2.0, y + d / 2.0))
    width,depth=w*SCALE,d*SCALE
    body=_model(name,cx,cy,height/2.0,"<box><size>{:.3f} {:.3f} {:.3f}</size></box>".format(width,depth,height),"0.43 0.47 0.52 1")
    roof=_visual_model(name+"_roof",cx,cy,height+.18,0,0,0,"<box><size>{:.3f} {:.3f} 0.36</size></box>".format(width+.35,depth+.35),"0.20 0.23 0.27 1")
    return body+roof


def _cylinder(name, x, y, radius, height):
    cx, cy = metric_xy((x, y))
    return _model(name, cx, cy, height / 2.0,
                  "<cylinder><radius>{:.3f}</radius><length>{:.3f}</length></cylinder>".format(radius*SCALE, height))


def _marker(name, point, colour):
    x, y = metric_xy(point)
    return '<model name="{}"><static>true</static><pose>{:.3f} {:.3f} -0.009 0 0 0</pose><link name="link"><visual name="visual"><cast_shadows>false</cast_shadows><geometry><cylinder><radius>1.5</radius><length>0.02</length></cylinder></geometry><material><ambient>{}</ambient><diffuse>{}</diffuse></material></visual></link></model>\n'.format(_safe_name(name), x, y, colour, colour)


def _route_context(scene):
    start=metric_xy(scene["start"]);goal=metric_xy(scene["goal"]);dx,dy=goal[0]-start[0],goal[1]-start[1]
    length=math.hypot(dx,dy);yaw=math.atan2(dy,dx);cx=(start[0]+goal[0])/2.;cy=(start[1]+goal[1])/2.
    chunks=[_visual_model("service_road",cx,cy,.012,0,0,yaw,"<box><size>{:.3f} 7.0 0.02</size></box>".format(length+8.),"0.12 0.13 0.14 1")]
    for index in range(13):
        ratio=index/12.;x=start[0]+dx*ratio;y=start[1]+dy*ratio
        chunks.append(_visual_model("road_mark_{:02d}".format(index),x,y,.026,0,0,yaw,"<box><size>2.4 0.12 0.012</size></box>","0.92 0.84 0.18 1"))
    return "".join(chunks)


def render_world(scene_id):
    scene = SCENES[scene_id]
    chunks = [HEADER.format(name="logistics_{}_{}".format(scene_id, scene["name"])),_route_context(scene)]
    for index, obstacle in enumerate(scene["obstacles"]):
        prefix = "obstacle_{:02d}_{}".format(index, obstacle["label"])
        if obstacle["kind"] == "box":
            chunks.append(_box(prefix, obstacle["x"], obstacle["y"], obstacle["w"], obstacle["d"], obstacle["height"]))
        elif obstacle["kind"] == "cylinder":
            chunks.append(_cylinder(prefix, obstacle["x"], obstacle["y"], obstacle["radius"], obstacle["height"]))
        else:
            for part, (x, y, w, d) in enumerate(obstacle["rects"]):
                chunks.append(_box("{}_part_{}".format(prefix, part), x, y, w, d, obstacle["height"]))
    chunks.extend((_marker("start_zone", scene["start"], "0.1 0.8 0.1 1"),
                   _marker("goal_zone", scene["goal"], "0.9 0.15 0.1 1"), FOOTER))
    result = "".join(chunks)
    ElementTree.fromstring(result)
    return result


def write_worlds(output_dir):
    os.makedirs(output_dir, exist_ok=True)
    paths = []
    for scene_id in sorted(SCENES):
        path = os.path.join(output_dir, "scene_{}.world".format(scene_id))
        with open(path, "w", encoding="utf-8") as stream:
            stream.write(render_world(scene_id))
        paths.append(path)
    return paths
