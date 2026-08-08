"""Metric 3D descriptions of the seven reference scenarios.

Reference coordinates span 0..500.  A 0.2 scale produces a 100 m square site;
the site centre becomes Gazebo's ENU origin.  Heights are physical metres and
are intentionally not derived from footprint units.
"""

SCALE = 0.2
ORIGIN = 250.0


def rect(x, y, w, d, label, height):
    return {"kind": "box", "label": label, "x": x, "y": y,
            "w": w, "d": d, "height": height}


def circle(x, y, radius, label, height):
    return {"kind": "cylinder", "label": label, "x": x, "y": y,
            "radius": radius, "height": height}


def lshape(x, y, w, d, cut_w, cut_d, label, height):
    # Same footprint convention as the reference planner: lower-left cut-out.
    return {"kind": "parts", "label": label, "height": height, "rects": [
        (x + cut_w, y, w - cut_w, d), (x, y + cut_d, cut_w, d - cut_d)]}


def tshape(x, y, stem_w, stem_h, top_w, top_h, label, height):
    # Reference x is the symmetry axis and y is the bottom of the stem.
    return {"kind": "parts", "label": label, "height": height, "rects": [
        (x - stem_w / 2.0, y, stem_w, stem_h),
        (x - top_w / 2.0, y + stem_h, top_w, top_h)]}


SCENES = {
    0: {"name": "urban_delivery", "start": (50, 50), "goal": (475, 475), "obstacles": [
        tshape(130, 100, 25, 60, 80, 30, "shopping_centre", 14),
        lshape(320, 80, 70, 130, 40, 60, "office", 22)]},
    1: {"name": "high_rise_delivery", "start": (50, 50), "goal": (450, 450), "obstacles": [
        rect(75, 400, 350, 100, "tower_a", 32), rect(75, 275, 350, 75, "tower_b", 28),
        rect(75, 150, 350, 75, "tower_c", 24), rect(75, 5, 350, 100, "podium", 8)]},
    2: {"name": "complex_crossing", "start": (125, 425), "goal": (225, 175), "obstacles": [
        rect(0, 350, 75, 150, "tower_1", 30), rect(425, 350, 75, 150, "tower_2", 30),
        rect(225, 400, 75, 100, "platform", 8), rect(225, 275, 75, 75, "plant_room", 12),
        rect(75, 225, 350, 50, "barrier", 6), rect(125, 50, 50, 175, "wall_a", 10),
        rect(325, 125, 50, 100, "wall_b", 10), rect(75, 0, 350, 50, "base", 5)]},
    3: {"name": "dense_facilities", "start": (50, 50), "goal": (450, 450), "obstacles": [
        rect(100,100,50,50,"facility_1",9), rect(300,300,150,50,"facility_2",12),
        rect(300,50,100,40,"facility_3",8), rect(50,350,100,50,"facility_4",10),
        rect(150,200,25,75,"facility_5",7), rect(200,150,50,100,"facility_6",11),
        rect(250,250,50,50,"facility_7",9), rect(350,100,50,125,"facility_8",14),
        rect(400,200,60,50,"facility_9",8), rect(100,400,100,40,"facility_10",9),
        rect(225,350,75,75,"facility_11",12)]},
    4: {"name": "urban_park_emergency", "start": (25, 25), "goal": (475, 475), "obstacles": [
        tshape(140,90,25,50,70,25,"library",16), lshape(380,330,80,100,40,50,"gallery",14),
        circle(275,100,40,"flower_bed",1.0), circle(200,325,60,"fountain",1.5),
        rect(350,250,80,60,"exhibition_hall",12), circle(150,225,40,"sculpture",6),
        rect(50,300,60,150,"office",20), circle(400,150,75,"lake",0.3)]},
    5: {"name": "industrial_transport", "start": (50, 250), "goal": (450, 250), "obstacles": [
        circle(150,250,75,"tank_a",16), circle(150,120,25,"tank_b",10),
        lshape(350,30,70,90,30,50,"factory",13), circle(375,425,50,"cooling_tower",24),
        rect(230,60,80,70,"warehouse",10), circle(220,400,55,"reactor",20),
        rect(50,400,100,75,"control_room",9), circle(370,300,35,"lightning_mast",30),
        tshape(280,180,18,45,60,25,"office",12), circle(100,50,50,"waste_pool",0.5)]},
    6: {"name": "mountain_medical", "start": (50, 50), "goal": (450, 350), "obstacles": [
        circle(115,160,75,"peak_a",32), circle(350,300,50,"peak_b",24),
        circle(250,400,60,"peak_c",28), rect(100,300,75,40,"gorge_bridge",8),
        rect(300,100,100,40,"cliff_road",7), circle(400,425,50,"signal_tower",35),
        rect(200,200,75,75,"forest",18)]},
}


def metric_xy(point):
    return ((point[0] - ORIGIN) * SCALE, (point[1] - ORIGIN) * SCALE)
