"""Shared fleet-size and spawn geometry validation."""
import math

MIN_FLEET_SIZE=1
MAX_FLEET_SIZE=8


def validate_vehicle_count(value, maximum=MAX_FLEET_SIZE):
    try:count=int(value)
    except (TypeError,ValueError):raise ValueError("vehicle_count must be an integer")
    if count<MIN_FLEET_SIZE or count>int(maximum):
        raise ValueError("vehicle_count must be between {} and {}".format(
            MIN_FLEET_SIZE,int(maximum)))
    return count


def line_spawn_offsets(count, spacing=3.0):
    count=validate_vehicle_count(count);spacing=float(spacing)
    if not math.isfinite(spacing) or spacing<=0.0:
        raise ValueError("spawn spacing must be positive")
    middle=(count-1)*0.5
    return [((index-middle)*spacing,0.0) for index in range(count)]


def indexed_palette(count):
    count=validate_vehicle_count(count)
    result=[]
    for index in range(count):
        hue=float(index)/max(1,count)
        # Bright HSV-to-RGB palette without an extra dependency.
        sector=int(hue*6.0)%6;fraction=hue*6.0-int(hue*6.0)
        p,q,t=0.25,1.0-fraction*0.75,0.25+fraction*0.75
        result.append(((1,t,p),(q,1,p),(p,1,t),(p,q,1),(t,p,1),(1,p,q))[sector])
    return result
