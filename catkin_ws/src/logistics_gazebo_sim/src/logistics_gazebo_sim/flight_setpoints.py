"""Explicit ENU position/velocity setpoints with bounded transport lifetime."""
import math
from mavros_msgs.msg import PositionTarget

VELOCITY_MASK=(PositionTarget.IGNORE_PX|PositionTarget.IGNORE_PY|PositionTarget.IGNORE_PZ|
               PositionTarget.IGNORE_AFX|PositionTarget.IGNORE_AFY|PositionTarget.IGNORE_AFZ|
               PositionTarget.IGNORE_YAW_RATE)
POSITION_MASK=(PositionTarget.IGNORE_VX|PositionTarget.IGNORE_VY|PositionTarget.IGNORE_VZ|
               PositionTarget.IGNORE_AFX|PositionTarget.IGNORE_AFY|PositionTarget.IGNORE_AFZ|
               PositionTarget.IGNORE_YAW_RATE)


def make_setpoint(position, stamp, velocity=None):
    msg=PositionTarget();msg.header.stamp=stamp;msg.header.frame_id="map"
    # MAVROS transforms ROS ENU data to MAVLink LOCAL_NED, not body coordinates.
    msg.coordinate_frame=PositionTarget.FRAME_LOCAL_NED
    msg.position.x,msg.position.y,msg.position.z=position
    msg.type_mask=POSITION_MASK if velocity is None else VELOCITY_MASK
    if velocity is not None:msg.velocity.x,msg.velocity.y,msg.velocity.z=velocity
    msg.yaw=0.
    return msg


def valid_setpoint(msg, now, timeout=.25):
    if msg.coordinate_frame!=PositionTarget.FRAME_LOCAL_NED or msg.type_mask not in (POSITION_MASK,VELOCITY_MASK):
        return False
    age=float(now)-msg.header.stamp.to_sec()
    p=msg.position;v=msg.velocity
    if not math.isfinite(age) or not 0.<=age<=timeout:return False
    if not all(math.isfinite(x) for x in (p.x,p.y,p.z,v.x,v.y,v.z,msg.yaw)):return False
    if msg.type_mask==VELOCITY_MASK:
        if math.sqrt(v.x*v.x+v.y*v.y+v.z*v.z)>2.0001 or abs(v.z)>.8001:return False
    return True
