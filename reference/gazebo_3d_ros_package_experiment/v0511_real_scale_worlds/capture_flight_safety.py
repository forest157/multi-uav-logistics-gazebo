#!/usr/bin/env python3
"""Read-only diagnostic observer. Gazebo truth is recorded for evaluation only."""
import json
import threading
import rospy
from mavros_msgs.msg import State
from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import PoseStamped, TwistStamped
from std_msgs.msg import String


class Capture:
    def __init__(self, output):
        self.output=output;self.lock=threading.RLock()
        self.poses={};self.truth={};self.state={};self.status={}
        self.targets={};self.velocities={};self.flight_states={};self.latest_tracks={};self.truth_stamp=None
        for i in range(3):
            rospy.Subscriber('/uav%d/mavros/local_position/pose'%i,PoseStamped,self.pose,i,queue_size=1)
            rospy.Subscriber('/uav%d/mavros/setpoint_position/local'%i,PoseStamped,self.target,i,queue_size=1)
            rospy.Subscriber('/uav%d/mavros/local_position/velocity_local'%i,TwistStamped,self.velocity,i,queue_size=1)
            rospy.Subscriber('/uav%d/mavros/state'%i,State,self.flight_state,i,queue_size=1)
        rospy.Subscriber('/gazebo/model_states',ModelStates,self.models,queue_size=1)
        rospy.Subscriber('/fleet/mission_state',String,self.mission,queue_size=1)
        rospy.Subscriber('/perception/status',String,self.perception,queue_size=1)
        rospy.Subscriber('/perception/dynamic_tracks',String,self.tracks,queue_size=5)
        self.timer=rospy.Timer(rospy.Duration(0.1),self.snapshot)

    def target(self,msg,i):
        p=msg.pose.position
        with self.lock:self.targets[i]=[msg.header.stamp.to_sec(),p.x,p.y,p.z]

    def velocity(self,msg,i):
        v=msg.twist.linear
        with self.lock:self.velocities[i]=[msg.header.stamp.to_sec(),v.x,v.y,v.z]

    def flight_state(self,msg,i):
        with self.lock:self.flight_states[i]=dict(stamp=msg.header.stamp.to_sec(),connected=msg.connected,armed=msg.armed,mode=msg.mode)

    def pose(self,msg,i):
        p=msg.pose.position
        with self.lock:self.poses[i]=[msg.header.stamp.to_sec(),p.x,p.y,p.z]

    def models(self,msg):
        with self.lock:
            self.truth_stamp=rospy.get_time()
            self.truth={name:dict(position=[p.position.x,p.position.y,p.position.z],velocity=[v.linear.x,v.linear.y,v.linear.z]) for name,p,v in zip(msg.name,msg.pose,msg.twist) if 'iris' in name or 'uav' in name or 'bird' in name}

    def mission(self,msg):
        with self.lock:self.state=json.loads(msg.data)

    def perception(self,msg):
        with self.lock:self.status=json.loads(msg.data)

    def tracks(self,msg):
        with self.lock:self.latest_tracks=json.loads(msg.data)

    def snapshot(self,event):
        # Independent timer preserves evidence during target loss and before detection.
        # ModelStates has no header: receipt time is not an exact physics timestamp.
        with self.lock:
            self.output.write(json.dumps(dict(stamp=rospy.get_time(),tracks=self.latest_tracks,poses=self.poses,targets=self.targets,velocities=self.velocities,flight_states=self.flight_states,truth_receipt_stamp=self.truth_stamp,truth_evaluation_only=self.truth,mission=self.state,perception=self.status))+'\n')
            self.output.flush()


if __name__=='__main__':
    import sys
    rospy.init_node('flight_safety_capture')
    with open(sys.argv[1],'w') as output:
        capture=Capture(output)
        try:rospy.spin()
        finally:
            capture.timer.shutdown()
            capture.timer.join(timeout=2.0)
