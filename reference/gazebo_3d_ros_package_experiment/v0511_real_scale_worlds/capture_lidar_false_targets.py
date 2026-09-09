#!/usr/bin/env python3
"""Read-only diagnostic observer. Gazebo truth is recorded for evaluation only."""
import json
import threading
import rospy
from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String


class Capture:
    def __init__(self, output):
        self.output=output;self.lock=threading.RLock()
        self.poses={};self.truth={};self.state={};self.status={}
        for i in range(3):
            rospy.Subscriber('/uav%d/mavros/local_position/pose'%i,PoseStamped,self.pose,i,queue_size=1)
        rospy.Subscriber('/gazebo/model_states',ModelStates,self.models,queue_size=1)
        rospy.Subscriber('/fleet/mission_state',String,self.mission,queue_size=1)
        rospy.Subscriber('/perception/status',String,self.perception,queue_size=1)
        rospy.Subscriber('/perception/dynamic_tracks',String,self.tracks,queue_size=5)

    def pose(self,msg,i):
        p=msg.pose.position
        with self.lock:self.poses[i]=[msg.header.stamp.to_sec(),p.x,p.y,p.z]

    def models(self,msg):
        with self.lock:
            self.truth={name:[p.position.x,p.position.y,p.position.z] for name,p in zip(msg.name,msg.pose) if 'iris' in name or 'uav' in name}

    def mission(self,msg):
        with self.lock:self.state=json.loads(msg.data)

    def perception(self,msg):
        with self.lock:self.status=json.loads(msg.data)

    def tracks(self,msg):
        d=json.loads(msg.data)
        if not d.get('obstacles'):return
        with self.lock:
            self.output.write(json.dumps(dict(tracks=d,poses=self.poses,truth_evaluation_only=self.truth,mission=self.state,perception=self.status))+'\n')
            self.output.flush()


if __name__=='__main__':
    import sys
    rospy.init_node('lidar_diagnostic_capture')
    with open(sys.argv[1],'w') as output:
        Capture(output);rospy.spin()
