import importlib.machinery
import os
import unittest
from unittest.mock import Mock, patch
import rospy
from geometry_msgs.msg import Pose, PoseStamped
from mavros_msgs.msg import PositionTarget
from logistics_gazebo_sim.flight_setpoints import make_setpoint, valid_setpoint, POSITION_MASK, VELOCITY_MASK

SCRIPT=os.path.join(os.path.dirname(__file__),"..","scripts","offboard_controller")
offboard=importlib.machinery.SourceFileLoader("setpoint_test_offboard",SCRIPT).load_module()


class FlightSetpointTest(unittest.TestCase):
    def test_velocity_is_unscaled_enu_and_position_is_ignored(self):
        m=make_setpoint((4,5,8),rospy.Time.from_sec(10.),(1.2,-.4,.2))
        self.assertEqual((m.velocity.x,m.velocity.y,m.velocity.z),(1.2,-.4,.2))
        self.assertEqual(m.type_mask,VELOCITY_MASK)
        self.assertEqual(m.coordinate_frame,PositionTarget.FRAME_LOCAL_NED)
        self.assertTrue(valid_setpoint(m,10.1))
        self.assertEqual(make_setpoint((4,5,8),m.header.stamp).type_mask,POSITION_MASK)

    def test_expired_future_nonfinite_and_excessive_velocity_are_rejected(self):
        m=make_setpoint((0,0,8),rospy.Time.from_sec(10.),(1,0,0))
        for t in (9.9,10.26,float("nan")):self.assertFalse(valid_setpoint(m,t))
        for v in ((3,0,0),(0,0,.9),(float("nan"),0,0)):
            self.assertFalse(valid_setpoint(make_setpoint((0,0,8),m.header.stamp,v),10.))
        m.coordinate_frame=PositionTarget.FRAME_BODY_NED
        self.assertFalse(valid_setpoint(m,10.))

    def controller(self):
        c=offboard.Controller.__new__(offboard.Controller)
        c.raw_seen=True;c.raw_target=make_setpoint((99,99,8),rospy.Time.from_sec(10.),(1,0,0))
        c.expired_hold=None;c.local_pose=Pose();c.local_pose.position.x=3;c.local_pose.orientation.w=1
        c.target=PoseStamped();c.target.pose.position.x=99
        c.mission_complete=False;c.publisher=Mock();c.raw_publisher=Mock()
        return c

    def test_fresh_raw_command_publishes_only_velocity_interface(self):
        c=self.controller()
        with patch.object(offboard.rospy.Time,"now",return_value=rospy.Time.from_sec(10.1)):c._publish()
        c.publisher.publish.assert_not_called();c.raw_publisher.publish.assert_called_once()
        self.assertEqual(c.raw_target.header.stamp.to_sec(),10.)

    def test_timeout_latches_current_position_without_resuming_old_target(self):
        c=self.controller()
        with patch.object(offboard.rospy.Time,"now",return_value=rospy.Time.from_sec(10.3)):
            c._publish();c.local_pose.position.x=4;c._publish()
        c.raw_publisher.publish.assert_not_called()
        self.assertEqual(c.publisher.publish.call_args.args[0].pose.position.x,3)

    def test_completion_disallows_velocity_and_invalid_input_fails_closed(self):
        c=self.controller();c.mission_complete=True
        with patch.object(offboard.rospy.Time,"now",return_value=rospy.Time.from_sec(10.1)):c._publish()
        c.raw_publisher.publish.assert_not_called()
        m=make_setpoint((0,0,8),rospy.Time.from_sec(10.),(float("nan"),0,0))
        with patch.object(offboard.rospy,"get_time",return_value=10.1):c._raw_cb(m)
        self.assertIsNone(c.raw_target)
