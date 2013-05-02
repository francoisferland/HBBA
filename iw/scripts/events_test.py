#! /usr/bin/env python
#
# Test script for events_generator.
# Configured for jn0_hbba_cfg's irl1_test_episodic_memory.

import roslib; roslib.load_manifest("iw")
import rospy;
from hbba_msgs.msg import *
from hbba_msgs.srv import *

rospy.init_node("events_test", anonymous=True)

create_cem=rospy.ServiceProxy("hbba/create_exploitation_matcher", CreateExploitationMatcher)
create_cem('/jn0/control_cart/right_arm/goal_force', [ExploitationMatch(0, ['ArmSafety']), ExploitationMatch(10, ['ArmKeepDown'])])
create_cem('/jn0/control_cart/left_arm/goal_pose', [ExploitationMatch(0, ['ArmSafety'])])
create_cem('/jn0/control_cart/right_arm/goal_pose', [ExploitationMatch(0, ['ArmSafety'])])
create_cem('/jn0/look_at_pose', [ExploitationMatch(35, ['LookAround']), ExploitationMatch(100, ['Teleop'])])
create_cem('/az3/cmd_eta', [ExploitationMatch(100, ['Teleop'])])
create_cem('/jn0/control_cart/right_arm/impedance_parameters', [ExploitationMatch(0, ['ArmSafety']), ExploitationMatch(10, ['ArmKeepDown'])])
create_cem('/jn0/control_cart/left_arm/impedance_parameters', [ExploitationMatch(0, ['ArmSafety']), ExploitationMatch(10, ['ArmKeepDown'])])
create_cem('/az3/cmd_vel', [ExploitationMatch(35, ['LookAround']), ExploitationMatch(5, ['Wander']), ExploitationMatch(12, ['WallFollow']), ExploitationMatch(15, ['Avoid']), ExploitationMatch(20, ['Repulse']), ExploitationMatch(30, ['GoTo'])])
create_cem('/jn0/r_gripper_up_joint_controller/command', [ExploitationMatch(0, ['ArmSafety']), ExploitationMatch(10, ['ArmKeepDown'])])
create_cem('/jn0/control_cart/left_arm/goal_force', [ExploitationMatch(0, ['ArmSafety']), ExploitationMatch(10, ['ArmKeepDown'])])
create_cem('/jn0/l_gripper_up_joint_controller/command', [ExploitationMatch(0, ['ArmSafety']), ExploitationMatch(10, ['ArmKeepDown'])])

print "Ready."
rospy.spin()
