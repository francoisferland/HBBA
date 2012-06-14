#!/usr/bin/env python

import roslib; roslib.load_manifest('hbba_test')
import rospy
from topic_filters_manager.srv import *

rospy.init_node('iw_solver_test')

rospy.wait_for_service('register_filter')
register_filter = rospy.ServiceProxy('register_filter', RegisterFilter)

# From percepts
register_filter("/leg_detection/laser_filter", "GenericDivider")

# From hri_behaviors 
register_filter("/attract/pose_filter", "GenericDivider")
register_filter("/attract/cmd_vel_filter", "GenericDivider")
register_filter("/attract/do_gesture_filter", "GenericDivider")
register_filter("/attract/emotion_choice_filter", "GenericDivider")
register_filter("/attract/voice_filter", "GenericDivider")
register_filter("/check_person/pose_filter", "GenericDivider")
register_filter("/check_person/cmd_vel_filter", "GenericDivider")
register_filter("/check_person/do_gesture_filter", "GenericDivider")
register_filter("/disengage_interaction/pose_filter", "GenericDivider")
register_filter("/disengage_interaction/pan_angle_filter", "GenericDivider")
register_filter("/disengage_interaction/cmd_vel_filter", "GenericDivider")
register_filter("/disengage_interaction/do_gesture_filter", "GenericDivider")
register_filter("/disengage_interaction/voice_filter", "GenericDivider")
register_filter("/engage_interaction/pose_filter", "GenericDivider")
register_filter("/engage_interaction/do_gesture_filter", "GenericDivider")
register_filter("/engage_interaction/voice_filter", "GenericDivider")
register_filter("/keep_interacting/pose_filter", "GenericDivider")
register_filter("/keep_interacting/pan_angle_filter", "GenericDivider")
register_filter("/keep_interacting/tilt_angle_filter", "GenericDivider")
register_filter("/keep_interacting/cmd_vel_filter", "GenericDivider")
register_filter("/keep_interacting/do_gesture_filter", "GenericDivider")
register_filter("/keep_interacting/voice_filter", "GenericDivider")
register_filter("/point_at/pose_filter", "GenericDivider")
register_filter("/point_at/pan_angle_filter", "GenericDivider")
register_filter("/point_at/cmd_vel_filter", "GenericDivider")
register_filter("/point_at/do_gesture_filter", "GenericDivider")
register_filter("/point_at/voice_filter", "GenericDivider")

r = rospy.Rate(1) # 1 Hz
