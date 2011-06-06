#!/usr/bin/env python

import roslib; roslib.load_manifest('hbba_test')
import rospy
from hbba_msgs.msg import *
from hbba_msgs.srv import *

rospy.init_node('iw_solver_test')
rospy.wait_for_service('add_strategy')
add_strat = rospy.ServiceProxy('add_strategy', AddStrategy)
set_max = rospy.ServiceProxy('set_resource_max', SetResourceMax)

import face_emotion
#import sound_play_fr
import gesture

rospy.wait_for_service('add_strategy')
add_strat = rospy.ServiceProxy('add_strategy', AddStrategy)

s1 = Strategy()
s1.id = "meet_person"
s1.bringup_function = "strat1_bup"
s1.bringdown_function = "strat1_bdn"
s1.cost = [ResourceUsage("CPU", 20)]
s1.utility = ResourceUsage("Meet_Person", 3)
s1.source = """
// These two methods will be available to all strategies.
function activate(name)
{
    setDivisorRate(name, 1);
}

function deactivate(name)
{
    setDivisorRate(name, 0);
}

function strat1_bup(params)
{
	se_log("meet_person bup");
	se_log("emotion: " + params.emotion)
	
	pub_emotion(params.emotion)
	activate("/leg_detection/laser_filter")
    activate("/engage_interaction/pose_filter")
}

function strat1_bdn(params)
{
	se_log("meet_person bdn");
	
	deactivate("/leg_detection/laser_filter")
    deactivate("/engage_interaction/pose_filter")
}
"""
add_strat(s1)

s2 = Strategy()
s2.id = "attract_person"
s2.bringup_function = "strat2_bup"
s2.bringdown_function = "strat2_bdn"
s2.cost = [ResourceUsage("CPU", 20)]
s2.utility = ResourceUsage("Attract_Person", 3)
s2.source = """
function strat2_bup(params)
{
	se_log("attract_person bup");
	se_log("emotion: " + params.emotion)
	
	//pub_emotion(params.emotion)
	
    //activate("/attract/voice_filter");
    //activate("/attract/do_gesture_filter");
	activate("/attract/pose_filter");
}

function strat2_bdn(params)
{
	se_log("attract_person bdn");
	
	deactivate("/attract/pose_filter");
    //deactivate("/attract/voice_filter");
    //deactivate("/attract/do_gesture_filter");
}
"""
add_strat(s2)

s3 = Strategy()
s3.id = "presentation"
s3.bringup_function = "strat3_bup"
s3.bringdown_function = "strat3_bdn"
s3.cost = [ResourceUsage("CPU", 10)]
s3.utility = ResourceUsage("Presentation", 3)
s3.source = """
function strat3_bup(params)
{
	se_log("presentation bup");
	
    activate("/point_at/pose_filter")
	pub_look_at('/point_at/pose_to_point','/odom',4.0,0.0,0.0)
}

function strat3_bdn(params)
{
	se_log("presentation bdn");
	
    deactivate("/point_at/pose_filter")
}
"""
add_strat(s3)

s4 = Strategy()
s4.id = "keep_engaged"
s4.bringup_function = "strat4_bup"
s4.bringdown_function = "strat4_bdn"
s4.cost = [ResourceUsage("CPU", 10)]
s4.utility = ResourceUsage("Keep_Engaged", 3)
s4.source = """
function strat4_bup(params)
{
	se_log("keep_engaged bup");
	//se_log("data: " + params.data);
	
	activate("/keep_interacting/pose_filter")
}

function strat4_bdn(params)
{
	se_log("keep_engaged bdn");
	
	deactivate("/keep_interacting/pose_filter")
}
"""
add_strat(s4)

s4b = Strategy()
s4b.id = "convince"
s4b.bringup_function = "strat4b_bup"
s4b.bringdown_function = "strat4b_bdn"
s4b.cost = [ResourceUsage("CPU", 20)]
s4b.utility = ResourceUsage("Convince", 3)
s4b.source = """
function strat4b_bup(params)
{
	se_log("convince bup");
	//se_log("data: " + params.data);

	//pub_voice_fr(params.speak)
}

function strat4b_bdn(params)
{
	se_log("convince bdn");

}
"""
add_strat(s4b)

s5 = Strategy()
s5.id = "check_person"
s5.bringup_function = "strat5_bup"
s5.bringdown_function = "strat5_bdn"
s5.cost = [ResourceUsage("CPU", 10)]
s5.utility = ResourceUsage("Check_Person", 3)
s5.source = """
function strat5_bup(params)
{
	se_log("check_person bup");
	//se_log("data: " + params.data);
	
	activate("/check_person/pose_filter")
	pub_look_at('/check_person/pose_to_check','/odom',4.0,0.0,0.0)
	
	//activate("/check_person/do_gesture_filter")
	//pub_gesture(params.gesture)
}

function strat5_bdn(params)
{
	se_log("check_person bdn");	
	
	deactivate("/check_person/pose_filter")
	//deactivate("/check_person/do_gesture_filter")
}
"""
add_strat(s5)

s6 = Strategy()
s6.id = "disengaged"
s6.bringup_function = "strat6_bup"
s6.bringdown_function = "strat6_bdn"
s6.cost = [ResourceUsage("CPU", 20)]
s6.utility = ResourceUsage("Disengage", 3)
s6.source = """
function strat6_bup(params)
{
	se_log("disengage bup");
	se_log("emotion: " + params.emotion);
	
	pub_emotion(params.emotion)
	pubString('voice', params.speak)
	
	activate("/disengage_interaction/pose_filter")
	pub_look_at('/disengage_interaction/pose_to_disengage','/odom',0.0,0.0,0.0)
	
	//activate("/disengage_interaction/do_gesture_filter")
	//nav_goto('/odom',0,0,0)
}

function strat6_bdn(params)
{
	se_log("disengage bdn");	
	
	//deactivate("/disengage_interaction/voice_filter")
	deactivate("/disengage_interaction/pose_filter")
	//deactivate("/disengage_interaction/do_gesture_filter")
}
"""
add_strat(s6)

##Resource Max
set_max("CPU", 21)

