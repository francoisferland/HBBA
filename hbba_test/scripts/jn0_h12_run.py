#!/usr/bin/env python

import roslib; roslib.load_manifest('hbba_test')
import rospy
from topic_filters_manager.srv import *
from hbba_msgs.msg import *
from hbba_msgs.srv import *

rospy.init_node('jn0_h12_run')

# Topic filters

rospy.wait_for_service('/hbba/register_filter')
register_filter = rospy.ServiceProxy('/hbba/register_filter', RegisterFilter)

register_filter("/manyears/stream_filter", "GenericDivider")
register_filter("/ball_tracking/kinect_filter", "GenericDivider")

# Strategies

rospy.wait_for_service('/hbba/add_strategy')
add_strat = rospy.ServiceProxy('/hbba/add_strategy', AddStrategy)
set_res_max = rospy.ServiceProxy('/hbba/set_resource_max', SetResourceMax)

s1 = Strategy()
s1.id = "locate_voice"
s1.bringup_function = "locate_voice_bup"
s1.bringdown_function = "locate_voice_bdn"
s1.cost = [ResourceUsage("CPU", 40)]
s1.utility = ResourceUsage("LocateVoice", 1)
s1.source = """
// These two methods will be available to all strategies.
function activate(name)
{
    setDividerRate(name, 1);
}

function deactivate(name)
{
    setDividerRate(name, 0);
}

function locate_voice_bup(params)
{
	se_log('locate_voice bringup');
	activate('/manyears/stream_filter')
}

function locate_voice_bdn(params)
{
	se_log('locate_voice bringdown');
	deactivate('/manyears/stream_filter')
}
"""
add_strat(s1)

s2 = Strategy()
s2.id = "track_ball_full"
s2.bringup_function = "track_ball_bup"
s2.bringdown_function = "track_ball_bdn"
s2.cost = [ResourceUsage("CPU", 80), ResourceUsage("BallTracker", 1)]
s2.utility = ResourceUsage("BallTracking", 4)
s2.source = """
function track_ball_bup(params)
{
	se_log('track_ball /1 bringup');
	activate('/ball_tracking/kinect_filter');
}

function track_ball_bdn(params)
{
	se_log('track_ball /1 bringdown');
	deactivate('/ball_tracking/kinect_filter');
}
"""
add_strat(s2)

s3 = Strategy()
s3.id = "track_ball_quarter"
s3.bringup_function = "track_ball_quarter_bup"
s3.bringdown_function = "track_ball_quarter_bdn"
s3.cost = [ResourceUsage("CPU", 40), ResourceUsage("BallTracker", 1)]
s3.utility = ResourceUsage("BallTracking", 1)
s3.source = """
function track_ball_quarter_bup(params)
{
	se_log('track_ball /4 bringup');
    setDividerRate('/ball_tracking/kinect_filter', 20);
}

function track_ball_quarter_bdn(params)
{
	se_log('track_ball /4 bringdown');
    setDividerRate('/ball_tracking/kinect_filter', 0);
}
"""
add_strat(s3)

# Resource allocations

set_res_max("BallTracker", 1)
set_res_max("CPU", 100)

