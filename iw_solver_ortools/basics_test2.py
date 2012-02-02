#!/usr/bin/env python

import roslib; roslib.load_manifest('iw_solver_ortools')
import rospy
from hbba_msgs.msg import *
from hbba_msgs.srv import *

rospy.init_node('iw_solver_test')
rospy.wait_for_service('add_strategy')
add_strat = rospy.ServiceProxy('add_strategy', AddStrategy)
set_max = rospy.ServiceProxy('set_resource_max', SetResourceMax)
pub_desires = rospy.Publisher('desires_set', DesiresSet)

##Detection Strategies 
s1 = Strategy() 
s1.id = "kinect detect person"
s1.bringup_function = "strat1_bup"
s1.bringdown_function = "strat1_bdn"
s1.cost = [ResourceUsage("CPU", 20)]
s1.utility = ResourceUsage("front detection", 3)
s1.source = """
function strat1_bup(params)
{
	se_log("strat1_bup");
	se_log("data: " + params.data);
}

function strat1_bdn(params)
{
	se_log("strat1_bdn");
}
"""
add_strat(s1)

s2 = Strategy() 
s2.id = "kinect with seg image detect person"
s2.bringup_function = "strat2_bup"
s2.bringdown_function = "strat2_bdn"
s2.cost = [ResourceUsage("CPU", 30)]
s2.utility = ResourceUsage("front detection", 4)
s2.source = """
function strat2_bup(params)
{
	se_log("strat2_bup");
	se_log("data: " + params.data);
}

function strat2_bdn(params)
{
	se_log("strat2_bdn");
}
"""
add_strat(s2)

s3 = Strategy() 
s3.id = "laser detect front"
s3.bringup_function = "strat3_bup"
s3.bringdown_function = "strat3_bdn"
s3.cost = [ResourceUsage("CPU", 10)]
s3.utility = ResourceUsage("front detection", 2)
s3.source = """
function strat3_bup(params)
{
	se_log("strat3_bup");
	se_log("data: " + params.data);
}

function strat3_bdn(params)
{
	se_log("strat3_bdn");
}
"""
add_strat(s3)

s4 = Strategy() 
s4.id = "kinect detect face"
s4.bringup_function = "strat4_bup"
s4.bringdown_function = "strat4_bdn"
s4.cost = [ResourceUsage("CPU", 25)]
s4.utility = ResourceUsage("face detection", 4)
s4.source = """
function strat4_bup(params)
{
	se_log("strat4_bup");
	se_log("data: " + params.data);
}

function strat4_bdn(params)
{
	se_log("strat4_bdn");
}
"""
add_strat(s4)



s5 = Strategy() 
s5.id = "kinect with seg image detect face"
s5.bringup_function = "strat5_bup"
s5.bringdown_function = "strat5_bdn"
s5.cost = [ResourceUsage("CPU", 35)]
s5.utility = ResourceUsage("face detection", 5)
s5.source = """
function strat5_bup(params)
{
	se_log("strat5_bup");
	se_log("data: " + params.data);
}

function strat5_bdn(params)
{
	se_log("strat5_bdn");
}
"""
add_strat(s5)

s6 = Strategy() 
s6.id = "webcam detect face"
s6.bringup_function = "strat6_bup"
s6.bringdown_function = "strat6_bdn"
s6.cost = [ResourceUsage("CPU", 15)]
s6.utility = ResourceUsage("face detection", 3)
s6.source = """
function strat6_bup(params)
{
	se_log("strat6_bup");
	se_log("data: " + params.data);
}

function strat6_bdn(params)
{
	se_log("strat6_bdn");
}
"""
add_strat(s6)



s7 = Strategy() 
s7.id = "laser detect large"
s7.bringup_function = "strat7_bup"
s7.bringdown_function = "strat7_bdn"
s7.cost = [ResourceUsage("CPU", 5)]
s7.utility = ResourceUsage("large detection", 5)
s7.utility_min = [ResourceUsage("face detection",7)]
s7.source = """
function strat7_bup(params)
{
	se_log("strat7_bup");
	se_log("data: " + params.data);
}

function strat7_bdn(params)
{
	se_log("strat7_bdn");
}
"""
add_strat(s7)

set_max("CPU", 100)

##Detection desires 
des1 = Desire("test_1", "front detection", 1, "{'data' : 1}")
des2 = Desire("test_2", "face detection", 1, "{'data' : 2}")
des3 = Desire("test_3", "large detection", 1, "{'data' : 3}")

desires = DesiresSet()
desires.desires = [des1, des2, des3]



r = rospy.Rate(1) # 1 Hz
while not rospy.is_shutdown():
    pub_desires.publish(desires)
    r.sleep()

