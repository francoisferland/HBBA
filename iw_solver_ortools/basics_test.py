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
s1a = Strategy()
s1a.id = "kinect detect person"
s1a.bringup_function = "strat1a_bup"
s1a.bringdown_function = "strat1a_bdn"
s1a.cost = [ResourceUsage("CPU", 20)]
s1a.utility = ResourceUsage("front detection", 3)
s1a.utility_min = [ResourceUsage("",0)]
s1a.source = """
function strat1a_bup(params)
{
	se_log("strat1a_bup");
	se_log("data: " + params.data);
}

function strat1a_bdn(params)
{
	se_log("strat1a_bdn");
}
"""
add_strat(s1a)

s1b = Strategy()
s1b.id = "kinect detect face"
s1b.bringup_function = "strat1b_bup"
s1b.bringdown_function = "strat1b_bdn"
s1b.cost = [ResourceUsage("CPU", 25)]
s1b.utility = ResourceUsage("face detection", 4)
s1b.utility_min = [ResourceUsage ("front detection",2)]
s1b.source = """
function strat1b_bup(params)
{
	se_log("strat1b_bup");
	se_log("data: " + params.data);
}

function strat1b_bdn(params)
{
	se_log("strat1b_bdn");
}
"""
add_strat(s1b)

s2a = Strategy()
s2a.id = "kinect with seg image detect person"
s2a.bringup_function = "strat2a_bup"
s2a.bringdown_function = "strat2a_bdn"
s2a.cost = [ResourceUsage("CPU", 30)]
s2a.utility = ResourceUsage("front detection", 4)
s2a.utility_min = [ResourceUsage("",0)]
s2a.source = """
function strat2a_bup(params)
{
	se_log("strat2a_bup");
	se_log("data: " + params.data);
}

function strat2a_bdn(params)
{
	se_log("strat2a_bdn");
}
"""
add_strat(s2a)

s2b = Strategy()
s2b.id = "kinect with seg image detect face"
s2b.bringup_function = "strat2b_bup"
s2b.bringdown_function = "strat2b_bdn"
s2b.cost = [ResourceUsage("CPU", 35)]
s2b.utility = ResourceUsage("face detection", 5)
s2b.utility_min = [ResourceUsage("",0)]
s2b.source = """
function strat2b_bup(params)
{
	se_log("strat2b_bup");
	se_log("data: " + params.data);
}

function strat2b_bdn(params)
{
	se_log("strat2b_bdn");
}
"""
add_strat(s2b)

s3 = Strategy()
s3.id = "webcam detect face"
s3.bringup_function = "strat3_bup"
s3.bringdown_function = "strat3_bdn"
s3.cost = [ResourceUsage("CPU", 15)]
s3.utility = ResourceUsage("face detection", 3)
s3.utility_min = [ResourceUsage("",0)]
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

s4a = Strategy()
s4a.id = "laser detect front"
s4a.bringup_function = "strat4a_bup"
s4a.bringdown_function = "strat4a_bdn"
s4a.cost = [ResourceUsage("CPU", 10)]
s4a.utility = ResourceUsage("front detection", 2)
s4a.utility_min = [ResourceUsage("",0)]
s4a.source = """
function strat4a_bup(params)
{
	se_log("strat4a_bup");
	se_log("data: " + params.data);
}

function strat4a_bdn(params)
{
	se_log("strat4a_bdn");
}
"""
add_strat(s4a)

s4b = Strategy()
s4b.id = "laser detect large"
s4b.bringup_function = "strat4b_bup"
s4b.bringdown_function = "strat4b_bdn"
s4b.cost = [ResourceUsage("CPU", 5)]
s4b.utility = ResourceUsage("large detection", 5)
s4b.utility_min = [ResourceUsage("",0)]
s4b.source = """
function strat4b_bup(params)
{
	se_log("strat4b_bup");
	se_log("data: " + params.data);
}

function strat4b_bdn(params)
{
	se_log("strat4b_bdn");
}
"""
add_strat(s4b)

##Follow Strategies 
s5 = Strategy()
s5.id = "kinect follow face"
s5.bringup_function = "strat5_bup"
s5.bringdown_function = "strat5_bdn"
s5.cost = [ResourceUsage("CPU", 15)]
s5.utility = ResourceUsage("face following", 4)
s5.utility_min = [ResourceUsage("",0)]
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
s6.id = "kinect with seg image follow face"
s6.bringup_function = "strat6_bup"
s6.bringdown_function = "strat6_bdn"
s6.cost = [ResourceUsage("CPU", 20)]
s6.utility = ResourceUsage("face following", 5)
s6.utility_min = [ResourceUsage("",0)]
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
s7.id = "webcam follow face"
s7.bringup_function = "strat7_bup"
s7.bringdown_function = "strat7_bdn"
s7.cost = [ResourceUsage("CPU", 10)]
s7.utility = ResourceUsage("face following", 3)
s7.utility_min = [ResourceUsage("",0)]
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

s8 = Strategy()
s8.id = "laser follow face"
s8.bringup_function = "strat8_bup"
s8.bringdown_function = "strat8_bdn"
s8.cost = [ResourceUsage("CPU", 5)]
s8.utility = ResourceUsage("face following", 2)
s8.utility_min = [ResourceUsage("",0)]
s8.source = """
function strat8_bup(params)
{
	se_log("strat8_bup");
	se_log("data: " + params.data);
}

function strat8_bdn(params)
{
	se_log("strat8_bdn");
}
"""
add_strat(s8)

##Arm motion strategy 
s9 = Strategy()
s9.id = "arm motion"
s9.bringup_function = "strat9_bup"
s9.bringdown_function = "strat9_bdn"
s9.cost = [ResourceUsage("CPU", 20)]
s9.utility = ResourceUsage("arm motion", 6)
s9.utility_min = [ResourceUsage("",0)]
s9.source = """
function strat9_bup(params)
{
	se_log("strat9_bup");
	se_log("data: " + params.data);
}

function strat9_bdn(params)
{
	se_log("strat9_bdn");
}
"""
add_strat(s9)

set_max("CPU", 200)

##Detection desires 
des1 = Desire("test_1", "front detection", 1, 1, "{'data' : 1}", False, rospy.Time())
des2 = Desire("test_2", "face detection", 1, 1, "{'data' : 2}", False, rospy.Time())
des3 = Desire("test_3", "large detection", 1, 1, "{'data' : 3}", False, rospy.Time())

##Following desire
des4 = Desire("test_4", "face following", 2, 1, "{'data' : 4}", False, rospy.Time())

##Arm motion desire 
des5 = Desire("test_5", "arm motion", 5, 1, "{'data' : 5}", False, rospy.Time())

desires = DesiresSet()
desires.desires = [des1, des2, des3, des4, des5]



r = rospy.Rate(1) # Once every sec
while not rospy.is_shutdown():
    pub_desires.publish(desires)
    r.sleep()


