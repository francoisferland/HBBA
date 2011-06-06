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
s1a.id = "laser detect legs"
s1a.bringup_function = "strat1a_bup"
s1a.bringdown_function = "strat1a_bdn"
s1a.cost = [ResourceUsage("CPU", 10)]
s1a.utility = ResourceUsage("legs detection", 3)
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

s1b= Strategy()
s1b.id = "kinect detect legs"
s1b.bringup_function = "strat1b_bup"
s1b.bringdown_function = "strat1b_bdn"
s1b.cost = [ResourceUsage("CPU", 20)]
s1b.utility = ResourceUsage("legs detection", 2)
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

s2= Strategy()
s2.id = "kinect detect pedestrian"
s2.bringup_function = "strat2_bup"
s2.bringdown_function = "strat2_bdn"
s2.cost = [ResourceUsage("CPU", 20)]
s2.utility = ResourceUsage("pedestrian detection", 5)
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

##Approach strategy
s3= Strategy()
s3.id = "approach"
s3.bringup_function = "strat3_bup"
s3.bringdown_function = "strat3_bdn"
s3.cost = [ResourceUsage("CPU", 30)]
s3.utility = ResourceUsage("approach",4)
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

##Maintain interaction strategy
s4= Strategy()
s4.id = "maintain interaction"
s4.bringup_function = "strat4_bup"
s4.bringdown_function = "strat4_bdn"
s4.cost = [ResourceUsage("CPU", 20)]
s4.utility = ResourceUsage("maintain interation",3)
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

##Showing strategy
s5= Strategy()
s5.id = "showing"
s5.bringup_function = "strat5_bup"
s5.bringdown_function = "strat5_bdn"
s5.cost = [ResourceUsage("CPU", 20)]
s5.utility = ResourceUsage("showing",2)
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

##Ending interaction strategy
s6= Strategy()
s6.id = "ending interaction"
s6.bringup_function = "strat6_bup"
s6.bringdown_function = "strat6_bdn"
s6.cost = [ResourceUsage("CPU", 20)]
s6.utility = ResourceUsage("ending interaction",5)
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

set_max("CPU", 200)

##Detection desires 
des1 = Desire("test_1", "legs detection", 1, "{'data' : 1}")
des2 = Desire("test_2", "pedestrian detection", 1, "{'data' : 1}")

##Approach desire 
des3 = Desire("test_3", "approach", 1, "{'data' : 1}")
 
##Maintain interation desire
des4 = Desire("test_4", "maintain interation", 1, "{'data' : 1}")

##Showing desire (= arm motion)
des5 = Desire("test_5", "showing", 1, "{'data' : 1}")

##ending interaction desire 
des6 = Desire("test_6", "ending interaction", 1, "{'data' : 1}")

desires = DesiresSet()
desires.desires = [des1, des2, des3, des4, des5, des6]



r = rospy.Rate(1) # 1 Hz
while not rospy.is_shutdown():
    pub_desires.publish(desires)
    r.sleep()


