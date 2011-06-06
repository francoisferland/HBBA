#!/usr/bin/env python

import roslib; roslib.load_manifest('hbba_test')
import rospy
from hbba_msgs.msg import *
from hbba_msgs.srv import *

rospy.init_node('iw_solver_test')
rospy.wait_for_service('add_strategy')
add_strat = rospy.ServiceProxy('add_strategy', AddStrategy)
set_max = rospy.ServiceProxy('set_resource_max', SetResourceMax)

##Legs Detection Strategies 
s1 = Strategy()
s1.id = "laser detect legs"
s1.bringup_function = "strat1_bup"
s1.bringdown_function = "strat1_bdn"
s1.cost = [ResourceUsage("CPU", 10)]
s1.utility = ResourceUsage("legs detection", 3)
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

##Resource Max
set_max("CPU", 200)

r = rospy.Rate(1) # 1 Hz
