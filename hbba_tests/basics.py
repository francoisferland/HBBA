#!/usr/bin/env python

import roslib; roslib.load_manifest('hbba_tests')
import rospy
from iw_translator_v8.srv import *
from iw_translator_v8.msg import *
from iw.srv import *
from iw_msgs.msg import *

rospy.wait_for_service('add_strategy')
add_strat = rospy.ServiceProxy('add_strategy', AddStrategy)
set_max = rospy.ServiceProxy('set_resource_max', SetResourceMax)
add_desire = rospy.ServiceProxy('add_desire', AddDesire)
s1 = Strategy()
s1.id = "strat1"
s1.bringup_function = "strat1_bup"
s1.bringdown_function = "strat1_bdn"
s1.cost = [ResourceUsage("cpu", 10)]
s1.utility = ResourceUsage("test", 1)
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
set_max("cpu", 100)
des = Desire("test_d", "test", 1, "{'data' : 1}")
add_desire(des)

