#!/usr/bin/env python

import roslib; roslib.load_manifest('hbba_test')
import rospy
from topic_filters_manager.srv import *

rospy.init_node('jn0_h12_run')

rospy.wait_for_service('hbba/register_filter')
register_filter = rospy.ServiceProxy('hbba/register_filter', RegisterFilter)

register_filter("/manyears/stream_filter", "GenericDivisor")
register_filter("/ball_tracking/kinect_filter", "GenericDivisor")
