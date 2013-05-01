#!/usr/bin/env python

import roslib; roslib.load_manifest('hbba_tools')
import rospy
from hbba_msgs.msg import Intention

def intentionCB(msg):
    print "---"
    print "Enabled strategies:"
    for i in range(0, len(msg.strategies)):
        if msg.enabled[i]:
            print " - ", msg.strategies[i]
    print "---"

if __name__ == "__main__":
    rospy.init_node('get_intention', anonymous=True)
    rospy.Subscriber("hbba/intention", Intention, intentionCB)

    print "Waiting for first intention message..." 

    rospy.spin()

