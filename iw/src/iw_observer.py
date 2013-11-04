#!/usr/bin/env python

import roslib; roslib.load_manifest("iw")
import rospy

from hbba_msgs.msg import *
from hbba_msgs.srv import *

# Prototype for desire production based on IW events.
# The first test is simply to disable specific classes of desires when certain
# events occur.
# The goal is to make this generic in the future.
class iw_observer:
    def __init__(self):
        self.sub   = rospy.Subscriber("events", Event, self.events_cb) 

        self.trigger_types = [
            "GoTo",
            "Teleop"
        ]

        self.max_intensity = 50.0

        self.block_des = [
            Desire(
                id        = "iw_obs_block_locate_legs",
                type      = "LocateLegs",
                utility   = 0.0,
                intensity = 0.0,
                params    = "",
                security  = False),
            Desire(
                id        = "iw_obs_block_locate_face",
                type      = "LocateFace",
                utility   = 0.0,
                intensity = 0.0,
                params    = "",
                security  = False),
        ] 

        self.scl_add = rospy.ServiceProxy("add_desires", AddDesires)

    def events_cb(self, msg):
        if (msg.desire_type in self.trigger_types):
            if (msg.type == Event.EXP_ON):
                self.update(self.max_intensity)
            elif (msg.type == Event.EXP_OFF):
                self.update(0)

    def update(self, intensity):
        for d in self.block_des:
            d.intensity = intensity
        self.scl_add.call(self.block_des)

if __name__ == "__main__":
    rospy.init_node('iw_observer')
    iwo = iw_observer()

    rospy.spin()

