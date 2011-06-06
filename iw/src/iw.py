#!/usr/bin/env python

import roslib; roslib.load_manifest("iw")
import rospy

from hbba_msgs.msg import *
from hbba_msgs.srv import *


class iw_server:
    def __init__(self):
        self.desires = {}
        self.srv_add = \
            rospy.Service('add_desires', AddDesires, self.add_desires_srv)
        self.srv_del = \
            rospy.Service('remove_desires', RemoveDesires, self.remove_desires_srv)
        self.pub_set = rospy.Publisher('desires_set', DesiresSet)

    def add_desires_srv(self, req):
        for d in req.desires:
            rospy.loginfo('Adding new desire with id ' + d.id)
            self.desires[d.id] = d
        self.publish_set()
        return AddDesiresResponse()

    def remove_desires_srv(self, req):
        for d in req.ids:
            rospy.loginfo('Removing desire with id ' + d)
            if d in self.desires:
                del self.desires[d]
            else:
                rospy.logwarn('No desire with id ' + d)
        self.publish_set()
        return RemoveDesiresResponse()

    def publish_set(self):
        dset = DesiresSet()
        dset.desires = self.desires.values()
        self.pub_set.publish(dset)
        
if __name__ == "__main__":
    rospy.init_node('iw')
    iws = iw_server()
    rospy.spin()

