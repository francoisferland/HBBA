#!/usr/bin/env python

import roslib; roslib.load_manifest("iw")
import rospy

from collections import defaultdict

from hbba_msgs.msg import *
from hbba_msgs.srv import *


class iw_server:
    def __init__(self):
        self.desires = {}
        self.srv_add = \
            rospy.Service('add_desires', AddDesires, self.add_desires_srv)
        self.srv_up = rospy.Service('set_desire_intensity', \
			SetDesireIntensity, self.set_desire_intensity_srv)
        self.srv_del = rospy.Service('remove_desires', RemoveDesires, \
			self.remove_desires_srv)
        self.pub_set = rospy.Publisher('desires_set', DesiresSet, latch=True)

    def add_desires_srv(self, req):
        for d in req.desires:
            rospy.loginfo('Adding new desire with id ' + d.id)
            self.desires[d.id] = d
        self.publish_set()
        return AddDesiresResponse()

    def set_desire_intensity_srv(self,req):
    	for d in self.desires:
            if req.id == d.id:
                d.intensity = req.value
        self.publish_set()
        return SetDesireIntensityResponse()

    def remove_desires_srv(self, req):
        for d in req.ids:
            rospy.loginfo('Removing desire with id ' + d)
            if d in self.desires:
                del self.desires[d]
            else:
                rospy.logwarn('No desire with id ' + d)
        self.publish_set()
        return RemoveDesiresResponse()

    def filter_set(self):
        # Go through each desires, find duplicates in a utility class, keep the
        # most intensive ones, flush those with intensity == 0.
        class_map = {}
        dset = []
        for d in self.desires.values():
            if d.intensity > 0.001:
                if d.type not in class_map:
                    class_map[d.type] = []
                class_map[d.type].append(d)
        for ds in class_map.values():
            dm = ds[0]
            for d in ds:
                if d.intensity > dm.intensity:
                    dm = d
            dset.append(dm)

        return dset

    def publish_set(self):
        dset = DesiresSet()
        fs = self.filter_set()
        for d in fs:
            dset.desires.append(d)
        self.pub_set.publish(dset)
        
if __name__ == "__main__":
    rospy.init_node('iw')
    iws = iw_server()
    rospy.spin()

