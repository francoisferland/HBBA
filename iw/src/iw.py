#!/usr/bin/env python

import roslib; roslib.load_manifest("iw")
import rospy

from iw.srv import *
from iw_msgs.msg import *

class iw_server:
	def __init__(self):
		self.desires = {}
		self.srv_add = \
			rospy.Service('add_desire', AddDesire, self.add_desire_srv)
		self.srv_del = \
			rospy.Service('remove_desire', RemoveDesire, self.remove_desire_srv)
		self.pub_set = rospy.Publisher('desires_set', DesiresSet)

	def add_desire_srv(self, req):
		rospy.loginfo('Adding new desire with id ' + req.desire.id)
		self.desires[req.desire.id] = req.desire
		self.publish_set()
		return AddDesireResponse()

	def remove_desire_srv(self, req):
		rospy.loginfo('Removing desire with id ' + req.id)
		if req.id in self.desires:
			del self.desires[req.id]
		else:
			rospy.logwarn('No desire with id ' + req.id)
		self.publish_set()
		return RemoveDesireResponse()

	def publish_set(self):
		dset = DesiresSet()
		dset.desires = self.desires.values()
		self.pub_set.publish(dset)
		
if __name__ == "__main__":
	rospy.init_node('iw')
	iws = iw_server()
	rospy.spin()

