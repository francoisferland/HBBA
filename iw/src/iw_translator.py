#!/usr/bin/env python

import roslib; roslib.load_manifest("iw")
import rospy

from iw.msg import *

class iw_translator:
	def __init__(self):
		self.sub_desires = rospy.Subscriber("desires_set", DesiresSet, self.desires_cb)

	def desires_cb(self, msg):
		for d in msg.desires:
			rospy.loginfo("Activating intention " + d.id + " ...")

if __name__ == "__main__":
	rospy.init_node("iw_translator")
	iwt = iw_translator()
	rospy.spin()

