#!/usr/bin/env python
import roslib; roslib.load_manifest('topic_filters')
from roslib.message import get_message_class
from topic_filters.srv import *
import rospy
import sys

class switch_filter:
	def __init__(self, topic_in, topic_out, typename):
		self.srv = rospy.Service(topic_out + '/switch_set_state', SetState, self.state_cb)
		self.srv = rospy.Service(topic_out + '/set_divisor_rate', SetDivisorRate, self.divisor_cb)
		self.active = rospy.get_param('~active', False)			#By default the switch is of
		self.divisor_rate = rospy.get_param('~divisor_rate', 1)	#By default the divisor rate does not change the normal rate

		self.counter = 0

		topic_type = get_message_class(typename)
		if topic_type == None:
			print "Error: " + typename + " is not a valid message type."
			exit(1)

		self.sub = rospy.Subscriber(topic_in, topic_type, self.recv_cb)
		self.pub = rospy.Publisher(topic_out, topic_type)

	def state_cb(self, req):
		self.active = req.state
		return SetStateResponse()

	def divisor_cb(self, req):
		self.divisor_rate = req.divisor
		self.counter = 0
		return SetDivisorRateResponse()

	def recv_cb(self, data):
		if self.active:
			if self.counter == self.divisor_rate - 1:
				self.pub.publish(data)
				self.counter = 0
			else:
				self.counter += 1

if __name__ == "__main__":
	if len(sys.argv) < 4:
		print "Usage: " + sys.argv[0] + " topic_in topic_out topic_type"
		exit()	

	topic_in = sys.argv[1]
	topic_out = sys.argv[2]
	topic_type = sys.argv[3]

	rospy.init_node('switch_filter')
	s = switch_filter(topic_in, topic_out, topic_type)
	rospy.spin()

