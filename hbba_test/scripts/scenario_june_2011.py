#!/usr/bin/env python

import roslib; roslib.load_manifest('hbba_test')
from roslib.message import get_message_class
import sys
import threading
import time
import rospy
import smach
import smach_ros
import face_emotion
#import sound_play_fr
import gesture
#from topic_filters.srv import SetState
from script_engine.srv import EvalScript
import geometry_msgs
from hbba_msgs.msg import *
from hbba_msgs.srv import *

class SleepState(smach.State):
	def __init__(self, time):
		smach.State.__init__(self, outcomes=['done','preempted'])
		self.sleep_t = time

	def execute(self,ud):
		rospy.sleep(self.sleep_t)
		if self.preempt_requested():
			self.service_preempt()
			return 'preempted'
		print "sleep = "+str(self.sleep_t)
		return 'done'

#define State who publish a msg on a topic
class PubMsg(smach.State):
	def __init__(self, topic_name, type_name):
		smach.State.__init__(self, outcomes=['succeeded','failed'], input_keys=['msg_to_pub'])
		#Get exact type
		topic_type = get_message_class(type_name)
		if topic_type == None:
			print "Error: " + type_name + " is not a valid type."
			return 'failed'
		self.pub = rospy.Publisher(topic_name, topic_type)

	def execute(self, userdata):
		self.pub.publish(userdata.msg_to_pub)
		return 'succeeded'

#define State who wait until there is a msg on a topic
#You can specify a timeout to finish the state if nothing is published
# timeout < 0 : wait infinite time
# time_until_succeed : < 0 => never, = 0 => imediate, > 0 => nb sec
class SubMsg(smach.State):
	def __init__(self, topic_name, type_name):
		smach.State.__init__(self, outcomes=['succeeded','timeout','preempted'], input_keys=['time_to_wait','time_until_succeed'])
		#To verify if a new message arrived or not
		self.new_msg = False
		#To take care of new msg only if the state is in execution
		self.executing = False
		self.topic_name = topic_name
		#Get exact type
		self.topic_type = get_message_class(type_name)
		if self.topic_type == None:
			print "Error: " + type_name + " is not a valid type."
			return 'failed'
		rospy.logdebug("Subscribe to %s with %s", topic_name, self.topic_type)
		self._trigger_cond = threading.Condition()	
	
	def execute(self, userdata):
		#Init seconds memory
		self.seconds_memory = rospy.get_time()
		#rospy.loginfo("Execute sub with %s in %s", self.new_msg, self.topic_type)
		self.new_msg = False
		self.subscriber = rospy.Subscriber(self.topic_name, self.topic_type, self.recv_msg)
		while True:
			self._trigger_cond.acquire()
			#wait for callback or time out
			if userdata.time_to_wait >= 0:
				self._trigger_cond.wait(userdata.time_to_wait)
			else:
				self._trigger_cond.wait()
			self._trigger_cond.release()

			if self.preempt_requested():
				self.service_preempt()
				return 'preempted'

			if self.new_msg == True:
				now = rospy.get_time()
				if self.seconds_memory + userdata.time_until_succeed < now:
					self.seconds_memory = now
					self.new_msg = False
					self.subscriber.unregister()
					return 'succeeded'
			else:
				break
		
		self.subscriber.unregister()
		return 'timeout'

	def recv_msg(self, data):
		self.new_msg = True
		self._trigger_cond.acquire()
		self._trigger_cond.notify()
		self._trigger_cond.release()

	#it does nothing ?
	def request_preempt(self):
		smach.State.request_preempt(self)
		rospy.loginfo("PREEMPT SubMsg")
		self._trigger_cond.acquire()
		self._trigger_cond.notify()
		self._trigger_cond.release()

#define State who wait until there is a msg on a topic
#You can specify a timeout to finish the state if nothing is published
# timeout < 0 : wait infinite time
# time_until_succeed : < 0 => never, = 0 => imediate, > 0 => nb sec
class SubBoolMsg(smach.State):
	def __init__(self, topic_name, type_name):
		smach.State.__init__(self, outcomes=['succeeded','failed','timeout','preempted'],\
				input_keys=['time_to_wait','time_until_succeed','time_until_failed','nb_occurence_in'], output_keys=['nb_occurence_out'])
		#To verify if a new message arrived or not
		self.new_msg = False
		#To take care of new msg only if the state is in execution
		self.executing = False
		self.topic_name = topic_name
		#Get exact type
		self.topic_type = get_message_class(type_name)
		if self.topic_type == None:
			print "Error: " + type_name + " is not a valid type."
			return 'failed'
		rospy.logdebug("Subscribe to %s with %s", topic_name, self.topic_type)
		self._trigger_cond = threading.Condition()	
	
	def execute(self, userdata):
		#Init seconds memory
		self.seconds_memory = rospy.get_time()
		#rospy.loginfo("Execute subBool with %s in %s", self.new_msg, self.topic_type)
		self.new_msg = False
		while True:
			self.subscriber = rospy.Subscriber(self.topic_name, self.topic_type, self.recv_msg)
			#rospy.loginfo("Enter sub pedest")
			self._trigger_cond.acquire()
			#wait for callback or time out
			if userdata.time_to_wait >= 0:
				self._trigger_cond.wait(userdata.time_to_wait)
			else:
				self._trigger_cond.wait()
			self._trigger_cond.release()

			if self.preempt_requested():
				self.service_preempt()
				return 'preempted'

			now = rospy.get_time()
			if self.new_msg == True and userdata.nb_occurence_in < 2:
				if self.msg_data == True:
					if self.seconds_memory + userdata.time_until_succeed < now:
						userdata.nb_occurence_out = 0
						self.subscriber.unregister()
						return 'succeeded'
				elif self.seconds_memory + userdata.time_until_failed < now:
					self.seconds_memory = now
					self.new_msg = False
					userdata.nb_occurence_out = userdata.nb_occurence_in + 1
					self.subscriber.unregister()
					return 'failed'
			elif userdata.nb_occurence_in >= 2 and self.seconds_memory + userdata.time_until_succeed < now:
				rospy.logdebug("Max occurence")
				userdata.nb_occurence_out = 0
				self.subscriber.unregister()
				if self.new_msg == True:
					if self.msg_data == True:
						return 'succeeded'
					else:
						return 'timeout'
				else:
					return 'timeout'
			else:
				rospy.logdebug("Continue sub pedest")
				userdata.nb_occurence_out = userdata.nb_occurence_in + 1
				continue
		
		self.subscriber.unregister()
		userdata.nb_occurence_out = 0
		return 'preempted'

	def recv_msg(self, data):
		self.new_msg = True
		self.msg_data = data.data
		self._trigger_cond.acquire()
		self._trigger_cond.notify()
		self._trigger_cond.release()

	#it does nothing ?
	def request_preempt(self):
		smach.State.request_preempt(self)
		rospy.loginfo("PREEMPT SubMsg")
		self._trigger_cond.acquire()
		self._trigger_cond.notify()
		self._trigger_cond.release()


def main():
	rospy.init_node('smach_usecase_executive')

	# Get interaction mode param to know in wich mode we are
	#interaction_mode = rospy.get_param("interaction_mode")
	#rospy.loginfo("Mode : "+interaction_mode)

	sm_root = smach.StateMachine(outcomes=['succeeded','preempted','aborted'])
    
	rospy.wait_for_service("eval_script")

	# Preempt the active state in case of ROS shutdown
	#smach_ros.set_preempt_handler(sm_root)
	with sm_root:

		sm_root.userdata.nb_convince = 0
		
		des1 = Desire("des1", "Meet_Person", 1, "{'emotion': %d}" % (face_emotion.ATTRACTIVE))
		des2 = Desire("des2", "Attract_Person", 1, "{'emotion': %d}" % (face_emotion.HAPPY))
		des3 = Desire("des3", "Presentation", 1, "{}") # gesture.GIVE_ME done here
		des4 = Desire("des4", "Keep_Engaged", 1, "{}") 
		des4b = Desire("des4b", "Convince", 1, "{'speak': %d}" % (0)) #sound_play_fr.CONVINCE)
		des5 = Desire("des5", "Check_Person", 1, "{}") 
		des6 = Desire("des6", "Disengage", 1, "{'emotion': %d, 'speak': 'Dommage.'}" % (face_emotion.SAD)) #sound_play_fr.TOO_LATE)
		des7 = Desire("des7", "Disengage", 1, "{'emotion': %d, 'speak': 'Tu ne veux pas ecouter.'}" % (face_emotion.ANGRY)) #sound_play_fr.BYE)
		des8 = Desire("des8", "Disengage", 1, "{'emotion': %d, 'speak': 'Merci !'}" % (face_emotion.HAPPY)) # sound_play_fr.FEED_BACK)
		
		#Add action 00
		smach.StateMachine.add('00A_INIT',
				smach_ros.ServiceState("add_desires",AddDesires,
					request = ([des1])),
				transitions={'succeeded':'00T_SUB_LEG','preempted':'preempted'})

		#Add transition 00
		sm_root.userdata.waiting_time_leg = 5
		sm_root.userdata.time_until_succeed_leg = 5
		smach.StateMachine.add('00T_SUB_LEG', SubMsg('pose_detected_leg','geometry_msgs/PoseStamped'), 
				transitions={'timeout':'00T_SUB_LEG','succeeded':'00T_SUB_LEG_STOP','preempted':'preempted'},
				remapping={'time_to_wait':'waiting_time_leg','time_until_succeed':'time_until_succeed_leg'})

		#Add transition 00 Stop
		smach.StateMachine.add('00T_SUB_LEG_STOP',
				smach_ros.ServiceState("remove_desires",RemoveDesires,
					request = (["des1"])),
				transitions={'succeeded':'10A_GREET','preempted':'preempted'})
		
		#Add action 10
		smach.StateMachine.add('10A_GREET',
				smach_ros.ServiceState("add_desires",AddDesires,
					request = ([des2])),
				transitions={'succeeded':'10T_SUB_MEET_PERSON','preempted':'preempted'})

		#Add transition 10
		sm_root.userdata.waiting_time_person = 6
		sm_root.userdata.time_until_succeed_person = 4
		smach.StateMachine.add('10T_SUB_MEET_PERSON', SubMsg('person_detected','geometry_msgs/PoseStamped'),
				transitions={'timeout':'10T_SUB_MEET_PERSON_STOP1','succeeded':'10T_SUB_MEET_PERSON_STOP2','preempted':'preempted'},
				remapping={'time_to_wait':'waiting_time_face','time_until_succeed':'time_until_succeed_face'})
				
		#Add transition 10 Stop1
		smach.StateMachine.add('10T_SUB_MEET_PERSON_STOP1',
				smach_ros.ServiceState("remove_desires",RemoveDesires,
					request = (["des2"])),
				transitions={'succeeded':'25A_TOO_BAD','preempted':'preempted'})

		#Add transition 10 Stop2
		smach.StateMachine.add('10T_SUB_MEET_PERSON_STOP2',
				smach_ros.ServiceState("remove_desires",RemoveDesires,
					request = (["des2"])),
				transitions={'succeeded':'20A_PRESENTATION','preempted':'preempted'})
				
		#Add action 20
		smach.StateMachine.add('20A_PRESENTATION',
				smach_ros.ServiceState("add_desires",AddDesires,
					request = ([des3, des4])),
				transitions={'succeeded':'20T_SUB_FACE','preempted':'preempted'})
				
		#Add transition 20
		sm_root.userdata.waiting_time_face = 6
		sm_root.userdata.time_until_succeed_face = 4
		smach.StateMachine.add('20T_SUB_FACE', SubMsg('face_detected','geometry_msgs/PoseStamped'),
				transitions={'timeout':'20T_SUB_FACE_STOP','succeeded':'310T_SLEEP_STOP','preempted':'preempted'},
		#		transitions={'timeout':'20T_SUB_FACE_STOP','succeeded':'310T_SLEEP','preempted':'preempted'},
				remapping={'time_to_wait':'waiting_time_face','time_until_succeed':'time_until_succeed_face'})
				
		#Add transition 20 Stop
		smach.StateMachine.add('20T_SUB_FACE_STOP',
				smach_ros.ServiceState("remove_desires",RemoveDesires,
					request = (["des3", "des4"])),
				transitions={'succeeded':'25A_TOO_BAD','preempted':'preempted'})

		#Add action 25
		smach.StateMachine.add('25A_TOO_BAD',
				smach_ros.ServiceState("add_desires",AddDesires,
					request = ([des6])),
				transitions={'succeeded':'25T_SLEEP_STOP','preempted':'preempted'})
		#		transitions={'succeeded':'25T_SLEEP','preempted':'preempted'})
	
		#Add transition 25 (sleep)
		#smach.StateMachine.add('25T_SLEEP', SleepState(3),
		#		transitions={'done':'25T_SLEEP_STOP','preempted':'preempted'})
	
		#Add transition 25 Stop
		smach.StateMachine.add('25T_SLEEP_STOP',
				smach_ros.ServiceState("remove_desires",RemoveDesires,
					request = (["des6"])),
				transitions={'succeeded':'50T_SLEEP','preempted':'preempted'})
		
		#Add transition 310	(sleep)	# Attente qu'il finisse de raconter l'histoire ... 
		#smach.StateMachine.add('310T_SLEEP', SleepState(3),
		#		transitions={'done':'310T_SLEEP_STOP','preempted':'preempted'})

		#Add transition 310 Stop
		smach.StateMachine.add('310T_SLEEP_STOP',
				smach_ros.ServiceState("remove_desires",RemoveDesires,
					request = (["des3"])),
				transitions={'succeeded':'315A_ACTIVE_CHECK_HUMAN','preempted':'preempted'})
				
		#Add action 315
		smach.StateMachine.add('315A_ACTIVE_CHECK_HUMAN',
				smach_ros.ServiceState("add_desires",AddDesires,
					request = ([des5])),
				transitions={'succeeded':'330T_SUB_PEDEST','preempted':'preempted'})
				
		#Add action 320
		smach.StateMachine.add('320A_CONVINCE',
				smach_ros.ServiceState("add_desires",AddDesires,
					request = ([des4b])),
				transitions={'succeeded':'320T_SLEEP_STOP','preempted':'preempted'})
		#		transitions={'succeeded':'320T_SLEEP','preempted':'preempted'})
								
		#Add transition 320 (sleep)
		#smach.StateMachine.add('320T_SLEEP', SleepState(3),
		#		transitions={'done':'320T_SLEEP_STOP','preempted':'preempted'})
				
		#Add transition 320 Stop
		smach.StateMachine.add('320T_SLEEP_STOP',
				smach_ros.ServiceState("remove_desires",RemoveDesires,
					request = (["des4b"])),
				transitions={'succeeded':'330T_SUB_PEDEST','preempted':'preempted'})
				
		#Add transition 330,
		sm_root.userdata.waiting_time_pedestrian = 5
		sm_root.userdata.time_until_succeed_pedestrian = 3
		sm_root.userdata.time_until_failed_pedestrian = 4
		smach.StateMachine.add('330T_SUB_PEDEST', SubBoolMsg('pedestrian_detected','std_msgs/Bool'),
				transitions={'timeout':'330T_SUB_PEDEST_STOP1','succeeded':'330T_SUB_PEDEST_STOP2','failed':'320A_CONVINCE','preempted':'preempted'},
				remapping={'time_to_wait':'waiting_time_pedestrian','time_until_failed':'time_until_failed_pedestrian','time_until_succeed':'time_until_succeed_pedestrian','nb_occurence_in':'nb_convince','nb_occurence_out':'nb_convince'})

		#Add transition 330 Stop1
		smach.StateMachine.add('330T_SUB_PEDEST_STOP1',
				smach_ros.ServiceState("remove_desires",RemoveDesires,
					request = (["des4", "des5"])),
				transitions={'succeeded':'340A_BYE_ANGRY','preempted':'preempted'})
				
		#Add transition 330 Stop2
		smach.StateMachine.add('330T_SUB_PEDEST_STOP2',
				smach_ros.ServiceState("remove_desires",RemoveDesires,
					request = (["des4", "des5"])),
				transitions={'succeeded':'350A_THANKS','preempted':'preempted'})
						
		#Add action 340
		smach.StateMachine.add('340A_BYE_ANGRY',
				smach_ros.ServiceState("add_desires",AddDesires,
					request = ([des7])),
				transitions={'succeeded':'340T_SLEEP_STOP','preempted':'preempted'})
		#		transitions={'succeeded':'340T_SLEEP','preempted':'preempted'})
								
		#Add transition 340 (sleep)
		#smach.StateMachine.add('340T_SLEEP', SleepState(3),
		#		transitions={'done':'340T_SLEEP_STOP','preempted':'preempted'})
						
		#Add transition 340 Stop 
		smach.StateMachine.add('340T_SLEEP_STOP',
				smach_ros.ServiceState("remove_desires",RemoveDesires,
					request = (["des7"])), 
				transitions={'succeeded':'50T_SLEEP','preempted':'preempted'})
				
		#Add action 350
		smach.StateMachine.add('350A_THANKS',
				smach_ros.ServiceState("add_desires",AddDesires,
					request = ([des8])),
				transitions={'succeeded':'350A_THANKS_STOP','preempted':'preempted'})
		#		transitions={'succeeded':'350T_SLEEP','preempted':'preempted'})
								
		#Add transition 350 (sleep)
		#smach.StateMachine.add('350T_SLEEP', SleepState(3),
		#		transitions={'done':'350A_THANKS_STOP','preempted':'preempted'})
				
		#Add transition 350 Stop
		smach.StateMachine.add('350A_THANKS_STOP',
				smach_ros.ServiceState("remove_desires",RemoveDesires,
					request = (["des8"])), 
				transitions={'succeeded':'50T_SLEEP','preempted':'preempted'})
				
		#Add transition 50 (sleep)
		sm_root.userdata.waiting_time_init = 8
		sm_root.userdata.time_until_succeed_init = 1
		smach.StateMachine.add('50T_SLEEP', SubMsg('init_detected','std_msgs/Empty'),
				transitions={'timeout':'50T_SLEEP','succeeded':'00A_INIT','preempted':'preempted'},
				remapping={'time_to_wait':'waiting_time_init','time_until_succeed':'time_until_succeed_init'})

	# Create and start the introspection server
	sis = smach_ros.IntrospectionServer('server_name', sm_root, '/SM_ROOT')
	sis.start()

	#Execute the state machine
	outcome = sm_root.execute()

	rospy.spin()
	sis.stop()

if __name__ == '__main__':
	main()
