#!/usr/bin/env python

#
# TODO try dynamic reconfigure for testing
#

import roslib; roslib.load_manifest('abtr_priority')
import rospy
import rosgraph.masterapi

import subprocess

#
# Connections, {target => sources}
#

connections = {}

#
# A source is a topic/action which want to publish on a particular
# target. There is usually multiple source for a target, thus
# a priority is used to select which source is forwarded to its target.
#
# ex. /repeater1/say is a valid source if
#      - /repeater1/say/abtr_target is defined
#      - /repeater1/say/abtr_priority is defined
#      - /repeater1/say is an advertised topic or action
#

class Source :

    ns = None

    def __init__(self, ns) :
        self.ns = ns

    def __repr__(self) :
        return '<< ' + self.ns + ' (' + str(self.priority()) + ')'

    def priority(self) :
        return float(rospy.get_param(self.ns + '/abtr_priority'))

    def target(self) :
        return Target(rospy.get_param(self.ns + '/abtr_target'))

#
# A target is a topic/action on which one or more sources want to publish.
# A Target object will be constructed for every topics mentioned in
# .../abtr_target.

class Target :

    ns   = None
    type = None
    mux  = None

    def __init__(self, ns) :
        self.ns = ns

    def __repr__(self) :
        return self.ns + ' <<'

    def __hash__(self) :
        return self.ns.__hash__()

    def __eq__(self, other) :
        return self.ns == other.ns

# A mux (multiplexer) is a node which is placed between a target and its
# sources. It allows forward a selected source to the target.

class Mux :

    process = None
    node_name = None

    def __init__(self, target) :
        self.node_name = target.ns + '/mux'
        self.process = subprocess.Popen(
            'rosrun abtr_priority action_mux ' +
            'action_mux:=' + self.node_name + ' ' +
            '_action_server:=' + target.ns, shell=True)

#
# Finds each namespace including an abtr_target parameter.
# Those namespaces are called sources, and must also be a ROS
# topic or action.
#
# TODO check priority?
#
# ex. /repeater1/say/abtr_target = /voice/say
#     /repeater2/say/abtr_target = /voice/say
#     [...] (other paramaters in the server)
#
# will give {'/voice/say' => ['/repeater1/say','/repeater2/say']}
#

def fill_connections():
    for param in rospy.get_param_names():
        i = param.rfind('abtr_target')
        if i != -1 :
            ns = param[0:i-1]
            source = Source(ns)
            target = source.target()
            if not target in connections :
                connections[target] = [source]
            else :
                connections[target].append(source)

# Tells if a certain topic is advertised
# (method found in rostopic utility)

def is_advertised(topic):
    master = rosgraph.masterapi.Master('/rostopic')
    pubs, _, _ = master.getSystemState()
    # pubs sould contain all the advertised topics.
    # pub[0] being the name of a topic.
    for pub in pubs :
        if pub[0] == topic :
            return True
    return False

def create_muxes():
    for target, clients in connections.items() :
        # Loop until one of the clients advertise its namespace
        # as a topic or an action.
        #
        # ex. if the parameter /repeater/say/abtr_priority exists
        # and /repeater/say is advertised as an action.
        while target.type == None :
            for client in clients :
                # We assume that a namespace is advertised as an
                # action if it contains a goal topic.
                if is_advertised(client.ns + '/goal') :
                    target.type = 'action'
                    break
                if is_advertised(client.ns) :
                    target.type = 'topic'
                    break
            # nothing advertised for now, wait a little
            rospy.sleep(0.5)
        # TODO standard topics handling
        if target.type == 'topic' :
            rospy.loginfo('Topic arbitration with multiplexers not implemented yet')
            continue
        # So, we have an action, named target.ns
        # Lets create a multiplexer connecting it with its clients
        rospy.loginfo('Launching a mux for the target ' + target.ns)
        target.mux = Mux(target)

def select_muxes():

    pass

#
# ROS node
#

rospy.init_node('say_action_client')

#abtr_ns = find_abtr_ns()

fill_connections()

#rospy.loginfo(str(connections))

create_muxes()

while not rospy.is_shutdown():
#    find_abtr_actions(abtr_ns)
    rospy.sleep(1.0)
#    client.wait_for_result()
