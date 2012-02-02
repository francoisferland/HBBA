#!/usr/bin/env python

import roslib; roslib.load_manifest("hbba_stats")
import rospy
import roslib.scriptutil as scriptutil
import rosnode
import xmlrpclib
import time
import Queue
import threading

from hbba_stats.msg import *

ID = "hbba_stats"

rospy.init_node("cpu_load")
pub_cpustats = rospy.Publisher("cpu_load", CPUStats)
sample_time = rospy.get_param("~sample_time", 0.5)
num_threads = rospy.get_param("num_threads", 4)

master = scriptutil.get_master()
proc_queue = Queue.Queue()
time_list = []


def get_systime():
    sys_file = open("/proc/stat", "r")
    sys_stats = sys_file.readline().split(" ")[2:5]
    sys_utime = int(sys_stats[0])
    sys_stime = int(sys_stats[2])
    return sys_utime + sys_stime

def get_cputime(pid):
    proc_file = open("/proc/" + str(pid) + "/stat", "r")
    proc_stats = proc_file.readline().split(" ")[13:15] # utime and stime
    proc_utime = int(proc_stats[0])
    proc_stime = int(proc_stats[1])
    return proc_utime + proc_stime

def sample_cpuload(pid):
    sys_before = get_systime()
    before = get_cputime(pid)
    time.sleep(sample_time)
    after = get_cputime(pid)
    sys_after = get_systime()
    sys_delta = float(sys_after - sys_before)
    if (sys_delta > 0):
        return float(after - before) / sys_delta
    else:
        return 0

def sample_node_cpuload(name):
    api_uri = rosnode.get_api_uri(master, name)
    node_api = xmlrpclib.ServerProxy(api_uri)
    node_pid = rosnode._succeed(node_api.getPid(ID))
    return sample_cpuload(node_pid)

def proc_worker():
    while (not rospy.is_shutdown()):
        item = proc_queue.get()
        try:
            # Many errors can happen if a node has been shut down before we
            # could get its pid.
            # Just catch everything and ignore for now.
            pt = sample_node_cpuload(item)
            time_list.append([item, pt])
        except:
            pass
        proc_queue.task_done()

def get_loads(node_names):
    del(time_list[:]) 
    for n in node_names:
        proc_queue.put(n)
    proc_queue.join()
    return time_list

for i in range(num_threads):
    t = threading.Thread(target=proc_worker)
    t.daemon = True
    t.start()

while (not rospy.is_shutdown()):
    nodes = rosnode.get_node_names()
    msg = CPUStats()
    tl = get_loads(nodes)
    for t in tl:
        msg.node_names.append(t[0])
        msg.loads.append(t[1])
    pub_cpustats.publish(msg)


