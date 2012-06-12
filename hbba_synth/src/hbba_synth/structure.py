from definitions import *
from xml.etree.ElementTree import ElementTree, Element, tostring 
from xml.dom import minidom

python_header = """
import roslib; roslib.load_manifest("hbba_synth")
import rospy;
from hbba_msgs.msg import *
from hbba_msgs.srv import *

rospy.init_node("hbba_struct", anonymous=True)

rospy.wait_for_service("add_strategy", 1.0)
add_strat = rospy.ServiceProxy("add_strategy", AddStrategy)

"""

class Structure:
    def __init__(self):
        self.behaviors = []
        self.procmodules = []
        self.strategies = []
        self.filters = {}
        self.filterTypes = {}

    def addBehavior(self, b):
        self.behaviors.append(b)

    def addProcModule(self, p):
        self.procmodules.append(p)

    def addStrategy(self, s):
        self.strategies.append(s)

    def addFilter(self, n, f):
        self.filters[n] = f

    def addFilterType(self, n, ft):
        self.filterTypes[n] = ft

    def generate(self, basepath, verbose):
        # XML launch file
        launch_elem = Element("launch")
        for p in self.procmodules:
            launch_elem.extend(p.generateXML())
        for b in self.behaviors:
            launch_elem.extend(b.generateXML())

        launch_tree = ElementTree(launch_elem)
        if verbose:
            print "Generated XML:\n"
            rough = tostring(launch_elem)
            reparsed = minidom.parseString(rough)
            print reparsed.toprettyxml(indent="  ")


        launch_tree.write(basepath + ".launch")

        # Python script
        pyscript = ""
        for s in self.strategies:
            pyscript += s.generatePy()

        if verbose:
            print "Generated Python script:\n"
            print pyscript

        pyfile = file(basepath + ".py", "w")
        pyfile.write(python_header)
        pyfile.write(pyscript)

        # Last pass: register behavior priorities and strategies to the
        # exploitation matcher.
        # TODO!

