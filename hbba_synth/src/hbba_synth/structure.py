from definitions import *
from xml.etree.ElementTree import ElementTree, Element, tostring 
from xml.dom import minidom
from sets import Set

# Some constant expressions that are formatted later:

python_header = """
import roslib; roslib.load_manifest("hbba_synth")
import rospy;
from hbba_msgs.msg import *
from hbba_msgs.srv import *

rospy.init_node("hbba_struct", anonymous=True)

rospy.wait_for_service("add_strategy", 1.0)
add_strat = rospy.ServiceProxy("add_strategy", AddStrategy)

"""

priority_match_sp = """
register_pm=rospy.ServiceProxy("{0}/register_priority_match", ExploitationMatch)"""
priority_match_call = """
register_pm({0}, {1})"""

def generateArbitration(topic):
    node_name = "abtr_{0}".format(topic)
    n = Element("node", attrib = {
        'name': node_name,
        'pkg': 'nodelet',
        'type': 'nodelet',
        'args': 'standalone abtr_priority/Generic'
        })
    n.append(Element("remap", attrib = {
        'from': "{0}/abtr_cmd".format(node_name),
        'to': topic}))
    n.append(Element("remap", attrib = {
        'from': "{0}/cmd/register".format(node_name),
        'to': "{0}/register".format(topic)}))

    return [n]


class Structure:
    def __init__(self):
        self.behaviors = {}
        self.procmodules = {}
        self.strategies = {}
        self.filters = {}
        self.filterTypes = {}
        self.priorityMatches = {}

    def addBehavior(self, b):
        self.behaviors[b.name] = b

    def addProcModule(self, p):
        self.procmodules[p.name] = p

    def addStrategy(self, s):
        self.strategies[s.name] = s

    def addFilter(self, f):
        self.filters[f.name] = f

    def addFilterType(self, ft):
        self.filterTypes[ft.name] = ft

    def registerPriorityMatch(self, b, d):
        p = b.priority
        for o in b.output:
            if o not in self.priorityMatches:
                self.priorityMatches[o] = {}
            matches = self.priorityMatches[o]
            if p not in matches:
                matches[p] = []
            matches[p].append(d)

    def generatePriorityMatchesPy(self):
        out = ""
        for t, ms in self.priorityMatches.iteritems():
            out += priority_match_sp.format(t)
            for p, ds in ms.iteritems():
                out += priority_match_call.format(p,ds)
        return out

    def generate(self, basepath, opts):
        verbose = opts.verbose
        # Analysis pass: Gather every behavior topics.
        behavior_topics = Set([])
        for b in self.behaviors.values():
            for o in b.output:
                behavior_topics.add(o)
        if verbose:
            print "Behavior topics: " + str(behavior_topics)

        # Analysis pass: Register behavior priorities and strategies to the
        # exploitation matcher.
        # NOTE: Could be combined to the previous pass ?
        for s in self.strategies.values():
            u_class = s.utility_class
            for m in s.modules:
                module_name = m.module_name
                if module_name in self.behaviors:
                    bhvr = self.behaviors[module_name]
                    # Registration script is generated later:
                    self.registerPriorityMatch(bhvr, u_class)

        if verbose:
            print "Priority matches: " + str(self.priorityMatches)
        
        # XML launch file
        launch_elem = Element("launch")
        for p in self.procmodules.values():
            launch_elem.extend(p.generateXML())
        for b in self.behaviors.values():
            launch_elem.extend(b.generateXML())
        if opts.generate_arbitration:
            for t in behavior_topics:
                launch_elem.extend(generateArbitration(t))

        launch_tree = ElementTree(launch_elem)
        if verbose:
            print "Generated XML:\n"
            rough = tostring(launch_elem)
            reparsed = minidom.parseString(rough)
            print reparsed.toprettyxml(indent="  ")


        launch_tree.write(basepath + ".launch")

        # Python script
        pyscript = ""
        for s in self.strategies.values():
            pyscript += s.generatePy()
        pyscript += "\n"
        pyscript += self.generatePriorityMatchesPy()

        if verbose:
            print "Generated Python script:\n"
            print pyscript
            print ""

        pyfile = file(basepath + ".py", "w")
        pyfile.write(python_header)
        pyfile.write(pyscript)



