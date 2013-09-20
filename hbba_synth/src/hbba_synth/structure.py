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
from script_engine.srv import EvalScript
from emotions_msgs.msg import EmoIntensity

rospy.init_node("hbba_struct", anonymous=True)

rospy.wait_for_service("hbba/add_strategy", 1.0)
add_strat = rospy.ServiceProxy("hbba/add_strategy", AddStrategy)

rospy.wait_for_service("hbba/set_resource_max", 1.0)
set_resource_max = rospy.ServiceProxy("hbba/set_resource_max", SetResourceMax)

rospy.wait_for_service("hbba/add_desires", 1.0)
add_desires = rospy.ServiceProxy("hbba/add_desires", AddDesires)

rospy.wait_for_service("hbba/eval_script", 1.0)
eval_script = rospy.ServiceProxy("hbba/eval_script", EvalScript)

pubEmoIntensity = rospy.Publisher("{0}", EmoIntensity)
"""

exploitation_match_sp = """
rospy.wait_for_service("hbba/create_exploitation_matcher", 1.0)
create_em = rospy.ServiceProxy("hbba/create_exploitation_matcher", CreateExploitationMatcher)
"""
exploitation_match_elem = """ExploitationMatch({0}, {1})"""
exploitation_match_call = """
create_em('{0}', {1})"""

def baseNodesXML(debug):
    if debug:
        fname = "base_nodes_debug.launch"
    else:
        fname = "base_nodes.launch"
    e = Element("include", attrib = {
        'file': "$(find hbba_synth)/launch/{0}".format(fname)})
    return e


class Structure:
    def __init__(self):
        self.includes = []
        self.behaviors = {}
        self.procmodules = {}
        self.strategies = {}
        self.filters = {}
        self.filterTypes = {}
        self.exploitationMatches = {}
        self.resources = {}
        self.rootRemaps = {}
        self.desires = {}
        self.arbitrationTypes = {}
        self.integratedArbitration = {}
        self.motivations = {}
        self.emoIntensities = {}
        self.customScript = ""

    def addInclude(self, i):
        self.includes.append(i)

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

    def addResource(self, r):
        self.resources[r.name] = r

    def addRootRemap(self, rm):
        self.addRootRemapEx(rm.to, rm.topic)

    def addRootRemapEx(self, to, topic):
        # Look if the remap target is already registered.
        # If so, map the incoming topic to the original target.
        if to in self.rootRemaps:
            self.rootRemaps[topic] = self.rootRemaps[to]
        else:
            self.rootRemaps[topic] = to

    def addDesire(self, d):
        self.desires[d.desire_id] = d

    def addArbitrationType(self, t):
        self.arbitrationTypes[t.topic] = t

    def addIntegratedArbitration(self, t):
        self.integratedArbitration[t] = True

    def addMotivation(self, m):
        self.motivations[m.name] = m

    def addEmoIntensity(self, e):
        self.emoIntensities[e.name] = e

    def addCustomScript(self, s):
        self.customScript += s + "\n"

    def registerExploitationMatch(self, b, d):
        p = b.priority
        for o in b.output:
            if o not in self.exploitationMatches:
                self.exploitationMatches[o] = {}
            matches = self.exploitationMatches[o]
            if p not in matches:
                matches[p] = []
            matches[p].append(d)

    def generateExploitationMatchesPy(self):
        out = exploitation_match_sp
        for t, ms in self.exploitationMatches.iteritems():
            ems = "["
            ft = self.getRootTopicFullName(t)
            p = ms.keys()
            ds = ms.values()
            l = len(p)
            for i in range(0, l-1):
                ems += exploitation_match_elem.format(p[i], ds[i]) + ", "
            ems += exploitation_match_elem.format(p[l-1], ds[l-1]) + "]"
            out += exploitation_match_call.format(ft, ems)
        return out

    def getRootTopicFullName(self, topic):
        if topic in self.rootRemaps:
            return self.rootRemaps[topic]
        else:
            return '/' + topic

    def generateRootRemapXML(self, topic):
        if type(topic) is dict:
            iname = topic.keys()[0]
            oname = topic.values()[0]
        else:
            iname = topic
            oname = topic

        return Element("remap", attrib = {
            'from': iname,
            'to': self.getRootTopicFullName(oname)})

    def generateArbitrationXML(self, topic):
        if topic in self.arbitrationTypes:
            abtr_pkg = self.arbitrationTypes[topic].pkg
            abtr_type = self.arbitrationTypes[topic].node
        else:
            abtr_pkg = "abtr_priority"
            abtr_type = "Generic"

        root_topic = self.getRootTopicFullName(topic)
        node_name = "abtr_{0}".format(topic)
        n = Element("node", attrib = {
            'name': node_name,
            'pkg': 'nodelet',
            'type': 'nodelet',
            'args': "standalone {0}/{1}".format(abtr_pkg, abtr_type)
            })
        n.append(Element("remap", attrib = {
            'from': "abtr_cmd",
            'to': root_topic}))
        n.append(Element("remap", attrib = {
            'from': "priority",
            'to': "{0}/priority".format(root_topic)}))
        n.append(Element("remap", attrib = {
            'from': "cmd/register",
            'to': "{0}/register".format(root_topic)}))

        return [n]

    def generateExploitationMatcherXML(self, topic):
        root_topic = self.getRootTopicFullName(topic)
        node_name = "exploitation_matcher_{0}".format(topic)
        n = Element("node", attrib = {
            'name': node_name,
            'pkg': 'iw',
            'type': 'exploitation_matcher'})
        n.append(Element("remap", attrib = {
            'from': 'priority',
            'to': "{0}/priority".format(root_topic)}))
        n.append(Element("remap", attrib = {
            'from': 'intention',
            'to': "hbba/intention"}))
        n.append(Element("remap", attrib = {
            'from': 'register_em',
            'to': "{0}/register_exploitation_match".format(root_topic)}))
        return [n]

    def solverModelXML(self, opts):
        if (opts.verbose):
            print "Generating static solver model as rosparams..."

        strats = []
        for strat in self.strategies.values():
            strats.append(strat.generateDict())

        caps = {}
        for r in self.resources.values():
            caps[r.name] = r.value

        s = {'hbba': {'solver_model': {'strategies': strats, 'res_caps': caps}}}
        n = Element("rosparam")
        n.text = str(s)

        return n

    def topicFiltersXML(self, opts):
        if (opts.verbose):
            print "Generating static topic filters list..."

        tfs = {}
        for tf in self.filters.values():
            tfs[tf.name] = tf.type 
        
        n = Element("rosparam")
        n.text = str({'hbba': {'topic_filters': tfs}})

        return n

    def generate(self, basepath, opts):
        if (opts.includes):
            # Just print out includes and quit here - only used for dependency
            # queries.
            for i in self.includes:
                print i
            exit()

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
                    self.registerExploitationMatch(bhvr, u_class)

        if verbose:
            print "Exploitation matches: " + str(self.exploitationMatches)
        
        # XML launch file, first pass
        launch_elem = Element("launch")
        if not opts.behavior_based:
            if verbose:
                print "Adding base HBBA nodes"
            launch_elem.append(baseNodesXML(opts.debug))
        for p in self.procmodules.values():
            launch_elem.extend(p.generateXML(self, opts))
        for b in self.behaviors.values():
            launch_elem.extend(b.generateXML(self, opts))
        if not opts.behavior_based:
            for m in self.motivations.values():
                launch_elem.extend(m.generateXML(self))
        if not opts.disable_arbitration:
            for t in behavior_topics:
                if t not in self.integratedArbitration:
                    launch_elem.extend(self.generateArbitrationXML(t))

        # Python script and second XML generation pass
        if not opts.behavior_based:
            pyscript = ""
            if (self.customScript != ""):
                pyscript += "#Custom script:\n"
                pyscript += "eval_script(\"\"\" \n"
                pyscript += self.customScript
                pyscript += "\n\"\"\")\n\n"
            for e in self.emoIntensities.values():
                pyscript += e.generatePy()
            for s in self.strategies.values():
                pyscript += s.generatePy()
            pyscript += "\n"
            for r in self.resources.values():
                pyscript += r.generatePy()
            pyscript += "\n"
            for d in self.desires.values():
                pyscript += d.generatePy()
            pyscript += "\n"
            if not opts.disable_arbitration:
                pyscript += "\n"
                pyscript += self.generateExploitationMatchesPy()
            pyscript += "\n\nprint \"Stop this script with Ctrl-C when ready.\"\n"
            pyscript += "rospy.spin()\n"

            if verbose:
                print "Generated Python script:\n"
                print pyscript
                print ""

            pyfile = file(basepath + ".py", "w")
            pyfile.write(python_header.format(
                self.getRootTopicFullName("emo_intensity")))
            pyfile.write(pyscript)

            # XML addition: the whole solver model as a rosparam:
            launch_elem.append(self.solverModelXML(opts))
            # XML additions: the static list of topic filters:
            launch_elem.append(self.topicFiltersXML(opts))

        elif verbose:
            print "Behavior-based mode - no Python script generated."

        # Final XML generation:
        launch_tree = ElementTree(launch_elem)
        xml_output = tostring(launch_elem)
        if opts.pretty:
            reparsed = minidom.parseString(xml_output)
            xml_output = reparsed.toprettyxml(indent="  ")

        if verbose:
            print "Generated XML:\n"
            print xml_output

        xmlfile = file(basepath + ".launch", "w")
        xmlfile.write(xml_output)

