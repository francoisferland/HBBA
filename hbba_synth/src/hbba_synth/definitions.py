from xml.etree.ElementTree import Element, tostring
from file_parser import FileParser
from time import time

# {0}: name
# {1}: utility (ResourceUsage string)
# {2}: cost (ResourceUsage array string)
# {3}: dependencies (ResourceUsage array string)
# {3}: source (javascript escaped string)
StratTemplate = "\n\
strat_{0} = Strategy()\n\
strat_{0}.id = '{0}'\n\
strat_{0}.bringup_function = '{0}_bup'\n\
strat_{0}.bringdown_function = '{0}_bdn'\n\
strat_{0}.utility = {1}\n\
strat_{0}.cost = {2}\n\
strat_{0}.utility_min = {3}\n\
strat_{0}.source = \"\"\"\n{4}\"\"\"\n\
add_strat(strat_{0})\n\
"

def uniqueName():
    return str(int(time() * 1e9))

class FilterDef:
    def __init__(self, name, type):
        self.name = name
        self.type = type

class FilterTypeDef:
    def __init__(self, content, structure, verbose=False):
        self.name = content['name']
        if verbose:
            print "Emitted filter type {0}.".format(self.name)
        self.function = content['function']
        structure.addFilterType(self)

    def generateSetLevel(self, name, level):
        return "{0}({1}, {2})".format(self.func, name, level)

class LaunchDef:
    def __init__(self, content, verbose=False):
        self.pkg = content['pkg']
        self.path = content['path']

        if verbose:
            print "Emitted launch element for {0}/{1}.".format(self.pkg,
                self.path)
    def generateXML(self, structure):
        elems = []
        elems.append(Element("include",
            attrib={
                'file': "$(find {0})/{1}".format(self.pkg, self.path)
            }))
        return elems

class BehaviorDef:
    def __init__(self, content, structure, verbose=False):
        if not 'name' in content:
            print "Error: behavior element with no name."
            exit(-1)
        self.name = content['name']
        try:
            self.output = content['output']
            self.priority = content['priority']
        except KeyError as e:
            print "Error: Missing {0} in {1}".format(e, self.name)
            exit(-1)
        if 'launch' in content:
            self.launch = LaunchDef(content['launch'], verbose)
        else:
            self.launch = ""
        if 'input' in content:
            self.input = content['input']
        else:
            self.input = []
        if 'services' in content:
            self.services = content['services']
        else:
            self.services = []

        structure.addBehavior(self)
        if verbose:
            print "Emitted Behavior '{0}'.".format(self.name)

    def inputFilterName(self, name):
        if type(name) is dict:
            n = name.keys()[0]
        else:
            n = name
        return self.name + "/" + n + "_input_filter"
    def inputFilterNodeName(self, name):
        return name + "_input_filter"
    def outputFilterName(self, name):
        return self.name + "/" + name + "_output_filter"
    def outputFilterNodeName(self, name):
        return name + "_output_filter"
    def outputFilterTopic(self, name):
        return name + "_out"

    def appendFilter(self, grp, elems, node_name, name, topic_in, topic_out):
        # elems should be a namespace containing grp
        grp.append(Element("node", attrib={
            'name': node_name,
            'pkg': 'nodelet',
            'type': 'nodelet',
            'args': 
            "standalone topic_filters/GenericDivider {0} {1}".format(
                topic_in, topic_out)
            }))
        elems.append(Element("node", attrib={
            'name': "register_{0}_{1}".format(
                node_name,
                uniqueName()), 
            'pkg': 'topic_filters_manager',
            'type': 'register',
            'args': "{0} GenericDivider".format(
                name)
            }))

    def generateXML(self, structure, opts):
        elems = []
        grp = Element("group", attrib={'ns': self.name})
        # Input remaps, have to go first to affect included nodes:
        # Note: the actual position is not important if inputs are filtered
        for i in self.input:
            topic = TopicDef(i, structure)
            if (not opts.behavior_based) and topic.filtered:
                filter_in = topic.getRootTopicFullName()
                filter_out = topic.name
                self.appendFilter(grp, elems, 
                    self.inputFilterNodeName(filter_out), 
                    self.inputFilterName(filter_out),
                    filter_in, filter_out)
            else:
                grp.append(topic.generateRootRemapXML())

        for s in self.services:
            grp.append(structure.generateRootRemapXML(s))

        if (self.launch != ""):
            grp.extend(self.launch.generateXML(structure))
        elems.append(grp)
        for o in self.output:
            root_topic = structure.getRootTopicFullName(o)
            if opts.behavior_based:
                abtr_output_topic = o 
            else:
                abtr_output_topic = self.outputFilterTopic(o)
                # Add output filter.
                self.appendFilter(grp, elems,
                    self.outputFilterNodeName(o),
                    self.outputFilterName(o),
                    o,
                    self.outputFilterTopic(o))

            # Add registration script.
            grp.append(Element("param", attrib={
                'name': abtr_output_topic + "/abtr_priority",
                'value': str(self.priority)
                }))
            elems.append(Element("node", attrib={
                'name': "register_{0}_{1}_{2}".format(
                    self.name,
                    abtr_output_topic,
                    uniqueName()),
                'pkg': 'abtr_priority',
                'type': 'register',
                'args': "{0} /{1}/{2}".format(root_topic, self.name,
                    abtr_output_topic)
                }))

        return elems

class ProcModuleDef:
    def __init__(self, content, structure, verbose=False):
        self.structure = structure
        self.verbose = verbose

        if not 'name' in content:
            print "Error: procmodule element with no name."
            exit(-1)
        self.name = content['name']

        try:
            self.launch = LaunchDef(content['launch'], verbose)
            if 'input' in content:
                self.input = content['input']
            else:
                self.input = []
        except KeyError as e:
            print "Error: Missing {0} in {1}".format(e, self.name)
        if 'output' in content:
            self.output = content['output']
        else:
            self.output = []

        if 'namespace_exports' in content:
            self.generateNSExports(content['namespace_exports'])
        else:
            self.namespace_exports = []
        structure.addProcModule(self)

        if verbose:
            print "Emitted ProcModule '{0}'.".format(self.name)

    def generateNSExports(self, exports):
        for e in exports:
            t = TopicDef(e, self.structure)
            self.structure.addRootRemapEx("/" + self.name + "/" + t.name, t.src)

    def createInputFilter(self, topic):
        elems = []
        filter_name = "{0}_{1}_filter".format(self.name, topic.src)
        node_name = "{0}_{1}_filter".format(self.name, topic.src)
        filter_type = "GenericDivider"
        if self.verbose:
            print "Adding filter {0}".format(filter_name)
        self.structure.addFilter(FilterDef(filter_name, filter_type))
        elems.append(Element("node", attrib = {
            'name': node_name,
            'pkg': 'nodelet',
            'type': 'nodelet',
            'args': 
                "standalone topic_filters/{0} {1} {2}/{3}".format(
                    filter_type, 
                    self.structure.getRootTopicFullName(topic.src), 
                    self.name, topic.name)
            }))
        elems.append(Element("node", attrib={
            'name': "register_{0}".format(node_name),
            'pkg': 'topic_filters_manager',
            'type': 'register',
            'args': "{0} {1}".format(
                filter_name, 
                filter_type)
            }))
        return elems

    def generateXML(self, structure, opts):
        elems = []
        grp = Element("group", attrib={'ns': self.name})
        elems.append(grp)

        # Output remaps (unfiltered, needed for included nodes.):
        for o in self.output: 
            grp.append(structure.generateRootRemapXML(o))

        for i in self.input:
            topic = TopicDef(i, structure)
            # Filter by default
            if (not opts.behavior_based) and (topic.filtered == True or topic.filtered == None):
                elems.extend(self.createInputFilter(topic))
            else:
                grp.append(topic.generateRootRemapXML())

        grp.extend(self.launch.generateXML(structure))

        return elems 

class CostDef:
    def __init__(self, key, val, verbose=False):
        self.name = key
        self.value = float(val)
        if (verbose):
            print "Added new Costdef with {0}, {1} ({2})".format(key, val, float(val))
    
    def generatePy(self):
        return "ResourceUsage(\"{0}\", {1})".format(self.name, self.value)

def generateCostDefArrayPy(costs):
    cstr = "["
    l = len(costs)
    for i in range(0,l-1):
        cstr += costs[i].generatePy() + ", "
    if (l > 0):
        cstr += costs[l-1].generatePy()
    return cstr + "]"

class ModuleLinkDef:
    def __init__(self, content, structure, verbose=False):
        self.structure = structure
        if type(content) is dict:
            self.module_name = content.keys()[0]
            self.filters = content.values()[0]
        else:
            # Behavior link:
            self.module_name = content
            bm = structure.behaviors[self.module_name]
            self.filters = []
            for i in bm.input:
                t = TopicDef(i, structure)
                if t.filtered:
                    self.filters.append(bm.inputFilterName(t.name))
            for o in bm.output:
                self.filters.append(bm.outputFilterName(o))

    def generateJSCall(self, func, fname, level):
        return "{0}('{1}', {2});\n".format(func, fname, level)

    def parseFilter(self, f):
        if type(f) is dict:
            fname = "{0}_{1}_filter".format(self.module_name, f.keys()[0])
            level = f.values()[0]
            try:
                ftype = self.structure.filters[fname].type
            except KeyError as e:
                print "Unknown filter {0}".format(e)
                exit(-1)
        else:
            # Behavior module link, default values:
            fname = f
            level = 1 # Activation default value
            ftype = "GenericDivider"
        try:
            ffunc = self.structure.filterTypes[ftype].function
        except KeyError as e:
            print "Unknown filter type {0}".format(e)
            exit(-1)

        return (fname, ftype, ffunc, level)

    def generateActivationJS(self):
        src = ""
        for f in self.filters:
            (fname, ftype, ffunc, level) = self.parseFilter(f)
            src += "  " + self.generateJSCall(ffunc, fname, level)
        return src

    def generateDeactivationJS(self):
        src = ""
        for f in self.filters:
            (fname, ftype, ffunc, level) = self.parseFilter(f)
            src += "  " + self.generateJSCall(ffunc, fname, 0)
        return src

class StratDef:
    def __init__(self, content, structure, verbose=False):
        if not 'name' in content:
            print "Error: strat element with no name."
            exit(-1)
        self.name = content['name']
        self.structure = structure
        try:
            self.utility_class = content['class']
            utility = content['utility']
            self.utility = CostDef(self.utility_class, utility, verbose)
            self.costs = []
            if ('costs' in content):
                for key,val in content['costs'].iteritems():
                    self.costs.append(CostDef(key, val, verbose))
            self.dependencies = []
            if ('dependencies' in content):
                deps = content['dependencies']
                if (type(deps) is dict):
                    for key,val in deps.iteritems():
                        self.dependencies.append(CostDef(key, val, verbose))
                else:
                    print "Warning: 'dependencies' in {0} is not a " \
                          "dictionary.".format(self.name)
            self.modules = []
            if ('modules' in content):
                for m in content['modules']:
                    self.modules.append(ModuleLinkDef(m, self.structure, verbose))
            self.custom_bringup = ""
            if ('custom_bringup' in content):
                cb = content['custom_bringup']
                if type(cb) is str:
                    if verbose:
                        print "Adding custom bringup script for " + self.name
                    self.custom_bringup = cb
                else:
                    print "Error: custom_bringup element is not a string."
                    exit(-1)
                
        except KeyError as e:
            print "Error: Missing {0} in {1}".format(e, self.name)
            exit(-1)
        
        if verbose:
            print "Emitted strategy {0}".format(self.name)

        structure.addStrategy(self)

    def generatePy(self):
        costs = "["
        l = len(self.costs)

        bup_name = "{0}_bup".format(self.name)
        bdn_name = "{0}_bdn".format(self.name)
        js_source = "function {0}(params) {{\n".format(bup_name)
        for m in self.modules:
            js_source += m.generateActivationJS()
        if (self.custom_bringup != ""):
            js_source += self.custom_bringup + "\n"
        js_source += "}\n"

        js_source += "function {0}(params) {{\n".format(bdn_name)
        for m in self.modules:
            js_source += m.generateDeactivationJS()
        js_source += "}\n"
        
        return StratTemplate.format(
            self.name,
            self.utility.generatePy(),
            generateCostDefArrayPy(self.costs),
            generateCostDefArrayPy(self.dependencies),
            js_source)

class ResourceDef:
    def __init__(self, name, value):
        self.name = name
        self.value = value

    def generatePy(self):
        return "set_resource_max('{0}', {1})\n".format(self.name, self.value)

class ResourceSetDef:
    def __init__(self, content, structure, verbose=False):
        if type(content) is not dict:
            print "Error: resources element is not a dictionary."
            print type(content)
            exit(-1)
        for r_k, r_v in content.iteritems():
            structure.addResource(ResourceDef(r_k, r_v))

class IncludeDef:
    def __init__(self, content, structure, verbose=False):
        if type(content) is not dict:
            print "Error: include clause is not a dictionary."
            exit(-1)
        try:
            self.pkg = content['pkg']
            self.fname = content['file']
        except KeyError as e:
            print "Error: Missing {0} in {1}.".format(e, content)
            exit(-1)

        if verbose:
            print "Including file {0} from package {1}.".format(
                self.fname,
                self.pkg)

        structure.addInclude(self.getPath())
        p = FileParser(self.getPath(), structure)
        p.parse(verbose)


    def getPath(self):
        from roslib.packages import find_resource
        loc = find_resource(self.pkg, self.fname)
        if len(loc) < 1:
            print "Error: {0} cannot be found in {1}.".format(
                self.fname, self.pkg)
            exit(-1)
        if len(loc) > 1:
            print "Warning: more than one {0} exists in {1}.".format(
                self.fname, self.pkg)
        return loc[0]

class RemapDef:
    def __init__(self, content, structure, verbose=False):
        if type(content) is not dict:
            print "Error: remap clause is not a dictionary."
            exit(-1)
        try:
            self.topic = content['from']
            self.to = content['to']
        except KeyError as e:
            print "Error: Missing {0} in {1}.".format(e, content)
            exit(-1)

        if verbose:
            print "Adding a root remap for {0}: {1}.".format(
                self.topic, self.to)

        structure.addRootRemap(self)

class DesireDef:
    def __init__(self, content, structure, verbose=False):
        if type(content) is not dict:
            print "Error: desire clause is not a dictionary: ", content
            exit(-1)
        try:
            self.desire_id = content['id']
            self.desire_type = content['type']
            self.utility = content['utility']
            self.intensity = content['intensity']
        except KeyError as e:
            print "Error: Missing {0} in {1}.".format(e, content)
            exit(-1)
        if 'params' in content:
            self.params = content['params']
        else:
            self.params = ""
        if 'security' in content:
            self.security = bool(content['security'])
        else:
            self.security = False
        # TODO: Add support for expected time.
        if 'expected_time' in content:
            print "Warning: 'expected_time' not supported in desire clauses."

        structure.addDesire(self)

    def generatePy(self):
        d_str = "desire_{0} = Desire()\n".format(self.desire_id)
        d_str += "desire_{0}.id = \"{0}\"\n".format(self.desire_id)
        d_str += "desire_{0}.type = \"{1}\"\n".format(self.desire_id,
            self.desire_type)
        d_str += "desire_{0}.utility = {1}\n".format(self.desire_id,
            self.utility)
        d_str += "desire_{0}.intensity = {1}\n".format(self.desire_id,
            self.intensity)
        d_str += "desire_{0}.params = \"{1}\"\n".format(self.desire_id,
            self.params)
        d_str += "desire_{0}.security = {1}\n".format(self.desire_id,
            self.security)
        d_str += "add_desires([desire_{0}])\n".format(self.desire_id)

        return d_str

class BehaviorPriorityDef:
    def __init__(self, content, structure, verbose=False):
        if type(content) is not dict:
            print "Error: behavior_priority clause is not a dictionary:"
            print content
            exit(-1)

        try:
            name = content['name']
            p = content['value']
        except KeyError as e:
            print "Error: Missing {0} in {1}".format(e, content)
            exit(-1)

        if verbose:
            print "Overriding priority for {0} with {1}".format(name, p)

        if name in structure.behaviors:
            structure.behaviors[name].priority = p
        elif verbose:
            print "Warning: Behavior priority override ignored for {0}".format(
                name)

class ArbitrationTypeDef:
    def __init__(self, content, structure, verbose=False):
        if type(content) is not dict:
            print "Error: arbitration_type clause is not a dictionary:"
            print content
            exit(-1)

        try:
            self.topic = content['topic']
            self.pkg = content['pkg']
            self.node = content['type']
        except KeyError as e:
            print "Error: Missing {0} in {1}".format(e, content)
            exit(-1)

        if verbose:
            print "Overriding default arbitration type for {0}".format(
                self.topic)

        structure.addArbitrationType(self)

class MotivationDef:
    def __init__(self, content, structure, verbose=False):
        if type(content) is not dict:
            print "Error: motivation clause is not a dictionary:"
            print content
            exit(-1)

        try:
            self.name = content['name']
            self.launch = LaunchDef(content['launch'], verbose)
        except KeyError as e:
            print "Error: Missing {0} in {1}".format(e, content)
            exit(-1)

        if 'input' in content:
            self.input = content['input']
        else:
            self.input = []
        if 'output' in content:
            self.output = content['output']
        else:
            self.output = []

        if 'auto_hbba_remap' in content:
            self.auto_hbba_remap = bool(content['auto_hbba_remap'])
        else:
            self.auto_hbba_remap = True

        if self.auto_hbba_remap:
            self.output.extend([
                'add_desires', 
                'remove_desires', 
                'eval_script'])

        if verbose:
            print "Adding motivation module {0}.".format(self.name)

        structure.addMotivation(self)

    def generateXML(self, structure):
        grp = Element("group", attrib={'ns': self.name})
        elems = [grp]
        # Input and output remaps :
        for o in self.output: 
            grp.append(structure.generateRootRemapXML(o))
        for i in self.input: 
            grp.append(structure.generateRootRemapXML(i))

        grp.extend(self.launch.generateXML(structure))

        return elems 
        
class EmoIntensityDef:
    def __init__(self, content, structure, verbose=False):
        if type(content) is not dict:
            print "Error: emo_intensity clause is not a dictionary:"
            print content
            exit(-1)

        try:
            self.name = content['name']
            self.value = content['value']
        except KeyError as e:
            print "Error: missing {0} in {1}".format(e, content)
            exit(-1)

        if verbose:
            print "Adding initial emotion intensity {0}: {1}.".format(
            self.name,
            self.value)
        structure.addEmoIntensity(self)

    def generatePy(self):
        code =  "emo_{0} = EmoIntensity()\n".format(self.name)
        code += "emo_{0}.name = \"{0}\"\n".format(self.name)
        code += "emo_{0}.value = {1}\n".format(self.name, self.value)
        code += "pubEmoIntensity.publish(emo_{0})\n".format(self.name)
        return code

class IntegratedArbitrationDef:
    def __init__(self, content, structure, verbose=False):
        try:
            for t in content:
                structure.addIntegratedArbitration(t)
                if verbose:
                    print "Integrated arbitration for {0}.".format(t)
        except Error as e:
            print "Error: Problem parsing '{0}' as an array.".format(content)
            exit(-1)

class TopicDef:
    def __init__(self, content, structure, verbose=False):
        self.structure = structure
        if type(content) is dict:
            self.name = content.keys()[0]
            if type(content.values()[0]) is dict:
                c = content.values()[0]
                if ('src' in c):
                    self.src = c['src']
                else:
                    self.src = self.name
                if 'filtered' in c:
                    self.filtered = c['filtered']
                else:
                    self.filtered = None
                if 'type' in c:
                    self.filter_type = c['type']
                else:
                    self.filter_type = None
                
            else:
                self.src = content.values()[0]
                self.filtered = None
        else:
            self.name = content
            self.src = content
            self.filtered = None

    def getRootTopicFullName(self):
        return self.structure.getRootTopicFullName(self.src)
    def generateRootRemapXML(self):
        return self.structure.generateRootRemapXML({self.name: self.src})

class CustomScriptDef:
    def __init__(self, content, structure, verbose=False):
        if type(content) is str:
            if verbose:
                print "Adding custom script: \n" + content
            structure.addCustomScript(content)
        else:
            print "Error: custom_script entry is not a string."

####

typemap = {
    'behavior': BehaviorDef,
    'procmodule': ProcModuleDef,
    'strat': StratDef,
    'filtertype': FilterTypeDef,
    'resources': ResourceSetDef,
    'include': IncludeDef,
    'remap': RemapDef,
    'desire': DesireDef,
    'behavior_priority': BehaviorPriorityDef,
    'arbitration_type': ArbitrationTypeDef,
    'integrated_arbitration': IntegratedArbitrationDef,
    'motivation': MotivationDef,
    'emo_intensity': EmoIntensityDef,
    'custom_script': CustomScriptDef
}

