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
            self.launch = LaunchDef(content['launch'], verbose)
            self.output = content['output']
            self.priority = content['priority']
        except KeyError as e:
            print "Error: Missing {0} in {1}".format(e, self.name)
            exit(-1)
        if 'input' in content:
            self.input = content['input']
        else:
            self.input = []

        structure.addBehavior(self)
        if verbose:
            print "Emitted Behavior '{0}'.".format(self.name)

    def outputFilterName(self, name):
        return self.name + "/" + name + "_output_filter"
    def outputFilterNodeName(self, name):
        return name + "_output_filter"
    def outputFilterTopic(self, name):
        return name + "_out"

    def generateXML(self, structure):
        elems = []
        grp = Element("group", attrib={'ns': self.name})
        # Input remaps, have to go first to affect included nodes:
        for i in self.input:
            grp.append(structure.generateRootRemapXML(i))

        grp.extend(self.launch.generateXML(structure))
        elems.append(grp)
        for o in self.output:
            root_topic = structure.getRootTopicFullName(o)
            # Add output filter, registration script.
            grp.append(Element("node", attrib={
                'name': self.outputFilterNodeName(o),
                'pkg': 'nodelet',
                'type': 'nodelet',
                'args': 
                    "standalone topic_filters/GenericDivider {0} {1}".format(
                        o, self.outputFilterTopic(o))
                }))
            grp.append(Element("param", attrib={
                'name': self.outputFilterTopic(o) + "/abtr_priority",
                'value': str(self.priority)
                }))
            elems.append(Element("node", attrib={
                'name': "register_{0}_{1}_{2}".format(
                    self.name,
                    self.outputFilterTopic(o),
                    uniqueName()),
                'pkg': 'abtr_priority',
                'type': 'register',
                'args': "{0} {1}/{2}".format(root_topic, self.name,
                    self.outputFilterTopic(o))
                }))
            elems.append(Element("node", attrib={
                'name': "register_{0}_{1}".format(
                    self.outputFilterNodeName(o),
                    uniqueName()), 
                'pkg': 'topic_filters_manager',
                'type': 'register',
                'args': "{0} GenericDivider".format(
                    self.outputFilterName(o))
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
            self.input = content['input']
        except KeyError as e:
            print "Error: Missing {0} in {1}".format(e, self.name)
        if 'output' in content:
            self.output = content['output']
        else:
            self.output = []

        structure.addProcModule(self)

        if verbose:
            print "Emitted ProcModule '{0}'.".format(self.name)

    def createInputFilter(self, topic):
        elems = []
        filter_name = "{0}_{1}_filter".format(self.name, topic)
        node_name = "{0}_{1}_filter".format(self.name, topic)
        filter_type = "GenericDivider"
        if self.verbose:
            print "Adding filter {0}".format(filter_name)
        self.structure.addFilter(FilterDef(filter_name, filter_type))
        elems.append(Element("node", attrib = {
            'name': node_name,
            'pkg': 'nodelet',
            'type': 'nodelet',
            'args': 
                "standalone topic_filters/{0} {1} {2}".format(
                    filter_type, topic, filter_name)
            }))
        elems.append(Element("node", attrib={
            'name': "register_{0}".format(node_name),
            'pkg': 'topic_filters_manager',
            'type': 'register',
            'args': "{0} {1}".format(filter_name, filter_type)
            }))
        return elems

    def generateXML(self, structure):
        elems = []
        grp = Element("group", attrib={'ns': self.name})
        elems.append(grp)

        # Output remaps (unfiltered, needed for included nodes.):
        for o in self.output: 
            grp.append(structure.generateRootRemapXML(o))

        grp.extend(self.launch.generateXML(structure))
        for i in self.input:
            if type(i) != dict:
                elems.extend(self.createInputFilter(i))
            else:
                # TODO: Switch on actual filter type.
                elems.extend(self.createInputFilter(i.keys()[0]))


        return elems 

class CostDef:
    def __init__(self, key, val, verbose=False):
        self.name = key
        self.value = val
    
    def generatePy(self):
        return "ResourceUsage(\"{0}\", {1})".format(self.name, self.value)

def generateCostDefArrayPy(costs):
    cstr = "["
    l = len(costs)
    for i in range(0,l-2):
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
            print "Error: procstrat element with no name."
            exit(-1)
        self.name = content['name']
        self.structure = structure
        try:
            self.utility_class = content['class']
            utility = content['utility']
            self.utility = CostDef(self.utility_class, utility)
            self.costs = []
            if ('costs' in content):
                for key,val in content['costs'].iteritems():
                    self.costs.append(CostDef(key, val, verbose))
            self.dependencies = []
            if ('dependencies' in content):
                for d in content['dependencies']:
                    self.dependencies.append(CostDef(d, verbose))
            self.modules = []
            for m in content['modules']:
                self.modules.append(ModuleLinkDef(m, self.structure, verbose))
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
            print "Error: behavior_priority clause is not a dictionary: "
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
        else:
            print "Warning: Behavior priority override ignored for {0}".format(
                name)

typemap = {
    'behavior': BehaviorDef,
    'procmodule': ProcModuleDef,
    'strat': StratDef,
    'filtertype': FilterTypeDef,
    'resources': ResourceSetDef,
    'include': IncludeDef,
    'remap': RemapDef,
    'desire': DesireDef,
    'behavior_priority': BehaviorPriorityDef
}

