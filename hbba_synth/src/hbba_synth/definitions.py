from xml.etree.ElementTree import Element, tostring

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

def generateRootRemapXML(topic):
    return Element("remap", attrib = {
        'from': topic,
        'to': '/' + topic})

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
    def generateXML(self):
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
        return self.name + "_" + name + "_output_filter"
    def outputFilterTopic(self, name):
        return name + "_out"

    def generateXML(self):
        elems = []
        grp = Element("group", attrib={'ns': self.name})
        # Input remaps, have to go first to affect included nodes:
        for i in self.input:
            grp.append(generateRootRemapXML(i))

        grp.extend(self.launch.generateXML())
        elems.append(grp)
        for o in self.output:
            # Add output filter, registration script.
            grp.append(Element("node", attrib={
                'name': self.outputFilterName(o),
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
                'name': "register_{0}_{1}".format(self.name,
                    self.outputFilterTopic(o)),
                'pkg': 'abtr_priority',
                'type': 'register',
                'args': "{0} {1}/{2}".format(o, self.name,
                    self.outputFilterTopic(o))
                }))
            elems.append(Element("node", attrib={
                'name': "register_{0}".format(self.outputFilterName(o)),
                'pkg': 'topic_filters_manager',
                'type': 'register',
                'args': "{0} {1}/{2}".format(self.outputFilterName(o), 
                    self.name, self.outputFilterName(o))
                }))

        # TODO: Filter registration (out of XML, actually)!

        return elems

class ProcModuleDef:
    def __init__(self, content, structure, verbose=False):
        self.structure = structure

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

    def createInputFilter(self, name):
        elems = []
        filter_name = "{0}_{1}_filter".format(self.name, name)
        filter_type = "GenericDivider"
        self.structure.addFilter(FilterDef(filter_name, filter_type))
        elems.append(Element("node", attrib = {
            'name': filter_name,
            'pkg': 'nodelet',
            'type': 'nodelet',
            'args': 
                "standalone topic_filters/GenericDivider {0} {1}/{0}".format(
                    name, self.name)
            }))
        elems.append(Element("node", attrib={
            'name': "register_{0}".format(filter_name),
            'pkg': 'topic_filters_manager',
            'type': 'register',
            'args': "{0} {1}/{2}".format(filter_name, self.name, filter_name)
            }))
        return elems

    def generateXML(self):
        elems = []
        grp = Element("group", attrib={'ns': self.name})
        elems.append(grp)

        # Output remaps (unfiltered, needed for included nodes.):
        for o in self.output: 
            grp.append(generateRootRemapXML(o))

        grp.extend(self.launch.generateXML())
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
            # TODO: Get output topics in time.
            self.filters = [content + "_output_filter"]

    def generateJSCall(self, func, fname, level):
        return "{0}('{1}', {2});\n".format(func, fname, level)

    def parseFilter(self, f):
        if type(f) is dict:
            fname = self.module_name + "_" + f.keys()[0] + "_filter"
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
            src += self.generateJSCall(ffunc, fname, level)
        return src

    def generateDeactivationJS(self):
        src = ""
        for f in self.filters:
            (fname, ftype, ffunc, level) = self.parseFilter(f)
            src += self.generateJSCall(ffunc, fname, 0)
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
            js_source += "  " + m.generateActivationJS()
        js_source += "}\n"

        js_source += "function {0}(params) {{\n".format(bdn_name)
        for m in self.modules:
            js_source += "  " + m.generateDeactivationJS()
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

typemap = {
    'behavior': BehaviorDef,
    'procmodule': ProcModuleDef,
    'strat': StratDef,
    'filtertype': FilterTypeDef,
    'resources': ResourceSetDef
}

