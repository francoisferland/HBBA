from xml.etree.ElementTree import Element, tostring

# {0}: name
# {1}: utility (ResourceUsage string)
# {2}: cost (ResourceUsage array string)
# {3}: dependencies (ResourceUsage array string)
# {3}: source (javascript escaped string)

StratTemplate = """
strat_{0} = Strategy()
strat_{0}.id = '{0}'
strat_{0}.bringup_function = '{0}_bringup'
strat_{0}.bringdown_function = '{0}_bringdown'
strat_{0}.utility = {1}
strat_{0}.cost = {2}
strat_{0}.utility_min = {3}
strat_{0}.source = "{4}"
"""

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
        grp.extend(self.launch.generateXML())
        elems.append(grp)
        for o in self.output:
            # Add output filter, registration script.
            grp.append(Element("node", attrib={
                'name': self.outputFilterName(o),
                'pkg': 'nodelet',
                'type': 'nodelet',
                'args': 
                    "standalone topic_filters/GenericDivisor {0} {1}".format(
                        o, self.outputFilterTopic(o))
                }))
            grp.append(Element("rosparam", attrib={
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
        if not 'name' in content:
            print "Error: procmodule element with no name."
            exit(-1)
        self.name = content['name']

        try:
            self.launch = LaunchDef(content['launch'], verbose)
            self.input = content['input']
        except KeyError as e:
            print "Error: Missing {0} in {1}".format(e, self.name)

        structure.addProcModule(self)

        if verbose:
            print "Emitted ProcModule '{0}'.".format(self.name)

    def createInputFilter(self, name):
        elems = []
        filter_name = "{0}_{1}_filter".format(self.name, name)
        elems.append(Element("node", attrib = {
            'name': filter_name,
            'pkg': 'nodelet',
            'type': 'nodelet',
            'args': 
                "standalone topic_filters/GenericDivisor {0} {1}/{0}".format(
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
        grp.extend(self.launch.generateXML())
        for i in self.input:
            if type(i) != dict:
                elems.extend(self.createInputFilter(i))
            else:
                # TODO: Switch on actual filter type.
                print "Generating for {0}".format(i.keys()[0])
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

class ModuleDef:
    def __init__(self, content, verbose=False):
        self.content = content # TODO!

class ProcStratDef:
    def __init__(self, content, structure, verbose=False):
        if not 'name' in content:
            print "Error: procstrat element with no name."
            exit(-1)
        self.name = content['name']
        try:
            utility_class = content['class']
            utility = content['utility']
            self.utility = CostDef(utility_class, utility)
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
                self.modules.append(ModuleDef(m, verbose))
        except KeyError as e:
            print "Error: Missing {0} in {1}".format(e, self.name)

        structure.addStrategy(self)

    def generatePy(self):
        costs = "["
        l = len(self.costs)
        
        return StratTemplate.format(
            self.name, # TODO!
            self.utility.generatePy(),
            generateCostDefArrayPy(self.costs),
            generateCostDefArrayPy(self.dependencies),
            "JAVASCRIPT")




typemap = {
    'behavior': BehaviorDef,
    'procmodule': ProcModuleDef,
    'procstrat': ProcStratDef
}

