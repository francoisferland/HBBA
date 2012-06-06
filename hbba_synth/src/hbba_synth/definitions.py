from xml.etree.ElementTree import Element, tostring

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



def outputFilterName(name):
    return name + "_output_filter"
def outputFilterTopic(name):
    return name + "_out"

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

    def generateXML(self):
        elems = []
        grp = Element("group", attrib={'ns': self.name})
        grp.extend(self.launch.generateXML())
        elems.append(grp)
        for o in self.output:
            # Add output filter, registration script.
            grp.append(Element("node", attrib={
                'name': outputFilterName(o),
                'pkg': 'nodelet',
                'type': 'nodelet',
                'args': 
                    "standalone topic_filters/GenericDivisor {0} {1}".format(
                        o, outputFilterTopic(o))
                }))
            grp.append(Element("rosparam", attrib={
                'name': outputFilterTopic(o) + "/abtr_priority",
                'value': str(self.priority)
                }))
            elems.append(Element("node", attrib={
                'name': "register_{0}_{1}".format(self.name,
                    outputFilterTopic(o)),
                'pkg': 'abtr_priority',
                'type': 'register',
                'args': "{0} {1}/{2}".format(o, self.name,
                    outputFilterTopic(o))
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
        return Element("node", attrib = {
            'name': "{0}_{1}_filter".format(self.name, name),
            'pkg': 'nodelet',
            'type': 'nodelet',
            'args': 
                "standalone topic_filters/GenericDivisor {0} {1}/{0}".format(
                    name, self.name)
            })

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
                elems.append(self.createInputFilter(i.keys()[0]))

        # TODO: Filter registration (out of XML, actually)!

        return elems 

typemap = {
    'behavior': BehaviorDef,
    'procmodule': ProcModuleDef
}

