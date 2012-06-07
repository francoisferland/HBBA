from definitions import *
from xml.etree.ElementTree import ElementTree, Element, tostring 
from xml.dom import minidom

class Structure:
    def __init__(self):
        self.behaviors = []
        self.procmodules = []
        self.strategies = []
        self.filters = []

    def addBehavior(self, b):
        self.behaviors.append(b)

    def addProcModule(self, p):
        self.procmodules.append(p)

    def addStrategy(self, s):
        self.strategies.append(s)

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
        # TODO: Pre-start template (filter activation methods ?).
        for s in self.strategies:
            pyscript += s.generatePy()

        if verbose:
            print "Generated Python script:\n"
            print pyscript

        # TODO: Write the file.
