from definitions import *
from xml.etree.ElementTree import ElementTree, Element, tostring 
from xml.dom import minidom

class Structure:
    def __init__(self):
        self.behaviors = []
        self.perceptmodules = []
        self.strategies = []
        self.filters = []

    def addBehavior(self, b):
        self.behaviors.append(b)

    def generate(self, basepath, verbose):
        launch_elem = Element("launch")
        for b in self.behaviors:
            elems = b.generateXML()
            launch_elem.extend(elems)

        launch_tree = ElementTree(launch_elem)
        if verbose:
            print "Generated XML:\n"
            rough = tostring(launch_elem)
            reparsed = minidom.parseString(rough)
            print reparsed.toprettyxml(indent="  ")


        launch_tree.write(basepath + ".launch")


