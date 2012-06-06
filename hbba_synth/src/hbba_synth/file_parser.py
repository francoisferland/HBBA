from structure import *
import yaml

class LaunchDef:
    def __init__(self, content, verbose=False):
        self.pkg = content['pkg']
        self.name = content['name']
        if verbose:
            print "Emitted launch element for {0}.".format(self.name)

class BehaviorDef:
    def __init__(self, content, structure, verbose=False):
        self.name = content['name']
        self.launch = LaunchDef(content['launch'], verbose)
        if verbose:
            print "Emitted Behavior '{0}'.".format(self.name)

class ProcModuleDef:
    def __init__(self, content, structure, verbose=False):
        self.name = content['name']
        self.launch = LaunchDef(content['launch'], verbose)
        if verbose:
            print "Emitted ProcModule '{0}'.".format(self.name)

typemap = {
    'behavior': BehaviorDef,
    'procmodule': ProcModuleDef
}

class FileParser:
    def __init__(self, filename, structure):
        self.structure = structure
        self.filename = filename

    def parse(self, verbose):
        if (verbose):
            print "Parsing {0} ...".format(self.filename)
        try:
            stream = file(self.filename, 'r')
        except IOError as (errno, errstr):
            print "Cannot open {0}, reason: {1}".format(self.filename, errstr)
            exit(-1)

        try:
            doc = yaml.load(stream)
        except yaml.YAMLError, exc:
            print "Cannot parse {0} as YAML, reason: {1}".format(self.filename,
                exc)
            exit(-1)
        
        if (verbose):
            print "Full document: \n {0}".format(doc)

        # The parser look for every main elements and hand it to other analysers 
        # for each recognized element.
        for key, content in doc.iteritems():
            if key in typemap:
                e = typemap[key](content, self.structure, verbose)
            else:
                if (verbose):
                    print "Warning: unrecognized element {0}".format(key)


        if (verbose):
            print "Parsing {0} done.".format(self.filename)
        return

