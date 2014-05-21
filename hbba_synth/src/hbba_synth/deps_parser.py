from structure import *
import yaml
from definitions import *

# Similar to DepsParser, but only gathers include files.
class DepsParser:
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

        from definitions import typemap
        for conf_name, conf_content in doc.iteritems():
            if (conf_content is None):
                return
            for elem in conf_content:
                key = elem.keys()[0]
                if key == "include":
                    content = elem.values()[0]
                    e = typemap[key](content, self.structure, verbose, DepsParser)
                else:
                    if (verbose):
                        print "Warning: unrecognized element {0}".format(key)


        if (verbose):
            print "Parsing {0} done.".format(self.filename)
        return

