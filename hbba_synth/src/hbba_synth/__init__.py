import roslib; roslib.load_manifest("hbba_synth")
from structure import *
from file_parser import FileParser
from optparse import OptionParser
import os

def main():
    opt_parser = OptionParser(usage="usage: %prog [options] file1 [file2, ...]")
    opt_parser.add_option("-o", "--output", dest="directory", default="./", 
            help="write output files to DIRECTORY (default: %default)", 
            metavar="DIRECTORY")
    opt_parser.add_option("-n", "--name", dest="basename",
            default="hbba_struct",
            help="generated files base name (default: %default)")
    opt_parser.add_option("-v", "--verbose", action="store_true",
            dest="verbose", default=False,
            help="verbose output")
    
    (opts, args) = opt_parser.parse_args()

    if len(args) < 1:
        parser.error("Incorrect number of files")
        exit(-1)

    structure = Structure()

    for f in args:
        p = FileParser(f, structure)
        p.parse(opts.verbose)

    if (opts.verbose):
        print "Generating XML..."

    basepath = os.path.abspath(opts.directory) + "/" + opts.basename
    structure.generate(basepath, opts.verbose)


