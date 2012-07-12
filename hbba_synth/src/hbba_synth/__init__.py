import roslib; roslib.load_manifest("hbba_synth")
from structure import *
from file_parser import FileParser
from optparse import OptionParser
import os

def main():
    opt_parser = OptionParser(usage="usage: %prog [options] file1 [file2, ...]")
    opt_parser.add_option("-o", "--output", dest="basename",
            default="./hbba_struct", 
            help="defines the base name for the produced files (default: %default)", 
            metavar="DIRECTORY")
    opt_parser.add_option("-a", "--generate-arbitration", action="store_true",
            dest="generate_arbitration", default=True,
            help="generic arbitration nodes are generated for each behavior output topic (default: %default)")
    opt_parser.add_option("-b", "--base_nodes", action="store_true",
            dest="base_nodes", default=True,
            help="add base HBBA nodes such as iw and script_engine (default: %default)")
    opt_parser.add_option("-d", "--debug", action="store_true",
            dest="debug", default=False,
            help="debug mode for HBBA base nodes (default: %default)")
    opt_parser.add_option("-p", "--pretty", action="store_true",
            dest="pretty", default=False,
            help="more human-readable XML output")
    opt_parser.add_option("-v", "--verbose", action="store_true",
            dest="verbose", default=False,
            help="verbose output")
    
    (opts, args) = opt_parser.parse_args()

    if len(args) < 1:
        opt_parser.error("Requires at least one input file")
        exit(-1)

    structure = Structure()

    for f in args:
        p = FileParser(f, structure)
        p.parse(opts.verbose)

    structure.generate(opts.basename, opts)


