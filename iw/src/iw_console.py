#!/usr/bin/env python

import roslib; roslib.load_manifest("iw")
import rospy
import readline

from hbba_msgs.msg import *
from hbba_msgs.srv import *

# Syntax: add type [intensity] [utility] [params]
# Anything passed [utility] is packed into the params string.
# Syntax: del id1 [id2] ...

class Completer:
    def __init__(self, shell):
        self.matches = []
        self.shell = shell

    def completer(self, text, state):
        if (state == 0):
            opts = shell.parseLine(readline.get_line_buffer())
            self.matches = [s + " "
                            for s in opts
                            if s and s.startswith(text)]

        if (state < len(self.matches)):
            return self.matches[state] 

        return [' '] 

class Shell:
    def __init__(self):
        self.comp = Completer(self)
        readline.set_completer(self.comp.completer)
        readline.parse_and_bind("tab: complete")

        self.commands = ['add', 'del']
        self.desire_classes = ['LocateLegs', 'LocateFaces']

        self.current_id = 0
        self.ids = []

    def parseLine(self, text, final = False):
        tokens = text.split()

        state  = 0 # Init
        cmd    = ""
        opts   = []

        desire  = Desire()
        del_ids = []

        for t in tokens:
            if (state == 0):    # Init
                if (t.lower() == "add"):
                    state = 1 # Add
                    cmd = "add"
                    opts = self.desire_classes
                    continue
                elif (t.lower() == "del"):
                    state = 5
                    cmd = "del"
                    opts = self.ids
                    continue
                opts = self.commands
                continue
            elif (state == 1):    # Add
                if (t in self.desire_classes):
                    state = 2   # AddType
                    opts  = []
                    desire.type = t
                    continue
            elif (state == 2):
                state = 3 # AddTypeInt
                desire.intensity = t
                continue
            elif (state == 3):
                state = 4 # AddTypeIntParams 
                desire.utility = t
                continue
            elif (state == 4):
                desire.params += t
                continue
            elif (state == 5):
                del_ids.append(t)

        if (final):
            if (cmd == "add"):
                if (state > 1):
                    self.addDesire(desire)
                else:
                    print "Missing critical desire type info."
            if (cmd == "del"):
                if (len(del_ids) > 0):
                    self.delDesires(del_ids)
                else:
                    print "Missing desire id(s) to delete."

        return opts

    def addDesire(self, desire):
        desire.id = "iw_cons_" + desire.type + "_" + str(self.current_id)
        self.ids.append(desire.id)
        self.current_id += 1
        print desire

    def delDesires(self, del_ids):
        print "Deleting ", del_ids, "..."
        for d in del_ids:
            if (d in self.ids):
                self.ids.remove(d)

    def loop(self):
        line = ""
        ok = True
        while ok:
            line = raw_input("> ")
            ok = self.analyse(line)

    def analyse(self, line):
        print line
        self.parseLine(line, final=True)

        if line == "quit":
            return False

        return True


shell = Shell()
shell.loop()

