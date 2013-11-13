#!/usr/bin/env python

import roslib; roslib.load_manifest("iw")
import rospy
import readline

from hbba_msgs.msg import *
from hbba_msgs.srv import *

# Syntax: add type [intensity] [utility] [params]
# Anything passed [utility] is packed into the params string.
# Syntax: del id1 [id2] ...

def gatherClasses():
    strats = rospy.get_param("hbba/solver_model/strategies")
    dtypes = [s['class'] for s in strats if 'class' in s]
    return dtypes

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

        self.scl_add = rospy.ServiceProxy("hbba/add_desires",    AddDesires)
        self.scl_del = rospy.ServiceProxy("hbba/remove_desires", RemoveDesires)

        self.commands = ['add', 'del', 'help', 'quit']
        self.desire_classes = gatherClasses()

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
                elif (t.lower() == "help"):
                    state = 6
                    cmd = "help"
                    opts = []
                    continue
                else:
                    opts = self.commands
                    continue

            elif (state == 1):    # Add
                if (t in self.desire_classes):
                    state = 2   # AddType
                    opts  = []
                    desire.type = str(t)
                    continue
            elif (state == 2):
                state = 3 # AddTypeInt
                desire.intensity = int(t)
                continue
            elif (state == 3):
                state = 4 # AddTypeIntParams 
                desire.utility = int(t)
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
            elif (cmd == "del"):
                if (len(del_ids) > 0):
                    self.delDesires(del_ids)
                else:
                    print "Missing desire id(s) to delete."
            elif (cmd == "help"):
                self.printSyntax()

        return opts

    def addDesire(self, desire):
        desire.id = "iw_cons_" + desire.type + "_" + str(self.current_id)
        self.ids.append(desire.id)
        self.current_id += 1
        print desire
        self.scl_add.call([desire])

    def delDesires(self, del_ids):
        print "Deleting ", del_ids, "..."
        for d in del_ids:
            if (d in self.ids):
                self.ids.remove(d)
        self.scl_del.call(del_ids)

    def printSyntax(self):
        print "Syntax :"
        print "  add Class [intensity] [utility] [params]"
        print "  del id1 [id2] ..."
        print ""
        print "For the add command, anything passed the [utility] arg is"
        print "appended in a single params string."

    def loop(self):
        print "Type 'help' for syntax."
        line = ""
        ok = True
        while ok:
            line = raw_input("> ")
            ok = self.analyse(line)

    def analyse(self, line):
        self.parseLine(line, final=True)

        line_tokens = line.split()
        if (len(line_tokens) > 0) and (line_tokens[0].lower() == "quit"):
            print "Quitting..."
            return False

        return True

rospy.init_node("iw_console", anonymous=True)

shell = Shell()
shell.loop()

