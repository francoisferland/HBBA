# A desire to add to the IW.

string  id           # Desire instance identification string.

string  type         # Desire utility class, e.g. "goto", "detect_person".
int64   utility      # Mimimum utility needed, varies from class to
                     # class.

int64   intensity     # The intensity value of this desire. Used for 
                      # priorization between desires in a common utility class
                      # and strategy selection when resources are scarce.
string  params        # YAML parameters string passed to the bringup script of 
                      # the selected strategy. Varies from class to class. For 
                      # instance, a goto resource requirement might define a 
                      # "target" parameter as a global {x,y} location.
                      # NOTE: Need a design review, we can't know for sure what
                      # the selected strategy supports / it might get ignored
                      # since a strategy isn't necessarily aware of the global 
                      # intent. But, since it's a YAML string and basically
                      # dynamically typed, we probably can live with that.

# Fields related to the emotional subsystem / IW modulation
bool    security      # Indicates a desire linked to emergency/security issues.
time    expected_time # Expected time of a desire's satisfaction. Default is 0.
