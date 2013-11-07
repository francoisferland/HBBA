# The intention of the robot, described as a set of strategies to enable/disable
string[] strategies    # The strategies' IDs.
string[] desires       # The corresponding desires' IDs, if applicable.
string[] desire_types  # The corresponding desires' types, if applicable.
int64[]  intensity     # The corresponding desires' intensity, if applicable.

string[] params        # The corresponding desires' parameters, if applicable.
bool[]   enabled       # The strategies' states.
time     stamp         # The time of publication.

