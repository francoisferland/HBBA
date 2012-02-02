string id					# This strategy's id.
string source           	# This strategy's script source (javascript)
string bringup_function		# Bringup function name. Can be empty.
string bringdown_function	# Bringdown function name. Can be empty.
ResourceUsage utility		# Effect on information classes.
ResourceUsage[] cost		# Usage cost description.
ResourceUsage[] utility_min #Utility needed on another class.

# The bringup and bringdown functions need to be declared in your script's
# source and follow this signature :
#    function bring[up|down](params) { ... }
# Where params is a JSON-formatted object coming from the selected
# intention/desire, e.g., for a goto, the translator could call your function
# like this, without quotes surrounding the JSON:
#    goto_bringup({"frame_id": "/odom", "x" : 0.0, "y" : 0.0, "t" : 0.0})
# Your method can then directly pull out data from this structure.
# The selected intention just need to format its parameters correctly.
