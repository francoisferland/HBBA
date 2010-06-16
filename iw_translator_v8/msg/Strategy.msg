string id				# This strategy's id.
string bringup			# Bringup script source.
string bringdown		# Bringdown script source.
ResourceUsage[] utility	# Effect on information classes.
ResourceUsage[] cost	# Usage cost description.

# The bringup script need to define a method with this signature :
#    void bringup(params)
# Where params is a JSON/YAML formatted string coming from the selected
# intention/desire.

