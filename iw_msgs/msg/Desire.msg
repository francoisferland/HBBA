# A desire to add to the IW.

string id		# Desire instance identification string.

string type		# Desire resource class, e.g. "goto", "detect_person".
int32 utility	# Mimimum resource utility needed, varies from class to
				# class.

string params	# YAML parameters string passed to the bringup script of the
				# selected strategy. Varies from class to class. For instance, 
				# a goto resource requirement might define a "target" 
				# parameter as a global {x,y} location.
				
				# NOTE: Need a design review, we can't know for sure what
				# the selected strategy supports / it might get ignored
				# since a strategy isn't necessarily aware of the global 
				# intent. But, since it's a YAML string and basically
				# dynamically typed, we probably can live with that.

