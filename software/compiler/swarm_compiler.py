#!/usr/bin/python

# A compiler/planner/action selector for turning user gesture specifications into a set of programs
# for swarm robots. 

# It looks like the first pass of the compiler would convert the user gestures in whatever format into some constant
# JSON format, which I guess is my Intermediate Representation, and then the next pass would expand that into the 
# set of avaialable primitives. The availability of primitives would be based on metaparameters to the compiler
# that describe the abilities of the system, and implementations of primitives would be based on the parameters as well.
# So e.g. finding an object might be a regular grid search, or brownian motion + beaconing, etc. 
# This heuristic system would be based on work in the literature, so that e.g. traveling to a specfic point may only be
# avaialable if global localization is available, or forming a circle may depend on various levels of robot sensing ability.
# So I'll need implementations of each primitive, as e.g. GCPR rules, and a heuristic table for retreiving them
# So I'll need a list of primitives and a set of taxonomies of swarm capabilities
# Given a list of primitives, come up with an implmentation per combination of swarm abilities
# or possibly implement as needed vs. determining it's not possible
# Can have some complicated dependencies, like local shape formation may depend on or be limited by the ability to form a 
# local coordinate system, which may in turn be limited by the ability to communicate or sense other robots. 
# It may be possible to specify a logical representation of the depenencies, which would then be used to determine whether 
# it is possible to generate the program. 

# If there's a need to specify which robots get which programs, that's informed by user gestures, so it should happen as 
# part of the IR generation, but before the primitive availability heuristics get used. 
# That way also leaves it open to highly heterogeneous swarms, as they may end up with different primitive availabilities.

def loadIR(infile):
	pass

def loadSwarmSpec(infile):
	pass

def loadCapMap(infile):
	pass


if __name__=="__main__":
	#Load the file describing the desired task
	loadIR("./test_IR.json")
	#Load the file describing the swarm
	loadSwarmSpec("./test_spec.json")
	#Load the file desribing the mapping of swarm capabilities to GCPR implementations
	loadCapMap("./test_capacities_map.json")
	
	print "So implement the compiler!"