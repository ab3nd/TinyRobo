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

# After working through the lists of primives for swarms in McLurkin 2004, and Nagpal 2003, it seems like pretty much anything
# can be implemented in terms of local communication. 

# If there's a need to specify which robots get which programs, that's informed by user gestures, so it should happen as 
# part of the IR generation, but before the primitive availability heuristics get used. 
# That way also leaves it open to highly heterogeneous swarms, as they may end up with different primitive availabilities.

import uuid


import rospy


# class RobotInterface(object):

# 	def __init__(self):
# 		pass

# 	def getAvailableRobots(self):
# 		#Return a list of the available robots
# 		pass

# 	def selectRobots(self):
# 		#map a UI gesture onto a set of robots
# 		pass

# 	def 

#Convert a command into a GCPR expression of that command
gcprLookup = {
	"SelectAll": "Foo selectall",
	"DisperseMax" : "Bar dispersemax",
	"MoveTo" : "moveto {0} {1}"
}

#TODO this is probably overly simplistic
def lookupGCPR(command):
	if hasParams(command):
		command, paramList = getList(command)
		return gcprLookup[command].format(*paramList)
	return gcprLookup[command]

def deployProg(robot, fname):
	print "Would deploy program in {0} to robot {1}".format(fname, robot)
	
#Converts a command of the form CommandName[int,int,int,int] into a list of ints
def getList(command):
	return command.split("[")[0], [int(x) for x in command.split("[")[1][:-1].split(",")]

def hasParams(command):
	#Just check if it has an open bracket in it
	return (command.find("[") > 0)

if __name__=="__main__":
	#Do the simplest thing that could possibly work
	
	#The list of known robots gets populated somehow
	knownRobots = [1,2,3,4,5,6,7,8,9]

	#command sequence for prototyping is select all, disperse
	commandSeq = ["SelectAll", "DisperseMax"]

	#command sequence that selects a group and moves them to a point
	#then selects another group and moves to a different point
	commandSeq = ["SelectGroup[1,2,4,6,8,9]", "MoveTo[132,23]", "SelectGroup[3,5,7,8]", "MoveTo[107,98]"]

	#The commands can be divided into meta-commands, which e.g. affect
	#the state of this code, and commands that are actually run on the robots
	metaCommands = ["SelectAll", "SelectGroup"] 

	#Storage of mapping of groups to programs
	groupProgs = []

	#Group of robots to get a command and the command they get
	commandBots = None
	gcprFile = None

	#Assumes that commands are ordered
	for command in commandSeq:
		
		cmdPrefix, args = getList(command)

		if cmdPrefix in metaCommands:
			#This command influences the state of this program
			if command == "SelectAll":
				commandBots = knownRobots
				#Generate the program file name
				outputProgFile = "{0}.gcpr".format(uuid.uuid4())
				gcprFile = open(outputProgFile, 'w')
				groupProgs.append([commandBots, outputProgFile])
			if command.startswith("SelectGroup"):
				#Parse the command to get the group members
				command, commandBots = getList(command)
				#Generate the program file name
				outputProgFile = "{0}.gcpr".format(uuid.uuid4())
				gcprFile = open(outputProgFile, 'w')
				groupProgs.append([commandBots, outputProgFile])
		else:
			#This is a command to run on a robot
			gcprExpr = lookupGCPR(command)
			gcprFile.write(gcprExpr)
	

	#Program the GCPR programs to the robots
	for commandGroup, file in groupProgs:
		for robot in commandGroup:
			deployProg(robot, file)

