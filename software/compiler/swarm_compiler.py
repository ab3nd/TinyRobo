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

#State machine for Python, install with 'sudo pip install transitions' (on Ubuntu at least)
#If you're not on Ubuntu, https://github.com/pytransitions/transitions
from transitions import Machine, MachineError

#Core of the translator, gets sequences of user input gestures and attempts to convert them to a 
#program to run on the robot. The language is based on Mark J. Micire. PhD Thesis: Multi-Touch 
# Interaction for Robot Command and Control. University of Massachusetts Lowell, Lowell, MA. December 2010. 
#There are extensions for handling objects in the world (box-pushing) and formations of robots
class GestureTranslator(object):
	states = ['start', 'subject', 'verb', 'predicate', 'end']

	def __init__(self):
		#A list of the IDs of the robots that the command will be applied to
		self.robots = []
		#Not sure about the type here yet, but the verb of the command gets passed in with some gestures
		self.verb = []

		self.machine = Machine(model = self, states = GestureTranslator.states, initial="start", send_event=True)

		##### Mark's basic language ####
		#Transitions based on gestures as they arrive
		#Selection gestures first, tapping a single robot or lassoing a set of robots
		self.machine.add_transition(trigger='tap_robot', source='start', dest='subject', before="addRobots")
		self.machine.add_transition(trigger='lasso_robots', source='start', dest='subject', before="addRobots")

		#Special case of selection, tap-and-drag a robot to select it and set path
		self.machine.add_transition(trigger='drag_robot', source='start', dest='verb', before="addRobots")

		#Adding robots to the gesture
		self.machine.add_transition(trigger='tap_robot', source='subject', dest='subject', before="addRobots")

		#Double-tap on a destination to finish command
		self.machine.add_transition(trigger='doubletap_ground', source='subject', dest='predicate')

		#Tap to set waypoint or drag to set path
		self.machine.add_transition(trigger='tap_ground', source='subject', dest='verb')
		self.machine.add_transition(trigger='drag_ground', source='subject', dest='verb')

		#Tap or lasso robot while in verb state starts next command with new (just-tapped) robot
		self.machine.add_transition(trigger='tap_robot', source='verb', dest="subject", before="addRobots")
		self.machine.add_transition(trigger='lasso_robots', source='verb', dest="subject", before="addRobots")

		#Add waypoints to path by tapping or dragging
		self.machine.add_transition(trigger='tap_ground', source='verb', dest='verb', before="addCommand")
		self.machine.add_transition(trigger='drag_ground', source='verb', dest='verb', before="addCommand")

		#Explicit end-of-command gesture
		self.machine.add_transition(trigger='doubletap_ground', source='verb', dest='predicate')


		##### Extensions for dealing with objects #####

		#TODO can have hooks 'on_enter_<statename>' and 'on_exit_<statename>' that will
		#fire at the appropriate time to do stuff, e.g. generate code

		#TODO probably needs transitions back to start to reject invalid sequences?
		#State machine will fire a transitions.core.MachineError if an invalid transition
		#is called, but what to do in that case is unclear. Dump the invalid command?
		#transitions adds a to_<state>() that can be used to return to start.

		#TODO this doesn't deal with objects in the world

		#TODO this doesn't deal with formations (lines, boxes)

		#TODO this doesn't deal with patrols

		#Add the callback for finishing the command to the machine
		self.machine.on_enter_predicate('finish_command')

	#When we enter the predicate state, it's time to actually do the command
	def finish_command(self, event):
		command = event.kwargs.get("command", None)
		if command is not None:
			self.verb.extend(command)
			print "Added {0}, now have {1}".format(command, self.verb)

		print "Entered predicate, time to do the thing! {0} -> {1}".format(self.robots, self.verb)
		#TODO this is where we deploy the action to the robots, and THEN clean out the lists
		self.robots = []
		self.verb = []

	def addRobots(self, event):
		robots = event.kwargs.get("robots", None)
		if robots is not None:
			self.robots.extend(robots)
			print "Added {0}, now have {1}".format(robots, self.robots)

	#TODO the command is probably more complex than just a list of points
	def addCommand(self, command=[]):
		command = event.kwargs.get("command", None)
		if command is not None:
			self.verb.extend(command)
			print "Added {0}, now have {1}".format(command, self.verb)

#TODO define a receiver for (ROS?) messages containing the gestures as they arrive

def testTranslator():
	#Get a translator instance 
	tr = GestureTranslator()

	########### Positive examples (should generate code) ###########
	#Tap a single robot, then double-tap a location ("This robot, go here")
	tr.to_start()
	tr.tap_robot(robots=[1])
	tr.doubletap_ground(command=['p1'])
	assert(tr.is_predicate())
	print "Tapping a robot worked"
	print "---"

	#Drag a robot to a location ("This robot, go here")
	tr.to_start()
	tr.drag_robot(robots=[1])
	assert(tr.is_verb()) #needs double-tap to end
	tr.doubletap_ground(command=['p1'])
	assert(tr.is_predicate())
	print "Dragging a robot worked"
	print "---"

	#Tap a sequence of robots, then drag a path ("These robots, go here")
	tr.to_start()
	tr.tap_robot(robots=[1])
	tr.tap_robot(robots=[2])
	tr.tap_robot(robots=[3])
	tr.drag_ground(command=['p1', 'p2', 'p3'])
	assert(tr.is_verb()) #needs double-tap to end
	tr.doubletap_ground(command=['p4'])
	assert(tr.is_predicate())
	print "Tapping a bunch of robots worked"
	print "---"

	#Lasso a set of robots, then drag a path ("These robots, go here")
	tr.to_start()
	tr.lasso_robots(robots=[1,2,3])
	tr.drag_ground(command=['p1', 'p2', 'p3'])
	assert(tr.is_verb()) #needs double-tap to end
	tr.doubletap_ground(command=['p4'])
	assert(tr.is_predicate())
	print "Lassoing a bunch of robots worked"
	print "---"

	#Tap a robot, then drag a path, then tap a robot, then drag a path
	#("First robot go here, second robot go there")
	tr.to_start()
	tr.tap_robot(robots=[1])
	tr.drag_ground(command=['p1', 'p2', 'p3'])
	assert(tr.is_verb())
	tr.tap_robot(robots=[2]) #This starts a new command on a new robot
	tr.drag_ground(command=['p4', 'p5', 'p6'])
	assert(tr.is_verb()) #needs double-tap to end
	tr.doubletap_ground(command=['p7'])
	assert(tr.is_predicate())
	print "Tapping a robot and then dragging a robot worked"
	print "---"

	#Tap a robot, then drag a path, then lasso a set of robots, then drag a path
	#("First robot go here, other robots go there")
	tr.to_start()
	tr.tap_robot(robots=[1])
	tr.drag_ground(command=['p1', 'p2', 'p3'])
	assert(tr.is_verb())
	tr.lasso_robots(robots=[2,3,4])
	tr.drag_ground(command=['p4', 'p5', 'p6'])
	assert(tr.is_verb()) #needs double-tap to end
	tr.doubletap_ground(command=['p7'])
	assert(tr.is_predicate())
	print "Dragging a robot and then lassoing some other robots worked"
	print "---"

	#TODO add positive cases for objects, formations, and patrols

	########### Negative examples (should throw exceptions) ###########

	#Drag ground without selecting a robot
	try:
		tr.to_start()
		tr.drag_ground(command=['p1', 'p2', 'p3'])
	except MachineError:
		print "Got expected exception, dragging ground is not valid in start state"

	#Doubletap on nothing, no robot selected
	try:
		tr.to_start()
		tr.doubletap_ground(command=['p1'])
	except MachineError:
		print "Got expected exception, double-tapping ground is not valid in start state"

	#Tap ground with no selected robot
	try:
		tr.to_start()
		tr.tap_ground(command=['p1'])
	except MachineError:
		print "Got expected exception, tapping ground is not valid in start state"

	#TODO Tap selected robot after issuing command (requires idea of selection vs. non-selection)
	#TODO this may not be an error, could be selecting to cancel command
	
	#Note that double-tapping a robot after selecting it isn't an 
	#error, but means "This robot, go where you are". Similarly, selecting another
	#robot as the end of a motion just means "Selected robot, go where this other robot is"

if __name__ == "__main__":
	testTranslator()

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

# #Convert a command into a GCPR expression of that command
# gcprLookup = {
# 	"SelectAll": "Foo selectall",
# 	"DisperseMax" : "Bar dispersemax",
# 	"MoveTo" : "moveto {0} {1}"
# }

# #TODO this is probably overly simplistic
# def lookupGCPR(command):
# 	if hasParams(command):
# 		command, paramList = getList(command)
# 		return gcprLookup[command].format(*paramList)
# 	return gcprLookup[command]

# def deployProg(robot, fname):
# 	print "Would deploy program in {0} to robot {1}".format(fname, robot)
	
# #Converts a command of the form CommandName[int,int,int,int] into a list of ints
# def getList(command):
# 	return command.split("[")[0], [int(x) for x in command.split("[")[1][:-1].split(",")]

# def hasParams(command):
# 	#Just check if it has an open bracket in it
# 	return (command.find("[") > 0)

# if __name__=="__main__":
# 	#Do the simplest thing that could possibly work
	
# 	#The list of known robots gets populated somehow
# 	knownRobots = [1,2,3,4,5,6,7,8,9]

# 	#command sequence for prototyping is select all, disperse
# 	commandSeq = ["SelectAll", "DisperseMax"]

# 	#command sequence that selects a group and moves them to a point
# 	#then selects another group and moves to a different point
# 	commandSeq = ["SelectGroup[1,2,4,6,8,9]", "MoveTo[132,23]", "SelectGroup[3,5,7,8]", "MoveTo[107,98]"]

# 	#The commands can be divided into meta-commands, which e.g. affect
# 	#the state of this code, and commands that are actually run on the robots
# 	metaCommands = ["SelectAll", "SelectGroup"] 

# 	#Storage of mapping of groups to programs
# 	groupProgs = []

# 	#Group of robots to get a command and the command they get
# 	commandBots = None
# 	gcprFile = None

# 	#Assumes that commands are ordered
# 	for command in commandSeq:
		
# 		cmdPrefix, args = getList(command)

# 		if cmdPrefix in metaCommands:
# 			#This command influences the state of this program
# 			if command == "SelectAll":
# 				commandBots = knownRobots
# 				#Generate the program file name
# 				outputProgFile = "{0}.gcpr".format(uuid.uuid4())
# 				gcprFile = open(outputProgFile, 'w')
# 				groupProgs.append([commandBots, outputProgFile])
# 			if command.startswith("SelectGroup"):
# 				#Parse the command to get the group members
# 				command, commandBots = getList(command)
# 				#Generate the program file name
# 				outputProgFile = "{0}.gcpr".format(uuid.uuid4())
# 				gcprFile = open(outputProgFile, 'w')
# 				groupProgs.append([commandBots, outputProgFile])
# 		else:
# 			#This is a command to run on a robot
# 			gcprExpr = lookupGCPR(command)
# 			gcprFile.write(gcprExpr)
	

# 	#Program the GCPR programs to the robots
# 	for commandGroup, file in groupProgs:
# 		for robot in commandGroup:
# 			deployProg(robot, file)

