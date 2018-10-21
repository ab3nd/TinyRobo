#!/usr/bin/python

# Turns gestures received from the UI and gesture nodes into programs to be dispatched to robots

# For the sake of simplicity, the first version of this program will only accept the "single move" command, 
# which is a drag of a single robot, from the location it is currently in, to a new location, where it stops. 

from user_interface.msg import Gesture
from robot_drivers.srv import MapPoint
from std_msgs.msg import String
import json
import rospy
import decompose_space
import math
import re
from lark import Lark, UnexpectedInput

class ProgSender(object):
	def __init__(self):
		self.robotPubs = {}
		self.topicRE = re.compile("\/bot([0-9]*)\/position")
		self.updatePubs()

	def updatePubs(self):
		#We can't get a list via rospy.get_published_topics(), because this node
		#is what's supposed to be publishing the topics, so get the IDs by looking at the 
		#other /botN/* topics
		for topic in rospy.get_published_topics():
			if self.topicRE.match(topic[0]):
				robot_id = self.topicRE.match(topic[0]).group(1)
				if robot_id not in self.robotPubs.keys():
					self.robotPubs[robot_id] = rospy.Publisher('/bot{}/robot_prog'.format(robot_id), String, queue_size=10)

	def pubProg(self, robot, program):
		#If we don't have a publisher for this robot yet, create one
		if robot not in self.robotPubs.keys():
			self.robotPubs[robot] = rospy.Publisher('/bot{}/robot_prog'.format(robot), String, queue_size=10)
			#Wait for publisher to be ready, and yes, this can be a problem. 
			rospy.sleep(0.5)

		self.robotPubs[robot].publish(json.dumps(program))


class ProgGen(object):
	def __init__(self):
		self.gestureSub = rospy.Subscriber("gestures", Gesture, self.addGesture)
		#All gestures seen so far, treated as a stack
		self.gestures = []

		#Spatial resolution of path points
		self.resolution = 0.3 #in meters

		#Object to handle sending programs to robots
		self.sender = ProgSender()

		# gesture_grammar='''
		# 	select : tap_select | box_select | lasso_select | select_group
		# 	tap_select : "tap_robot"+ 
		# 	box_select : "box_select"
		# 	lasso_select : "lasso_select"
		# 	select_group : "tap_robot" "select_group" | "select_group" "tap_robot" //order doesn't matter

		# 	patrol : "patrol"
		# 	formation : "formation"
		# 	move : "move_object"
		# 	remove : "remove_object"
			
		# 	disperse : "disperse_gesture"

		# 	drag_path : "drag_robot"

		# 	tap_waypoint : "tap_waypoint"
		# 	path : "path" | tap_waypoint+ //may need to add option to treat box/lasso as path

		# 	patrol_cmd : select? patrol path
		# 	formation_cmd : select? formation path
		# 	move_obj_cmd : select? move path 
		# 	disperse_cmd : select? disperse
		# 	path_cmd : drag_path | select? path

		# 	start : (patrol_cmd | formation_cmd | move_obj_cmd | disperse_cmd | path_cmd) end

		# 	end : "end"

		# 	%import common.WS
		# 	%ignore WS
		# '''


		#Implemented as per my thesis paper, only without CamelCase because lark is case sensitive
		gesture_grammar='''
			start: cmd "end"
			cmd: patrol | makeformation | moveobject | removerobot | disperse | gohere
			patrol: selection "patrol" path
			makeformation: selection "formation" path
			moveobject: selection "move_object" selection path
			removerobot: "remove_object" selection
			disperse: selection ("drag_robot" | path) ~ 4..5 
			gohere: selection path | "drag_robot"
			path: "path" | "tap_waypoint"+
			selection: gestureselect | groupselect
			gestureselect: "tap_robot"+ | "lasso_select" | "box_select"
			groupselect: "select_group" | "tap_robot"
			%import common.WS
			%ignore WS
		'''

		self.parser = Lark(gesture_grammar)

	def addGesture(self, gestureMsg):
		self.gestures.append(gestureMsg)
		#TODO checking the gesture list can be made conditional on the arriving gesture. 
		#This allows timeouts (sort-of, more than Nsec since the previous gesture), a "go" gesture,
		#or firing off the previous command when selection begins a new command. 
		self.checkGestureList()

	def distance(self, p1, p2):
		return math.sqrt(math.pow(p1[0]-p2[0],2) + math.pow(p1[1]-p2[1],2))

	def pathToGCPR(self, points):
		#Given a list of points, convert them to a GCPR program to steer along those points
		#Get the bounding box of the points
		maxX = maxY = float('-inf')
		minX = minY = float('inf')
		for point in points:
			maxX = max(point[0], maxX)
			maxY = max(point[1], maxY)
			minX = min(point[0], minX)
			minY = min(point[1], minY)

		#Pad min and max to ensure that there are spaces around each end of the path
		maxX += 2 * self.resolution
		maxY += 2 * self.resolution
		minX -= 2 * self.resolution
		minY -= 2 * self.resolution
		
		#Generate GCPR for the area inside the bounding box 
		program = []
		dec = decompose_space.get_decomposition((minX, maxY), (maxX, minY), points, self.resolution)
		for square in dec:
			program.append(("self.is_in({0}, {1})".format(square.tl, square.br), "self.set_desired_heading({0})".format(math.pi - square.heading), 0.9))

		#Reactive obstacle avoidance
		program.append(("self.is_near_left() and not(self.is_near_right()) and not(self.is_near_center())", "self.move_turn(-0.9)", 0.9))
		program.append(("self.is_near_right() and not(self.is_near_left()) and not(self.is_near_center())", "self.move_turn(0.9)", 0.9))
		#Back and turn away
		program.append(("self.is_near_left() and self.is_near_right() and not(self.is_near_center())", "self.move_arc(2.0, -0.5)", 0.8))
		program.append(("self.is_near_center() and not(self.is_near_right()) and not(self.is_near_left())", "self.move_arc(2.0, -0.5)", 1.0))
		#Move away from stuff behind
		program.append(("self.is_near_anything() and not(self.is_near_left()) and not(self.is_near_center()) and not(self.is_near_right())", "self.move_fwd(0.4)", 1.0))

		#Generate GCPR for the area outside the bounding box (all remaining space)
		program.append(("self.x_between({0}, {1}) and self.y_gt({2})".format(minX, maxX, maxY), "self.set_desired_heading(-math.pi/2.0)", 1.0)) #math.pi)", 1.0))
		program.append(("self.x_between({0}, {1}) and self.y_lt({2})".format(minX, maxX, minY), "self.set_desired_heading(math.pi/2.0)", 1.0))
		program.append(("self.y_between({0}, {1}) and self.x_gt({2})".format(minX, maxX, maxX), "self.set_desired_heading(0)", 1.0))#math.pi/2.0)", 1.0))
		program.append(("self.y_between({0}, {1}) and self.x_lt({2})".format(minX, maxX, minX), "self.set_desired_heading(-math.pi)", 1.0))#-math.pi/2.0)", 1.0))
		program.append(("not(self.x_between({0}, {1})) and self.y_gt({2}) and self.x_gt({3})".format(minX, maxX, maxY, maxX), "self.set_desired_heading(-math.pi/4.0)", 1.0))#math.pi/4.0)", 1.0))
		program.append(("not(self.x_between({0}, {1})) and self.y_lt({2}) and self.x_gt({3})".format(minX, maxX, minY, maxX), "self.set_desired_heading(math.pi/4.0)", 1.0))#-math.pi/4.0)", 1.0))
		program.append(("not(self.y_between({0}, {1})) and self.y_gt({2}) and self.x_lt({3})".format(minX, maxX, maxY, minX), "self.set_desired_heading((-3 * math.pi)/4)", 1.0))#-(3*math.pi)/4.0)", 1.0))
		program.append(("not(self.y_between({0}, {1})) and self.y_lt({2}) and self.x_lt({3})".format(minX, maxX, minY, minX), "self.set_desired_heading((3 * math.pi)/4)", 1.0))#(3*math.pi)/4.0)", 1.0))

		#Add motion commands to turn to bearing and move forward
		program.append(("self.on_heading() and not(self.is_near_anything())", "self.move_fwd(0.3)", 1.0))
		program.append(("not(self.on_heading()) and not(self.is_near_anything())", "self.turn_heading(1)", 1.0))

		return program


	def parseGestures(self, gesture_list):
		#Get a list of the gestures
		prog_str = " ".join([g.eventName for g in gesture_list])

		try:
			#Parse it to build the tree
			parse_tree = self.parser.parse(prog_str)

			#For now just prettyprint it
			print prog_str
			print parse_tree.pretty()
			print "---"
		except UnexpectedInput as e:
			print e

	def checkGestureList(self):
		#If the last thing in is an end gesture, we're good to try to parse the gestures
		if self.gestures[-1].eventName == "end":
			print "--> gesture ended"

			#Call the parser on it
			self.parseGestures(self.gestures)

			#Flush the gesture buffer
			self.gestures = []

		#If the last thing in is a select, and there are any selects already present in the buffer,
		#then it's time to end the previous gesture and parse that, and leave the select in as
		#the beginning of a new gesture
		elif self.isSelect(self.gestures[-1]) and any([self.isSelect(x) for x in self.gestures[:-1]]):
			print "--> select started new gesture"
			#insert an end to it and ship it to the parser
			toParse = self.gestures[:-1]
			evt = Gesture()
			evt.eventName = "end"
			evt.stamp = rospy.Time.now()
			evt.isButton = False 
			evt.robots = []
			evt.strokes = []
			toParse.append(evt)

			self.parseGestures(toParse)
			
			#Clear the stuff that was just sent to the parser, but leave the 
			#selection gesture that kicked off this parse attempt
			self.gestures = [self.gestures[-1]]

	def isPath(self, gestureMsg):
		if gestureMsg.eventName == "path":
			return True
		return False

	def isSelect(self, gestureMsg):
		if gestureMsg.eventName in ["box_select", "lasso_select", "tap_select"]:
			return True
		return False

	def publishProgram(self, robots, program):
		for robot in robots:
			self.sender.pubProg(robot, program)

#Start everything up and then just spin
rospy.init_node('gcpr_gen')
progam_generator = ProgGen()
rospy.spin()