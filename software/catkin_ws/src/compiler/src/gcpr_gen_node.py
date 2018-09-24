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


	def checkGestureList(self):
		#print the gesture list
		for g in self.gestures:
			print g.stamp, g.eventName

		#If we have two gestures, the top gesture is a path, and the previous gesture is a select of any sort
		if len(self.gestures) >= 2 and self.isPath(self.gestures[-1]) and self.isSelect(self.gestures[-2]):
			#Get the selected robots
			selected = self.gestures[-2].robots

			#Generate the GCPR path representation for them
			#First, get the points on the path in meters
			path_points = []
			#Persist so we don't have to set it up for each call (could be lots)
			point_proxy = rospy.ServiceProxy("map_point", MapPoint, persistent=True)

			for stroke in self.gestures[-1].strokes:
				for event in stroke.events:
					#Convert points to meters from pixels
					pt = point_proxy(event.point)
					path_points.append((pt.inMeters.x, pt.inMeters.y))
					
			#Done using the proxy, close it
			point_proxy.close()

			#TODO, check that these are sorted by time
			
			#Simplify the path by dropping points that are less than the configured 
			#resolution away from the next point in the path. Path ends up being entirely
			#composed of points greater than the resolution from the next point in the path. 
			simple_path = [path_points[0]]
			for point in path_points:
				if self.distance(point, simple_path[-1]) >= self.resolution:
					#Add it to the path
					simple_path.append(point)

			print simple_path
			program = self.pathToGCPR(simple_path)
			#Send the program
			self.publishProgram(selected, program)

			#The last and second-from-last gestures have been handled, remove from the stack
			self.gestures = self.gestures[:-2]

		#TODO expire gestures due to old age?
		print "----"

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