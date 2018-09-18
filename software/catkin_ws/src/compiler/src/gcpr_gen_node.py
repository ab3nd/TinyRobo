#!/usr/bin/python

# Turns gestures received from the UI and gesture nodes into programs to be dispatched to robots

# For the sake of simplicity, the first version of this program will only accept the "single move" command, 
# which is a drag of a single robot, from the location it is currently in, to a new location, where it stops. 

from user_interface.msg import Gesture
import rospy
import decompose_space
import math

class ProgGen(object):
	def __init__(self):
		self.gestureSub = rospy.Subscriber("gestures", Gesture, self.addGesture)
		#All gestures seen so far, treated as a stack
		self.gestures = []

		#Spatial resolution of path points
		self.resolution = 0.15 #in meters

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

		#Generate GCPR for the area inside the bounding box 
		program = []
		dec = decompose_space.get_decomposition((maxX, maxY), (minX, minY), points, self.resolution)
		for square in dec:
			program.append(("self.is_in({0}, {1})".format(square.tl, square.br), "self.set_desired_heading({0})".format(math.pi - square.heading), 0.9))

		#Generate GCPR for the area outside the bounding box (all remaining space)
		program.append(("self.x_between({0}, {1}) and self.y_gt({2})".format(minX, maxX, maxY), "self.set_desired_heading()", 1.0))
		program.append(("self.x_between({0}, {1}) and self.y_lt({2})".format(minX, maxX, minY), "self.set_desired_heading()", 1.0))
		program.append(("self.y_between({0}, {1}) and self.x_gt({2})".format(minX, maxX, maxX), "self.set_desired_heading()", 1.0))
		program.append(("self.y_between({0}, {1}) and self.x_lt({2})".format(minX, maxX, minX), "self.set_desired_heading()", 1.0))
		program.append(("not(self.x_between({0}, {1})) and self.y_gt({2}) and self.x_gt({3})".format(minX, maxX, maxY, maxX), "self.set_desired_heading()", 1.0))
		program.append(("not(self.x_between({0}, {1})) and self.y_lt({2}) and self.x_gt({3})".format(minX, maxX, minY, maxX), "self.set_desired_heading()", 1.0))
		program.append(("not(self.y_between({0}, {1})) and self.y_gt({2}) and self.x_lt({3})".format(minX, maxX, maxY, minX), "self.set_desired_heading()", 1.0))
		program.append(("not(self.y_between({0}, {1})) and self.y_lt({2}) and self.x_lt({3})".format(minX, maxX, minY, minX), "self.set_desired_heading()", 1.0))

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
			point_proxy = rospy.ServiceProxy(name, service_class, persistent=True)
			for stroke in self.gestures[-1].strokes:
				for event in stroke.events:
					#Convert points to meters from pixels
					pt = point_proxy(event.point)
					path_points.append((pt.x, pt.y))
			#Done using the proxy, close it
			point_proxy.close()

			#TODO, assure that these are sorted
			
			#Simplify the path by dropping points that are less than the configured 
			#resolution away from the next point in the path. Path ends up being entirely
			#composed of points greater than the resolution from the next point in the path. 
			simple_path = [path_points[0]]
			for point in path_points:
				if self.distance(point, path_points[-1]) >= self.resolution:
					#Add it to the path
					simple_path.append(point)

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
		print "{} gets {}".format(robots, program)

#Start everything up and then just spin
rospy.init_node('gcpr_gen')
progam_generator = ProgGen()
rospy.spin()