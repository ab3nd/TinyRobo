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

	def checkGestureList(self):
		#print the gesture list
		for g in self.gestures:
			print g.stamp, g.eventName

		#If we have two gestures, the top gesture is a path, and the previous gesture is a select of any sort
		if len(self.gestures) >= 2 and self.isPath(self.gestures[-1]) and self.isSelect(self.gestures[-2]):
			#Get the selected robots
			selected = self.gestures[-2].robots

			#Generate the GCPR path representation for them
			#First, get the points on the path
			path_points = []
			for stroke in self.gestures[-1].strokes:
				for event in stroke.events:
					path_points.append((event.point.x, event.point.y))
			#TODO, assure that these are sorted
			
			#Simplify the path by dropping points that are less than the configured 
			#resolution away from the next point in the path. Path ends up being entirely
			#composed of points greater than the resolution from the next point in the path. 
			simple_path = [path_points[0]]
			for point in path_points:
				if self.distance(point, path_points[-1]) >= self.resolution:
					#Add it to the path
					simple_path.append(point)

			print len(path_points), len(simple_path)

			program = "foo"
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
		print "{} get {}".format(robots, program)

#Start everything up and then just spin
rospy.init_node('gcpr_gen')
progam_generator = ProgGen()
rospy.spin()