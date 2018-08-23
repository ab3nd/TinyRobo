#!/usr/bin/python

import rospy
from user_interface.msg import Kivy_Event, Stroke, Gesture
from geometry_msgs.msg import Point
import math
from apriltags_ros.msg import *
import classification_heuristics


# Receives the locations of the robots and the strokes made by the user, and determines if the 
# resulting configuration could be a box selection. Generally, this means that the user drew a line
# and that the bounding box of the line has robots inside of it. 
# This is actually too permissive of a criterion for box selection, but higher level nodes will
# accept or reject the box selection canidates based on state and previous actions

class PathDetector(object):
	def __init__(self):
		self.currentTags = {}
		self.gesturePub = rospy.Publisher("gestures", Gesture, queue_size=10)


	def check_stroke(self, msg):
		#circles and lines are checked by the lasso select detector and box selection detector
		if classification_heuristics.is_arc(msg):
			#Arcs are always treated as paths
			evt = Gesture()
			evt.eventName = "path"
			evt.stamp = rospy.Time.now()
			evt.isButton = False 
			evt.strokes = [msg]
			self.gesturePub.publish(evt)
		elif classification_heuristics.is_circle(msg):
			pass #Handled by lasso detector, is a path if it doesn't select robots
		elif classification_heuristics.is_line(msg):
			pass #Handled by box selection detector, is a path if it doesn't select robots

rospy.init_node('path_detect')
pd = PathDetector()
strokeSub = rospy.Subscriber("/strokes", Stroke, pd.check_stroke)

rospy.spin()
