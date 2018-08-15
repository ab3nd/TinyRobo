#!/usr/bin/python

import rospy
from user_interface.msg import Kivy_Event, Stroke
from geometry_msgs.msg import Point
import math
from apriltags_ros.msg import *
import classification_heuristics
import numpy as np

# Receives the locations of the robots and the strokes made by the user, and determines if the 
# resulting configuration could be a box selection. Generally, this means that the user drew a line
# and that the bounding box of the line has robots inside of it. 
# This is actually too permissive of a criterion for box selection, but higher level nodes will
# accept or reject the box selection canidates based on state and previous actions


class TapSelectDetector(object):
	def __init__(self):
		pass

	def update_robot_points(self, msg):
		#Just saves the detections
		self.currentTags = {}
		for ii in range(len(msg.detections)):
			self.currentTags[int(msg.detections[ii].id)] = msg.detections[ii]

	def distance(self, x1, y1, x2, y2):
		return math.sqrt(pow(x1 - x2, 2) + pow(y1 -y2, 2))

	def check_stroke(self, msg):
		#Make sure the gesture is a tap of any sort
		if classification_heuristics.is_tap(msg) or msg.isDoubletap or msg.isTripletap:
		
			#Get the tag closest to the tap
			closest_tag = None
			min_dist = float('inf')
			for tag in self.currentTags.values():
				# Rescale from pixels in camera view to pixels in UI view (cropped, embiggened image)
				tag_x = tag.tagCenterPx.x * 1.640625
				tag_y = (tag.tagCenterPx.y - 120) * 1.640625

				#Tap is going to have a centroid v. near the various taps
				tap_x = msg.centroid.x
				tap_y = msg.centroid.y

				#Check distance
				d = self.distance(tag_x, tag_y, tap_x, tap_y)
				if d < min_dist:
					rospy.logwarn("distance: {0}".format(d))
					min_dist = d
					closest_tag = tag.id

			#This is about the width of a fingertip in pixels on the screen
			if min_dist < 80 and closest_tag is not None:
				#This is possibly a tap select, pack it up and publish it
				rospy.loginfo("{0} selects {1}".format(msg.uid, closest_tag))
				closest_tag = None
		
rospy.init_node('lasso_select_detect')
tsd = TapSelectDetector()
strokeSub = rospy.Subscriber("/strokes", Stroke, tsd.check_stroke)
tagSub = rospy.Subscriber("/tag_detections", AprilTagDetectionArray, tsd.update_robot_points)

rospy.spin()
