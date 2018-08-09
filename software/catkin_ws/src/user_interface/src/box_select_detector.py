#!/usr/bin/python

import rospy
from user_interface.msg import Kivy_Event, Stroke
from geometry_msgs.msg import Point
import math
from apriltags_ros.msg import *

# Receives the locations of the robots and the strokes made by the user, and determines if the 
# resulting configuration could be a box selection. Generally, this means that the user drew a line
# and that the bounding box of the line has robots inside of it. 
# This is actually too permissive of a criterion for box selection, but higher level nodes will
# accept or reject the box selection canidates based on state and previous actions

class BoxSelectDetector(object):
	def __init__(self):
		self.currentTags = {}

	def update_robot_points(self, msg):
		#Just saves the detections
		self.currentTags = {}
		for ii in range(len(msg.detections)):
			self.currentTags[int(msg.detections[ii].id)] = msg.detections[ii]
			
	def check_stroke(msg):
		if len(self.currentTags) > 0:
			#Get the bounding box of the stroke
			#TODO might want to check that it's not too thin?
			#For each tag
				#For each corner of the tag
				#If the corner is inside the box, this robot is selected
				#Calculate pixels per meter from tag size
				tag = self.currentTags.values()[0]
				d = 0
				for ii in range(1, len(tag.tagCornersPx)):
					d += self.distancePixels(tag.tagCornersPx[ii], tag.tagCornersPx[ii-1])
				d = d/len(tag.tagCornersPx)
				self.avgPxPerM = d/tag.size
			#If any tags are selected, this stroke could be a box selection
		
		
rospy.init_node('touch_destutter')
bsd = BoxSelectDetector()
strokeSub = rospy.Subscriber("/strokes", Stroke, bsd.check_stroke)
strokeSub = rospy.Subscriber("/tag_detections", , bsd.check_stroke)

rospy.spin()
