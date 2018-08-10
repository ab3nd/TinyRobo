#!/usr/bin/python

import rospy
from user_interface.msg import Kivy_Event, Stroke
from geometry_msgs.msg import Point
import math
from apriltags_ros.msg import *

#For debugging
import Image, ImageDraw, ImageFont
import numpy as np

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

	def check_stroke(self, msg):
		#For debugging
		#dumpStroke(msg)
		
		selected_tags = []
		if len(self.currentTags) > 0:
			#Get the bounding box of the stroke
			xs = [event.point.x for event in msg.events]
			minX = min(xs)
			maxX = max(xs)
			ys = [event.point.y for event in msg.events]
			minY = min(ys)
			maxY = max(ys)
			#Get all the tags that have at least one corner in the box
			for tag in self.currentTags.values():
				# print "minX: {0} tagCenterX: {1} maxX: {2}".format(minX, tag.tagCenterPx.x, maxX)
				# print "minY: {0} tagCenterY: {1} maxY: {2}".format(minY, tag.tagCenterPx.y, maxY)
				# print "---"
				if (minX < tag.tagCenterPx.x < maxX) and (minY < tag.tagCenterPx.y < maxY):
					selected_tags.append(tag.id)

			if len(selected_tags) > 0:
				#This is possibly a box select, pack it up and publish it
				rospy.loginfo("{0} selects {1}".format(msg.uid, selected_tags))
		
rospy.init_node('box_select_detect')
bsd = BoxSelectDetector()
strokeSub = rospy.Subscriber("/strokes", Stroke, bsd.check_stroke)
tagSub = rospy.Subscriber("/tag_detections", AprilTagDetectionArray, bsd.update_robot_points)

rospy.spin()
