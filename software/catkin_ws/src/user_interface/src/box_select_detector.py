#!/usr/bin/python

import rospy
from user_interface.msg import Kivy_Event, Stroke
from geometry_msgs.msg import Point
import math
from apriltags_ros.msg import *
import classification_heuristics


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
		#Only line gestures can be box selections
		if classification_heuristics.is_line(msg):		
			#Clear out selected tags and see if the new gesture selects any of them
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
					# Rescale from pixels in camera view to pixels in UI view (cropped, embiggened image)
					tag_x = tag.tagCenterPx.x * 1.640625
					tag_y = (tag.tagCenterPx.y - 120) * 1.640625

					if (minX < tag_x < maxX) and (minY < tag_y < maxY):
						selected_tags.append(tag.id)

				if len(selected_tags) > 0:
					#This is possibly a box select, pack it up and publish it
					rospy.loginfo("{0} selects {1}".format(msg.uid, selected_tags))
		
rospy.init_node('box_select_detect')
bsd = BoxSelectDetector()
strokeSub = rospy.Subscriber("/strokes", Stroke, bsd.check_stroke)
tagSub = rospy.Subscriber("/tag_detections", AprilTagDetectionArray, bsd.update_robot_points)

rospy.spin()
