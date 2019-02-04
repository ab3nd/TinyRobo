#!/usr/bin/python

import math
import rospy
from geometry_msgs.msg import Point
from apriltags_ros.msg import *
from trianglesolver import solve

from robot_drivers.srv import MapPoint

#Dot product
def dot(a, b):
	assert len(a) == len(b)
	return sum([i * j for i, j in zip(a, b)])

#Euclidean distance
def dist(a, b):
	assert len(a) == len(b)
	return math.sqrt(sum([math.pow(i-j, 2) for i, j in zip(a,b)]))

class PointConverter():
	def __init__(self):
		#Get the sizes of all the tags
		self.tagsizes = {0:0.051} #default, but we try to update it
		try:
			all_tags = rospy.get_param('/apriltag_detector/tag_descriptions')
			for tag in all_tags:
				self.tagsizes[tag['id']] = tag['size']
			#For converting pixels to meters, and camera geometry math
			self.conversion = 0.0
		except KeyError as e:
			#We're running without april tag detection, so make up a bunch
			#of fake tags. This is for testing the gesture compiler, not live runs			
			for tag in range(1000):
				#The default tag size I was using
				self.tagsizes[tag] = 0.051
			#Again, a fake value, gets reset in the real case
			self.conversion = 20/0.051

	def update_tags(self, tags_msg):
		#We can use pretty much any tag, we just need to get the tag size
		tag = None
		for tag in tags_msg.detections:
			if tag is None:
				return
			if tag.id in self.tagsizes.keys():
				break

		if tag is None:
			#This was an empty tag detection message, so nothing updates
			return

		tagsize = self.tagsizes[tag.id]
		#Calculate the pixel to mm conversion for this tag
		#Get the distance between two adjacent corners in pixels
		d = dist((tag.tagCornersPx[0].x, tag.tagCornersPx[0].y), (tag.tagCornersPx[1].x, tag.tagCornersPx[1].y))
		#Convert to pixels/m
		self.conversion = d/tagsize

	def update_target(self, point_req):
		#The simulated area maps 1:1 with the 1680x1050 image, so this conversion is a lot 
		#simpler than the unsimulated point mapper, which uses camera intrinsics. 

		#The Kivy window is 1680x1050, the Argos arena is 8x6. 
		#Kivy's (0,0) is at the bottom left of the screen, the arena (0,0) is at the center of the arena,
		#so in full screen mode, arena (0,0) is at Kivy (1680/2, 1050/2)
		screen_w = 1680
		screen_h = 1050
		#Map the center shift, then convert to meters
		px_x = (point_req.inPixels.x - screen_w/2)/self.conversion
		px_y = (max(screen_h-point_req.inPixels.y, 0) - screen_h/2)/self.conversion 

		#Ship it out!
		point = Point()
		point.x = px_x
		point.y = px_y
		point.z = 0
		
		return point
		
if __name__ == '__main__':
	rospy.init_node('point_converter', anonymous=True)

	pc = PointConverter()

	loc_sub = rospy.Subscriber('/tag_detections', AprilTagDetectionArray, pc.update_tags)
	
	svc = rospy.Service("map_point", MapPoint, pc.update_target)
	rospy.spin()

	