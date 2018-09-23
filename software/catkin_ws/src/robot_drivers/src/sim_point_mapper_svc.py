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
		#Location of the click, to be calculated
		self.targetX = 0.0
		self.targetY = 0.0
		self.targetZ = 0.0

		#Location of pretty much any tag, for making a triangle.
		#Turns out robotics is pretty much trig and linear algebra on wheels. 
		self.currentX = 0.0
		self.currentY = 0.0
		self.currentZ = 0.0
		self.currentX_px = 0.0
		self.currentY_px = 0.0

		#Get the sizes of all the tags
		all_tags = rospy.get_param('/apriltag_detector/tag_descriptions')
		self.tagsizes = {0:0.051} #default, but we try to update it
		for tag in all_tags:
			self.tagsizes[tag['id']] = tag['size']

		#For converting pixels to meters, and camera geometry math
		self.conversion = 0.0

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

		self.currentX = tag.pose.pose.position.x 
		self.currentY = tag.pose.pose.position.y 
		self.currentZ = tag.pose.pose.position.z 
		self.currentX_px = tag.tagCenterPx.x
		self.currentY_px = tag.tagCenterPx.y

	def update_target(self, point_req):
		#The simulated area maps 1:1 with the 1024x768 image, so this conversion is a lot 
		#simpler than the unsimulated point mapper, which uses camera intrinsics. 

		#The Argos arena is 1024x768, the Argos arena is 8x6, so they have the same aspect ratio.
		#Kivy's (0,0) is at the bottom left of the screen, the arena (0,0) is at the center of the arena,
		#so in full screen mode, arena (0,0) is at Kivy (1024/2, 768/2)
		screen_w = 1024
		screen_h = 768
		#Map the center shift, then convert to meters
		px_x = (point_req.inPixels.x - screen_w/2)/self.conversion
		px_y = (point_req.inPixels.y - screen_h/2)/self.conversion 

		#Ship it out!
		point = Point()
		point.x = click_point[0]
		point.y = click_point[1]
		point.z = click_point[2]
		
		return point
		
if __name__ == '__main__':
	rospy.init_node('point_converter', anonymous=True)

	pc = PointConverter()

	loc_sub = rospy.Subscriber('/tag_detections', AprilTagDetectionArray, pc.update_tags)
	cam_sub = rospy.Subscriber('/overhead_cam/camera_info', CameraInfo, pc.update_cam)

	svc = rospy.Service("map_point", MapPoint, pc.update_target)
	rospy.spin()

	