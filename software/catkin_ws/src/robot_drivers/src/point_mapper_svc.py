#!/usr/bin/python

import math
import rospy
from geometry_msgs.msg import Point
from apriltags_ros.msg import *
from image_geometry import PinholeCameraModel
from sensor_msgs.msg import CameraInfo
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
		self.camera_model = None
		self.camera_frame = None

	def update_tags(self, tags_msg):
		#We can use pretty much any tag, we just need to get the tag size
		tag = None
		for tag in tags_msg.detections:
			if tag is None:
				return
			if tag.id in self.tagsizes.keys():
				break

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

	def update_cam(self, cam_info):
		self.camera_model = PinholeCameraModel()
		self.camera_model.fromCameraInfo(cam_info)
		self.camera_frame = cam_info.header.frame_id

	def update_target(self, point_req):
		#flip points on y axis because kivy (0,0) is bl, image (0,0) is tl
		image_height = 768
		px_x = point_req.inPixels.x
		px_y = max(image_height-point_req.inPixels.y, 0)

		#If we don't have a camera model yet, we can't figure out where the click is
		if self.camera_model is None:
			rospy.logwarn("No camera model yet, returning without updating target")
			return

		#Figure out where the point is in camera space
		#Unit vector pointing to the clicked point, in the camera frame (origin at center of camera)
		ray_to_click = [float(x) for x in self.camera_model.projectPixelTo3dRay((px_x, px_y))]
		#Second point, needed to create a triangle to figure out the distance along the ray
		ray_to_tag = [float(x) for x in self.camera_model.projectPixelTo3dRay((self.currentX_px, self.currentY_px))]
		
		#Get distance from camera to tag in meters (Euclidian distance from origin)
		cam_to_tag = dist((self.currentX, self.currentY, self.currentZ), (0,0,0))
		#Get distance from tag center to clicked point in px, then convert to meters
		tag_to_click = dist((self.currentX_px, self.currentY_px), (px_x,  px_y))
		tag_to_click = tag_to_click/self.conversion
		#Get angle between rays to clicked point and tag in radians
		#This is dot product in 3d, then take the arc cosine to get radians
		cam_angle = dot(ray_to_click, ray_to_tag)
		cam_angle = math.acos(cam_angle)

		# Triangle is set up like this:
		#
		# 	   		   cam_angle
		# 			     /  \
		# 			    /    \
		#   cam_to_tag /      \ cam_to_click (?)
		#             /        \
		#            /          \
		#    tag_angle (?)----click_angle (?)
		#                   |
		#              tag_to_click
        #
		# The values with a question mark are initially unknown

		#The triangle solver needs more information to disambiguate triangles, so find tag angle
		#The ray to the camera points to the origin, so it's just the tag location, normalized to a unit vector
		mag = dist((self.currentX_px, self.currentY_px, self.currentZ/self.conversion), (0,0,0))
		ray_to_cam = (self.currentX_px/mag, self.currentY_px/mag, (self.currentZ/self.conversion)/mag)

		# #The ray to the click is the click location minus the tag location, the Z values are the same,
		# #so the result of subtracting them is 0
		mag = dist((self.currentX_px, self.currentY_px), (px_x, px_y))
		ray_to_click_from_tag = ((self.currentX_px - px_x)/mag, (self.currentY_px - px_y)/mag, 0)

		#Get the angle between the ray from the tag to the camera, and the ray from the tag to the click
		tag_angle = math.acos(dot(ray_to_cam, ray_to_click_from_tag))
		
		#Solve for the unknowns with trianglesolver (https://pypi.org/project/trianglesolver/)
		cam_to_click, cam_to_tag, tag_to_click, tag_angle, click_angle, cam_angle = solve(b=cam_to_tag, c=tag_to_click, A=tag_angle)#, C=cam_angle)

		#Set up the target point, this is all in meters in the camera frame
		click_point = [x * cam_to_click for x in ray_to_click]
		
		#For debugging, making sure that if I click on the tag, click point and current are the same
		#print "Current: {} {} {} Click: {}".format(self.currentX, self.currentY, self.currentZ, click_point)

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

	