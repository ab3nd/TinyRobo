#!/usr/bin/python


import rospy
from geometry_msgs.msg import Twist, PointStamped
from apriltags_ros.msg import *
import random
import math
from tf import transformations as trans

from tf import TransformListener
from image_geometry import PinholeCameraModel
from sensor_msgs.msg import CameraInfo
from trianglesolver import solve

def dot(a, b):
	assert len(a) == len(b)
	d = 0
	for i, j in zip(a, b):
		d += i*j
	return d

class Point_Driver():
	def __init__(self):
		self.pub = rospy.Publisher('point_twists', Twist, queue_size=0)
		self.targetX = 0.0
		self.targetY = 0.0


		self.robot_id = 8

		self.currentX = 0.0
		self.currentY = 0.0
		self.currentZ = 0.0
		self.currentX_px = 0.0
		self.currentY_px = 0.0

		all_tags = rospy.get_param('/apriltag_detector/tag_descriptions')
		self.tagsize = 0.051 #default, but we try to update it
		for tag in all_tags:
			if tag['id'] == self.robot_id:
				self.tagsize = tag['size']

		self.conversion = 0.0

		self.tf = TransformListener()

		self.camera_model = None

	def update_tags(self, tags_msg):
		#Get the location of this robot from the tag
		tag = None
		try:
			tag = [x for x in tags_msg.detections if x.id == self.robot_id][0]
		except IndexError as ie:
			#This is caused by there not being a detection of the tag in this set of 
			#tag detections
			rospy.logwarn("Didn't see tag {0} in this frame".format(self.robot_id))
			return 

		#Calculate the pixel to mm conversion for this tag
		#Get the distance between two adjacent corners in pixels
		dist = math.sqrt(math.pow((tag.tagCornersPx[0].x - tag.tagCornersPx[1].x), 2) + math.pow((tag.tagCornersPx[0].y - tag.tagCornersPx[1].y), 2))
		#Convert to pixels/m
		self.conversion = dist/self.tagsize

		self.currentX = tag.pose.pose.position.x 
		self.currentY = tag.pose.pose.position.y 
		self.currentZ = tag.pose.pose.position.z 
		self.currentX_px = tag.tagCenterPx.x
		self.currentY_px = tag.tagCenterPx.y


	def update_cam(self, cam_info):
		self.camera_model = PinholeCameraModel()
		self.camera_model.fromCameraInfo(cam_info)

	def update_target(self, point_msg):
		
		#Kivy coordinates have (0,0) at bl of image/window
		#Camera coordinates have (0,0) at center of image
		#These magical numbers (the image size in meters) are in Very Poor Style
		#img_h_m = 768.0 / self.conversion
		#img_w_m = 1024.0 / self.conversion

		#flip points on y axis because kivy (0,0) is bl, image (0,0) is tl
		image_height = 768
		px_x = point_msg.point.x
		px_y = max(image_height-point_msg.point.y, 0)


		#Figure out where the point is in camera space
		if self.camera_model is not None:
			#Unit vector pointing to the clicked point, in the camera frame (origin at center of camera)
			ray_to_click = self.camera_model.projectPixelTo3dRay((px_x, px_y))
			#Second point, needed to create a triangle to figure out the distance along the ray
			ray_to_tag = self.camera_model.projectPixelTo3dRay((self.currentX_px, self.currentY_px))
			
			#use trianglesolver to get the sides and angles I don't have

			#Get distance from camera to tag in meters (Euclidian distance from origin)
			cam_to_tag = math.sqrt(math.pow(self.currentX, 2) + math.pow(self.currentY, 2) + math.pow(self.currentZ, 2))
			#Get distance from tag center to clicked point in px, then convert to meters
			tag_to_click = math.sqrt(math.pow(self.currentX_px - px_x, 2) + math.pow(self.currentY_px - px_y, 2))
			tag_to_click = tag_to_click/self.conversion
			#Get angle between rays to clicked point and tag in radians
			#This is dot product in 3d, then take the arc cosine to get radians
			cam_angle = (float(ray_to_click[0])*float(ray_to_tag[0])) + (float(ray_to_click[1])*float(ray_to_tag[1])) + (float(ray_to_click[2])*float(ray_to_tag[2]))
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
   			mag = math.sqrt(math.pow(self.currentX_px, 2) + math.pow(self.currentY_px, 2) + math.pow(self.currentZ/self.conversion,2))
   			ray_to_cam = (self.currentX_px/mag, self.currentY_px/mag, (self.currentZ/self.conversion)/mag)
   			# #The ray to the click is the click location minus the tag location, the Z values are the same,
   			# #so the result of subtracting them is 0
   			mag = math.sqrt(math.pow(self.currentX_px - px_x, 2) + math.pow(self.currentY_px - px_y, 2))
   			ray_to_click_from_tag = ((self.currentX_px - px_x)/mag, (self.currentY_px - px_y)/mag, 0)

   			tag_angle = math.acos(dot(ray_to_cam, ray_to_click_from_tag))
   			print dot(ray_to_cam, ray_to_click_from_tag), tag_angle


			#Solve for the unknowns with trianglesolver (https://pypi.org/project/trianglesolver/)
			cam_to_click, cam_to_tag, tag_to_click, tag_angle, click_angle, cam_angle = solve(b=cam_to_tag, c=tag_to_click, A=tag_angle)#, C=cam_angle)

			print cam_to_click, cam_to_tag, tag_to_click, tag_angle, click_angle, cam_angle

		print "Click ({}, {}), Current px ({}, {})".format(point_msg.point.x, point_msg.point.y, self.currentX_px, self.currentY_px)		
		# #Convert point in pixels to meters in app frame
		# targetPoint = PointStamped()
		# targetPoint.header.frame_id = "ui_app"
		# targetPoint.header.stamp = rospy.Time(0)
		# #Convert to meters and move origin
		# targetPoint.point.x = (point_msg.point.x / self.conversion) - (img_w_m/2)
		# targetPoint.point.y = (point_msg.point.y / self.conversion) - (img_h_m/2)
		# targetPoint.point.z = 0.0 #Don't care about 3d location?

		# #Get a transform from the app to the camera frame
		# if self.tf.frameExists("ui_app") and self.tf.frameExists("camera_frame"):
		# 	targetPoint = self.tf.transformPoint("camera_frame", targetPoint)


		# self.targetX = targetPoint.point.x
		# self.targetY = targetPoint.point.y

		# print self.conversion
		# print ("Current ({}, {}), Target ({}, {})").format(self.currentX, self.currentY, self.targetX, self.targetY)

		# #Calculate the error between the rotation of the robot and orientation towards the point
		# x = self.targetX - tag.pose.pose.position.x 
		# y = self.targetY - tag.pose.pose.position.y
		# z = tag.pose.pose.position.z #I don't care about the height of the tag

		# #Get the robot's orientation vector by multiplying its orientation quaternion with an unrotated vector
		# #Of length one pointing forwards (x is forwards in ROS)
		# p = [1,0,0,0]
		# q = tag.pose.pose.orientation
		# q = [q.x, q.y, q.z, q.z]

		# q_prime = trans.quaternion_conjugate(q)
		# p_prime = trans.quaternion_multiply(trans.quaternion_multiply(q,p), q_prime)

		# #Get the vector out
		# x1 = p_prime[0]
		# y1 = p_prime[1]
		# z1 = p_prime[2]

		# #Angle between two vectors 
		# v1 = [x,y,z]
		# v2 = [x1,y1,z1]
		# mag1 = math.sqrt(sum([pow(v, 2) for v in v1]))
		# mag2 = math.sqrt(sum([pow(v, 2) for v in v2]))
		# dot = sum([v[0] * v[1] for v in zip(v1,v2)])

		# #Angle
		# print math.acos(dot/(mag1*mag2))


		# #Caclulate the error between the location of the robot and the location of the point
		# errDist = math.sqrt(pow(tag.pose.pose.position.x - self.targetX,2) + pow(tag.pose.pose.position.y - self.targetY,2))
		# print "distance", errDist
		#Generate a twist message and send it to the robot


		# rate = rospy.Rate(0.3) #Every three seconds
		# while not rospy.is_shutdown():
		# 	#Random linear and rotational velocites
		# 	#Twists are usually in m/sec, but 1m/sec is plenty fast for any of my robots
		# 	#These are in the range +/-1, more or less 
		# 	linear = (random.random()*2)-1
		# 	rotational = (random.random()*2)-1

		# rTwist = Twist()
		# #only two params are used for robots on a table
		# rTwist.linear.x = linear
		# rTwist.angular.z = rotational
		# #The rest are not used
		# rTwist.linear.y = rTwist.linear.z = 0
		# rTwist.angular.x = rTwist.angular.y = 0

		# #publish it and wait
		# self.pub.publish(rTwist)
		
if __name__ == '__main__':
	rospy.init_node('point_driver', anonymous=True)

	pd = Point_Driver()

	loc_sub = rospy.Subscriber('/tag_detections', AprilTagDetectionArray, pd.update_tags)

	go_sub = rospy.Subscriber('/touches', PointStamped, pd.update_target)

	cam_sub = rospy.Subscriber('/overhead_cam/camera_info', CameraInfo, pd.update_cam)

	rospy.spin()

	
#Attempt at a controller that picks random points and moves a single robot to them

#Subscribe to position feed

#Subscribe to camera to show points for debug

#Pick a location

#Highlight the location

#Move the robot