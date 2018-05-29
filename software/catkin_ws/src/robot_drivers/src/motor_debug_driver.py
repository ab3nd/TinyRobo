#!/usr/bin/python


import rospy
from geometry_msgs.msg import Twist
from apriltags_ros.msg import *
import random
import math
from tf import transformations as trans
from geometry_msgs.msg import PointStamped
from tf import TransformListener

class Point_Driver():
	def __init__(self):
		self.pub = rospy.Publisher('point_twists', Twist, queue_size=0)
		self.targetX = 0.0
		self.targetY = 0.0


		self.robot_id = 8

		self.currentX = 0.0
		self.currentY = 0.0

		all_tags = rospy.get_param('/apriltag_detector/tag_descriptions')
		self.tagsize = 0.051 #default, but we try to update it
		for tag in all_tags:
			if tag['id'] == self.robot_id:
				self.tagsize = tag['size']

		self.conversion = 0.0

		self.tf = TransformListener()

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

	def update_target(self, point_msg):
		
		#Kivy coordinates have (0,0) at bl of image/window
		#Camera coordinates have (0,0) at center of image
		#These magical numbers (the image size in meters) are in Very Poor Style
		img_h_m = 768.0 / self.conversion
		img_w_m = 1024.0 / self.conversion
		
		#Convert point in pixels to meters in app frame
		targetPoint = PointStamped()
		targetPoint.header.frame_id = "ui_app"
		targetPoint.header.stamp = rospy.Time(0)
		#Convert to meters and move origin
		targetPoint.point.x = (point_msg.point.x / self.conversion) - (img_w_m/2)
		targetPoint.point.y = (point_msg.point.y / self.conversion) - (img_h_m/2)
		targetPoint.point.z = 0.0 #Don't care about 3d location?

		#Get a transform from the app to the camera frame
		if self.tf.frameExists("ui_app") and self.tf.frameExists("camera_frame"):
			targetPoint = self.tf.transformPoint("camera_frame", targetPoint)


		self.targetX = targetPoint.point.x
		self.targetY = targetPoint.point.y

		print self.conversion
		print ("Current ({}, {}), Target ({}, {})").format(self.currentX, self.currentY, self.targetX, self.targetY)

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


	rospy.spin()

	
#Attempt at a controller that picks random points and moves a single robot to them

#Subscribe to position feed

#Subscribe to camera to show points for debug

#Pick a location

#Highlight the location

#Move the robot