#!/usr/bin/python

import rospy
from geometry_msgs.msg import Twist, PointStamped
from apriltags_ros.msg import *
import math
from image_geometry import PinholeCameraModel
from sensor_msgs.msg import CameraInfo
from trianglesolver import solve


class PointDriver(object):

	def __init__(self):
		self.targetX = self.currentX = 0
		self.targetY = self.currentY = 0
		self.targetZ = self.currentZ = 0

		self.robot_id = 8

	def updatePoint(self, point_msg):

		# Get the error in x and y directions between current position and target position
		x = self.targetX - self.currentX
		y = self.targetY - self.currentY
		z = self.targetZ - self.currentZ

		#Get the robot's orientation vector by multiplying its orientation quaternion with an unrotated vector
		#Of length one pointing forwards (x is forwards in ROS)
		p = [1,0,0,0]
		q = tag.pose.pose.orientation
		q = [q.x, q.y, q.z, q.z]

		q_prime = trans.quaternion_conjugate(q)
		p_prime = trans.quaternion_multiply(trans.quaternion_multiply(q,p), q_prime)

		#Get the vector out
		x1 = p_prime[0]
		y1 = p_prime[1]
		z1 = p_prime[2]

		#Angle between two vectors 
		v1 = [x,y,z]
		v2 = [x1,y1,z1]
		mag1 = math.sqrt(sum([pow(v, 2) for v in v1]))
		mag2 = math.sqrt(sum([pow(v, 2) for v in v2]))
		dot = sum([v[0] * v[1] for v in zip(v1,v2)])

		#Angle to the clicked point
		print "Angle {}".format(math.acos(dot/(mag1*mag2)))

		#Calculate the error between the location of the robot and the location of the point
		errDist = math.sqrt(pow(self.currentX - self.targetX,2) + pow(self.currentY- self.targetY,2))
		print "Distance {}".format(errDist)
		
	def updateTags():
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

	pd = PointDriver()

	loc_sub = rospy.Subscriber('/tag_detections', AprilTagDetectionArray, pd.update_tags)

	rospy.spin()
