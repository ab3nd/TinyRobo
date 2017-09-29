#!/usr/bin/python

#Given a fixed X, Y point, drive towards the point

import rospy
from geometry_msgs.msg import Twist
from apriltags_ros.msg import *
import random
import math
from tf import transformations as trans

class Point_Driver():
	def __init__(self):
		self.pub = rospy.Publisher('point_twists', Twist, queue_size=0)
		#TODO these should be configurable
		self.robot_id = 7
		self.targetX = 0.7
		self.targetY = 0.7

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

		#Calculate the error between the rotation of the robot and orientation towards the point
		x = self.targetX - tag.pose.pose.position.x 
		y = self.targetY - tag.pose.pose.position.y
		z = tag.pose.pose.position.z #I don't care about the height of the tag

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

		#Angle
		print math.acos(dot/(mag1*mag2))


		#Caclulate the error between the location of the robot and the location of the point
		errDist = math.sqrt(pow(tag.pose.pose.position.x - self.targetX,2) + pow(tag.pose.pose.position.y - self.targetY,2))
		print "distance", errDist

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

	rospy.spin()