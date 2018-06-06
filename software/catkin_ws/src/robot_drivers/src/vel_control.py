#!/usr/bin/python

import rospy
from geometry_msgs.msg import Twist, PointStamped
from apriltags_ros.msg import *
import math
from image_geometry import PinholeCameraModel
from sensor_msgs.msg import CameraInfo
from trianglesolver import solve
from tf import transformations as transf

#Euclidean distance
def dist(a, b):
	assert len(a) == len(b)
	return math.sqrt(sum([math.pow(i-j, 2) for i, j in zip(a,b)]))


#Dot product
def dot(a, b):
	assert len(a) == len(b)
	return sum([i * j for i, j in zip(a, b)])

class PointDriver(object):

	def __init__(self):
		self.targetX = self.currentX = 0
		self.targetY = self.currentY = 0
		self.targetZ = self.currentZ = 0

		self.robot_id = 0

		self.pub = rospy.Publisher('cmd_vel', Twist, queue_size=0)

	#Doesn't do much, tag update is the real workhorse
	def updatePoint(self, point_msg):
		self.targetX = point_msg.point.x
		self.targetY = point_msg.point.y
		self.targetZ = point_msg.point.z

		
	def updateTags(self, tags_msg):
		#Get the location of this robot from the tag
		tag = None
		try:
			tag = [x for x in tags_msg.detections if x.id == self.robot_id][0]
		except IndexError as ie:
			#This is caused by there not being a detection of the tag in this set of 
			#tag detections
			rospy.logwarn("Didn't see tag {0} in this frame".format(self.robot_id))
			return 

		#Get the current location
		self.currentX = tag.pose.pose.position.x 
		self.currentY = tag.pose.pose.position.y 
		self.currentZ = tag.pose.pose.position.z 

		w = tag.pose.pose.orientation.w
		x = tag.pose.pose.orientation.x
		y = tag.pose.pose.orientation.y
		z = tag.pose.pose.orientation.z

		#Update the robot's heading.
		#The roll direction is what I'd call yaw.
		#RPY are ambiguious, nothing to be done for it.
		(roll, pitch, yaw) = transf.euler_from_quaternion([w, x, y, z]) 
		current_heading = roll
		#print "Robot heading {}".format(current_heading)
		
		# Get the ray to the target point
		x = self.targetX - self.currentX
		y = self.targetY - self.currentY
		z = self.targetZ - self.currentZ

		#I'm not concerned about z at the moment
		target_angle = math.atan2(y,x)
		#print "Click angle {}".format(target_angle)

		#This will be between +/- Pi
		smallest_angle = math.atan2(math.sin(current_heading-target_angle), math.cos(current_heading-target_angle))
 	 	print "Turn angle: {}".format(smallest_angle)
 		
		# #Calculate the error between the location of the robot and the location of the point
		# errDist = dist((self.currentX, self.currentY), (self.targetX, self.targetY))
		# print "Distance {}".format(errDist)

		# #Generate a twist message and send it to the robot
		linear = 0
		rotational = smallest_angle/math.pi

		rTwist = Twist()
		#only two params are used for robots on a table
		rTwist.linear.x = linear
		rTwist.angular.z = rotational
		#The rest are not used
		rTwist.linear.y = rTwist.linear.z = 0
		rTwist.angular.x = rTwist.angular.y = 0

		#Ship it!
		self.pub.publish(rTwist)

if __name__ == '__main__':
	rospy.init_node('vel_driver', anonymous=True)

	pd = PointDriver()

	loc_sub = rospy.Subscriber('/tag_detections', AprilTagDetectionArray, pd.updateTags)

	point_sub = rospy.Subscriber('/converted_point', PointStamped, pd.updatePoint)
	rospy.spin()
