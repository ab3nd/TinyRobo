#!/usr/bin/python

# Drive a TinyRobo in a straight line, unless it sees something, 
# then turn to avoid the thing. 

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

class AvoidDriver():

	def __init__(self, robotID):
		#Subscribe to the laser scans for this robot
		self.laserSub = rospy.Subscriber('/laser_driver_{0}'.format(robotID), LaserScan, self.laserHandler)

		#Publish twist messages that control the robot
		self.twistPub = rospy.Publisher('/avoid_twists_{0}'.format(robotID), Twist, queue_size=0)

	def laserHandler(self, msg):
		rTwist = Twist()

		#Default to no motion
		linear = rotational = 0

		#Check the left and right sides of the scan
		sideSize = msg.ranges/2
		lAvg = sum(msg.ranges[:sideSize])/sideSize
		rAvg = sum(msg.ranges[sideSize:])/sideSize

		#Debugging
		rospy.loginfo("L: {0} R: {0}".format(lAvg, rAvg))
		
		#Decide to turn

		#No turn, go straight

		#only two params are used for robots on a table
		rTwist.linear.x = linear
		rTwist.angular.z = rotational
		#The rest are not used
		rTwist.linear.y = rTwist.linear.z = 0
		rTwist.angular.x = rTwist.angular.y = 0

		self.twistPub.publish(rTwist)


if __name__ = '__main__':

	rospy.init_node("obstacle_avoid_driver", anonymous=True)

	robotID = rospy.get_param("~robot_id", 0)

	driver = AvoidDriver(robotID)
	
	pass