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
		sideSize = len(msg.ranges)/2
		bump = len(msg.ranges) % 2 #Account for odd sizes
		lAvg = sum(msg.ranges[:sideSize])/sideSize
		#This does ignore the middle range scan, if any
		rAvg = sum(msg.ranges[sideSize + bump:])/sideSize

		#Debugging
		#rospy.loginfo("L: {0} R: {1}".format(lAvg, rAvg))

		#Decide to turn, based on difference between sides,
		#scaled to the range 0..1 by the laser max range
		scaleFactor = msg.range_max
		rotational = (lAvg - rAvg)/scaleFactor

		#Linear speed is inversely proportional to rotational speed
		#so as it turns harder, it goes forward less
		linear = 1.0 - abs(rotational)

		#Debugging
		#rospy.loginfo("Linear: {0} Rotation: {1}".format(linear, rotational))

		#only two params are used for robots on a table
		#Multiplied by < 1 to slow the robot down
		rTwist.linear.x = linear * 0.4
		rTwist.angular.z = rotational * 0.4
		#The rest are not used
		rTwist.linear.y = rTwist.linear.z = 0
		rTwist.angular.x = rTwist.angular.y = 0

		self.twistPub.publish(rTwist)


if __name__ == '__main__':

	rospy.init_node("obstacle_avoid_driver", anonymous=True)

	robotID = rospy.get_param("~robot_id", 0)

	driver = AvoidDriver(robotID)
	
	rospy.spin()