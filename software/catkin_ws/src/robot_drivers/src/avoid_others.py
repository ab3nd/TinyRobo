#!/usr/bin/python

# Gets twist messages from the joystick, but if the command would likely lead to collision,
# modify it so the robot turns

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

class AvoidOthersDriver():

	def __init__(self, robotID):
		#Subscribe to the twist messages from the joystick for this robot
		self.twistSub = rospy.Subscriber('/js_twists_{0}'.format(robotID), Twist, self.js_handler)

		#Publish twist messages that control the robot
		self.twistPub = rospy.Publisher('/avoid_twists_{0}'.format(robotID), Twist, queue_size=0)

		#Sense range and bearing to other robots
		self.rabSub = rospy.Subscriber('/rab_sense_{0}'.format(robotID), RangeAndBearing, self.update_rab)
		self.ranges = {}


	def js_handler(self, msg):
	
		if other_on_vector() and other_close():
			#This is the only param we change, linear.x is kept the same
			msg.angular.z += 0.01

		#The rest are not used
		msg.linear.y = msg.linear.z = 0
		msg.angular.x = msg.angular.y = 0

		self.twistPub.publish(msg)


if __name__ == '__main__':

	rospy.init_node("obstacle_avoid_driver", anonymous=True)

	robotID = rospy.get_param("~robot_id", 0)

	driver = AvoidDriver(robotID)
	
	rospy.spin()