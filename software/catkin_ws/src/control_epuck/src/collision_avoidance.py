#!/usr/bin/python

#Simple collision avoidance with the laser scan emulation on the epucks

#Callback for the motor speeds

def laserCallback(laserData):
	rospy.loginfo("got called")

import rospy
from sensor_msgs.msg import LaserScan
if __name__=="__main__":
	rospy.init_node('controlNode', anonymous=True)
	botname = "epuck_robot_9"

	#subscribe to the laser messages
	rospy.Subscriber("/{0}/scan".format(botname), LaserScan, laserCallback)

	rospy.spin()