#!/usr/bin/python

# Given a robot ID, regualrly output laser data for that robot

import rospy
from laser_oracle.srv import *
from sensor_msgs.msg import LaserScan
import math

def LaserDriver():
	rospy.init_node('laser_driver', anonymous=True)
	id = rospy.get_param("~robot_id", 0)	

	#Parameters for laser
	angleMin = rospy.get_param("~angleMin", -135 *(math.pi/180))
	angleMax = rospy.get_param("~angleMax", 135 *(math.pi/180))
	angleIncrement = rospy.get_param("~angleIncrement", 20 * (math.pi/180))
	rangeMin = rospy.get_param("~rangeMin", 0.0)
	rangeMax = rospy.get_param("~rangeMax", 0.3)

	pub = rospy.Publisher('laser_driver_{0}'.format(id), LaserScan, queue_size=0)
	
	#TODO make this a parameter
	rate = rospy.Rate(3)

	while not rospy.is_shutdown():
		rospy.wait_for_service("laser_oracle")
		#Try the service call
		try:
			laserMsg = rospy.ServiceProxy("laser_oracle", LaserOracle)
			response = laserMsg(robotID = id, angleMin = angleMin, angleMax = angleMax, angleIncrement = angleIncrement, rangeMin = rangeMin, rangeMax = rangeMax)
			#TODO calculate and update the scanTime and time_increment
			pub.publish(response.laserScan)
		except rospy.ServiceException, e:
			print "Service call failed: {0}".format(e)

		rate.sleep()

if __name__ == '__main__':
    try:
        LaserDriver()
    except rospy.ROSInterruptException:
        pass
