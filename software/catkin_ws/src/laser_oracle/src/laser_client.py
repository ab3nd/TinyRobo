#!/usr/bin/python

# Given a robot ID, regualrly output laser data for that robot

import rospy
from laser_oracle.srv import *
from sensor_msgs.msg import LaserScan

def LaserDriver():
	rospy.init_node('laser_driver', anonymous=True)
	id = rospy.get_param("~robot_id", 0)	

	#TODO add parameters for laser
	#And reasonable defaults

	pub = rospy.Publisher('laser_driver_{0}'.format(id), LaserScan, queue_size=0)
	
	#TODO make this a parameter
	rate = rospy.Rate(3)

	while not rospy.is_shutdown():
		rospy.wait_for_service("laser_oracle")
		#Try the service call
		try:
			laserMsg = rospy.ServiceProxy("laser_oracle", LaserOracle)
			response = laserMsg(id)
			pub.publish(response.laserScan)
		except rospy.ServiceException, e:
			print "Service call failed: {0}".format(e)

		rate.sleep()

if __name__ == '__main__':
    try:
        LaserDriver()
    except rospy.ROSInterruptException:
        pass
