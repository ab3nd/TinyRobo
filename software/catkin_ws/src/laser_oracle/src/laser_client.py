#!/usr/bin/python

# Given a robot ID, regualrly output laser data for that robot

import rospy
from laser_oracle.srv import *
from sensor_msgs.msg import LaserScan
import math

def LaserDriver():
	rospy.init_node('laser_driver', anonymous=True)
	#The identifier of this robot
	id = rospy.get_param("~robot_id", 0)	
	#Rotation of the april tag relative to the robot front, in radians
	tag_rotation = rospy.get_param("~tag_rotation", 0)

	#Parameters for laser
	angleMin = rospy.get_param("~angleMin", -65 *(math.pi/180) + tag_rotation)
	angleMax = rospy.get_param("~angleMax", 65 *(math.pi/180) + tag_rotation)
	angleIncrement = rospy.get_param("~angleIncrement", 20 * (math.pi/180))
	rangeMin = rospy.get_param("~rangeMin", 0.0)
	rangeMax = rospy.get_param("~rangeMax", 0.3)
	scanRate = rospy.get_param("~scanRate", 3)

	pub = rospy.Publisher('laser_driver_{0}'.format(id), LaserScan, queue_size=0)
	
	rate = rospy.Rate(scanRate)

	lastScan = None

	rospy.wait_for_service("laser_oracle")
		
	while not rospy.is_shutdown():
		#Try the service call
		try:
			laserMsg = rospy.ServiceProxy("laser_oracle", LaserOracle)
			response = laserMsg(robotID = id, angleMin = angleMin, angleMax = angleMax, angleIncrement = angleIncrement, rangeMin = rangeMin, rangeMax = rangeMax)
			#Calculate and update the scanTime and time_increment
			if lastScan is not None:
				nowTime = rospy.Time.now()
				deltaT = nowTime - lastScan
				#Write the time between scans to the response laserScan object
				response.laserScan.scan_time = deltaT.to_sec()

				#This does rather assume that we start each scan immedately
				#After the previous one, and the time is divided equally
				scanCount = len(response.laserScan.ranges)
				if scanCount != 0: #Avoid div/0
					response.laserScan.time_increment = deltaT.to_sec()/scanCount
				
				#Laser scans are in the frame of the tag, which is in the frame of the robot
				#TODO this probably isn't perfect, esp. if tag is rotated relative to robot
				response.laserScan.header.frame_id="tag_{0}".format(id)
				response.laserScan.header.stamp = nowTime
				
	
			pub.publish(response.laserScan)
			#Update time last message was published
			lastScan = rospy.Time.now()
		except rospy.ServiceException, e:
			print "Service call failed: {0}".format(e)

		rate.sleep()

if __name__ == '__main__':
    try:
        LaserDriver()
    except rospy.ROSInterruptException:
        pass
